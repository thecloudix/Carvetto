#include "carvetto/viewmodel/text_tool.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/text_commands.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"

namespace carvetto::viewmodel {

namespace {

using document::TextMode;
using document::TextParams;
using geometry::Vec2;

TextEditResult refused(std::string message) {
    TextEditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

TextEditResult changed() {
    TextEditResult r;
    r.draftChanged = true;
    return r;
}

// Normalises typed text: CRLF and CR become LF; control characters other than LF and TAB are dropped.
std::string sanitize(std::string_view in) {
    std::string out;
    out.reserve(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(in[i]);
        if (c == '\r') {
            if (i + 1 < in.size() && in[i + 1] == '\n') {
                ++i;
            }
            out.push_back('\n');
        } else if (c == '\n' || c == '\t' || (c >= 0x20 && c != 0x7F)) {
            out.push_back(static_cast<char>(c));
        }
    }
    return out;
}

bool finiteInRange(double v, double lo, double hi) {
    return std::isfinite(v) && v >= lo && v <= hi;
}

}  // namespace

TextEditor::TextEditor(document::Document* doc) : doc_(doc) {}
TextEditor::~TextEditor() = default;

void TextEditor::setDocument(document::Document* doc) {
    // The old document may already be gone: nothing of it is touched here. The draft owns copies of
    // everything it needs, so it is simply discarded.
    clearDraft();
    pressed_ = false;
    dragging_ = false;
    doc_ = doc;
}

void TextEditor::clearDraft() {
    draft_.reset();
    preview_.clear();
    previewMessage_.clear();
    overflow_ = false;
    dragging_ = false;
}

void TextEditor::cancel() {
    clearDraft();
    pressed_ = false;
}

bool TextEditor::hasFont() const {
    return font_ != nullptr && font_->valid();
}

const text::Font* TextEditor::font() const {
    return hasFont() ? font_.get() : nullptr;
}

bool TextEditor::loadFont(const std::string& key, const std::vector<std::uint8_t>& bytes, std::string* message) {
    text::FontLoad load = text::Font::load(bytes);
    if (!load.ok()) {
        *message = load.message.empty() ? "the font file cannot be read (TrueType or OpenType expected)"
                                        : load.message;
        return false;
    }
    font_ = std::make_unique<text::Font>(std::move(load.font));
    fontKey_ = key;
    fontBytes_ = bytes;
    fontName_ = font_->familyName();
    return true;
}

TextEditResult TextEditor::setFont(std::vector<std::uint8_t> bytes, std::string displayName) {
    if (bytes.empty()) {
        return refused("the font file is empty");
    }
    if (bytes.size() > document::kMaxEmbeddedFontBytes) {
        return refused("the font file is larger than " + std::to_string(document::kMaxEmbeddedFontBytes >> 20) +
                       " MiB, the most a document embeds");
    }
    const std::string key = document::fontKeyFor(bytes);
    std::string message;
    if (!loadFont(key, bytes, &message)) {
        return refused(message);
    }
    if (!displayName.empty()) {
        fontName_ = std::move(displayName);
    }
    if (draft_) {
        draft_->params.fontKey = key;
        return relayout();
    }
    return {};
}

TextEditResult TextEditor::setFontFile(const std::filesystem::path& path) {
    std::error_code ec;
    const std::uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec) {
        return refused("the font file cannot be read: " + ec.message());
    }
    if (size > document::kMaxEmbeddedFontBytes) {
        return refused("the font file is larger than " + std::to_string(document::kMaxEmbeddedFontBytes >> 20) +
                       " MiB, the most a document embeds");
    }
    std::ifstream in(path, std::ios::binary);
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (!in.eof() && in.fail() && bytes.empty()) {
        return refused("the font file cannot be read");
    }
    return setFont(std::move(bytes), path.filename().string());
}

// --- Settings -------------------------------------------------------------------------------

TextEditResult TextEditor::setSize(double sizeMm) {
    if (!finiteInRange(sizeMm, 1e-3, 10000.0)) {
        return refused("the text size must be between 0.001 and 10000 mm");
    }
    settings_.sizeMm = sizeMm;
    if (draft_) {
        draft_->params.sizeMm = sizeMm;
        return relayout();
    }
    return {};
}

TextEditResult TextEditor::setAlign(document::TextAlign align) {
    settings_.align = align;
    if (draft_) {
        draft_->params.align = align;
        return relayout();
    }
    return {};
}

TextEditResult TextEditor::setLetterSpacing(double mm) {
    if (!finiteInRange(mm, -1000.0, 1000.0)) {
        return refused("the letter spacing must be between -1000 and 1000 mm");
    }
    settings_.letterSpacingMm = mm;
    if (draft_) {
        draft_->params.letterSpacingMm = mm;
        return relayout();
    }
    return {};
}

TextEditResult TextEditor::setLineSpacing(double factor) {
    if (!finiteInRange(factor, 0.0, 100.0)) {
        return refused("the line spacing must be between 0 and 100");
    }
    settings_.lineSpacing = factor;
    if (draft_) {
        draft_->params.lineSpacing = factor;
        return relayout();
    }
    return {};
}

TextEditResult TextEditor::setKerning(bool on) {
    settings_.kerning = on;
    if (draft_) {
        draft_->params.kerning = on;
        return relayout();
    }
    return {};
}

TextEditResult TextEditor::setMode(TextMode mode) {
    TextEditResult r;
    if (draft_) {
        r = commit();
    }
    settings_.mode = mode;
    return r;
}

// --- Layout ---------------------------------------------------------------------------------

TextEditResult TextEditor::relayout() {
    preview_.clear();
    previewMessage_.clear();
    overflow_ = false;
    if (!draft_) {
        return changed();
    }
    const text::Font* f = font();
    if (f == nullptr) {
        previewMessage_ = "choose a font first";
        return changed();
    }
    draft_->params.fontKey = fontKey_;
    document::TextBuild b = document::buildTextContours(*f, draft_->params);
    if (!b.ok()) {
        previewMessage_ = b.message.empty() ? "the text cannot be laid out" : b.message;
        TextEditResult r = changed();
        r.ok = false;
        r.message = previewMessage_;
        return r;
    }
    preview_ = std::move(b.contours);
    overflow_ = b.overflow;
    TextEditResult r = changed();
    if (b.missingGlyphs) {
        r.message = "the font has no glyph for some characters";
    }
    return r;
}

std::optional<geometry::Box2> TextEditor::boxMm() const {
    if (!draft_ || draft_->params.mode != TextMode::Box) {
        return std::nullopt;
    }
    const TextParams& p = draft_->params;
    return geometry::Box2(p.placement.apply({0.0, 0.0}), p.placement.apply({p.boxWidthMm, -p.boxHeightMm}));
}

std::optional<geometry::Box2> TextEditor::dragBoxMm() const {
    if (!pressed_ || !dragging_ || settings_.mode != TextMode::Box) {
        return std::nullopt;
    }
    return geometry::Box2(pressMm_, curMm_);
}

TextEditResult TextEditor::begin(TextParams params, document::ObjectId editing) {
    draft_ = Draft{std::move(params), editing};
    return relayout();
}

TextEditResult TextEditor::startFor(Vec2 anchorMm) {
    if (!hasFont()) {
        return refused("choose a font first");
    }
    TextParams p;
    p.fontKey = fontKey_;
    p.sizeMm = settings_.sizeMm;
    p.align = settings_.align;
    p.letterSpacingMm = settings_.letterSpacingMm;
    p.lineSpacing = settings_.lineSpacing;
    p.kerning = settings_.kerning;
    p.mode = settings_.mode;
    p.placement = geometry::Transform2::translation(anchorMm);
    return begin(std::move(p), document::ObjectId{});
}

// --- Pointer --------------------------------------------------------------------------------

TextEditResult TextEditor::press(Vec2 px, Vec2 snappedMm) {
    pressed_ = false;
    dragging_ = false;
    if (doc_ == nullptr) {
        return refused("no document");
    }
    // A click on the open draft's own glyphs keeps typing (no caret movement: text goes at the end).
    if (draft_ && !preview_.empty()) {
        geometry::Box2 b;
        for (const geometry::Contour& c : preview_) {
            b.expand(c.bounds());
        }
        const double tol = view_.scale() > 0.0 ? 4.0 / view_.scale() : 0.0;
        if (!b.isEmpty() && snappedMm.x >= b.min.x - tol && snappedMm.x <= b.max.x + tol &&
            snappedMm.y >= b.min.y - tol && snappedMm.y <= b.max.y + tol) {
            return {};
        }
    }
    TextEditResult done;
    if (draft_) {
        done = commit();
        if (!done.ok) {
            return done;
        }
    }
    // Existing text under the pointer: edit it in place.
    HitOptions fillHit;
    fillHit.fill = FillHit::StrokeAndFill;
    if (const auto hit = hitTest(*doc_, view_, px, fillHit)) {
        const document::Object* obj = doc_->findObject(hit->leaf);
        const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
        if (vec != nullptr && std::holds_alternative<TextParams>(vec->params)) {
            TextEditResult r = beginEdit(hit->leaf);
            r.committed = done.committed;
            return r;
        }
    }
    if (settings_.mode == TextMode::Curve) {
        const auto hit = hitTest(*doc_, view_, px, HitOptions{});
        const document::Object* obj = hit ? doc_->findObject(hit->leaf) : nullptr;
        const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
        if (vec == nullptr || vec->contours.empty()) {
            return refused("click on a curve to run the text along it");
        }
        if (!hasFont()) {
            return refused("choose a font first");
        }
        const geometry::Contour* best = nullptr;
        double bestDistance = 0.0;
        for (const geometry::Contour& c : vec->contours) {
            const double d = geometry::distanceTo(c, snappedMm);
            if (best == nullptr || d < bestDistance) {
                best = &c;
                bestDistance = d;
            }
        }
        TextParams p;
        p.fontKey = fontKey_;
        p.sizeMm = settings_.sizeMm;
        p.align = settings_.align;
        p.letterSpacingMm = settings_.letterSpacingMm;
        p.lineSpacing = settings_.lineSpacing;
        p.kerning = settings_.kerning;
        p.mode = TextMode::Curve;
        p.guide = *best;
        p.guideId = hit->leaf;
        TextEditResult r = begin(std::move(p), document::ObjectId{});
        r.committed = done.committed;
        return r;
    }
    if (settings_.mode == TextMode::Box) {
        pressed_ = true;
        pressPx_ = px;
        pressMm_ = snappedMm;
        curMm_ = snappedMm;
        return {};
    }
    TextEditResult r = startFor(snappedMm);
    r.committed = done.committed;
    return r;
}

TextEditResult TextEditor::move(Vec2 px, Vec2 snappedMm) {
    if (pressed_) {
        curMm_ = snappedMm;
        dragging_ = dragging_ || geometry::distance(px, pressPx_) >= kDragThresholdPx;
        return changed();
    }
    return {};
}

TextEditResult TextEditor::release(Vec2 px, Vec2 snappedMm) {
    if (!pressed_) {
        return {};
    }
    pressed_ = false;
    const bool dragged = dragging_ || geometry::distance(px, pressPx_) >= kDragThresholdPx;
    dragging_ = false;
    if (!dragged) {
        return refused("drag a box for the text");
    }
    if (!hasFont()) {
        return refused("choose a font first");
    }
    const Vec2 a = pressMm_;
    const Vec2 b = snappedMm;
    const double w = std::fabs(b.x - a.x);
    const double h = std::fabs(b.y - a.y);
    if (!(w > 0.0)) {
        return refused("the box has no width");
    }
    TextParams p;
    p.fontKey = fontKey_;
    p.sizeMm = settings_.sizeMm;
    p.align = settings_.align;
    p.letterSpacingMm = settings_.letterSpacingMm;
    p.lineSpacing = settings_.lineSpacing;
    p.kerning = settings_.kerning;
    p.mode = TextMode::Box;
    p.boxWidthMm = w;
    p.boxHeightMm = h;
    p.placement = geometry::Transform2::translation({std::min(a.x, b.x), std::max(a.y, b.y)});
    return begin(std::move(p), document::ObjectId{});
}

// --- Typing ---------------------------------------------------------------------------------

TextEditResult TextEditor::insertText(std::string_view utf8) {
    if (!draft_) {
        return refused("click on the canvas to place text first");
    }
    const std::string add = sanitize(utf8);
    if (draft_->params.text.size() + add.size() > document::kMaxTextObjectBytes) {
        return refused("the text is too long");
    }
    draft_->params.text += add;
    return relayout();
}

TextEditResult TextEditor::backspace() {
    if (!draft_) {
        return refused("there is no text being edited");
    }
    std::string& t = draft_->params.text;
    if (!t.empty()) {
        std::size_t i = t.size() - 1;
        while (i > 0 && (static_cast<unsigned char>(t[i]) & 0xC0U) == 0x80U) {
            --i;
        }
        t.erase(i);
    }
    return relayout();
}

TextEditResult TextEditor::setText(std::string_view utf8) {
    if (!draft_) {
        return refused("there is no text being edited");
    }
    const std::string t = sanitize(utf8);
    if (t.size() > document::kMaxTextObjectBytes) {
        return refused("the text is too long");
    }
    draft_->params.text = t;
    return relayout();
}

// --- Editing existing text --------------------------------------------------------------------

TextEditResult TextEditor::beginEdit(document::ObjectId id) {
    if (doc_ == nullptr) {
        return refused("no document");
    }
    const document::Object* obj = doc_->findObject(id);
    const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
    const auto* params = vec != nullptr ? std::get_if<TextParams>(&vec->params) : nullptr;
    if (params == nullptr) {
        return refused("that object is not text");
    }
    const document::FontAsset* asset = doc_->findFont(params->fontKey);
    if (asset == nullptr) {
        return refused("the font of this text is not embedded in the document");
    }
    std::string message;
    if (!loadFont(asset->key, asset->bytes, &message)) {
        return refused("the embedded font cannot be read: " + message);
    }
    settings_.sizeMm = params->sizeMm;
    settings_.align = params->align;
    settings_.letterSpacingMm = params->letterSpacingMm;
    settings_.lineSpacing = params->lineSpacing;
    settings_.kerning = params->kerning;
    settings_.mode = params->mode;
    pressed_ = false;
    return begin(*params, id);
}

TextEditResult TextEditor::beginEditSelected() {
    if (doc_ == nullptr) {
        return refused("no document");
    }
    const document::ObjectId id = doc_->selection().single();
    if (id.isNull()) {
        return refused("select one text object");
    }
    return beginEdit(id);
}

TextEditResult TextEditor::reverseGuide() {
    if (!draft_ || draft_->params.mode != TextMode::Curve) {
        return refused("this text is not on a curve");
    }
    draft_->params.guide.reverse();
    return relayout();
}

TextEditResult TextEditor::setStartOffset(double mm) {
    if (!draft_ || draft_->params.mode != TextMode::Curve) {
        return refused("this text is not on a curve");
    }
    if (!std::isfinite(mm) || std::fabs(mm) > 1.0e9) {
        return refused("the start offset must be a finite length");
    }
    draft_->params.startOffsetMm = mm;
    return relayout();
}

// --- Commit ---------------------------------------------------------------------------------

bool TextEditor::layerWritable(std::string* why) const {
    const document::Layer* layer = doc_ != nullptr ? doc_->findLayer(doc_->activeLayer()) : nullptr;
    if (layer == nullptr) {
        *why = "there is no active layer";
        return false;
    }
    if (layer->locked) {
        *why = "the active layer is locked";
        return false;
    }
    if (!layer->visible) {
        *why = "the active layer is hidden";
        return false;
    }
    return true;
}

TextEditResult TextEditor::commit() {
    if (!draft_) {
        return {};
    }
    if (doc_ == nullptr) {
        clearDraft();
        return refused("no document");
    }
    if (draft_->params.text.empty()) {
        clearDraft();
        TextEditResult r;
        r.message = "empty text was not placed";
        return r;
    }
    relayout();
    if (!previewMessage_.empty() || preview_.empty()) {
        return refused(previewMessage_.empty() ? "the text has no outlines (the font has no glyphs for it)"
                                               : previewMessage_);
    }
    if (fontKey_ != draft_->params.fontKey || !hasFont()) {
        return refused("choose a font first");
    }
    const TextParams params = draft_->params;
    const document::ObjectId editing = draft_->editing;
    auto macro = std::make_unique<document::MacroCommand>(editing.isNull() ? "Add text" : "Edit text");
    if (doc_->findFont(fontKey_) == nullptr) {
        macro->add(std::make_unique<document::AddFontCommand>(document::FontAsset{fontKey_, fontBytes_}));
    }
    document::ObjectId resultId = editing;
    if (editing.isNull()) {
        std::string why;
        if (!layerWritable(&why)) {
            return refused(why);
        }
        if (!doc_->idGenerator().canAllocate(1)) {
            return refused("the document has no free object ids left");
        }
        document::Object obj =
            document::Object::vector(preview_, doc_->activeLayer(), doc_->activeSheet(), params);
        obj.name = "Text";
        auto add = std::make_unique<document::AddObjectCommand>(std::move(obj));
        const document::AddObjectCommand* addPtr = add.get();
        macro->add(std::move(add));
        if (!doc_->execute(std::move(macro))) {
            return refused("the text could not be added");
        }
        resultId = addPtr->id();
    } else {
        const document::Object* obj = doc_->findObject(editing);
        const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
        if (vec == nullptr || !std::holds_alternative<TextParams>(vec->params)) {
            clearDraft();
            return refused("the text object no longer exists");
        }
        const document::Layer* layer = doc_->findLayer(obj->layer);
        if (layer == nullptr || layer->locked) {
            return refused("the layer of this text is locked");
        }
        if (std::get<TextParams>(vec->params) == params && vec->contours == preview_ &&
            doc_->findFont(fontKey_) != nullptr) {
            clearDraft();
            TextEditResult r;
            r.message = "no change";
            return r;
        }
        macro->add(std::make_unique<document::SetTextCommand>(editing, params, preview_));
        if (!doc_->execute(std::move(macro))) {
            return refused("the text could not be changed");
        }
    }
    doc_->selection().assign({resultId});
    clearDraft();
    TextEditResult r;
    r.committed = true;
    r.selectionChanged = true;
    r.draftChanged = true;
    return r;
}

TextEditResult TextEditor::convertSelectionToCurves() {
    if (doc_ == nullptr) {
        return refused("no document");
    }
    std::vector<document::ObjectId> ids;
    for (const document::ObjectId id : doc_->selection().ids()) {
        const document::Object* obj = doc_->findObject(id);
        const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
        if (vec != nullptr && std::holds_alternative<TextParams>(vec->params)) {
            ids.push_back(id);
        }
    }
    if (ids.empty()) {
        return refused("select text to convert to curves");
    }
    if (draft_) {
        const document::ObjectId editing = draft_->editing;
        if (std::find(ids.begin(), ids.end(), editing) != ids.end()) {
            clearDraft();  // the object stops being text
        }
    }
    if (!doc_->execute(std::make_unique<document::ConvertTextToCurvesCommand>(std::move(ids)))) {
        return refused("the text could not be converted");
    }
    TextEditResult r;
    r.committed = true;
    return r;
}

}  // namespace carvetto::viewmodel
