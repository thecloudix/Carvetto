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
#include "carvetto/text/layout.hpp"
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

// Largest code point boundary <= i.
std::size_t floorBoundary(const std::string& t, std::size_t i) {
    i = std::min(i, t.size());
    while (i > 0 && i < t.size() && (static_cast<unsigned char>(t[i]) & 0xC0U) == 0x80U) {
        --i;
    }
    return i;
}

std::size_t prevBoundary(const std::string& t, std::size_t i) {
    if (i == 0) {
        return 0;
    }
    return floorBoundary(t, i - 1);
}

std::size_t nextBoundary(const std::string& t, std::size_t i) {
    if (i >= t.size()) {
        return t.size();
    }
    ++i;
    while (i < t.size() && (static_cast<unsigned char>(t[i]) & 0xC0U) == 0x80U) {
        ++i;
    }
    return i;
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
    caretSeg_.reset();
    selQuads_.clear();
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
    caretSeg_.reset();
    selQuads_.clear();
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
    refreshCaretGeometry();
    TextEditResult r = changed();
    if (b.missingGlyphs) {
        r.message = "the font has no glyph for some characters";
    }
    return r;
}

void TextEditor::refreshCaretGeometry() {
    caretSeg_.reset();
    selQuads_.clear();
    const text::Font* f = font();
    if (!draft_ || f == nullptr || draft_->params.mode != TextMode::Point) {
        return;
    }
    const TextParams& p = draft_->params;
    text::TextStyle style;
    style.sizeMm = p.sizeMm;
    style.alignment = p.align == document::TextAlign::Centre  ? text::Alignment::Centre
                      : p.align == document::TextAlign::Right ? text::Alignment::Right
                                                               : text::Alignment::Left;
    style.letterSpacingMm = p.letterSpacingMm;
    style.lineSpacing = p.lineSpacing;
    style.kerning = p.kerning;
    const text::TextLayout l = text::layoutText(*f, p.text, style);
    if (!l.ok()) {
        return;
    }
    const double upm = std::max(1, f->metrics().unitsPerEm);
    const double up = f->metrics().ascender / upm * p.sizeMm;
    const double down = -f->metrics().descender / upm * p.sizeMm;
    const auto lineOf = [&](std::size_t offset) {
        std::size_t line = 0;
        for (std::size_t i = 0; i < offset && i < p.text.size(); ++i) {
            line += p.text[i] == '\n' ? 1U : 0U;
        }
        return line;
    };
    // x of the caret position `offset` on its line (left edge of the first glyph at or after it).
    const auto xAt = [&](std::size_t offset, std::size_t line) -> std::optional<Vec2> {
        if (l.lines.empty()) {
            return Vec2{0.0, 0.0};
        }
        const text::TextLine& tl = l.lines[std::min(line, l.lines.size() - 1)];
        for (std::size_t i = 0; i < tl.glyphCount; ++i) {
            const text::PlacedGlyph& g = l.glyphs[tl.firstGlyph + i];
            if (g.cluster >= offset) {
                return Vec2{g.origin.x, tl.baselineStart.y};
            }
        }
        return Vec2{tl.baselineStart.x + tl.widthMm, tl.baselineStart.y};
    };
    const std::size_t caretLine = lineOf(draft_->caret);
    if (const auto c = xAt(draft_->caret, caretLine)) {
        caretSeg_ = std::make_pair(p.placement.apply({c->x, c->y - down}), p.placement.apply({c->x, c->y + up}));
    }
    if (draft_->caret != draft_->anchor) {
        const std::size_t a = std::min(draft_->caret, draft_->anchor);
        const std::size_t b = std::max(draft_->caret, draft_->anchor);
        const std::size_t la = lineOf(a);
        const std::size_t lb = lineOf(b);
        for (std::size_t line = la; line <= lb; ++line) {
            // Line start and end offsets in the text.
            std::size_t lineBegin = 0;
            for (std::size_t n = 0; n < line; ++n) {
                lineBegin = p.text.find('\n', lineBegin) + 1;
            }
            std::size_t lineEnd = p.text.find('\n', lineBegin);
            if (lineEnd == std::string::npos) {
                lineEnd = p.text.size();
            }
            const auto from = xAt(line == la ? a : lineBegin, line);
            const auto to = line == lb ? xAt(b, line) : xAt(lineEnd, line);
            if (!from || !to || to->x < from->x) {
                continue;
            }
            const double y0 = from->y - down;
            const double y1 = from->y + up;
            selQuads_.push_back({p.placement.apply({from->x, y0}), p.placement.apply({to->x, y0}),
                                 p.placement.apply({to->x, y1}), p.placement.apply({from->x, y1})});
        }
    }
}

std::optional<geometry::Box2> TextEditor::frameMm() const {
    if (!draft_) {
        return std::nullopt;
    }
    if (draft_->params.mode == TextMode::Box) {
        return boxMm();
    }
    geometry::Box2 b;
    for (const geometry::Contour& c : preview_) {
        b.expand(c.bounds());
    }
    if (b.isEmpty()) {
        return std::nullopt;
    }
    const double g = draft_->params.sizeMm * 0.25;
    return geometry::Box2({b.min.x - g, b.min.y - g}, {b.max.x + g, b.max.y + g});
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
    draft_ = Draft{std::move(params), editing, 0, 0};
    draft_->caret = draft_->anchor = draft_->params.text.size();
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

std::optional<std::pair<std::size_t, std::size_t>> TextEditor::selectionRange() const {
    if (!draft_ || draft_->caret == draft_->anchor) {
        return std::nullopt;
    }
    return std::make_pair(std::min(draft_->caret, draft_->anchor), std::max(draft_->caret, draft_->anchor));
}

// Replaces [from, to) of the draft text with `with` and puts the caret after it.
TextEditResult TextEditor::replaceRange(std::size_t from, std::size_t to, std::string_view with) {
    std::string& t = draft_->params.text;
    if (t.size() - (to - from) + with.size() > document::kMaxTextObjectBytes) {
        return refused("the text is too long");
    }
    t.replace(from, to - from, with);
    draft_->caret = draft_->anchor = from + with.size();
    return relayout();
}

TextEditResult TextEditor::insertText(std::string_view utf8) {
    if (!draft_) {
        return refused("click on the canvas to place text first");
    }
    const std::string add = sanitize(utf8);
    const auto sel = selectionRange();
    const std::size_t from = sel ? sel->first : draft_->caret;
    const std::size_t to = sel ? sel->second : draft_->caret;
    return replaceRange(from, to, add);
}

TextEditResult TextEditor::backspace() {
    if (!draft_) {
        return refused("there is no text being edited");
    }
    if (const auto sel = selectionRange()) {
        return replaceRange(sel->first, sel->second, {});
    }
    const std::size_t from = prevBoundary(draft_->params.text, draft_->caret);
    return replaceRange(from, draft_->caret, {});
}

TextEditResult TextEditor::deleteForward() {
    if (!draft_) {
        return refused("there is no text being edited");
    }
    if (const auto sel = selectionRange()) {
        return replaceRange(sel->first, sel->second, {});
    }
    const std::size_t to = nextBoundary(draft_->params.text, draft_->caret);
    return replaceRange(draft_->caret, to, {});
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
    draft_->caret = draft_->anchor = t.size();
    return relayout();
}

TextEditResult TextEditor::setCaret(std::size_t byteOffset, bool extend) {
    if (!draft_) {
        return refused("there is no text being edited");
    }
    draft_->caret = floorBoundary(draft_->params.text, byteOffset);
    if (!extend) {
        draft_->anchor = draft_->caret;
    }
    refreshCaretGeometry();
    return changed();
}

TextEditResult TextEditor::moveCaret(CaretMove move, bool extend) {
    if (!draft_) {
        return refused("there is no text being edited");
    }
    const std::string& t = draft_->params.text;
    const std::size_t c = draft_->caret;
    std::size_t to = c;
    switch (move) {
        case CaretMove::Left:
            // Without shift a selection collapses to its near edge instead of moving on.
            to = (!extend && draft_->anchor != c) ? std::min(c, draft_->anchor) : prevBoundary(t, c);
            break;
        case CaretMove::Right:
            to = (!extend && draft_->anchor != c) ? std::max(c, draft_->anchor) : nextBoundary(t, c);
            break;
        case CaretMove::LineStart: {
            const std::size_t nl = c == 0 ? std::string::npos : t.rfind('\n', c - 1);
            to = nl == std::string::npos ? 0 : nl + 1;
            break;
        }
        case CaretMove::LineEnd: {
            const std::size_t nl = t.find('\n', c);
            to = nl == std::string::npos ? t.size() : nl;
            break;
        }
        case CaretMove::TextStart:
            to = 0;
            break;
        case CaretMove::TextEnd:
            to = t.size();
            break;
    }
    return setCaret(to, extend);
}

TextEditResult TextEditor::selectAll() {
    if (!draft_) {
        return refused("there is no text being edited");
    }
    draft_->anchor = 0;
    draft_->caret = draft_->params.text.size();
    refreshCaretGeometry();
    return changed();
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
        const std::string why = doc_->fontProblem(document::FontAsset{fontKey_, fontBytes_});
        if (!why.empty()) {
            return refused("cannot embed the font: " + why);
        }
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
