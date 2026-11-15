#include "carvetto/viewmodel/relief_sculpt.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <memory>
#include <utility>

#include "carvetto/document/relief_commands.hpp"

namespace carvetto::viewmodel {

namespace {

using document::ComponentId;
using document::Document;

constexpr std::size_t kMaxPathPoints = 50000;
constexpr std::size_t kMaxPresetNameBytes = 64;

EditResult refused(std::string message) {
    EditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

bool sameField(const relief::HeightField& a, const relief::HeightField& b) {
    if (a.width() != b.width() || a.height() != b.height()) {
        return false;
    }
    const std::vector<float>& x = a.data();
    const std::vector<float>& y = b.data();
    for (std::size_t i = 0; i < x.size(); ++i) {
        const bool nx = std::isnan(x[i]);
        const bool ny = std::isnan(y[i]);
        if (nx != ny || (!nx && x[i] != y[i])) {
            return false;
        }
    }
    return true;
}

}  // namespace

const std::vector<relief::BrushKind>& allBrushKinds() {
    static const std::vector<relief::BrushKind> kinds = {
        relief::BrushKind::Deposit, relief::BrushKind::Remove, relief::BrushKind::Smooth,
        relief::BrushKind::Smudge, relief::BrushKind::UndoBrush};
    return kinds;
}

const char* brushKindName(relief::BrushKind kind) {
    switch (kind) {
        case relief::BrushKind::Smooth: return "Smooth";
        case relief::BrushKind::Smudge: return "Smudge";
        case relief::BrushKind::Deposit: return "Deposit";
        case relief::BrushKind::Remove: return "Remove";
        case relief::BrushKind::UndoBrush: return "Undo Brush";
    }
    return "Deposit";
}

const char* sculptModeName(relief::SculptMode mode) {
    switch (mode) {
        case relief::SculptMode::Normal: return "Normal";
        case relief::SculptMode::Raise: return "Raise";
        case relief::SculptMode::Lower: return "Lower";
    }
    return "Normal";
}

Range diameterRange() {
    return {0.5, 200.0};
}

Range strengthRange(relief::BrushKind kind) {
    const bool heightRate = kind == relief::BrushKind::Deposit || kind == relief::BrushKind::Remove;
    return {0.0, heightRate ? 2.0 : 1.0};
}

BrushSettings clampedBrush(BrushSettings s) {
    const Range d = diameterRange();
    const Range st = strengthRange(s.kind);
    auto clampFinite = [](double v, Range r, double fallback) {
        return std::isfinite(v) ? std::clamp(v, r.lo, r.hi) : fallback;
    };
    s.diameter = clampFinite(s.diameter, d, 10.0);
    s.strength = clampFinite(s.strength, st, 0.5 * st.hi);
    s.smoothness = clampFinite(s.smoothness, {0.0, 1.0}, 0.5);
    return s;
}

std::string brushProblem(const BrushSettings& s) {
    const Range d = diameterRange();
    if (!std::isfinite(s.diameter) || s.diameter < d.lo || s.diameter > d.hi) {
        return "Brush diameter must be between 0.5 and 200 mm";
    }
    const Range st = strengthRange(s.kind);
    if (!std::isfinite(s.strength) || s.strength < st.lo || s.strength > st.hi) {
        return "Brush strength is out of range";
    }
    if (!std::isfinite(s.smoothness) || s.smoothness < 0.0 || s.smoothness > 1.0) {
        return "Brush smoothness must be between 0 and 1";
    }
    return {};
}

relief::BrushParams toBrushParams(const BrushSettings& s) {
    relief::BrushParams p;
    p.kind = s.kind;
    p.diameter = s.diameter;
    p.strength = s.strength;
    p.smoothness = s.smoothness;
    p.mode = s.mode;
    p.preserveTransparency = s.preserveTransparency;
    p.custom = s.custom;
    return p;
}

std::shared_ptr<const relief::CustomBrush> customBrushFromImage(const RgbaImage& image, bool invert,
                                                                double angleRad, bool dynamicRotation,
                                                                std::string& problem) {
    problem.clear();
    constexpr std::size_t kMaxSide = 16384;
    if (image.width == 0 || image.height == 0) {
        problem = "The brush image is empty";
        return nullptr;
    }
    if (image.width > kMaxSide || image.height > kMaxSide ||
        image.rgba.size() != image.width * image.height * 4) {
        problem = "The brush image is malformed or too large";
        return nullptr;
    }
    if (!std::isfinite(angleRad)) {
        problem = "The brush angle is not valid";
        return nullptr;
    }
    const std::size_t longer = std::max(image.width, image.height);
    const std::size_t block = (longer + kMaxStampDimension - 1) / kMaxStampDimension;  // source pixels per cell
    const std::size_t w = (image.width + block - 1) / block;
    const std::size_t h = (image.height + block - 1) / block;
    std::vector<float> cells(w * h, std::numeric_limits<float>::quiet_NaN());
    bool anyOpaque = false;
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            double sum = 0.0;
            std::size_t opaque = 0, total = 0;
            for (std::size_t y = j * block; y < std::min(image.height, (j + 1) * block); ++y) {
                for (std::size_t x = i * block; x < std::min(image.width, (i + 1) * block); ++x) {
                    const std::uint8_t* px = &image.rgba[(y * image.width + x) * 4];
                    ++total;
                    if (px[3] >= 128) {
                        ++opaque;
                        sum += (0.2126 * px[0] + 0.7152 * px[1] + 0.0722 * px[2]) / 255.0;
                    }
                }
            }
            if (opaque * 2 >= total && opaque > 0) {
                const double v = sum / static_cast<double>(opaque);
                cells[j * w + i] = static_cast<float>(invert ? 1.0 - v : v);
                anyOpaque = true;
            }
        }
    }
    if (!anyOpaque) {
        problem = "The brush image has no opaque pixel";
        return nullptr;
    }
    auto cb = std::make_shared<relief::CustomBrush>();
    if (relief::HeightField::fromData(relief::Grid{w, h, 1.0, {0.0, 0.0}}, std::move(cells), cb->stamp) !=
        relief::Status::Ok) {
        problem = "The brush image could not be used";
        return nullptr;
    }
    cb->angleRad = angleRad;
    cb->dynamicRotation = dynamicRotation;
    return cb;
}

// --- Presets -----------------------------------------------------------------------------------------

BrushPresets::BrushPresets() {
    auto add = [this](const char* name, relief::BrushKind kind, double diameter, double strength,
                      double smoothness, relief::SculptMode mode) {
        BrushPreset p;
        p.name = name;
        p.brush.kind = kind;
        p.brush.diameter = diameter;
        p.brush.strength = strength;
        p.brush.smoothness = smoothness;
        p.brush.mode = mode;
        p.builtin = true;
        presets_.push_back(std::move(p));
    };
    add("Soft build-up", relief::BrushKind::Deposit, 20.0, 0.4, 1.0, relief::SculptMode::Normal);
    add("Fine detail", relief::BrushKind::Deposit, 3.0, 0.3, 0.3, relief::SculptMode::Normal);
    add("Carve out", relief::BrushKind::Remove, 8.0, 0.6, 0.5, relief::SculptMode::Normal);
    add("Raise only", relief::BrushKind::Deposit, 12.0, 0.5, 0.6, relief::SculptMode::Raise);
    add("Lower only", relief::BrushKind::Remove, 12.0, 0.5, 0.6, relief::SculptMode::Lower);
    add("Gentle smooth", relief::BrushKind::Smooth, 25.0, 0.3, 1.0, relief::SculptMode::Normal);
    add("Smudge", relief::BrushKind::Smudge, 15.0, 0.5, 0.8, relief::SculptMode::Normal);
}

const BrushPreset* BrushPresets::find(const std::string& name) const {
    for (const BrushPreset& p : presets_) {
        if (p.name == name) {
            return &p;
        }
    }
    return nullptr;
}

std::string BrushPresets::save(const std::string& name, const BrushSettings& brush) {
    if (name.empty() || name.size() > kMaxPresetNameBytes) {
        return "A preset name needs 1 to 64 characters";
    }
    for (const char c : name) {
        if (static_cast<unsigned char>(c) < 0x20 || c == 0x7F) {
            return "A preset name cannot contain control characters";
        }
    }
    if (const std::string why = brushProblem(brush); !why.empty()) {
        return why;
    }
    for (BrushPreset& p : presets_) {
        if (p.name == name) {
            if (p.builtin) {
                return "\"" + name + "\" is a built-in preset; choose another name";
            }
            p.brush = brush;
            p.brush.custom.reset();
            return {};
        }
    }
    BrushPreset p;
    p.name = name;
    p.brush = brush;
    p.brush.custom.reset();  // stamps are a session choice, not part of a preset
    presets_.push_back(std::move(p));
    return {};
}

bool BrushPresets::remove(const std::string& name) {
    for (auto it = presets_.begin(); it != presets_.end(); ++it) {
        if (it->name == name) {
            if (it->builtin) {
                return false;
            }
            presets_.erase(it);
            return true;
        }
    }
    return false;
}

namespace {

constexpr const char* kPresetHeader = "carvetto-brush-presets 1";
constexpr std::size_t kMaxUserPresets = 256;
constexpr std::size_t kMaxPresetTextBytes = std::size_t{1} << 20;

bool parseDouble(const std::string& text, double& out) {
    if (text.empty() || text.size() > 40) {
        return false;
    }
    std::istringstream in(text);
    in.imbue(std::locale::classic());
    in >> out;
    return !in.fail() && in.peek() == std::istringstream::traits_type::eof() && std::isfinite(out);
}

}  // namespace

std::string BrushPresets::serialiseUser() const {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out.precision(17);
    out << kPresetHeader << '\n';
    for (const BrushPreset& p : presets_) {
        if (p.builtin) {
            continue;
        }
        out << p.name << '\t' << static_cast<int>(p.brush.kind) << '\t' << p.brush.diameter << '\t'
            << p.brush.strength << '\t' << p.brush.smoothness << '\t' << static_cast<int>(p.brush.mode) << '\t'
            << (p.brush.preserveTransparency ? 1 : 0) << '\n';
    }
    return out.str();
}

std::size_t BrushPresets::loadUser(const std::string& text) {
    if (text.size() > kMaxPresetTextBytes) {
        return 0;
    }
    std::istringstream in(text);
    std::string line;
    if (!std::getline(in, line) || line != kPresetHeader) {
        return 0;
    }
    std::size_t added = 0;
    std::size_t users = 0;
    for (const BrushPreset& p : presets_) {
        users += p.builtin ? 0 : 1;
    }
    while (std::getline(in, line)) {
        std::vector<std::string> f;
        std::size_t start = 0;
        for (;;) {
            const std::size_t tab = line.find('\t', start);
            f.push_back(line.substr(start, tab == std::string::npos ? std::string::npos : tab - start));
            if (tab == std::string::npos) {
                break;
            }
            start = tab + 1;
        }
        double kind = 0, d = 0, st = 0, sm = 0, mode = 0, keepT = 0;
        if (f.size() != 7 || !parseDouble(f[1], kind) || !parseDouble(f[2], d) || !parseDouble(f[3], st) ||
            !parseDouble(f[4], sm) || !parseDouble(f[5], mode) || !parseDouble(f[6], keepT)) {
            continue;
        }
        if (kind < 0 || kind > 4 || kind != std::floor(kind) || mode < 0 || mode > 2 || mode != std::floor(mode)) {
            continue;
        }
        BrushSettings b;
        b.kind = static_cast<relief::BrushKind>(static_cast<int>(kind));
        b.diameter = d;
        b.strength = st;
        b.smoothness = sm;
        b.mode = static_cast<relief::SculptMode>(static_cast<int>(mode));
        b.preserveTransparency = keepT != 0.0;
        const bool known = find(f[0]) != nullptr;
        if (!known && users >= kMaxUserPresets) {
            continue;
        }
        if (save(f[0], b).empty()) {
            ++added;
            users += known ? 0 : 1;
        }
    }
    return added;
}

// --- Controller --------------------------------------------------------------------------------------

ComponentId defaultSculptTarget(const Document& doc) {
    if (const ComponentId one = doc.componentSelection().single()) {
        if (doc.findReliefComponent(one) != nullptr) {
            return one;
        }
    }
    ComponentId found;
    for (const document::LevelId l : doc.reliefLevelsOnSheet(doc.activeSheet())) {
        if (const document::ReliefLevel* level = doc.findReliefLevel(l)) {
            for (const document::ReliefComponent& c : level->components) {
                if (found) {
                    return {};
                }
                found = c.id;
            }
        }
    }
    return found;
}

EditResult SculptController::begin(const Document& doc, ComponentId id) {
    end();
    if (!id) {
        id = defaultSculptTarget(doc);
    }
    if (!id) {
        return refused("Select the component to sculpt in the Component Tree first");
    }
    const document::ReliefComponent* c = doc.findReliefComponent(id);
    if (c == nullptr) {
        return refused("That component no longer exists");
    }
    relief::SculptSession s;
    const relief::Status st = relief::SculptSession::begin(c->data.field, {}, s);
    if (st != relief::Status::Ok) {
        return refused(relief::statusMessage(st));
    }
    session_ = std::move(s);
    target_ = id;
    dirty_ = SculptDirty{true, true, {}, {}};
    return {};
}

void SculptController::end() {
    session_ = relief::SculptSession{};
    target_ = {};
    path_.clear();
    tailLength_ = 0.0;
    strokeOpen_ = false;
    tailApplied_ = false;
    openSteps_ = 0;
    strokeSteps_.clear();
}

bool SculptController::dirty() const {
    return session_.active() && !sameField(session_.working(), session_.baseline());
}

bool SculptController::documentMatches(const Document& doc) const {
    const document::ReliefComponent* c = doc.findReliefComponent(target_);
    return session_.active() && c != nullptr && sameField(c->data.field, session_.baseline());
}

EditResult SculptController::beginStroke(const BrushSettings& brush, relief::Point at) {
    if (!session_.active()) {
        return refused("Start sculpting first");
    }
    if (strokeOpen_) {
        endStroke();
    }
    const BrushSettings b = clampedBrush(brush);
    if (const std::string why = brushProblem(b); !why.empty() || !std::isfinite(at.x) || !std::isfinite(at.y)) {
        return refused(why.empty() ? "The pointer position is not valid" : why);
    }
    brush_ = b;
    path_.assign(1, at);
    tailLength_ = 0.0;
    openSteps_ = 0;
    tailApplied_ = false;
    strokeOpen_ = true;
    return applyTail();
}

// Re-applies the open tail of the stroke as one session step (replacing its previous step).
EditResult SculptController::applyTail() {
    if (tailApplied_) {
        session_.undoStroke();
        --openSteps_;
        tailApplied_ = false;
    }
    std::vector<relief::Dab> dabs;
    if (relief::dabsAlong(path_, 0.2 * brush_.diameter, brush_.diameter, dabs) != relief::Status::Ok) {
        return refused("The stroke could not be built");
    }
    const relief::Status st = session_.applyStroke(toBrushParams(brush_), dabs);
    if (st != relief::Status::Ok) {
        endStroke();
        return refused(relief::statusMessage(st));
    }
    tailApplied_ = true;
    ++openSteps_;
    {
        relief::Point lo = path_.front(), hi = path_.front();
        for (const relief::Point& p : path_) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
        }
        markDirty({lo.x - brush_.diameter, lo.y - brush_.diameter}, {hi.x + brush_.diameter, hi.y + brush_.diameter});
    }
    EditResult r;
    r.changed = false;  // the document is untouched until Keep
    return r;
}

void SculptController::markDirty(relief::Point lo, relief::Point hi) {
    if (dirty_.all) {
        return;
    }
    if (!dirty_.any) {
        dirty_.lo = lo;
        dirty_.hi = hi;
        dirty_.any = true;
        return;
    }
    dirty_.lo = {std::min(dirty_.lo.x, lo.x), std::min(dirty_.lo.y, lo.y)};
    dirty_.hi = {std::max(dirty_.hi.x, hi.x), std::max(dirty_.hi.y, hi.y)};
}

SculptDirty SculptController::takeDirty() {
    const SculptDirty d = dirty_;
    dirty_ = SculptDirty{};
    return d;
}

EditResult SculptController::extendStroke(relief::Point at) {
    if (!strokeOpen_) {
        return refused("No stroke is open");
    }
    if (!std::isfinite(at.x) || !std::isfinite(at.y)) {
        return refused("The pointer position is not valid");
    }
    const relief::Point last = path_.back();
    const double step = std::hypot(at.x - last.x, at.y - last.y);
    // Ignore jitter well under one dab spacing; the result only depends on the path shape.
    if (step < 0.02 * brush_.diameter) {
        return {};
    }
    if (path_.size() >= kMaxPathPoints) {
        return refused("The stroke is too long; release and start another");
    }
    path_.push_back(at);
    tailLength_ += step;
    EditResult r = applyTail();
    if (r.ok && tailLength_ >= kChunkDiameters * brush_.diameter) {
        // Freeze the tail: later moves start a new chunk at this point, so a move stays cheap.
        path_.assign(1, at);
        tailLength_ = 0.0;
        tailApplied_ = false;
    }
    return r;
}

void SculptController::endStroke() {
    if (strokeOpen_ && openSteps_ > 0) {
        strokeSteps_.push_back(openSteps_);
    }
    strokeOpen_ = false;
    path_.clear();
    tailLength_ = 0.0;
    tailApplied_ = false;
    openSteps_ = 0;
}

EditResult SculptController::stroke(const BrushSettings& brush, const std::vector<relief::Point>& path) {
    if (path.empty()) {
        return refused("The stroke has no points");
    }
    EditResult r = beginStroke(brush, path.front());
    for (std::size_t i = 1; r.ok && i < path.size(); ++i) {
        r = extendStroke(path[i]);
    }
    endStroke();
    return r;
}

bool SculptController::undoStroke() {
    if (strokeOpen_) {
        endStroke();
    }
    if (strokeSteps_.empty()) {
        return false;
    }
    std::size_t n = strokeSteps_.back();
    strokeSteps_.pop_back();
    bool any = false;
    while (n-- > 0 && session_.undoStroke()) {  // the session may have dropped the oldest steps
        any = true;
    }
    dirty_ = SculptDirty{true, true, {}, {}};
    return any;
}

EditResult SculptController::keep(Document& doc) {
    if (!session_.active()) {
        return refused("Start sculpting first");
    }
    endStroke();
    const document::ReliefComponent* c = doc.findReliefComponent(target_);
    if (c == nullptr) {
        return refused("The sculpted component no longer exists");
    }
    const relief::HeightField& w = session_.working();
    if (c->data.field.width() != w.width() || c->data.field.height() != w.height()) {
        return refused("The component changed while sculpting; sculpt again");
    }
    if (!dirty()) {
        return refused("Nothing to keep: no strokes since the last Keep");
    }
    relief::Component next = c->data;
    next.field = w;
    doc.undoStack().breakCoalescing();
    const bool ok = doc.execute(std::make_unique<document::ReplaceReliefComponentCommand>(target_, std::move(next)));
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refused("The model could not take the sculpted component (size limits)");
    }
    session_.keep();
    strokeSteps_.clear();
    EditResult r;
    r.changed = true;
    return r;
}

void SculptController::discard() {
    endStroke();
    session_.discard();
    strokeSteps_.clear();
    dirty_ = SculptDirty{true, true, {}, {}};
}

}  // namespace carvetto::viewmodel
