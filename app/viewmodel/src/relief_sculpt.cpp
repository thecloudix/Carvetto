#include "carvetto/viewmodel/relief_sculpt.hpp"

#include <algorithm>
#include <cmath>
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
    return p;
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
    if (const std::string why = brushProblem(brush); !why.empty()) {
        return why;
    }
    for (BrushPreset& p : presets_) {
        if (p.name == name) {
            if (p.builtin) {
                return "\"" + name + "\" is a built-in preset; choose another name";
            }
            p.brush = brush;
            return {};
        }
    }
    BrushPreset p;
    p.name = name;
    p.brush = brush;
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
    return {};
}

void SculptController::end() {
    session_ = relief::SculptSession{};
    target_ = {};
    path_.clear();
    strokeOpen_ = false;
    strokeApplied_ = false;
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
    strokeOpen_ = true;
    strokeApplied_ = false;
    return extendStroke(at);
}

EditResult SculptController::extendStroke(relief::Point at) {
    if (!strokeOpen_) {
        return refused("No stroke is open");
    }
    if (!std::isfinite(at.x) || !std::isfinite(at.y)) {
        return refused("The pointer position is not valid");
    }
    const relief::Point last = path_.back();
    const bool first = !strokeApplied_;
    if (!first) {
        // Ignore jitter well under one dab spacing; the result only depends on the path shape.
        if (std::hypot(at.x - last.x, at.y - last.y) < 0.02 * brush_.diameter) {
            return {};
        }
        if (path_.size() >= kMaxPathPoints) {
            return refused("The stroke is too long; release and start another");
        }
        path_.push_back(at);
        session_.undoStroke();  // re-apply the whole path as one stroke
    }
    std::vector<relief::Dab> dabs;
    if (relief::dabsAlong(path_, 0.2 * brush_.diameter, brush_.diameter, dabs) != relief::Status::Ok) {
        return refused("The stroke could not be built");
    }
    const relief::Status st = session_.applyStroke(toBrushParams(brush_), dabs);
    if (st != relief::Status::Ok) {
        strokeOpen_ = false;
        return refused(relief::statusMessage(st));
    }
    strokeApplied_ = true;
    EditResult r;
    r.changed = false;  // the document is untouched until Keep
    return r;
}

void SculptController::endStroke() {
    strokeOpen_ = false;
    path_.clear();
    strokeApplied_ = false;
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
    return session_.undoStroke();
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
    EditResult r;
    r.changed = true;
    return r;
}

void SculptController::discard() {
    endStroke();
    session_.discard();
}

}  // namespace carvetto::viewmodel
