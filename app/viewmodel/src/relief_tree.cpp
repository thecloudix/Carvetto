#include "carvetto/viewmodel/relief_tree.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <set>
#include <sstream>
#include <utility>

#include "carvetto/document/object_property_commands.hpp"
#include "carvetto/document/relief_commands.hpp"

namespace carvetto::viewmodel {

namespace {

using document::ComponentId;
using document::Document;
using document::LevelId;

EditResult refused(std::string message) {
    EditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

EditResult run(Document& doc, std::unique_ptr<document::Command> cmd, const char* failure) {
    doc.undoStack().breakCoalescing();
    const bool ok = doc.execute(std::move(cmd));
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refused(failure);
    }
    EditResult r;
    r.changed = true;
    return r;
}

class MacroGuard {
   public:
    MacroGuard(Document& doc, const char* text) : doc_(doc) {
        doc_.undoStack().breakCoalescing();
        doc_.undoStack().beginMacro(text);
    }
    ~MacroGuard() { finish(); }
    void finish() {
        if (open_) {
            open_ = false;
            doc_.undoStack().endMacro();
            doc_.undoStack().breakCoalescing();
        }
    }

   private:
    Document& doc_;
    bool open_{true};
};

bool parseFinite(const std::string& text, double& out) {
    const char* begin = text.c_str();
    while (*begin == ' ' || *begin == '\t') {
        ++begin;
    }
    if (*begin == '\0') {
        return false;
    }
    char* end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end == begin) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (*end != '\0' || !std::isfinite(v) || std::fabs(v) > 1.0e9) {
        return false;
    }
    out = v;
    return true;
}

std::string fmt(double v) {
    std::ostringstream os;
    os.precision(10);
    os << v;
    return os.str();
}

constexpr double kRadPerDeg = std::numbers::pi / 180.0;

}  // namespace

const char* combineModeName(relief::CombineMode mode) {
    switch (mode) {
        case relief::CombineMode::Add: return "Add";
        case relief::CombineMode::Subtract: return "Subtract";
        case relief::CombineMode::MergeHigh: return "Merge High";
        case relief::CombineMode::MergeLow: return "Merge Low";
        case relief::CombineMode::Multiply: return "Multiply";
    }
    return "Add";
}

const std::vector<relief::CombineMode>& allCombineModes() {
    static const std::vector<relief::CombineMode> modes{
        relief::CombineMode::Add, relief::CombineMode::Subtract, relief::CombineMode::MergeHigh,
        relief::CombineMode::MergeLow, relief::CombineMode::Multiply};
    return modes;
}

std::vector<ReliefTreeRow> buildReliefTree(const Document& doc, document::SheetId sheet) {
    std::vector<ReliefTreeRow> rows;
    const std::vector<LevelId> levels = doc.reliefLevelsOnSheet(sheet);
    for (std::size_t li = levels.size(); li-- > 0;) {
        const document::ReliefLevel* level = doc.findReliefLevel(levels[li]);
        if (level == nullptr) {
            continue;
        }
        ReliefTreeRow row;
        row.isLevel = true;
        row.level = level->id;
        row.name = level->name;
        row.mode = level->mode;
        row.visible = level->visible;
        row.index = li;
        rows.push_back(row);
        for (std::size_t ci = level->components.size(); ci-- > 0;) {
            const document::ReliefComponent& c = level->components[ci];
            ReliefTreeRow cr;
            cr.level = level->id;
            cr.component = c.id;
            cr.name = c.data.name;
            cr.mode = c.data.mode;
            cr.visible = c.data.visible;
            cr.selected = doc.componentSelection().contains(c.id);
            cr.index = ci;
            rows.push_back(cr);
        }
    }
    return rows;
}

EditResult addReliefLevel(Document& doc, document::SheetId sheet, const std::string& name, LevelId* created) {
    if (doc.findSheet(sheet) == nullptr) {
        return refused("the sheet no longer exists");
    }
    if (doc.reliefLevels().size() >= document::kMaxReliefLevels) {
        return refused("the document already has the maximum number of levels");
    }
    std::string use = name;
    if (use.empty()) {
        std::set<std::string> used;
        for (const document::ReliefLevel& l : doc.reliefLevels()) {
            used.insert(l.name);
        }
        const std::size_t base = doc.reliefLevelsOnSheet(sheet).size() + 1;
        for (std::size_t n = base;; ++n) {
            use = "Level " + std::to_string(n);
            if (!used.contains(use)) {
                break;
            }
        }
    } else if (const std::string why = document::nameProblem(use, true); !why.empty()) {
        return refused(why);
    }
    auto cmd = std::make_unique<document::AddReliefLevelCommand>(sheet, use);
    const document::AddReliefLevelCommand* raw = cmd.get();
    EditResult r = run(doc, std::move(cmd), "the level cannot be added");
    if (r.changed && created != nullptr) {
        *created = raw->createdId();
    }
    return r;
}

EditResult renameReliefComponent(Document& doc, ComponentId id, const std::string& name) {
    const document::ReliefComponent* c = doc.findReliefComponent(id);
    if (c == nullptr) {
        return refused("the component no longer exists");
    }
    if (const std::string why = document::nameProblem(name, true); !why.empty()) {
        return refused(why);
    }
    if (c->data.name == name) {
        return {};
    }
    return run(doc, std::make_unique<document::RenameReliefComponentCommand>(id, name),
               "the component cannot be renamed");
}

EditResult renameReliefLevel(Document& doc, LevelId id, const std::string& name) {
    const document::ReliefLevel* l = doc.findReliefLevel(id);
    if (l == nullptr) {
        return refused("the level no longer exists");
    }
    if (const std::string why = document::nameProblem(name, true); !why.empty()) {
        return refused(why);
    }
    if (l->name == name) {
        return {};
    }
    return run(doc, std::make_unique<document::RenameReliefLevelCommand>(id, name), "the level cannot be renamed");
}

EditResult setReliefComponentMode(Document& doc, ComponentId id, relief::CombineMode mode) {
    const document::ReliefComponent* c = doc.findReliefComponent(id);
    if (c == nullptr) {
        return refused("the component no longer exists");
    }
    if (c->data.mode == mode) {
        return {};
    }
    return run(doc, std::make_unique<document::SetReliefComponentModeCommand>(id, mode),
               "the combine mode cannot be changed");
}

EditResult setReliefLevelMode(Document& doc, LevelId id, relief::CombineMode mode) {
    const document::ReliefLevel* l = doc.findReliefLevel(id);
    if (l == nullptr) {
        return refused("the level no longer exists");
    }
    if (l->mode == mode) {
        return {};
    }
    return run(doc, std::make_unique<document::SetReliefLevelPropertiesCommand>(id, mode, l->baseHeight, l->visible),
               "the combine mode cannot be changed");
}

EditResult setReliefComponentVisible(Document& doc, ComponentId id, bool visible) {
    const document::ReliefComponent* c = doc.findReliefComponent(id);
    if (c == nullptr) {
        return refused("the component no longer exists");
    }
    if (c->data.visible == visible) {
        return {};
    }
    document::ReliefComponentProperties p{c->data.shapeHeightPercent, c->data.baseHeight, c->data.fade,
                                          c->data.tilt, visible};
    return run(doc, std::make_unique<document::SetReliefComponentPropertiesCommand>(id, p),
               "the visibility cannot be changed");
}

EditResult setReliefLevelVisible(Document& doc, LevelId id, bool visible) {
    const document::ReliefLevel* l = doc.findReliefLevel(id);
    if (l == nullptr) {
        return refused("the level no longer exists");
    }
    if (l->visible == visible) {
        return {};
    }
    return run(doc, std::make_unique<document::SetReliefLevelPropertiesCommand>(id, l->mode, l->baseHeight, visible),
               "the visibility cannot be changed");
}

EditResult setAllReliefComponentsVisible(Document& doc, document::SheetId sheet, bool visible) {
    std::vector<ComponentId> targets;
    for (const LevelId lid : doc.reliefLevelsOnSheet(sheet)) {
        for (const document::ReliefComponent& c : doc.findReliefLevel(lid)->components) {
            if (c.data.visible != visible) {
                targets.push_back(c.id);
            }
        }
    }
    if (targets.empty()) {
        return {};
    }
    MacroGuard guard(doc, visible ? "Show all components" : "Hide all components");
    EditResult result;
    for (const ComponentId id : targets) {
        EditResult r = setReliefComponentVisible(doc, id, visible);
        if (!r.ok) {
            return r;
        }
        result.changed = result.changed || r.changed;
    }
    return result;
}

EditResult showOnlyReliefComponent(Document& doc, ComponentId id) {
    const std::optional<Document::ReliefLocation> where = doc.locateReliefComponent(id);
    if (!where) {
        return refused("the component no longer exists");
    }
    const document::ReliefLevel* level = doc.findReliefLevel(where->level);
    std::vector<std::pair<ComponentId, bool>> wanted;
    for (const document::ReliefLevel& l : doc.reliefLevels()) {
        if (l.sheet != level->sheet) {
            continue;
        }
        for (const document::ReliefComponent& c : l.components) {
            const bool want = c.id == id;
            if (c.data.visible != want) {
                wanted.emplace_back(c.id, want);
            }
        }
    }
    if (wanted.empty()) {
        return {};
    }
    MacroGuard guard(doc, "Show only this component");
    EditResult result;
    for (const auto& [cid, want] : wanted) {
        EditResult r = setReliefComponentVisible(doc, cid, want);
        if (!r.ok) {
            return r;
        }
        result.changed = result.changed || r.changed;
    }
    return result;
}

EditResult deleteReliefComponents(Document& doc, const std::vector<ComponentId>& ids) {
    std::vector<ComponentId> existing;
    for (const ComponentId id : ids) {
        if (doc.findReliefComponent(id) != nullptr && std::find(existing.begin(), existing.end(), id) == existing.end()) {
            existing.push_back(id);
        }
    }
    if (existing.empty()) {
        return refused("there is no component to delete");
    }
    return run(doc, std::make_unique<document::DeleteReliefComponentsCommand>(std::move(existing)),
               "the components cannot be deleted");
}

EditResult deleteReliefLevel(Document& doc, LevelId id) {
    if (doc.findReliefLevel(id) == nullptr) {
        return refused("the level no longer exists");
    }
    return run(doc, std::make_unique<document::DeleteReliefLevelCommand>(id), "the level cannot be deleted");
}

EditResult duplicateReliefComponent(Document& doc, ComponentId id, ComponentId* created) {
    if (doc.findReliefComponent(id) == nullptr) {
        return refused("the component no longer exists");
    }
    auto cmd = std::make_unique<document::DuplicateReliefComponentCommand>(id);
    const document::DuplicateReliefComponentCommand* raw = cmd.get();
    EditResult r = run(doc, std::move(cmd), "the component cannot be duplicated (a limit is reached)");
    if (r.changed && created != nullptr) {
        *created = raw->createdId();
    }
    return r;
}

EditResult moveReliefComponent(Document& doc, ComponentId id, bool up) {
    const std::optional<Document::ReliefLocation> where = doc.locateReliefComponent(id);
    if (!where) {
        return refused("the component no longer exists");
    }
    const std::size_t count = doc.findReliefLevel(where->level)->components.size();
    if (up ? where->index + 1 >= count : where->index == 0) {
        return {};
    }
    const std::size_t target = up ? where->index + 1 : where->index - 1;
    return run(doc, std::make_unique<document::ReorderReliefComponentCommand>(id, target),
               "the component cannot be moved");
}

EditResult moveReliefLevel(Document& doc, LevelId id, bool up) {
    const std::optional<std::size_t> index = doc.reliefLevelIndex(id);
    if (!index) {
        return refused("the level no longer exists");
    }
    const std::size_t count = doc.reliefLevelsOnSheet(doc.findReliefLevel(id)->sheet).size();
    if (up ? *index + 1 >= count : *index == 0) {
        return {};
    }
    return run(doc, std::make_unique<document::ReorderReliefLevelCommand>(id, up ? *index + 1 : *index - 1),
               "the level cannot be moved");
}

EditResult moveReliefComponentToLevel(Document& doc, ComponentId id, LevelId level) {
    const std::optional<Document::ReliefLocation> where = doc.locateReliefComponent(id);
    if (!where) {
        return refused("the component no longer exists");
    }
    const document::ReliefLevel* target = doc.findReliefLevel(level);
    if (target == nullptr) {
        return refused("the level no longer exists");
    }
    if (where->level == level) {
        return {};
    }
    if (target->sheet != doc.findReliefLevel(where->level)->sheet) {
        return refused("a component cannot move to a level of another sheet");
    }
    return run(doc, std::make_unique<document::MoveReliefComponentToLevelCommand>(id, level),
               "the component cannot be moved to that level");
}

EditResult dropReliefComponent(Document& doc, ComponentId id, LevelId level, std::size_t index) {
    const std::optional<Document::ReliefLocation> where = doc.locateReliefComponent(id);
    if (!where) {
        return refused("the component no longer exists");
    }
    const document::ReliefLevel* target = doc.findReliefLevel(level);
    if (target == nullptr) {
        return refused("the level no longer exists");
    }
    if (where->level == level) {
        const std::size_t last = target->components.size() - 1;
        const std::size_t to = std::min(index, last);
        if (to == where->index) {
            return {};
        }
        return run(doc, std::make_unique<document::ReorderReliefComponentCommand>(id, to),
                   "the component cannot be moved");
    }
    if (target->sheet != doc.findReliefLevel(where->level)->sheet) {
        return refused("a component cannot move to a level of another sheet");
    }
    const std::size_t to = std::min(index, target->components.size());
    return run(doc, std::make_unique<document::MoveReliefComponentToLevelCommand>(id, level, to),
               "the component cannot be moved to that level");
}

void selectReliefComponents(Document& doc, const std::vector<ComponentId>& ids) {
    std::vector<ComponentId> keep;
    for (const ComponentId id : ids) {
        if (doc.findReliefComponent(id) != nullptr) {
            keep.push_back(id);
        }
    }
    doc.componentSelection().assign(keep);
}

bool describeReliefComponent(const Document& doc, ComponentId id, ReliefPropertiesForm& out) {
    const document::ReliefComponent* c = doc.findReliefComponent(id);
    if (c == nullptr) {
        return false;
    }
    const relief::Component& d = c->data;
    out = ReliefPropertiesForm{};
    out.name = d.name;
    out.mode = d.mode;
    out.shapeHeightPercent = fmt(d.shapeHeightPercent);
    out.baseHeight = fmt(d.baseHeight);
    out.visible = d.visible;
    out.fadeEnabled = d.fade.enabled;
    out.fadePercent = fmt(d.fade.percent);
    out.fadeAnchorX = fmt(d.fade.anchor.x);
    out.fadeAnchorY = fmt(d.fade.anchor.y);
    out.fadeSecondX = fmt(d.fade.second.x);
    out.fadeSecondY = fmt(d.fade.second.y);
    out.tiltEnabled = d.tilt.enabled;
    out.tiltAngleDeg = fmt(d.tilt.angleRad / kRadPerDeg);
    out.tiltPivotX = fmt(d.tilt.pivot.x);
    out.tiltPivotY = fmt(d.tilt.pivot.y);
    out.tiltRaiseX = fmt(d.tilt.raise.x);
    out.tiltRaiseY = fmt(d.tilt.raise.y);
    out.fieldInfo = std::to_string(d.field.width()) + " x " + std::to_string(d.field.height()) + " cells, " +
                    fmt(d.field.cellSize()) + " mm";
    return true;
}

EditResult applyReliefProperties(Document& doc, ComponentId id, const ReliefPropertiesForm& form) {
    const document::ReliefComponent* c = doc.findReliefComponent(id);
    if (c == nullptr) {
        return refused("the component no longer exists");
    }
    if (const std::string why = document::nameProblem(form.name, true); !why.empty()) {
        return refused(why);
    }
    document::ReliefComponentProperties p;
    p.visible = form.visible;
    struct Num {
        const std::string* text;
        double* target;
        const char* label;
    };
    double angleDeg = 0.0;
    relief::Fade fade;
    relief::Tilt tilt;
    const Num numbers[] = {
        {&form.shapeHeightPercent, &p.shapeHeightPercent, "Shape Height"},
        {&form.baseHeight, &p.baseHeight, "Base Height"},
        {&form.fadePercent, &fade.percent, "Fade percent"},
        {&form.fadeAnchorX, &fade.anchor.x, "Fade anchor X"},
        {&form.fadeAnchorY, &fade.anchor.y, "Fade anchor Y"},
        {&form.fadeSecondX, &fade.second.x, "Fade second point X"},
        {&form.fadeSecondY, &fade.second.y, "Fade second point Y"},
        {&form.tiltAngleDeg, &angleDeg, "Tilt angle"},
        {&form.tiltPivotX, &tilt.pivot.x, "Tilt pivot X"},
        {&form.tiltPivotY, &tilt.pivot.y, "Tilt pivot Y"},
        {&form.tiltRaiseX, &tilt.raise.x, "Tilt raise X"},
        {&form.tiltRaiseY, &tilt.raise.y, "Tilt raise Y"},
    };
    for (const Num& n : numbers) {
        if (!parseFinite(*n.text, *n.target)) {
            return refused(std::string(n.label) + " is not a valid number");
        }
    }
    if (p.shapeHeightPercent < 0.0 || p.shapeHeightPercent > 1000.0) {
        return refused("Shape Height must be between 0 and 1000 %");
    }
    if (std::fabs(angleDeg) >= 89.0) {
        return refused("Tilt angle must be less than 89 degrees");
    }
    fade.enabled = form.fadeEnabled;
    tilt.enabled = form.tiltEnabled;
    tilt.angleRad = angleDeg * kRadPerDeg;
    p.fade = fade;
    p.tilt = tilt;
    // Dry-run the properties so a refusal leaves the document alone.
    relief::Component probe;
    probe.shapeHeightPercent = p.shapeHeightPercent;
    probe.baseHeight = p.baseHeight;
    probe.fade = p.fade;
    probe.tilt = p.tilt;
    if (relief::validateProperties(probe) != relief::Status::Ok) {
        return refused("the fade or tilt direction needs two different points");
    }
    MacroGuard guard(doc, "Edit relief component");
    EditResult result;
    if (EditResult r = renameReliefComponent(doc, id, form.name); !r.ok) {
        return r;
    } else {
        result.changed = result.changed || r.changed;
    }
    if (EditResult r = setReliefComponentMode(doc, id, form.mode); !r.ok) {
        return r;
    } else {
        result.changed = result.changed || r.changed;
    }
    const relief::Component& d = c->data;
    const bool same = d.shapeHeightPercent == p.shapeHeightPercent && d.baseHeight == p.baseHeight &&
                      d.visible == p.visible && d.fade.enabled == p.fade.enabled && d.tilt.enabled == p.tilt.enabled &&
                      d.fade.percent == p.fade.percent && d.fade.anchor.x == p.fade.anchor.x &&
                      d.fade.anchor.y == p.fade.anchor.y && d.fade.second.x == p.fade.second.x &&
                      d.fade.second.y == p.fade.second.y && d.tilt.angleRad == p.tilt.angleRad &&
                      d.tilt.pivot.x == p.tilt.pivot.x && d.tilt.pivot.y == p.tilt.pivot.y &&
                      d.tilt.raise.x == p.tilt.raise.x && d.tilt.raise.y == p.tilt.raise.y;
    if (!same) {
        EditResult r = run(doc, std::make_unique<document::SetReliefComponentPropertiesCommand>(id, p),
                           "the properties cannot be applied");
        if (!r.ok) {
            return r;
        }
        result.changed = true;
    }
    return result;
}

EditResult applyReliefLevelProperties(Document& doc, LevelId id, const std::string& name, relief::CombineMode mode,
                                      const std::string& baseHeight) {
    const document::ReliefLevel* l = doc.findReliefLevel(id);
    if (l == nullptr) {
        return refused("the level no longer exists");
    }
    double base = 0.0;
    if (!parseFinite(baseHeight, base)) {
        return refused("Base Height is not a valid number");
    }
    if (const std::string why = document::nameProblem(name, true); !why.empty()) {
        return refused(why);
    }
    MacroGuard guard(doc, "Edit relief level");
    EditResult result;
    if (EditResult r = renameReliefLevel(doc, id, name); !r.ok) {
        return r;
    } else {
        result.changed = result.changed || r.changed;
    }
    if (l->mode != mode || l->baseHeight != base) {
        EditResult r = run(doc, std::make_unique<document::SetReliefLevelPropertiesCommand>(id, mode, base, l->visible),
                           "the level properties cannot be applied");
        if (!r.ok) {
            return r;
        }
        result.changed = true;
    }
    return result;
}

}  // namespace carvetto::viewmodel
