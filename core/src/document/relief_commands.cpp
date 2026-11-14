#include "carvetto/document/relief_commands.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "carvetto/document/document.hpp"

namespace carvetto::document {

namespace {

bool nameOk(const std::string& name) {
    return !name.empty() && name.size() <= kMaxReliefNameBytes;
}

std::uint64_t cellsOf(const relief::HeightField& f) {
    return static_cast<std::uint64_t>(f.width()) * f.height();
}

/// True when one more component of `cells` cells fits every cap and an id is available.
bool roomForComponent(const Document& doc, std::uint64_t cells, std::uint64_t ids) {
    return doc.reliefComponentCount() < kMaxReliefComponents &&
           doc.reliefCellCount() + cells <= kMaxReliefTotalCells && doc.idGenerator().canAllocate(ids);
}

}  // namespace

// --- AddReliefLevelCommand ---------------------------------------------------------------------

bool AddReliefLevelCommand::apply(Document& doc) {
    if (id_.isNull()) {
        if (!doc.findSheet(sheet_) || !nameOk(name_) || doc.reliefLevels().size() >= kMaxReliefLevels ||
            !doc.idGenerator().canAllocate(1)) {
            return false;
        }
        id_ = doc.allocateLevelId();
    }
    ReliefLevel level;
    level.id = id_;
    level.sheet = sheet_;
    level.name = name_;
    return !doc.insertReliefLevel(std::move(level), index_).isNull();
}

void AddReliefLevelCommand::revert(Document& doc) {
    doc.takeReliefLevel(id_);
}

// --- AddReliefComponentCommand -----------------------------------------------------------------

bool AddReliefComponentCommand::apply(Document& doc) {
    if (held_.id.isNull()) {
        if (!doc.findReliefLevel(level_) || !nameOk(held_.data.name) ||
            !reliefComponentProblem(held_.data).empty() ||
            !roomForComponent(doc, cellsOf(held_.data.field), 1)) {
            return false;
        }
        held_.id = doc.allocateComponentId();
    }
    return !doc.insertReliefComponent(level_, std::move(held_), index_).isNull();
}

void AddReliefComponentCommand::revert(Document& doc) {
    const ComponentId id = held_.id;
    if (auto c = doc.takeReliefComponent(id)) {
        held_ = std::move(*c);
    }
}

// --- ReplaceReliefComponentCommand ---------------------------------------------------------------

bool ReplaceReliefComponentCommand::apply(Document& doc) {
    ReliefComponent* c = doc.findReliefComponent(id_);
    if (!c || !nameOk(held_.name) || !reliefComponentProblem(held_).empty()) {
        return false;
    }
    const std::uint64_t oldCells = cellsOf(c->data.field);
    const std::uint64_t newCells = cellsOf(held_.field);
    if (doc.reliefCellCount() - oldCells + newCells > kMaxReliefTotalCells) {
        return false;
    }
    std::swap(c->data, held_);
    return true;
}

void ReplaceReliefComponentCommand::revert(Document& doc) {
    if (ReliefComponent* c = doc.findReliefComponent(id_)) {
        std::swap(c->data, held_);
    }
}

// --- Deletes -----------------------------------------------------------------------------------

bool DeleteReliefComponentsCommand::apply(Document& doc) {
    removed_.clear();
    for (const ComponentId id : ids_) {
        const auto where = doc.locateReliefComponent(id);
        if (!where) {
            continue;  // unknown, or listed twice
        }
        removed_.push_back({where->level, where->index, std::move(*doc.takeReliefComponent(id))});
    }
    return !removed_.empty();
}

void DeleteReliefComponentsCommand::revert(Document& doc) {
    for (auto it = removed_.rbegin(); it != removed_.rend(); ++it) {
        doc.insertReliefComponent(it->level, std::move(it->component), it->index);
    }
    removed_.clear();
}

bool DeleteReliefLevelCommand::apply(Document& doc) {
    const auto index = doc.reliefLevelIndex(id_);
    if (!index) {
        return false;
    }
    index_ = *index;
    removed_ = doc.takeReliefLevel(id_);
    return true;
}

void DeleteReliefLevelCommand::revert(Document& doc) {
    if (removed_) {
        doc.insertReliefLevel(std::move(*removed_), index_);
        removed_.reset();
    }
}

// --- Renames and properties ----------------------------------------------------------------------

bool RenameReliefComponentCommand::apply(Document& doc) {
    ReliefComponent* c = doc.findReliefComponent(id_);
    if (!c || !nameOk(after_) || c->data.name == after_) {
        return false;
    }
    before_ = c->data.name;
    c->data.name = after_;
    return true;
}

void RenameReliefComponentCommand::revert(Document& doc) {
    if (ReliefComponent* c = doc.findReliefComponent(id_)) {
        c->data.name = before_;
    }
}

bool RenameReliefLevelCommand::apply(Document& doc) {
    ReliefLevel* l = doc.findReliefLevel(id_);
    if (!l || !nameOk(after_) || l->name == after_) {
        return false;
    }
    before_ = l->name;
    l->name = after_;
    return true;
}

void RenameReliefLevelCommand::revert(Document& doc) {
    if (ReliefLevel* l = doc.findReliefLevel(id_)) {
        l->name = before_;
    }
}

bool SetReliefComponentModeCommand::apply(Document& doc) {
    ReliefComponent* c = doc.findReliefComponent(id_);
    if (!c || c->data.mode == after_) {
        return false;
    }
    before_ = c->data.mode;
    c->data.mode = after_;
    return true;
}

void SetReliefComponentModeCommand::revert(Document& doc) {
    if (ReliefComponent* c = doc.findReliefComponent(id_)) {
        c->data.mode = before_;
    }
}

namespace {
bool sameFade(const relief::Fade& a, const relief::Fade& b) {
    return a.enabled == b.enabled && a.anchor.x == b.anchor.x && a.anchor.y == b.anchor.y &&
           a.second.x == b.second.x && a.second.y == b.second.y && a.percent == b.percent;
}
bool sameTilt(const relief::Tilt& a, const relief::Tilt& b) {
    return a.enabled == b.enabled && a.pivot.x == b.pivot.x && a.pivot.y == b.pivot.y &&
           a.raise.x == b.raise.x && a.raise.y == b.raise.y && a.angleRad == b.angleRad;
}
bool sameProps(const ReliefComponentProperties& a, const ReliefComponentProperties& b) {
    return a.shapeHeightPercent == b.shapeHeightPercent && a.baseHeight == b.baseHeight &&
           sameFade(a.fade, b.fade) && sameTilt(a.tilt, b.tilt) && a.visible == b.visible;
}
}  // namespace

bool SetReliefComponentPropertiesCommand::apply(Document& doc) {
    ReliefComponent* c = doc.findReliefComponent(id_);
    if (!c) {
        return false;
    }
    relief::Component probe;  // properties only; validated without copying the field
    probe.shapeHeightPercent = after_.shapeHeightPercent;
    probe.baseHeight = after_.baseHeight;
    probe.fade = after_.fade;
    probe.tilt = after_.tilt;
    if (!reliefPropertiesProblem(probe).empty()) {
        return false;
    }
    before_ = {c->data.shapeHeightPercent, c->data.baseHeight, c->data.fade, c->data.tilt, c->data.visible};
    if (sameProps(before_, after_)) {
        return false;
    }
    c->data.shapeHeightPercent = after_.shapeHeightPercent;
    c->data.baseHeight = after_.baseHeight;
    c->data.fade = after_.fade;
    c->data.tilt = after_.tilt;
    c->data.visible = after_.visible;
    return true;
}

void SetReliefComponentPropertiesCommand::revert(Document& doc) {
    if (ReliefComponent* c = doc.findReliefComponent(id_)) {
        c->data.shapeHeightPercent = before_.shapeHeightPercent;
        c->data.baseHeight = before_.baseHeight;
        c->data.fade = before_.fade;
        c->data.tilt = before_.tilt;
        c->data.visible = before_.visible;
    }
}

bool SetReliefComponentPropertiesCommand::mergeWith(const Command& next) {
    const auto* n = dynamic_cast<const SetReliefComponentPropertiesCommand*>(&next);
    if (!n || n->id_ != id_) {
        return false;
    }
    after_ = n->after_;
    return true;
}

bool SetReliefLevelPropertiesCommand::apply(Document& doc) {
    ReliefLevel* l = doc.findReliefLevel(id_);
    if (!l || !std::isfinite(after_.baseHeight)) {
        return false;
    }
    before_ = {l->mode, l->baseHeight, l->visible};
    if (before_.mode == after_.mode && before_.baseHeight == after_.baseHeight &&
        before_.visible == after_.visible) {
        return false;
    }
    l->mode = after_.mode;
    l->baseHeight = after_.baseHeight;
    l->visible = after_.visible;
    return true;
}

void SetReliefLevelPropertiesCommand::revert(Document& doc) {
    if (ReliefLevel* l = doc.findReliefLevel(id_)) {
        l->mode = before_.mode;
        l->baseHeight = before_.baseHeight;
        l->visible = before_.visible;
    }
}

bool SetReliefLevelPropertiesCommand::mergeWith(const Command& next) {
    const auto* n = dynamic_cast<const SetReliefLevelPropertiesCommand*>(&next);
    if (!n || n->id_ != id_) {
        return false;
    }
    after_ = n->after_;
    return true;
}

// --- Reordering and moving -----------------------------------------------------------------------

namespace {
/// Moves the component at `from` to `to` within one level, keeping the others' relative order.
void moveWithin(std::vector<ReliefComponent>& v, std::size_t from, std::size_t to) {
    if (from < to) {
        std::rotate(v.begin() + static_cast<std::ptrdiff_t>(from), v.begin() + static_cast<std::ptrdiff_t>(from) + 1,
                    v.begin() + static_cast<std::ptrdiff_t>(to) + 1);
    } else {
        std::rotate(v.begin() + static_cast<std::ptrdiff_t>(to), v.begin() + static_cast<std::ptrdiff_t>(from),
                    v.begin() + static_cast<std::ptrdiff_t>(from) + 1);
    }
}
}  // namespace

bool ReorderReliefComponentCommand::apply(Document& doc) {
    const auto where = doc.locateReliefComponent(id_);
    if (!where) {
        return false;
    }
    ReliefLevel* level = doc.findReliefLevel(where->level);
    if (after_ >= level->components.size() || after_ == where->index) {
        return false;
    }
    before_ = where->index;
    moveWithin(level->components, before_, after_);
    return true;
}

void ReorderReliefComponentCommand::revert(Document& doc) {
    if (const auto where = doc.locateReliefComponent(id_)) {
        moveWithin(doc.findReliefLevel(where->level)->components, where->index, before_);
    }
}

bool ReorderReliefLevelCommand::apply(Document& doc) {
    const auto index = doc.reliefLevelIndex(id_);
    if (!index || after_ == *index) {
        return false;
    }
    before_ = *index;
    return doc.moveReliefLevel(id_, after_);
}

void ReorderReliefLevelCommand::revert(Document& doc) {
    doc.moveReliefLevel(id_, before_);
}

bool MoveReliefComponentToLevelCommand::apply(Document& doc) {
    const auto where = doc.locateReliefComponent(id_);
    const ReliefLevel* target = doc.findReliefLevel(target_);
    if (!where || !target || where->level == target_ ||
        target->sheet != doc.findReliefLevel(where->level)->sheet) {
        return false;
    }
    fromLevel_ = where->level;
    fromIndex_ = where->index;
    const std::vector<ComponentId> keep = doc.componentSelection().ids();
    auto c = doc.takeReliefComponent(id_);
    doc.insertReliefComponent(target_, std::move(*c), index_);
    doc.componentSelection().assign(keep);
    return true;
}

void MoveReliefComponentToLevelCommand::revert(Document& doc) {
    const std::vector<ComponentId> keep = doc.componentSelection().ids();
    if (auto c = doc.takeReliefComponent(id_)) {
        doc.insertReliefComponent(fromLevel_, std::move(*c), fromIndex_);
    }
    doc.componentSelection().assign(keep);
}

// --- Duplicate ---------------------------------------------------------------------------------

bool DuplicateReliefComponentCommand::apply(Document& doc) {
    if (!applied_) {
        const auto where = doc.locateReliefComponent(id_);
        if (!where) {
            return false;
        }
        const ReliefComponent& src = *doc.findReliefComponent(id_);
        if (!roomForComponent(doc, cellsOf(src.data.field), 1)) {
            return false;
        }
        ReliefComponent copy = src;
        std::string name = copy.data.name + " copy";
        if (name.size() > kMaxReliefNameBytes) {
            name.resize(kMaxReliefNameBytes);
        }
        copy.data.name = std::move(name);
        copy.id = doc.allocateComponentId();
        copy_ = copy.id;
        level_ = where->level;
        index_ = where->index + 1;
        held_ = std::move(copy);
        applied_ = true;
    }
    if (!held_) {
        return false;
    }
    doc.insertReliefComponent(level_, std::move(*held_), index_);
    held_.reset();
    return true;
}

void DuplicateReliefComponentCommand::revert(Document& doc) {
    held_ = doc.takeReliefComponent(copy_);
}

}  // namespace carvetto::document
