#include "carvetto/document/commands.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <type_traits>
#include <variant>

#include "carvetto/document/document.hpp"

namespace carvetto::document {

// --- ObjectChangeSet -----------------------------------------------------------------------------

void ObjectChangeSet::recordBefore(const Document& doc, ObjectId id) {
    if (before_.contains(id)) {
        return;
    }
    const Object* obj = doc.findObject(id);
    before_.emplace(id, obj ? std::optional<Object>(*obj) : std::nullopt);
    order_.push_back(id);
}

void ObjectChangeSet::captureAfter(const Document& doc) {
    after_.clear();
    for (ObjectId id : order_) {
        const Object* obj = doc.findObject(id);
        after_.emplace(id, obj ? std::optional<Object>(*obj) : std::nullopt);
    }
}

void ObjectChangeSet::restore(Document& doc, ObjectId id, const std::optional<Object>& state) {
    Object* current = doc.findObject(id);
    if (!state) {
        if (current) {
            doc.takeObject(id);
        }
    } else if (current) {
        *current = *state;
    } else {
        doc.insertObject(*state);
    }
}

void ObjectChangeSet::revert(Document& doc) const {
    for (ObjectId id : order_) {
        restore(doc, id, before_.at(id));
    }
}

void ObjectChangeSet::reapply(Document& doc) const {
    for (ObjectId id : order_) {
        restore(doc, id, after_.at(id));
    }
}

bool ObjectChangeSet::changed() const {
    return std::any_of(order_.begin(), order_.end(),
                       [this](ObjectId id) { return before_.at(id) != after_.at(id); });
}

void ObjectChangeSet::clear() {
    order_.clear();
    before_.clear();
    after_.clear();
}

namespace {

// True when `value` is the id of any sheet, layer or object. The three kinds share one generator,
// so a value names at most one item of any kind (ADR-0007, amendment A2).
bool idInUse(const Document& doc, std::uint64_t value) {
    return doc.findSheet(SheetId{value}) != nullptr || doc.findLayer(LayerId{value}) != nullptr ||
           doc.hasObject(ObjectId{value});
}

// Id-allocation guard shared by every command that creates something (ADR-0007, amendments A1
// and A2). A null id needs one free id in the generator. A preset id must be fresh: below the
// ceiling, not below the generator's next id (so it was never handed out, and an id freed by
// undoing an add cannot be taken again), and not used by any sheet, layer or object. When this
// fails the command's first `apply` returns false, so nothing changes and nothing is recorded.
// Redo must not call it: the command's own id is below the next id by then.
bool canTakeId(const Document& doc, std::uint64_t presetId) {
    if (presetId == 0) {
        return doc.idGenerator().canAllocate(1);
    }
    return presetId < IdGenerator::kMaxId && presetId >= doc.idGenerator().peek() && !idInUse(doc, presetId);
}

void transformData(ObjectData& data, const geometry::Transform2& m) {
    std::visit(
        [&m](auto& payload) {
            using T = std::decay_t<decltype(payload)>;
            if constexpr (!std::is_same_v<T, GroupObject>) {
                payload.transform(m);
            }
        },
        data);
}

// --- Group maintenance shared by the structural commands (rules in commands.hpp / ADR-0007) ----

// Removes `id` from its parent's child list and clears its parent link. Returns the former parent.
std::optional<ObjectId> detachFromParent(Document& doc, ObjectChangeSet& cs, ObjectId id) {
    Object* obj = doc.findObject(id);
    if (!obj || !obj->parent) {
        return std::nullopt;
    }
    const ObjectId parentId = *obj->parent;
    cs.recordBefore(doc, id);
    obj->parent.reset();
    if (Object* parent = doc.findObject(parentId)) {
        if (auto* g = parent->asGroup()) {
            cs.recordBefore(doc, parentId);
            std::erase(g->children, id);
        }
    }
    return parentId;
}

// Removes `groupId` and then each ancestor while they are groups with no children left.
void pruneEmptyGroups(Document& doc, ObjectChangeSet& cs, std::optional<ObjectId> groupId) {
    while (groupId) {
        const Object* obj = doc.findObject(*groupId);
        if (!obj || !obj->isGroup() || !obj->asGroup()->children.empty()) {
            return;
        }
        const ObjectId current = *groupId;
        groupId = detachFromParent(doc, cs, current);
        cs.recordBefore(doc, current);
        doc.takeObject(current);
    }
}

// Removes an object and all its descendants, detaching it from its parent group first.
void removeTree(Document& doc, ObjectChangeSet& cs, ObjectId id) {
    if (!doc.hasObject(id)) {
        return;
    }
    const std::optional<ObjectId> parent = detachFromParent(doc, cs, id);
    for (ObjectId d : doc.withDescendants({id})) {
        cs.recordBefore(doc, d);
        doc.takeObject(d);
    }
    pruneEmptyGroups(doc, cs, parent);
}

// Removes a group but keeps its children: they take the group's place in its parent's child list
// (or become top-level), optionally moving to `moveTo`.
void dissolveGroup(Document& doc, ObjectChangeSet& cs, ObjectId groupId, std::optional<LayerId> moveTo) {
    const Object* found = doc.findObject(groupId);
    if (!found || !found->isGroup()) {
        return;
    }
    const Object group = *found;
    const std::vector<ObjectId>& children = group.asGroup()->children;
    for (ObjectId child : children) {
        if (Object* c = doc.findObject(child)) {
            cs.recordBefore(doc, child);
            c->parent = group.parent;
            if (moveTo) {
                c->layer = *moveTo;
            }
        }
    }
    if (group.parent) {
        if (Object* parent = doc.findObject(*group.parent)) {
            if (auto* g = parent->asGroup()) {
                cs.recordBefore(doc, *group.parent);
                std::vector<ObjectId> spliced;
                spliced.reserve(g->children.size() + children.size());
                for (ObjectId sibling : g->children) {
                    if (sibling == groupId) {
                        spliced.insert(spliced.end(), children.begin(), children.end());
                    } else {
                        spliced.push_back(sibling);
                    }
                }
                g->children = std::move(spliced);
            }
        }
    }
    cs.recordBefore(doc, groupId);
    doc.takeObject(groupId);
    if (children.empty()) {
        pruneEmptyGroups(doc, cs, group.parent);
    }
}

// Deletes every object matching `pred`. Matching groups are dissolved first, so their children
// that do not match survive in the group's place; the remaining matches are then removed (which
// detaches them from non-matching groups and prunes groups left empty).
template <class Pred>
void removeMatching(Document& doc, ObjectChangeSet& cs, Pred pred) {
    const auto matching = [&doc, &pred](bool groups) {
        std::vector<ObjectId> out;
        for (const auto& [id, obj] : doc.objects()) {
            if (obj.isGroup() == groups && pred(obj)) {
                out.push_back(id);
            }
        }
        return out;
    };
    for (ObjectId id : matching(true)) {
        dissolveGroup(doc, cs, id, std::nullopt);
    }
    for (ObjectId id : matching(false)) {
        removeTree(doc, cs, id);
    }
}

}  // namespace

// --- AddObjectCommand ----------------------------------------------------------------------------

bool AddObjectCommand::apply(Document& doc) {
    if (!changes_.empty()) {
        changes_.reapply(doc);
        return true;
    }
    if (!canTakeId(doc, object_.id.value)) {
        return false;
    }
    if (object_.id.isNull()) {
        object_.id = doc.allocateObjectId();
    }
    Object* parent = object_.parent ? doc.findObject(*object_.parent) : nullptr;
    if (parent && parent->isGroup()) {
        changes_.recordBefore(doc, parent->id);
        parent->asGroup()->children.push_back(object_.id);
        object_.sheet = parent->sheet;
    } else {
        object_.parent.reset();
    }
    changes_.recordBefore(doc, object_.id);
    doc.insertObject(object_);
    changes_.captureAfter(doc);
    return true;
}

void AddObjectCommand::revert(Document& doc) {
    changes_.revert(doc);
}

// --- RemoveObjectsCommand ------------------------------------------------------------------------

bool RemoveObjectsCommand::apply(Document& doc) {
    if (!changes_.empty()) {
        changes_.reapply(doc);
        return true;
    }
    for (ObjectId id : ids_) {
        removeTree(doc, changes_, id);
    }
    changes_.captureAfter(doc);
    return !changes_.empty();
}

void RemoveObjectsCommand::revert(Document& doc) {
    changes_.revert(doc);
}

// --- MoveObjectsToLayerCommand -------------------------------------------------------------------

bool MoveObjectsToLayerCommand::apply(Document& doc) {
    if (!changes_.empty()) {
        changes_.reapply(doc);
        return true;
    }
    if (!doc.findLayer(target_)) {
        return false;
    }
    for (ObjectId id : doc.withDescendants(ids_)) {
        Object* obj = doc.findObject(id);
        if (obj->layer != target_) {
            changes_.recordBefore(doc, id);
            obj->layer = target_;
        }
    }
    changes_.captureAfter(doc);
    return !changes_.empty();
}

void MoveObjectsToLayerCommand::revert(Document& doc) {
    changes_.revert(doc);
}

// --- MoveObjectsToSheetCommand -------------------------------------------------------------------

bool MoveObjectsToSheetCommand::apply(Document& doc) {
    if (!changes_.empty()) {
        changes_.reapply(doc);
        return true;
    }
    if (!doc.findSheet(target_)) {
        return false;
    }
    const std::vector<ObjectId> moving = doc.withDescendants(ids_);
    const std::set<ObjectId> movingSet(moving.begin(), moving.end());
    for (ObjectId id : moving) {
        const Object* obj = doc.findObject(id);
        if (obj && obj->sheet != target_ && obj->parent && !movingSet.contains(*obj->parent)) {
            pruneEmptyGroups(doc, changes_, detachFromParent(doc, changes_, id));
        }
    }
    for (ObjectId id : moving) {
        Object* obj = doc.findObject(id);
        if (obj && obj->sheet != target_) {
            changes_.recordBefore(doc, id);
            obj->sheet = target_;
        }
    }
    changes_.captureAfter(doc);
    return !changes_.empty();
}

void MoveObjectsToSheetCommand::revert(Document& doc) {
    changes_.revert(doc);
}

// --- TransformObjectsCommand ---------------------------------------------------------------------

bool TransformObjectsCommand::apply(Document& doc) {
    if (!after_.empty()) {
        // Redo: restore the recorded result rather than re-transforming.
        for (const auto& [id, data] : after_) {
            if (Object* obj = doc.findObject(id)) {
                obj->data = data;
            }
        }
        return true;
    }
    before_.clear();
    for (ObjectId id : doc.withDescendants(ids_)) {
        Object* obj = doc.findObject(id);
        if (!obj || obj->isGroup()) {
            continue;
        }
        before_.emplace_back(id, obj->data);
        transformData(obj->data, transform_);
        after_.emplace_back(id, obj->data);
    }
    return !before_.empty();
}

void TransformObjectsCommand::revert(Document& doc) {
    for (const auto& [id, data] : before_) {
        if (Object* obj = doc.findObject(id)) {
            obj->data = data;
        }
    }
}

bool TransformObjectsCommand::mergeWith(const Command& next) {
    const auto* other = dynamic_cast<const TransformObjectsCommand*>(&next);
    if (!other || other->ids_ != ids_) {
        return false;
    }
    after_ = other->after_;
    transform_ = geometry::compose(other->transform_, transform_);
    return true;
}

// --- PasteObjectsCommand -------------------------------------------------------------------------

bool PasteObjectsCommand::apply(Document& doc) {
    if (pasted_.empty()) {
        if (clipboard_.empty() || !doc.idGenerator().canAllocate(clipboard_.size())) {
            return false;  // all-or-nothing: never paste part of the clipboard
        }
        std::map<ObjectId, ObjectId> remap;
        for (const Object& src : clipboard_) {
            remap[src.id] = doc.allocateObjectId();
        }
        const geometry::Transform2 shift = geometry::Transform2::translation(offset_);
        for (const Object& src : clipboard_) {
            Object copy = src;
            copy.id = remap.at(src.id);
            if (copy.parent) {
                const auto it = remap.find(*copy.parent);
                if (it == remap.end()) {
                    copy.parent.reset();
                } else {
                    copy.parent = it->second;
                }
            }
            if (auto* g = copy.asGroup()) {
                std::vector<ObjectId> children;
                for (ObjectId child : g->children) {
                    if (const auto it = remap.find(child); it != remap.end()) {
                        children.push_back(it->second);
                    }
                }
                g->children = std::move(children);
            }
            if (layer_ || !doc.findLayer(copy.layer)) {
                copy.layer = layer_ ? layer_ : doc.activeLayer();
            }
            if (sheet_ || !doc.findSheet(copy.sheet)) {
                copy.sheet = sheet_ ? sheet_ : doc.activeSheet();
            }
            if (offset_ != geometry::Vec2{}) {
                transformData(copy.data, shift);
            }
            pastedIds_.push_back(copy.id);
            pasted_.push_back(std::move(copy));
        }
    }
    for (const Object& obj : pasted_) {
        doc.insertObject(obj);
    }
    return !pasted_.empty();
}

void PasteObjectsCommand::revert(Document& doc) {
    for (const Object& obj : pasted_) {
        doc.takeObject(obj.id);
    }
}

// --- GroupObjectsCommand -------------------------------------------------------------------------

bool GroupObjectsCommand::apply(Document& doc) {
    if (!changes_.empty()) {
        changes_.reapply(doc);
        return true;
    }
    std::vector<ObjectId> members;
    for (ObjectId id : ids_) {
        const Object* obj = doc.findObject(id);
        if (!obj || obj->parent || std::find(members.begin(), members.end(), id) != members.end()) {
            continue;
        }
        if (!members.empty() && obj->sheet != doc.findObject(members.front())->sheet) {
            continue;
        }
        members.push_back(id);
    }
    if (members.empty() || !canTakeId(doc, 0)) {
        return false;
    }
    const Object& first = *doc.findObject(members.front());
    Object group;
    group.id = doc.allocateObjectId();
    group.layer = first.layer;
    group.sheet = first.sheet;
    group.name = "Group";
    group.data = GroupObject{members};
    groupId_ = group.id;
    changes_.recordBefore(doc, groupId_);
    doc.insertObject(std::move(group));
    for (ObjectId id : members) {
        changes_.recordBefore(doc, id);
        doc.findObject(id)->parent = groupId_;
    }
    changes_.captureAfter(doc);
    return true;
}

void GroupObjectsCommand::revert(Document& doc) {
    changes_.revert(doc);
}

// --- UngroupCommand ------------------------------------------------------------------------------

bool UngroupCommand::apply(Document& doc) {
    if (!changes_.empty()) {
        changes_.reapply(doc);
        return true;
    }
    const Object* obj = doc.findObject(groupId_);
    if (!obj || !obj->isGroup()) {
        return false;
    }
    const std::optional<LayerId> moveTo = toGroupLayer_ ? std::optional<LayerId>(obj->layer) : std::nullopt;
    dissolveGroup(doc, changes_, groupId_, moveTo);
    changes_.captureAfter(doc);
    return true;
}

void UngroupCommand::revert(Document& doc) {
    changes_.revert(doc);
}

// --- AddLayerCommand -----------------------------------------------------------------------------

bool AddLayerCommand::apply(Document& doc) {
    if (!applied_) {
        if (!canTakeId(doc, layer_.id.value)) {
            return false;
        }
        if (layer_.id.isNull()) {
            layer_.id = doc.allocateLayerId();
        }
        applied_ = true;
    }
    previousActive_ = doc.activeLayer();
    doc.insertLayer(layer_, index_);
    if (makeActive_) {
        doc.setActiveLayer(layer_.id);
    }
    return true;
}

void AddLayerCommand::revert(Document& doc) {
    doc.takeLayer(layer_.id);
    doc.setActiveLayer(previousActive_);
}

// --- DeleteLayerCommand --------------------------------------------------------------------------

bool DeleteLayerCommand::apply(Document& doc) {
    if (!layer_.id.isNull()) {
        // Redo: replay the recorded object changes, then drop the (now empty) layer.
        changes_.reapply(doc);
        doc.takeLayer(id_);
        return true;
    }
    const Layer* layer = doc.findLayer(id_);
    if (!layer || doc.layers().size() == 1) {
        return false;
    }
    layer_ = *layer;
    index_ = *doc.layerIndex(id_);
    previousActive_ = doc.activeLayer();
    if (deleteContents_) {
        removeMatching(doc, changes_, [this](const Object& obj) { return obj.layer == id_; });
    } else {
        const LayerId target = (index_ == 0) ? doc.layers()[1].id : doc.defaultLayer();
        for (ObjectId id : doc.objectsOnLayer(id_)) {
            changes_.recordBefore(doc, id);
            doc.findObject(id)->layer = target;
        }
    }
    changes_.captureAfter(doc);
    doc.takeLayer(id_);
    return true;
}

void DeleteLayerCommand::revert(Document& doc) {
    doc.insertLayer(layer_, index_);
    changes_.revert(doc);
    doc.setActiveLayer(previousActive_);
}

// --- SetLayerPropertiesCommand -------------------------------------------------------------------

bool SetLayerPropertiesCommand::apply(Document& doc) {
    Layer* layer = doc.findLayer(after_.id);
    if (!layer || *layer == after_) {
        return false;
    }
    before_ = *layer;
    *layer = after_;
    return true;
}

void SetLayerPropertiesCommand::revert(Document& doc) {
    if (Layer* layer = doc.findLayer(after_.id)) {
        *layer = before_;
    }
}

bool SetLayerPropertiesCommand::mergeWith(const Command& next) {
    const auto* other = dynamic_cast<const SetLayerPropertiesCommand*>(&next);
    if (!other || other->after_.id != after_.id) {
        return false;
    }
    after_ = other->after_;
    return true;
}

// --- MoveLayerCommand ----------------------------------------------------------------------------

bool MoveLayerCommand::apply(Document& doc) {
    const auto index = doc.layerIndex(id_);
    if (!index || *index == newIndex_ || !doc.moveLayer(id_, newIndex_)) {
        return false;
    }
    oldIndex_ = *index;
    return true;
}

void MoveLayerCommand::revert(Document& doc) {
    doc.moveLayer(id_, oldIndex_);
}

// --- AddSheetCommand -----------------------------------------------------------------------------

bool AddSheetCommand::apply(Document& doc) {
    if (!applied_) {
        if (!canTakeId(doc, sheet_.id.value)) {
            return false;
        }
        if (sheet_.id.isNull()) {
            sheet_.id = doc.allocateSheetId();
        }
        applied_ = true;
    }
    previousActive_ = doc.activeSheet();
    doc.insertSheet(sheet_);
    if (makeActive_) {
        doc.setActiveSheet(sheet_.id);
    }
    return true;
}

void AddSheetCommand::revert(Document& doc) {
    doc.takeSheet(sheet_.id);
    doc.setActiveSheet(previousActive_);
}

// --- DeleteSheetCommand --------------------------------------------------------------------------

bool DeleteSheetCommand::apply(Document& doc) {
    if (!sheet_.id.isNull()) {
        changes_.reapply(doc);
        doc.takeSheet(id_);
        return true;
    }
    const Sheet* sheet = doc.findSheet(id_);
    if (!sheet || doc.sheets().size() == 1) {
        return false;
    }
    sheet_ = *sheet;
    index_ = *doc.sheetIndex(id_);
    previousActive_ = doc.activeSheet();
    // Pending owner decision: the sheet's objects are deleted with it. Groups never span sheets,
    // so this removes whole trees; removeMatching keeps that true even for inconsistent input.
    removeMatching(doc, changes_, [this](const Object& obj) { return obj.sheet == id_; });
    changes_.captureAfter(doc);
    doc.takeSheet(id_);
    return true;
}

void DeleteSheetCommand::revert(Document& doc) {
    doc.insertSheet(sheet_, index_);
    changes_.revert(doc);
    doc.setActiveSheet(previousActive_);
}

// --- SetSheetPropertiesCommand -------------------------------------------------------------------

bool SetSheetPropertiesCommand::apply(Document& doc) {
    Sheet* sheet = doc.findSheet(after_.id);
    if (!sheet || *sheet == after_) {
        return false;
    }
    before_ = *sheet;
    *sheet = after_;
    return true;
}

void SetSheetPropertiesCommand::revert(Document& doc) {
    if (Sheet* sheet = doc.findSheet(after_.id)) {
        *sheet = before_;
    }
}

bool SetSheetPropertiesCommand::mergeWith(const Command& next) {
    const auto* other = dynamic_cast<const SetSheetPropertiesCommand*>(&next);
    if (!other || other->after_.id != after_.id) {
        return false;
    }
    after_ = other->after_;
    return true;
}

// --- SetJobCommand -------------------------------------------------------------------------------

bool SetJobCommand::apply(Document& doc) {
    if (doc.job() == after_) {
        return false;
    }
    before_ = doc.job();
    doc.setJob(after_);
    return true;
}

void SetJobCommand::revert(Document& doc) {
    doc.setJob(before_);
}

bool SetJobCommand::mergeWith(const Command& next) {
    const auto* other = dynamic_cast<const SetJobCommand*>(&next);
    if (!other) {
        return false;
    }
    after_ = other->after_;
    return true;
}

// --- SetDocumentVariableCommand ------------------------------------------------------------------

bool SetDocumentVariableCommand::apply(Document& doc) {
    if (!DocumentVariables::isValidName(name_)) {
        return false;
    }
    previous_ = doc.variables().get(name_);
    if (previous_ && *previous_ == value_) {
        return false;
    }
    return doc.variables().set(name_, value_);
}

void SetDocumentVariableCommand::revert(Document& doc) {
    if (previous_) {
        doc.variables().set(name_, *previous_);
    } else {
        doc.variables().forceRemove(name_);
    }
}

bool SetDocumentVariableCommand::mergeWith(const Command& next) {
    const auto* other = dynamic_cast<const SetDocumentVariableCommand*>(&next);
    if (!other || other->name_ != name_) {
        return false;
    }
    value_ = other->value_;
    return true;
}

// --- RemoveDocumentVariableCommand ---------------------------------------------------------------

bool RemoveDocumentVariableCommand::apply(Document& doc) {
    const auto value = doc.variables().get(name_);
    if (!value || !doc.variables().remove(name_)) {
        return false;
    }
    previous_ = *value;
    return true;
}

void RemoveDocumentVariableCommand::revert(Document& doc) {
    doc.variables().set(name_, previous_);
}

}  // namespace carvetto::document
