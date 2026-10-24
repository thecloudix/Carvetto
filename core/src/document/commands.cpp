#include "carvetto/document/commands.hpp"

#include <algorithm>
#include <functional>
#include <set>
#include <type_traits>
#include <variant>

#include "carvetto/document/document.hpp"

namespace carvetto::document {

namespace {

// Ids plus every descendant of groups among them, in id order, skipping unknown ids.
std::vector<ObjectId> withDescendants(const Document& doc, const std::vector<ObjectId>& ids) {
    std::set<ObjectId> all;
    std::function<void(ObjectId)> collect = [&](ObjectId id) {
        const Object* obj = doc.findObject(id);
        if (!obj || !all.insert(id).second) {
            return;
        }
        if (const auto* g = obj->asGroup()) {
            for (ObjectId child : g->children) {
                collect(child);
            }
        }
    };
    for (ObjectId id : ids) {
        collect(id);
    }
    return {all.begin(), all.end()};
}

void transformData(ObjectData& data, const geometry::Transform2& m) {
    std::visit(
        [&](auto& payload) {
            using T = std::decay_t<decltype(payload)>;
            if constexpr (!std::is_same_v<T, GroupObject>) {
                payload.transform(m);
            }
        },
        data);
}

}  // namespace

// --- AddObjectCommand ----------------------------------------------------------------------------

bool AddObjectCommand::apply(Document& doc) {
    if (object_.id.isNull()) {
        object_.id = doc.allocateObjectId();
    }
    doc.insertObject(object_);
    return true;
}

void AddObjectCommand::revert(Document& doc) {
    doc.takeObject(object_.id);
}

// --- RemoveObjectsCommand ------------------------------------------------------------------------

bool RemoveObjectsCommand::apply(Document& doc) {
    removed_.clear();
    detachedFromGroups_.clear();
    const std::vector<ObjectId> all = withDescendants(doc, ids_);
    const std::set<ObjectId> removing(all.begin(), all.end());
    for (ObjectId id : all) {
        const Object* obj = doc.findObject(id);
        if (obj->parent && !removing.contains(*obj->parent)) {
            if (Object* parent = doc.findObject(*obj->parent)) {
                if (auto* g = parent->asGroup()) {
                    detachedFromGroups_.emplace_back(parent->id, g->children);
                    std::erase(g->children, id);
                }
            }
        }
        removed_.push_back(*doc.takeObject(id));
    }
    return !removed_.empty();
}

void RemoveObjectsCommand::revert(Document& doc) {
    for (const Object& obj : removed_) {
        doc.insertObject(obj);
    }
    // Restore child lists in reverse so a parent touched twice ends with its original list.
    for (auto it = detachedFromGroups_.rbegin(); it != detachedFromGroups_.rend(); ++it) {
        if (Object* parent = doc.findObject(it->first)) {
            if (auto* g = parent->asGroup()) {
                g->children = it->second;
            }
        }
    }
}

// --- MoveObjectsToLayerCommand -------------------------------------------------------------------

bool MoveObjectsToLayerCommand::apply(Document& doc) {
    previous_.clear();
    if (!doc.findLayer(target_)) {
        return false;
    }
    for (ObjectId id : ids_) {
        Object* obj = doc.findObject(id);
        if (obj && obj->layer != target_) {
            previous_.emplace_back(id, obj->layer);
            obj->layer = target_;
        }
    }
    return !previous_.empty();
}

void MoveObjectsToLayerCommand::revert(Document& doc) {
    for (const auto& [id, layer] : previous_) {
        if (Object* obj = doc.findObject(id)) {
            obj->layer = layer;
        }
    }
}

// --- MoveObjectsToSheetCommand -------------------------------------------------------------------

bool MoveObjectsToSheetCommand::apply(Document& doc) {
    previous_.clear();
    if (!doc.findSheet(target_)) {
        return false;
    }
    for (ObjectId id : withDescendants(doc, ids_)) {
        Object* obj = doc.findObject(id);
        if (obj && obj->sheet != target_) {
            previous_.emplace_back(id, obj->sheet);
            obj->sheet = target_;
        }
    }
    return !previous_.empty();
}

void MoveObjectsToSheetCommand::revert(Document& doc) {
    for (const auto& [id, sheet] : previous_) {
        if (Object* obj = doc.findObject(id)) {
            obj->sheet = sheet;
        }
    }
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
    for (ObjectId id : withDescendants(doc, ids_)) {
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
    if (group_.id.isNull()) {
        std::vector<ObjectId> members;
        for (ObjectId id : ids_) {
            const Object* obj = doc.findObject(id);
            if (obj && !obj->parent && std::find(members.begin(), members.end(), id) == members.end()) {
                members.push_back(id);
            }
        }
        if (members.empty()) {
            return false;
        }
        const Object& first = *doc.findObject(members.front());
        group_.id = doc.allocateObjectId();
        group_.layer = first.layer;
        group_.sheet = first.sheet;
        group_.name = "Group";
        group_.data = GroupObject{std::move(members)};
    }
    doc.insertObject(group_);
    for (ObjectId id : group_.asGroup()->children) {
        if (Object* obj = doc.findObject(id)) {
            obj->parent = group_.id;
        }
    }
    return true;
}

void GroupObjectsCommand::revert(Document& doc) {
    for (ObjectId id : group_.asGroup()->children) {
        if (Object* obj = doc.findObject(id)) {
            obj->parent.reset();
        }
    }
    doc.takeObject(group_.id);
}

// --- UngroupCommand ------------------------------------------------------------------------------

bool UngroupCommand::apply(Document& doc) {
    const Object* obj = doc.findObject(groupId_);
    if (!obj || !obj->isGroup()) {
        return false;
    }
    group_ = *obj;
    previousLayers_.clear();
    for (ObjectId child : group_.asGroup()->children) {
        if (Object* c = doc.findObject(child)) {
            c->parent = group_.parent;
            if (toGroupLayer_) {
                previousLayers_.emplace_back(child, c->layer);
                c->layer = group_.layer;
            }
        }
    }
    doc.takeObject(groupId_);
    return true;
}

void UngroupCommand::revert(Document& doc) {
    doc.insertObject(group_);
    for (ObjectId child : group_.asGroup()->children) {
        if (Object* c = doc.findObject(child)) {
            c->parent = groupId_;
        }
    }
    for (const auto& [id, layer] : previousLayers_) {
        if (Object* c = doc.findObject(id)) {
            c->layer = layer;
        }
    }
}

// --- AddLayerCommand -----------------------------------------------------------------------------

bool AddLayerCommand::apply(Document& doc) {
    if (layer_.id.isNull()) {
        layer_.id = doc.allocateLayerId();
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
    const Layer* layer = doc.findLayer(id_);
    if (!layer || doc.layers().size() == 1) {
        return false;
    }
    layer_ = *layer;
    index_ = *doc.layerIndex(id_);
    previousActive_ = doc.activeLayer();
    removed_.clear();
    moved_.clear();
    const std::vector<ObjectId> objects = doc.objectsOnLayer(id_);
    if (deleteContents_) {
        for (ObjectId id : objects) {
            removed_.push_back(*doc.takeObject(id));
        }
    } else {
        const LayerId target = (index_ == 0) ? doc.layers()[1].id : doc.defaultLayer();
        for (ObjectId id : objects) {
            doc.findObject(id)->layer = target;
            moved_.push_back(id);
        }
    }
    doc.takeLayer(id_);
    return true;
}

void DeleteLayerCommand::revert(Document& doc) {
    doc.insertLayer(layer_, index_);
    for (ObjectId id : moved_) {
        if (Object* obj = doc.findObject(id)) {
            obj->layer = id_;
        }
    }
    for (const Object& obj : removed_) {
        doc.insertObject(obj);
    }
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
    if (sheet_.id.isNull()) {
        sheet_.id = doc.allocateSheetId();
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
    const Sheet* sheet = doc.findSheet(id_);
    if (!sheet || doc.sheets().size() == 1) {
        return false;
    }
    sheet_ = *sheet;
    index_ = *doc.sheetIndex(id_);
    previousActive_ = doc.activeSheet();
    removed_.clear();
    for (ObjectId id : doc.objectsOnSheet(id_)) {
        removed_.push_back(*doc.takeObject(id));
    }
    doc.takeSheet(id_);
    return true;
}

void DeleteSheetCommand::revert(Document& doc) {
    doc.insertSheet(sheet_, index_);
    for (const Object& obj : removed_) {
        doc.insertObject(obj);
    }
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
