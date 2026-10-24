#include "carvetto/document/document.hpp"

#include <algorithm>
#include <cassert>
#include <set>
#include <string>
#include <utility>

namespace carvetto::document {

namespace {

template <class T, class IdT>
auto findById(T& items, IdT id) {
    return std::find_if(items.begin(), items.end(), [id](const auto& item) { return item.id == id; });
}

}  // namespace

Document::Document() : Document(Job{}) {}

Document::Document(Job job) : job_(job) {
    activeSheet_ = insertSheet(Sheet::fromJob(job_));
    activeLayer_ = insertLayer(Layer{});
}

// --- Sheets --------------------------------------------------------------------------------------

const Sheet* Document::findSheet(SheetId id) const {
    const auto it = findById(sheets_, id);
    return it == sheets_.end() ? nullptr : &*it;
}

Sheet* Document::findSheet(SheetId id) {
    const auto it = findById(sheets_, id);
    return it == sheets_.end() ? nullptr : &*it;
}

std::optional<std::size_t> Document::sheetIndex(SheetId id) const {
    const auto it = findById(sheets_, id);
    if (it == sheets_.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(it - sheets_.begin());
}

bool Document::setActiveSheet(SheetId id) {
    if (!findSheet(id)) {
        return false;
    }
    activeSheet_ = id;
    return true;
}

SheetId Document::insertSheet(Sheet sheet, std::optional<std::size_t> index) {
    if (sheet.id.isNull()) {
        sheet.id = allocateSheetId();
    } else {
        assert(!findSheet(sheet.id) && "duplicate sheet id");
        ids_.reserveThrough(sheet.id.value);
    }
    const SheetId id = sheet.id;
    const std::size_t at = std::min(index.value_or(sheets_.size()), sheets_.size());
    sheets_.insert(sheets_.begin() + static_cast<std::ptrdiff_t>(at), std::move(sheet));
    if (activeSheet_.isNull()) {
        activeSheet_ = id;
    }
    return id;
}

std::optional<Sheet> Document::takeSheet(SheetId id) {
    const auto it = findById(sheets_, id);
    if (it == sheets_.end() || sheets_.size() == 1 || !objectsOnSheet(id).empty()) {
        return std::nullopt;
    }
    Sheet sheet = std::move(*it);
    sheets_.erase(it);
    if (activeSheet_ == id) {
        activeSheet_ = sheets_.front().id;
    }
    return sheet;
}

std::vector<ObjectId> Document::objectsOnSheet(SheetId sheet) const {
    std::vector<ObjectId> out;
    for (const auto& [id, obj] : objects_) {
        if (obj.sheet == sheet) {
            out.push_back(id);
        }
    }
    return out;
}

// --- Layers --------------------------------------------------------------------------------------

const Layer* Document::findLayer(LayerId id) const {
    const auto it = findById(layers_, id);
    return it == layers_.end() ? nullptr : &*it;
}

Layer* Document::findLayer(LayerId id) {
    const auto it = findById(layers_, id);
    return it == layers_.end() ? nullptr : &*it;
}

std::optional<std::size_t> Document::layerIndex(LayerId id) const {
    const auto it = findById(layers_, id);
    if (it == layers_.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(it - layers_.begin());
}

bool Document::setActiveLayer(LayerId id) {
    if (!findLayer(id)) {
        return false;
    }
    activeLayer_ = id;
    return true;
}

LayerId Document::insertLayer(Layer layer, std::optional<std::size_t> index) {
    if (layer.id.isNull()) {
        layer.id = allocateLayerId();
    } else {
        assert(!findLayer(layer.id) && "duplicate layer id");
        ids_.reserveThrough(layer.id.value);
    }
    const LayerId id = layer.id;
    const std::size_t at = std::min(index.value_or(layers_.size()), layers_.size());
    layers_.insert(layers_.begin() + static_cast<std::ptrdiff_t>(at), std::move(layer));
    if (activeLayer_.isNull()) {
        activeLayer_ = id;
    }
    return id;
}

std::optional<Layer> Document::takeLayer(LayerId id) {
    const auto it = findById(layers_, id);
    if (it == layers_.end() || layers_.size() == 1 || !layerIsEmpty(id)) {
        return std::nullopt;
    }
    Layer layer = std::move(*it);
    layers_.erase(it);
    if (activeLayer_ == id) {
        activeLayer_ = defaultLayer();
    }
    return layer;
}

bool Document::moveLayer(LayerId id, std::size_t newIndex) {
    const auto from = layerIndex(id);
    if (!from || newIndex >= layers_.size()) {
        return false;
    }
    if (*from == newIndex) {
        return true;
    }
    Layer layer = std::move(layers_[*from]);
    layers_.erase(layers_.begin() + static_cast<std::ptrdiff_t>(*from));
    layers_.insert(layers_.begin() + static_cast<std::ptrdiff_t>(newIndex), std::move(layer));
    return true;
}

std::vector<ObjectId> Document::objectsOnLayer(LayerId layer) const {
    std::vector<ObjectId> out;
    for (const auto& [id, obj] : objects_) {
        if (obj.layer == layer) {
            out.push_back(id);
        }
    }
    return out;
}

bool Document::layerIsEmpty(LayerId layer) const {
    return std::none_of(objects_.begin(), objects_.end(),
                        [layer](const auto& kv) { return kv.second.layer == layer; });
}

// --- Objects -------------------------------------------------------------------------------------

const Object* Document::findObject(ObjectId id) const {
    const auto it = objects_.find(id);
    return it == objects_.end() ? nullptr : &it->second;
}

Object* Document::findObject(ObjectId id) {
    const auto it = objects_.find(id);
    return it == objects_.end() ? nullptr : &it->second;
}

std::vector<ObjectId> Document::objectIds() const {
    std::vector<ObjectId> out;
    out.reserve(objects_.size());
    for (const auto& [id, obj] : objects_) {
        out.push_back(id);
    }
    return out;
}

ObjectId Document::insertObject(Object obj) {
    if (obj.id.isNull()) {
        obj.id = allocateObjectId();
    } else {
        assert(!objects_.contains(obj.id) && "duplicate object id");
        ids_.reserveThrough(obj.id.value);
    }
    assert(findLayer(obj.layer) && "object layer must exist");
    assert(findSheet(obj.sheet) && "object sheet must exist");
    const ObjectId id = obj.id;
    objects_.emplace(id, std::move(obj));
    return id;
}

std::optional<Object> Document::takeObject(ObjectId id) {
    const auto it = objects_.find(id);
    if (it == objects_.end()) {
        return std::nullopt;
    }
    Object obj = std::move(it->second);
    objects_.erase(it);
    selection_.remove(id);
    return obj;
}

std::optional<geometry::Box2> Document::objectBounds(ObjectId id) const {
    const Object* obj = findObject(id);
    if (!obj) {
        return std::nullopt;
    }
    if (const auto* v = obj->asVector()) {
        return v->bounds();
    }
    if (const auto* b = obj->asBitmap()) {
        return b->bounds();
    }
    geometry::Box2 box;
    for (ObjectId child : obj->asGroup()->children) {
        if (const auto childBox = objectBounds(child)) {
            box.expand(*childBox);
        }
    }
    return box;
}

std::vector<ObjectId> Document::withDescendants(const std::vector<ObjectId>& ids) const {
    std::set<ObjectId> all;
    std::vector<ObjectId> stack(ids.rbegin(), ids.rend());
    while (!stack.empty()) {
        const ObjectId id = stack.back();
        stack.pop_back();
        const Object* obj = findObject(id);
        if (!obj || !all.insert(id).second) {
            continue;
        }
        if (const auto* g = obj->asGroup()) {
            stack.insert(stack.end(), g->children.begin(), g->children.end());
        }
    }
    return {all.begin(), all.end()};
}

std::vector<Object> Document::copyObjects(const std::vector<ObjectId>& ids) const {
    const std::vector<ObjectId> wantedIds = withDescendants(ids);
    const std::set<ObjectId> wanted(wantedIds.begin(), wantedIds.end());
    std::vector<Object> out;
    out.reserve(wantedIds.size());
    for (ObjectId id : wantedIds) {
        Object copy = *findObject(id);
        // A copied child whose parent is not in the clipboard becomes top-level.
        if (copy.parent && !wanted.contains(*copy.parent)) {
            copy.parent.reset();
        }
        out.push_back(std::move(copy));
    }
    return out;
}

// --- Integrity -----------------------------------------------------------------------------------

namespace {

std::string idText(ObjectId id) {
    return "#" + std::to_string(id.value);
}

}  // namespace

std::vector<IntegrityIssue> Document::checkIntegrity() const {
    using Kind = IntegrityIssue::Kind;
    std::vector<IntegrityIssue> issues;
    const auto report = [&issues](Kind kind, ObjectId object, ObjectId related, std::string message) {
        issues.push_back(IntegrityIssue{kind, object, related, std::move(message)});
    };

    if (!findLayer(activeLayer_)) {
        report(Kind::BadActiveLayer, {}, {}, "active layer does not exist");
    }
    if (!findSheet(activeSheet_)) {
        report(Kind::BadActiveSheet, {}, {}, "active sheet does not exist");
    }
    for (ObjectId id : selection_.ids()) {
        if (!hasObject(id)) {
            report(Kind::StaleSelection, id, {}, "selection holds missing object " + idText(id));
        }
    }

    std::map<ObjectId, ObjectId> listedBy;  // child -> group listing it
    for (const auto& [id, obj] : objects_) {
        if (!findLayer(obj.layer)) {
            report(Kind::MissingLayer, id, {}, idText(id) + " is on a missing layer");
        }
        if (!findSheet(obj.sheet)) {
            report(Kind::MissingSheet, id, {}, idText(id) + " is on a missing sheet");
        }
        if (const auto* g = obj.asGroup()) {
            for (ObjectId child : g->children) {
                const Object* c = findObject(child);
                if (!c) {
                    report(Kind::DanglingChild, id, child,
                           "group " + idText(id) + " lists missing child " + idText(child));
                    continue;
                }
                if (const auto [it, inserted] = listedBy.emplace(child, id); !inserted) {
                    report(Kind::DuplicateChild, id, child,
                           idText(child) + " is listed by " + idText(it->second) + " and " + idText(id));
                    continue;
                }
                if (c->parent != id) {
                    report(Kind::ChildParentMismatch, id, child,
                           "group " + idText(id) + " lists " + idText(child) + " whose parent differs");
                }
                if (c->sheet != obj.sheet) {
                    report(Kind::SheetMismatch, id, child,
                           "child " + idText(child) + " is on another sheet than group " + idText(id));
                }
            }
        }
        if (obj.parent) {
            const Object* p = findObject(*obj.parent);
            if (!p || !p->isGroup()) {
                report(Kind::BadParent, id, *obj.parent,
                       idText(id) + " has missing or non-group parent " + idText(*obj.parent));
            } else {
                const auto& kids = p->asGroup()->children;
                if (std::find(kids.begin(), kids.end(), id) == kids.end()) {
                    report(Kind::ChildNotListed, id, *obj.parent,
                           idText(id) + " is not listed by its parent " + idText(*obj.parent));
                }
            }
        }
        // Cycle detection: walk up at most objectCount() links.
        std::set<ObjectId> seen{id};
        for (const Object* cur = &obj; cur && cur->parent;) {
            if (!seen.insert(*cur->parent).second) {
                report(Kind::Cycle, id, *cur->parent, "parent chain of " + idText(id) + " loops");
                break;
            }
            cur = findObject(*cur->parent);
        }
    }
    return issues;
}

}  // namespace carvetto::document
