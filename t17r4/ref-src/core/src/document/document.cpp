#include "carvetto/document/document.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

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
        if (sheet.id.isNull()) {
            return {};  // generator exhausted (asserted in allocate)
        }
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
        if (layer.id.isNull()) {
            return {};  // generator exhausted (asserted in allocate)
        }
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
        if (obj.id.isNull()) {
            return {};  // generator exhausted (asserted in allocate)
        }
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

/// For each object whose parent chain loops, the first object on the loop that the chain reaches.
std::map<ObjectId, ObjectId> findParentCycles(const std::map<ObjectId, Object>& objects) {
    // Parent links form a functional graph (at most one parent per object). Each object is walked
    // at most once: the walk from an unvisited object follows parent links, pushing objects on an
    // explicit path (no recursion, so very deep chains cannot overflow the stack), until it hits
    // no parent, a missing object, an object finished by an earlier walk, or an object already on
    // the path. In the last case the path suffix from that object is a loop; every loop object
    // enters the loop at itself. Objects before the loop, and objects whose walk reaches a finished
    // object, inherit that object's entry (if any). This matches walking each chain separately and
    // naming the first repeated object, in O(N log N) overall.
    enum class Mark : unsigned char { OnPath, Done };
    std::map<ObjectId, Mark> marks;
    std::map<ObjectId, ObjectId> entry;
    std::vector<ObjectId> path;
    for (const auto& entryPair : objects) {
        const ObjectId start = entryPair.first;
        if (marks.count(start) != 0) {
            continue;
        }
        path.clear();
        std::optional<ObjectId> inherited;
        for (ObjectId cur = start;;) {
            marks[cur] = Mark::OnPath;
            path.push_back(cur);
            const Object& obj = objects.at(cur);
            if (!obj.parent || objects.count(*obj.parent) == 0) {
                break;
            }
            const ObjectId next = *obj.parent;
            const auto mark = marks.find(next);
            if (mark == marks.end()) {
                cur = next;
                continue;
            }
            if (mark->second == Mark::OnPath) {
                // Loop: the path from `next` to the end. Those objects enter the loop at themselves.
                const auto loopBegin = std::find(path.begin(), path.end(), next);
                for (auto it = loopBegin; it != path.end(); ++it) {
                    entry[*it] = *it;
                    marks[*it] = Mark::Done;
                }
                path.erase(loopBegin, path.end());
                inherited = next;
            } else if (const auto e = entry.find(next); e != entry.end()) {
                inherited = e->second;
            }
            break;
        }
        for (ObjectId id : path) {
            marks[id] = Mark::Done;
            if (inherited) {
                entry[id] = *inherited;
            }
        }
    }
    return entry;
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

    // Ids: every id must lie in [1, kMaxId) and below the generator's next id, or a later
    // allocation could hand it out again.
    const std::uint64_t next = ids_.peek();
    const auto checkId = [&report, next](std::uint64_t value, ObjectId object, const char* what) {
        const std::string name = std::string(what) + " #" + std::to_string(value);
        if (value == 0 || value >= IdGenerator::kMaxId) {
            report(Kind::IdOutOfRange, object, {}, name + " is outside [1, 2^62)");
        }
        if (value >= next) {
            report(Kind::IdNotBelowNext, object, {},
                   name + " is not below the next id #" + std::to_string(next));
        }
    };
    for (const Sheet& sheet : sheets_) {
        checkId(sheet.id.value, {}, "sheet");
    }
    for (const Layer& layer : layers_) {
        checkId(layer.id.value, {}, "layer");
    }
    for (const auto& [id, obj] : objects_) {
        checkId(id.value, id, "object");
        if (obj.id != id) {
            report(Kind::IdKeyMismatch, id, obj.id,
                   "object stored under " + idText(id) + " has id " + idText(obj.id));
            checkId(obj.id.value, id, "object (stored id)");
        }
    }

    // One shared generator hands out every kind of id, so a value may belong to only one sheet,
    // layer or object (ADR-0007 amendment A2). Keyed by value so the report order is deterministic.
    struct IdOwners {
        std::vector<std::string> names;
        ObjectId object;  // first object owning the value, if any
    };
    std::map<std::uint64_t, IdOwners> owners;
    for (const Sheet& sheet : sheets_) {
        owners[sheet.id.value].names.push_back("sheet");
    }
    for (const Layer& layer : layers_) {
        owners[layer.id.value].names.push_back("layer");
    }
    // An object's stored id is a value it claims too (ADR-0007 amendment A3): a mismatching one
    // is counted separately, so it collides with whatever else uses that value.
    const auto addObjectOwner = [&owners](std::uint64_t value, ObjectId key, const char* name) {
        IdOwners& entry = owners[value];
        entry.names.push_back(name);
        if (entry.object.isNull()) {
            entry.object = key;
        }
    };
    for (const auto& [id, obj] : objects_) {
        addObjectOwner(id.value, id, "object");
        if (obj.id != id) {
            addObjectOwner(obj.id.value, id, "object (stored id)");
        }
    }
    for (const auto& [value, entry] : owners) {
        if (value == 0 || entry.names.size() < 2) {
            continue;  // the null id is already reported as IdOutOfRange
        }
        std::string message = "id #" + std::to_string(value) + " is used by";
        for (std::size_t i = 0; i < entry.names.size(); ++i) {
            message += (i == 0 ? " " : ", ") + entry.names[i];
        }
        report(Kind::DuplicateId, entry.object, {}, std::move(message));
    }

    // Pre-passes so the per-object loop below is O(log N) per check whatever the group depth or
    // width (T-0014): every (group, child) link, and for each object whose parent chain loops, the
    // first node on the loop that its chain reaches.
    std::set<std::pair<ObjectId, ObjectId>> listedPairs;  // (group, child)
    for (const auto& [id, obj] : objects_) {
        if (const auto* g = obj.asGroup()) {
            for (ObjectId child : g->children) {
                listedPairs.emplace(id, child);
            }
        }
    }
    const std::map<ObjectId, ObjectId> cycleEntry = findParentCycles(objects_);

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
                if (listedPairs.count({*obj.parent, id}) == 0) {
                    report(Kind::ChildNotListed, id, *obj.parent,
                           idText(id) + " is not listed by its parent " + idText(*obj.parent));
                }
            }
        }
        if (const auto it = cycleEntry.find(id); it != cycleEntry.end()) {
            report(Kind::Cycle, id, it->second, "parent chain of " + idText(id) + " loops");
        }
    }
    return issues;
}

}  // namespace carvetto::document
