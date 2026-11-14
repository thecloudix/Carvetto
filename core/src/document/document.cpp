#include "carvetto/document/document.hpp"
#include "carvetto/document/dimension.hpp"

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
    if (it == sheets_.end() || sheets_.size() == 1 || !objectsOnSheet(id).empty() ||
        !reliefLevelsOnSheet(id).empty()) {
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
    if (const auto* d = obj->asDimension()) {
        const DimensionGeometry g = dimensionGeometry(*this, *d);
        geometry::Box2 box;
        if (g.valid) {
            for (const geometry::Contour& c : g.contours) {
                box.expand(c.bounds());
            }
            box.expand(g.textCentre);
        } else {
            const auto pts = resolveDimensionPoints(*this, *d);
            for (std::size_t i = 0; i < dimensionPointCount(d->kind); ++i) {
                box.expand(pts[i]);
            }
        }
        return box;
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
        // Copy-construct straight into the vector and patch the element in place. A local
        // `Object copy` that is then moved into `out` makes GCC 15 -O3 inline the variant's move
        // and destructor dispatch and report the inactive alternatives' members as
        // -Wmaybe-uninitialized (a false positive, T-0144); this form has no such temporary.
        Object& copy = out.emplace_back(*findObject(id));
        // A copied child whose parent is not in the clipboard becomes top-level.
        if (copy.parent && !wanted.contains(*copy.parent)) {
            copy.parent.reset();
        }
    }
    return out;
}

// --- Toolpaths -----------------------------------------------------------------------------------

const ToolpathRecord* Document::findToolpath(ToolpathId id) const {
    const auto it = findById(toolpaths_, id);
    return it == toolpaths_.end() ? nullptr : &*it;
}

ToolpathRecord* Document::findToolpath(ToolpathId id) {
    const auto it = findById(toolpaths_, id);
    return it == toolpaths_.end() ? nullptr : &*it;
}

std::optional<std::size_t> Document::toolpathIndex(ToolpathId id) const {
    const auto it = findById(toolpaths_, id);
    if (it == toolpaths_.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(it - toolpaths_.begin());
}

ToolpathId Document::insertToolpath(ToolpathRecord record, std::optional<std::size_t> index) {
    if (record.id.isNull()) {
        record.id = allocateToolpathId();
        if (record.id.isNull()) {
            return {};  // generator exhausted (asserted in allocate)
        }
    } else {
        assert(!findToolpath(record.id) && "duplicate toolpath id");
        ids_.reserveThrough(record.id.value);
    }
    const ToolpathId id = record.id;
    const std::size_t at = std::min(index.value_or(toolpaths_.size()), toolpaths_.size());
    toolpaths_.insert(toolpaths_.begin() + static_cast<std::ptrdiff_t>(at), std::move(record));
    return id;
}

std::optional<ToolpathRecord> Document::takeToolpath(ToolpathId id) {
    const auto it = findById(toolpaths_, id);
    if (it == toolpaths_.end()) {
        return std::nullopt;
    }
    ToolpathRecord record = std::move(*it);
    toolpaths_.erase(it);
    return record;
}

bool Document::moveToolpath(ToolpathId id, std::size_t newIndex) {
    const auto from = toolpathIndex(id);
    if (!from || newIndex >= toolpaths_.size()) {
        return false;
    }
    if (*from == newIndex) {
        return true;
    }
    ToolpathRecord record = std::move(toolpaths_[*from]);
    toolpaths_.erase(toolpaths_.begin() + static_cast<std::ptrdiff_t>(*from));
    toolpaths_.insert(toolpaths_.begin() + static_cast<std::ptrdiff_t>(newIndex), std::move(record));
    return true;
}

std::vector<ObjectId> Document::selectedVectors(const VectorSelection& selection) const {
    std::vector<ObjectId> out;
    if (selection.all) {
        for (const auto& [id, obj] : objects_) {
            if (obj.isVector()) {
                out.push_back(id);
            }
        }
        return out;
    }
    std::set<ObjectId> taken;
    const auto take = [&](ObjectId id) {
        const Object* obj = findObject(id);
        if (obj && obj->isVector() && taken.insert(id).second) {
            out.push_back(id);
        }
    };
    for (const ObjectId id : selection.objects) {
        for (const ObjectId member : withDescendants({id})) {  // id order; just `id` for a vector
            take(member);
        }
    }
    if (!selection.layers.empty()) {
        const std::set<LayerId> layers(selection.layers.begin(), selection.layers.end());
        for (const auto& [id, obj] : objects_) {
            if (layers.contains(obj.layer)) {
                take(id);
            }
        }
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
    for (const Sheet& sheet : sheets_) {
        if (sheet.partner.value == 0) {
            continue;
        }
        const Sheet* other = findSheet(sheet.partner);
        if (!other || other->partner != sheet.id || other->side == sheet.side) {
            report(Kind::BadSheetPartner, {}, {},
                   "sheet " + std::to_string(sheet.id.value) + " is not paired with a sheet of the other side");
        }
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
    for (const ToolpathRecord& tp : toolpaths_) {
        checkId(tp.id.value, {}, "toolpath");
    }
    for (const ReliefLevel& level : reliefLevels_) {
        checkId(level.id.value, {}, "relief level");
        if (!findSheet(level.sheet)) {
            report(Kind::MissingReliefSheet, {}, {},
                   "relief level #" + std::to_string(level.id.value) + " is on a missing sheet");
        }
        if (const std::string problem = reliefEffectsProblem(level.effects); !problem.empty()) {
            report(Kind::BadReliefEffects, {}, {},
                   "relief level #" + std::to_string(level.id.value) + ": " + problem);
        }
        for (const ReliefComponent& c : level.components) {
            checkId(c.id.value, {}, "relief component");
            if (const std::string problem = reliefComponentProblem(c.data); !problem.empty()) {
                report(Kind::BadReliefComponent, {}, {},
                       "relief component #" + std::to_string(c.id.value) + ": " + problem);
            }
        }
    }
    for (const ComponentId id : componentSelection_.ids()) {
        if (!findReliefComponent(id)) {
            report(Kind::StaleComponentSelection, {}, {},
                   "component selection holds missing component #" + std::to_string(id.value));
        }
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
    for (const ToolpathRecord& tp : toolpaths_) {
        owners[tp.id.value].names.push_back("toolpath");
    }
    for (const ReliefLevel& level : reliefLevels_) {
        owners[level.id.value].names.push_back("relief level");
        for (const ReliefComponent& c : level.components) {
            owners[c.id.value].names.push_back("relief component");
        }
    }
    for (const auto& [id, obj] : objects_) {
        if (const VectorObject* v = obj.asVector()) {
            if (const auto* t = std::get_if<TextParams>(&v->params); t && !fonts_.contains(t->fontKey)) {
                report(Kind::MissingFont, id, {}, "text object " + idText(id) + " names a font that is not embedded");
            }
        }
    }
    for (const auto& [id, obj] : objects_) {
        if (const BitmapObject* b = obj.asBitmap(); b && isFontKey(b->sourceRef) && !bitmaps_.contains(b->sourceRef)) {
            report(Kind::MissingBitmap, id, {}, "bitmap object " + idText(id) + " names an image that is not embedded");
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

    // Toolpath selections (ADR-0021): a reference may name a deleted item (ids are never reused,
    // so it can never come to name another one), but never an id that was not handed out yet or
    // an item of another kind, and never the same id twice. At most one issue per toolpath, so a
    // hostile file cannot make the report itself huge.
    for (const ToolpathRecord& tp : toolpaths_) {
        const std::string who = "toolpath #" + std::to_string(tp.id.value);
        const VectorSelection& sel = tp.selection;
        const auto problem = [&]() -> std::optional<std::string> {
            if (sel.all && (!sel.layers.empty() || !sel.objects.empty())) {
                return who + " selects all vectors and also lists layers or objects";
            }
            const auto checkRef = [&](std::uint64_t value, bool isLayer) -> std::optional<std::string> {
                const std::string what = std::string(isLayer ? "layer" : "object") + " #" + std::to_string(value);
                if (value == 0 || value >= IdGenerator::kMaxId || value >= next) {
                    return who + " selects " + what + ", which was never allocated";
                }
                const auto it = owners.find(value);
                if (it == owners.end()) {
                    return std::nullopt;  // a deleted item
                }
                const char* expected = isLayer ? "layer" : "object";
                for (const std::string& owner : it->second.names) {
                    if (owner != expected) {
                        return who + " selects " + what + ", which is a " + owner;
                    }
                }
                return std::nullopt;
            };
            std::set<std::uint64_t> seen;
            for (const LayerId layer : sel.layers) {
                if (auto p = checkRef(layer.value, true)) {
                    return p;
                }
                if (!seen.insert(layer.value).second) {
                    return who + " lists layer #" + std::to_string(layer.value) + " more than once";
                }
            }
            seen.clear();
            for (const ObjectId object : sel.objects) {
                if (auto p = checkRef(object.value, false)) {
                    return p;
                }
                if (!seen.insert(object.value).second) {
                    return who + " lists object #" + std::to_string(object.value) + " more than once";
                }
            }
            return std::nullopt;
        }();
        if (problem) {
            report(Kind::BadToolpathSelection, {}, {}, *problem);
        }
    }
    return issues;
}

// --- Embedded fonts (T-0211) ---------------------------------------------------------------------

std::string fontKeyFor(const std::vector<std::uint8_t>& bytes) {
    std::uint64_t h = 14695981039346656037ull;
    for (const std::uint8_t b : bytes) {
        h ^= b;
        h *= 1099511628211ull;
    }
    static const char* const kDigits = "0123456789abcdef";
    std::string key(16, '0');
    for (std::size_t i = 16; i-- > 0;) {
        key[i] = kDigits[h & 0xFu];
        h >>= 4;
    }
    return key;
}

bool isFontKey(const std::string& key) {
    if (key.size() != 16) {
        return false;
    }
    for (const char c : key) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
            return false;
        }
    }
    return true;
}

const KeepOutZone* Document::findKeepOutZone(KeepOutZoneId id) const {
    const std::optional<std::size_t> i = keepOutZoneIndex(id);
    return i ? &keepOutZones_[*i] : nullptr;
}

std::optional<std::size_t> Document::keepOutZoneIndex(KeepOutZoneId id) const {
    for (std::size_t i = 0; i < keepOutZones_.size(); ++i) {
        if (keepOutZones_[i].id == id) {
            return i;
        }
    }
    return std::nullopt;
}

KeepOutZoneId Document::insertKeepOutZone(KeepOutZone zone, std::optional<std::size_t> index) {
    if (zone.id.isNull()) {
        zone.id = allocateKeepOutZoneId();
        if (zone.id.isNull()) {
            return {};  // generator exhausted (asserted in allocate)
        }
    } else {
        assert(!findKeepOutZone(zone.id) && "duplicate keep-out zone id");
        ids_.reserveThrough(zone.id.value);
    }
    const KeepOutZoneId id = zone.id;
    const std::size_t at = std::min(index.value_or(keepOutZones_.size()), keepOutZones_.size());
    keepOutZones_.insert(keepOutZones_.begin() + static_cast<std::ptrdiff_t>(at), std::move(zone));
    return id;
}

std::optional<KeepOutZone> Document::takeKeepOutZone(KeepOutZoneId id) {
    const std::optional<std::size_t> i = keepOutZoneIndex(id);
    if (!i) {
        return std::nullopt;
    }
    KeepOutZone zone = std::move(keepOutZones_[*i]);
    keepOutZones_.erase(keepOutZones_.begin() + static_cast<std::ptrdiff_t>(*i));
    return zone;
}

const FontAsset* Document::findFont(const std::string& key) const {
    const auto it = fonts_.find(key);
    return it == fonts_.end() ? nullptr : &it->second;
}

std::string Document::fontProblem(const FontAsset& font) const {
    if (!isFontKey(font.key) || font.bytes.empty()) {
        return "the font is empty or has no valid key";
    }
    if (const auto it = fonts_.find(font.key); it != fonts_.end()) {
        return it->second.bytes == font.bytes ? std::string{} : std::string("a different font has the same key");
    }
    if (font.bytes.size() > kMaxEmbeddedFontBytes) {
        return "the font file is larger than " + std::to_string(kMaxEmbeddedFontBytes >> 20) +
               " MiB, the most a document embeds";
    }
    if (fonts_.size() >= kMaxEmbeddedFonts) {
        return "the document already embeds " + std::to_string(kMaxEmbeddedFonts) +
               " fonts, the most it can hold";
    }
    std::size_t total = font.bytes.size();
    for (const auto& [k, f] : fonts_) {
        total += f.bytes.size();
    }
    if (total > kMaxEmbeddedFontsTotalBytes) {
        return "the embedded fonts would exceed " + std::to_string(kMaxEmbeddedFontsTotalBytes >> 20) +
               " MiB together";
    }
    return {};
}

bool Document::insertFont(FontAsset font) {
    if (!isFontKey(font.key) || font.bytes.empty() || font.bytes.size() > kMaxEmbeddedFontBytes ||
        fonts_.contains(font.key) || fonts_.size() >= kMaxEmbeddedFonts) {
        return false;
    }
    std::size_t total = font.bytes.size();
    for (const auto& [k, f] : fonts_) {
        total += f.bytes.size();
    }
    if (total > kMaxEmbeddedFontsTotalBytes) {
        return false;
    }
    const std::string key = font.key;
    fonts_.emplace(key, std::move(font));
    return true;
}

std::optional<FontAsset> Document::takeFont(const std::string& key) {
    const auto it = fonts_.find(key);
    if (it == fonts_.end()) {
        return std::nullopt;
    }
    FontAsset f = std::move(it->second);
    fonts_.erase(it);
    return f;
}

// --- Embedded bitmaps (T-0273) -------------------------------------------------------------------

const BitmapAsset* Document::findBitmap(const std::string& key) const {
    const auto it = bitmaps_.find(key);
    return it == bitmaps_.end() ? nullptr : &it->second;
}

std::string Document::bitmapProblem(const BitmapAsset& image) const {
    if (!isFontKey(image.key) || image.bytes.empty()) {
        return "the image is empty or has no valid key";
    }
    if (const auto it = bitmaps_.find(image.key); it != bitmaps_.end()) {
        return it->second.bytes == image.bytes ? std::string{} : std::string("a different image has the same key");
    }
    if (image.bytes.size() > kMaxEmbeddedBitmapBytes) {
        return "the image file is larger than " + std::to_string(kMaxEmbeddedBitmapBytes >> 20) +
               " MiB, the most a document embeds";
    }
    if (bitmaps_.size() >= kMaxEmbeddedBitmaps) {
        return "the document already embeds " + std::to_string(kMaxEmbeddedBitmaps) + " images, the most it can hold";
    }
    std::size_t total = image.bytes.size();
    for (const auto& [k, b] : bitmaps_) {
        total += b.bytes.size();
    }
    if (total > kMaxEmbeddedBitmapsTotalBytes) {
        return "the embedded images would exceed " + std::to_string(kMaxEmbeddedBitmapsTotalBytes >> 20) +
               " MiB together";
    }
    return {};
}

bool Document::insertBitmap(BitmapAsset image) {
    if (bitmaps_.contains(image.key) || !bitmapProblem(image).empty()) {
        return false;
    }
    const std::string key = image.key;
    bitmaps_.emplace(key, std::move(image));
    return true;
}

std::optional<BitmapAsset> Document::takeBitmap(const std::string& key) {
    const auto it = bitmaps_.find(key);
    if (it == bitmaps_.end()) {
        return std::nullopt;
    }
    BitmapAsset b = std::move(it->second);
    bitmaps_.erase(it);
    return b;
}

}  // namespace carvetto::document
