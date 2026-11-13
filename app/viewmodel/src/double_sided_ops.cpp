#include "carvetto/viewmodel/double_sided_ops.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <unordered_set>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/double_sided.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"

namespace carvetto::viewmodel {

namespace {

using document::Document;
using document::ObjectId;
using document::SheetId;

/// Ends the open macro; when it failed, takes back whatever it did (and nothing else).
void closeMacro(Document& doc, std::size_t stepsBefore, bool ok) {
    doc.undoStack().endMacro();
    if (!ok && doc.undoStack().count() > stepsBefore) {
        doc.undoStack().undoAndDiscard(doc);
    }
    doc.undoStack().breakCoalescing();
}

SideCopyResult refuse(std::string why) {
    SideCopyResult r;
    r.edit.ok = false;
    r.edit.message = std::move(why);
    return r;
}

struct Frame {
    const document::Sheet* from{nullptr};
    const document::Sheet* to{nullptr};
    geometry::Transform2 mirror;
};

/// The mirror from sheet `from` to its partner, or an error text.
std::string frameFor(const Document& doc, SheetId from, Frame& out) {
    if (doc.job().type != document::JobType::DoubleSided) {
        return "the job is not double-sided";
    }
    out.from = doc.findSheet(from);
    if (out.from == nullptr) {
        return "the sheet no longer exists";
    }
    out.to = doc.findSheet(out.from->partner);
    if (out.to == nullptr) {
        return "the sheet has no other side";
    }
    const geometry::Vec2 c = out.to->materialBoundsMm().centre();
    out.mirror = doc.job().flipDirection == document::FlipDirection::Horizontal
                     ? geometry::Transform2::mirrorX(c.x)
                     : geometry::Transform2::mirrorY(c.y);
    return {};
}

/// Top-level members of `ids` (an object whose parent is also listed is covered by it).
std::vector<ObjectId> topLevel(const Document& doc, const std::vector<ObjectId>& ids) {
    const std::unordered_set<ObjectId> set(ids.begin(), ids.end());
    std::vector<ObjectId> out;
    std::unordered_set<ObjectId> seen;
    for (const ObjectId id : ids) {
        const document::Object* o = doc.findObject(id);
        if (o == nullptr || (o->parent && set.contains(*o->parent)) || !seen.insert(id).second) {
            continue;
        }
        out.push_back(id);
    }
    return out;
}

/// The first locked layer among `roots` and their descendants, or empty.
std::string lockedProblem(const Document& doc, const std::vector<ObjectId>& roots) {
    for (const ObjectId id : doc.withDescendants(roots)) {
        const document::Object* o = doc.findObject(id);
        const document::Layer* layer = o != nullptr ? doc.findLayer(o->layer) : nullptr;
        if (layer != nullptr && layer->locked) {
            return "layer '" + layer->name + "' is locked";
        }
    }
    return {};
}

/// Pastes mirrored copies of `roots` onto the partner sheet; returns the copies (clipboard order).
/// Fails (empty result) when ids run out. Runs inside the caller's macro.
std::vector<ObjectId> pasteMirrored(Document& doc, const std::vector<ObjectId>& roots, const Frame& f) {
    std::vector<document::Object> clipboard = doc.copyObjects(roots);
    if (clipboard.empty() || !doc.idGenerator().canAllocate(clipboard.size())) {
        return {};
    }
    std::vector<ObjectId> clipboardIds;
    for (const document::Object& o : clipboard) {
        clipboardIds.push_back(o.id);
    }
    auto paste = std::make_unique<document::PasteObjectsCommand>(std::move(clipboard), geometry::Vec2{},
                                                                 document::LayerId{}, f.to->id);
    const document::PasteObjectsCommand* raw = paste.get();
    if (!doc.execute(std::move(paste))) {
        return {};
    }
    const std::unordered_set<ObjectId> rootSet(roots.begin(), roots.end());
    std::vector<ObjectId> created;
    const std::vector<ObjectId>& pasted = raw->pastedIds();
    for (std::size_t i = 0; i < pasted.size() && i < clipboardIds.size(); ++i) {
        if (rootSet.contains(clipboardIds[i])) {
            created.push_back(pasted[i]);
        }
    }
    if (created.empty() || !doc.execute(std::make_unique<document::TransformObjectsCommand>(created, f.mirror))) {
        return {};
    }
    return created;
}

}  // namespace

SideCopyResult copySelectionToOtherSide(Document& doc, bool move) {
    Frame f;
    if (const std::string bad = frameFor(doc, doc.activeSheet(), f); !bad.empty()) {
        return refuse(bad);
    }
    std::vector<ObjectId> picked;
    for (const ObjectId id : doc.selection().ids()) {
        const document::Object* o = doc.findObject(id);
        if (o != nullptr && o->sheet == f.from->id) {
            picked.push_back(id);
        }
    }
    const std::vector<ObjectId> roots = topLevel(doc, picked);
    if (roots.empty()) {
        return refuse("select objects on the active sheet first");
    }
    if (const std::string locked = lockedProblem(doc, roots); !locked.empty()) {
        return refuse(locked);
    }
    SideCopyResult result;
    doc.undoStack().breakCoalescing();
    const std::size_t stepsBefore = doc.undoStack().count();
    doc.undoStack().beginMacro(move ? "Move to other side" : "Copy to other side");
    bool ok = false;
    if (move) {
        ok = doc.execute(std::make_unique<document::MoveObjectsToSheetCommand>(roots, f.to->id)) &&
             doc.execute(std::make_unique<document::TransformObjectsCommand>(roots, f.mirror));
        if (ok) {
            result.objects = roots;
        }
    } else {
        result.objects = pasteMirrored(doc, roots, f);
        ok = !result.objects.empty();
    }
    closeMacro(doc, stepsBefore, ok);
    if (!ok) {
        return refuse(move ? "the objects could not be moved" : "the copies could not be created");
    }
    if (move) {
        doc.selection().clear();
    }
    result.edit.changed = true;
    return result;
}

namespace {

/// Top-level vector-ish objects a record machines, on `sheet`.
std::vector<ObjectId> machinedRoots(const Document& doc, const document::ToolpathRecord& rec, SheetId sheet) {
    std::vector<ObjectId> candidates;
    const document::VectorSelection& sel = rec.selection;
    if (sel.all || !sel.layers.empty()) {
        const std::set<document::LayerId> layers(sel.layers.begin(), sel.layers.end());
        for (const ObjectId id : doc.objectsOnSheet(sheet)) {
            const document::Object* o = doc.findObject(id);
            if (o != nullptr && !o->parent && !o->isDimension() && (sel.all || layers.contains(o->layer))) {
                candidates.push_back(id);
            }
        }
    }
    for (const ObjectId id : sel.objects) {
        const document::Object* o = doc.findObject(id);
        if (o != nullptr && o->sheet == sheet) {
            candidates.push_back(id);
        }
    }
    return topLevel(doc, candidates);
}

std::string uniqueToolpathName(const Document& doc, const std::set<std::string>& extra, const std::string& base) {
    const auto taken = [&](const std::string& n) {
        return extra.contains(n) || std::any_of(doc.toolpaths().begin(), doc.toolpaths().end(),
                                                [&](const document::ToolpathRecord& r) { return r.name == n; });
    };
    std::string name = base;
    for (int n = 2; taken(name); ++n) {
        name = base + " " + std::to_string(n);
    }
    return name;
}

}  // namespace

SideCopyResult copyToolpathsToOtherSide(Document& doc, const std::vector<document::ToolpathId>& ids, bool move) {
    std::vector<document::ToolpathRecord> sources;
    std::set<document::ToolpathId> seen;
    for (const document::ToolpathId id : ids) {
        if (const document::ToolpathRecord* r = doc.findToolpath(id); r != nullptr && seen.insert(id).second) {
            sources.push_back(*r);
        }
    }
    if (sources.empty()) {
        return refuse("select toolpaths first");
    }
    struct Plan {
        document::ToolpathRecord rec;
        Frame frame;
        std::vector<ObjectId> roots;
    };
    std::vector<Plan> plans;
    for (document::ToolpathRecord& rec : sources) {
        Plan p;
        const SheetId from = rec.sheet.isNull() ? doc.activeSheet() : rec.sheet;
        if (const std::string bad = frameFor(doc, from, p.frame); !bad.empty()) {
            return refuse("toolpath '" + rec.name + "': " + bad);
        }
        p.roots = machinedRoots(doc, rec, from);
        if (const std::string locked = lockedProblem(doc, p.roots); !locked.empty()) {
            return refuse("toolpath '" + rec.name + "': " + locked);
        }
        p.rec = std::move(rec);
        plans.push_back(std::move(p));
    }

    SideCopyResult result;
    doc.undoStack().breakCoalescing();
    const std::size_t stepsBefore = doc.undoStack().count();
    doc.undoStack().beginMacro(move ? "Move toolpaths to other side" : "Copy toolpaths to other side");
    bool ok = true;
    // One copy of each machined object, shared by the toolpaths that use it.
    std::map<ObjectId, ObjectId> copyOf;
    for (const Plan& p : plans) {
        std::vector<ObjectId> fresh;
        for (const ObjectId r : p.roots) {
            if (!copyOf.contains(r)) {
                fresh.push_back(r);
            }
        }
        if (fresh.empty()) {
            continue;
        }
        // Mirror in the frame of this toolpath's sheet (all toolpaths of one pair share it).
        std::vector<document::Object> clipboard = doc.copyObjects(fresh);
        std::vector<ObjectId> clipIds;
        for (const document::Object& o : clipboard) {
            clipIds.push_back(o.id);
        }
        if (clipboard.empty() || !doc.idGenerator().canAllocate(clipboard.size())) {
            ok = false;
            break;
        }
        auto paste = std::make_unique<document::PasteObjectsCommand>(std::move(clipboard), geometry::Vec2{},
                                                                     document::LayerId{}, p.frame.to->id);
        const document::PasteObjectsCommand* raw = paste.get();
        if (!doc.execute(std::move(paste))) {
            ok = false;
            break;
        }
        std::vector<ObjectId> created;
        const std::set<ObjectId> freshSet(fresh.begin(), fresh.end());
        for (std::size_t i = 0; i < raw->pastedIds().size() && i < clipIds.size(); ++i) {
            if (freshSet.contains(clipIds[i])) {
                copyOf[clipIds[i]] = raw->pastedIds()[i];
                created.push_back(raw->pastedIds()[i]);
            }
        }
        if (!created.empty() &&
            !doc.execute(std::make_unique<document::TransformObjectsCommand>(created, p.frame.mirror))) {
            ok = false;
            break;
        }
        result.objects.insert(result.objects.end(), created.begin(), created.end());
    }
    std::set<std::string> usedNames;
    std::vector<document::ToolpathId> originals;
    for (std::size_t i = 0; ok && i < plans.size(); ++i) {
        const Plan& p = plans[i];
        document::ToolpathRecord rec = p.rec;
        originals.push_back(rec.id);
        rec.id = {};
        rec.sheet = p.frame.to->id;
        rec.selection = {};
        for (const ObjectId r : p.roots) {
            rec.selection.objects.push_back(copyOf.at(r));
        }
        if (rec.selection.objects.empty()) {
            ok = false;  // nothing to machine on the other side
            break;
        }
        if (rec.moves) {
            rec.moves = document::shareMoves(document::flipMoves(*rec.moves, p.frame.to->materialBoundsMm(),
                                                                  doc.job().flipDirection));
        }
        rec.name = uniqueToolpathName(
            doc, usedNames, rec.name + (p.frame.to->side == document::Side::Back ? " (back)" : " (front)"));
        usedNames.insert(rec.name);
        auto add = std::make_unique<document::AddToolpathCommand>(std::move(rec));
        const document::AddToolpathCommand* raw = add.get();
        if (!doc.execute(std::move(add))) {
            ok = false;
            break;
        }
        result.toolpaths.push_back(raw->id());
    }
    if (ok && move) {
        ok = doc.execute(std::make_unique<document::RemoveToolpathsCommand>(originals));
    }
    closeMacro(doc, stepsBefore, ok);
    if (!ok) {
        return refuse("the toolpaths could not be copied to the other side (the copies of their vectors, or the "
                      "vectors they machine, are missing)");
    }
    result.edit.changed = true;
    return result;
}

EditResult makeSingleSided(Document& doc) {
    EditResult r;
    if (doc.job().type != document::JobType::DoubleSided) {
        r.ok = false;
        r.message = "the job is not double-sided";
        return r;
    }
    document::Job job = doc.job();
    job.type = document::JobType::SingleSided;
    doc.undoStack().breakCoalescing();
    const std::size_t stepsBefore = doc.undoStack().count();
    doc.undoStack().beginMacro("Single-sided job");
    bool ok = doc.execute(std::make_unique<document::SetJobCommand>(job));
    for (const document::Sheet& s : std::vector<document::Sheet>(doc.sheets())) {
        if (!ok) {
            break;
        }
        if (s.side != document::Side::Front || !s.partner.isNull()) {
            document::Sheet plain = s;
            plain.side = document::Side::Front;
            plain.partner = {};
            ok = doc.execute(std::make_unique<document::SetSheetPropertiesCommand>(std::move(plain)));
        }
    }
    closeMacro(doc, stepsBefore, ok);
    if (!ok) {
        r.ok = false;
        r.message = "the job cannot be made single-sided";
        return r;
    }
    r.changed = true;
    return r;
}

EditResult applyJobSetup(Document& doc, const document::Job& job) {
    EditResult r;
    const bool wasDouble = doc.job().type == document::JobType::DoubleSided;
    const bool isDouble = job.type == document::JobType::DoubleSided;
    if (!wasDouble && !isDouble && job == doc.job()) {
        return r;
    }
    doc.undoStack().breakCoalescing();
    const std::size_t stepsBefore = doc.undoStack().count();
    doc.undoStack().beginMacro("Job setup");
    bool ok = true;
    std::string why;
    if (isDouble) {
        const EditResult e = makeDoubleSided(doc, job.flipDirection, job.zeroOffSameSide);
        ok = e.ok;
        why = e.message;
    } else if (wasDouble) {
        const EditResult e = makeSingleSided(doc);
        ok = e.ok;
        why = e.message;
    }
    if (ok && !(job == doc.job())) {
        ok = doc.execute(std::make_unique<document::SetJobCommand>(job));
        why = "the job cannot be changed";
    }
    closeMacro(doc, stepsBefore, ok);
    if (!ok) {
        r.ok = false;
        r.message = why;
        return r;
    }
    r.changed = doc.undoStack().count() > stepsBefore;
    return r;
}

EditResult activateSide(Document& doc, document::Side side) {
    const document::Sheet* active = doc.findSheet(doc.activeSheet());
    if (active != nullptr && active->side == side && doc.findSheet(active->partner) != nullptr) {
        return {};
    }
    return swapSides(doc);
}

}  // namespace carvetto::viewmodel
