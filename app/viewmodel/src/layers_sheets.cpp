#include "carvetto/viewmodel/layers_sheets.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/double_sided.hpp"
#include "carvetto/document/object_property_commands.hpp"

namespace carvetto::viewmodel {

namespace {

using document::Document;

EditResult refused(std::string message) {
    EditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

/// Runs one command as its own undo step (never merged with a neighbouring edit of the same thing).
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

template <typename Container>
std::string freeName(const Container& items, const std::string& stem) {
    std::set<std::string> used;
    for (const auto& item : items) {
        used.insert(item.name);
    }
    for (std::size_t n = items.size() + 1;; ++n) {
        std::string candidate = stem + " " + std::to_string(n);
        if (!used.contains(candidate)) {
            return candidate;
        }
    }
}

/// Opens an undo macro and closes it on `finish()` (or on scope exit).
class UndoStackGuard {
   public:
    UndoStackGuard(Document& doc, const char* text) : doc_(doc) {
        doc_.undoStack().breakCoalescing();
        doc_.undoStack().beginMacro(text);
    }
    ~UndoStackGuard() { finish(); }
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

bool sizeOk(double v) {
    return std::isfinite(v) && v > 0.0 && v <= kMaxSheetSizeMm;
}

}  // namespace

// --- Layers -----------------------------------------------------------------------------------

EditResult addLayer(Document& doc, const std::string& name) {
    if (doc.layers().size() >= kMaxLayers) {
        return refused("a document holds at most " + std::to_string(kMaxLayers) + " layers");
    }
    const std::string chosen = name.empty() ? freeName(doc.layers(), "Layer") : name;
    if (const std::string why = document::nameProblem(chosen, true); !why.empty()) {
        return refused(why);
    }
    document::Layer layer;
    layer.name = chosen;
    // Distinct colours help tell layers apart: cycle a small palette.
    static const document::Colour palette[] = {{0, 0, 0},     {200, 40, 40}, {40, 140, 40},
                                               {40, 70, 200}, {200, 120, 0}, {140, 40, 160}};
    layer.colour = palette[doc.layers().size() % (sizeof(palette) / sizeof(palette[0]))];
    return run(doc, std::make_unique<document::AddLayerCommand>(std::move(layer), true),
               "the layer cannot be added (no ids left)");
}

EditResult renameLayer(Document& doc, document::LayerId id, const std::string& name) {
    const document::Layer* layer = doc.findLayer(id);
    if (layer == nullptr) {
        return refused("the layer no longer exists");
    }
    if (const std::string why = document::nameProblem(name, true); !why.empty()) {
        return refused(why);
    }
    if (layer->name == name) {
        return {};
    }
    document::Layer next = *layer;
    next.name = name;
    return run(doc, std::make_unique<document::SetLayerPropertiesCommand>(std::move(next)),
               "the layer cannot be renamed");
}

EditResult setLayerVisible(Document& doc, document::LayerId id, bool visible) {
    const document::Layer* layer = doc.findLayer(id);
    if (layer == nullptr) {
        return refused("the layer no longer exists");
    }
    if (layer->visible == visible) {
        return {};
    }
    document::Layer next = *layer;
    next.visible = visible;
    return run(doc, std::make_unique<document::SetLayerPropertiesCommand>(std::move(next)),
               "the layer cannot be changed");
}

EditResult setLayerLocked(Document& doc, document::LayerId id, bool locked) {
    const document::Layer* layer = doc.findLayer(id);
    if (layer == nullptr) {
        return refused("the layer no longer exists");
    }
    if (layer->locked == locked) {
        return {};
    }
    document::Layer next = *layer;
    next.locked = locked;
    return run(doc, std::make_unique<document::SetLayerPropertiesCommand>(std::move(next)),
               "the layer cannot be changed");
}

EditResult setLayerColour(Document& doc, document::LayerId id, document::Colour colour) {
    const document::Layer* layer = doc.findLayer(id);
    if (layer == nullptr) {
        return refused("the layer no longer exists");
    }
    if (layer->colour == colour) {
        return {};
    }
    document::Layer next = *layer;
    next.colour = colour;
    return run(doc, std::make_unique<document::SetLayerPropertiesCommand>(std::move(next)),
               "the layer cannot be changed");
}

EditResult deleteLayer(Document& doc, document::LayerId id, bool deleteContents) {
    if (doc.findLayer(id) == nullptr) {
        return refused("the layer no longer exists");
    }
    if (doc.layers().size() <= 1) {
        return refused("the last layer cannot be deleted");
    }
    return run(doc, std::make_unique<document::DeleteLayerCommand>(id, deleteContents),
               "the layer cannot be deleted");
}

EditResult moveLayer(Document& doc, document::LayerId id, std::size_t newIndex) {
    const auto index = doc.layerIndex(id);
    if (!index) {
        return refused("the layer no longer exists");
    }
    if (newIndex >= doc.layers().size()) {
        return refused("there is no such position");
    }
    if (*index == newIndex) {
        return {};
    }
    return run(doc, std::make_unique<document::MoveLayerCommand>(id, newIndex), "the layer cannot be moved");
}

EditResult moveSelectionToLayer(Document& doc, document::LayerId id) {
    const document::Layer* target = doc.findLayer(id);
    if (target == nullptr) {
        return refused("the layer no longer exists");
    }
    if (doc.selection().empty()) {
        return refused("nothing is selected");
    }
    if (target->locked) {
        return refused("the layer \"" + target->name + "\" is locked");
    }
    std::vector<document::ObjectId> ids;
    bool any = false;
    for (const document::ObjectId oid : doc.selection().ids()) {
        const document::Object* obj = doc.findObject(oid);
        if (obj == nullptr) {
            continue;
        }
        const document::Layer* from = doc.findLayer(obj->layer);
        if (from != nullptr && from->locked) {
            return refused("an object is on a locked layer");
        }
        any = any || obj->layer != id;
        ids.push_back(oid);
    }
    if (!any) {
        return {};
    }
    return run(doc, std::make_unique<document::MoveObjectsToLayerCommand>(std::move(ids), id),
               "the objects cannot be moved");
}

// --- Sheets -----------------------------------------------------------------------------------

EditResult addSheet(Document& doc, const std::string& name) {
    if (doc.sheets().size() >= kMaxSheets) {
        return refused("a document holds at most " + std::to_string(kMaxSheets) + " sheets");
    }
    const std::string chosen = name.empty() ? freeName(doc.sheets(), "Sheet") : name;
    if (const std::string why = document::nameProblem(chosen, true); !why.empty()) {
        return refused(why);
    }
    document::Sheet sheet;
    if (const document::Sheet* active = doc.findSheet(doc.activeSheet())) {
        sheet = *active;
    }
    sheet.id = {};
    sheet.side = document::Side::Front;  // a new sheet is never part of a front/back pair
    sheet.partner = {};
    sheet.name = chosen;
    sheet.visible = true;
    return run(doc, std::make_unique<document::AddSheetCommand>(std::move(sheet), true),
               "the sheet cannot be added (no ids left)");
}

EditResult renameSheet(Document& doc, document::SheetId id, const std::string& name) {
    const document::Sheet* sheet = doc.findSheet(id);
    if (sheet == nullptr) {
        return refused("the sheet no longer exists");
    }
    if (const std::string why = document::nameProblem(name, true); !why.empty()) {
        return refused(why);
    }
    if (sheet->name == name) {
        return {};
    }
    document::Sheet next = *sheet;
    next.name = name;
    return run(doc, std::make_unique<document::SetSheetPropertiesCommand>(std::move(next)),
               "the sheet cannot be renamed");
}

EditResult deleteSheet(Document& doc, document::SheetId id) {
    if (doc.findSheet(id) == nullptr) {
        return refused("the sheet no longer exists");
    }
    if (doc.sheets().size() <= 1) {
        return refused("the last sheet cannot be deleted");
    }
    const document::Sheet* partner = doc.findSheet(doc.findSheet(id)->partner);
    if (partner == nullptr) {
        return run(doc, std::make_unique<document::DeleteSheetCommand>(id), "the sheet cannot be deleted");
    }
    // Unpair the surviving face first so no sheet names a deleted one; one undo step.
    document::Sheet unpaired = *partner;
    unpaired.partner = {};
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Delete sheet");
    bool ok = doc.execute(std::make_unique<document::SetSheetPropertiesCommand>(std::move(unpaired)));
    ok = ok && doc.execute(std::make_unique<document::DeleteSheetCommand>(id));
    doc.undoStack().endMacro();
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refused("the sheet cannot be deleted");
    }
    EditResult r;
    r.changed = true;
    return r;
}

EditResult setSheetMaterial(Document& doc, document::SheetId id, const SheetMaterialEdit& edit) {
    const document::Sheet* sheet = doc.findSheet(id);
    if (sheet == nullptr) {
        return refused("the sheet no longer exists");
    }
    document::Sheet next = *sheet;
    if (edit.widthMm) {
        next.sizeMm.x = *edit.widthMm;
    }
    if (edit.heightMm) {
        next.sizeMm.y = *edit.heightMm;
    }
    if (edit.thicknessMm) {
        next.sizeMm.z = *edit.thicknessMm;
    }
    if (edit.zZero) {
        next.zZero = *edit.zZero;
    }
    if (edit.datum) {
        next.datum = *edit.datum;
    }
    if (!sizeOk(next.sizeMm.x) || !sizeOk(next.sizeMm.y) || !sizeOk(next.sizeMm.z)) {
        return refused("the material size must be a positive number of at most " +
                       std::to_string(static_cast<long long>(kMaxSheetSizeMm)) + " mm");
    }
    if (next == *sheet) {
        return {};
    }
    return run(doc, std::make_unique<document::SetSheetPropertiesCommand>(std::move(next)),
               "the material cannot be changed");
}

// --- Double-sided jobs --------------------------------------------------------------------------

EditResult makeDoubleSided(Document& doc, document::FlipDirection flip, bool zeroOffSameSide) {
    if (doc.job().type == document::JobType::Rotary) {
        return refused("a rotary job cannot be double-sided");
    }
    const document::Sheet* active = doc.findSheet(doc.activeSheet());
    if (active == nullptr) {
        return refused("the sheet no longer exists");
    }
    const bool needsBack = doc.findSheet(active->partner) == nullptr;
    if (needsBack && active->side == document::Side::Back) {
        return refused("the back sheet has no front sheet");
    }
    if (needsBack && doc.sheets().size() >= kMaxSheets) {
        return refused("a document holds at most " + std::to_string(kMaxSheets) + " sheets");
    }
    document::Job job = doc.job();
    job.type = document::JobType::DoubleSided;
    job.flipDirection = flip;
    job.zeroOffSameSide = zeroOffSameSide;

    UndoStackGuard guard(doc, "Double-sided job");
    bool ok = job == doc.job() || doc.execute(std::make_unique<document::SetJobCommand>(job));
    if (needsBack) {
        document::Sheet back = *active;
        back.id = {};
        back.side = document::Side::Back;
        back.partner = active->id;
        back.visible = true;
        back.name = active->name + " (back)";
        if (!document::nameProblem(back.name, true).empty()) {
            back.name = freeName(doc.sheets(), "Back");
        }
        const document::SheetId frontId = active->id;
        auto add = std::make_unique<document::AddSheetCommand>(std::move(back), false);
        document::AddSheetCommand* raw = add.get();
        ok = ok && doc.execute(std::move(add));
        if (ok) {
            document::Sheet front = *doc.findSheet(frontId);
            front.partner = raw->id();
            ok = doc.execute(std::make_unique<document::SetSheetPropertiesCommand>(std::move(front)));
        }
    }
    guard.finish();
    if (!ok) {
        return refused("the job cannot be made double-sided");
    }
    EditResult r;
    r.changed = true;
    return r;
}

EditResult setFlipDirection(Document& doc, document::FlipDirection flip) {
    if (doc.job().type != document::JobType::DoubleSided) {
        return refused("the job is not double-sided");
    }
    if (doc.job().flipDirection == flip) {
        return {};
    }
    document::Job job = doc.job();
    job.flipDirection = flip;
    return run(doc, std::make_unique<document::SetJobCommand>(job), "the flip direction cannot be changed");
}

EditResult swapSides(Document& doc) {
    const document::Sheet* active = doc.findSheet(doc.activeSheet());
    if (active == nullptr || doc.findSheet(active->partner) == nullptr) {
        return refused("the active sheet has no other side");
    }
    if (!doc.setActiveSheet(active->partner)) {
        return refused("the other side cannot be activated");
    }
    EditResult r;
    r.changed = true;
    return r;
}

namespace {

geometry::Box2 sheetContent(const Document& doc, document::SheetId sheet) {
    geometry::Box2 box;
    for (const document::ObjectId id : doc.objectsOnSheet(sheet)) {
        if (const auto b = doc.objectBounds(id)) {
            box.expand(*b);
        }
    }
    return box;
}

}  // namespace

std::vector<SideOutline> multiSidedView(const Document& doc) {
    std::vector<SideOutline> out;
    const document::Sheet* active = doc.findSheet(doc.activeSheet());
    if (active == nullptr) {
        return out;
    }
    const document::Sheet* partner =
        doc.job().type == document::JobType::DoubleSided ? doc.findSheet(active->partner) : nullptr;
    const auto describe = [&](const document::Sheet& s, const document::Sheet* other, bool isActive) {
        SideOutline o;
        o.sheet = s.id;
        o.side = s.side;
        o.active = isActive;
        o.material = s.materialBoundsMm();
        o.content = sheetContent(doc, s.id);
        if (other != nullptr) {
            const geometry::Box2 theirs = sheetContent(doc, other->id);
            if (!theirs.isEmpty()) {
                o.otherSideContent = document::flipBox(o.material, doc.job().flipDirection, theirs);
            }
        }
        return o;
    };
    out.push_back(describe(*active, partner, true));
    if (partner != nullptr) {
        out.push_back(describe(*partner, active, false));
    }
    return out;
}

double sheetTopZ(const Document& doc, document::SheetId sheet) {
    const document::Sheet* s = doc.findSheet(sheet);
    if (s == nullptr) {
        return 0.0;
    }
    return document::materialTopZ(s->zZero, doc.job().zeroOffSameSide, s->sizeMm.z, s->side);
}

// --- History ------------------------------------------------------------------------------------

std::vector<HistoryEntry> historyEntries(const Document& doc) {
    std::vector<HistoryEntry> out;
    const document::UndoStack& stack = doc.undoStack();
    out.reserve(stack.count());
    for (std::size_t i = 0; i < stack.count(); ++i) {
        out.push_back({i, stack.stepText(i), i < stack.index()});
    }
    return out;
}

EditResult jumpToHistory(Document& doc, std::size_t applied) {
    document::UndoStack& stack = doc.undoStack();
    if (stack.inMacro()) {
        return refused("an edit is in progress");
    }
    if (applied > stack.count()) {
        return refused("there is no such step");
    }
    EditResult r;
    stack.breakCoalescing();
    while (stack.index() > applied && doc.undo()) {
        r.changed = true;
    }
    while (stack.index() < applied && doc.redo()) {
        r.changed = true;
    }
    stack.breakCoalescing();
    return r;
}

}  // namespace carvetto::viewmodel
