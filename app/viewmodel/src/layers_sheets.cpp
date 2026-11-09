#include "carvetto/viewmodel/layers_sheets.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <utility>

#include "carvetto/document/commands.hpp"
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
    return run(doc, std::make_unique<document::DeleteSheetCommand>(id), "the sheet cannot be deleted");
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
