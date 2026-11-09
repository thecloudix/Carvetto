#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/layer.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Qt-free operations of the Layers and Sheets panels and the undo history list (T-0210, ADR-0014).
// Stateless functions over a `Document&` passed per call (no reference is kept, so a replaced
// document cannot dangle). Each edit is ONE undo step through the commands of `commands.hpp`,
// validated completely first: a refusal returns a message and leaves document and history alone.
// Names are bounded (`document::nameProblem`); the number of layers and sheets is capped.

namespace carvetto::viewmodel {

inline constexpr std::size_t kMaxLayers = 1000;
inline constexpr std::size_t kMaxSheets = 1000;
/// Largest material block edge (mm) a sheet accepts.
inline constexpr double kMaxSheetSizeMm = 1.0e6;

// --- Layers -----------------------------------------------------------------------------------

/// Adds a layer on top and makes it active. An empty `name` picks the next free "Layer N".
EditResult addLayer(document::Document& doc, const std::string& name = {});
EditResult renameLayer(document::Document& doc, document::LayerId id, const std::string& name);
EditResult setLayerVisible(document::Document& doc, document::LayerId id, bool visible);
EditResult setLayerLocked(document::Document& doc, document::LayerId id, bool locked);
EditResult setLayerColour(document::Document& doc, document::LayerId id, document::Colour colour);
/// Deletes a layer. Its objects move to the default layer, or are deleted with `deleteContents`.
/// The last layer cannot be deleted.
EditResult deleteLayer(document::Document& doc, document::LayerId id, bool deleteContents);
/// Moves a layer to position `newIndex` of the stacking order (0 = bottom).
EditResult moveLayer(document::Document& doc, document::LayerId id, std::size_t newIndex);
/// Moves the selected objects to `id`. Refused when that layer or a source layer is locked.
EditResult moveSelectionToLayer(document::Document& doc, document::LayerId id);

// --- Sheets -----------------------------------------------------------------------------------

/// Adds a sheet with the material block of the active sheet and makes it active. An empty `name`
/// picks the next free "Sheet N".
EditResult addSheet(document::Document& doc, const std::string& name = {});
EditResult renameSheet(document::Document& doc, document::SheetId id, const std::string& name);
/// Deletes a sheet and the objects on it. The last sheet cannot be deleted.
EditResult deleteSheet(document::Document& doc, document::SheetId id);

/// A material-block edit; unset members keep the sheet's value. Sizes are millimetres.
struct SheetMaterialEdit {
    std::optional<double> widthMm;
    std::optional<double> heightMm;
    std::optional<double> thicknessMm;
    std::optional<document::ZZero> zZero;
    std::optional<document::Datum> datum;
};
/// Edits the per-sheet material block. Sizes must be finite, positive and at most `kMaxSheetSizeMm`.
EditResult setSheetMaterial(document::Document& doc, document::SheetId id, const SheetMaterialEdit& edit);

// --- Undo history ------------------------------------------------------------------------------

struct HistoryEntry {
    std::size_t index{0};  ///< Position in the stack (0 = oldest step).
    std::string text;
    bool applied{false};   ///< Done (false: undone, can be redone).
};
/// Every recorded step, oldest first; `applied` marks the steps currently done.
std::vector<HistoryEntry> historyEntries(const document::Document& doc);
/// Brings the document to the state after `applied` steps (0 = the start, `count()` = all) by
/// undoing or redoing. Refused while a macro is open or when `applied` is beyond the history.
EditResult jumpToHistory(document::Document& doc, std::size_t applied);

}  // namespace carvetto::viewmodel
