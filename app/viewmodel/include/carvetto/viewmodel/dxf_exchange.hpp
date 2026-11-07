#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"

namespace carvetto::viewmodel {

/// One entry of the import dialog's "Units" list.
struct DxfUnitChoice {
    std::string label;
    /// Millimetres per drawing unit; nullopt = use the file's `$INSUNITS` (ADR-0015).
    std::optional<double> mmPerUnit;
};

/// The units the DXF import can be forced to: the first entry is "from the file" (the default),
/// followed by millimetres, centimetres, metres, inches and feet.
const std::vector<DxfUnitChoice>& dxfUnitChoices();

/// What an import did, or why it did nothing.
struct DxfImportOutcome {
    bool ok{false};
    /// Set when `!ok`: one line for the error dialog.
    std::string error;
    /// Vector objects added (one per DXF contour).
    std::size_t objectsAdded{0};
    /// Layers created (DXF layers whose name did not exist in the document are added; layers with
    /// an existing name are reused).
    std::size_t layersAdded{0};
    /// Text items that were not imported (text import is not supported yet).
    std::size_t textSkipped{0};
    /// Import warnings, each one line ("line N: ... (xK)"), plus a note about skipped text.
    std::vector<std::string> warnings;
};

/// Imports DXF `bytes` into `doc` (its active sheet) as ONE undoable step ("Import DXF"): DXF
/// layers are matched to document layers by name (new ones are added with their colour and
/// visibility), every contour becomes a vector object, and the new objects become the selection.
/// `forcedMmPerUnit` overrides the file's units. On failure (unreadable file, id space exhausted)
/// the document is left untouched. A drawing without contours succeeds with a warning and adds
/// nothing (no undo step).
DxfImportOutcome importDxfIntoDocument(document::Document& doc, std::span<const std::uint8_t> bytes,
                                       std::optional<double> forcedMmPerUnit = std::nullopt);

/// Result of `exportDocumentDxf`.
struct DxfExportOutcome {
    bool ok{false};
    std::string error;
    std::vector<std::uint8_t> bytes;
    /// Vector contours written.
    std::size_t contoursWritten{0};
};

/// DXF (R2000, mm) of the vector objects on the active sheet, or of the selection (groups
/// expanded) when `selectionOnly`. Only layers that hold exported geometry are written. Fails
/// with an error text when there is nothing to export.
DxfExportOutcome exportDocumentDxf(const document::Document& doc, bool selectionOnly);

}  // namespace carvetto::viewmodel
