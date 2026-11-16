#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/io/parts_import.hpp"

// Turns a parsed parts list into document content (T-0329, ADR-0110): panels grouped by material and
// thickness, nested onto sheets (`nestRectangles`), each panel a rectangle on the panel layer with its
// holes (circles) and dados (rectangles) on their own layers. ONE undo step for everything.
// Toolpaths are made by the viewmodel on top of this result (the session lives there).

namespace carvetto::io {

struct BuiltFeature {
    document::ObjectId object;
    /// Hole or dado depth, mm; 0 = through (holes only).
    double depthMm{0.0};
};

struct BuiltSheet {
    document::SheetId sheet;
    std::string material;
    double thicknessMm{0.0};
    std::vector<document::ObjectId> panels;
    std::vector<BuiltFeature> holes;
    std::vector<BuiltFeature> dados;
};

struct BuildResult {
    bool ok{false};
    std::vector<PartsDiagnostic> diagnostics;
    std::vector<BuiltSheet> sheets;
    document::LayerId panelLayer, holeLayer, dadoLayer;
    /// Panels placed / panels that fit no sheet.
    std::size_t placed{0};
    std::size_t unplaced{0};
};

/// Builds `list` into `doc` as one undo step (a macro, so it nests in a caller's). Nothing is
/// changed when nothing can be placed or a layer of the mapping's name is hidden or locked. The
/// document's active sheet is reused for the first group when it holds no objects, else sheets are
/// added; the first sheet built becomes active.
BuildResult buildPanels(document::Document& doc, const PartsList& list, const PartsMapping& mapping);

}  // namespace carvetto::io
