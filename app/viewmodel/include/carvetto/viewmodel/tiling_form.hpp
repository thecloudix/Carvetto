#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/machine.hpp"
#include "carvetto/toolpath/tiling.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"

// Tiling Manager model (T-0266): Qt-free logic behind the "Tile Toolpaths" check box of the Save
// Toolpaths dialog and the tile preview. The form holds what the user typed (millimetres), turns
// into the core `TileSpec` / `SaveOptions` of T-0242 (ADR-0043), and `previewTiling` runs the same
// tiling as `prepareSave` so the preview, the tile numbers and the "too large" marks are exactly
// what the saved files will be.

namespace carvetto::viewmodel {

struct TilingForm {
    /// "Tile Toolpaths" ticked.
    bool enabled{false};
    toolpath::TileStrategy strategy{toolpath::TileStrategy::Individual};
    /// Tile size, mm. `height` is unused for feed-through X, `width` for feed-through Y.
    double width{0.0};
    double height{0.0};
    double overlap{0.0};
    bool smallestFirst{false};
    bool originalPosition{false};

    bool operator==(const TilingForm&) const = default;
};

/// A form with the machine's work area as tile size (0 when there is no machine or none is set).
TilingForm defaultTilingForm(const tooling::Machine* machine);

/// Why the form cannot be used (empty when it can): size not above zero, overlap outside
/// [0, size). Only the fields the strategy uses are checked.
std::string validateTilingForm(const TilingForm& form);

/// The core spec of a form; nullopt when the form is not enabled or invalid.
std::optional<toolpath::TileSpec> tileSpecOf(const TilingForm& form);

/// Applies the form to save options: `tiling` set when enabled, work area from the machine
/// (0 when none). Leaves everything else untouched.
void applyTiling(SaveOptions& options, const TilingForm& form, const tooling::Machine* machine);

struct TilePreviewEntry {
    geometry::Box2 rect;
    /// 1-based number of the saved tile ("tile N" in the output names), 0 when the tile has no cuts
    /// and is not saved.
    std::size_t number{0};
    /// The tile is larger than the work area (saving is refused).
    bool oversize{false};
};

struct TilePreview {
    /// Non-empty when the form is invalid or the tiling is refused (then `tiles` is empty).
    std::string error;
    /// The job extent (all toolpaths).
    geometry::Box2 extent;
    /// Every tile rectangle in saving order; empty tiles have number 0.
    std::vector<TilePreviewEntry> tiles;
    /// Number of tiles that will be saved.
    std::size_t saved{0};
    /// Number of saved tiles larger than the work area.
    std::size_t oversize{0};
    /// A thinned outline of the cutting moves for drawing (segments, original coordinates).
    std::vector<std::pair<geometry::Vec2, geometry::Vec2>> outline;
    /// The job extent exceeds the work area, i.e. tiling is needed.
    bool jobExceedsWorkArea{false};

    bool ok() const { return error.empty(); }
};

/// Previews the tiling of the records that have moves. `workAreaX/Y` as in `SaveOptions` (0 = not
/// set). The form must be enabled; otherwise the preview is empty with an error.
TilePreview previewTiling(const std::vector<document::ToolpathRecord>& records, const TilingForm& form,
                          double workAreaX, double workAreaY);

/// One line summarising the preview for the status label.
std::string describeTilePreview(const TilePreview& preview);

}  // namespace carvetto::viewmodel
