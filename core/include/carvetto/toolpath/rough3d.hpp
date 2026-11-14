#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/simulate/tool_shape.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// 3D roughing of a composed relief model (T-0296, ADR-0071; feature-inventory "3D Rough Toolpath").
// The tool is "dropped" onto the model: at every model grid node the lowest tip height at which
// the tool does not go below the model (plus the allowance) is computed with the tool's exact
// profile (`simulate::ToolShape`). Toolpaths run on the model's grid nodes, so all of this is exact
// at the grid and never gouges at it (see the ADR for the segment argument).

namespace carvetto::toolpath {

/// How the model is roughed.
enum class Rough3dStrategy {
    /// Constant-Z slices: at every level the nodes the tool may visit at that height are cleared by
    /// parallel rows (pocket-style raster clearing), from the top level down.
    ZLevel,
    /// One pass of parallel rows over the drop-tool surface (the tip follows the surface).
    Raster,
};

/// Region the roughing may machine.
enum class Rough3dBoundary {
    Material,  ///< The whole model grid (the material block).
    Model,     ///< The opaque (non-transparent) cells of the model.
    Vectors,   ///< Inside the selected closed vectors (even-odd).
};

/// Direction of the raster rows.
enum class RasterAxis { AlongX, AlongY };

/// Whether neighbouring rows are cut back and forth or all in the same direction.
enum class RasterTravel { ZigZag, OneWay };

/// Drop the tool onto the model (never gouges) or project the tip straight onto the surface.
enum class Rough3dTipMode { DropTool, Project };

/// One tool of the multi-tool list: the first is the clearance tool, later ones are rest tools that
/// only cut where the tools before them left more than `Rough3dParams::restTolerance`. Stepover and
/// stepdown come from the cutting data (`stepover`, `passDepth`).
struct Rough3dTool {
    tooling::Tool tool;
    /// Empty = the tool's default data (`effectiveCuttingData`).
    std::optional<tooling::CuttingData> cuttingData;
    /// Flute length for the shank diagnostics; empty = unlimited.
    std::optional<double> fluteLength;

    bool operator==(const Rough3dTool&) const = default;
};

/// Parameters of a 3D rough. Lengths in mm, angles in radians. Model heights are machine Z plus
/// `zOffset`.
struct Rough3dParams {
    std::string name{"3D Rough"};
    Rough3dStrategy strategy{Rough3dStrategy::ZLevel};
    Rough3dTipMode tip{Rough3dTipMode::DropTool};

    /// Added to every model height to give machine Z.
    double zOffset{0.0};
    /// Z of the top of the material and the lowest Z the tool tip may reach (the floor).
    double stockTop{0.0};
    double stockBottom{-20.0};
    /// Material left on the model (>= 0), measured vertically at grid nodes.
    double allowance{0.0};
    /// Height of rapid moves; must be above `stockTop`.
    double safeZ{5.0};

    Rough3dBoundary boundary{Rough3dBoundary::Material};
    /// Model: grows (>0) or shrinks (<0) the opaque region by this distance. Vectors: grows or
    /// shrinks the selected region (distance to the polygon edges).
    double boundaryOffset{0.0};
    /// Closed polylines for `Rough3dBoundary::Vectors` (even-odd).
    std::vector<geometry::Polyline> vectors;

    RasterAxis axis{RasterAxis::AlongX};
    RasterTravel travel{RasterTravel::ZigZag};
    /// Climb: the uncut material stays on the right of the travel direction (rows step towards -Y
    /// for AlongX, towards +X for AlongY); Conventional steps the other way.
    CutDirection direction{CutDirection::Climb};
    /// Reverses the order of the rows (the inventory's "Reverse Step Direction").
    bool reverseSteps{false};

    /// Enter with a ramp of angle `rampAngle` along the cut instead of a vertical plunge.
    bool rampPlunge{false};
    double rampAngle{0.17453292519943295};  // 10 degrees

    /// Consecutive path segments whose heights differ by no more than this are merged at the
    /// higher height (never gouges, leaves at most this much more material). 0 = no merging.
    double tolerance{0.0};
    /// Rest tools cut only where the stock left is more than this above what they can reach.
    double restTolerance{0.01};

    bool operator==(const Rough3dParams&) const = default;
};

/// Result of `rough3dToolpath`: one toolpath per input tool, in tool order (empty when the tool has
/// nothing to cut), and the diagnostics.
struct Rough3dResult {
    std::vector<Toolpath> toolpaths;
    std::vector<ToolpathDiagnostic> diagnostics;
};

/// Default cap on the work of one run: model cells examined by the drop plus nodes visited.
inline constexpr std::uint64_t kRough3dDefaultWorkLimit = 4'000'000'000ULL;
/// Default cap on the moves of one toolpath.
inline constexpr std::uint64_t kRough3dDefaultMoveLimit = 20'000'000ULL;

enum class DropStatus { Ok, Cancelled, WorkLimit, TooLarge };

/// The drop-tool surface: for every cell of `model` the lowest tool tip Z (tip centred on the cell
/// centre) at which no opaque model cell centre within reach is cut, i.e.
///   max over cells q of (model(q) + zOffset - h(|p - q|)),
/// -infinity when nothing is within reach. Values are rounded up to float. Cell columns are
/// bounded by a block-maximum pruning, so a smooth surface costs far less than the full
/// footprint. `work` receives (and is limited by `workLimit`) the cell/block evaluations;
/// `cancel` is polled per row. Output is row-major like the model.
DropStatus dropToolSurface(const relief::HeightField& model, const simulate::ToolShape& tool, double zOffset,
                           std::vector<float>& out, std::uint64_t workLimit, std::uint64_t& work,
                           const std::atomic<bool>* cancel = nullptr);

/// Roughs `model` with the tool list (see the file comment and ADR-0071). Invalid input gives an
/// `InvalidParameters`/`InvalidTool` diagnostic and no toolpaths' moves.
Rough3dResult rough3dToolpath(const relief::HeightField& model, const std::vector<Rough3dTool>& tools,
                              const Rough3dParams& params, const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
