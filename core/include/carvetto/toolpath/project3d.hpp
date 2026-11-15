#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/relief/relief.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/tab_placement.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Project Toolpath onto 3D Model and 3D tabs (T-0298, ADR-0073; feature-inventory "Project toolpath
// onto 3D Model" and "Create 3D Tabs").
//
// A finished 2D toolpath (Profile, Pocket, V-Carve, ...) is re-heighted onto a composed relief
// model. The input Z of a cutting move is a depth relative to the surface (0 = the surface, the usual
// "Z zero at the material top", negative = into the material); the output Z is that depth below
// the model surface at the move's XY. Z above 0 (safe heights) and rapids are left alone, so
// retracts clear the model only when the input safe height does.

namespace carvetto::toolpath {

/// Which surface the depth is measured from.
enum class ProjectTipMode {
    /// The drop-tool surface (`dropToolSurface`, ADR-0071): the lowest tip height at which the tool
    /// does not cut the model. Depth 0 therefore never gouges the model at the grid.
    DropTool,
    /// The model surface itself under the tool tip (a ball-nose tip follows it, a flat tool gouges).
    Project,
};

struct Project3dParams {
    /// Name of the result; empty = the input's name + " (3D)".
    std::string name;
    ProjectTipMode mode{ProjectTipMode::DropTool};
    /// Machine Z = model height + `zOffset`.
    double zOffset{0.0};
    /// Added to the surface where the model is opaque (>= 0).
    double allowance{0.0};
    /// Surface height used where the model is absent or transparent (the material top).
    double outsideSurface{0.0};
    /// Lowest Z the tool tip may reach: the material bottom. Deeper projected Z is capped here.
    double stockBottom{-20.0};
    /// Largest deviation (mm) of a straight piece of the result from the projected surface (also the
    /// chordal tolerance of flattened arcs). > 0.
    double tolerance{0.01};
    /// Longest XY length of one piece before the surface is looked at again; 0 = the model's cell size.
    double maxSegment{0.0};
    /// 3D tabs (`TabOptions::enabled`; length, thickness, spacing, count, distance, avoidCorners; `threeD`
    /// ramps the tab at 45 degrees). Thickness is measured above the lowest depth of the input toolpath.
    TabOptions tabs;

    bool operator==(const Project3dParams&) const = default;
};

struct Project3dResult {
    Toolpath path;  ///< Empty (no moves) when refused.
    std::vector<ToolpathDiagnostic> diagnostics;
};

/// Default caps of one run: surface evaluations and output moves.
inline constexpr std::uint64_t kProject3dDefaultWorkLimit = 200'000'000ULL;
inline constexpr std::uint64_t kProject3dDefaultMoveLimit = 20'000'000ULL;

/// Projects `in` onto `model` (see the file comment and ADR-0073). Feed, plunge, retract and arc
/// moves are cut into straight Feed/Plunge/Retract pieces no longer than `maxSegment`, split further
/// wherever a piece would leave the surface by more than `tolerance`. `tool` selects the drop
/// surface (unused by `Project`). Invalid input gives `InvalidParameters`/`InvalidTool`/
/// `WorkLimitExceeded`/`Cancelled` and an empty result.
Project3dResult projectToolpath(const Toolpath& in, const relief::HeightField& model, const tooling::Tool& tool,
                                const Project3dParams& params, const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
