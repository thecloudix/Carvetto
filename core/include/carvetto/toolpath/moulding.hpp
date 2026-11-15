#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/rough3d.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Moulding toolpath (T-0307, ADR-0086; feature-inventory "Moulding Toolpath"): a moulding cross-section
// (an open "profile" vector) is swept along drive rails. The profile is drawn in a plane across the rail:
// its X is the distance to the LEFT of the direction of travel of the rail (negative = right), its Y the
// height of the moulding (machine Z = Y + `zOffset`). The moulding surface is the extrusion of that
// section along each rail: its height at a point is the profile height at the point's signed distance
// from the rail. Each tool pass follows the rail offset by a distance u (an offset curve, constant Z);
// its Z is the exact drop height of the tool on the section at u (the tool's exact profile, so the pass
// and every pass position in between never cut the section below it): max over t of H(t) - h(|t - u|).

namespace carvetto::toolpath {

enum class MouldingMode {
    Finish,  ///< Passes at the stepover over the whole section (flat regions optionally skipped).
    Rough,   ///< Z-level clearing down to the section plus `allowance`.
};

/// Toolpath Position: which part of the drawn section lies on the rail (offset 0).
enum class MouldingPosition {
    AsDrawn,  ///< The profile's own X = 0 (the origin of the drawing) is on the rail.
    Start,    ///< The first point of the profile is on the rail; the section extends to its left.
    Centre,   ///< The middle of the profile's X extent is on the rail.
    End,      ///< The last point of the profile is on the rail; the section extends to its right.
};

struct MouldingParams {
    std::string name{"Moulding"};
    MouldingMode mode{MouldingMode::Finish};
    MouldingPosition position{MouldingPosition::AsDrawn};

    /// Open vector: points (x across, y height) with non-decreasing x (single valued; equal x makes a
    /// vertical step). Needs at least two points and a positive x extent.
    geometry::Polyline profile;
    /// Walk the profile from its last point to its first (mirrors the section across the rail).
    bool reverseProfile{false};
    /// Drive rails (open or closed polylines); `reverseRails` flips each one's direction (left <-> right).
    std::vector<geometry::Polyline> rails;
    bool reverseRails{false};

    /// Added to every profile height to give machine Z.
    double zOffset{0.0};
    /// Top of the material (Rough clears from here) and the lowest Z the tool tip may reach.
    double stockTop{0.0};
    double stockBottom{-20.0};
    /// Height of rapid moves; must be above `stockTop`.
    double safeZ{5.0};
    /// Rough only: material left on the section (>= 0).
    double allowance{0.0};

    /// Extends the machined band beyond both ends of the section by this distance (>= 0), the section
    /// continuing flat at its end heights. `autoBoundaryOffset` uses the tool radius instead.
    double boundaryOffset{0.0};
    bool autoBoundaryOffset{false};

    /// Zig-zag alternates the travel direction of consecutive passes, one-way keeps the rail direction.
    RasterTravel travel{RasterTravel::ZigZag};
    /// Climb: passes step towards the right of the travel direction (the next pass is uncut material on
    /// the right), conventional towards the left. `reverseSteps` flips the order.
    CutDirection direction{CutDirection::Climb};
    bool reverseSteps{false};

    /// Finish: the plan step between passes follows the section, so the step measured along the profile
    /// stays at the stepover (smaller across steep parts).
    bool varyStepover{false};
    /// Finish: no passes strictly inside flat regions. Rough: flat regions are cleared only if
    /// `machineFlatRegions`. A flat region is a run of profile segments no steeper than `flatAngle` that
    /// is at least one stepover wide.
    bool skipFlatRegions{false};
    bool machineFlatRegions{true};
    double flatAngle{0.017453292519943295};  // 1 degree
    /// Corners of the rails: sharp (mitred) instead of round on the outside of a turn.
    bool sharpCorners{false};

    /// Rough: enter each level along the pass instead of plunging.
    bool rampPlunge{false};
    double rampAngle{0.17453292519943295};  // 10 degrees
    /// Rough: first clear with a larger tool (the "[Clear]" toolpath, which also clears flat regions),
    /// then rough only where the clearance tool left more than `restTolerance` above the main tool's
    /// reach.
    std::optional<Rough3dTool> clearTool;
    double restTolerance{0.01};

    /// Chordal tolerance of the offset curves (> 0).
    double tolerance{0.01};

    bool operator==(const MouldingParams&) const = default;
};

/// Result: with a clearance tool, `toolpaths[0]` is the "[Clear]" one and `toolpaths[1]` the main one;
/// otherwise a single toolpath. A toolpath with nothing to cut is empty.
using MouldingResult = Rough3dResult;

/// Default caps of one run: tool-drop evaluations and output moves.
inline constexpr std::uint64_t kMouldingDefaultWorkLimit = 400'000'000ULL;
inline constexpr std::uint64_t kMouldingDefaultMoveLimit = 20'000'000ULL;

/// The tip height at which `tool` (a drop with the tool's exact profile) lies on the section when its
/// centre is at offset `u` from the rail, i.e. max over t of H(t) - h(|t - u|), in the section's own
/// heights (no `zOffset`); -infinity when no section point is within the tool's reach. `profile` is the
/// section as (x, y) points with non-decreasing x. Exposed for tests.
double mouldingDropHeight(const std::vector<geometry::Vec2>& profile, const simulate::ToolShape& tool, double u);

/// Plans the moulding toolpath(s) (see the file comment and ADR-0086). Invalid input gives a
/// diagnostic (`InvalidParameters`, `InvalidTool`, `WorkLimitExceeded`, `Cancelled`, `NoPasses`) and no moves.
MouldingResult mouldingToolpath(const Rough3dTool& tool, const MouldingParams& params,
                                const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
