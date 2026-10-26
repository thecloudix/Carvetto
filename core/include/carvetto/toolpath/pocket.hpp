#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::toolpath {

/// How the inside of a pocket is cleared (feature-inventory §5 "Pocketing Toolpath").
enum class PocketStrategy {
    /// Rings parallel to the boundary, stepping inwards by the stepover; cut from the inside out.
    Offset,
    /// Parallel straight lines at `PocketParams::rasterAngle`, cut back and forth (zig-zag).
    Raster,
};

/// Milling direction along the pocket walls, for a spindle turning clockwise seen from above.
enum class CutDirection {
    /// Material on the right of the direction of travel: outer walls counter-clockwise, island
    /// walls clockwise.
    Climb,
    /// The opposite direction of travel.
    Conventional,
};

/// When the boundary (profile) pass of a Raster pocket is cut, at every depth level.
enum class ProfilePass { None, First, Last };

/// One tool of the multi-tool list with the values the pocket needs from its cutting data.
/// The first tool is the clearance tool; every later tool is a rest tool that only cuts what
/// the tools before it could not reach.
struct PocketTool {
    /// Tool snapshot copied into the resulting toolpath (diameter, feeds, number, name).
    ToolSnapshot tool;
    /// Lateral step between neighbouring rings or raster lines, mm; 0 < stepover <= diameter.
    double stepover{0.0};
    /// Depth of each pass, mm (> 0); `PocketParams` decides exact-step or equalised passes.
    double passDepth{0.0};

    bool operator==(const PocketTool&) const = default;
};

/// Parameters of a pocket toolpath. Lengths in mm, angles in radians.
struct PocketParams {
    /// Toolpath name; with more than one tool each toolpath is named "<name> [<tool name>]".
    std::string name{"Pocket"};
    /// Depth below the material surface (Z = 0) at which cutting starts (>= 0).
    double startDepth{0.0};
    /// Depth of the pocket below `startDepth` (> 0).
    double cutDepth{0.0};
    /// Passed to `PassPlan` for every tool (with the tool's own pass depth).
    bool maintainExactStep{true};
    /// Passed to `PassPlan::lastPassThickness` (0 = off).
    double lastPassThickness{0.0};
    /// Height of rapid moves (absolute Z).
    double safeZ{5.0};

    PocketStrategy strategy{PocketStrategy::Offset};
    CutDirection direction{CutDirection::Climb};
    /// Raster line direction measured from +X (the inventory's 0–90°, any value accepted).
    double rasterAngle{0.0};
    /// Boundary pass of a Raster pocket. Ignored by the Offset strategy, whose outermost ring is
    /// the boundary pass and is always cut last (ADR-0017).
    ProfilePass profilePass{ProfilePass::Last};
    /// Material left on every wall and island (>= 0) ("Pocket Allowance").
    double allowance{0.0};

    /// Enter the material with a zig-zag ramp along the path instead of a vertical plunge.
    bool rampPlunge{false};
    /// Ramp length (one leg of the zig-zag), mm.
    double rampDistance{10.0};

    /// Geometric tolerance. `chordal` governs polygonisation and the safety margin (ADR-0017).
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};
};

/// What a planned pocket path is for.
enum class PocketPathRole {
    Ring,     ///< An Offset-strategy ring (the outermost one is the boundary).
    Raster,   ///< One Raster-strategy line piece.
    Profile,  ///< A Raster-strategy boundary pass.
};

/// One planned path: tool-centre positions at constant Z, in cutting order and direction.
struct PocketPath {
    geometry::Polyline path;
    PocketPathRole role{PocketPathRole::Ring};
    /// Index of the cluster the path belongs to; every cluster is cut through all its depth
    /// levels before the next one starts (a connected region for the clearance tool, a
    /// connected piece of rest material for a rest tool).
    std::size_t cluster{0};
};

/// The 2D plan of one tool, exposed for inspection and tests.
struct PocketToolPlan {
    /// Region the tool centre may visit (the pocket shrunk by radius + allowance + margin),
    /// polygonal: outers counter-clockwise, holes clockwise.
    std::vector<geometry::Contour> centreRegion;
    /// Material the earlier tools left (rest tools only; empty for the clearance tool).
    std::vector<geometry::Contour> restMaterial;
    /// The paths cut at every depth level, in order (clusters in order, paths within a cluster
    /// in order).
    std::vector<PocketPath> paths;
    /// Z levels from `PassPlan`, shallowest first.
    std::vector<double> levels;
};

/// Result of `pocketToolpath`: one toolpath and one plan per input tool, in tool order (a tool
/// that has nothing to cut gets an empty toolpath), plus human-readable notes about anything
/// skipped (open contours, a tool too large for the pocket, invalid parameters, ...).
struct PocketResult {
    std::vector<Toolpath> toolpaths;
    std::vector<PocketToolPlan> plans;
    std::vector<std::string> messages;
};

/// Computes a 2D pocket (feature-inventory §5 "Pocketing Toolpath").
///
/// The pocket region is the even-odd union of the closed `contours` (nested contours are
/// islands, contours inside islands are pockets again); open contours are ignored with a
/// message. For each tool the region is shrunk by tool radius + allowance + a safety margin of
/// `tolerance.chordal` (Clipper2 offsets, ADR-0006) to get the centre region; the Offset strategy
/// cuts the rings offset(centre, −k·stepover) inside-out (outermost ring = boundary, last), the
/// Raster strategy clips parallel lines spaced ≤ stepover to the centre region, links them
/// zig-zag and adds the boundary pass first/last/none. Rest tools cut their own pattern clipped
/// to where their disc touches the rest material: the pocket (less allowance) minus the area the
/// earlier tools swept. Links between paths are straight feeds when they stay inside the centre
/// region (and, inside one depth level, are no longer than two tool diameters); otherwise the
/// tool retracts to safe Z. Every depth level of a cluster is cut before the next cluster, and
/// the clearance tool goes from one level to the next without retracting when the way back to the
/// level's first path lies inside the centre region. Entry into the material is a vertical
/// plunge or, with `rampPlunge`, a zig-zag ramp along the path (`appendPathRamp`).
///
/// Every move is a straight line (polygonal geometry); output is deterministic. See ADR-0017.
PocketResult pocketToolpath(const std::vector<geometry::Contour>& contours,
                            const std::vector<PocketTool>& tools, const PocketParams& params);

}  // namespace carvetto::toolpath
