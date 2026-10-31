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

/// When the boundary (profile) pass is cut, at every depth level (owner decision 2026-10-26: for
/// both strategies). For Offset, the boundary pass is the outermost ring (outer wall and island
/// walls); None leaves it as an ordinary clearing ring, which the inside-out order cuts last.
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
    /// Boundary pass: First (before clearing), Last (after) or None (Raster: no boundary pass;
    /// Offset: the outermost ring stays an ordinary clearing ring). See `ProfilePass`.
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
    Ring,     ///< An Offset-strategy clearing ring.
    Raster,   ///< One Raster-strategy line piece.
    Profile,  ///< A boundary pass (Raster boundary, or Offset outermost ring with First/Last).
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
/// cuts the rings offset(centre, −k·stepover) inside-out (with the boundary ring first, last or
/// as an ordinary ring per `profilePass`), the Raster strategy clips parallel lines spaced
/// ≤ stepover to the centre region, links them zig-zag and adds the boundary pass first/last/none.
/// Every point the tool can reach is cut for any stepover ≤ diameter (residue thinner than
/// min(chordal, 0.01 mm) aside). Rest tools cut their own pattern clipped to where their disc
/// overlaps the rest material: the pocket (less allowance) minus the area the earlier tools swept.
/// Every depth level of a cluster is cut before the next cluster. A short link (≤ two tool
/// diameters) that stays inside the centre region is a feed at depth; any other way within a
/// cluster lifts to the floor of the previous level (the surface for the first level), which is
/// already cleared there, and feeds along a straight segment or a route through the offset ring
/// tree inside the centre region (Raster without a boundary pass leaves wall scallops, so it never
/// travels); only when there is no such way does the tool retract to safe
/// Z. Entry into the material is a vertical plunge or, with `rampPlunge`, a zig-zag ramp along the
/// path (`appendPathRamp`). Parameters that would make the work explode (pass depth, stepover or
/// chordal tolerance too small for the pocket) are refused with a message and an empty toolpath.
///
/// Every move is a straight line (polygonal geometry); output is deterministic. See ADR-0017.
PocketResult pocketToolpath(const std::vector<geometry::Contour>& contours,
                            const std::vector<PocketTool>& tools, const PocketParams& params);

/// The region the centre of a tool of `toolDiameter` may visit in the pocket of `contours`, exactly
/// as `pocketToolpath` computes it (`PocketToolPlan::centreRegion`): the even-odd region of the
/// closed contours inset by radius + `params.allowance` + `params.tolerance.chordal`, polygonal,
/// outers counter-clockwise, holes clockwise. Empty when the tool fits nowhere. Vertices that
/// Clipper2 leaves too close to a wall are pulled in or the part is inset slightly further
/// (ADR-0017 §4); a part that still cannot be kept within the safety margin is reported in
/// `messages`, never dropped silently. Open contours are ignored and reported, and input over the
/// vertex cap (ADR-0017 §12) is refused with a message, as in `pocketToolpath`. Cheap compared to
/// a whole toolpath: useful to preview where a tool fits.
std::vector<geometry::Contour> pocketCentreRegion(const std::vector<geometry::Contour>& contours,
                                                  double toolDiameter, const PocketParams& params,
                                                  std::vector<std::string>* messages = nullptr);

}  // namespace carvetto::toolpath
