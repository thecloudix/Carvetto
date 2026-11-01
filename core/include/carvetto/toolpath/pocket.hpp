#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::toolpath {

/// How the inside of a pocket is cleared (feature-inventory §5 "Pocketing Toolpath").
enum class PocketStrategy {
    /// Rings parallel to the boundary, stepping inwards by the stepover; cut from the inside out.
    Offset,
    /// Parallel straight lines at `PocketParams::rasterAngle`, cut back and forth (zig-zag).
    Raster,
};

/// When the boundary (profile) pass is cut, at every depth level (owner decision 2026-10-26: for
/// both strategies). For Offset, the boundary pass is the outermost ring (outer wall and island
/// walls); None leaves it as an ordinary clearing ring, which the inside-out order cuts last.
enum class ProfilePass { None, First, Last };

/// One tool of the multi-tool list: a tool-database tool and the cutting data to use with it.
/// The first tool is the clearance tool; every later tool is a rest tool that only cuts what
/// the tools before it could not reach. The pocket takes from it (`passTool`): the cutting
/// radius (half `tooling::cuttingWidthAtDepth` at the bottom of the pocket: D/2 for end mills),
/// the stepover (0 < stepover <= that width) and pass depth (> 0) of the cutting data, and the
/// snapshot (name, diameter, number, feeds) copied into the toolpath.
struct PocketTool {
    tooling::Tool tool;
    /// Feeds, speeds, tool number, stepover and pass depth; empty = the tool's default data
    /// (`effectiveCuttingData`), the same rule as `ProfileParams::cuttingData`.
    std::optional<tooling::CuttingData> cuttingData;

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
    /// levels before the next one starts (a connected part of the centre region for the
    /// clearance tool, a connected rest area for a rest tool).
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
/// that has nothing to cut gets an empty toolpath), plus diagnostics about anything skipped
/// (open contours, a tool too large for the pocket, invalid parameters, ...) in the shared
/// `ToolpathDiagnostic` type (`tool` = index in the tool list when the issue concerns one tool).
struct PocketResult {
    std::vector<Toolpath> toolpaths;
    std::vector<PocketToolPlan> plans;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// True when some toolpath has moves.
    bool ok() const;
    /// True when some diagnostic has the given issue.
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// Computes a 2D pocket (feature-inventory §5 "Pocketing Toolpath").
///
/// The pocket region is the even-odd union of the closed `contours` (nested contours are
/// islands, contours inside islands are pockets again); open contours are ignored with a
/// diagnostic. Each tool is converted with `passTool` (radius, stepover, pass depth, snapshot).
/// For each tool the region is shrunk by its radius + allowance + a safety margin of
/// `tolerance.chordal` (Clipper2 offsets, ADR-0006) to get the centre region; the Offset strategy
/// cuts the rings offset(centre, −k·stepover) inside-out (with the boundary ring first, last or
/// as an ordinary ring per `profilePass`), the Raster strategy clips parallel lines spaced
/// ≤ stepover to the centre region, links them zig-zag and adds the boundary pass first/last/none.
/// Every point the tool can reach is cut for any stepover ≤ diameter (residue thinner than
/// min(chordal, 0.01 mm) aside). Rest tools build their own pattern only over their rest areas:
/// the centres in the centre region whose disc overlaps the rest material (the pocket less
/// allowance, minus the area the earlier tools swept); each connected rest area is a cluster.
/// Every depth level of a cluster is cut before the next cluster. A short link (≤ two tool
/// diameters) that stays inside the centre region is a feed at depth; any other way within a
/// cluster lifts to the floor of the previous level (the surface for the first level), which is
/// already cleared there, and feeds along a straight segment or a route along the rings of the
/// offset tree (Offset) or of the part itself (Raster, rest areas) inside the region (Raster
/// without a boundary pass leaves wall scallops, so it never travels at depth); between rest
/// clusters in one part of the centre region the tool lifts to the top of the cut and travels
/// inside that part. Only when there is no such way does the tool retract to safe Z, so a
/// connected pocket retracts once whatever its islands. Entry into the material is a vertical
/// plunge or, with `rampPlunge`, a zig-zag ramp along the path (`appendPathRamp`, the shared ramp
/// module). Parameters that would make the work explode (pass depth, stepover or chordal
/// tolerance too small for the pocket) are refused with a diagnostic and an empty toolpath.
///
/// Every move is a straight line (polygonal geometry); output is deterministic. See ADR-0017
/// and ADR-0018.
PocketResult pocketToolpath(const std::vector<geometry::Contour>& contours,
                            const std::vector<PocketTool>& tools, const PocketParams& params);

/// The region the centre of a tool of `toolDiameter` may visit in the pocket of `contours`, exactly
/// as `pocketToolpath` computes it (`PocketToolPlan::centreRegion`): the even-odd region of the
/// closed contours inset by radius + `params.allowance` + `params.tolerance.chordal`, polygonal,
/// outers counter-clockwise, holes clockwise. Empty when the tool fits nowhere. Vertices that
/// Clipper2 leaves too close to a wall are pulled in (every edge so changed is checked exactly)
/// or the part is inset further (ADR-0017 §4); a part that still cannot be kept within the
/// safety margin, or is inset by more than the chordal tolerance to keep it, is reported in
/// `diagnostics`, never dropped or trimmed silently. Open contours are ignored and reported, and
/// input over the vertex cap (ADR-0017 §12) is refused with a message, as in `pocketToolpath`.
/// Cheap compared to a whole toolpath: useful to preview where a tool fits.
std::vector<geometry::Contour> pocketCentreRegion(const std::vector<geometry::Contour>& contours,
                                                  double toolDiameter, const PocketParams& params,
                                                  std::vector<ToolpathDiagnostic>* diagnostics = nullptr);

}  // namespace carvetto::toolpath
