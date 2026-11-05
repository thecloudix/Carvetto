#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// V-Carve toolpath (T-0153, ADR-0025; feature-inventory "V-Carve Toolpath"): carve closed regions
// with a V-bit (or a ball nose) whose depth follows the region's medial axis, so the tool touches
// both walls everywhere; optionally limited to a flat depth, the flat-bottomed areas then being
// cleared with Pocket's machinery by clearance tools (or by the carving tool itself).

namespace carvetto::toolpath {

/// Parameters of a V-carve toolpath. Lengths in mm, angles in radians.
struct VCarvingParams {
    /// Toolpath name; with clearance tools every toolpath is named "<name> [<tool name>]".
    std::string name{"V-Carve"};
    /// Depth below the material surface (Z = 0) of the plane the carve is computed from (>= 0):
    /// 0 carves from the surface; a pocket's depth carves into the floor of that pocket (the
    /// material above the plane is assumed gone over the regions).
    double startDepth{0.0};
    /// Depth below the start plane at which carving stops and the bottom turns flat (> 0), or 0
    /// for none (the carve then goes as deep as the regions and the tool allow).
    double flatDepth{0.0};
    /// Absolute Z of the moves between cuts (above the material surface, > 0).
    double safeZ{5.0};

    /// How clearance tools clear the flat areas (Pocket's Offset rings or Raster lines).
    PocketStrategy clearanceStrategy{PocketStrategy::Offset};
    /// Cut direction of the flat-area clearance (every clearance tool).
    CutDirection clearanceDirection{CutDirection::Climb};
    /// Raster direction from +X for the Raster clearance strategy.
    double clearanceRasterAngle{0.0};
    /// Enter the material of the flat-area clearance with a zig-zag ramp instead of a plunge.
    bool rampPlunge{false};
    /// Ramp length (one leg of the zig-zag), mm.
    double rampDistance{10.0};

    /// Order of the regions (connected parts of the region: an outer vector with its holes,
    /// e.g. a letter and its counters).
    /// None set = nearest region first from `startPosition`.
    OrderOptions order;
    /// Tool position before the first cut (XY).
    geometry::Vec2 startPosition{0.0, 0.0};
    /// Geometric tolerance: `chordal` (0.0003 to 1 mm) is the lateral accuracy of the carve; the
    /// medial axis and the flat-area Pocket runs use a third of it (ADR-0025 §1).
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const VCarvingParams&) const = default;
};

/// Result of `vCarvingToolpath`.
struct VCarvingResult {
    /// Clearance toolpaths, one per clearance tool in list order (empty moves when a tool had
    /// nothing to clear, or when there is no flat area). Machined before `toolpath`.
    std::vector<Toolpath> clearance;
    /// The carving tool's toolpath (V-bit or ball nose): flat-area clearing when it clears the
    /// flat areas itself, then the carve.
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// Number of separate regions (outer boundaries with their holes) found.
    std::size_t regions{0};

    /// True when the carving toolpath has moves.
    bool ok() const { return !toolpath.isEmpty(); }
    /// True when some diagnostic has the given issue.
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
    /// Every toolpath with moves in machining order: the clearance tools, then the carving tool.
    std::vector<Toolpath> toolpaths() const;
};

/// True for the carving tools a V-carve accepts (ADR-0025): V-bit and ball nose.
bool vCarvingToolAllowed(tooling::ToolType type);

/// True for the clearance tools a V-carve accepts (ADR-0025): end mills (flat-bottomed).
bool vCarvingClearanceToolAllowed(tooling::ToolType type);

/// Depth below the start plane at which the carving tool reaches its full width (V-bit: the top
/// of its cone, (D/2) / tan(angle/2); ball nose: its ball radius D/2). 0 for other tool types.
double vCarvingFullWidthDepth(const tooling::Tool& tool);

/// Depth below the start plane at which the carving tool cuts a footprint of radius `r` there:
/// V-bit r / tan(angle/2), ball nose D/2 − sqrt((D/2)² − r²); the inverse of half of
/// `tooling::cuttingWidthAtDepth`. Clamped to [0, `vCarvingFullWidthDepth`].
double vCarvingDepthForRadius(const tooling::Tool& tool, double r);

/// Computes a V-carve toolpath (feature-inventory "V-Carve Toolpath", ADR-0025).
///
/// Regions: the even-odd region of the closed `contours` (nested vectors are islands). Vectors
/// that cross, touch or overlap (each other or themselves) are first resolved into their
/// even-odd region with `geometry::unionContours`, polygonised within chordal/8, as Pocket does
/// (`CrossingVectorsResolved`; the carve margin of those parts grows by chordal/8; a hole or
/// sliver thinner than the distance tolerance does not survive this and is cut through). The region
/// is then split into parts, an outer boundary with the boundaries directly inside it (a letter
/// and its counters; a frame round text is a part of its own), each carved on its own. Open
/// vectors are ignored (`OpenContourIgnored`); vectors without spans, with gaps or without area
/// are skipped (`EmptyContour`, `DiscontinuousContour`, `NoRegion`).
///
/// Carve: `geometry::medialAxis` gives, along the skeleton of each group, the radius r of the
/// largest inscribed circle. The tool follows every skeleton branch at the depth where its
/// footprint at the start plane has radius r − margin (V-bit: depth = r / tan(angle/2) less the
/// margin), so it touches both walls within the chordal tolerance and never crosses one;
/// the depth is limited to the flat depth (when set) and to the tool's full-width depth.
/// Accuracy: the medial axis runs at chordal/3 and the margin is chordal/6, so every point of the
/// region whose depth is above the limit is carved to within the chordal tolerance of its depth
/// as a footprint radius (ADR-0025 §1).
/// Branches are joined into as few continuous cuts as the skeleton allows (each: rapid at safe
/// Z, plunge, cut along the skeleton with Z following the depth, retract). Groups are ordered
/// with the shared `orderCuts` and `order`; the cuts of a group nearest first.
///
/// Flat areas: where the depth is limited (flat depth, or the tool's full width reached:
/// `CarveDepthLimited`), the areas wider than the tool's footprint at that depth are flat
/// bottomed. With clearance tools, `pocketToolpath` clears them (the tools in list order, later
/// ones as rest tools; allowance = the carving tool's footprint radius at the flat depth, so
/// the end mills keep off the carved walls) and the carving tool cuts the walls of the flat
/// areas along `pocketCentreRegion` at the flat depth; the carving tool also follows the end
/// mills as Pocket's last rest tool, clearing what they could not reach, its stepover limited
/// to the width it cuts at the flat bottom (`StepoverLimited` when that cut a wider stepover
/// down and it had rest material: ridges up to the depth limit). Pocket runs region by
/// region. Without
/// clearance tools the carving tool
/// clears the flat areas itself with `pocketToolpath` (its cutting data's stepover and pass
/// depth), the boundary pass being the same walls. Pocket's diagnostics are passed on; when it
/// refuses (invalid tool, parameters or work limit) the whole V-carve is refused.
///
/// The medial axis must be accurate: when it reports `accuracyCapped` (sample or work budget, or
/// coordinates too far from the origin for the tolerance) or ignores a boundary that encloses
/// area, for any part, nothing is produced and `MedialAxisCapped` names the part's vectors.
///
/// Tools: `tool` must be a V-bit or ball nose (`vCarvingToolAllowed`) passing `tooling::validate`
/// with feed and plunge rates > 0; clearance tools must be end mills. `cuttingData` empty = the
/// tool's default data. Every rapid is at safe Z; output is deterministic; an empty result
/// always comes with at least one diagnostic.
VCarvingResult vCarvingToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                                const std::optional<tooling::CuttingData>& cuttingData,
                                const std::vector<PocketTool>& clearanceTools, const VCarvingParams& params);

}  // namespace carvetto::toolpath
