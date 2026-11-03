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
#include "carvetto/toolpath/toolpath.hpp"

// Drilling toolpath (T-0150, ADR-0022; feature-inventory §5 "Drilling Toolpath"): one hole at the
// centre of every selected closed vector, plunged straight down, optionally pecked, with an
// optional dwell at the bottom of each pass. Written as plain plunge / retract / dwell moves, so
// every post can output it (no canned cycles).

namespace carvetto::toolpath {

/// Where the tool goes between the passes of a peck-drilled hole.
enum class PeckRetract {
    /// Up to the cutting start (`DrillParams::startDepth` below the surface) plus the retract gap,
    /// clearing the chips out of the hole after every pass.
    AboveStart,
    /// Up by the retract gap above the depth the pass just reached (chip breaking; the tool stays
    /// in the hole).
    AbovePreviousPass,
};

/// Parameters of a drilling toolpath. Lengths in mm, times in seconds.
struct DrillParams {
    std::string name{"Drill"};
    /// Depth below the material surface (Z = 0) at which drilling starts (>= 0).
    double startDepth{0.0};
    /// Depth of the holes below `startDepth` (> 0).
    double cutDepth{0.0};
    /// Absolute Z of the moves between holes (above the cutting start).
    double safeZ{5.0};

    /// Peck drilling: drill each hole in passes of the cutting data's pass depth, retracting
    /// between passes as `peckRetract` says (feature-inventory "Use Peck Drilling").
    bool peck{false};
    PeckRetract peckRetract{PeckRetract::AboveStart};
    /// Retract gap R (> 0 when pecking): how far above the cutting start (AboveStart) or above the
    /// depth just reached (AbovePreviousPass) the tool retracts between passes.
    double retractGap{1.0};
    /// Dwell at the bottom of each pass, in seconds; 0 = no dwell (>= 0).
    double dwellTime{0.0};

    /// Order of the holes (shared with Profile); none set = nearest neighbour from `startPosition`.
    OrderOptions order;
    /// Tool position before the first hole (XY), for nearest ordering.
    geometry::Vec2 startPosition{0.0, 0.0};
    /// `distance`: holes closer than this are drilled once; `chordal`: point-in-vector test of Beziers.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const DrillParams&) const = default;
};

/// One drilled hole.
struct DrillHole {
    /// Centre of the hole (XY).
    geometry::Vec2 centre;
    /// Index of the input vector it was made from.
    std::size_t contour{0};

    bool operator==(const DrillHole&) const = default;
};

/// Result of `drillToolpath`.
struct DrillResult {
    Toolpath toolpath;
    /// The holes in drilling order.
    std::vector<DrillHole> holes;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// True when the toolpath has moves.
    bool ok() const { return !toolpath.isEmpty(); }
    /// True when some diagnostic has the given issue.
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// The centre of a closed vector as `drillToolpath` drills it (ADR-0022): the area centroid of
/// the region the contour encloses (Green's theorem; exact for lines and arcs, Beziers integrated
/// exactly by Gauss–Legendre quadrature), so circles of any construction, regular polygons, slots
/// and other symmetric shapes give their geometric centre, and the result turns with the shape.
/// Empty when the contour is open, empty, or encloses no area (|area| <= tol.distance²).
std::optional<geometry::Vec2> drillCentre(const geometry::Contour& contour, const geometry::Tolerance& tol);

/// True for the tool types a drilling toolpath accepts: `Drill` and `EndMill` (ADR-0022).
bool drillToolAllowed(tooling::ToolType type);

/// Computes a drilling toolpath: one hole at `drillCentre` of every closed vector of `contours`.
///
/// - Open vectors are ignored (`OpenContourIgnored`); vectors without spans (`EmptyContour`),
///   with gaps (`DiscontinuousContour`) or without area (`NoRegion`), and vectors whose centroid
///   is not inside them (`CentreOutsideVector`, e.g. a C shape) are skipped and named.
/// - A hole wider than the vector round its centre (the tool radius exceeds the distance from
///   the centre to the vector, e.g. a small marker circle) is drilled and reported
///   (`HoleWiderThanVector`). Holes within `tolerance.distance` of an earlier hole are drilled
///   once (`DuplicateHole`).
/// - Holes are ordered with the shared `orderCuts` (selection, left-to-right, bottom-to-top,
///   grid, or nearest from `startPosition`).
/// - Each hole: rapid at safe Z above it, plunge to each pass level at the plunge rate (a single
///   pass to the bottom without pecking; passes of the cutting data's pass depth with it, the last
///   one taking the remainder), a dwell of `dwellTime` at the bottom of every pass when > 0, a
///   retract between passes (see `PeckRetract`; never above safe Z), and a retract to safe Z.
///   Every rapid is at safe Z.
/// - Tool: a drill or an end mill (`InvalidTool` otherwise) with a valid geometry, plunge and
///   feed rates > 0 and, when pecking, a pass depth > 0. `cuttingData` empty = the tool's
///   default data (`effectiveCuttingData`).
///
/// Output is deterministic. An empty toolpath always comes with at least one diagnostic.
DrillResult drillToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                          const std::optional<tooling::CuttingData>& cuttingData, const DrillParams& params);

}  // namespace carvetto::toolpath
