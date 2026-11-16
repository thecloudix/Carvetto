#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Drill Bank toolpath (T-0328, ADR-0106; feature-inventory "Drill Bank Toolpath"): a multi-spindle drill head
// drills several holes in one stroke. The holes of the selected closed vectors are matched to the head's spindle
// pattern; the toolpath is the path of the head (its reference spindle), written as the plain plunge / retract /
// dwell moves of Drilling (ADR-0022), one stroke per placement of the head. T-0335 (ADR-0111): every move of a stroke
// also carries the set of spindles that drill in it (`Move::bank`) and the toolpath the head (`Toolpath::bankHead`),
// so simulation drills one hole per spindle and the post writes the drill-bank blocks.

namespace carvetto::toolpath {

/// What happens when some hole fits no placement of the head (diameter or pattern).
enum class BankUnmatched {
    /// Nothing is cut: an empty toolpath with a `BankHoleUnmatched` diagnostic for every such hole.
    Refuse,
    /// The matched holes are drilled; every unmatched hole is reported (`BankHoleUnmatched`) and left undrilled.
    Skip,
};

/// Parameters of a drill bank toolpath. Lengths in mm, times in seconds. The inherited `DrillParams` fields keep
/// their meaning (depths, safe Z, peck, dwell, order of the placements, start position of the reference spindle,
/// tolerances); `cutDepth` is the depth of a spindle without its own depth.
struct DrillBankParams : DrillParams {
    DrillBankParams() { name = "Drill bank"; }

    /// Spindles on the horizontal row (along X), numbered 1..H from left to right (0..32).
    int horizontalDrills{4};
    /// Spindles on the vertical column (along Y), numbered H+1..H+V from bottom to top (0..32).
    int verticalDrills{0};
    /// Distance between neighbouring spindles of a row or column (> 0).
    double pitch{32.0};
    /// Y offset of the horizontal row from the head's central axes (the row lies at Y = rowOffset).
    double rowOffset{0.0};
    /// X offset of the vertical column from the head's central axes (the column lies at X = columnOffset).
    double columnOffset{0.0};
    /// The spindle (its number) the head's position names: the toolpath moves are this spindle's moves.
    int referenceDrill{1};
    /// Numbers of spindles that are switched off (they drill nothing). May include the reference drill.
    std::vector<int> inactiveSpindles;
    /// Diameter of each spindle's drill, by spindle number order; empty = every spindle has the tool's diameter.
    std::vector<double> spindleDiameters;
    /// Depth of each spindle's hole below `startDepth`, by spindle number order; empty = `cutDepth` for all.
    /// An entry 0 also means `cutDepth`.
    std::vector<double> spindleDepths;
    /// A hole is drilled by a spindle only when their diameters differ by at most `diameterTolerance`; off = the
    /// diameters are not compared (the vectors only mark positions).
    bool matchDiameters{false};
    /// Largest distance between a hole's centre and the spindle that drills it (>= 1e-6; less than half the
    /// distance between the two closest spindles).
    double pitchTolerance{0.1};
    /// Largest difference between a hole's diameter and its spindle's (>= 0), when `matchDiameters`.
    double diameterTolerance{0.1};
    /// A placement of the head is made only when at least this many holes are drilled by it (1..active spindles).
    int minHoles{2};
    BankUnmatched unmatched{BankUnmatched::Refuse};

    bool operator==(const DrillBankParams&) const = default;
};

/// One spindle of the head. Positions are relative to the crossing of the head's central axes.
struct DrillBankSpindle {
    int number{0};
    geometry::Vec2 offset;
    bool active{true};
    /// The spindle's drill diameter; 0 = the tool's.
    double diameter{0.0};
    /// The spindle's hole depth below the start depth; 0 = the toolpath's cut depth.
    double depth{0.0};

    bool operator==(const DrillBankSpindle&) const = default;
};

/// The spindles of the head `params` describe, in number order (row left to right, then column bottom to top).
/// Settings the lists do not cover (empty or short lists) are the defaults. Does not validate.
std::vector<DrillBankSpindle> drillBankSpindles(const DrillBankParams& params);

/// The head as a toolpath carries it (`Toolpath::bankHead`, ADR-0111): every spindle (active or not), its position
/// relative to the reference spindle, its drill diameter (`toolDiameter` when the parameters give none) and its hole
/// depth (`cutDepth` when none). Does not validate; `referenceDrill` outside the head counts as 1.
std::vector<BankSpindleSpec> drillBankHead(const DrillBankParams& params, double toolDiameter);

/// The set of spindles `Move::bank` names, for spindle numbers 1..64 (bit k-1); numbers outside are ignored.
std::uint64_t drillBankSpindleBit(int number);

/// What is wrong with the bank parameters (counts, pitch, offsets, reference drill, spindle lists, tolerances,
/// spindles closer than twice the pitch tolerance); empty when usable. Does not check the `DrillParams` part
/// (`drillParamProblem`).
std::string drillBankParamProblem(const DrillBankParams& params);

/// One hole drilled by a spindle.
struct DrillBankHole {
    /// The hole as drawn.
    geometry::Vec2 centre;
    /// Where the spindle drills it (within the pitch tolerance of `centre`).
    geometry::Vec2 drilledAt;
    /// Index of the input vector.
    std::size_t contour{0};
    int spindle{0};
    /// Depth below the start depth.
    double depth{0.0};
    /// Distance from `centre` to `drilledAt`.
    double deviation{0.0};

    bool operator==(const DrillBankHole&) const = default;
};

/// One stroke of the head.
struct DrillBankPlacement {
    /// Position of the reference spindle.
    geometry::Vec2 reference;
    /// The holes it drills, by spindle number.
    std::vector<DrillBankHole> holes;
    /// Depth of the stroke: the deepest of its holes.
    double depth{0.0};

    bool operator==(const DrillBankPlacement&) const = default;
};

/// A hole that fits no placement.
struct DrillBankUnmatched {
    geometry::Vec2 centre;
    std::size_t contour{0};
    /// True when no active spindle has the hole's diameter (with `matchDiameters`); false when no placement
    /// drills it together with enough other holes.
    bool diameter{false};

    bool operator==(const DrillBankUnmatched&) const = default;
};

/// Work done by one run, for tests that bound the cost without a clock (deterministic).
struct DrillBankStats {
    /// Placements evaluated (each looks at every active spindle).
    std::size_t placementEvaluations{0};
    /// Spatial queries of those evaluations (the work unit of the limit).
    std::size_t spindleQueries{0};
};

/// Result of `drillBankToolpath`.
struct DrillBankResult {
    Toolpath toolpath;
    /// The strokes of the head in drilling order.
    std::vector<DrillBankPlacement> placements;
    /// Holes drilled by no stroke, in vector order (with the refuse setting the toolpath is empty then).
    std::vector<DrillBankUnmatched> unmatched;
    std::vector<ToolpathDiagnostic> diagnostics;
    DrillBankStats stats;
    /// True when the toolpath has moves.
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// The diameter of the hole a closed vector marks: that of the circle of equal area.
double drillBankHoleDiameter(const geometry::Contour& contour);

/// Default work limit of one run in spindle queries (`ToolpathRunControl::workLimit` 0).
inline constexpr std::uint64_t kDrillBankWorkLimit = 30000000;

/// Computes a drill bank toolpath (ADR-0106).
///
/// - Holes: `drillHoles` of the vectors (centroid of every closed vector, shared with Drilling and with the same
///   diagnostics; the `HoleWiderThanVector` check is not made, the spindles' diameters vary).
/// - Matching: a placement of the head puts the reference spindle at some position; every active spindle then
///   drills the not yet drilled hole within `pitchTolerance` of it (nearest, then lower vector index) that its
///   diameter allows. Placements are chosen greedily, each time the one drilling most holes (ties: the lower
///   anchor hole, then the lower spindle), as long as one drills at least `minHoles`. Holes left over are
///   `unmatched` (`BankHoleUnmatched`, refused or skipped as `unmatched` says).
/// - Order: the placements are ordered with the shared `orderCuts` from `startPosition` (nearest neighbour unless
///   an order option is set).
/// - Moves: per placement the Drilling moves of the reference spindle at the placement's depth (passes of the
///   cutting data's pass depth when pecking); holes drilled off their centre are reported once (`BankHoleMoved`).
/// - Tool: as Drilling (a drill or an end mill with valid rates); the spindles' diameters are the tool's unless set.
///
/// Output is deterministic. An empty toolpath always comes with at least one diagnostic.
DrillBankResult drillBankToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                                  const std::optional<tooling::CuttingData>& cuttingData,
                                  const DrillBankParams& params, const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
