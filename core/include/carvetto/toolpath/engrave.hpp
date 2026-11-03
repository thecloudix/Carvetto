#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Quick Engrave toolpath (T-0151, ADR-0024; feature-inventory §5 "Quick Engraving Toolpath"):
// mark the selected vectors at a shallow depth, either by following them with the tool centre
// (Outline) or by filling the closed regions they enclose with Pocket's offset or raster pattern
// (Fill), in a number of equal passes.

namespace carvetto::toolpath {

/// How the vectors are engraved.
enum class EngraveStrategy {
    /// The tool centre follows every vector (open or closed) at the engrave depth.
    Outline,
    /// The closed regions are cleared at the engrave depth with Pocket's pattern (`fillPattern`).
    Fill,
};

/// Parameters of a quick engrave toolpath. Lengths in mm, angles in radians.
struct EngraveParams {
    std::string name{"Quick Engrave"};
    /// Engrave depth below the material surface (Z = 0) (> 0).
    double depth{0.0};
    /// Number of equal passes to reach the depth (1 to 1000).
    int passes{1};
    /// Absolute Z of the moves between cuts (above the surface).
    double safeZ{5.0};

    EngraveStrategy strategy{EngraveStrategy::Outline};
    /// Fill pattern (Fill only): Pocket's Offset rings or Raster lines.
    PocketStrategy fillPattern{PocketStrategy::Offset};
    /// Lateral step of the fill pattern (Fill only), mm; 0 = the cutting data's stepover. Must not
    /// exceed the width the tool cuts at the engrave depth (`tooling::cuttingWidthAtDepth`).
    double stepover{0.0};
    /// Raster line direction from +X (Fill with the Raster pattern).
    double rasterAngle{0.0};

    /// Tool position before the first cut (XY), for the nearest-first order of Outline cuts.
    geometry::Vec2 startPosition{0.0, 0.0};
    /// Geometric tolerance: `chordal` flattens Beziers (Outline) and is the Fill safety margin
    /// as in Pocket.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const EngraveParams&) const = default;
};

/// Result of `engraveToolpath`.
struct EngraveResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// True when the toolpath has moves.
    bool ok() const { return !toolpath.isEmpty(); }
    /// True when some diagnostic has the given issue.
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// True for the tool types quick engraving accepts (ADR-0024): V-bit, engraving tool, end mill,
/// ball nose and diamond drag.
bool engraveToolAllowed(tooling::ToolType type);

/// The Z level of every pass: `passes` equal steps down to −`depth`, the last exactly −`depth`.
/// Empty when the depth is not > 0 or `passes` is not in 1..1000.
std::vector<double> engraveLevels(double depth, int passes);

/// Computes a quick engrave toolpath.
///
/// Outline: the tool centre follows each vector at every pass level (`engraveLevels`). A closed
/// vector is cut round once per level, stepping down at its start point; an open vector is cut
/// back and forth, stepping down at whichever end the tool reached. Lines stay lines and arcs
/// arcs; Beziers are flattened within `tolerance.chordal`. Vectors are ordered with the shared
/// `orderCuts` (nearest first from `startPosition`; inner closed vectors before the ones round
/// them). Each vector: rapid at safe Z above its start, plunge, cut, retract to safe Z. Vectors
/// without spans or length (`EmptyContour`) or with gaps (`DiscontinuousContour`) are skipped.
///
/// Fill: the closed vectors (even-odd: nested vectors are islands) are cleared by
/// `pocketToolpath` with one tool, the given pattern and stepover, cut depth = engrave depth,
/// pass depth = depth / passes and a boundary pass last; so the tool keeps the width it cuts at
/// the engrave depth (a V-bit's cone at that depth) inside the vectors, exactly as Pocket does.
/// Open vectors are ignored (`OpenContourIgnored`).
///
/// Tool: one of `engraveToolAllowed` (`InvalidTool` otherwise) passing `tooling::validate`, with
/// feed and plunge rates > 0 and a cutting width > 0 at the depth. `cuttingData` empty = the
/// tool's default data (`effectiveCuttingData`). Every rapid is at safe Z. Output is
/// deterministic; an empty toolpath always comes with at least one diagnostic.
EngraveResult engraveToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData, const EngraveParams& params);

}  // namespace carvetto::toolpath
