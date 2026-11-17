#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Rounding toolpath (T-0322, ADR-0115; feature-inventory "Wrapping submenu"): turns the blank of a rotary job
// down to a given radius. Like every toolpath it is computed flat in the unwrapped design space (ADR-0098:
// along the axis, and round the circumference, with Z the machine height) and wrapped by the post (ADR-0103).
// It needs no vectors: the cylinder is the work piece.

namespace carvetto::toolpath {

/// How the surface is swept. Radial: strokes along the axis at evenly spaced angles, the tool staying down
/// between neighbouring strokes (a zig-zag). Raster: one full turn per stroke, stepping along the axis, the
/// tool lifted between strokes (all strokes run the same way). OptimizedRaster: the same strokes, but the
/// direction alternates and the tool stays down between neighbouring turns.
enum class RoundingMethod { Radial, Raster, OptimizedRaster };

/// The blank before rounding: a cylinder of the job's radius, or the square bar inscribed in it (its corners
/// touch the cylinder; side = radius * sqrt(2), faces centred on angles 0, 90, 180 and 270 degrees).
enum class RoundingBlank { Round, Square };

/// Parameters of a Rounding toolpath. Lengths in mm. The circumference runs from `circStart` for one full
/// turn of `blankRadius` (2 pi r).
struct RoundingParams {
    std::string name{"Rounding"};
    RoundingMethod method{RoundingMethod::OptimizedRaster};
    RoundingBlank blank{RoundingBlank::Round};
    /// The cylinder lies along X (the circumference is Y) when true, along Y when false.
    bool alongX{true};
    /// Radius of the cylinder (the job's radius), > 0.
    double blankRadius{25.0};
    /// Radius to turn down to, > 0.
    double targetRadius{20.0};
    /// Rest machining: the blank has already been turned to this radius (by an earlier Rounding); 0 = not.
    double startRadius{0.0};
    /// Radial material left on the finished surface (>= 0).
    double allowance{0.0};
    /// Machine Z of the cylinder surface (the job's rotary surface Z); the axis is at surfaceZ - blankRadius.
    double surfaceZ{0.0};
    /// Extent along the axis the tool centre covers (alongEnd > alongStart).
    double alongStart{0.0};
    double alongEnd{100.0};
    /// Circumference coordinate where one turn starts.
    double circStart{0.0};
    /// Distance between neighbouring strokes (mm; along the axis for the rasters, round the circumference at the
    /// blank radius for Radial); 0 = the tool's cutting data. Wider than the tool is limited to its diameter.
    double stepover{0.0};
    /// Radial depth of a pass (mm); 0 = the tool's cutting data.
    double passDepth{0.0};
    /// Absolute Z the tool travels at between strokes: above the blank.
    double safeZ{30.0};

    bool operator==(const RoundingParams&) const = default;
};

struct RoundingResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// Radius of each pass, outermost first (the last is targetRadius + allowance).
    std::vector<double> radii;
    /// Strokes cut over all passes.
    std::size_t strokes{0};
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// End mill (flat) and ball nose tools.
bool roundingToolAllowed(tooling::ToolType type);

/// Radii of the passes from `top` down to `bottom` in equal steps of at most `passDepth`: the last is `bottom`,
/// the first `top - (top - bottom) / n`. Empty when `top <= bottom`, or `passDepth` is not positive.
std::vector<double> roundingPassRadii(double top, double bottom, double passDepth);

/// The stretches of circumference `[c0, c0 + 2 pi radius]` where the blank is higher than radius `rho`, as
/// (from, to) coordinates in ascending order. A round blank: the whole turn (when `rho < radius`). A square blank
/// (side radius * sqrt(2), faces centred on angles 0, 90, ...): the whole turn at or below the inscribed circle,
/// four stretches round the corners above it, none at or above the corners.
std::vector<std::pair<double, double>> roundingMaterial(RoundingBlank blank, double radius, double rho, double c0);

/// Computes the Rounding toolpath of `params` for `tool`. Pass k runs at Z = axis + radius_k (axis = surfaceZ -
/// blankRadius); a square blank is cut only where material is left above the pass. Strokes follow `method`.
/// Bad parameters or tools give an empty toolpath and a diagnostic; nothing to remove gives `NoRestMaterial`;
/// a safe Z within the blank gives `InvalidParameters`; a stepover wider than the tool is limited
/// (`StepoverLimited`); too many strokes or passes give `WorkLimitExceeded`.
RoundingResult roundingToolpath(const tooling::Tool& tool, const std::optional<tooling::CuttingData>& cuttingData,
                                const RoundingParams& params, const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
