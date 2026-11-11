#pragma once

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

// Fluting toolpath (T-0239, ADR-0041; feature-inventory "Fluting Toolpath"): the tool centre follows
// each vector at a depth that varies along it: a flute that ramps in from the surface at the start
// of the vector (and out again at its end) and is cut at the flute depth between the ramps.

namespace carvetto::toolpath {

/// Where the depth ramps are. RampOverLength: depth rises from 0 at the start to the flute depth
/// at the end, over the complete length. RampAtStart: ramps in over the ramp length, then runs at
/// the flute depth to the end. RampAtStartAndEnd: ramps in at the start and out at the end.
enum class FluteType { RampOverLength, RampAtStart, RampAtStartAndEnd };
/// Shape of a ramp: Linear (constant slope) or Smooth (smoothstep 3t^2 - 2t^3, no slope at either
/// end, so the flute leaves the surface and meets the full depth tangentially).
enum class FluteRampType { Linear, Smooth };
/// How the ramp length is given: Length (mm) or Percent of the vector's length.
enum class FluteRampBasis { Length, Percent };

/// Parameters of a fluting toolpath. Lengths in mm.
struct FlutingParams {
    std::string name{"Fluting"};
    /// Flute depth D below the material surface (> 0).
    double fluteDepth{2.0};
    FluteType type{FluteType::RampAtStart};
    FluteRampBasis rampBasis{FluteRampBasis::Length};
    /// Ramp length (mm, > 0) when `rampBasis` is Length; ignored for RampOverLength.
    double rampLength{10.0};
    /// Ramp length as a percentage of the vector's length (0, 100] when `rampBasis` is Percent.
    double rampPercent{20.0};
    FluteRampType rampType{FluteRampType::Linear};
    /// Absolute Z for moves between flutes (> 0).
    double safeZ{5.0};
    OrderOptions order;
    geometry::Vec2 startPosition{0.0, 0.0};
    /// `chordal`: polygonising curved vectors; `distance`: a vector no longer than this is skipped.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const FlutingParams&) const = default;
};

/// A flute made.
struct FlutingFlute {
    /// Index of the input vector.
    std::size_t contour{0};
    /// Length of the vector's path (mm) the depth profile is laid over.
    double length{0.0};
    /// Length of each ramp actually used (after clamping to the vector), mm.
    double rampLength{0.0};
    bool operator==(const FlutingFlute&) const = default;
};

struct FlutingResult {
    Toolpath toolpath;
    /// The flutes in machining order.
    std::vector<FlutingFlute> flutes;
    std::vector<ToolpathDiagnostic> diagnostics;
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// End mill (flat) and ball nose tools.
bool flutingToolAllowed(tooling::ToolType type);

/// Length of each ramp for a vector of `totalLength`: the whole length for RampOverLength; else the
/// ramp length (or percentage of the length), at most the length (RampAtStart) or half of it
/// (RampAtStartAndEnd, so the two ramps never overlap). 0 for non-finite or non-positive input.
double flutingRampLength(const FlutingParams& params, double totalLength);

/// Depth (>= 0, mm below the surface) of the flute at arc length `s` (clamped to [0, totalLength])
/// along a vector of `totalLength`; the profile of the toolpath (ADR-0041).
double flutingDepthAt(const FlutingParams& params, double totalLength, double s);

/// Computes the fluting toolpath. Per vector (open or closed; its direction and start point are
/// used as drawn): rapid to the start at safe Z, plunge, feed moves along the vector (curves
/// polygonised to `tolerance.chordal`) whose Z follows `flutingDepthAt`, retract. A flute deeper
/// than the cutting data's pass depth is cut in several passes, each limited to its level and
/// cutting only where the profile is deeper than the pass before. Smooth ramps are sampled in 32
/// steps. Bad parameters or tools give an empty toolpath and a diagnostic; empty, discontinuous
/// and zero-length vectors are skipped and named; a ramp longer than the vector is shortened
/// (`LeadShortened`).
FlutingResult flutingToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData, const FlutingParams& params);

}  // namespace carvetto::toolpath
