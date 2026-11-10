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

// Prism carving toolpath (T-0240, ADR-0042; feature-inventory "Prism Carving Toolpath"): carve the
// inside of closed vectors into a faceted prism with a V-bit (or ball nose): the facets fall from
// the vector, at the surface, towards the middle of the shape, by the tool's flank angle.

namespace carvetto::toolpath {

/// Parameters of a prism toolpath. Lengths in mm.
struct PrismParams {
    std::string name{"Prism"};
    /// Prism depth: the depth below the surface of the lowest point of the prism (> 0). Ignored
    /// with `fullPrism`.
    double depth{2.0};
    /// Set Depth for Full Prism: carve as deep as the tool allows (`prismMaxDepth`); the prism
    /// then ends where the facets meet or at the tool's limit, whichever is first.
    bool fullPrism{false};
    /// Horizontal distance between the cuts that form the facets (> 0, <= 100).
    double stepover{0.25};
    CutDirection direction{CutDirection::Climb};
    /// Absolute Z for moves between cuts (> 0).
    double safeZ{5.0};
    OrderOptions order;
    geometry::Vec2 startPosition{0.0, 0.0};
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const PrismParams&) const = default;
};

/// Result of `prismToolpath`.
struct PrismResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// Depth of the lowest cut (positive, below the surface); 0 when nothing was cut.
    double reachedDepth{0.0};
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// V-bit and ball nose tools.
bool prismToolAllowed(tooling::ToolType type);

/// The deepest cut of the tool (the full-width depth: a V-bit's cone height, a ball's radius);
/// 0 for other tools. This is the depth `fullPrism` asks for.
double prismMaxDepth(const tooling::Tool& tool);

/// Computes a prism toolpath. The region is the even-odd region of the closed vectors (holes
/// stay raised); open vectors are ignored with a diagnostic. The facets are cut as inward
/// offsets of the vectors: the cut at inset s runs the tool at the depth where its cutting
/// width at the surface is 2 s, so its edge sits on the vector. Cuts go from the vector inwards,
/// every `stepover`, the last at the depth limit (the prism depth, the tool's maximum, or the
/// largest inset the shape has, whichever is least; `PrismResult::reachedDepth`). A depth beyond
/// the tool's maximum is limited with `CarveDepthLimited`. Every cut is a `profileToolpath` run
/// (order, direction, trimming near other vectors and diagnostics as Profile).
/// Bad parameters or tools, no closed region, or more rings than 2000 give an empty toolpath
/// and a diagnostic.
PrismResult prismToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                          const std::optional<tooling::CuttingData>& cuttingData, const PrismParams& params);

}  // namespace carvetto::toolpath
