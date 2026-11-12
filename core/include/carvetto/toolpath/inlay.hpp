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

// Straight inlay toolpath (T-0238, ADR-0040; feature-inventory "Inlay Toolpath"): the male piece
// (a plug, straight or stepped) and the female piece (a pocket or a through hole) of the same
// vectors, made so that the male fits the female with a stated allowance (glue gap) all round.
// The V-carve inlay (V-bit plug and pocket) is a separate card.

namespace carvetto::toolpath {

/// Which piece of the inlay this toolpath cuts.
enum class InlayPiece { Male, Female };
/// Male: a straight plug, or a stepped one (a shoulder `stepWidth` wide all round, from the top
/// down to `stepDepth`; the body below is the inlay shape).
enum class InlayMaleStyle { Straight, Stepped };
/// Female: a pocket of `cutDepth`, or a hole cut through (a profile inside the vector).
enum class InlayFemaleStyle { Pocket, Hole };

/// Parameters of a straight inlay toolpath. Lengths in mm.
struct InlayParams {
    std::string name{"Inlay"};
    InlayPiece piece{InlayPiece::Male};
    InlayMaleStyle maleStyle{InlayMaleStyle::Straight};
    InlayFemaleStyle femaleStyle{InlayFemaleStyle::Pocket};
    /// Depth below the surface at which cutting starts (>= 0).
    double startDepth{0.0};
    /// Depth of the inlay below `startDepth` (> 0): the male's height and the pocket's depth.
    double cutDepth{3.0};
    /// Stepped male: depth of the shoulder (0 < stepDepth < cutDepth) and its width (> 0).
    double stepDepth{1.0};
    double stepWidth{2.0};
    /// Gap (>= 0) between male and female all round: the male is made smaller than the vector
    /// by this much, the female is cut to the vector (the inventory's "Allowance Offset").
    double allowance{0.0};
    /// Absolute Z for moves between cuts (> 0).
    double safeZ{5.0};
    CutDirection direction{CutDirection::Climb};
    /// Order of the male / hole cuts; none set = nearest first from `startPosition`.
    OrderOptions order;
    geometry::Vec2 startPosition{0.0, 0.0};
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const InlayParams&) const = default;
};

/// Result of `inlayToolpath`.
struct InlayResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// The shapes the two pieces are cut to, polygonal (outers counter-clockwise, holes clockwise).
struct InlayShapes {
    /// The cavity: the vector region with its corners rounded to the tool radius, so both pieces
    /// can be cut with the same round tool (convex corners are what a pocket leaves, concave
    /// corners what an outside profile leaves).
    std::vector<geometry::Contour> female;
    /// The plug: `female` inset by the allowance.
    std::vector<geometry::Contour> male;
    std::vector<ToolpathDiagnostic> diagnostics;
};

/// Flat end mills only (the tool radius is what rounds the corners).
bool inlayToolAllowed(tooling::ToolType type);

/// The shapes for `contours` (closed vectors, even-odd regions; open ones are ignored with a
/// diagnostic) with a tool of `toolDiameter`. `female` is empty, with a diagnostic, when the tool
/// fits nowhere or the input is unusable; `male` when the allowance consumes the shape.
InlayShapes inlayShapes(const std::vector<geometry::Contour>& contours, double toolDiameter, double allowance,
                        const geometry::Tolerance& tolerance);

/// Computes the toolpath of one piece. Male: an outside profile of the plug (stepped: the shoulder
/// level cut `stepWidth` further out, the levels below it clearing the band between the shoulder
/// and the plug, so the shoulder is free). Female pocket: the pocket of the cavity (Offset
/// strategy). Female hole: an inside profile of the cavity. Passes and stepover come from the
/// cutting data. Bad parameters or tools, no closed vector, a tool too large for the shape, or
/// an allowance that consumes it give an empty toolpath and a diagnostic.
InlayResult inlayToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                          const std::optional<tooling::CuttingData>& cuttingData, const InlayParams& params);

}  // namespace carvetto::toolpath
