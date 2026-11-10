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

// Chamfer toolpath (T-0236, ADR-0035; feature-inventory "Chamfer Toolpath"): bevel the edge of
// vectors with a V-bit (or a round-nosed tool) at one constant depth, the tool centre following
// an offset of each vector, so the tool's flank forms the chamfer face.

namespace carvetto::toolpath {

/// Which side of the vector the chamfer face lies on, in Profile's terms: closed vectors are
/// regions (nesting parity), Outside is outside outer boundaries and inside holes, Inside is
/// inside the region. Open vectors: Outside is the right of the direction of travel.
enum class ChamferType { Inside, Outside };

/// How the face slopes. Down: the face falls towards the vector (deepest at the vector, rising
/// away from it: the bevel of an edge, the tool's tip on the vector). Up: the face rises towards
/// the vector (at the surface on the vector, deepest away from it: a V-groove beside the vector).
enum class ChamferSlope { Down, Up };

/// Parameters of a chamfer toolpath. Lengths in mm, angles in radians.
///
/// Geometry (the face is a plane at the half angle A from the vertical): `width` W is the full
/// width the V cuts at the material surface, so the face spans W/2 horizontally, and the cut
/// depth is C = W / (2 tan A) (`chamferCutDepth`; width and cut depth are linked, 90 degree V-bit
/// with W = 2: C = 1).
struct ChamferParams {
    std::string name{"Chamfer"};
    ChamferType type{ChamferType::Inside};
    ChamferSlope slope{ChamferSlope::Down};
    /// Half angle A of the chamfer cone (> 0, < pi/2). V-bit: taken from the tool, this value is
    /// ignored. Round-nosed tool (ball nose): required, the face angle the tool is run at.
    double angle{0.0};
    /// Width W cut at the material surface (> 0).
    double width{2.0};
    /// Overcut (>= 0): the face is carried on this far past its bottom edge, along its plane, so
    /// the chamfer runs cleanly through the edge (the tool moves further out and down).
    double overcut{0.0};
    CutDirection direction{CutDirection::Climb};
    /// Absolute Z for moves between cuts (> 0).
    double safeZ{5.0};
    /// Order of the cuts; none set = nearest first from `startPosition`.
    OrderOptions order;
    geometry::Vec2 startPosition{0.0, 0.0};
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const ChamferParams&) const = default;
};

/// Result of `chamferToolpath`.
struct ChamferResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// V-bit and ball nose (round-nosed) tools.
bool chamferToolAllowed(tooling::ToolType type);

/// Cut depth C = W / (2 tan A) of a chamfer of width W (the linked value); 0 when the inputs are
/// not finite or not positive, or A is not below pi/2.
double chamferCutDepth(double angle, double width);
/// Width W = 2 C tan A for a cut depth C; 0 for invalid input.
double chamferWidth(double angle, double cutDepth);

/// The deepest cut of the tool at half angle `angle` (read-only "Max Cut Depth"): a V-bit's cone
/// height (D/2) / tan(A); a ball nose's radius D/2 below the surface. 0 for other tools.
double chamferMaxCutDepth(const tooling::Tool& tool);

/// Z depth (positive, below the surface) the tool tip is run at for `params` with `tool`, or
/// nullopt when the tool or parameters are unusable. A V-bit: C + overcut / tan A. A ball nose:
/// that less R (1 / sin A - 1), the tip sitting above the virtual cone apex.
std::optional<double> chamferTipDepth(const tooling::Tool& tool, const ChamferParams& params);

/// Computes a chamfer toolpath: each vector is machined once at the tip depth (`chamferTipDepth`),
/// the tool centre on the offset of the vector by the signed distance: type Inside/Outside picks
/// the face side; Down: -overcut towards the face (away from it by the overcut); Up: W/2 +
/// overcut into the face side. Implemented with `profileToolpath` (same vectors handling, order,
/// direction, diagnostics). A cut that would exceed the tool's depth, a round nose that cannot
/// form the face, bad parameters or tools give an empty toolpath and a diagnostic.
ChamferResult chamferToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData, const ChamferParams& params);

}  // namespace carvetto::toolpath
