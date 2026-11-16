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
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Laser Cut / Fill toolpath (T-0323, ADR-0099; feature-inventory "Laser Module"): the beam follows the
// vectors (cut) or fills closed regions with hatch lines. Moves carry the beam power (`Move::jetPower`);
// the post switches the beam with its JET_TOOL_POWER / JET_TOOL_ON / JET_TOOL_OFF blocks.

namespace carvetto::toolpath {

/// What the laser does with the vectors. CutOutside / CutInside run the beam centre half a kerf
/// (the tool's diameter) plus the allowance away from the vector: closed vectors are taken as regions
/// (nesting parity), Outside keeps the region (outside outer boundaries, inside holes), Inside cuts
/// inside it; for open vectors Outside is the right-hand side of the direction of travel and Inside
/// the left. CutOn follows the vector. HatchFill fills closed regions with parallel lines.
enum class LaserStrategy { CutOutside, CutInside, CutOn, HatchFill };

/// Hatch fill order: Together treats all regions as one (a line crosses every region it meets before
/// the next line); Regionally fills each region (an outer boundary and its holes) completely before the next.
enum class LaserFillMode { Together, Regionally };

/// Parameters of a Laser Cut / Fill toolpath. Lengths in mm, angles in radians, power in percent.
struct LaserCutFillParams {
    std::string name{"Laser Cut"};
    LaserStrategy strategy{LaserStrategy::CutOutside};
    /// Beam power in percent of the machine's maximum, in (0, 100].
    double power{100.0};
    /// Speed of every move in mm/min; 0 takes the tool's feed rate (cutting data).
    double moveSpeed{0.0};
    /// Material thickness to cut through (mm, >= 0). 0 = one pass at the surface (engraving).
    /// Otherwise the number of passes is ceil(cutDepth / pass depth) and pass k runs at Z = -k x step.
    double cutDepth{0.0};
    /// Depth per pass (mm); 0 takes the cutting data's pass depth, and without one a single pass.
    double passDepth{0.0};
    /// How many times every pass is run over (>= 1), at the same Z.
    int repeats{1};
    /// Hatch lines run this far past the region with the beam off, so the beam is on at full speed
    /// while it is over the material (mm, >= 0). Ignored by the cut strategies.
    double overscan{0.0};
    /// Extra distance between the beam centre and the vector (negative cuts into it); for HatchFill the
    /// fill and outline stay this far inside the region (beyond the beam radius).
    double allowance{0.0};
    CutDirection direction{CutDirection::Climb};
    /// HatchFill: distance between neighbouring hatch lines (mm, > 0).
    double stepover{0.1};
    /// HatchFill: angle of the hatch lines from the X axis (radians).
    double hatchAngle{0.0};
    /// HatchFill: a second set of lines at right angles to the first.
    bool crossHatch{false};
    LaserFillMode fillMode{LaserFillMode::Together};
    /// HatchFill: also cut the outline of the filled region (after the fill).
    bool outline{false};
    /// Absolute Z the head rises to after the last pass.
    double safeZ{5.0};
    OrderOptions order;
    StartAt startAt{StartAt::VectorStart};
    BoxAnchor startAnchor{BoxAnchor::BottomLeft};
    geometry::Vec2 startPosition{0.0, 0.0};
    /// `chordal`: polygonising curves; `distance`: shorter pieces are dropped.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const LaserCutFillParams&) const = default;
};

struct LaserCutFillResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// Depth passes made (before repeats).
    std::size_t passes{0};
    /// Cut paths (cut strategies) or hatch strokes (HatchFill, outline paths included) per pass.
    std::size_t strokes{0};
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// The laser's beam is modelled by an end mill tool whose diameter is the kerf / spot size.
bool laserToolAllowed(tooling::ToolType type);

/// Z of each pass (0 or negative, shallowest first) for a cut of `cutDepth` in steps of `passDepth`
/// (`passDepth` <= 0 or >= `cutDepth`: one pass at the bottom). `cutDepth` 0 gives one pass at Z = 0.
std::vector<double> laserPassLevels(double cutDepth, double passDepth);

/// Computes the laser toolpath over `contours`. Cut strategies reuse the profile machinery for side,
/// kerf, nesting, direction and order (no tabs, leads or ramps); HatchFill builds the hatch lines of the
/// region (even-odd union of the closed vectors, inset by the beam radius plus the allowance) aligned to
/// the origin, so a cross hatch and neighbouring runs line up, and runs them as a serpentine.
/// Each stroke is: rapid to its start (to the overscan point first), the cut moves with the beam on, the
/// overscan out. Passes lower Z; repeats re-run a pass. After the last pass the head rises to safe Z.
/// Bad parameters or tools give an empty toolpath and a diagnostic; open vectors are ignored by HatchFill.
LaserCutFillResult laserCutFillToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                                        const std::optional<tooling::CuttingData>& cuttingData,
                                        const LaserCutFillParams& params, const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
