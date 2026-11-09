#include "carvetto/toolpath/chamfer.hpp"

#include <cmath>
#include <string>
#include <utility>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"

namespace carvetto::toolpath {

namespace {

constexpr double kMinWidth = 1e-6;
constexpr double kMinAngle = 1e-3;
constexpr double kPiHalf = 1.5707963267948966;

double halfAngle(const tooling::Tool& tool, const ChamferParams& p) {
    return tool.type == tooling::ToolType::VBit ? 0.5 * tool.includedAngle : p.angle;
}

ChamferResult refuse(ChamferResult r, ToolpathIssue issue, std::string message) {
    r.diagnostics.push_back(ToolpathDiagnostic{issue, std::nullopt, std::move(message), std::nullopt});
    return r;
}

std::string mm(double v) {
    return std::to_string(v);
}

struct Plan {
    double angle{0.0};
    double apexDepth{0.0};  // depth of the (virtual) cone apex
    double tipDepth{0.0};   // depth of the tool tip
    double signedOffset{0.0};  // tool centre offset towards the face side (positive = into the face side)
};

/// Checks and plans; returns an empty string and fills `plan` when valid.
std::string plan(const tooling::Tool& tool, const ChamferParams& p, Plan& out) {
    const double values[] = {p.angle, p.width, p.overcut, p.safeZ, p.startPosition.x, p.startPosition.y,
                             p.tolerance.distance, p.tolerance.chordal, p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!chamferToolAllowed(tool.type)) {
        return "tool '" + tool.name + "' must be a V-bit or a round-nosed (ball nose) tool for a chamfer";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return "tool '" + tool.name + "': " + *bad;
    }
    const double a = halfAngle(tool, p);
    if (!(a >= kMinAngle) || !(a <= kPiHalf - kMinAngle)) {
        return tool.type == tooling::ToolType::VBit
                   ? "the V-bit's half angle must be between 0.001 and just below 90 degrees"
                   : "the chamfer angle must be between 0.001 radians and just below 90 degrees";
    }
    if (!(p.width >= kMinWidth) || p.width > 2.0 * kProfileMaxOffsetMm) {
        return "chamfer width must be between 0.000001 and " + std::to_string(2.0 * kProfileMaxOffsetMm) + " mm";
    }
    if (p.overcut < 0.0 || p.overcut > kProfileMaxOffsetMm) {
        return "overcut must be between 0 and " + std::to_string(kProfileMaxOffsetMm) + " mm";
    }
    if (!(p.safeZ > 0.0)) {
        return "safe Z must be above the material surface";
    }
    const double t = std::tan(a);
    const double cutDepth = 0.5 * p.width / t;
    const double apex = cutDepth + p.overcut / t;
    const double maxDepth = chamferMaxCutDepth(tool);
    if (!std::isfinite(apex)) {
        return "the chamfer depth is not finite";
    }
    double tip = apex;
    if (tool.type == tooling::ToolType::VBit) {
        if (apex > maxDepth * (1.0 + 1e-12)) {
            return "the chamfer needs a cut " + mm(apex) + " mm deep; the tool cuts at most " + mm(maxDepth) + " mm";
        }
    } else {
        const double r = 0.5 * tool.diameter;
        const double s = std::sin(a);
        // The ball is tangent to the face plane; the face must lie below the surface there.
        const double contactDepth = apex - r * (1.0 - s * s) / s;
        if (contactDepth < 0.0) {
            return "the round-nosed tool cannot form this face: the chamfer is too shallow (cut depth " +
                   mm(apex) + " mm) for a tool of radius " + mm(r) + " mm at this angle";
        }
        tip = apex - r * (1.0 / s - 1.0);
        if (!(tip > 0.0) || tip > maxDepth * (1.0 + 1e-12)) {
            return "the chamfer needs the tool tip " + mm(tip) + " mm deep; the tool cuts at most " + mm(maxDepth) +
                   " mm";
        }
    }
    out.angle = a;
    out.apexDepth = apex;
    out.tipDepth = tip;
    const double h = 0.5 * p.width;
    out.signedOffset = p.slope == ChamferSlope::Down ? -p.overcut : h + p.overcut;
    return {};
}

}  // namespace

bool chamferToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::VBit || type == tooling::ToolType::BallNose;
}

double chamferCutDepth(double angle, double width) {
    if (!std::isfinite(angle) || !std::isfinite(width) || !(angle > 0.0) || !(angle < kPiHalf) || !(width > 0.0)) {
        return 0.0;
    }
    return 0.5 * width / std::tan(angle);
}

double chamferWidth(double angle, double cutDepth) {
    if (!std::isfinite(angle) || !std::isfinite(cutDepth) || !(angle > 0.0) || !(angle < kPiHalf) ||
        !(cutDepth > 0.0)) {
        return 0.0;
    }
    return 2.0 * cutDepth * std::tan(angle);
}

double chamferMaxCutDepth(const tooling::Tool& tool) {
    if (tool.type == tooling::ToolType::VBit || tool.type == tooling::ToolType::BallNose) {
        return vCarvingFullWidthDepth(tool);
    }
    return 0.0;
}

std::optional<double> chamferTipDepth(const tooling::Tool& tool, const ChamferParams& params) {
    Plan p;
    if (!plan(tool, params, p).empty()) {
        return std::nullopt;
    }
    return p.tipDepth;
}

ChamferResult chamferToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData, const ChamferParams& params) {
    ChamferResult result;
    result.toolpath.name = params.name;
    Plan pl;
    if (const std::string bad = plan(tool, params, pl); !bad.empty()) {
        return refuse(std::move(result), chamferToolAllowed(tool.type) ? ToolpathIssue::InvalidParameters
                                                                       : ToolpathIssue::InvalidTool,
                      bad);
    }
    ProfileParams pp;
    pp.name = params.name;
    pp.direction = params.direction;
    pp.passes.startDepth = 0.0;
    pp.passes.cutDepth = pl.tipDepth;
    pp.passes.passDepth = pl.tipDepth;
    pp.safeZ = params.safeZ;
    pp.order = params.order;
    pp.startPosition = params.startPosition;
    pp.cuttingData = cuttingData;
    pp.tolerance = params.tolerance;
    const double r = profileToolRadius(tool, pp.passes);
    // Signed offset towards the face side: Outside = away from the region.
    const double towardsFace = pl.signedOffset;
    const double outward = params.type == ChamferType::Outside ? towardsFace : -towardsFace;
    if (outward == 0.0) {
        pp.side = ProfileSide::On;
        pp.allowance = 0.0;
    } else {
        pp.side = outward > 0.0 ? ProfileSide::Outside : ProfileSide::Inside;
        pp.allowance = std::fabs(outward) - r;
    }
    ProfileResult pr = profileToolpath(contours, tool, pp);
    result.toolpath = std::move(pr.toolpath);
    result.toolpath.name = params.name;
    result.diagnostics = std::move(pr.diagnostics);
    return result;
}

}  // namespace carvetto::toolpath
