#include "carvetto/toolpath/prism.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "carvetto/geometry/offset.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"

namespace carvetto::toolpath {

namespace {

constexpr std::size_t kMaxRings = 2000;
constexpr double kMinStepover = 1e-3;
constexpr double kMaxStepover = 100.0;
constexpr int kBisections = 22;
constexpr double kPeakMargin = 1e-3;  // the last cut stays this far inside the shape's maximum inset

PrismResult refuse(PrismResult r, ToolpathIssue issue, std::string message) {
    r.diagnostics.push_back(ToolpathDiagnostic{issue, std::nullopt, std::move(message), std::nullopt});
    return r;
}

void note(PrismResult& r, const ToolpathDiagnostic& d) {
    if (!hasIssue(r.diagnostics, d.issue)) {
        r.diagnostics.push_back(d);
    }
}

std::string validate(const tooling::Tool& tool, const PrismParams& p) {
    const double values[] = {p.depth,           p.stepover,          p.safeZ,
                             p.startPosition.x, p.startPosition.y,   p.tolerance.distance,
                             p.tolerance.chordal, p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!prismToolAllowed(tool.type)) {
        return "tool '" + tool.name + "' must be a V-bit or a ball nose tool for a prism";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return "tool '" + tool.name + "': " + *bad;
    }
    if (!p.fullPrism && !(p.depth > 0.0)) {
        return "prism depth must be above zero";
    }
    if (!(p.stepover >= kMinStepover) || p.stepover > kMaxStepover) {
        return "stepover must be between 0.001 and 100 mm";
    }
    if (!(p.safeZ > 0.0)) {
        return "safe Z must be above the material surface";
    }
    return {};
}

bool regionSurvives(const std::vector<geometry::Contour>& closed, double inset, const geometry::Tolerance& tol) {
    return !geometry::offsetContours(closed, -inset, geometry::JoinType::Round, tol).empty();
}

}  // namespace

bool prismToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::VBit || type == tooling::ToolType::BallNose;
}

double prismMaxDepth(const tooling::Tool& tool) {
    return prismToolAllowed(tool.type) ? vCarvingFullWidthDepth(tool) : 0.0;
}

PrismResult prismToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                          const std::optional<tooling::CuttingData>& cuttingData, const PrismParams& params) {
    PrismResult result;
    result.toolpath.name = params.name;
    if (const std::string bad = validate(tool, params); !bad.empty()) {
        return refuse(std::move(result),
                      prismToolAllowed(tool.type) ? ToolpathIssue::InvalidParameters : ToolpathIssue::InvalidTool, bad);
    }
    const double maxDepth = prismMaxDepth(tool);
    if (!(maxDepth > 0.0) || !std::isfinite(maxDepth)) {
        return refuse(std::move(result), ToolpathIssue::InvalidTool, "the tool has no usable cutting depth");
    }
    double depth = params.fullPrism ? maxDepth : params.depth;
    if (depth > maxDepth) {
        depth = maxDepth;
        result.diagnostics.push_back(ToolpathDiagnostic{
            ToolpathIssue::CarveDepthLimited, std::nullopt,
            "the prism depth was limited to the tool's maximum cut depth of " + std::to_string(maxDepth) + " mm",
            std::nullopt});
    }

    std::vector<geometry::Contour> closed;
    for (const geometry::Contour& c : contours) {
        if (c.closed) {
            closed.push_back(c);
        } else {
            note(result, ToolpathDiagnostic{ToolpathIssue::OpenContourIgnored, std::nullopt,
                                            "open vectors are ignored: a prism needs closed vectors", std::nullopt});
        }
    }
    if (closed.empty()) {
        return refuse(std::move(result), ToolpathIssue::NoRegion, "no closed vector to carve");
    }

    // Largest inset of the cuts: the width the tool cuts at the prism depth, or where the shape runs out.
    double reach = 0.5 * tooling::cuttingWidthAtDepth(tool, depth);
    if (!(reach > 0.0) || !std::isfinite(reach)) {
        return refuse(std::move(result), ToolpathIssue::InvalidParameters, "the prism depth cuts no width");
    }
    const bool fullDepth = regionSurvives(closed, reach, params.tolerance);
    if (!fullDepth) {
        double lo = 0.0;
        double hi = reach;
        for (int i = 0; i < kBisections; ++i) {
            const double mid = 0.5 * (lo + hi);
            (regionSurvives(closed, mid, params.tolerance) ? lo : hi) = mid;
        }
        // Stay clear of the sliver that is left where the shape is about to vanish.
        reach = lo - kPeakMargin;
        if (!(reach > params.stepover * 1e-3)) {
            return refuse(std::move(result), ToolpathIssue::ToolTooLarge, "the shape is too narrow to carve");
        }
    }
    const double ringCount = std::ceil(reach / params.stepover);
    if (ringCount > static_cast<double>(kMaxRings)) {
        return refuse(std::move(result), ToolpathIssue::WorkLimitExceeded,
                      "the stepover would need more than " + std::to_string(kMaxRings) + " cuts; use a larger stepover");
    }
    const std::size_t rings = static_cast<std::size_t>(ringCount);

    bool first = true;
    for (std::size_t k = 1; k <= rings; ++k) {
        const bool last = k == rings;
        const double inset = last ? reach : static_cast<double>(k) * params.stepover;
        const double z = last && fullDepth ? depth : vCarvingDepthForRadius(tool, inset);
        if (!(z > 0.0)) {
            continue;
        }
        ProfileParams pp;
        pp.name = params.name;
        pp.side = ProfileSide::Inside;
        pp.allowance = 0.0;
        pp.direction = params.direction;
        pp.passes.startDepth = 0.0;
        pp.passes.cutDepth = z;
        pp.passes.passDepth = z;
        pp.safeZ = params.safeZ;
        pp.order = params.order;
        pp.startPosition = params.startPosition;
        pp.cuttingData = cuttingData;
        pp.tolerance = params.tolerance;
        ProfileResult pr = profileToolpath(closed, tool, pp);
        if (!pr.ok()) {
            for (const ToolpathDiagnostic& d : pr.diagnostics) {
                if (d.issue != ToolpathIssue::ToolTooLarge) {
                    note(result, d);
                }
            }
            // A refusal (parameters, budget, cancel) ends the run: nothing partial.
            if (pr.has(ToolpathIssue::InvalidParameters) || pr.has(ToolpathIssue::WorkLimitExceeded) ||
                pr.has(ToolpathIssue::Cancelled)) {
                result.toolpath.moves.clear();
                result.reachedDepth = 0.0;
                return result;
            }
            continue;
        }
        for (const ToolpathDiagnostic& d : pr.diagnostics) {
            note(result, d);
        }
        if (first) {
            result.toolpath.tool = pr.toolpath.tool;
            first = false;
        }
        result.toolpath.append(pr.toolpath);
        result.reachedDepth = std::max(result.reachedDepth, z);
    }
    if (result.toolpath.isEmpty()) {
        note(result, ToolpathDiagnostic{ToolpathIssue::ToolTooLarge, std::nullopt, "nothing could be carved", std::nullopt});
    }
    return result;
}

}  // namespace carvetto::toolpath
