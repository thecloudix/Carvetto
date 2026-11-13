#include "carvetto/toolpath/vcarve_inlay.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/toolpath/profile.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::BooleanOp;
using geometry::Box2;
using geometry::Contour;
using geometry::JoinType;

constexpr double kMaxLength = 100000.0;
constexpr std::size_t kMaxSpans = 100000;
constexpr std::size_t kMaxDerivedSpans = 20000;  // spans of an offset or boundary shape handed on to the V-carve

ToolpathDiagnostic diag(ToolpathIssue issue, std::string message) {
    return ToolpathDiagnostic{issue, std::nullopt, std::move(message), std::nullopt};
}

std::size_t spanCount(const std::vector<Contour>& cs) {
    std::size_t n = 0;
    for (const Contour& c : cs) {
        n += c.spans.size();
    }
    return n;
}

double netArea(const std::vector<Contour>& cs) {
    double a = 0.0;
    for (const Contour& c : cs) {
        a += geometry::signedArea(c);
    }
    return a;
}

bool inRange(double v, double lo, double hi) {
    return std::isfinite(v) && v >= lo && v <= hi;
}

std::string checkParams(const tooling::Tool& tool, const VCarveInlayParams& p) {
    const double values[] = {p.pocketDepth,      p.glueGap,        p.surfaceClearance, p.boundaryOffset,
                             p.safeZ,            p.clearanceRasterAngle, p.rampDistance,  p.startPosition.x,
                             p.startPosition.y,  p.tolerance.distance, p.tolerance.chordal, p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!vCarveInlayToolAllowed(tool.type)) {
        return "tool '" + tool.name + "' must be a V-bit for a V-carve inlay";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return "tool '" + tool.name + "': " + *bad;
    }
    if (!inRange(p.pocketDepth, 1e-6, kMaxLength)) {
        return "pocket depth must be between 0.000001 and " + std::to_string(kMaxLength) + " mm";
    }
    if (!inRange(p.glueGap, 0.0, kMaxLength)) {
        return "glue gap must be between 0 and " + std::to_string(kMaxLength) + " mm";
    }
    if (!inRange(p.surfaceClearance, 0.0, kMaxLength)) {
        return "surface clearance must be between 0 and " + std::to_string(kMaxLength) + " mm";
    }
    if (!inRange(p.boundaryOffset, 0.0, kProfileMaxOffsetMm)) {
        return "boundary offset must be between 0 and " + std::to_string(kProfileMaxOffsetMm) + " mm";
    }
    if (!(p.safeZ > 0.0) || p.safeZ > kMaxLength) {
        return "safe Z must be above the material surface";
    }
    if (p.tolerance.chordal < 1e-4 || p.tolerance.chordal > 1.0 || p.tolerance.distance < 1e-6 ||
        p.tolerance.distance > p.tolerance.chordal) {
        return "tolerance: 0.0001 <= chordal <= 1 mm and 0.000001 <= distance <= chordal";
    }
    return {};
}

/// The even-odd region of the closed vectors; empty with a diagnostic when unusable.
std::vector<Contour> closedRegion(const std::vector<Contour>& contours, const geometry::Tolerance& tol,
                                  std::vector<ToolpathDiagnostic>& out) {
    std::vector<Contour> closed;
    std::size_t spans = 0;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const Contour& c = contours[i];
        if (!c.closed) {
            out.push_back(ToolpathDiagnostic{ToolpathIssue::OpenContourIgnored, i,
                                             "open vector ignored: a V-carve inlay needs closed vectors", std::nullopt});
            continue;
        }
        if (c.spans.empty()) {
            out.push_back(ToolpathDiagnostic{ToolpathIssue::EmptyContour, i, "vector without spans skipped", std::nullopt});
            continue;
        }
        if (!std::isfinite(c.length())) {
            out.push_back(diag(ToolpathIssue::InvalidParameters, "vector has a non-finite coordinate"));
            return {};
        }
        const Box2 b = c.bounds();
        if (!(b.min.x >= -kProfileMaxCoordinateMm && b.max.x <= kProfileMaxCoordinateMm &&
              b.min.y >= -kProfileMaxCoordinateMm && b.max.y <= kProfileMaxCoordinateMm)) {
            out.push_back(diag(ToolpathIssue::WorkLimitExceeded, "vector coordinates out of range"));
            return {};
        }
        spans += c.spans.size();
        if (spans > kMaxSpans) {
            out.push_back(diag(ToolpathIssue::WorkLimitExceeded, "too many spans for a V-carve inlay"));
            return {};
        }
        closed.push_back(c);
    }
    if (closed.empty()) {
        out.push_back(diag(ToolpathIssue::NoRegion, "no closed vector: nothing to inlay"));
        return {};
    }
    std::vector<Contour> region = geometry::unionContours(closed, tol, geometry::FillRule::EvenOdd);
    if (region.empty()) {
        out.push_back(diag(ToolpathIssue::NoRegion, "the closed vectors enclose no area"));
    }
    return region;
}

Box2 boundsOf(const std::vector<Contour>& cs) {
    Box2 b;
    for (const Contour& c : cs) {
        b.expand(c.bounds());
    }
    return b;
}

}  // namespace

bool vCarveInlayToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::VBit;
}

VCarvePlug vCarvePlugGeometry(const std::vector<Contour>& contours, const tooling::Tool& tool,
                              const VCarveInlayParams& params, const Box2& plugSheet) {
    VCarvePlug out;
    const auto fail = [&](ToolpathIssue issue, std::string why) {
        out.region.clear();
        out.diagnostics.push_back(diag(issue, std::move(why)));
        return out;
    };
    if (const std::string bad = checkParams(tool, params); !bad.empty()) {
        return fail(vCarveInlayToolAllowed(tool.type) ? ToolpathIssue::InvalidParameters : ToolpathIssue::InvalidTool,
                    bad);
    }
    const geometry::Tolerance& tol = params.tolerance;
    std::vector<Contour> region = closedRegion(contours, tol, out.diagnostics);
    if (region.empty()) {
        return out;
    }
    const double t = std::tan(0.5 * tool.includedAngle);
    const double full = vCarvingFullWidthDepth(tool);
    out.pocketDepth = std::min(params.pocketDepth, full);
    out.plugDepth = std::min(out.pocketDepth + params.surfaceClearance, full);
    if (!(params.glueGap < out.pocketDepth)) {
        return fail(ToolpathIssue::InvalidParameters, "the glue gap must be smaller than the pocket depth");
    }

    // Mirror about the vertical line through the middle of the vectors; on another sheet also centre.
    const Box2 own = boundsOf(region);
    const geometry::Vec2 centre = own.centre();
    geometry::Transform2 place = geometry::Transform2::mirrorX(centre.x);
    if (params.plugSheet != 0) {
        if (plugSheet.isEmpty()) {
            return fail(ToolpathIssue::InvalidParameters, "the plug's sheet is unknown");
        }
        place = geometry::Transform2::translation(plugSheet.centre() - centre) * place;
    }
    for (Contour& c : region) {
        c.transform(place);
    }
    region = geometry::unionContours(region, tol, geometry::FillRule::EvenOdd);

    // The plug's top face: the vectors inset by (pocket depth - glue gap) x t.
    const double inset = (out.pocketDepth - params.glueGap) * t;
    std::vector<Contour> top = geometry::offsetContours(region, -inset, JoinType::Round, tol);
    if (geometry::cancelRequested()) {
        return fail(ToolpathIssue::Cancelled, "cancelled");
    }
    if (top.empty()) {
        return fail(ToolpathIssue::ToolTooLarge,
                    "the shape is narrower than twice the plug's inset (" + std::to_string(inset) + " mm): no plug is left");
    }
    if (spanCount(top) > kMaxDerivedSpans) {
        return fail(ToolpathIssue::WorkLimitExceeded, "the plug shape has too many spans (limit " +
                                                          std::to_string(kMaxDerivedSpans) + ")");
    }
    // Thin parts (narrower than twice the inset) have no top face: say so.
    {
        const double eps = std::max(2.0 * tol.chordal, 0.02 * inset);
        std::vector<Contour> regrown = geometry::offsetContours(top, inset, JoinType::Sharp, tol, 1000.0);
        std::vector<Contour> lost = geometry::booleanOp(BooleanOp::Difference, region, regrown, tol).closed;
        std::vector<Contour> solid = geometry::offsetContours(lost, -eps, JoinType::Round, tol);
        if (!solid.empty()) {
            out.diagnostics.push_back(diag(ToolpathIssue::RegionTrimmed,
                                           "parts of the vectors thinner than " + std::to_string(2.0 * inset) +
                                               " mm have no plug (pocket depth x 2 x tan(half the V-bit angle)); use a "
                                               "shallower pocket"));
        }
    }

    // The plug's outer boundary.
    std::vector<Contour> boundary;
    const Box2 placed = boundsOf(region);
    switch (params.plugBoundary) {
        case PlugBoundary::SheetLimits:
            if (plugSheet.isEmpty()) {
                return fail(ToolpathIssue::InvalidParameters, "the plug's sheet is unknown: Sheet Limits needs it");
            }
            boundary.push_back(Contour::rectangle(plugSheet));
            break;
        case PlugBoundary::BoundaryOffset: boundary.push_back(Contour::rectangle(placed.inflated(params.boundaryOffset))); break;
        case PlugBoundary::VectorOffset: {
            std::vector<Contour> grown = geometry::offsetContours(region, params.boundaryOffset, JoinType::Round, tol);
            for (Contour& c : grown) {
                if (geometry::signedArea(c) > 0.0) {  // outers only: islands are inside the plug area
                    boundary.push_back(std::move(c));
                }
            }
            break;
        }
    }
    if (boundary.empty() || spanCount(boundary) > kMaxDerivedSpans) {
        return fail(ToolpathIssue::WorkLimitExceeded, "the plug boundary is empty or has too many spans");
    }
    // The flank (the V-bit cuts from the top face out to 2 x plug depth x t) must fit inside it.
    const double need = 2.0 * out.plugDepth * t;
    const std::vector<Contour> reach = geometry::offsetContours(top, need, JoinType::Round, tol);
    const std::vector<Contour> outside = geometry::booleanOp(BooleanOp::Difference, reach, boundary, tol).closed;
    if (geometry::cancelRequested()) {
        return fail(ToolpathIssue::Cancelled, "cancelled");
    }
    if (netArea(outside) > tol.chordal * tol.chordal) {
        return fail(ToolpathIssue::ToolTooLarge,
                    "the plug's outer boundary is too tight: it must leave " + std::to_string(need) +
                        " mm (2 x plug depth x tan(half the V-bit angle)) beyond the plug's top face for the flank to reach full depth");
    }
    out.region = geometry::booleanOp(BooleanOp::Difference, boundary, top, tol).closed;
    out.top = std::move(top);
    out.boundary = std::move(boundary);
    if (out.region.empty()) {
        return fail(ToolpathIssue::NoRegion, "nothing is left to carve round the plug");
    }
    return out;
}

VCarvingResult vCarveInlayToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                                   const std::optional<tooling::CuttingData>& cuttingData,
                                   const std::vector<PocketTool>& clearanceTools, const VCarveInlayParams& params,
                                   const Box2& plugSheet) {
    VCarvingResult result;
    result.toolpath.name = params.name;
    if (const std::string bad = checkParams(tool, params); !bad.empty()) {
        result.diagnostics.push_back(diag(
            vCarveInlayToolAllowed(tool.type) ? ToolpathIssue::InvalidParameters : ToolpathIssue::InvalidTool, bad));
        return result;
    }
    VCarvingParams vp;
    vp.name = params.name;
    vp.startDepth = 0.0;
    vp.safeZ = params.safeZ;
    vp.clearanceStrategy = params.clearanceStrategy;
    vp.clearanceDirection = params.clearanceDirection;
    vp.clearanceRasterAngle = params.clearanceRasterAngle;
    vp.rampPlunge = params.rampPlunge;
    vp.rampDistance = params.rampDistance;
    vp.order = params.order;
    vp.startPosition = params.startPosition;
    vp.tolerance = params.tolerance;

    if (params.piece == VCarveInlayPiece::Pocket) {
        std::vector<Contour> region = closedRegion(contours, params.tolerance, result.diagnostics);
        if (region.empty()) {
            return result;
        }
        const double full = vCarvingFullWidthDepth(tool);
        vp.flatDepth = std::min(params.pocketDepth, full);
        if (params.pocketDepth > full) {
            result.diagnostics.push_back(diag(ToolpathIssue::CarveDepthLimited,
                                              "the pocket depth was limited to " + std::to_string(full) +
                                                  " mm, where the V-bit reaches its full width"));
        }
        VCarvingResult r = vCarvingToolpath(region, tool, cuttingData, clearanceTools, vp);
        r.diagnostics.insert(r.diagnostics.begin(), result.diagnostics.begin(), result.diagnostics.end());
        return r;
    }
    VCarvePlug plug = vCarvePlugGeometry(contours, tool, params, plugSheet);
    result.diagnostics = plug.diagnostics;
    if (plug.region.empty()) {
        return result;
    }
    if (params.pocketDepth + params.surfaceClearance > plug.plugDepth && plug.plugDepth > 0.0) {
        result.diagnostics.push_back(diag(ToolpathIssue::CarveDepthLimited,
                                          "the plug depth was limited to " + std::to_string(plug.plugDepth) +
                                              " mm, where the V-bit reaches its full width"));
    }
    vp.flatDepth = plug.plugDepth;
    VCarvingResult r = vCarvingToolpath(plug.region, tool, cuttingData, clearanceTools, vp);
    r.diagnostics.insert(r.diagnostics.begin(), result.diagnostics.begin(), result.diagnostics.end());
    return r;
}

}  // namespace carvetto::toolpath
