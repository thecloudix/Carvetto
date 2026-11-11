#include "carvetto/toolpath/inlay.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::JoinType;

constexpr double kMaxLength = 100000.0;
constexpr std::size_t kMaxSpans = 100000;
constexpr std::size_t kMaxDerivedSpans = 4000;  // spans of the offset shapes handed on to the profile / pocket
constexpr const char* kAllowanceConsumes = "the allowance consumes the whole shape";
constexpr std::size_t kMaxBandPasses = 1000;
constexpr double kBandOverlap = 0.9;  // band pass spacing as a fraction of the tool diameter

ToolpathDiagnostic diag(ToolpathIssue issue, std::string message, std::optional<std::size_t> contour = std::nullopt) {
    return ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt};
}

InlayResult refuse(InlayResult r, ToolpathIssue issue, std::string message) {
    r.diagnostics.push_back(diag(issue, std::move(message)));
    return r;
}

std::size_t spanCount(const std::vector<Contour>& cs) {
    std::size_t n = 0;
    for (const Contour& c : cs) {
        n += c.spans.size();
    }
    return n;
}

bool inRange(double v, double lo, double hi) {
    return std::isfinite(v) && v >= lo && v <= hi;
}

std::string checkParams(const tooling::Tool& tool, const InlayParams& p) {
    const double values[] = {p.startDepth,       p.cutDepth,           p.stepDepth,
                             p.stepWidth,        p.allowance,          p.safeZ,
                             p.startPosition.x,  p.startPosition.y,    p.tolerance.distance,
                             p.tolerance.chordal, p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!inlayToolAllowed(tool.type)) {
        return "tool '" + tool.name + "' must be a flat end mill for an inlay";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return "tool '" + tool.name + "': " + *bad;
    }
    if (!inRange(p.startDepth, 0.0, kMaxLength)) {
        return "start depth must be between 0 and " + std::to_string(kMaxLength) + " mm";
    }
    if (!inRange(p.cutDepth, 1e-6, kMaxLength)) {
        return "inlay depth must be between 0.000001 and " + std::to_string(kMaxLength) + " mm";
    }
    if (!inRange(p.allowance, 0.0, kProfileMaxOffsetMm)) {
        return "allowance must be between 0 and " + std::to_string(kProfileMaxOffsetMm) + " mm";
    }
    if (!(p.safeZ > 0.0) || p.safeZ > kMaxLength) {
        return "safe Z must be above the material surface";
    }
    if (p.tolerance.chordal < 1e-4 || p.tolerance.chordal > 1.0 || p.tolerance.distance < 1e-6 ||
        p.tolerance.distance > p.tolerance.chordal) {
        return "tolerance: 0.0001 <= chordal <= 1 mm and 0.000001 <= distance <= chordal";
    }
    if (p.piece == InlayPiece::Male && p.maleStyle == InlayMaleStyle::Stepped) {
        if (!(p.stepDepth > 0.0) || !(p.stepDepth < p.cutDepth)) {
            return "step depth must be above 0 and below the inlay depth";
        }
        if (!inRange(p.stepWidth, 1e-6, kProfileMaxOffsetMm)) {
            return "step width must be between 0.000001 and " + std::to_string(kProfileMaxOffsetMm) + " mm";
        }
    }
    return {};
}

}  // namespace

bool inlayToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::EndMill;
}

InlayShapes inlayShapes(const std::vector<Contour>& contours, double toolDiameter, double allowance,
                        const geometry::Tolerance& tolerance) {
    InlayShapes out;
    if (!(toolDiameter > 0.0) || !std::isfinite(toolDiameter) || !(allowance >= 0.0) || !std::isfinite(allowance) ||
        !std::isfinite(tolerance.chordal) || !std::isfinite(tolerance.distance)) {
        out.diagnostics.push_back(diag(ToolpathIssue::InvalidParameters, "tool diameter or allowance is not usable"));
        return out;
    }
    const double r = 0.5 * toolDiameter;
    std::vector<Contour> closed;
    std::size_t spans = 0;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const Contour& c = contours[i];
        if (!c.closed) {
            out.diagnostics.push_back(diag(ToolpathIssue::OpenContourIgnored, "open vector ignored: an inlay needs closed vectors", i));
            continue;
        }
        if (c.spans.empty()) {
            out.diagnostics.push_back(diag(ToolpathIssue::EmptyContour, "vector without spans skipped", i));
            continue;
        }
        if (!std::isfinite(c.length())) {
            out.diagnostics.push_back(diag(ToolpathIssue::InvalidParameters, "vector has a non-finite coordinate", i));
            return out;
        }
        const geometry::Box2 b = c.bounds();
        if (!(b.min.x >= -kProfileMaxCoordinateMm && b.max.x <= kProfileMaxCoordinateMm &&
              b.min.y >= -kProfileMaxCoordinateMm && b.max.y <= kProfileMaxCoordinateMm)) {
            out.diagnostics.push_back(diag(ToolpathIssue::WorkLimitExceeded, "vector coordinates out of range", i));
            return out;
        }
        spans += c.spans.size();
        if (spans > kMaxSpans) {
            out.diagnostics.push_back(diag(ToolpathIssue::WorkLimitExceeded, "too many spans for an inlay"));
            return out;
        }
        closed.push_back(c);
    }
    if (closed.empty()) {
        out.diagnostics.push_back(diag(ToolpathIssue::NoRegion, "no closed vector: nothing to inlay"));
        return out;
    }
    const geometry::Tolerance& tol = tolerance;
    std::vector<Contour> region = geometry::unionContours(closed, tol, geometry::FillRule::EvenOdd);
    if (region.empty()) {
        out.diagnostics.push_back(diag(ToolpathIssue::NoRegion, "the closed vectors enclose no area"));
        return out;
    }
    // Opening (what a pocket leaves), then closing (what an outside profile leaves). The offset
    // arcs are polygonised, so each stage can have more spans than its input: the span count is
    // bounded after every stage, and a cancel request ends the run between stages.
    const double steps[] = {-r, r, r, -r};
    std::vector<Contour> shape = std::move(region);
    for (const double step : steps) {
        shape = geometry::offsetContours(shape, step, JoinType::Round, tol);
        if (geometry::cancelRequested()) {
            out.diagnostics.push_back(diag(ToolpathIssue::Cancelled, "cancelled"));
            return out;
        }
        if (shape.empty()) {
            break;
        }
        if (spanCount(shape) > kMaxDerivedSpans) {
            out.diagnostics.push_back(
                diag(ToolpathIssue::WorkLimitExceeded, "the offset shape has too many spans for an inlay (limit " +
                                                           std::to_string(kMaxDerivedSpans) + ")"));
            return out;
        }
    }
    if (shape.empty()) {
        out.diagnostics.push_back(diag(ToolpathIssue::ToolTooLarge, "the tool is too large for the vectors"));
        return out;
    }
    out.female = shape;
    if (allowance > 0.0) {
        out.male = geometry::offsetContours(shape, -allowance, JoinType::Round, tol);
        if (geometry::cancelRequested()) {
            return InlayShapes{{}, {}, {diag(ToolpathIssue::Cancelled, "cancelled")}};
        }
        if (spanCount(out.male) > kMaxDerivedSpans) {
            return InlayShapes{{}, {}, {diag(ToolpathIssue::WorkLimitExceeded, "the offset shape has too many spans for an inlay (limit " +
                                                                                   std::to_string(kMaxDerivedSpans) + ")")}};
        }
    } else {
        out.male = shape;
    }
    if (out.male.empty()) {
        out.diagnostics.push_back(diag(ToolpathIssue::NoRegion, kAllowanceConsumes));
    }
    return out;
}

InlayResult inlayToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                          const std::optional<tooling::CuttingData>& cuttingData, const InlayParams& params) {
    InlayResult result;
    result.toolpath.name = params.name;
    if (const std::string bad = checkParams(tool, params); !bad.empty()) {
        return refuse(std::move(result),
                      inlayToolAllowed(tool.type) ? ToolpathIssue::InvalidParameters : ToolpathIssue::InvalidTool, bad);
    }
    InlayShapes shapes = inlayShapes(contours, tool.diameter, params.allowance, params.tolerance);
    const bool male = params.piece == InlayPiece::Male;
    if (geometry::cancelRequested()) {
        return refuse(std::move(result), ToolpathIssue::Cancelled, "cancelled");
    }
    result.diagnostics = std::move(shapes.diagnostics);
    if (!male && !shapes.female.empty()) {
        // The allowance only shrinks the plug: a female cut fine without it needs no warning.
        std::erase_if(result.diagnostics, [](const ToolpathDiagnostic& d) {
            return d.issue == ToolpathIssue::NoRegion && d.message == kAllowanceConsumes;
        });
    }
    const std::vector<Contour>& shape = male ? shapes.male : shapes.female;
    if (shape.empty()) {
        if (result.diagnostics.empty()) {
            result.diagnostics.push_back(diag(ToolpathIssue::NoRegion, "nothing to cut"));
        }
        return result;
    }

    ProfileParams pp;
    pp.name = params.name;
    pp.direction = params.direction;
    pp.safeZ = params.safeZ;
    pp.order = params.order;
    pp.startPosition = params.startPosition;
    pp.cuttingData = cuttingData;
    pp.tolerance = params.tolerance;

    // Appends one profile run; false when it made nothing (the whole piece is then refused).
    const auto absorb = [&](ProfileResult&& pr) {
        if (geometry::cancelRequested()) {
            result.diagnostics.push_back(diag(ToolpathIssue::Cancelled, "cancelled"));
            return false;
        }
        const bool ok = !pr.toolpath.isEmpty();
        if (result.toolpath.tool.name.empty() && result.toolpath.tool.diameter == 0.0) {
            result.toolpath.tool = pr.toolpath.tool;
        }
        result.toolpath.append(pr.toolpath);
        for (ToolpathDiagnostic& d : pr.diagnostics) {
            result.diagnostics.push_back(std::move(d));
        }
        return ok;
    };

    const auto finish = [&](bool ok) {
        if (!ok) {
            result.toolpath = Toolpath{};
        }
        result.toolpath.name = params.name;
        return std::move(result);
    };

    if (!male && params.femaleStyle == InlayFemaleStyle::Pocket) {
        // Grown by the chordal margin the pocket keeps at its walls, so the walls land on the shape.
        std::vector<Contour> grown = geometry::offsetContours(shape, params.tolerance.chordal, JoinType::Round, params.tolerance);
        PocketParams kp;
        kp.name = params.name;
        kp.startDepth = params.startDepth;
        kp.cutDepth = params.cutDepth;
        kp.safeZ = params.safeZ;
        kp.direction = params.direction;
        kp.tolerance = params.tolerance;
        if (geometry::cancelRequested()) {
            return refuse(std::move(result), ToolpathIssue::Cancelled, "cancelled");
        }
        if (spanCount(grown) > kMaxDerivedSpans) {
            return refuse(std::move(result), ToolpathIssue::WorkLimitExceeded,
                          "the offset shape has too many spans for an inlay (limit " + std::to_string(kMaxDerivedSpans) + ")");
        }
        PocketResult pr = pocketToolpath(grown.empty() ? shape : grown, {PocketTool{tool, cuttingData}}, kp);
        for (ToolpathDiagnostic& d : pr.diagnostics) {
            result.diagnostics.push_back(std::move(d));
        }
        if (geometry::cancelRequested()) {
            return refuse(std::move(result), ToolpathIssue::Cancelled, "cancelled");
        }
        if (!pr.toolpaths.empty()) {
            result.toolpath = std::move(pr.toolpaths.front());
            result.toolpath.name = params.name;
        }
        return result;
    }

    if (!male) {
        pp.side = ProfileSide::Inside;
        pp.passes.startDepth = params.startDepth;
        pp.passes.cutDepth = params.cutDepth;
        return finish(absorb(profileToolpath(shape, tool, pp)));
    }

    pp.side = ProfileSide::Outside;
    if (params.maleStyle == InlayMaleStyle::Straight) {
        pp.passes.startDepth = params.startDepth;
        pp.passes.cutDepth = params.cutDepth;
        return finish(absorb(profileToolpath(shape, tool, pp)));
    }

    // Stepped: the shoulder level at stepWidth out, then the band below it, outermost ring first.
    const double d = tool.diameter;
    std::vector<double> bands{0.0};
    while (bands.back() + d < params.stepWidth) {
        if (bands.size() >= kMaxBandPasses) {
            return refuse(std::move(result), ToolpathIssue::WorkLimitExceeded,
                          "the step is too wide for the tool: it needs more than " + std::to_string(kMaxBandPasses) +
                              " passes");
        }
        bands.push_back(bands.back() + kBandOverlap * d);
    }
    ProfileParams shoulder = pp;
    shoulder.passes.startDepth = params.startDepth;
    shoulder.passes.cutDepth = params.stepDepth;
    shoulder.allowance = params.stepWidth;
    if (!absorb(profileToolpath(shape, tool, shoulder))) {
        return finish(false);
    }
    for (auto it = bands.rbegin(); it != bands.rend(); ++it) {
        ProfileParams below = pp;
        below.passes.startDepth = params.startDepth + params.stepDepth;
        below.passes.cutDepth = params.cutDepth - params.stepDepth;
        below.allowance = *it;
        if (!absorb(profileToolpath(shape, tool, below))) {
            return finish(false);
        }
    }
    return finish(true);
}

}  // namespace carvetto::toolpath
