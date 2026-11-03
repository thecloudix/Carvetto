#include "carvetto/toolpath/engrave.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pass_plan.hpp"
#include "carvetto/toolpath/path_walk.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;

// Work caps (as Pocket's and Drill's, ADR-0017 §12): refused with a message before any moves.
constexpr double kMinChordal = 1e-4;      // mm
constexpr int kMaxPasses = 1000;          // passes per toolpath
constexpr double kMaxMoves = 20000000.0;  // moves per toolpath

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

std::string vecName(std::size_t i) {
    return "vector " + std::to_string(i);
}

/// Every parameter finite and in range; an empty string when valid.
std::string paramProblem(const EngraveParams& p) {
    const double values[] = {p.depth,           p.safeZ,           p.stepover,           p.rasterAngle,
                             p.startPosition.x, p.startPosition.y, p.tolerance.distance, p.tolerance.chordal,
                             p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!(p.depth > 0.0)) {
        return "engrave depth must be greater than zero";
    }
    if (p.passes < 1 || p.passes > kMaxPasses) {
        return "number of passes must be between 1 and " + std::to_string(kMaxPasses);
    }
    if (!(p.safeZ > 0.0)) {
        return "safe Z must be above the material surface";
    }
    if (p.stepover < 0.0) {
        return "stepover must not be negative";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance > 0.0) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal";
    }
    return {};
}

const char* toolTypeText(tooling::ToolType type) {
    switch (type) {
        case tooling::ToolType::EndMill: return "an end mill";
        case tooling::ToolType::BallNose: return "a ball nose";
        case tooling::ToolType::VBit: return "a V-bit";
        case tooling::ToolType::Engraving: return "an engraving tool";
        case tooling::ToolType::TaperedBallNose: return "a tapered ball nose";
        case tooling::ToolType::RadiusedEndMill: return "a radiused end mill";
        case tooling::ToolType::FormTool: return "a form tool";
        case tooling::ToolType::DiamondDrag: return "a diamond drag tool";
        case tooling::ToolType::Drill: return "a drill";
    }
    return "an unknown tool type";
}

/// What is wrong with the tool for this toolpath; empty when usable.
std::string toolProblem(const tooling::Tool& tool, const PassTool& pt) {
    const std::string label = "tool '" + tool.name + "'";
    if (tool.type == tooling::ToolType::DiamondDrag) {
        return label + " is a diamond drag tool; diamond-drag tools need spindle-off support, not yet available";
    }
    if (!engraveToolAllowed(tool.type)) {
        return label + " is " + toolTypeText(tool.type) +
               "; quick engraving needs a V-bit, an engraving tool, an end mill or a ball nose";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return label + ": " + *bad;
    }
    if (!std::isfinite(pt.radius) || !(pt.radius > 0.0)) {
        return label + " has no cutting width at the engrave depth";
    }
    const ToolSnapshot& s = pt.snapshot;
    if (!std::isfinite(s.plungeRate) || !(s.plungeRate > 0.0)) {
        return label + ": the plunge rate must be greater than zero";
    }
    if (!std::isfinite(s.feedRate) || !(s.feedRate > 0.0)) {
        return label + ": the feed rate must be greater than zero";
    }
    return {};
}

void outline(const std::vector<Contour>& contours, const EngraveParams& params, const std::vector<double>& levels,
             EngraveResult& result) {
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    const geometry::Tolerance& tol = params.tolerance;
    std::vector<ArcLengthPath> paths;
    std::vector<OrderItem> items;
    double estimate = 0.0;
    const bool even = levels.size() % 2 == 0;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const Contour& c = contours[i];
        if (c.isEmpty()) {
            say(ToolpathIssue::EmptyContour, i, vecName(i) + " has no spans; skipped");
            continue;
        }
        if (!c.isContinuous(tol)) {
            say(ToolpathIssue::DiscontinuousContour, i, vecName(i) + " has gaps between spans; skipped");
            continue;
        }
        ArcLengthPath path(c, tol);
        if (path.isEmpty() || !(path.length() > tol.distance)) {
            say(ToolpathIssue::EmptyContour, i, vecName(i) + " has no length; skipped");
            continue;
        }
        estimate += 2.0 + static_cast<double>(levels.size()) * (static_cast<double>(path.spanCount()) + 1.0);
        OrderItem item;
        item.path = c;
        item.selectionIndex = i;
        item.start = path.pointAt(0.0);
        // An open vector cut an even number of times ends back at its start.
        item.end = path.closed() || even ? item.start : path.pointAt(path.length());
        items.push_back(std::move(item));
        paths.push_back(std::move(path));
    }
    if (items.empty()) {
        if (result.diagnostics.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to engrave");
        }
        return;
    }
    if (!(estimate <= kMaxMoves)) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "the toolpath would have more than " + num(kMaxMoves) + " moves (too many vectors, spans or passes)");
        return;
    }
    const std::vector<std::size_t> order = orderCuts(items, OrderOptions{}, params.startPosition, tol);
    const double safeZ = params.safeZ;
    for (const std::size_t k : order) {
        const ArcLengthPath& path = paths[k];
        const double length = path.length();
        const Vec2 start = path.pointAt(0.0);
        result.toolpath.moves.push_back(Move::rapid({start.x, start.y, safeZ}));
        bool forward = true;
        for (const double z : levels) {
            // appendLeg plunges straight down to z where the tool stands, then cuts along the path.
            if (forward) {
                appendLeg(path, 0.0, length, ZProfile::flat(z), result.toolpath);
            } else {
                appendLeg(path, length, 0.0, ZProfile::flat(z), result.toolpath);
            }
            if (!path.closed()) {
                forward = !forward;
            }
        }
        const Vec3 end = result.toolpath.moves.back().to;
        result.toolpath.moves.push_back(Move::retract({end.x, end.y, safeZ}));
    }
}

void replaceAll(std::string& text, const std::string& from, const std::string& to) {
    for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
        text.replace(at, from.size(), to);
    }
}

/// Pocket's diagnostics in the words of an engraving: "tool 1 (name)" becomes "tool 'name'" and
/// the pocket becomes the fill area (ADR-0024 §3).
std::string fillWording(std::string message, const std::string& toolName) {
    replaceAll(message, "tool 1 (" + toolName + ")", "tool '" + toolName + "'");
    replaceAll(message, "invalid pocket parameters", "invalid fill parameters");
    replaceAll(message, "at the pocket depth", "at the engrave depth");
    replaceAll(message, "a pocket needs closed vectors", "filling needs closed vectors");
    replaceAll(message, "region to pocket", "region to fill");
    replaceAll(message, "pocket size", "fill area");
    replaceAll(message, "pocket boundary", "fill boundary");
    replaceAll(message, "the pocket", "the fill area");
    replaceAll(message, "pocket", "fill area");
    return message;
}

void fill(const std::vector<Contour>& contours, const tooling::Tool& tool, const tooling::CuttingData& data,
          const PassTool& pt, const EngraveParams& params, EngraveResult& result) {
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    const double width = 2.0 * pt.radius;
    const double stepover = params.stepover > 0.0 ? params.stepover : data.stepover;
    const std::string label = "tool '" + tool.name + "'";
    if (!std::isfinite(stepover) || !(stepover > 0.0)) {
        say(ToolpathIssue::InvalidTool, std::nullopt,
            label + ": filling needs a stepover greater than zero (in the parameters or the cutting data)");
        return;
    }
    if (stepover > width * (1.0 + 1e-12)) {
        say(ToolpathIssue::InvalidTool, std::nullopt,
            label + ": the stepover " + num(stepover) + " mm is wider than the " + num(width) +
                " mm the tool cuts at the engrave depth of " + num(params.depth) + " mm");
        return;
    }

    std::vector<Contour> closed;
    std::size_t openCount = 0;
    std::optional<std::size_t> firstOpen;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const Contour& c = contours[i];
        if (c.isEmpty()) {
            continue;  // encloses nothing
        }
        if (!c.closed) {
            ++openCount;
            if (!firstOpen) {
                firstOpen = i;
            }
            continue;
        }
        closed.push_back(c);
    }
    if (openCount > 0) {
        say(ToolpathIssue::OpenContourIgnored, firstOpen,
            std::to_string(openCount) + " open vector(s) ignored: filling needs closed vectors");
    }

    // Pocket's machinery does the filling (AC2): one tool, the engrave depth as the cut depth in
    // `passes` equal passes, the boundary pass last so the edges of the marking are clean.
    tooling::CuttingData fillData = data;
    fillData.stepover = stepover;
    fillData.passDepth = params.depth / static_cast<double>(params.passes);
    PocketParams pocket;
    pocket.name = params.name;
    pocket.startDepth = 0.0;
    pocket.cutDepth = params.depth;
    pocket.maintainExactStep = true;
    pocket.lastPassThickness = 0.0;
    pocket.safeZ = params.safeZ;
    pocket.strategy = params.fillPattern;
    pocket.direction = CutDirection::Climb;
    pocket.rasterAngle = params.rasterAngle;
    pocket.profilePass = ProfilePass::Last;
    pocket.allowance = 0.0;
    pocket.rampPlunge = false;
    pocket.tolerance = params.tolerance;
    PocketResult r = pocketToolpath(closed, {PocketTool{tool, fillData}}, pocket);
    for (ToolpathDiagnostic& d : r.diagnostics) {
        d.tool.reset();
        d.message = fillWording(std::move(d.message), tool.name);
        result.diagnostics.push_back(std::move(d));
    }
    if (!r.toolpaths.empty()) {
        result.toolpath.moves = std::move(r.toolpaths.front().moves);
    }
    if (result.toolpath.isEmpty() && result.diagnostics.empty()) {
        say(ToolpathIssue::NoRegion, std::nullopt, "no closed region to fill");
    }
}

}  // namespace

bool engraveToolAllowed(tooling::ToolType type) {
    switch (type) {
        case tooling::ToolType::VBit:
        case tooling::ToolType::Engraving:
        case tooling::ToolType::EndMill:
        case tooling::ToolType::BallNose: return true;
        // Owner decision 2026-11-03 (ADR-0024 §6): refused until toolpaths can run with the
        // spindle off.
        case tooling::ToolType::DiamondDrag:
        case tooling::ToolType::TaperedBallNose:
        case tooling::ToolType::RadiusedEndMill:
        case tooling::ToolType::FormTool:
        case tooling::ToolType::Drill: return false;
    }
    return false;
}

std::vector<double> engraveLevels(double depth, int passes) {
    if (!std::isfinite(depth) || !(depth > 0.0) || passes < 1 || passes > kMaxPasses) {
        return {};
    }
    // The same plan Fill hands to Pocket: exact steps of depth / passes, the last at the bottom.
    PassPlan plan;
    plan.startDepth = 0.0;
    plan.cutDepth = depth;
    plan.passDepth = depth / static_cast<double>(passes);
    plan.maintainExactStep = true;
    return plan.levels();
}

EngraveResult engraveToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData, const EngraveParams& params) {
    EngraveResult result;
    result.toolpath.name = params.name;
    if (const std::string bad = paramProblem(params); !bad.empty()) {
        result.diagnostics.push_back(
            ToolpathDiagnostic{ToolpathIssue::InvalidParameters, std::nullopt, bad, std::nullopt});
        return result;
    }
    const tooling::CuttingData& data = effectiveCuttingData(tool, cuttingData);
    const PassTool pt = passTool(tool, data, params.depth);
    result.toolpath.tool = pt.snapshot;
    if (const std::string bad = toolProblem(tool, pt); !bad.empty()) {
        result.diagnostics.push_back(ToolpathDiagnostic{ToolpathIssue::InvalidTool, std::nullopt, bad, std::nullopt});
        return result;
    }
    if (params.strategy == EngraveStrategy::Fill) {
        fill(contours, tool, data, pt, params, result);
    } else {
        outline(contours, params, engraveLevels(params.depth, params.passes), result);
    }
    return result;
}

}  // namespace carvetto::toolpath
