#include "carvetto/toolpath/fluting.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <utility>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;

constexpr double kMinChordal = 1e-4;
constexpr double kMaxLength = 100000.0;   // depths and lengths (mm)
constexpr double kMaxMoves = 20000000.0;  // moves per toolpath
constexpr double kMaxPasses = 1000.0;
constexpr std::size_t kSmoothSteps = 32;
constexpr double kEps = 1e-9;

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

std::string vecName(std::size_t i) {
    return "vector " + std::to_string(i);
}

double ramp(FluteRampType type, double t) {
    t = std::clamp(t, 0.0, 1.0);
    return type == FluteRampType::Smooth ? t * t * (3.0 - 2.0 * t) : t;
}

std::string check(const tooling::Tool& tool, const FlutingParams& p, ToolpathIssue& issue) {
    issue = ToolpathIssue::InvalidParameters;
    const double values[] = {p.fluteDepth,   p.rampLength,       p.rampPercent,       p.safeZ,
                             p.startPosition.x, p.startPosition.y, p.tolerance.distance, p.tolerance.chordal,
                             p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!(p.fluteDepth > 0.0) || p.fluteDepth > kMaxLength) {
        return "flute depth must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (p.type != FluteType::RampOverLength) {
        if (p.rampBasis == FluteRampBasis::Length && (!(p.rampLength > 0.0) || p.rampLength > kMaxLength)) {
            return "ramp length must be between 0 and " + num(kMaxLength) + " mm";
        }
        if (p.rampBasis == FluteRampBasis::Percent && (!(p.rampPercent > 0.0) || p.rampPercent > 100.0)) {
            return "ramp percentage must be greater than 0 and at most 100";
        }
    }
    if (!(p.safeZ > 0.0) || p.safeZ > kMaxLength) {
        return "safe Z must be above the material surface";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance > 0.0) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal";
    }
    if (!flutingToolAllowed(tool.type)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "' must be an end mill or a ball nose tool for fluting";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "': " + *bad;
    }
    return {};
}

/// A point of the flute: position, arc length and profile depth.
struct Sample {
    Vec2 p;
    double s{0.0};
    double depth{0.0};
};

/// Splits every segment of `v` that crosses `level` (strictly) at the crossing.
std::vector<Sample> splitAt(const std::vector<Sample>& v, double level) {
    std::vector<Sample> out;
    out.reserve(v.size() + 4);
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i > 0) {
            const Sample& a = v[i - 1];
            const Sample& b = v[i];
            if ((a.depth - level) * (b.depth - level) < 0.0) {
                const double t = (level - a.depth) / (b.depth - a.depth);
                out.push_back({{a.p.x + (b.p.x - a.p.x) * t, a.p.y + (b.p.y - a.p.y) * t},
                               a.s + (b.s - a.s) * t,
                               level});
            }
        }
        out.push_back(v[i]);
    }
    return out;
}

}  // namespace

bool flutingToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::EndMill || type == tooling::ToolType::BallNose;
}

double flutingRampLength(const FlutingParams& params, double totalLength) {
    if (!std::isfinite(totalLength) || !(totalLength > 0.0)) {
        return 0.0;
    }
    if (params.type == FluteType::RampOverLength) {
        return totalLength;
    }
    const double wanted = params.rampBasis == FluteRampBasis::Length ? params.rampLength
                                                                      : totalLength * params.rampPercent / 100.0;
    if (!std::isfinite(wanted) || !(wanted > 0.0)) {
        return 0.0;
    }
    const double cap = params.type == FluteType::RampAtStartAndEnd ? 0.5 * totalLength : totalLength;
    return std::min(wanted, cap);
}

double flutingDepthAt(const FlutingParams& params, double totalLength, double s) {
    const double r = flutingRampLength(params, totalLength);
    if (!(r > 0.0) || !std::isfinite(s) || !std::isfinite(params.fluteDepth)) {
        return 0.0;
    }
    s = std::clamp(s, 0.0, totalLength);
    double f = 1.0;
    if (s < r) {
        f = ramp(params.rampType, s / r);
    } else if (params.type == FluteType::RampAtStartAndEnd && s > totalLength - r) {
        f = ramp(params.rampType, (totalLength - s) / r);
    }
    return params.fluteDepth * f;
}

FlutingResult flutingToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData, const FlutingParams& params) {
    FlutingResult result;
    result.toolpath.name = params.name;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    ToolpathIssue issue{};
    if (const std::string bad = check(tool, params, issue); !bad.empty()) {
        say(issue, std::nullopt, bad);
        return result;
    }
    const geometry::Tolerance& tol = params.tolerance;
    const PassTool pt = passTool(tool, effectiveCuttingData(tool, cuttingData), params.fluteDepth);
    result.toolpath.tool = pt.snapshot;
    if (!std::isfinite(pt.snapshot.plungeRate) || !(pt.snapshot.plungeRate > 0.0) ||
        !std::isfinite(pt.snapshot.feedRate) || !(pt.snapshot.feedRate > 0.0) || !std::isfinite(pt.passDepth) ||
        !(pt.passDepth > 0.0)) {
        say(ToolpathIssue::InvalidTool, std::nullopt,
            "tool '" + tool.name + "': the feed rate, plunge rate and pass depth must be greater than zero");
        return result;
    }
    const double passCount = std::ceil(params.fluteDepth / pt.passDepth - kEps);
    if (!(passCount <= kMaxPasses)) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "more than " + num(kMaxPasses) + " passes (flute depth " + num(params.fluteDepth) +
                " mm, pass depth " + num(pt.passDepth) + " mm)");
        return result;
    }
    const auto passes = static_cast<std::size_t>(std::max(1.0, passCount));

    struct Flute {
        std::vector<Sample> samples;
        std::size_t contour{0};
        double length{0.0};
        double ramp{0.0};
    };
    std::vector<Flute> flutes;
    double moveEstimate = 0.0;
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
        geometry::Polyline pl = c.polygonise(tol);
        std::vector<Vec2> pts = pl.points;
        if (c.closed && !pts.empty()) {
            pts.push_back(pts.front());
        }
        bool finite = true;
        for (const Vec2 q : pts) {
            finite = finite && std::isfinite(q.x) && std::isfinite(q.y);
        }
        if (!finite) {
            say(ToolpathIssue::InvalidParameters, i, vecName(i) + " has a coordinate that is not finite; skipped");
            continue;
        }
        std::vector<double> cum(pts.size(), 0.0);
        for (std::size_t k = 1; k < pts.size(); ++k) {
            cum[k] = cum[k - 1] + geometry::distance(pts[k - 1], pts[k]);
        }
        const double total = pts.empty() ? 0.0 : cum.back();
        if (!std::isfinite(total) || total <= tol.distance) {
            say(ToolpathIssue::NoRegion, i, vecName(i) + " has no length to flute; skipped");
            continue;
        }
        const double rampLen = flutingRampLength(params, total);
        const double wanted = params.type == FluteType::RampOverLength
                                  ? rampLen
                                  : (params.rampBasis == FluteRampBasis::Length ? params.rampLength
                                                                                : total * params.rampPercent / 100.0);
        if (wanted > rampLen * (1.0 + 1e-12)) {
            say(ToolpathIssue::LeadShortened, i,
                vecName(i) + " is too short for the ramp (" + num(wanted) + " mm); shortened to " + num(rampLen) +
                    " mm");
        }
        // Positions along the vector: the vertices, the ends of the ramps and the steps of smooth ramps.
        std::vector<double> pos(cum);
        const bool both = params.type == FluteType::RampAtStartAndEnd;
        pos.push_back(rampLen);
        if (both) {
            pos.push_back(total - rampLen);
        }
        if (params.rampType == FluteRampType::Smooth) {
            for (std::size_t k = 1; k < kSmoothSteps; ++k) {
                const double f = static_cast<double>(k) / static_cast<double>(kSmoothSteps);
                pos.push_back(rampLen * f);
                if (both) {
                    pos.push_back(total - rampLen * f);
                }
            }
        }
        std::sort(pos.begin(), pos.end());
        std::vector<double> uniq;
        for (const double s : pos) {
            if (s >= 0.0 && s <= total && (uniq.empty() || s - uniq.back() > kEps)) {
                uniq.push_back(s);
            }
        }
        moveEstimate += static_cast<double>(uniq.size() + 3) * static_cast<double>(passes);
        if (!(moveEstimate <= kMaxMoves)) {
            say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
                "the toolpath would have more than " + num(kMaxMoves) + " moves");
            return result;
        }
        Flute f;
        f.contour = i;
        f.length = total;
        f.ramp = rampLen;
        f.samples.reserve(uniq.size());
        std::size_t seg = 0;
        for (const double s : uniq) {
            while (seg + 2 < pts.size() && cum[seg + 1] < s) {
                ++seg;
            }
            const double span = cum[seg + 1] - cum[seg];
            const double t = span > 0.0 ? std::clamp((s - cum[seg]) / span, 0.0, 1.0) : 0.0;
            const Vec2 q{pts[seg].x + (pts[seg + 1].x - pts[seg].x) * t, pts[seg].y + (pts[seg + 1].y - pts[seg].y) * t};
            f.samples.push_back({q, s, flutingDepthAt(params, total, s)});
        }
        flutes.push_back(std::move(f));
    }
    if (flutes.empty()) {
        if (result.diagnostics.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to flute");
        }
        return result;
    }

    std::vector<OrderItem> items;
    items.reserve(flutes.size());
    for (const Flute& f : flutes) {
        OrderItem item;
        item.path = contours[f.contour];
        item.path.closed = false;
        item.selectionIndex = f.contour;
        item.start = f.samples.front().p;
        item.end = f.samples.back().p;
        items.push_back(std::move(item));
    }
    const std::vector<std::size_t> order = orderCuts(items, params.order, params.startPosition, tol);
    std::vector<Move>& moves = result.toolpath.moves;
    for (const std::size_t k : order) {
        const Flute& f = flutes[k];
        result.flutes.push_back({f.contour, f.length, f.ramp});
        for (std::size_t pass = 1; pass <= passes; ++pass) {
            const double prev = static_cast<double>(pass - 1) * pt.passDepth;
            const double level = pass < passes ? static_cast<double>(pass) * pt.passDepth : params.fluteDepth;
            std::vector<Sample> v = f.samples;
            if (pass > 1) {
                v = splitAt(v, prev);
            }
            if (pass < passes) {
                v = splitAt(v, level);
            }
            const auto z = [&](const Sample& s) { return -std::min(s.depth, level); };
            bool inRun = false;
            for (std::size_t i = 0; i + 1 < v.size(); ++i) {
                const bool active = std::max(v[i].depth, v[i + 1].depth) > prev + kEps;
                if (active && !inRun) {
                    moves.push_back(Move::rapid({v[i].p.x, v[i].p.y, params.safeZ}));
                    moves.push_back(Move::plunge({v[i].p.x, v[i].p.y, z(v[i])}));
                    inRun = true;
                }
                if (active) {
                    moves.push_back(Move::feed({v[i + 1].p.x, v[i + 1].p.y, z(v[i + 1])}));
                }
                const bool nextActive = i + 2 < v.size() && std::max(v[i + 1].depth, v[i + 2].depth) > prev + kEps;
                if (inRun && !nextActive) {
                    moves.push_back(Move::retract({v[i + 1].p.x, v[i + 1].p.y, params.safeZ}));
                    inRun = false;
                }
            }
        }
    }
    return result;
}

}  // namespace carvetto::toolpath
