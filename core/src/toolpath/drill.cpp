#include "carvetto/toolpath/drill.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pass_plan.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;

// Work caps (as Pocket's, ADR-0017 §12): refused with a message before any moves are made.
constexpr double kMinChordal = 1e-4;     // mm
constexpr double kMaxLevels = 10000.0;   // passes per hole
constexpr double kMaxMoves = 20000000.0; // moves per toolpath

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

std::string vecName(std::size_t i) {
    return "vector " + std::to_string(i);
}

/// Area and first moments of a closed contour about the origin, by Green's theorem:
/// A = 1/2 ∮ (x dy - y dx), Mx = 1/3 ∮ x (x dy - y dx), My = 1/3 ∮ y (x dy - y dx).
struct Moments {
    double area{0.0};
    double mx{0.0};
    double my{0.0};

    void addLine(Vec2 a, Vec2 b) {
        const double c = a.cross(b);
        area += 0.5 * c;
        mx += c * (a.x + b.x) / 6.0;
        my += c * (a.y + b.y) / 6.0;
    }

    // x = cx + r cos t, y = cy + r sin t: x y' - y x' = r^2 + r cx cos t + r cy sin t; the products
    // with x and y are trigonometric polynomials of degree 2, integrated in closed form.
    void addArc(Vec2 c, double r, double a0, double sweep) {
        const double a1 = a0 + sweep;
        const double ic = std::sin(a1) - std::sin(a0);
        const double is = std::cos(a0) - std::cos(a1);
        const double s2 = 0.25 * (std::sin(2.0 * a1) - std::sin(2.0 * a0));
        const double icc = 0.5 * sweep + s2;
        const double iss = 0.5 * sweep - s2;
        const double isc = 0.5 * (std::sin(a1) * std::sin(a1) - std::sin(a0) * std::sin(a0));
        const double r2 = r * r;
        area += 0.5 * (r2 * sweep + r * c.x * ic + r * c.y * is);
        mx += (c.x * r2 * sweep + (r * c.x * c.x + r2 * r) * ic + r * c.x * c.y * is + r2 * c.x * icc +
               r2 * c.y * isc) /
              3.0;
        my += (c.y * r2 * sweep + r * c.x * c.y * ic + (r * c.y * c.y + r2 * r) * is + r2 * c.x * isc +
               r2 * c.y * iss) /
              3.0;
    }

    // x(t) y'(t) - y(t) x'(t) has degree 5 and its products with x and y degree 8: a 5-point
    // Gauss-Legendre rule (exact to degree 9) integrates all three exactly.
    void addBezier(const geometry::BezierSpan& b) {
        constexpr std::array<double, 5> nodes{-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831,
                                              0.9061798459386640};
        constexpr std::array<double, 5> weights{0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                                                0.4786286704993665, 0.2369268850561891};
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const double t = 0.5 * (nodes[i] + 1.0);
            const double w = 0.5 * weights[i];
            const Vec2 p = b.pointAt(t);
            const double f = p.cross(b.derivativeAt(t));
            area += 0.5 * w * f;
            mx += w * p.x * f / 3.0;
            my += w * p.y * f / 3.0;
        }
    }
};

geometry::Span translated(const geometry::Span& s, Vec2 by) {
    return std::visit(
        [by](const auto& v) -> geometry::Span {
            using T = std::decay_t<decltype(v)>;
            T out = v;
            if constexpr (std::is_same_v<T, geometry::LineSpan>) {
                out.start = v.start - by;
                out.end = v.end - by;
            } else if constexpr (std::is_same_v<T, geometry::ArcSpan>) {
                out.centre = v.centre - by;
            } else {
                out.p0 = v.p0 - by;
                out.p1 = v.p1 - by;
                out.p2 = v.p2 - by;
                out.p3 = v.p3 - by;
            }
            return out;
        },
        s);
}

/// Every parameter finite, depths and times in range; an empty string when valid.
std::string paramProblem(const DrillParams& p) {
    const double values[] = {p.startDepth,         p.cutDepth,         p.safeZ,
                             p.retractGap,         p.dwellTime,        p.startPosition.x,
                             p.startPosition.y,    p.tolerance.distance, p.tolerance.chordal,
                             p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!(p.cutDepth > 0.0)) {
        return "cut depth must be greater than zero";
    }
    if (p.startDepth < 0.0) {
        return "start depth must not be negative";
    }
    if (!(p.safeZ > -p.startDepth)) {
        return "safe Z must be above the start of the cut";
    }
    if (p.dwellTime < 0.0) {
        return "dwell time must not be negative";
    }
    if (p.peck && !(p.retractGap > 0.0)) {
        return "peck drilling needs a retract gap greater than zero";
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
std::string toolProblem(const tooling::Tool& tool, const PassTool& pt, const DrillParams& p) {
    const std::string label = "tool '" + tool.name + "'";
    if (!drillToolAllowed(tool.type)) {
        return label + " is " + toolTypeText(tool.type) + "; drilling needs a drill or an end mill";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return label + ": " + *bad;
    }
    if (!std::isfinite(pt.radius) || !(pt.radius > 0.0)) {
        return label + " has no cutting width at the hole depth";
    }
    const ToolSnapshot& s = pt.snapshot;
    if (!std::isfinite(s.plungeRate) || !(s.plungeRate > 0.0)) {
        return label + ": the plunge rate must be greater than zero";
    }
    if (!std::isfinite(s.feedRate) || !(s.feedRate > 0.0)) {
        return label + ": the feed rate must be greater than zero";
    }
    if (p.peck && (!std::isfinite(pt.passDepth) || !(pt.passDepth > 0.0))) {
        return label + ": peck drilling needs a pass depth greater than zero in the cutting data";
    }
    return {};
}

bool centreInside(const Contour& c, Vec2 centre, const geometry::Tolerance& tol) {
    switch (geometry::locatePoint(c, centre, geometry::FillRule::EvenOdd, tol)) {
        case geometry::PointLocation::Inside: return true;
        case geometry::PointLocation::Outside: return false;
        case geometry::PointLocation::OnBoundary: break;
    }
    // Within the distance tolerance of the vector (a vector barely wider than the tolerance):
    // the crossing count decides.
    return geometry::windingNumber(c, centre, tol) % 2 != 0;
}

}  // namespace

bool drillToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::Drill || type == tooling::ToolType::EndMill;
}

std::optional<Vec2> drillCentre(const Contour& contour, const geometry::Tolerance& tol) {
    if (!contour.closed || contour.isEmpty()) {
        return std::nullopt;
    }
    // Moments about the bounding-box centre, so far-away coordinates do not cancel.
    const geometry::Box2 box = contour.bounds();
    const Vec2 ref = box.centre();
    if (!std::isfinite(ref.x) || !std::isfinite(ref.y)) {
        return std::nullopt;
    }
    Moments m;
    for (const geometry::Span& raw : contour.spans) {
        const geometry::Span s = translated(raw, ref);
        if (const auto* line = std::get_if<geometry::LineSpan>(&s)) {
            m.addLine(line->start, line->end);
        } else if (const auto* arc = std::get_if<geometry::ArcSpan>(&s)) {
            m.addArc(arc->centre, arc->radius, arc->startAngle, arc->sweepAngle());
        } else {
            m.addBezier(std::get<geometry::BezierSpan>(s));
        }
    }
    // A contour whose last span does not end exactly on its start is closed by a straight line
    // (as in `geometry::signedArea`).
    m.addLine(contour.endPoint() - ref, contour.startPoint() - ref);
    if (!std::isfinite(m.area) || !(std::fabs(m.area) > tol.distance * tol.distance)) {
        return std::nullopt;
    }
    // Offsets from the box centre at rounding level (a symmetric shape: circle, rectangle, slot,
    // regular polygon) are rounding noise; dropping them gives such shapes their exact centre.
    Vec2 offset{m.mx / m.area, m.my / m.area};
    const double noise = 1e-12 * std::max(box.width(), box.height());
    if (std::fabs(offset.x) <= noise) {
        offset.x = 0.0;
    }
    if (std::fabs(offset.y) <= noise) {
        offset.y = 0.0;
    }
    const Vec2 c = ref + offset;
    if (!std::isfinite(c.x) || !std::isfinite(c.y)) {
        return std::nullopt;
    }
    return c;
}

DrillResult drillToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                          const std::optional<tooling::CuttingData>& cuttingData, const DrillParams& params) {
    DrillResult result;
    result.toolpath.name = params.name;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };

    if (const std::string bad = paramProblem(params); !bad.empty()) {
        say(ToolpathIssue::InvalidParameters, std::nullopt, bad);
        return result;
    }
    const geometry::Tolerance& tol = params.tolerance;
    const double bottomDepth = params.startDepth + params.cutDepth;
    const PassTool pt = passTool(tool, effectiveCuttingData(tool, cuttingData), bottomDepth);
    result.toolpath.tool = pt.snapshot;
    if (const std::string bad = toolProblem(tool, pt, params); !bad.empty()) {
        say(ToolpathIssue::InvalidTool, std::nullopt, bad);
        return result;
    }

    PassPlan plan;
    plan.startDepth = params.startDepth;
    plan.cutDepth = params.cutDepth;
    plan.passDepth = params.peck ? pt.passDepth : 0.0;  // 0: one pass to the bottom
    plan.maintainExactStep = true;
    // The cap is on the passes PassPlan actually makes: exactly kMaxLevels passes are allowed,
    // one more is refused (T-0143). The estimate ceil(cut/pass) can exceed the true count by one
    // (PassPlan's relative 1e-9 round-off guard), so it only screens out plans far too large to build.
    const auto tooManyPasses = [&] {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "more than " + num(kMaxLevels) + " passes per hole (pass depth " + num(pt.passDepth) +
                " mm too small for the cut depth)");
    };
    if (params.peck && !(std::ceil(params.cutDepth / pt.passDepth) <= kMaxLevels + 1.0)) {
        tooManyPasses();
        return result;
    }
    const std::vector<double> levels = plan.levels();
    if (static_cast<double>(levels.size()) > kMaxLevels) {
        tooManyPasses();
        return result;
    }
    if (levels.empty()) {
        say(ToolpathIssue::InvalidParameters, std::nullopt, "the cut depth yields no passes");
        return result;
    }

    // Holes: the centre of every usable closed vector.
    std::vector<DrillHole> candidates;
    std::size_t openCount = 0;
    std::optional<std::size_t> firstOpen;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const Contour& c = contours[i];
        if (c.isEmpty()) {
            say(ToolpathIssue::EmptyContour, i, vecName(i) + " has no spans; skipped");
            continue;
        }
        if (!c.closed) {
            ++openCount;
            if (!firstOpen) {
                firstOpen = i;
            }
            continue;
        }
        if (!c.isContinuous(tol)) {
            say(ToolpathIssue::DiscontinuousContour, i, vecName(i) + " has gaps between spans; skipped");
            continue;
        }
        const std::optional<Vec2> centre = drillCentre(c, tol);
        if (!centre) {
            say(ToolpathIssue::NoRegion, i, vecName(i) + " encloses no area, so it has no centre; skipped");
            continue;
        }
        if (!centreInside(c, *centre, tol)) {
            say(ToolpathIssue::CentreOutsideVector, i,
                vecName(i) + ": its centre (" + num(centre->x) + ", " + num(centre->y) +
                    ") is not inside it; not drilled");
            continue;
        }
        const double room = geometry::distanceTo(c, *centre);
        if (room < pt.radius - tol.distance) {
            say(ToolpathIssue::HoleWiderThanVector, i,
                vecName(i) + ": the " + num(2.0 * pt.radius) + " mm hole is wider than the vector round its centre (" +
                    num(room) + " mm from the centre to the vector); drilled anyway");
        }
        candidates.push_back({*centre, i});
    }
    if (openCount > 0) {
        say(ToolpathIssue::OpenContourIgnored, firstOpen,
            std::to_string(openCount) + " open vector(s) ignored: drilling needs closed vectors");
    }

    // Coincident holes (e.g. concentric circles of a counterbore drawing) are drilled once: a hole
    // within the distance tolerance of an earlier kept hole is dropped.
    std::vector<geometry::Box2> centreBoxes;
    centreBoxes.reserve(candidates.size());
    for (const DrillHole& h : candidates) {
        centreBoxes.push_back(geometry::Box2{h.centre, h.centre});
    }
    const BoxIndex centreIndex(std::move(centreBoxes));
    std::vector<bool> kept(candidates.size(), false);
    std::vector<OrderItem> items;
    std::vector<std::size_t> itemHole;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const Vec2 c = candidates[i].centre;
        const Vec2 d{tol.distance, tol.distance};
        std::optional<std::size_t> same;
        for (const std::size_t j : centreIndex.query(geometry::Box2{c - d, c + d})) {
            if (j < i && kept[j] && geometry::distance(candidates[j].centre, c) <= tol.distance) {
                same = j;
                break;
            }
        }
        if (same) {
            say(ToolpathIssue::DuplicateHole, candidates[i].contour,
                vecName(candidates[i].contour) + " has the same centre as " + vecName(candidates[*same].contour) +
                    "; the hole is drilled once");
            continue;
        }
        kept[i] = true;
        // The vector itself (as an open path, so no hole "contains" another) gives the box the
        // left-to-right, bottom-to-top and grid orders sort by; the hole starts and ends at the centre.
        OrderItem item;
        item.path = contours[candidates[i].contour];
        item.path.closed = false;
        item.selectionIndex = candidates[i].contour;
        item.start = c;
        item.end = c;
        items.push_back(std::move(item));
        itemHole.push_back(i);
    }
    if (items.empty()) {
        if (result.diagnostics.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to drill");
        }
        return result;
    }

    const double perHole = 2.0 + static_cast<double>(levels.size()) * (params.dwellTime > 0.0 ? 3.0 : 2.0);
    if (!(perHole * static_cast<double>(items.size()) <= kMaxMoves)) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "the toolpath would have more than " + num(kMaxMoves) + " moves (too many holes or passes)");
        return result;
    }

    const std::vector<std::size_t> order = orderCuts(items, params.order, params.startPosition, tol);
    const double startZ = -params.startDepth;
    const double safeZ = params.safeZ;
    std::vector<Move>& moves = result.toolpath.moves;
    moves.reserve(static_cast<std::size_t>(perHole) * order.size());
    for (const std::size_t k : order) {
        const DrillHole& hole = candidates[itemHole[k]];
        result.holes.push_back(hole);
        const double x = hole.centre.x;
        const double y = hole.centre.y;
        moves.push_back(Move::rapid({x, y, safeZ}));
        for (std::size_t lv = 0; lv < levels.size(); ++lv) {
            const double z = levels[lv];
            moves.push_back(Move::plunge({x, y, z}));
            if (params.dwellTime > 0.0) {
                moves.push_back(Move::dwell({x, y, z}, params.dwellTime));
            }
            if (lv + 1 < levels.size()) {
                const double above = params.peckRetract == PeckRetract::AboveStart ? startZ : z;
                moves.push_back(Move::retract({x, y, std::min(above + params.retractGap, safeZ)}));
            }
        }
        moves.push_back(Move::retract({x, y, safeZ}));
    }
    return result;
}

}  // namespace carvetto::toolpath
