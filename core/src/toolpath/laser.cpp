#include "carvetto/toolpath/laser.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pass_plan.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;

constexpr double kMinChordal = 1e-4;
constexpr double kMaxLength = 100000.0;
constexpr double kMaxOffset = 1000.0;
constexpr int kMaxRepeats = 1000;
constexpr double kMaxPasses = 1000.0;
constexpr double kMaxMoves = 20000000.0;
constexpr double kMaxLines = 2000000.0;        // hatch lines per angle
constexpr double kMaxCrossings = 50000000.0;   // line/edge crossings per angle
constexpr std::size_t kPollEvery = 1024;

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

/// One piece of beam travel: from `start` (after `pre`, when overscanned) through `body` (cut moves, z and
/// power set at emission) to its end (then `post`).
struct Stroke {
    Vec2 start;
    std::vector<Move> body;
    bool overscan{false};
    Vec2 pre;
    Vec2 post;
    Vec2 end;
};

std::string check(const tooling::Tool& tool, const LaserCutFillParams& p, ToolpathIssue& issue) {
    issue = ToolpathIssue::InvalidParameters;
    const double values[] = {p.power,      p.moveSpeed,        p.cutDepth,         p.passDepth,
                             p.overscan,   p.allowance,        p.stepover,         p.hatchAngle,
                             p.safeZ,      p.startPosition.x,  p.startPosition.y,  p.tolerance.distance,
                             p.tolerance.chordal, p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!(p.power > 0.0) || p.power > 100.0) {
        return "power must be above 0 and at most 100 percent";
    }
    if (p.moveSpeed < 0.0 || p.moveSpeed > 1.0e6) {
        return "move speed must be 0 (the tool's feed rate) or between 0 and 1,000,000 mm/min";
    }
    if (p.cutDepth < 0.0 || p.cutDepth > kMaxLength) {
        return "cut depth must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (p.passDepth < 0.0 || p.passDepth > kMaxLength) {
        return "pass depth must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (p.repeats < 1 || p.repeats > kMaxRepeats) {
        return "repeats must be between 1 and " + std::to_string(kMaxRepeats);
    }
    if (p.overscan < 0.0 || p.overscan > kMaxOffset) {
        return "overscan must be between 0 and " + num(kMaxOffset) + " mm";
    }
    if (std::fabs(p.allowance) > kMaxOffset) {
        return "allowance must be within " + num(kMaxOffset) + " mm";
    }
    if (p.strategy == LaserStrategy::HatchFill && (!(p.stepover > 0.0) || p.stepover > kMaxLength)) {
        return "hatch stepover must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.safeZ > 0.0) || p.safeZ > kMaxLength) {
        return "safe Z must be above the material surface";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance >= 1e-6) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance between 1e-6 mm and chordal";
    }
    if (!laserToolAllowed(tool.type)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "' must be an end mill (its diameter is the beam kerf) for laser toolpaths";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "': " + *bad;
    }
    return {};
}

/// Appends the strokes of the cut strategies: the profile machinery's cuts, as paths of feed and arc moves.
bool cutStrokes(const std::vector<Contour>& contours, const tooling::Tool& tool, const tooling::CuttingData& data,
                const LaserCutFillParams& p, const ToolpathRunControl& control, std::vector<Stroke>& strokes,
                std::vector<ToolpathDiagnostic>& diagnostics) {
    ProfileParams pp;
    pp.name = p.name;
    pp.side = p.strategy == LaserStrategy::CutOutside ? ProfileSide::Outside
              : p.strategy == LaserStrategy::CutInside ? ProfileSide::Inside
                                                       : ProfileSide::On;
    pp.direction = p.direction;
    pp.allowance = p.allowance;
    pp.passes = PassPlan{0.0, 1.0, 1.0, true, 0.0};
    pp.safeZ = p.safeZ;
    pp.order = p.order;
    pp.startAt = p.startAt;
    pp.startAnchor = p.startAnchor;
    pp.startPosition = p.startPosition;
    pp.cuttingData = data;
    pp.tolerance = p.tolerance;
    ProfileResult pr = profileToolpath(contours, tool, pp, control);
    for (ToolpathDiagnostic& d : pr.diagnostics) {
        diagnostics.push_back(std::move(d));
    }
    if (pr.toolpath.isEmpty()) {
        return false;
    }
    // A profile cut is rapid, plunge, cuts, retract: keep the cuts (flat: no ramps, leads or tabs).
    Stroke cur;
    bool open = false;
    for (const Move& m : pr.toolpath.moves) {
        switch (m.kind) {
            case MoveKind::Plunge:
                cur = Stroke{};
                cur.start = {m.to.x, m.to.y};
                open = true;
                break;
            case MoveKind::Retract:
                if (open && !cur.body.empty()) {
                    cur.end = {cur.body.back().to.x, cur.body.back().to.y};
                    strokes.push_back(std::move(cur));
                }
                open = false;
                break;
            case MoveKind::Feed:
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                if (open) {
                    Move c = m;
                    c.feedOverride.reset();
                    cur.body.push_back(c);
                }
                break;
            default:
                break;
        }
    }
    return !strokes.empty();
}

/// A closed polygon ring.
struct Ring {
    std::vector<Vec2> pts;
};

double signedArea(const std::vector<Vec2>& p) {
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2& u = p[i];
        const Vec2& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return 0.5 * a;
}

/// Hatch lines (serpentine) over `rings` at `angle`, lines on the multiples of `step` across their direction.
/// Returns false when over the work limits.
bool hatch(const std::vector<const Ring*>& rings, double angle, double step, double minLength, bool overscan,
           double os, std::vector<Stroke>& out, const ToolpathRunControl& control, bool& cancelled) {
    const double ca = std::cos(angle);
    const double sa = std::sin(angle);
    double vmin = 0.0;
    double vmax = 0.0;
    bool any = false;
    for (const Ring* r : rings) {
        for (const Vec2& p : r->pts) {
            const double v = -sa * p.x + ca * p.y;
            vmin = any ? std::min(vmin, v) : v;
            vmax = any ? std::max(vmax, v) : v;
            any = true;
        }
    }
    if (!any) {
        return true;
    }
    if ((vmax - vmin) / step > kMaxLines) {
        return false;
    }
    struct Crossing {
        long long k;
        double u;
    };
    std::vector<Crossing> xs;
    std::size_t polled = 0;
    for (const Ring* r : rings) {
        const std::size_t n = r->pts.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = r->pts[i];
            const Vec2 b = r->pts[(i + 1) % n];
            const double va = -sa * a.x + ca * a.y;
            const double vb = -sa * b.x + ca * b.y;
            if (va == vb) {
                continue;
            }
            const double ua = ca * a.x + sa * a.y;
            const double ub = ca * b.x + sa * b.y;
            const double lo = std::min(va, vb);
            const double hi = std::max(va, vb);
            const auto k0 = static_cast<long long>(std::ceil(lo / step));
            const auto k1 = static_cast<long long>(std::ceil(hi / step)) - 1;  // k * step < hi
            if (k1 >= k0 && static_cast<double>(xs.size()) + static_cast<double>(k1 - k0 + 1) > kMaxCrossings) {
                return false;
            }
            for (long long k = k0; k <= k1; ++k) {
                if (control.cancel != nullptr && ++polled % kPollEvery == 0 && control.cancel->load()) {
                    cancelled = true;
                    return true;
                }
                const double t = (static_cast<double>(k) * step - va) / (vb - va);
                xs.push_back({k, ua + (ub - ua) * t});
            }
        }
    }
    std::sort(xs.begin(), xs.end(), [](const Crossing& l, const Crossing& r) {
        return l.k != r.k ? l.k < r.k : l.u < r.u;
    });
    bool forward = true;
    std::size_t i = 0;
    while (i < xs.size()) {
        std::size_t j = i;
        while (j < xs.size() && xs[j].k == xs[i].k) {
            ++j;
        }
        const double v = static_cast<double>(xs[i].k) * step;
        std::vector<std::pair<double, double>> segs;
        for (std::size_t q = i; q + 1 < j; q += 2) {
            if (xs[q + 1].u - xs[q].u > minLength) {
                segs.emplace_back(xs[q].u, xs[q + 1].u);
            }
        }
        if (!forward) {
            std::reverse(segs.begin(), segs.end());
        }
        for (const auto& [u0, u1] : segs) {
            const double ua = forward ? u0 : u1;
            const double ub = forward ? u1 : u0;
            const double dir = forward ? 1.0 : -1.0;
            const auto at = [&](double u) { return Vec2{ca * u - sa * v, sa * u + ca * v}; };
            Stroke s;
            s.start = at(ua);
            s.end = at(ub);
            s.body.push_back(Move::feed({s.end.x, s.end.y, 0.0}));
            if (overscan) {
                s.overscan = true;
                s.pre = at(ua - dir * os);
                s.post = at(ub + dir * os);
            }
            out.push_back(std::move(s));
        }
        if (!segs.empty()) {
            forward = !forward;
        }
        i = j;
    }
    return true;
}

}  // namespace

bool laserToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::EndMill;
}

std::vector<double> laserPassLevels(double cutDepth, double passDepth) {
    if (!(cutDepth > 0.0)) {
        return {0.0};
    }
    PassPlan plan{0.0, cutDepth, passDepth > 0.0 ? passDepth : cutDepth, true, 0.0};
    return plan.levels();
}

LaserCutFillResult laserCutFillToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                                        const std::optional<tooling::CuttingData>& cuttingData,
                                        const LaserCutFillParams& params, const ToolpathRunControl& control) {
    LaserCutFillResult result;
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
    // Speed and pass depth come from the parameters, else the cutting data.
    tooling::CuttingData data = effectiveCuttingData(tool, cuttingData);
    const double speed = params.moveSpeed > 0.0 ? params.moveSpeed : data.feedRate;
    if (!std::isfinite(speed) || !(speed > 0.0)) {
        say(ToolpathIssue::InvalidTool, std::nullopt,
            "tool '" + tool.name + "': the move speed (or the cutting data's feed rate) must be greater than zero");
        return result;
    }
    const double passStep = params.passDepth > 0.0 ? params.passDepth : data.passDepth;
    data.feedRate = speed;
    data.plungeRate = speed;
    data.passDepth = 1.0;
    data.stepover = std::max(data.stepover, 0.0);
    if (!(data.spindleSpeed > 0.0)) {
        data.spindleSpeed = 1.0;  // the profile machinery wants one; a laser toolpath records none
    }
    result.toolpath.tool = toolSnapshot(tool, data);
    result.toolpath.tool.spindleSpeed = 0.0;
    result.toolpath.tool.laser = true;

    const std::vector<double> levels = laserPassLevels(params.cutDepth, passStep);
    if (levels.empty()) {
        say(ToolpathIssue::NoPasses, std::nullopt, "the cut depth and pass depth give no passes");
        return result;
    }
    if (params.cutDepth > 0.0 && params.cutDepth / (passStep > 0.0 ? passStep : params.cutDepth) > kMaxPasses) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "more than " + num(kMaxPasses) + " passes (cut depth " + num(params.cutDepth) + " mm, pass depth " +
                num(passStep) + " mm)");
        return result;
    }
    result.passes = levels.size();

    std::vector<Stroke> strokes;
    const double r = 0.5 * tool.diameter;
    if (params.strategy != LaserStrategy::HatchFill) {
        if (!cutStrokes(contours, tool, data, params, control, strokes, result.diagnostics)) {
            if (result.diagnostics.empty()) {
                say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to cut");
            }
            return result;
        }
    } else {
        std::vector<Contour> closed;
        for (std::size_t i = 0; i < contours.size(); ++i) {
            const Contour& c = contours[i];
            if (c.isEmpty()) {
                say(ToolpathIssue::EmptyContour, i, "vector " + std::to_string(i) + " has no spans; skipped");
            } else if (!c.closed) {
                say(ToolpathIssue::OpenContourIgnored, i,
                    "vector " + std::to_string(i) + " is open; hatch fill needs closed vectors");
            } else if (!c.isContinuous(tol)) {
                say(ToolpathIssue::DiscontinuousContour, i,
                    "vector " + std::to_string(i) + " has gaps between spans; skipped");
            } else if (geometry::refusedByClippers(c, tol)) {
                result.diagnostics.push_back(refusedContourDiagnostic(i, "vector " + std::to_string(i)));
            } else {
                closed.push_back(c);
            }
        }
        if (closed.empty()) {
            if (result.diagnostics.empty()) {
                say(ToolpathIssue::NoRegion, std::nullopt, "no closed vectors to fill");
            }
            return result;
        }
        // The region (even-odd: a vector inside one other is a hole), inset by the beam radius and allowance.
        std::vector<Contour> region = geometry::unionContours(closed, tol, geometry::FillRule::EvenOdd);
        const double inset = r + params.allowance;
        if (inset != 0.0) {
            region = geometry::offsetContours(region, -inset, geometry::JoinType::Round, tol);
        }
        if (region.empty()) {
            say(ToolpathIssue::ToolTooLarge, std::nullopt, "the region is too small for the beam and allowance");
            return result;
        }
        // Rings, grouped: each CCW outer with the CW holes inside it.
        std::vector<Ring> rings;
        std::vector<Contour> ringContours;
        for (const Contour& c : region) {
            Ring ring;
            for (const Vec2 q : c.polygonise(tol).points) {
                ring.pts.push_back(q);
            }
            if (ring.pts.size() >= 3) {
                rings.push_back(std::move(ring));
                ringContours.push_back(c);
            }
        }
        std::vector<std::size_t> outers;
        std::vector<std::size_t> holes;
        for (std::size_t i = 0; i < rings.size(); ++i) {
            (signedArea(rings[i].pts) > 0.0 ? outers : holes).push_back(i);
        }
        std::vector<std::vector<const Ring*>> groups;
        if (params.fillMode == LaserFillMode::Together) {
            std::vector<const Ring*> all;
            for (const Ring& g : rings) {
                all.push_back(&g);
            }
            groups.push_back(std::move(all));
        } else {
            std::vector<std::vector<const Ring*>> byOuter(outers.size());
            std::vector<double> outerArea(outers.size());
            for (std::size_t o = 0; o < outers.size(); ++o) {
                byOuter[o].push_back(&rings[outers[o]]);
                outerArea[o] = signedArea(rings[outers[o]].pts);
            }
            for (const std::size_t h : holes) {
                std::size_t best = outers.size();
                for (std::size_t o = 0; o < outers.size(); ++o) {
                    if ((best == outers.size() || outerArea[o] < outerArea[best]) &&
                        geometry::isInside(ringContours[outers[o]], ringContours[h].startPoint(),
                                           geometry::FillRule::EvenOdd, tol)) {
                        best = o;
                    }
                }
                if (best < outers.size()) {
                    byOuter[best].push_back(&rings[h]);
                }
            }
            std::vector<OrderItem> items;
            for (std::size_t o = 0; o < outers.size(); ++o) {
                OrderItem item;
                item.path = ringContours[outers[o]];
                item.selectionIndex = o;
                item.start = item.end = ringContours[outers[o]].startPoint();
                items.push_back(std::move(item));
            }
            for (const std::size_t o : orderCuts(items, params.order, params.startPosition, tol)) {
                groups.push_back(std::move(byOuter[o]));
            }
        }
        const double angle1 = params.hatchAngle;
        const double angle2 = params.hatchAngle + 1.5707963267948966;
        const bool overscan = params.overscan > 0.0;
        bool cancelled = false;
        const auto fill = [&](const std::vector<const Ring*>& g, double angle) {
            return hatch(g, angle, params.stepover, tol.distance, overscan, params.overscan, strokes, control,
                         cancelled);
        };
        bool ok = true;
        if (params.fillMode == LaserFillMode::Together) {
            ok = fill(groups[0], angle1) && (!params.crossHatch || fill(groups[0], angle2));
        } else {
            for (const auto& g : groups) {
                ok = ok && fill(g, angle1) && (!params.crossHatch || fill(g, angle2));
            }
        }
        if (cancelled) {
            say(ToolpathIssue::Cancelled, std::nullopt, "the calculation was cancelled");
            return result;
        }
        if (!ok) {
            say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
                "the hatch fill would need more than " + num(kMaxLines) + " lines or " + num(kMaxCrossings) +
                    " crossings (stepover " + num(params.stepover) + " mm)");
            return result;
        }
        if (params.outline) {
            std::vector<OrderItem> items;
            for (std::size_t i = 0; i < ringContours.size(); ++i) {
                OrderItem item;
                item.path = ringContours[i];
                item.selectionIndex = i;
                item.start = item.end = ringContours[i].startPoint();
                items.push_back(std::move(item));
            }
            for (const std::size_t i : orderCuts(items, params.order, params.startPosition, tol)) {
                Contour c = ringContours[i];
                const bool ccw = signedArea(rings[i].pts) > 0.0;
                // Climb: the region (waste outside it) stays on the right of travel, i.e. outer rings run CW.
                if ((params.direction == CutDirection::Climb) == ccw) {
                    c.reverse();
                }
                Stroke s;
                s.start = c.startPoint();
                for (const geometry::Span& span : c.spans) {
                    if (const auto* ln = std::get_if<geometry::LineSpan>(&span)) {
                        s.body.push_back(Move::feed({ln->end.x, ln->end.y, 0.0}));
                    }
                }
                if (!s.body.empty()) {
                    s.end = {s.body.back().to.x, s.body.back().to.y};
                    strokes.push_back(std::move(s));
                }
            }
        }
        if (strokes.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "the region holds no hatch line (check the stepover)");
            return result;
        }
    }
    result.strokes = strokes.size();

    std::size_t bodyMoves = 0;
    for (const Stroke& s : strokes) {
        bodyMoves += s.body.size() + (s.overscan ? 4 : 1);
    }
    const double moveEstimate = static_cast<double>(bodyMoves) * static_cast<double>(levels.size()) *
                                static_cast<double>(params.repeats) + 1.0;
    const double moveLimit = control.moveLimit > 0 ? static_cast<double>(control.moveLimit) : kMaxMoves;
    if (!(moveEstimate <= moveLimit)) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "the toolpath would have about " + num(moveEstimate) + " moves; the limit is " + num(moveLimit));
        result.toolpath.moves.clear();
        return result;
    }

    std::vector<Move>& moves = result.toolpath.moves;
    moves.reserve(static_cast<std::size_t>(moveEstimate));
    Vec2 last{};
    for (const double z : levels) {
        for (int rep = 0; rep < params.repeats; ++rep) {
            for (const Stroke& s : strokes) {
                if (s.overscan) {
                    moves.push_back(Move::rapid({s.pre.x, s.pre.y, z}));
                    moves.push_back(Move::feed({s.start.x, s.start.y, z}));
                } else {
                    moves.push_back(Move::rapid({s.start.x, s.start.y, z}));
                }
                for (const Move& m : s.body) {
                    Move c = m;
                    c.to.z = z;
                    c.jetPower = params.power;
                    moves.push_back(c);
                }
                last = s.end;
                if (s.overscan) {
                    moves.push_back(Move::feed({s.post.x, s.post.y, z}));
                    last = s.post;
                }
            }
        }
    }
    moves.push_back(Move::rapid({last.x, last.y, params.safeZ}));
    return result;
}

}  // namespace carvetto::toolpath
