#pragma once

// Helpers shared by the profile toolpath tests (T-0120). The distance functions here are written
// independently of the geometry library (no offset, closest-point or intersection code), so the
// gouge checks do not trust the code under test.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::test {

inline tooling::Tool endMill(double diameter = 6.0) {
    tooling::Tool t;
    t.id = "em";
    t.name = "End Mill (" + std::to_string(static_cast<int>(diameter)) + " mm)";
    t.type = tooling::ToolType::EndMill;
    t.diameter = diameter;
    t.defaultCuttingData.passDepth = 4.0;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 18000.0;
    t.defaultCuttingData.toolNumber = 1;
    return t;
}

inline const char* kindName(toolpath::MoveKind k) {
    switch (k) {
        case toolpath::MoveKind::Rapid:
            return "rapid";
        case toolpath::MoveKind::Feed:
            return "feed";
        case toolpath::MoveKind::Plunge:
            return "plunge";
        case toolpath::MoveKind::Retract:
            return "retract";
        case toolpath::MoveKind::ArcCW:
            return "arc_cw";
        case toolpath::MoveKind::ArcCCW:
            return "arc_ccw";
        case toolpath::MoveKind::Dwell:
            return "dwell";
    }
    return "?";
}

/// Fixed-point text with "-0.000000" folded to "0.000000".
inline std::string num(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.6f", v);
    std::string s = buf;
    if (s == "-0.000000") {
        s = "0.000000";
    }
    return s;
}

/// One line per move: kind, end X Y Z and (arcs) centre X Y. Six decimals, LF endings.
inline std::string formatMoves(const toolpath::Toolpath& tp) {
    std::string out;
    for (const toolpath::Move& m : tp.moves) {
        out += kindName(m.kind);
        out += ' ' + num(m.to.x) + ' ' + num(m.to.y) + ' ' + num(m.to.z);
        if (toolpath::isArc(m.kind)) {
            out += " c " + num(m.centre.x) + ' ' + num(m.centre.y);
        }
        out += '\n';
    }
    return out;
}

// ------------------------------------------------------------- independent distance oracle

inline double bruteDistanceToLine(geometry::Vec2 p, geometry::Vec2 a, geometry::Vec2 b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

/// Distance from p to an arc: radial distance when p's direction is inside the sweep (tested by
/// walking the sweep in 1e-4 rad steps around the projected angle), else the nearer end.
inline double bruteDistanceToArc(geometry::Vec2 p, const geometry::ArcSpan& a) {
    const double pi2 = 6.283185307179586;
    const double sweep = a.sweepAngle();  // signed
    const double ang = std::atan2(p.y - a.centre.y, p.x - a.centre.x);
    // Parameter of p's direction along the sweep.
    double rel = (ang - a.startAngle) * (sweep >= 0.0 ? 1.0 : -1.0);
    rel = std::fmod(rel, pi2);
    if (rel < 0.0) {
        rel += pi2;
    }
    const double rp = std::hypot(p.x - a.centre.x, p.y - a.centre.y);
    double best = std::numeric_limits<double>::infinity();
    if (rel <= std::fabs(sweep)) {
        best = std::fabs(rp - a.radius);
    }
    const double sx = a.centre.x + a.radius * std::cos(a.startAngle);
    const double sy = a.centre.y + a.radius * std::sin(a.startAngle);
    const double ex = a.centre.x + a.radius * std::cos(a.startAngle + sweep);
    const double ey = a.centre.y + a.radius * std::sin(a.startAngle + sweep);
    best = std::min(best, std::hypot(p.x - sx, p.y - sy));
    best = std::min(best, std::hypot(p.x - ex, p.y - ey));
    return best;
}

/// Distance from p to the line and arc spans of the contours (Beziers: sampled at 1/2000).
inline double bruteDistance(geometry::Vec2 p, const std::vector<geometry::Contour>& cs) {
    double best = std::numeric_limits<double>::infinity();
    for (const geometry::Contour& c : cs) {
        for (const geometry::Span& s : c.spans) {
            if (const auto* l = std::get_if<geometry::LineSpan>(&s)) {
                best = std::min(best, bruteDistanceToLine(p, l->start, l->end));
            } else if (const auto* a = std::get_if<geometry::ArcSpan>(&s)) {
                best = std::min(best, bruteDistanceToArc(p, *a));
            } else if (const auto* b = std::get_if<geometry::BezierSpan>(&s)) {
                geometry::Vec2 prev = b->pointAt(0.0);
                for (int i = 1; i <= 2000; ++i) {
                    const geometry::Vec2 q = b->pointAt(i / 2000.0);
                    best = std::min(best, bruteDistanceToLine(p, prev, q));
                    prev = q;
                }
            }
        }
    }
    return best;
}

/// Samples every cutting move (feeds, plunges, retracts, arcs) of `tp` every `step` mm in XY
/// (arcs by angle, from start/centre/direction alone) and calls f(point) on each sample.
template <class F>
void sampleCuttingMoves(const toolpath::Toolpath& tp, double step, F&& f) {
    for (std::size_t i = 1; i < tp.moves.size(); ++i) {
        const toolpath::Move& m = tp.moves[i];
        if (!toolpath::isCutting(m.kind)) {
            continue;
        }
        const geometry::Vec2 a = tp.moves[i - 1].to.xy();
        const geometry::Vec2 b = m.to.xy();
        if (toolpath::isArc(m.kind)) {
            const double r = std::hypot(a.x - m.centre.x, a.y - m.centre.y);
            const double a0 = std::atan2(a.y - m.centre.y, a.x - m.centre.x);
            const double a1 = std::atan2(b.y - m.centre.y, b.x - m.centre.x);
            const double pi2 = 6.283185307179586;
            double sw = a1 - a0;
            if (m.kind == toolpath::MoveKind::ArcCCW) {
                while (sw <= 0.0) sw += pi2;
            } else {
                while (sw >= 0.0) sw -= pi2;
            }
            if (std::hypot(a.x - b.x, a.y - b.y) == 0.0) {
                sw = m.kind == toolpath::MoveKind::ArcCCW ? pi2 : -pi2;
            }
            const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(sw) * r / step)));
            for (int k = 0; k <= n; ++k) {
                const double t = a0 + sw * k / n;
                f(geometry::Vec2{m.centre.x + r * std::cos(t), m.centre.y + r * std::sin(t)});
            }
        } else {
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
            for (int k = 0; k <= n; ++k) {
                f(geometry::Vec2{a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n});
            }
        }
    }
}

/// Like `sampleCuttingMoves`, with the tool-tip height (Z interpolated linearly along the move,
/// which is exact for lines and for helical arcs): f(point, z).
template <class F>
void sampleCuttingMoves3(const toolpath::Toolpath& tp, double step, F&& f,
                         double zBelow = std::numeric_limits<double>::infinity()) {
    for (std::size_t i = 1; i < tp.moves.size(); ++i) {
        if (!toolpath::isCutting(tp.moves[i].kind) ||
            std::min(tp.moves[i - 1].to.z, tp.moves[i].to.z) > zBelow) {
            continue;  // (moves entirely above zBelow are skipped when it is given)
        }
        toolpath::Toolpath one;
        one.moves = {tp.moves[i - 1], tp.moves[i]};
        std::vector<geometry::Vec2> pts;
        sampleCuttingMoves(one, step, [&](geometry::Vec2 p) { pts.push_back(p); });
        const double z0 = tp.moves[i - 1].to.z;
        const double z1 = tp.moves[i].to.z;
        for (std::size_t k = 0; k < pts.size(); ++k) {
            const double t = pts.size() > 1 ? static_cast<double>(k) / static_cast<double>(pts.size() - 1) : 1.0;
            f(pts[k], z0 + (z1 - z0) * t);
        }
    }
}

/// Smallest distance from any cutting-move sample of `tp` to `cs`.
inline double minCutDistance(const toolpath::Toolpath& tp, const std::vector<geometry::Contour>& cs,
                             double step = 0.05) {
    double best = std::numeric_limits<double>::infinity();
    sampleCuttingMoves(tp, step, [&](geometry::Vec2 p) { best = std::min(best, bruteDistance(p, cs)); });
    return best;
}

/// Smallest cone-vs-wall margin of a V-bit toolpath: at every cutting sample with the tip below
/// the surface (Z = 0), distance to the vectors - allowance - the radius the cone cuts at the
/// surface (tip depth * tanHalf, capped at `radius`). Negative = the cone passes a wall.
inline double minConeClearance(const toolpath::Toolpath& tp, const std::vector<geometry::Contour>& cs,
                               double tanHalf, double radius, double allowance, double step = 0.01) {
    double worst = std::numeric_limits<double>::infinity();
    sampleCuttingMoves3(
        tp, step,
        [&](geometry::Vec2 q, double z) {
            if (z >= 0.0) {
                return;
            }
            const double cone = std::min(-z * tanHalf, radius);
            worst = std::min(worst, bruteDistance(q, cs) - allowance - cone);
        },
        0.0);
    return worst;
}

/// Points of the toolpath's cutting moves at Z <= `zMax` (sampled every `step`), bucketed for
/// "is this outline point cut?" queries.
class CutCoverage {
   public:
    CutCoverage(const toolpath::Toolpath& tp, double zMax, double step = 0.02) : cell_(4.0 * step) {
        sampleCuttingMoves3(
            tp, step,
            [&](geometry::Vec2 q, double z) {
                if (z <= zMax + 1e-9) {
                    cells_[key(q)].push_back(q);
                }
            },
            zMax + 1e-9);
    }
    /// True when some sample lies within `reach` of p (reach <= cell size).
    bool covers(geometry::Vec2 p, double reach) const {
        const auto [cx, cy] = key(p);
        for (long dx = -1; dx <= 1; ++dx) {
            for (long dy = -1; dy <= 1; ++dy) {
                const auto it = cells_.find({cx + dx, cy + dy});
                if (it == cells_.end()) {
                    continue;
                }
                for (const geometry::Vec2& q : it->second) {
                    if (std::hypot(q.x - p.x, q.y - p.y) <= reach) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

   private:
    std::pair<long, long> key(geometry::Vec2 p) const {
        return {static_cast<long>(std::floor(p.x / cell_)), static_cast<long>(std::floor(p.y / cell_))};
    }
    double cell_;
    std::map<std::pair<long, long>, std::vector<geometry::Vec2>> cells_;
};

/// Analytic tool-centre outline of a circle (centre c, radius r) offset by `d` (sign: + outwards),
/// sampled every `step` mm.
inline std::vector<geometry::Vec2> circleOutline(geometry::Vec2 c, double r, double d, double step) {
    std::vector<geometry::Vec2> pts;
    const double R = r + d;
    const int n = std::max(8, static_cast<int>(std::ceil(6.283185307179586 * R / step)));
    for (int i = 0; i < n; ++i) {
        const double a = 6.283185307179586 * i / n;
        pts.push_back({c.x + R * std::cos(a), c.y + R * std::sin(a)});
    }
    return pts;
}

/// Analytic tool-centre outline of an axis-aligned rectangle offset by `d`: outwards (d > 0) a
/// rounded rectangle with corner radius d, inwards a smaller sharp rectangle; sampled every `step`.
inline std::vector<geometry::Vec2> rectOutline(geometry::Box2 b, double d, double step) {
    std::vector<geometry::Vec2> pts;
    auto line = [&](geometry::Vec2 a, geometry::Vec2 e) {
        const double len = std::hypot(e.x - a.x, e.y - a.y);
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int i = 0; i < n; ++i) {
            pts.push_back({a.x + (e.x - a.x) * i / n, a.y + (e.y - a.y) * i / n});
        }
    };
    if (d < 0.0) {
        const double e = -d;
        const geometry::Vec2 p0{b.min.x + e, b.min.y + e}, p1{b.max.x - e, b.min.y + e}, p2{b.max.x - e, b.max.y - e},
            p3{b.min.x + e, b.max.y - e};
        line(p0, p1);
        line(p1, p2);
        line(p2, p3);
        line(p3, p0);
        return pts;
    }
    // In traversal order (CCW): bottom edge, corner, right edge, corner, top, corner, left, corner.
    auto corner = [&](geometry::Vec2 c, double a0) {
        const int n = std::max(2, static_cast<int>(std::ceil(1.5707963267948966 * d / step)));
        for (int i = 0; i < n; ++i) {
            const double a = a0 + 1.5707963267948966 * i / n;
            pts.push_back({c.x + d * std::cos(a), c.y + d * std::sin(a)});
        }
    };
    line({b.min.x, b.min.y - d}, {b.max.x, b.min.y - d});
    corner({b.max.x, b.min.y}, -1.5707963267948966);
    line({b.max.x + d, b.min.y}, {b.max.x + d, b.max.y});
    corner({b.max.x, b.max.y}, 0.0);
    line({b.max.x, b.max.y + d}, {b.min.x, b.max.y + d});
    corner({b.min.x, b.max.y}, 1.5707963267948966);
    line({b.min.x - d, b.max.y}, {b.min.x - d, b.min.y});
    corner({b.min.x, b.min.y}, 3.141592653589793);
    return pts;
}

/// Lengths (in outline samples x step) of the maximal runs of `outline` (a closed loop of samples)
/// that are not cut at Z <= zMax. Empty = every outline point is cut at that depth.
inline std::vector<double> uncutRuns(const std::vector<geometry::Vec2>& outline, const CutCoverage& cov, double reach,
                                     double step) {
    std::vector<bool> cut(outline.size());
    for (std::size_t i = 0; i < outline.size(); ++i) {
        cut[i] = cov.covers(outline[i], reach);
    }
    std::vector<double> runs;
    const std::size_t n = outline.size();
    std::size_t first = n;
    for (std::size_t i = 0; i < n; ++i) {
        if (cut[i]) {
            first = i;
            break;
        }
    }
    if (first == n) {
        return {static_cast<double>(n) * step};
    }
    std::size_t len = 0;
    for (std::size_t k = 1; k <= n; ++k) {
        const std::size_t i = (first + k) % n;
        if (!cut[i]) {
            ++len;
        } else if (len > 0) {
            runs.push_back(static_cast<double>(len) * step);
            len = 0;
        }
    }
    return runs;
}

/// The Z levels at which the tool cuts flat (feed end points), deduplicated, descending.
inline std::vector<double> feedLevels(const toolpath::Toolpath& tp) {
    std::vector<double> zs;
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind == toolpath::MoveKind::Feed || toolpath::isArc(m.kind)) {
            zs.push_back(m.to.z);
        }
    }
    std::sort(zs.begin(), zs.end(), [](double a, double b) { return a > b; });
    zs.erase(std::unique(zs.begin(), zs.end(), [](double a, double b) { return std::fabs(a - b) < 1e-9; }), zs.end());
    return zs;
}

}  // namespace carvetto::test
