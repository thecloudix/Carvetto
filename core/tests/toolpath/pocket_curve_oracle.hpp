#pragma once

// Exact wall clearance of straight tool moves against the ORIGINAL curves (T-0169). Nothing here
// uses polygonisation, offsets or booleans: lines and arcs are measured in closed form, Beziers by
// certified subdivision (each piece lies in the disc round its control-point centroid that holds
// its control points, so the distance to that disc is a lower bound).
//
// `wallClearance` checks every non-rapid move below the surface: the tool centre, along the whole
// move, must keep at least `need` from every span of the input (`certified`). It also reports the
// smallest centre-to-wall distance less the radius it saw (exact for lines and arcs; for Beziers
// the smallest distance found, which is an upper bound within the subdivision's resolution).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::test::pocket_curve {

using geometry::Vec2;
using geometry::Vec3;

inline double pointSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
    return geometry::distance(p, a + ab * t);
}

/// Exact distance between segments ab and cd (0 when they cross or touch).
inline double segmentSegment(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const Vec2 ab = b - a;
    const Vec2 cd = d - c;
    const double o1 = ab.cross(c - a);
    const double o2 = ab.cross(d - a);
    const double o3 = cd.cross(a - c);
    const double o4 = cd.cross(b - c);
    if (((o1 < 0.0 && o2 > 0.0) || (o1 > 0.0 && o2 < 0.0)) && ((o3 < 0.0 && o4 > 0.0) || (o3 > 0.0 && o4 < 0.0))) {
        return 0.0;
    }
    return std::min(std::min(pointSegment(a, c, d), pointSegment(b, c, d)),
                    std::min(pointSegment(c, a, b), pointSegment(d, a, b)));
}

/// True when the direction at angle `ang` from the centre lies on the arc.
inline bool onArc(const geometry::ArcSpan& arc, double ang) {
    const double sweep = arc.sweepAngle();
    double delta = sweep >= 0.0 ? ang - arc.startAngle : arc.startAngle - ang;
    delta = std::fmod(delta, geometry::kTwoPi);
    if (delta < 0.0) {
        delta += geometry::kTwoPi;
    }
    return delta <= std::fabs(sweep) + 1e-15;
}

/// Exact distance from p to the arc: the radial point when its direction is on the arc, else the
/// nearer end.
inline double pointArc(Vec2 p, const geometry::ArcSpan& arc) {
    const Vec2 v = p - arc.centre;
    double best = std::min(geometry::distance(p, arc.startPoint()), geometry::distance(p, arc.pointAt(1.0)));
    if (v.lengthSquared() > 0.0 && onArc(arc, std::atan2(v.y, v.x))) {
        best = std::min(best, std::fabs(v.length() - arc.radius));
    }
    return best;
}

/// Exact distance between segment ab and an arc. The closest pair is a crossing, involves an end
/// of either, or has both points inside: then it is perpendicular to the segment and radial for
/// the circle, so the arc point is the centre ± radius along the segment's normal.
inline double segmentArc(Vec2 a, Vec2 b, const geometry::ArcSpan& arc) {
    double best = std::min(pointArc(a, arc), pointArc(b, arc));
    best = std::min(best, std::min(pointSegment(arc.startPoint(), a, b), pointSegment(arc.pointAt(1.0), a, b)));
    const Vec2 ab = b - a;
    const double len = ab.length();
    if (len <= 0.0) {
        return best;
    }
    const Vec2 u = ab * (1.0 / len);
    const Vec2 n = u.perp();
    for (const double s : {1.0, -1.0}) {
        const Vec2 q = arc.centre + n * (s * arc.radius);
        if (onArc(arc, std::atan2(q.y - arc.centre.y, q.x - arc.centre.x))) {
            best = std::min(best, pointSegment(q, a, b));
        }
    }
    // Crossings of the segment's line with the circle inside the segment and on the arc.
    const Vec2 w = a - arc.centre;
    const double bq = w.dot(u);
    const double disc = bq * bq - (w.lengthSquared() - arc.radius * arc.radius);
    if (disc >= 0.0) {
        for (const double s : {1.0, -1.0}) {
            const double t = -bq + s * std::sqrt(disc);
            if (t >= 0.0 && t <= len) {
                const Vec2 q = a + u * t;
                if (onArc(arc, std::atan2(q.y - arc.centre.y, q.x - arc.centre.x))) {
                    return 0.0;
                }
            }
        }
    }
    return best;
}

/// Lower bound of the distance from segment ab to a Bezier piece: the distance to the convex hull
/// of its control points (the union of the four triangles they form), which holds the piece; 0
/// when the segment meets a triangle. Tight for long flat pieces, where the disc bound is not
/// (T-0175: a raster line along a 400 mm flat Bezier needed millions of discs).
inline double segmentHull(Vec2 a, Vec2 b, Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3) {
    auto inTriangle = [](Vec2 x, Vec2 p, Vec2 q, Vec2 r) {
        const double d1 = (q - p).cross(x - p);
        const double d2 = (r - q).cross(x - q);
        const double d3 = (p - r).cross(x - r);
        return (d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0) || (d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0);
    };
    const Vec2 pts[4] = {p0, p1, p2, p3};
    double best = std::numeric_limits<double>::infinity();
    for (int skip = 0; skip < 4; ++skip) {
        Vec2 t[3];
        for (int i = 0, k = 0; i < 4; ++i) {
            if (i != skip) {
                t[k++] = pts[i];
            }
        }
        if (inTriangle(a, t[0], t[1], t[2]) || inTriangle(b, t[0], t[1], t[2])) {
            return 0.0;
        }
        for (int e = 0; e < 3; ++e) {
            best = std::min(best, segmentSegment(a, b, t[e], t[(e + 1) % 3]));
        }
    }
    return best;
}

/// Result of measuring one segment against one Bezier for a threshold.
struct BezierVerdict {
    bool atLeast{true};  ///< certified: every point of the curve is at least `need` from the segment
    double seen{std::numeric_limits<double>::infinity()};  ///< smallest distance found (an upper bound)
};

/// Certified "distance(segment ab, curve) >= need" by subdivision, plus the smallest distance seen.
/// Pieces whose disc or control-hull bound is at least `need` are dropped; a piece end closer than `need` is a
/// counter-example. `resolution` stops refining a piece once its disc is that small (its bound is
/// then accepted within `resolution`).
inline BezierVerdict segmentBezier(Vec2 a, Vec2 b, const geometry::BezierSpan& bz, double need,
                                   double resolution) {
    struct Piece {
        Vec2 p0, p1, p2, p3;
    };
    BezierVerdict v;
    std::vector<Piece> stack{{bz.p0, bz.p1, bz.p2, bz.p3}};
    v.seen = std::min(pointSegment(bz.p0, a, b), pointSegment(bz.p3, a, b));
    if (v.seen < need) {
        v.atLeast = false;
    }
    std::size_t guard = 0;
    while (!stack.empty() && v.atLeast) {
        if (++guard > 2000000) {
            v.atLeast = false;  // not resolved: report as a failure rather than pass silently
            break;
        }
        const Piece pc = stack.back();
        stack.pop_back();
        const Vec2 m = (pc.p0 + pc.p1 + pc.p2 + pc.p3) * 0.25;
        const double rho = std::max({geometry::distance(m, pc.p0), geometry::distance(m, pc.p1),
                                     geometry::distance(m, pc.p2), geometry::distance(m, pc.p3)});
        const double lower = std::max(pointSegment(m, a, b) - rho, segmentHull(a, b, pc.p0, pc.p1, pc.p2, pc.p3));
        if (lower >= need) {
            continue;
        }
        if (rho <= resolution) {
            continue;  // resolved within `resolution` (its ends were already measured)
        }
        // de Casteljau at 1/2.
        const Vec2 q0 = (pc.p0 + pc.p1) * 0.5, q1 = (pc.p1 + pc.p2) * 0.5, q2 = (pc.p2 + pc.p3) * 0.5;
        const Vec2 r0 = (q0 + q1) * 0.5, r1 = (q1 + q2) * 0.5;
        const Vec2 mid = (r0 + r1) * 0.5;
        const double dm = pointSegment(mid, a, b);
        v.seen = std::min(v.seen, dm);
        if (dm < need) {
            v.atLeast = false;
            break;
        }
        stack.push_back({pc.p0, q0, r0, mid});
        stack.push_back({mid, r1, q2, pc.p3});
    }
    return v;
}

/// Even-odd inside test over the original curves (points on a boundary count as inside).
inline bool insideRegion(const std::vector<geometry::Contour>& contours, Vec2 p) {
    const geometry::Tolerance tol{1e-7, 1e-9, 1e-5};
    bool in = false;
    for (const geometry::Contour& c : contours) {
        if (!c.closed || c.isEmpty()) {
            continue;
        }
        const auto loc = geometry::locatePoint(c, p, geometry::FillRule::EvenOdd, tol);
        if (loc == geometry::PointLocation::OnBoundary) {
            return true;
        }
        if (loc == geometry::PointLocation::Inside) {
            in = !in;
        }
    }
    return in;
}

struct ClearanceReport {
    /// Every checked move keeps at least `need` from every span (certified).
    bool certified{true};
    /// Smallest centre-to-wall distance less `radius` seen over the checked moves.
    double minClearance{std::numeric_limits<double>::infinity()};
    std::size_t worstMove{0};
    std::size_t checkedMoves{0};
    /// Every plunge below the surface lands inside the region.
    bool plungesInside{true};
};

/// Checks every non-rapid move of `tp` that reaches below `surface` against every span of
/// `walls`: the centre must stay at least radius + `minExtra` away (see the file comment).
inline ClearanceReport wallClearance(const toolpath::Toolpath& tp, const std::vector<geometry::Contour>& walls,
                                     double radius, double minExtra, double surface) {
    ClearanceReport rep;
    const double need = radius + minExtra;
    struct Item {
        const geometry::Span* span;
        geometry::Box2 box;
    };
    std::vector<Item> items;
    for (const geometry::Contour& c : walls) {
        for (const geometry::Span& s : c.spans) {
            geometry::Box2 box;
            if (const auto* bz = std::get_if<geometry::BezierSpan>(&s)) {
                for (const Vec2 q : {bz->p0, bz->p1, bz->p2, bz->p3}) {
                    box.expand(q);
                }
            } else if (const auto* arc = std::get_if<geometry::ArcSpan>(&s)) {
                box.expand(arc->centre - Vec2{arc->radius, arc->radius});
                box.expand(arc->centre + Vec2{arc->radius, arc->radius});
            } else {
                const auto& l = std::get<geometry::LineSpan>(s);
                box.expand(l.start);
                box.expand(l.end);
            }
            items.push_back({&s, box});
        }
    }
    // A generous window: spans farther than this from a move cannot set the minimum we report.
    const double window = need + 1.0;
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const toolpath::Move& m = tp.moves[i];
        if (m.kind == toolpath::MoveKind::Rapid) {
            continue;
        }
        const Vec3 s3 = tp.startOf(i);
        if (std::min(s3.z, m.to.z) >= surface - 1e-9) {
            continue;
        }
        ++rep.checkedMoves;
        const Vec2 a = s3.xy();
        const Vec2 b = m.to.xy();
        if (m.kind == toolpath::MoveKind::Plunge && !insideRegion(walls, b)) {
            rep.plungesInside = false;
        }
        geometry::Box2 mb;
        mb.expand(a);
        mb.expand(b);
        mb = mb.inflated(window);
        for (const Item& it : items) {
            if (!mb.intersects(it.box)) {
                continue;
            }
            double d = 0.0;
            if (const auto* bz = std::get_if<geometry::BezierSpan>(it.span)) {
                const BezierVerdict v = segmentBezier(a, b, *bz, need, 1e-9);
                rep.certified = rep.certified && v.atLeast;
                d = v.seen;
            } else if (const auto* arc = std::get_if<geometry::ArcSpan>(it.span)) {
                d = segmentArc(a, b, *arc);
                rep.certified = rep.certified && d >= need;
            } else {
                const auto& l = std::get<geometry::LineSpan>(*it.span);
                d = segmentSegment(a, b, l.start, l.end);
                rep.certified = rep.certified && d >= need;
            }
            if (d - radius < rep.minClearance) {
                rep.minClearance = d - radius;
                rep.worstMove = i;
            }
        }
    }
    return rep;
}

}  // namespace carvetto::test::pocket_curve
