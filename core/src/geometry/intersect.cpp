#include "carvetto/geometry/intersect.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"

namespace carvetto::geometry {

namespace {

// ------------------------------------------------------------------ small helpers

// Parameter slack equivalent to `tol.distance` along a span of length `len`.
double paramSlack(double len, const Tolerance& tol) {
    return len > 0.0 ? tol.distance / len : 1.0;
}

// Angular slack equivalent to `tol.distance` on a circle of radius `r`.
double angleSlack(double r, const Tolerance& tol) {
    return r > 0.0 ? tol.distance / r : kPi;
}

// Travelled angle from the arc's start to `angle`, measured in the arc's direction, in [0, 2pi).
double arcDelta(const ArcSpan& arc, double angle) {
    return arc.direction == ArcDirection::CCW ? normalizeAngle(angle - arc.startAngle)
                                              : normalizeAngle(arc.startAngle - angle);
}

// Parameter of the absolute `angle` on the arc, clamped to [0,1]. Angles just past the end
// (within `angTol`) map to 1, angles just before the start wrap to 0.
double arcParam(const ArcSpan& arc, double angle, double angTol) {
    const double sweep = std::fabs(arc.sweepAngle());
    double delta = arcDelta(arc, angle);
    if (delta > sweep + angTol) {
        delta -= kTwoPi;
    }
    return std::clamp(delta / sweep, 0.0, 1.0);
}

bool pointInOverlaps(double tA, const std::vector<Overlap>& overlaps, double slack) {
    return std::any_of(overlaps.begin(), overlaps.end(),
                       [&](const Overlap& o) { return tA >= o.tA0 - slack && tA <= o.tA1 + slack; });
}

void addPoint(IntersectionResult& out, Intersection hit, const Tolerance& tol) {
    for (const Intersection& existing : out.points) {
        if (tol.equalPoints(existing.point, hit.point)) {
            return;
        }
    }
    out.points.push_back(hit);
}

void finish(IntersectionResult& out, double slackA) {
    if (!out.overlaps.empty()) {
        std::erase_if(out.points,
                      [&](const Intersection& p) { return pointInOverlaps(p.tA, out.overlaps, slackA); });
    }
    std::stable_sort(out.points.begin(), out.points.end(),
                     [](const Intersection& l, const Intersection& r) { return l.tA < r.tA; });
    std::stable_sort(out.overlaps.begin(), out.overlaps.end(),
                     [](const Overlap& l, const Overlap& r) { return l.tA0 < r.tA0; });
}

// A contact whose gap is at most this is exact (a crossing or a shared point), not a touch.
double exactGap(const Tolerance& tol) {
    return 1e-6 * tol.distance;
}

// A full circle has no ends: its start is an arbitrary seam.
bool isClosedCircle(const Span& s) {
    const auto* arc = std::get_if<ArcSpan>(&s);
    return arc != nullptr && arc->isFullCircle();
}

// A degenerate span is a point: it intersects where it lies on the other span.
IntersectionResult pointAgainstSpan(Vec2 p, const Span& b, bool pointIsA, const Tolerance& tol) {
    IntersectionResult out;
    const ClosestPoint cp = closestPoint(b, p);
    if (cp.distance <= tol.distance) {
        out.points.push_back(pointIsA ? Intersection{cp.point, 0.0, cp.t, false}
                                      : Intersection{cp.point, cp.t, 0.0, false});
    }
    return out;
}

// Derivative of the span position with respect to its own parameter.
Vec2 derivativeAt(const Span& s, double t) {
    return std::visit(
        [t](const auto& v) -> Vec2 {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return v.end - v.start;
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                return v.tangentAt(t) * v.length();
            } else {
                return v.derivativeAt(t);
            }
        },
        s);
}

bool tangentsParallel(const Span& a, double tA, const Span& b, double tB) {
    const Vec2 ta = tangentAt(a, tA);
    const Vec2 tb = tangentAt(b, tB);
    return std::fabs(ta.cross(tb)) <= 1e-6;
}

// Same answer as `isDegenerate(s, tol)`, but avoids the high-precision Bezier arc length in all
// but borderline cases: the chord bounds a Bezier's length from below and the control polygon
// from above.
bool isZeroLength(const Span& s, const Tolerance& tol) {
    if (const auto* bz = std::get_if<BezierSpan>(&s)) {
        const double poly = distance(bz->p0, bz->p1) + distance(bz->p1, bz->p2) + distance(bz->p2, bz->p3);
        if (poly <= tol.distance) {
            return true;
        }
        if (distance(bz->p0, bz->p3) > tol.distance) {
            return false;
        }
        return bz->length(1e-6) <= tol.distance;
    }
    return isDegenerate(s, tol);
}

// ------------------------------------------------------------- analytic line / arc

// Canonical "less than" for segments, independent of their direction of travel. Used to break
// ties between equal-length segments so that `lineLine(a, b)` and `lineLine(b, a)` pick the same
// reference line.
bool segmentLess(const LineSpan& a, const LineSpan& b) {
    const auto key = [](const LineSpan& s) {
        const bool fwd = std::tie(s.start.x, s.start.y) <= std::tie(s.end.x, s.end.y);
        const Vec2 lo = fwd ? s.start : s.end;
        const Vec2 hi = fwd ? s.end : s.start;
        return std::array<double, 4>{lo.x, lo.y, hi.x, hi.y};
    };
    return key(a) < key(b);
}

// Parameter of the orthogonal projection of `p` onto the segment, clamped to [0,1].
double projectParam(const LineSpan& s, Vec2 p) {
    const Vec2 d = s.end - s.start;
    const double len2 = d.lengthSquared();
    return len2 > 0.0 ? std::clamp((p - s.start).dot(d) / len2, 0.0, 1.0) : 0.0;
}

// Line-line intersection, symmetric in its arguments: swapping A and B yields the same points
// and overlaps with the parameters swapped.
//
// Collinearity is judged against the infinite line of the LONGER segment, using both end points
// of the shorter one (a long segment's direction is the better-conditioned reference). When the
// segments are not collinear the infinite lines are intersected directly, however small the angle
// between them; only an exactly zero cross product means "parallel, never meeting". A crossing
// that lands outside either segment is not one; an end point within `tol.distance` of the other
// segment is an end contact (`endContacts`), whatever the angle.
IntersectionResult lineLine(const LineSpan& a, const LineSpan& b, const Tolerance& tol) {
    IntersectionResult out;
    const double la = a.length();
    const double lb = b.length();
    const bool aIsLong = la > lb || (la == lb && !segmentLess(b, a));
    const LineSpan& L = aIsLong ? a : b;
    const LineSpan& S = aIsLong ? b : a;
    const Vec2 dL = L.end - L.start;
    const double lL = std::max(la, lb);
    const auto lineDistance = [&](Vec2 p) { return std::fabs((p - L.start).cross(dL)) / lL; };

    if (lineDistance(S.start) <= tol.distance && lineDistance(S.end) <= tol.distance) {
        // Collinear: intersect the parameter intervals on L.
        const auto onL = [&](Vec2 p) { return (p - L.start).dot(dL) / (lL * lL); };
        const double s0 = onL(S.start);
        const double s1 = onL(S.end);
        const double lo = std::max(0.0, std::min(s0, s1));
        const double hi = std::min(1.0, std::max(s0, s1));
        const double slackL = paramSlack(lL, tol);
        if (hi - lo > slackL) {
            const Vec2 p0 = L.pointAt(lo);
            const Vec2 p1 = L.pointAt(hi);
            Overlap o{projectParam(a, p0), projectParam(a, p1), projectParam(b, p0), projectParam(b, p1)};
            if (o.tA0 > o.tA1) {
                std::swap(o.tA0, o.tA1);
                std::swap(o.tB0, o.tB1);
            }
            out.overlaps.push_back(o);
        } else if (hi - lo >= -slackL) {
            const Vec2 p = L.pointAt(std::clamp(0.5 * (lo + hi), 0.0, 1.0));
            out.points.push_back({p, projectParam(a, p), projectParam(b, p), true});
        }
        return out;
    }

    const Vec2 d1 = a.end - a.start;
    const Vec2 d2 = b.end - b.start;
    const double cross = d1.cross(d2);
    if (cross != 0.0) {
        // Both expressions negate exactly under an argument swap, so t and u swap exactly too.
        const Vec2 w = b.start - a.start;
        const double t = w.cross(d2) / cross;
        const double u = w.cross(d1) / cross;
        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
            // For nearly parallel lines t and u are ill-conditioned along the lines (A(t) and B(u)
            // can sit far apart along-line while both lie on both lines within tolerance). Report
            // the midpoint and re-derive both parameters from it by projection, so that the point
            // always agrees with A(tA) and B(tB). Projection is symmetric under the swap.
            const Vec2 p = (a.pointAt(t) + b.pointAt(u)) * 0.5;
            out.points.push_back({p, projectParam(a, p), projectParam(b, p), false});
        }
    }
    return out;
}

// True when the absolute `angle` lies on the arc (no slack beyond rounding; contacts just past an
// end are end contacts).
bool onArc(const ArcSpan& arc, double angle) {
    return arc.isFullCircle() || arc.containsAngle(angle, 0.0);
}

// Line-arc intersection from the foot of the perpendicular F from the centre onto the line:
// hits lie at F +- dir * sqrt(r^2 - h^2). Unlike the expanded quadratic this has no
// cancellation when the line is much longer than the circle is wide.
//
// Two crossings further apart than `tol.distance` are both reported. Closer ones (the line within
// tolerance of tangency) fold into one tangent hit at the foot, as does a line passing outside
// the circle within `tol.distance`. A hit is reported only where it lies on both spans; one just
// beyond an end is found as an end contact (`endContacts`).
IntersectionResult lineArc(const LineSpan& a, const ArcSpan& b, const Tolerance& tol) {
    IntersectionResult out;
    const Vec2 d = a.end - a.start;
    const double la = d.length();
    const Vec2 dir = d / la;
    const Vec2 f = a.start - b.centre;
    const double footAlong = -f.dot(dir);   // distance from a.start to the foot along the line
    const Vec2 foot = f + dir * footAlong;  // foot relative to the centre
    const double h = foot.length();

    const auto consider = [&](double along, bool tangent) {
        const double t = along / la;
        if (t < 0.0 || t > 1.0) {
            return;
        }
        const Vec2 p = b.centre + foot + dir * (along - footAlong);
        const double angle = (p - b.centre).angle();
        if (!onArc(b, angle)) {
            return;
        }
        addPoint(out, {p, t, arcParam(b, angle, 0.0), tangent}, tol);
    };

    if (h >= b.radius) {
        if (h - b.radius <= tol.distance) {
            consider(footAlong, true);
        }
        return out;
    }
    // (r - h)(r + h) avoids squaring large numbers.
    const double half = std::sqrt(std::max(0.0, (b.radius - h) * (b.radius + h)));
    if (2.0 * half <= tol.distance) {
        consider(footAlong, true);
        return out;
    }
    consider(footAlong - half, false);
    consider(footAlong + half, false);
    return out;
}

// Arcs on the same circle: compare angular intervals in A's travelled-angle space.
void coincidentArcs(const ArcSpan& a, const ArcSpan& b, IntersectionResult& out, const Tolerance& tol) {
    const double sA = std::fabs(a.sweepAngle());
    const double sB = std::fabs(b.sweepAngle());
    const double angTol = angleSlack(a.radius, tol);
    const double bStart = arcDelta(a, b.startAngle);
    const bool sameDir = a.direction == b.direction;
    const double bLo = sameDir ? bStart : bStart - sB;
    const double bHi = sameDir ? bStart + sB : bStart;
    const auto absAngle = [&](double delta) {
        return a.direction == ArcDirection::CCW ? a.startAngle + delta : a.startAngle - delta;
    };
    for (const double shift : {-kTwoPi, 0.0, kTwoPi}) {
        const double lo = std::max(0.0, bLo + shift);
        const double hi = std::min(sA, bHi + shift);
        // B's parameter at A's travelled angle x, computed in the same unwrapped frame so that a
        // full-circle B maps its end to 1 rather than wrapping back to 0.
        const auto bParam = [&](double x) {
            const double along = sameDir ? x - (bLo + shift) : (bHi + shift) - x;
            return std::clamp(along / sB, 0.0, 1.0);
        };
        if (hi - lo > angTol) {
            const Overlap o{lo / sA, hi / sA, bParam(lo), bParam(hi)};
            const bool dup = std::any_of(out.overlaps.begin(), out.overlaps.end(), [&](const Overlap& e) {
                return std::fabs(e.tA0 - o.tA0) <= 1e-12 && std::fabs(e.tA1 - o.tA1) <= 1e-12;
            });
            if (!dup) {
                out.overlaps.push_back(o);
            }
        } else if (hi - lo >= -angTol) {
            const double mid = std::clamp(0.5 * (lo + hi), 0.0, sA);
            const double angle = absAngle(mid);
            addPoint(out, {a.centre + Vec2::fromAngle(angle, a.radius), mid / sA, bParam(mid), true}, tol);
        }
    }
}

IntersectionResult swapped(IntersectionResult r);

// Arc-arc intersection. Callers pass the pair in canonical order (`analyticPair`), so the result
// is symmetric. Two crossings further apart than `tol.distance` are both reported; closer ones,
// and circles that pass within `tol.distance` of each other without crossing, give one tangent
// hit. A hit is reported only where it lies on both arcs; one just beyond an end is found as an
// end contact (`endContacts`).
IntersectionResult arcArc(const ArcSpan& a, const ArcSpan& b, const Tolerance& tol) {
    IntersectionResult out;
    const Vec2 cc = b.centre - a.centre;
    const double d = cc.length();
    const double r1 = a.radius;
    const double r2 = b.radius;

    if (d <= tol.distance && std::fabs(r1 - r2) <= tol.distance) {
        coincidentArcs(a, b, out, tol);
        return out;
    }
    // Nearly concentric circles (d <= tol) of different radii are not special: their closest
    // approach is |r1 - r2| - d, so they either miss (rejected here; this also covers d == 0) or
    // touch internally within tolerance and take the tangent branch below like any other pair.
    if (d > r1 + r2 + tol.distance || d < std::fabs(r1 - r2) - tol.distance) {
        return out;
    }
    const Vec2 u = cc / d;

    const auto consider = [&](Vec2 p, bool tangent) {
        const double angA = (p - a.centre).angle();
        const double angB = (p - b.centre).angle();
        if (!onArc(a, angA) || !onArc(b, angB)) {
            return;
        }
        addPoint(out, {p, arcParam(a, angA, 0.0), arcParam(b, angB, 0.0), tangent}, tol);
    };

    if (d < r1 + r2 && d > std::fabs(r1 - r2)) {
        // The circles cross. Distance from A's centre to the radical line, and half the chord;
        // the factored forms avoid cancellation between nearly equal squares. (Crossing circles
        // are more than `tol.distance` apart at the centres, see the overlap test above, so the
        // division by d is well conditioned.)
        const double along = ((r1 - r2) * (r1 + r2) + d * d) / (2.0 * d);
        const double h = std::sqrt(std::max(0.0, (r1 - along) * (r1 + along)));
        if (2.0 * h > tol.distance) {
            const Vec2 base = a.centre + u * along;
            consider(base + u.perp() * h, false);
            consider(base - u.perp() * h, false);
            return out;
        }
    }
    // The circles touch, nearly touch, or cross at two points the tolerance cannot separate: one
    // hit on the centre line. Take each circle's point on that line facing the other and report
    // their midpoint: both lie exactly on their circles and are at most `tol.distance` apart, so
    // the midpoint is within tolerance of both. (The two-circle formula for `along` divides the
    // radius gap by d and is useless here when the centres are close: its error grows like
    // r * gap / d.)
    const bool external = std::fabs(d - (r1 + r2)) <= std::fabs(d - std::fabs(r1 - r2));
    Vec2 onA;
    Vec2 onB;
    if (external) {
        onA = a.centre + u * r1;
        onB = b.centre - u * r2;
    } else if (r1 >= r2) {  // B inside A, touching on the +u side
        onA = a.centre + u * r1;
        onB = b.centre + u * r2;
    } else {  // A inside B, touching on the -u side
        onA = a.centre - u * r1;
        onB = b.centre - u * r2;
    }
    consider((onA + onB) * 0.5, true);
    return out;
}

// A box containing the span, cheap to compute: a line's own box, an arc's whole circle.
Box2 coarseBounds(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return Box2{l->start, l->end};
    }
    const auto& arc = std::get<ArcSpan>(s);
    return Box2::fromCentre(arc.centre, 2.0 * arc.radius, 2.0 * arc.radius);
}

// A lower bound of the distance from `p` to a line or arc span, cheap to compute: the distance to
// the infinite line, or to the whole circle.
double distanceLowerBound(const Span& s, Vec2 p) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        const Vec2 d = l->end - l->start;
        const double len = d.length();
        return len > 0.0 ? std::fabs((p - l->start).cross(d)) / len : distance(p, l->start);
    }
    const auto& arc = std::get<ArcSpan>(s);
    return std::fabs(distance(p, arc.centre) - arc.radius);
}

// True when no point of a line or arc span comes within `tol.distance` of the other's circle:
// circles further apart, or one inside the other, by more than the tolerance; a segment further
// than the tolerance outside a circle, or with both ends (so all of it) further inside.
bool apartBeyondTolerance(const Span& a, const Span& b, const Tolerance& tol) {
    const auto* arcA = std::get_if<ArcSpan>(&a);
    const auto* arcB = std::get_if<ArcSpan>(&b);
    if (arcA != nullptr && arcB != nullptr) {
        const double d = distance(arcA->centre, arcB->centre);
        return d > arcA->radius + arcB->radius + tol.distance ||
               d < std::fabs(arcA->radius - arcB->radius) - tol.distance;
    }
    if (arcA == nullptr && arcB == nullptr) {
        return false;
    }
    const ArcSpan& arc = arcA != nullptr ? *arcA : *arcB;
    const LineSpan& line = std::get<LineSpan>(arcA != nullptr ? b : a);
    const double inner = arc.radius - tol.distance;
    if (distance(line.start, arc.centre) < inner && distance(line.end, arc.centre) < inner) {
        return true;
    }
    const Vec2 d = line.end - line.start;
    const double len = d.length();
    return len > 0.0 && std::fabs((arc.centre - line.start).cross(d)) / len > arc.radius + tol.distance;
}

// Every end of either span that lies within `tol.distance` of the other span is a contact: the
// end paired with its nearest point on the other span, reported at their midpoint unless a hit is
// already within `tol.distance` of it. The ends are taken most exact first. An end contact is a
// touch (`tangent`) unless it is exact, i.e. the end lies on the other span (to 1e-6 tolerance),
// in which case it is tangent only where the two tangents are parallel. A full circle has no ends.
//
// This holds whether or not the end is a local minimum of the gap: an end inside a stretch where
// the spans run within tolerance of each other, or on the slope down to a crossing further than
// the tolerance away, is reported too (ADR-0009, T-0008 amendment).
std::vector<std::pair<double, Intersection>> endContacts(const Span& a, const Span& b, const Tolerance& tol) {
    std::vector<std::pair<double, Intersection>> ends;
    for (const bool endOfA : {true, false}) {
        const Span& s = endOfA ? a : b;
        const Span& o = endOfA ? b : a;
        if (isClosedCircle(s)) {
            continue;
        }
        for (const double t : {0.0, 1.0}) {
            const Vec2 e = pointAt(s, t);
            if (distanceLowerBound(o, e) > tol.distance) {
                continue;
            }
            const ClosestPoint cp = closestPoint(o, e);
            if (cp.distance > tol.distance) {
                continue;
            }
            const double tA = endOfA ? t : cp.t;
            const double tB = endOfA ? cp.t : t;
            const bool tangent = cp.distance > exactGap(tol) || tangentsParallel(a, tA, b, tB);
            ends.push_back({cp.distance, {(e + cp.point) * 0.5, tA, tB, tangent}});
        }
    }
    std::stable_sort(ends.begin(), ends.end(),
                     [](const auto& l, const auto& r) { return l.first < r.first; });
    return ends;
}

// Canonical argument order for the analytic pairs: lines before arcs, then by the defining
// numbers (start before end, so a segment and its reverse differ). Solving every pair in one
// order makes `intersect(a, b)` and `intersect(b, a)` exact mirror images.
bool analyticOrder(const Span& a, const Span& b) {
    if (a.index() != b.index()) {
        return a.index() < b.index();
    }
    const auto key = [](const Span& s) {
        if (const auto* l = std::get_if<LineSpan>(&s)) {
            return std::array<double, 6>{l->start.x, l->start.y, l->end.x, l->end.y, 0.0, 0.0};
        }
        const auto& c = std::get<ArcSpan>(s);
        return std::array<double, 6>{c.centre.x,   c.centre.y, c.radius,
                                     c.startAngle, c.endAngle, c.direction == ArcDirection::CCW ? 1.0 : 0.0};
    };
    return !(key(b) < key(a));
}

// Line/line, line/arc and arc/arc pairs: the analytic crossings (or overlaps), then the end
// contacts, all in canonical order.
IntersectionResult analyticPair(const Span& a, const Span& b, const Tolerance& tol) {
    IntersectionResult out;
    if (!coarseBounds(a).inflated(tol.distance).intersects(coarseBounds(b)) ||
        apartBeyondTolerance(a, b, tol)) {
        return out;  // no point of one span within tolerance of the other
    }
    if (!analyticOrder(a, b)) {
        return swapped(analyticPair(b, a, tol));
    }
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    if (la != nullptr && lb != nullptr) {
        out = lineLine(*la, *lb, tol);
    } else if (la != nullptr) {
        out = lineArc(*la, std::get<ArcSpan>(b), tol);
    } else {
        out = arcArc(std::get<ArcSpan>(a), std::get<ArcSpan>(b), tol);
    }
    for (const auto& end : endContacts(a, b, tol)) {
        addPoint(out, end.second, tol);
    }
    finish(out, out.overlaps.empty() ? 0.0 : paramSlack(length(a), tol));
    return out;
}

// --------------------------------------------------- polygonise + refine (Beziers)

struct ParamPoint {
    double t;
    Vec2 p;
};

void bezierSamples(const BezierSpan& b, double t0, double t1, double chordal, int depth,
                   std::vector<ParamPoint>& out) {
    if (b.flatness() <= chordal || depth >= 16) {
        out.push_back({t1, b.p3});
        return;
    }
    const auto [l, r] = b.split(0.5);
    const double tm = 0.5 * (t0 + t1);
    bezierSamples(l, t0, tm, chordal, depth + 1, out);
    bezierSamples(r, tm, t1, chordal, depth + 1, out);
}

std::vector<ParamPoint> sampleSpan(const Span& s, const Tolerance& tol) {
    std::vector<ParamPoint> out;
    std::visit(
        [&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                out.push_back({0.0, v.start});
                out.push_back({1.0, v.end});
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                const std::size_t n = v.segmentCountFor(tol.chordal);
                for (std::size_t i = 0; i <= n; ++i) {
                    const double t = static_cast<double>(i) / static_cast<double>(n);
                    out.push_back({t, v.pointAt(t)});
                }
            } else {
                out.push_back({0.0, v.p0});
                bezierSamples(v, 0.0, 1.0, tol.chordal, 0, out);
            }
        },
        s);
    return out;
}

struct Candidate {
    double tA;
    double tB;
    // Extra distance, beyond 1.5 * tol.distance, within which the candidate is still worth
    // settling: a sample next to a minimum that lies beyond the sampled range (see scanCrossings).
    double reach{0.0};
    // Set for a span end paired with its nearest point on the other span (see polygoniseAndRefine).
    bool spanEnd{false};
};

// Closest parameters between two straight segments (0 distance when they cross).
std::optional<std::pair<Candidate, double>> segmentProximity(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1,
                                                             double maxDist) {
    const Vec2 d1 = a1 - a0;
    const Vec2 d2 = b1 - b0;
    const double cross = d1.cross(d2);
    const Vec2 w = b0 - a0;
    if (std::fabs(cross) > 1e-14) {
        const double t = w.cross(d2) / cross;
        const double u = w.cross(d1) / cross;
        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
            return std::pair{Candidate{t, u}, 0.0};
        }
    }
    // No crossing: nearest pair is at an end point of one segment.
    std::optional<std::pair<Candidate, double>> best;
    const auto consider = [&](Candidate c, double dist) {
        if (dist <= maxDist && (!best || dist < best->second)) {
            best = std::pair{c, dist};
        }
    };
    const auto project = [](Vec2 p, Vec2 s0, Vec2 s1) {
        const Vec2 d = s1 - s0;
        const double len2 = d.lengthSquared();
        const double t = len2 > 0.0 ? std::clamp((p - s0).dot(d) / len2, 0.0, 1.0) : 0.0;
        return std::pair{t, distance(p, s0 + d * t)};
    };
    for (const auto& [tA, pa] : {std::pair{0.0, a0}, std::pair{1.0, a1}}) {
        const auto [u, dist] = project(pa, b0, b1);
        consider({tA, u}, dist);
    }
    for (const auto& [tB, pb] : {std::pair{0.0, b0}, std::pair{1.0, b1}}) {
        const auto [t, dist] = project(pb, a0, a1);
        consider({t, tB}, dist);
    }
    return best;
}

double residual(const Span& a, const Span& b, const Candidate& c) {
    return distance(pointAt(a, c.tA), pointAt(b, c.tB));
}

// Newton on A(t) - B(s) = 0: quadratic convergence at a transversal crossing. Stops when the
// Jacobian degenerates (parallel tangents), which the caller handles separately.
Candidate newton(const Span& a, const Span& b, Candidate c) {
    for (int iter = 0; iter < 40; ++iter) {
        const Vec2 r = pointAt(a, c.tA) - pointAt(b, c.tB);
        if (r.lengthSquared() <= 1e-30) {
            break;
        }
        const Vec2 da = derivativeAt(a, c.tA);
        const Vec2 db = derivativeAt(b, c.tB);
        // Solve [da, -db] * [dt, ds]^T = -r by Cramer's rule.
        const double det = -da.cross(db);
        if (std::fabs(det) <= 1e-9 * da.length() * db.length()) {
            break;
        }
        const double dt = ((-r).cross(-db)) / det;
        const double ds = (da.cross(-r)) / det;
        const Candidate next{std::clamp(c.tA + dt, 0.0, 1.0), std::clamp(c.tB + ds, 0.0, 1.0)};
        const bool settled = std::fabs(next.tA - c.tA) < 1e-15 && std::fabs(next.tB - c.tB) < 1e-15;
        c = next;
        if (settled) {
            break;
        }
    }
    return c;
}

// Closest point on `o` near parameter `seed`. Lines and arcs use the closed form. A Bezier is
// projected by Newton on (O(s) - p) . O'(s) = 0 from the seed, which costs a few evaluations
// instead of the global quintic root search; when the iteration does not settle on a minimum the
// global search is used. Callers walk along a curve in small steps and seed each query with the
// previous answer, so the local foot point is the one they want (and moves continuously).
ClosestPoint nearestFrom(const Span& o, Vec2 p, double seed) {
    const auto* z = std::get_if<BezierSpan>(&o);
    if (z == nullptr) {
        return closestPoint(o, p);
    }
    double s = std::clamp(seed, 0.0, 1.0);
    for (int iter = 0; iter < 20; ++iter) {
        const Vec2 d1 = z->derivativeAt(s);
        const Vec2 d2 = ((z->p2 - z->p1 * 2.0 + z->p0) * (1.0 - s) + (z->p3 - z->p2 * 2.0 + z->p1) * s) * 6.0;
        const Vec2 r = z->pointAt(s) - p;
        const double g = r.dot(d1);
        const double gp = d1.dot(d1) + r.dot(d2);
        if (!(gp > 0.0)) {
            break;
        }
        const double next = std::clamp(s - g / gp, 0.0, 1.0);
        // Quadratic convergence: a step this small means the previous one was already exact to
        // rounding (a 1e-15 threshold would never trigger once rounding noise dominates).
        if (std::fabs(next - s) <= 1e-12) {
            const Vec2 q = z->pointAt(next);
            return {q, next, distance(p, q)};
        }
        s = next;
    }
    return closestPoint(o, p);
}

// Refines a chord seed by Newton on A(t) - B(s) = 0. At a transversal crossing this converges
// quadratically; where the tangents are (nearly) parallel Newton stops early and the caller
// discards the seed in favour of the signed-distance scan.
Candidate refine(const Span& a, const Span& b, Candidate seed) {
    return newton(a, b, seed);
}

IntersectionResult polygoniseAndRefine(const Span& a, const Span& b, const Tolerance& tol);

// Signed distance from S(t) to the other span O (positive on O's left), with O's parameter of
// the closest point. Changes sign exactly where S crosses O, wherever the closest point moves
// continuously.
struct SideSample {
    double t;
    double f;
    double tO;
};

// `seedO` < 0 asks for the global closest point; otherwise the foot point near O(seedO).
SideSample sideSample(const Span& s, const Span& o, double t, double seedO) {
    const Vec2 p = pointAt(s, t);
    const ClosestPoint cp = seedO < 0.0 ? closestPoint(o, p) : nearestFrom(o, p, seedO);
    return {t, tangentAt(o, cp.t).cross(p - cp.point), cp.t};
}

// Root of the signed distance on a bracket [l, r] where it changes sign: regula falsi with the
// Illinois modification (superlinear, and never leaves the bracket), stopping once the distance
// is below `exact`.
SideSample bisectSide(const Span& s, const Span& o, SideSample l, SideSample r, double exact) {
    double fl = l.f;
    double fr = r.f;
    int side = 0;
    for (int iter = 0; iter < 100 && r.t - l.t > 1e-15; ++iter) {
        double t = (l.t * fr - r.t * fl) / (fr - fl);
        if (!(t > l.t && t < r.t)) {
            t = 0.5 * (l.t + r.t);
        }
        const SideSample m = sideSample(s, o, t, 0.5 * (l.tO + r.tO));
        if (std::fabs(m.f) <= exact) {
            return m;
        }
        if ((m.f > 0.0) == (l.f > 0.0)) {
            l = m;
            fl = m.f;
            if (side == -1) {
                fr *= 0.5;
            }
            side = -1;
        } else {
            r = m;
            fr = m.f;
            if (side == 1) {
                fl *= 0.5;
            }
            side = 1;
        }
    }
    return std::fabs(l.f) <= std::fabs(r.f) ? l : r;
}

// Scan of S's signed distance to O over S's parameter interval [t0, t1] (one chord of S),
// appending candidates as (tO, tS). The interval is sampled at 9 points: a sign change between
// neighbours is a crossing; a sample no further from O than both neighbours marks a dip, which is
// searched for its extremum: beyond O it brackets a shallow double crossing that no chord-based
// seed resolves reliably, on the same side within `tol` it is a touch. Samples at chord and span
// ends, and samples whose nearest point is clamped at an end of O, are touch candidates too when
// within reach of `tol`; `settleTouch` later walks every touch candidate to the true minimum.
// `spanStart`/`spanEnd` say whether t0/t1 are the ends of S itself. Signed distances within
// `noise` of zero carry no sign information (e.g. along coincident stretches) and are skipped.
void scanCrossings(const Span& s, const Span& o, double t0, double t1, bool spanStart, bool spanEnd,
                   double noise, double tol, std::vector<Candidate>& out) {
    constexpr std::size_t kPieces = 8;
    const auto clamped = [](const SideSample& x) { return x.tO <= 0.0 || x.tO >= 1.0; };
    std::array<SideSample, kPieces + 1> samples{};
    for (std::size_t k = 0; k <= kPieces; ++k) {
        const double f = static_cast<double>(k) / static_cast<double>(kPieces);
        samples[k] =
            sideSample(s, o, k == kPieces ? t1 : t0 + (t1 - t0) * f, k == 0 ? -1.0 : samples[k - 1].tO);
    }
    const auto bracket = [&](const SideSample& l, const SideSample& r) {
        const SideSample root = bisectSide(s, o, l, r, 1e-3 * noise);
        out.push_back({root.tO, root.t});
    };
    // Sign changes between neighbouring samples are crossings.
    for (std::size_t k = 0; k < kPieces; ++k) {
        const SideSample& l = samples[k];
        const SideSample& r = samples[k + 1];
        if (std::fabs(l.f) <= noise && std::fabs(r.f) <= noise) {
            continue;
        }
        if ((l.f > 0.0) != (r.f > 0.0)) {
            bracket(l, r);
        }
    }
    // A sample at zero distance (to rounding) carries no sign: typically the junction S shares
    // with O, or a crossing that falls exactly on a sample. The neighbouring piece may still
    // swing to the other side of O and come back to the zero, i.e. cross O once more, without
    // any sign change between samples. The signed distance, oriented by the non-zero neighbour,
    // is unimodal over such a piece (down through the crossing to the swing's peak, back up to
    // the zero), so its minimum is found by golden section and, when it lies beyond O, brackets
    // the extra crossing against the non-zero sample.
    constexpr double invPhi = 0.6180339887498949;
    for (std::size_t k = 0; k < kPieces; ++k) {
        const SideSample& l = samples[k];
        const SideSample& r = samples[k + 1];
        const bool lz = std::fabs(l.f) <= noise;
        const bool rz = std::fabs(r.f) <= noise;
        if (lz == rz) {
            continue;
        }
        const SideSample& nz = lz ? r : l;
        const double sign = nz.f > 0.0 ? 1.0 : -1.0;
        double lo = l.t;
        double hi = r.t;
        SideSample x1 = sideSample(s, o, hi - invPhi * (hi - lo), nz.tO);
        SideSample x2 = sideSample(s, o, lo + invPhi * (hi - lo), nz.tO);
        for (int iter = 0; iter < 16 && sign * x1.f >= -noise && sign * x2.f >= -noise; ++iter) {
            if (sign * x1.f < sign * x2.f) {
                hi = x2.t;
                x2 = x1;
                x1 = sideSample(s, o, hi - invPhi * (hi - lo), x2.tO);
            } else {
                lo = x1.t;
                x1 = x2;
                x2 = sideSample(s, o, lo + invPhi * (hi - lo), x1.tO);
            }
        }
        const SideSample& m = sign * x1.f < sign * x2.f ? x1 : x2;
        if (sign * m.f < -noise) {
            if (lz) {
                bracket(m, r);
            } else {
                bracket(l, m);
            }
        }
    }
    // An interior sample at which the signed distance is (weakly) closer to O than at both
    // neighbours marks a dip: a shallow double crossing hiding between samples, or a touch. The
    // extremum towards O is searched by golden section over the two pieces around the sample.
    // Only dips that can reach tolerance are worth searching: over a piece the distance can fall
    // below its sampled minimum by at most about the variation to the neighbours, so samples
    // further away than 1.5 tol plus twice that variation are skipped (whatever the curvature of
    // O: the bound comes from the samples, not from a chordal assumption).
    for (std::size_t k = 1; k < kPieces; ++k) {
        const SideSample& l = samples[k - 1];
        const SideSample& c = samples[k];
        const SideSample& r = samples[k + 1];
        if ((l.f > 0.0) != (c.f > 0.0) || (r.f > 0.0) != (c.f > 0.0) || std::fabs(c.f) <= noise) {
            continue;  // a crossing already bracketed, or a coincident stretch
        }
        const double sign = c.f > 0.0 ? 1.0 : -1.0;
        // A neighbour whose nearest point is clamped at an end of O measures the distance to that
        // end point, not a foot on O, so it says nothing about the slope: it never rules a dip out.
        const double lf = clamped(l) ? std::numeric_limits<double>::infinity() : sign * l.f;
        const double rf = clamped(r) ? std::numeric_limits<double>::infinity() : sign * r.f;
        if (sign * c.f > lf || sign * c.f > rf) {
            continue;  // not a discrete minimum
        }
        double variation = 0.0;
        if (!clamped(l)) {
            variation = std::max(variation, std::fabs(l.f - c.f));
        }
        if (!clamped(r)) {
            variation = std::max(variation, std::fabs(r.f - c.f));
        }
        if (sign * c.f > 1.5 * tol + 2.0 * variation) {
            continue;
        }
        double lo = l.t;
        double hi = r.t;
        SideSample x1 = sideSample(s, o, hi - invPhi * (hi - lo), c.tO);
        SideSample x2 = sideSample(s, o, lo + invPhi * (hi - lo), c.tO);
        // Twelve steps shrink the bracket to under 1% of the pieces: enough to tell a crossing
        // pair from a touch, and a touch candidate is polished later by settleTouch.
        for (int iter = 0; iter < 12 && sign * x1.f >= -noise && sign * x2.f >= -noise; ++iter) {
            if (sign * x1.f < sign * x2.f) {
                hi = x2.t;
                x2 = x1;
                x1 = sideSample(s, o, hi - invPhi * (hi - lo), x2.tO);
            } else {
                lo = x1.t;
                x1 = x2;
                x2 = sideSample(s, o, lo + invPhi * (hi - lo), x1.tO);
            }
        }
        SideSample m = sign * x1.f < sign * x2.f ? x1 : x2;
        if (sign * c.f < sign * m.f) {
            m = c;
        }
        if (sign * m.f < -noise) {
            bracket(l, m);
            bracket(m, r);
        } else if (!clamped(m) && std::fabs(m.f) <= 1.5 * tol) {
            // A touch. The slack covers the unfinished search; acceptance settles and re-measures
            // the minimum. A sample whose nearest point is clamped at an end of O measures the
            // distance to that end point, not a foot on O; those are handled below.
            out.push_back({m.tO, m.t});
        }
    }
    // A closest approach that no piece shows as an interior dip: the minimum lies at an end of
    // this chord (the distance slopes down to it), at an end of S, or on or beyond a cut of O (a
    // sample whose nearest point is clamped there). Within reach of tolerance such samples become
    // candidates, and `settleTouch` walks them to the true minimum on the unsplit spans.
    for (std::size_t k = 0; k <= kPieces; ++k) {
        const SideSample& x = samples[k];
        const bool chordEnd = k == 0 || k == kPieces;
        // A chord end that is no further from O than its inner neighbour is the sampled bottom of
        // a dip that the interior test cannot see, because the dip straddles the end of the chord
        // (the neighbouring chord or piece has the same sample as its own end) or the end of S.
        // Within the same reach as an interior dip, the extremum towards O over the end piece is
        // searched by golden section: beyond O it brackets a double crossing confined to that
        // piece (e.g. S swinging across O and back just before its end); otherwise the end is a
        // touch candidate, settled later on the unsplit spans (the minimum may lie beyond the
        // end, so the candidate carries the variation as extra reach).
        if (chordEnd) {
            const SideSample& inner = samples[k == 0 ? 1 : kPieces - 1];
            const double fx = std::fabs(x.f);
            const double fi = std::fabs(inner.f);
            const double allowance = clamped(inner) ? 0.0 : 2.0 * (fi - fx);
            if ((x.f > 0.0) == (inner.f > 0.0) && fx <= fi && fx > noise && fx <= 1.5 * tol + allowance) {
                const double sign = x.f > 0.0 ? 1.0 : -1.0;
                double lo = std::min(x.t, inner.t);
                double hi = std::max(x.t, inner.t);
                SideSample x1 = sideSample(s, o, hi - invPhi * (hi - lo), x.tO);
                SideSample x2 = sideSample(s, o, lo + invPhi * (hi - lo), x.tO);
                for (int iter = 0; iter < 12 && sign * x1.f >= -noise && sign * x2.f >= -noise; ++iter) {
                    if (sign * x1.f < sign * x2.f) {
                        hi = x2.t;
                        x2 = x1;
                        x1 = sideSample(s, o, hi - invPhi * (hi - lo), x2.tO);
                    } else {
                        lo = x1.t;
                        x1 = x2;
                        x2 = sideSample(s, o, lo + invPhi * (hi - lo), x1.tO);
                    }
                }
                const SideSample& m = sign * x1.f < sign * x2.f ? x1 : x2;
                if (sign * m.f < -noise) {
                    bracket(k == 0 ? x : inner, m);
                    bracket(m, k == 0 ? inner : x);
                    continue;
                }
                if (fx > 1.5 * tol) {
                    out.push_back({x.tO, x.t, allowance});
                    continue;
                }
            }
        }
        if (std::fabs(x.f) > 1.5 * tol) {
            continue;
        }
        const bool chordEndMinimum = (k == 0 && std::fabs(x.f) <= std::fabs(samples[1].f)) ||
                                     (k == kPieces && std::fabs(x.f) <= std::fabs(samples[kPieces - 1].f));
        const bool atEndOfS = (k == 0 && spanStart) || (k == kPieces && spanEnd);
        if (clamped(x) || atEndOfS || chordEndMinimum) {
            out.push_back({x.tO, x.t});
        }
    }
}

// Crossing candidates of one pair of well-behaved pieces (`b` is a Bezier piece), in the pieces'
// own parameters: chord seeds refined by Newton, plus the signed-distance scan of B's near chords.
void pieceCandidates(const Span& a, const Span& b, const Tolerance& tol, std::vector<Candidate>& found) {
    const std::vector<ParamPoint> pa = sampleSpan(a, tol);
    const std::vector<ParamPoint> pb = sampleSpan(b, tol);
    // Chords deviate from the curves by up to tol.chordal each, so near misses up to twice that
    // (plus the acceptance tolerance) may still be genuine contacts.
    const double reach = 2.0 * tol.chordal + tol.distance;
    std::vector<bool> nearB(pb.size(), false);
    for (std::size_t i = 1; i < pa.size(); ++i) {
        const Box2 boxA = Box2{pa[i - 1].p, pa[i].p}.inflated(reach);
        for (std::size_t j = 1; j < pb.size(); ++j) {
            if (!boxA.intersects(Box2{pb[j - 1].p, pb[j].p})) {
                continue;
            }
            const auto prox = segmentProximity(pa[i - 1].p, pa[i].p, pb[j - 1].p, pb[j].p, reach);
            if (!prox) {
                continue;
            }
            nearB[j] = true;
            const double wA = pa[i].t - pa[i - 1].t;
            const Candidate seed{pa[i - 1].t + wA * prox->first.tA,
                                 pb[j - 1].t + (pb[j].t - pb[j - 1].t) * prox->first.tB};
            // A seed that Newton converges is a crossing. One that it cannot converge lies where the
            // tangents are nearly parallel: a near miss or a touch. Every touch within tolerance has
            // such a chord pair, so the seed is kept as a touch candidate too, to be walked to the
            // local minimum of the gap by `settleTouch`. (The scan below also finds the samples
            // near a touch, but only within bounds estimated from the samples, which narrow dips
            // near piece cuts and span ends defeat.)
            const Candidate c = refine(a, b, seed);
            if (residual(a, b, c) <= 1e-6 * tol.distance) {
                found.push_back(c);
            } else {
                // The same fraction along a chord need not be the nearest curve point (a Bezier's
                // speed varies along a chord), so the closest point of A's chord is projected onto
                // A, and that onto B. Each curve lies within tol.chordal of its chord and the chords
                // come within `reach`, so the gap is below 4 * tol.chordal + tol.distance.
                const Vec2 onChordA = pa[i - 1].p + (pa[i].p - pa[i - 1].p) * prox->first.tA;
                const double tA = nearestFrom(a, onChordA, seed.tA).t;
                const ClosestPoint cp = nearestFrom(b, pointAt(a, tA), seed.tB);
                found.push_back({tA, cp.t, 4.0 * tol.chordal});
            }
        }
    }
    // Newton from chord seeds misses one of two crossings that lie close together along a shallow
    // contact (both seeds converge to the same root). Every chord of B near A is therefore also
    // scanned for sign changes of its distance to A.
    std::vector<Candidate> scanned;
    for (std::size_t j = 1; j < pb.size(); ++j) {
        if (nearB[j]) {
            scanCrossings(b, a, pb[j - 1].t, pb[j].t, j == 1, j + 1 == pb.size(), 1e-6 * tol.distance,
                          tol.distance, scanned);
        }
    }
    found.insert(found.end(), scanned.begin(), scanned.end());
}

// A piece of a span: the span restricted to its parameter interval [u0, u1].
struct Piece {
    Span span;
    double u0;
    double u1;
};

// Largest angle between two non-negligible first-derivative control vectors of a cubic. B'(t) is a
// non-negative combination of them, so the tangent stays inside the cone they span and this
// bounds how far it turns. A vanishing end vector is skipped; near a cusp the others still point
// in widely different directions, which keeps the piece splitting.
double hodographSpread(const BezierSpan& z) {
    const std::array<Vec2, 3> d{z.p1 - z.p0, z.p2 - z.p1, z.p3 - z.p2};
    const double scale = std::max({d[0].length(), d[1].length(), d[2].length()});
    if (scale <= 0.0) {
        return 0.0;
    }
    double spread = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = i + 1; j < 3; ++j) {
            const double li = d[i].length();
            const double lj = d[j].length();
            if (li <= 1e-9 * scale || lj <= 1e-9 * scale) {
                continue;  // a vanishing end derivative alone does not turn the tangent
            }
            spread = std::max(spread, std::atan2(std::fabs(d[i].cross(d[j])), d[i].dot(d[j])));
        }
    }
    return spread;
}

// Splits a Bezier by halving until the tangent turns by at most 45 degrees over each piece, so a
// loop, cusp or tight bend is cut into pieces that each have one well-defined nearest point for
// queries near them. Splitting stops at depth 10 or when a piece is shorter than the tolerance.
void splitBezier(const BezierSpan& z, double u0, double u1, int depth, const Tolerance& tol,
                 std::vector<Piece>& out) {
    const double poly = distance(z.p0, z.p1) + distance(z.p1, z.p2) + distance(z.p2, z.p3);
    if (depth >= 10 || poly <= tol.distance || hodographSpread(z) <= 0.25 * kPi) {
        out.push_back({Span{z}, u0, u1});
        return;
    }
    const auto [l, r] = z.split(0.5);
    const double um = 0.5 * (u0 + u1);
    splitBezier(l, u0, um, depth + 1, tol, out);
    splitBezier(r, um, u1, depth + 1, tol, out);
}

std::vector<Piece> piecesOf(const Span& s, const Tolerance& tol) {
    std::vector<Piece> out;
    if (const auto* z = std::get_if<BezierSpan>(&s)) {
        splitBezier(*z, 0.0, 1.0, 0, tol, out);
    } else {
        out.push_back({s, 0.0, 1.0});
    }
    return out;
}

// Gap between A(t) and B, with B's parameter of the nearest point (locally, seeded from `seedB`).
struct GapSample {
    double t;
    double tB;
    double gap;
};

GapSample gapAlongA(const Span& a, const Span& b, double t, double seedB) {
    const ClosestPoint cp = nearestFrom(b, pointAt(a, t), seedB);
    return {t, cp.t, cp.distance};
}

// Nearest-point evaluations carry rounding noise of this order; gap differences below it say
// nothing about the slope.
constexpr double kGapNoise = 1e-12;
// Walk steps stay well below the spacing of features along near-coincident curves, so a descent
// does not leap across a crossing into another dip.
constexpr double kMaxWalkStep = 1.0 / 64.0;

// Follows the gap along A downhill from `mid` in the direction of `next` (a sample one `step`
// away whose gap is lower), with growing steps until the gap rises again, then narrows the
// bracket by golden section. Stops at an end of A, unless A is `periodic` (a full circle, whose
// parameter wraps around and which has no ends). Returns the lowest sample seen; `probe` evaluates
// and records a sample.
template <typename Probe>
GapSample descend(GapSample mid, GapSample next, double step, bool periodic,
                  const std::vector<GapSample>* known, const Probe& probe) {
    const bool forward = next.t > mid.t;
    GapSample behind = mid;
    for (int iter = 0; iter < 80 && next.gap < mid.gap; ++iter) {
        if (!periodic && ((forward && next.t >= 1.0) || (!forward && next.t <= 0.0))) {
            return next;  // ran into an end of A while still descending: an end touch
        }
        behind = mid;
        mid = next;
        step = std::min(2.0 * step, kMaxWalkStep);
        next = probe(forward ? mid.t + step : mid.t - step, mid.tB);
    }
    // Bracket [behind, next] around mid; golden section inside it.
    constexpr double invPhi = 0.6180339887498949;
    double l = std::min(behind.t, next.t);
    double h = std::max(behind.t, next.t);
    // A minimum that an earlier walk settled inside this bracket, at or below its lowest sample,
    // is where this walk would end too: return it instead of searching again (along
    // near-coincident curves many candidates descend into the same few minima).
    if (known != nullptr) {
        const GapSample* found = nullptr;
        for (const GapSample& k : *known) {
            if (k.t > l && k.t < h && k.gap <= mid.gap && (found == nullptr || k.gap < found->gap)) {
                found = &k;
            }
        }
        if (found != nullptr) {
            return *found;
        }
    }
    GapSample x1 = probe(h - invPhi * (h - l), mid.tB);
    GapSample x2 = probe(l + invPhi * (h - l), mid.tB);
    for (int iter = 0; iter < 100 && h - l > 1e-10; ++iter) {
        if (x1.gap < x2.gap) {
            h = x2.t;
            x2 = x1;
            x1 = probe(h - invPhi * (h - l), x2.tB);
        } else {
            l = x1.t;
            x1 = x2;
            x2 = probe(l + invPhi * (h - l), x1.tB);
        }
    }
    GapSample best = x1.gap < x2.gap ? x1 : x2;
    for (const GapSample* g : {&mid, &behind, &next}) {
        if (g->gap < best.gap) {
            best = *g;
        }
    }
    return best;
}

// Settles a touch candidate to the local minima of the distance between the curves that are
// reached by walking downhill along the unsplit A. A descent that reaches an end of A stops there
// (an end touch). When the gap falls away on both sides of the candidate (it sits on a ridge, for
// example at a cut between pieces), both descents are followed and both minima are returned.
// Nothing is dropped for being at a cut between pieces, because the walk runs on the original
// span. Each result is checked to be a local minimum of the distance to B's nearest branch, i.e.
// no closer approach a step either side (up to noise); a walk that ended on a slope (a bracket
// that was not unimodal, or a branch change of the nearest point) is dropped, and the real minima
// are reached from other candidates.
//
// `minima` collects the minima accepted by earlier walks for the same span pair; a walk whose
// final bracket contains one at or below its lowest sample ends there (see `descend`).
//
// `trail` collects every gap evaluation of earlier walks for the same span pair (sorted by t). A
// new candidate that lands between two earlier samples less than a step apart, with its gap
// between theirs, sits on a stretch already walked downhill and is skipped: the minimum at the
// bottom of that slope was reached from the earlier candidate. Along near-coincident curves this
// turns hundreds of chord seeds into a handful of walks.
//
// A full circle has no ends: its seam is an arbitrary point, so the walk wraps around it (samples
// are evaluated and recorded at the wrapped parameter) instead of stopping there.
std::vector<Candidate> settleTouch(const Span& a, const Span& b, Candidate c, std::vector<GapSample>& trail,
                                   std::vector<GapSample>& minima) {
    std::vector<Candidate> out;
    const auto* arcA = std::get_if<ArcSpan>(&a);
    const bool periodic = arcA != nullptr && arcA->isFullCircle();
    const auto wrap = [periodic](double t) { return periodic ? t - std::floor(t) : std::clamp(t, 0.0, 1.0); };
    const auto record = [&](const GapSample& g) {
        const auto pos = std::lower_bound(trail.begin(), trail.end(), g.t,
                                          [](const GapSample& e, double t) { return e.t < t; });
        trail.insert(pos, g);
    };
    const GapSample mid = gapAlongA(a, b, std::clamp(c.tA, 0.0, 1.0), c.tB);
    // Earlier samples either side of the candidate, if any (looked up before this walk records).
    std::optional<GapSample> trailL;
    std::optional<GapSample> trailR;
    {
        const auto right = std::lower_bound(trail.begin(), trail.end(), mid.t,
                                            [](const GapSample& e, double t) { return e.t < t; });
        if (right != trail.end() && right != trail.begin()) {
            trailL = *(right - 1);
            trailR = *right;
        }
    }
    const bool walked = trailL && trailR && trailR->t - trailL->t <= kMaxWalkStep;
    const auto between = [&] {
        const GapSample& l = *trailL;
        const GapSample& r = *trailR;
        return (l.gap <= mid.gap && mid.gap <= r.gap) || (l.gap >= mid.gap && mid.gap >= r.gap);
    };
    // A chord seed (a candidate with extra reach) is a backstop for touches the scan cannot
    // bound; along near-coincident curves there are hundreds of them. One that lands between two
    // earlier samples less than a walk step apart, with its gap between theirs, is skipped
    // outright, like any slope candidate below, without probing which way it descends.
    if (!c.spanEnd && c.reach > 0.0 && walked && between()) {
        return out;
    }
    record(mid);
    const auto probe = [&](double t, double seed) {
        GapSample g = gapAlongA(a, b, wrap(t), seed);
        record(g);
        if (periodic) {
            g.t = t;  // the walk itself runs on the unwrapped parameter
        }
        return g;
    };
    // Initial probes a tiny step either side, widened while the gap is flat to within noise.
    double step = 1e-6;
    GapSample lo = probe(mid.t - step, mid.tB);
    GapSample hi = probe(mid.t + step, mid.tB);
    for (int iter = 0; iter < 40 && step < kMaxWalkStep && std::fabs(lo.gap - mid.gap) <= kGapNoise &&
                       std::fabs(hi.gap - mid.gap) <= kGapNoise;
         ++iter) {
        step *= 2.0;
        lo = probe(mid.t - step, mid.tB);
        hi = probe(mid.t + step, mid.tB);
    }
    // A candidate on a slope that descends towards an earlier sample less than a walk step away,
    // with its gap between those of the two earlier samples around it, would only retrace a
    // stretch already walked downhill: skip it. A candidate at the bottom of the gap (no lower
    // probe either side) or on a ridge is always walked, because the earlier samples around it say
    // nothing about a minimum between them (e.g. a touch next to a crossing). A span end is always
    // walked: where the curves run within tolerance of each other the gap is flat to rounding, the
    // trail around the end says nothing about whether the end is the bottom, and an end touch there
    // was lost.
    if (walked && !c.spanEnd) {
        const GapSample& l = *trailL;
        const GapSample& r = *trailR;
        const bool downLeft = lo.gap < mid.gap && hi.gap >= mid.gap && l.gap < mid.gap;
        const bool downRight = hi.gap < mid.gap && lo.gap >= mid.gap && r.gap < mid.gap;
        if (between() && (downLeft || downRight)) {
            return out;
        }
    }
    const auto accept = [&](const GapSample& best) {
        for (const double t : {best.t - 1e-5, best.t + 1e-5}) {
            if ((periodic || (t >= 0.0 && t <= 1.0)) &&
                gapAlongA(a, b, wrap(t), best.tB).gap < best.gap - kGapNoise) {
                return;
            }
        }
        out.push_back({wrap(best.t), best.tB});
        minima.push_back({wrap(best.t), best.tB, best.gap});
    };
    if (lo.gap < mid.gap) {
        accept(descend(mid, lo, step, periodic, periodic ? nullptr : &minima, probe));
    }
    if (hi.gap < mid.gap) {
        accept(descend(mid, hi, step, periodic, periodic ? nullptr : &minima, probe));
    }
    if (lo.gap >= mid.gap && hi.gap >= mid.gap) {
        accept(descend(mid, lo.gap <= hi.gap ? lo : hi, step, periodic, periodic ? nullptr : &minima,
                       probe));  // already at the bottom
    }
    return out;
}

// Pairs involving a Bezier. Callers pass them in canonical order (`b` is always the Bezier), so
// the result does not depend on which span the user passed first.
//
// Each Bezier is first cut into pieces over which its tangent turns by at most 45 degrees. Near a
// loop or cusp a single curve passes the same neighbourhood on several branches; working piece
// against piece keeps every seed, Newton step and nearest-point query on one branch of each
// curve, so a crossing of one branch with another branch cannot be absorbed by a nearby crossing
// of the same branches.
IntersectionResult polygoniseAndRefine(const Span& a, const Span& b, const Tolerance& tol) {
    const double reach = 2.0 * tol.chordal + tol.distance;
    const std::vector<Piece> piecesA = piecesOf(a, tol);
    const std::vector<Piece> piecesB = piecesOf(b, tol);
    std::vector<Box2> boxesB;
    for (const Piece& q : piecesB) {
        boxesB.push_back(bounds(q.span).inflated(reach));
    }
    std::vector<Candidate> found;
    std::vector<Candidate> local;
    for (const Piece& p : piecesA) {
        const Box2 boxA = bounds(p.span);
        for (std::size_t j = 0; j < piecesB.size(); ++j) {
            if (!boxA.intersects(boxesB[j])) {
                continue;
            }
            const Piece& q = piecesB[j];
            local.clear();
            pieceCandidates(p.span, q.span, tol, local);
            for (const Candidate& c : local) {
                // De Casteljau halving is an affine reparametrisation, so this mapping is exact.
                found.push_back({p.u0 + (p.u1 - p.u0) * c.tA, q.u0 + (q.u1 - q.u0) * c.tB, c.reach});
            }
        }
    }
    // The ends of both spans, paired with their nearest points on the other span. An end touch is
    // a minimum of the distance along one span only (along the other the distance may keep falling
    // towards a crossing), so neither the scan, which runs along B, nor the walk, which runs along
    // A, need reach it from elsewhere; seeding it directly makes end touches symmetric.
    // (The bounding boxes rule out most ends before the global nearest-point search.)
    //
    // An end within `tol.distance` of the other span is moreover a contact in its own right, kept
    // as it is (`endHits`): where the spans run within tolerance of each other the gap along that
    // stretch is flat or slopes away from the end, so the settling walk leaves the end, yet the end
    // is where the stretch stops (T-0008). A full circle (only A can be one) has no ends.
    const Box2 nearA = bounds(a).inflated(1.5 * tol.distance);
    const Box2 nearB = bounds(b).inflated(1.5 * tol.distance);
    const bool aHasEnds = !isClosedCircle(a);
    std::vector<Candidate> endHits;
    for (const double t : {0.0, 1.0}) {
        const Vec2 endA = pointAt(a, t);
        if (nearB.contains(endA)) {
            const ClosestPoint onB = closestPoint(b, endA);
            if (onB.distance <= 1.5 * tol.distance) {
                found.push_back({t, onB.t, 0.0, true});
            }
            if (aHasEnds && onB.distance <= tol.distance) {
                endHits.push_back({t, onB.t, 0.0, true});
            }
        }
        const Vec2 endB = pointAt(b, t);
        if (nearA.contains(endB)) {
            const ClosestPoint onA = closestPoint(a, endB);
            if (onA.distance <= 1.5 * tol.distance) {
                found.push_back({onA.t, t, 0.0, true});
            }
            if (onA.distance <= tol.distance) {
                endHits.push_back({onA.t, t, 0.0, true});
            }
        }
    }

    // Accept, then merge hits within tol.distance, most exact first so that each merged group is
    // represented by its best contact. Two distinct crossings merged into one hit are a contact
    // the tolerance cannot resolve, so the hit becomes tangent (end contacts never count as
    // crossings for this rule: an end lying on the other span is not a second crossing).
    struct Raw {
        double res;
        Intersection hit;
        bool end{false};
    };
    std::vector<Raw> raw;
    std::vector<GapSample> trail;
    std::vector<GapSample> minima;
    std::vector<std::pair<double, Candidate>> touches;
    const auto emit = [&](Candidate c, double res, bool exact, bool end) {
        if (exact) {
            // Along a tangential contact Newton stops anywhere in a band where the curves agree
            // to rounding; when that band reaches a span end, report the end itself.
            const auto snap = [](double t) { return t < 1e-6 ? 0.0 : (t > 1.0 - 1e-6 ? 1.0 : t); };
            Candidate snapped = c;
            if (snap(c.tA) != c.tA) {
                snapped.tA = snap(c.tA);
                snapped.tB = nearestFrom(b, pointAt(a, snapped.tA), c.tB).t;
            } else if (snap(c.tB) != c.tB) {
                snapped.tB = snap(c.tB);
                snapped.tA = nearestFrom(a, pointAt(b, snapped.tB), c.tA).t;
            }
            if ((snapped.tA != c.tA || snapped.tB != c.tB) &&
                residual(a, b, snapped) <= 1e-6 * tol.distance) {
                c = snapped;
                res = residual(a, b, c);
            }
        }
        const Vec2 p = (pointAt(a, c.tA) + pointAt(b, c.tB)) * 0.5;
        // A contact that is not an exact crossing is a touch within tolerance. An exact contact is
        // a touch when the tangents are parallel, or when they are nearly so and A stays on one
        // side of B half a tolerance either way along A: at an exact tangency Newton stops
        // anywhere in the band where the curves agree to rounding, and there the tangents can
        // differ by more than the parallel test allows.
        bool tangent = !exact || tangentsParallel(a, c.tA, b, c.tB);
        if (!tangent && std::fabs(tangentAt(a, c.tA).cross(tangentAt(b, c.tB))) <= 1e-3) {
            const double speed = derivativeAt(a, c.tA).length();
            const double step = speed > 0.0 ? 0.5 * tol.distance / speed : 0.0;
            const auto side = [&](double t) { return sideSample(a, b, std::clamp(t, 0.0, 1.0), c.tB).f; };
            const double before = side(c.tA - step);
            const double after = side(c.tA + step);
            const double noise = 1e-6 * tol.distance;
            tangent = step > 0.0 && std::fabs(before) > noise && std::fabs(after) > noise &&
                      (before > 0.0) == (after > 0.0);
        }
        raw.push_back({res, {p, c.tA, c.tB, tangent}, end});
    };
    for (const Candidate& c : found) {
        const double res = residual(a, b, c);
        if (res > 1.5 * tol.distance + c.reach) {
            continue;
        }
        const bool exact = res <= 1e-6 * tol.distance;
        if (!exact) {
            // Not a crossing, so it must be a touch: a local minimum of the distance between the
            // curves. The candidate may sit anywhere in the band where the curves are within
            // tolerance (wide for a shallow touch), or at a cut between pieces while the curves
            // keep getting closer beyond it. Settle it to the nearest local minimum of the gap
            // along the unsplit A; a candidate whose descent ends at a crossing becomes that
            // crossing (its exact twin, found from the other side, wins the merge).
            for (const Candidate& settled : settleTouch(a, b, c, trail, minima)) {
                const double r = residual(a, b, settled);
                if (r <= tol.distance) {
                    touches.push_back({r, settled});
                }
            }
            continue;
        }
        emit(c, res, true, false);
    }
    // A settled touch may itself be an exact crossing (a descent that ran into one).
    for (const auto& [res, c] : touches) {
        emit(c, res, res <= 1e-6 * tol.distance, false);
    }
    for (const Candidate& c : endHits) {
        const double res = residual(a, b, c);
        emit(c, res, res <= exactGap(tol), true);
    }
    std::stable_sort(raw.begin(), raw.end(), [](const Raw& l, const Raw& r) {
        return std::tie(l.res, l.hit.tA, l.hit.tB) < std::tie(r.res, r.hit.tA, r.hit.tB);
    });
    IntersectionResult out;
    std::vector<double> keptRes;
    std::vector<bool> keptEnd;
    for (const Raw& r : raw) {
        const Intersection& hit = r.hit;
        const auto it = std::find_if(out.points.begin(), out.points.end(), [&](const Intersection& e) {
            return tol.equalPoints(e.point, hit.point);
        });
        if (it == out.points.end()) {
            out.points.push_back(hit);
            keptRes.push_back(r.res);
            keptEnd.push_back(r.end);
            continue;
        }
        const auto k = static_cast<std::size_t>(it - out.points.begin());
        if (distance(it->point, hit.point) > 1e-3 * tol.distance && r.res <= 1e-6 * tol.distance &&
            keptRes[k] <= 1e-6 * tol.distance && !r.end && !keptEnd[k]) {
            // Two distinct exact crossings closer than the tolerance: a touch, not a crossing. (A
            // touch absorbed by a nearby crossing leaves the crossing as it is.)
            it->tangent = true;
        }
    }
    finish(out, 0.0);
    return out;
}

// Canonical argument order for the polygonise-and-refine path: by span kind, then (two Beziers)
// by control points. Solving every pair in one order makes the result exactly symmetric.
bool canonicalOrder(const Span& a, const Span& b) {
    if (a.index() != b.index()) {
        return a.index() < b.index();
    }
    const auto* ba = std::get_if<BezierSpan>(&a);
    const auto* bb = std::get_if<BezierSpan>(&b);
    if (ba == nullptr || bb == nullptr) {
        return true;
    }
    const auto key = [](const BezierSpan& z) {
        return std::array<double, 8>{z.p0.x, z.p0.y, z.p1.x, z.p1.y, z.p2.x, z.p2.y, z.p3.x, z.p3.y};
    };
    return !(key(*bb) < key(*ba));
}

IntersectionResult swapped(IntersectionResult r) {
    for (Intersection& p : r.points) {
        std::swap(p.tA, p.tB);
    }
    for (Overlap& o : r.overlaps) {
        Overlap s{o.tB0, o.tB1, o.tA0, o.tA1};
        if (s.tA0 > s.tA1) {
            std::swap(s.tA0, s.tA1);
            std::swap(s.tB0, s.tB1);
        }
        o = s;
    }
    finish(r, 0.0);
    return r;
}

// ------------------------------------------------------------- contour machinery

void appendContourHits(ContourIntersections& out, std::size_t i, std::size_t j, const IntersectionResult& r,
                       const Tolerance& tol) {
    for (const Intersection& hit : r.points) {
        const bool dup = std::any_of(out.points.begin(), out.points.end(), [&](const ContourIntersection& e) {
            return tol.equalPoints(e.hit.point, hit.point);
        });
        if (!dup) {
            out.points.push_back({i, j, hit});
        }
    }
    for (const Overlap& o : r.overlaps) {
        out.overlaps.push_back({i, j, o});
    }
}

// The double point of a cubic, from its power-basis form B(t) = a t^3 + b t^2 + c t + d: for
// s != t, (B(t) - B(s)) / (t - s) = a (t^2 + ts + s^2) + b (t + s) + c = 0. In terms of the sum
// u = s + t and the product v = s t this is a (u^2 - v) + b u + c = 0; crossing with a gives
// u (a x b) + (a x c) = 0, and projecting on a gives v. s and t are the roots of
// x^2 - u x + v = 0. A cubic has at most one double point; it is a loop of this span when both
// roots are real, distinct and in [0, 1]. Returned unrefined (s < t).
std::optional<Candidate> cubicDoublePoint(const BezierSpan& z) {
    const Vec2 a = z.p3 - z.p0 + (z.p1 - z.p2) * 3.0;
    const Vec2 b = (z.p0 - z.p1 * 2.0 + z.p2) * 3.0;
    const Vec2 c = (z.p1 - z.p0) * 3.0;
    const double ab = a.cross(b);
    const double aa = a.dot(a);
    if (ab == 0.0 || aa == 0.0) {
        return std::nullopt;
    }
    const double u = -a.cross(c) / ab;
    const double v = u * u + (b * u + c).dot(a) / aa;
    const double disc = u * u - 4.0 * v;
    if (!(disc > 0.0) || !std::isfinite(disc)) {
        return std::nullopt;
    }
    const double root = std::sqrt(disc);
    const double t0 = 0.5 * (u - root);
    const double t1 = 0.5 * (u + root);
    constexpr double kSlack = 1e-9;
    if (t0 < -kSlack || t1 > 1.0 + kSlack) {
        return std::nullopt;
    }
    return Candidate{std::clamp(t0, 0.0, 1.0), std::clamp(t1, 0.0, 1.0)};
}

// Self-intersections of a single cubic Bezier (a loop crossing itself). `closingJunction` is set
// when the Bezier alone forms a closed contour: its start meeting its end there is not a crossing.
// The double point is computed analytically (`cubicDoublePoint`), so loops of any size are found,
// including thin slivers whose chords never cross at the chordal sampling; a loop that stays
// within `tol.distance` of its double point is a cusp within tolerance and is not reported. Chord crossings
// are also refined, as a fallback when rounding puts the analytic roots just outside [0, 1].
void bezierSelfIntersections(const BezierSpan& bz, std::size_t index, std::optional<Vec2> closingJunction,
                             ContourIntersections& out, const Tolerance& tol) {
    const Span s{bz};
    if (const std::optional<Candidate> dp = cubicDoublePoint(bz)) {
        Candidate c = newton(s, s, *dp);
        if (c.tA > c.tB) {
            std::swap(c.tA, c.tB);
        }
        const Vec2 p = pointAt(s, c.tA);
        // The loop's size: its largest distance from the double point, from 16 samples refined by
        // golden section around the farthest (the distance is smooth and, for a loop, has one
        // maximum, but the sampling keeps a lopsided loop from fooling the search).
        const auto reachOf = [&](double u) { return distance(p, pointAt(s, u)); };
        constexpr int kLoopSamples = 16;
        const double du = (c.tB - c.tA) / kLoopSamples;
        int far = 1;
        for (int k = 2; k < kLoopSamples; ++k) {
            if (reachOf(c.tA + du * k) > reachOf(c.tA + du * far)) {
                far = k;
            }
        }
        double lo = c.tA + du * (far - 1);
        double hi = c.tA + du * (far + 1);
        for (int iter = 0; iter < 40; ++iter) {
            const double m1 = hi - 0.6180339887498949 * (hi - lo);
            const double m2 = lo + 0.6180339887498949 * (hi - lo);
            if (reachOf(m1) > reachOf(m2)) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        const double extent = std::max(reachOf(0.5 * (lo + hi)), reachOf(c.tA + du * far));
        if (std::fabs(c.tA - c.tB) > 1e-6 && residual(s, s, c) <= tol.distance && extent > tol.distance &&
            !(closingJunction && tol.equalPoints(p, *closingJunction))) {
            IntersectionResult r;
            r.points.push_back({p, c.tA, c.tB, false});
            appendContourHits(out, index, index, r, tol);
        }
    }
    const std::vector<ParamPoint> pts = sampleSpan(s, tol);
    for (std::size_t i = 1; i < pts.size(); ++i) {
        for (std::size_t j = i + 2; j < pts.size(); ++j) {
            const auto prox = segmentProximity(pts[i - 1].p, pts[i].p, pts[j - 1].p, pts[j].p, 0.0);
            if (!prox) {
                continue;
            }
            const Candidate seed{pts[i - 1].t + (pts[i].t - pts[i - 1].t) * prox->first.tA,
                                 pts[j - 1].t + (pts[j].t - pts[j - 1].t) * prox->first.tB};
            const Candidate c = newton(s, s, seed);
            if (std::fabs(c.tA - c.tB) > 1e-6 && residual(s, s, c) <= tol.distance) {
                const Vec2 p = pointAt(s, c.tA);
                if (closingJunction && tol.equalPoints(p, *closingJunction)) {
                    continue;
                }
                IntersectionResult r;
                r.points.push_back({p, c.tA, c.tB, false});
                appendContourHits(out, index, index, r, tol);
            }
        }
    }
}

ContourIntersections selfIntersectionsImpl(const Contour& c, const Tolerance& tol, bool stopAtFirst) {
    ContourIntersections out;
    const std::size_t n = c.spans.size();
    std::vector<Box2> boxes(n);
    for (std::size_t i = 0; i < n; ++i) {
        boxes[i] = bounds(c.spans[i]).inflated(tol.distance);
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (const auto* bz = std::get_if<BezierSpan>(&c.spans[i])) {
            // A closed contour of one span joins that span's end to its own start.
            const std::optional<Vec2> closing =
                c.closed && n == 1 ? std::optional<Vec2>{bz->p0} : std::nullopt;
            bezierSelfIntersections(*bz, i, closing, out, tol);
            if (stopAtFirst && !out.isEmpty()) {
                return out;
            }
        }
        for (std::size_t j = i + 1; j < n; ++j) {
            if (!boxes[i].intersects(boxes[j])) {
                continue;
            }
            IntersectionResult r = intersect(c.spans[i], c.spans[j], tol);
            // Consecutive spans legitimately meet at their shared junction. In a closed contour of
            // exactly two spans the pair (0, 1) shares two junctions: the inner one (end of span 0)
            // and the closing one (start of span 0), and both are ignored.
            std::array<std::optional<Vec2>, 2> junctions{};
            if (j == i + 1) {
                junctions[0] = endPoint(c.spans[i]);
            }
            if (c.closed && i == 0 && j == n - 1) {
                junctions[1] = startPoint(c.spans[0]);
            }
            std::erase_if(r.points, [&](const Intersection& p) {
                return std::any_of(junctions.begin(), junctions.end(), [&](const std::optional<Vec2>& q) {
                    return q && tol.equalPoints(p.point, *q);
                });
            });
            appendContourHits(out, i, j, r, tol);
            if (stopAtFirst && !out.isEmpty()) {
                return out;
            }
        }
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------------- public API

IntersectionResult intersect(const LineSpan& a, const LineSpan& b, const Tolerance& tol) {
    return intersect(Span{a}, Span{b}, tol);
}

IntersectionResult intersect(const LineSpan& a, const ArcSpan& b, const Tolerance& tol) {
    return intersect(Span{a}, Span{b}, tol);
}

IntersectionResult intersect(const ArcSpan& a, const ArcSpan& b, const Tolerance& tol) {
    return intersect(Span{a}, Span{b}, tol);
}

IntersectionResult intersect(const Span& a, const Span& b, const Tolerance& tol) {
    const bool degA = isZeroLength(a, tol);
    const bool degB = isZeroLength(b, tol);
    if (degA && degB) {
        IntersectionResult out;
        if (tol.equalPoints(startPoint(a), startPoint(b))) {
            out.points.push_back({startPoint(a), 0.0, 0.0, false});
        }
        return out;
    }
    if (degA) {
        return pointAgainstSpan(startPoint(a), b, true, tol);
    }
    if (degB) {
        return pointAgainstSpan(startPoint(b), a, false, tol);
    }

    if (!std::holds_alternative<BezierSpan>(a) && !std::holds_alternative<BezierSpan>(b)) {
        return analyticPair(a, b, tol);
    }
    if (!canonicalOrder(a, b)) {
        return swapped(polygoniseAndRefine(b, a, tol));
    }
    return polygoniseAndRefine(a, b, tol);
}

ContourIntersections intersect(const Contour& a, const Contour& b, const Tolerance& tol) {
    ContourIntersections out;
    std::vector<Box2> boxesB(b.spans.size());
    for (std::size_t j = 0; j < b.spans.size(); ++j) {
        boxesB[j] = bounds(b.spans[j]).inflated(tol.distance);
    }
    for (std::size_t i = 0; i < a.spans.size(); ++i) {
        const Box2 boxA = bounds(a.spans[i]).inflated(tol.distance);
        for (std::size_t j = 0; j < b.spans.size(); ++j) {
            if (!boxA.intersects(boxesB[j])) {
                continue;
            }
            appendContourHits(out, i, j, intersect(a.spans[i], b.spans[j], tol), tol);
        }
    }
    return out;
}

ContourIntersections selfIntersections(const Contour& c, const Tolerance& tol) {
    return selfIntersectionsImpl(c, tol, false);
}

bool hasSelfIntersections(const Contour& c, const Tolerance& tol) {
    return !selfIntersectionsImpl(c, tol, true).isEmpty();
}

}  // namespace carvetto::geometry
