#include "carvetto/geometry/intersect.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

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
// that lands just outside either segment is recovered by testing the four end points against the
// other segment, which keeps end-to-end contact within `tol.distance` independent of the angle.
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
            return out;
        }
    }

    // No crossing within the segments: an end point lying within tolerance of the other segment
    // is still a contact. Keep the closest such pair (they all describe the same neighbourhood,
    // since the segments are not collinear).
    struct EndHit {
        Intersection hit;
        double dist;
    };
    std::optional<EndHit> best;
    const auto consider = [&](Vec2 endPt, const LineSpan& other, bool endOnA, double endT) {
        const double tOther = projectParam(other, endPt);
        const Vec2 q = other.pointAt(tOther);
        const double dist = distance(endPt, q);
        if (dist > tol.distance || (best && dist >= best->dist)) {
            return;
        }
        const Vec2 mid = (endPt + q) * 0.5;
        best = EndHit{
            endOnA ? Intersection{mid, endT, tOther, false} : Intersection{mid, tOther, endT, false}, dist};
    };
    // Visit end points in a canonical order so ties resolve identically whichever span is A.
    const bool aFirst = !segmentLess(b, a);
    for (int pass = 0; pass < 2; ++pass) {
        const bool useA = (pass == 0) == aFirst;
        const LineSpan& seg = useA ? a : b;
        const LineSpan& other = useA ? b : a;
        consider(seg.start, other, useA, 0.0);
        consider(seg.end, other, useA, 1.0);
    }
    if (best) {
        out.points.push_back(best->hit);
    }
    return out;
}

// Line-arc intersection from the foot of the perpendicular F from the centre onto the line:
// hits lie at F +- dir * sqrt(r^2 - h^2). Unlike the expanded quadratic this has no
// cancellation when the line is much longer than the circle is wide.
IntersectionResult lineArc(const LineSpan& a, const ArcSpan& b, const Tolerance& tol) {
    IntersectionResult out;
    const Vec2 d = a.end - a.start;
    const double la = d.length();
    const Vec2 dir = d / la;
    const double slackA = paramSlack(la, tol);
    const double angTol = angleSlack(b.radius, tol);
    const Vec2 f = a.start - b.centre;
    const double footAlong = -f.dot(dir);   // distance from a.start to the foot along the line
    const Vec2 foot = f + dir * footAlong;  // foot relative to the centre
    const double h = foot.length();

    const auto consider = [&](double along, bool tangent) {
        const double t = along / la;
        if (t < -slackA || t > 1.0 + slackA) {
            return;
        }
        const double tc = std::clamp(t, 0.0, 1.0);
        const Vec2 p = tc == t ? b.centre + foot + dir * (along - footAlong) : a.pointAt(tc);
        const double angle = (p - b.centre).angle();
        if (!b.containsAngle(angle, angTol)) {
            return;
        }
        addPoint(out, {p, tc, arcParam(b, angle, angTol), tangent}, tol);
    };

    if (std::fabs(h - b.radius) <= tol.distance) {
        consider(footAlong, true);
        return out;
    }
    if (h > b.radius) {
        return out;
    }
    // (r - h)(r + h) avoids squaring large numbers.
    const double half = std::sqrt(std::max(0.0, (b.radius - h) * (b.radius + h)));
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

IntersectionResult arcArc(const ArcSpan& a, const ArcSpan& b, const Tolerance& tol) {
    IntersectionResult out;
    const Vec2 cc = b.centre - a.centre;
    const double d = cc.length();
    const double r1 = a.radius;
    const double r2 = b.radius;

    if (d <= tol.distance) {
        if (std::fabs(r1 - r2) <= tol.distance) {
            coincidentArcs(a, b, out, tol);
            finish(out, angleSlack(r1, tol) / std::fabs(a.sweepAngle()));
        }
        return out;  // concentric circles of different radii never meet
    }
    if (d > r1 + r2 + tol.distance || d < std::fabs(r1 - r2) - tol.distance) {
        return out;
    }
    const bool external = std::fabs(d - (r1 + r2)) <= tol.distance;
    const bool internal = !external && std::fabs(d - std::fabs(r1 - r2)) <= tol.distance;
    const bool tangent = external || internal;
    const Vec2 u = cc / d;
    const double angTolA = angleSlack(r1, tol);
    const double angTolB = angleSlack(r2, tol);

    const auto consider = [&](Vec2 p) {
        const double angA = (p - a.centre).angle();
        const double angB = (p - b.centre).angle();
        if (!a.containsAngle(angA, angTolA) || !b.containsAngle(angB, angTolB)) {
            return;
        }
        addPoint(out, {p, arcParam(a, angA, angTolA), arcParam(b, angB, angTolB), tangent}, tol);
    };

    if (tangent) {
        // The circles touch (or nearly touch) on the centre line. Take each circle's point on that
        // line facing the other and report their midpoint: both lie exactly on their circles and
        // are at most `tol.distance` apart, so the midpoint is within tolerance of both. (The
        // two-circle formula for `along` divides the radius gap by d and is useless here when the
        // centres are close: its error grows like r * gap / d.)
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
        consider((onA + onB) * 0.5);
        finish(out, 0.0);
        return out;
    }

    // Distance from A's centre to the radical line, and half the chord. The factored forms avoid
    // cancellation between nearly equal squares.
    const double along = ((r1 - r2) * (r1 + r2) + d * d) / (2.0 * d);
    const double h = std::sqrt(std::max(0.0, (r1 - along) * (r1 + along)));
    const Vec2 base = a.centre + u * along;
    consider(base + u.perp() * h);
    consider(base - u.perp() * h);
    finish(out, 0.0);
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

// Golden-section minimisation of dist(A(t), B) over t in [lo, hi]: handles tangential contacts,
// where Newton has nothing to bite on, by never needing the Jacobian at all.
Candidate goldenSection(const Span& a, const Span& b, double lo, double hi) {
    constexpr double invPhi = 0.6180339887498949;
    const auto f = [&](double t) { return closestPoint(b, pointAt(a, t)).distance; };
    double x1 = hi - invPhi * (hi - lo);
    double x2 = lo + invPhi * (hi - lo);
    double f1 = f(x1);
    double f2 = f(x2);
    for (int iter = 0; iter < 80 && hi - lo > 1e-15; ++iter) {
        if (f1 < f2) {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - invPhi * (hi - lo);
            f1 = f(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + invPhi * (hi - lo);
            f2 = f(x2);
        }
    }
    const double t = f1 < f2 ? x1 : x2;
    return {t, closestPoint(b, pointAt(a, t)).t};
}

// Refines a seed from the polygon stage. `lo`/`hi` bound A's parameter around the chord that
// produced the seed, so the fallback search stays within that neighbourhood.
Candidate refine(const Span& a, const Span& b, Candidate seed, double lo, double hi, const Tolerance& tol) {
    const Candidate c = newton(a, b, seed);
    if (residual(a, b, c) <= tol.distance) {
        return c;
    }
    const Candidate g = goldenSection(a, b, lo, hi);
    return residual(a, b, g) < residual(a, b, c) ? g : c;
}

IntersectionResult polygoniseAndRefine(const Span& a, const Span& b, const Tolerance& tol) {
    IntersectionResult out;
    const std::vector<ParamPoint> pa = sampleSpan(a, tol);
    const std::vector<ParamPoint> pb = sampleSpan(b, tol);
    // Chords deviate from the curves by up to tol.chordal each, so near misses up to twice that
    // (plus the acceptance tolerance) may still be genuine contacts.
    const double reach = 2.0 * tol.chordal + tol.distance;
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
            const double wA = pa[i].t - pa[i - 1].t;
            const Candidate seed{pa[i - 1].t + wA * prox->first.tA,
                                 pb[j - 1].t + (pb[j].t - pb[j - 1].t) * prox->first.tB};
            const Candidate c =
                refine(a, b, seed, std::max(0.0, pa[i - 1].t - wA), std::min(1.0, pa[i].t + wA), tol);
            if (residual(a, b, c) <= tol.distance) {
                const Vec2 p = (pointAt(a, c.tA) + pointAt(b, c.tB)) * 0.5;
                addPoint(out, {p, c.tA, c.tB, tangentsParallel(a, c.tA, b, c.tB)}, tol);
            }
        }
    }
    finish(out, 0.0);
    return out;
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

// Self-intersections of a single cubic Bezier (a loop crossing itself).
void bezierSelfIntersections(const BezierSpan& bz, std::size_t index, ContourIntersections& out,
                             const Tolerance& tol) {
    const Span s{bz};
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
                IntersectionResult r;
                r.points.push_back({pointAt(s, c.tA), c.tA, c.tB, false});
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
            bezierSelfIntersections(*bz, i, out, tol);
            if (stopAtFirst && !out.isEmpty()) {
                return out;
            }
        }
        for (std::size_t j = i + 1; j < n; ++j) {
            if (!boxes[i].intersects(boxes[j])) {
                continue;
            }
            IntersectionResult r = intersect(c.spans[i], c.spans[j], tol);
            // Consecutive spans legitimately meet at their shared junction.
            std::optional<Vec2> junction;
            if (j == i + 1) {
                junction = endPoint(c.spans[i]);
            } else if (c.closed && i == 0 && j == n - 1) {
                junction = startPoint(c.spans[0]);
            }
            if (junction) {
                std::erase_if(r.points,
                              [&](const Intersection& p) { return tol.equalPoints(p.point, *junction); });
            }
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

    if (const auto* la = std::get_if<LineSpan>(&a)) {
        if (const auto* lb = std::get_if<LineSpan>(&b)) {
            IntersectionResult r = lineLine(*la, *lb, tol);
            finish(r, paramSlack(la->length(), tol));
            return r;
        }
        if (const auto* ab = std::get_if<ArcSpan>(&b)) {
            IntersectionResult r = lineArc(*la, *ab, tol);
            finish(r, 0.0);
            return r;
        }
    } else if (const auto* aa = std::get_if<ArcSpan>(&a)) {
        if (const auto* lb = std::get_if<LineSpan>(&b)) {
            return swapped(lineArc(*lb, *aa, tol));
        }
        if (const auto* ab = std::get_if<ArcSpan>(&b)) {
            return arcArc(*aa, *ab, tol);
        }
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
