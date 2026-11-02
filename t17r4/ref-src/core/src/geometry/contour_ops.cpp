#include "carvetto/geometry/contour_ops.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

namespace carvetto::geometry {

// ------------------------------------------------------------- area / orientation

namespace {

// Contribution of one span to the Green's theorem integral 1/2 * integral(x dy - y dx).
double spanAreaContribution(const Span& s) {
    return std::visit(
        [](const auto& v) -> double {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return 0.5 * v.start.cross(v.end);
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                // x = cx + r cos(a), y = cy + r sin(a):  x y' - y x' = r^2 + r (cx cos a + cy sin a).
                const double sweep = v.sweepAngle();
                const double a0 = v.startAngle;
                const double a1 = a0 + sweep;
                const double r = v.radius;
                const double integral = r * r * sweep + r * (v.centre.x * (std::sin(a1) - std::sin(a0)) -
                                                             v.centre.y * (std::cos(a1) - std::cos(a0)));
                return 0.5 * integral;
            } else {
                // x(t) y'(t) - y(t) x'(t) is a degree-5 polynomial: 3-point Gauss-Legendre is exact.
                constexpr double h = 0.3872983346207417;  // sqrt(15)/10
                constexpr std::array<double, 3> nodes{0.5 - h, 0.5, 0.5 + h};
                constexpr std::array<double, 3> weights{5.0 / 18.0, 8.0 / 18.0, 5.0 / 18.0};
                double sum = 0.0;
                for (std::size_t i = 0; i < 3; ++i) {
                    const Vec2 p = v.pointAt(nodes[i]);
                    const Vec2 d = v.derivativeAt(nodes[i]);
                    sum += weights[i] * p.cross(d);
                }
                return 0.5 * sum;
            }
        },
        s);
}

}  // namespace

double signedArea(const Contour& c) {
    if (!c.closed || c.spans.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (const Span& s : c.spans) {
        total += spanAreaContribution(s);
    }
    // A contour whose last span does not land exactly on the start is closed by a straight gap.
    const Vec2 e = c.endPoint();
    const Vec2 s0 = c.startPoint();
    total += 0.5 * e.cross(s0);
    return total;
}

double area(const Contour& c) {
    return std::fabs(signedArea(c));
}

Orientation orientation(const Contour& c, const Tolerance& tol) {
    if (!c.closed) {
        return Orientation::Degenerate;
    }
    const double a = signedArea(c);
    if (std::fabs(a) <= tol.distance * tol.distance) {
        return Orientation::Degenerate;
    }
    return a > 0.0 ? Orientation::CCW : Orientation::CW;
}

void setOrientation(Contour& c, Orientation want, const Tolerance& tol) {
    if (want == Orientation::Degenerate) {
        return;
    }
    const Orientation have = orientation(c, tol);
    if (have != Orientation::Degenerate && have != want) {
        c.reverse();
    }
}

// ----------------------------------------------------------------- point in contour

namespace {

struct RayCaster {
    Vec2 p;
    int winding{0};

    bool above(Vec2 q) const { return q.y >= p.y; }

    // A y-monotone piece from a to b crossing the horizontal through p at x.
    void countCrossing(double x, bool upward) {
        if (x > p.x) {
            winding += upward ? 1 : -1;
        }
    }

    void addSegment(Vec2 a, Vec2 b) {
        const bool aAbove = above(a);
        const bool bAbove = above(b);
        if (aAbove == bAbove) {
            return;
        }
        const double x = a.x + (b.x - a.x) * (p.y - a.y) / (b.y - a.y);
        countCrossing(x, bAbove);
    }

    void addArc(const ArcSpan& arc) {
        // Split at the angles where the tangent is horizontal (cos = 0), so that each piece is
        // y-monotone and lies entirely in one half (x >= cx or x <= cx) of the circle.
        std::vector<double> params{0.0};
        const double sweep = arc.sweepAngle();
        for (const double a : {kPi * 0.5, kPi * 1.5}) {
            const double delta = arc.direction == ArcDirection::CCW ? normalizeAngle(a - arc.startAngle)
                                                                    : normalizeAngle(arc.startAngle - a);
            const double t = delta / std::fabs(sweep);
            if (t > 0.0 && t < 1.0) {
                params.push_back(t);
            }
        }
        params.push_back(1.0);
        std::sort(params.begin(), params.end());
        const double dy = p.y - arc.centre.y;
        const double half = std::sqrt(std::max(0.0, arc.radius * arc.radius - dy * dy));
        for (std::size_t i = 1; i < params.size(); ++i) {
            const Vec2 a = arc.pointAt(params[i - 1]);
            const Vec2 b = arc.pointAt(params[i]);
            const bool aAbove = above(a);
            const bool bAbove = above(b);
            if (aAbove == bAbove) {
                continue;
            }
            const double mid = arc.angleAt(0.5 * (params[i - 1] + params[i]));
            const double x = arc.centre.x + (std::cos(mid) >= 0.0 ? half : -half);
            countCrossing(x, bAbove);
        }
    }

    void addSpan(const Span& s, const Tolerance& tol) {
        std::visit(
            [&](const auto& v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, LineSpan>) {
                    addSegment(v.start, v.end);
                } else if constexpr (std::is_same_v<T, ArcSpan>) {
                    addArc(v);
                } else {
                    std::vector<Vec2> pts{v.p0};
                    polygoniseInto(Span{v}, tol, pts);
                    for (std::size_t i = 1; i < pts.size(); ++i) {
                        addSegment(pts[i - 1], pts[i]);
                    }
                }
            },
            s);
    }
};

}  // namespace

int windingNumber(const Contour& c, Vec2 p, const Tolerance& tol) {
    if (c.spans.empty()) {
        return 0;
    }
    RayCaster caster{p};
    for (const Span& s : c.spans) {
        caster.addSpan(s, tol);
    }
    // Close the loop (explicitly when open, and across any small gap when closed).
    caster.addSegment(c.endPoint(), c.startPoint());
    return caster.winding;
}

namespace {

PointLocation classify(int winding, bool onBoundary, FillRule rule) {
    if (onBoundary) {
        return PointLocation::OnBoundary;
    }
    const bool inside = rule == FillRule::EvenOdd ? (winding % 2) != 0 : winding != 0;
    return inside ? PointLocation::Inside : PointLocation::Outside;
}

}  // namespace

PointLocation locatePoint(const Contour& c, Vec2 p, FillRule rule, const Tolerance& tol) {
    const bool onBoundary = distanceTo(c, p) <= tol.distance;
    return classify(windingNumber(c, p, tol), onBoundary, rule);
}

PointLocation locatePoint(const std::vector<Contour>& contours, Vec2 p, FillRule rule, const Tolerance& tol) {
    int winding = 0;
    bool onBoundary = false;
    for (const Contour& c : contours) {
        if (distanceTo(c, p) <= tol.distance) {
            onBoundary = true;
        }
        winding += windingNumber(c, p, tol);
    }
    return classify(winding, onBoundary, rule);
}

bool isInside(const Contour& c, Vec2 p, FillRule rule, const Tolerance& tol) {
    return locatePoint(c, p, rule, tol) != PointLocation::Outside;
}

bool isInside(const std::vector<Contour>& contours, Vec2 p, FillRule rule, const Tolerance& tol) {
    return locatePoint(contours, p, rule, tol) != PointLocation::Outside;
}

// -------------------------------------------------------------------- closest point

ClosestPoint closestPoint(const LineSpan& s, Vec2 p) {
    const Vec2 d = s.end - s.start;
    const double len2 = d.lengthSquared();
    double t = 0.0;
    if (len2 > 0.0) {
        t = std::clamp((p - s.start).dot(d) / len2, 0.0, 1.0);
    }
    const Vec2 q = s.pointAt(t);
    return {q, t, distance(p, q)};
}

ClosestPoint closestPoint(const ArcSpan& s, Vec2 p) {
    const Vec2 radial = p - s.centre;
    if (radial.lengthSquared() > 0.0) {
        const double angle = radial.angle();
        // No angular slack (T-0022): a fixed slack in angle is a real distance on a large arc
        // (1e-9 rad is 1e-3 mm at r = 1e6), and a direction inside it but beyond an end was
        // projected radially onto the circle past the arc's end. When the direction lies outside
        // the sweep the nearer end point is the answer; where that is only a rounding error of the
        // direction, the end point and the radial projection are r times that rounding apart, so
        // the distance is right to rounding either way. A full circle (as `isFullCircle` defines
        // it) has no ends.
        if (s.isFullCircle() || s.containsAngle(angle, 0.0)) {
            const double sweep = std::fabs(s.sweepAngle());
            const double delta = s.direction == ArcDirection::CCW ? normalizeAngle(angle - s.startAngle)
                                                                  : normalizeAngle(s.startAngle - angle);
            // On a circle that `isFullCircle` accepts although its sweep falls a rounding margin
            // short of 2pi, the travelled angle can lie in that gap: just past the end it is slightly
            // above the sweep, just before the start it wraps to just under 2pi. Either way the
            // parameter is that of the nearer end (in angle).
            const double t = delta <= sweep ? delta / sweep : (delta - sweep <= kTwoPi - delta ? 1.0 : 0.0);
            const Vec2 q = s.centre + Vec2::fromAngle(angle, s.radius);
            return {q, t, distance(p, q)};
        }
    }
    const Vec2 a = s.startPoint();
    const Vec2 b = s.endPoint();
    const double da = distance(p, a);
    const double db = distance(p, b);
    return da <= db ? ClosestPoint{a, 0.0, da} : ClosestPoint{b, 1.0, db};
}

namespace {

// g(t) = (B(t) - q) . B'(t) is a degree-5 polynomial whose roots in [0,1] are the interior
// stationary points of the squared distance. These helpers isolate every root by recursive
// de Casteljau subdivision of its Bernstein coefficients: by the variation-diminishing property
// a coefficient sequence with no sign change has no root in the open interval, and one with a
// single sign change has exactly one, which is then bracketed and bisected.
using Quintic = std::array<double, 6>;

int signChanges(const Quintic& c) {
    int changes = 0;
    int last = 0;
    for (const double v : c) {
        const int sign = v > 0.0 ? 1 : (v < 0.0 ? -1 : 0);
        if (sign != 0) {
            if (last != 0 && sign != last) {
                ++changes;
            }
            last = sign;
        }
    }
    return changes;
}

std::pair<Quintic, Quintic> splitHalf(const Quintic& c) {
    Quintic work = c;
    Quintic left{};
    Quintic right{};
    for (std::size_t level = 0; level < 6; ++level) {
        left[level] = work[0];
        right[5 - level] = work[5 - level];
        for (std::size_t i = 0; i + level < 5; ++i) {
            work[i] = 0.5 * (work[i] + work[i + 1]);
        }
    }
    return {left, right};
}

double stationaryFunction(const BezierSpan& s, Vec2 q, double t) {
    return (s.pointAt(t) - q).dot(s.derivativeAt(t));
}

void isolateRoots(const BezierSpan& s, Vec2 q, const Quintic& c, double t0, double t1, int depth,
                  std::vector<double>& roots) {
    const int changes = signChanges(c);
    if (changes == 0) {
        return;
    }
    if (changes == 1) {
        double lo = t0;
        double hi = t1;
        double glo = stationaryFunction(s, q, lo);
        const double ghi = stationaryFunction(s, q, hi);
        // Bisect only on a strict bracket. An end value that is zero or has the wrong sign (a root
        // on the boundary, or rounding) is handled by subdividing further below.
        if ((glo < 0.0 && ghi > 0.0) || (glo > 0.0 && ghi < 0.0)) {
            for (int iter = 0; iter < 200; ++iter) {
                const double mid = 0.5 * (lo + hi);
                if (mid <= lo || mid >= hi) {
                    break;
                }
                const double gm = stationaryFunction(s, q, mid);
                if (gm == 0.0) {
                    lo = mid;
                    hi = mid;
                    break;
                }
                if ((gm < 0.0) == (glo < 0.0)) {
                    lo = mid;
                    glo = gm;
                } else {
                    hi = mid;
                }
            }
            roots.push_back(0.5 * (lo + hi));
            return;
        }
    }
    if (depth >= 60 || t1 - t0 < 1e-15) {
        roots.push_back(0.5 * (t0 + t1));
        return;
    }
    const auto [left, right] = splitHalf(c);
    const double tm = 0.5 * (t0 + t1);
    isolateRoots(s, q, left, t0, tm, depth + 1, roots);
    // A root sitting exactly on the split point has a rounding-noise boundary coefficient in both
    // halves and may be claimed by neither, so the split point itself is always a candidate.
    roots.push_back(tm);
    isolateRoots(s, q, right, tm, t1, depth + 1, roots);
}

}  // namespace

ClosestPoint closestPoint(const BezierSpan& s, Vec2 p) {
    // Global minimum of |B(t) - p| over [0,1]: it is attained at an end point or at a root of
    // g(t) = (B(t) - p) . B'(t). All roots of that quintic are isolated (see isolateRoots), so
    // every local minimum is a candidate and the best one is global, unlike a seeded Newton search.
    const std::array<Vec2, 4> rel{s.p0 - p, s.p1 - p, s.p2 - p, s.p3 - p};
    const std::array<Vec2, 3> der{(s.p1 - s.p0) * 3.0, (s.p2 - s.p1) * 3.0, (s.p3 - s.p2) * 3.0};
    constexpr std::array<double, 4> binom3{1.0, 3.0, 3.0, 1.0};
    constexpr std::array<double, 3> binom2{1.0, 2.0, 1.0};
    constexpr std::array<double, 6> binom5{1.0, 5.0, 10.0, 10.0, 5.0, 1.0};
    Quintic coeffs{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            coeffs[i + j] += binom3[i] * binom2[j] * rel[i].dot(der[j]);
        }
    }
    for (std::size_t k = 0; k < 6; ++k) {
        coeffs[k] /= binom5[k];
    }

    std::vector<double> candidates{0.0};
    isolateRoots(s, p, coeffs, 0.0, 1.0, 0, candidates);
    candidates.push_back(1.0);

    ClosestPoint best{s.p0, 0.0, distance(p, s.p0)};
    for (const double t : candidates) {
        const Vec2 q = t == 0.0 ? s.p0 : (t == 1.0 ? s.p3 : s.pointAt(t));
        const double d = distance(p, q);
        if (d < best.distance) {
            best = {q, t, d};
        }
    }
    return best;
}

ClosestPoint closestPoint(const Span& s, Vec2 p) {
    return std::visit([p](const auto& v) { return closestPoint(v, p); }, s);
}

ContourClosestPoint closestPoint(const Contour& c, Vec2 p) {
    ContourClosestPoint best{p, 0, 0.0, std::numeric_limits<double>::infinity()};
    for (std::size_t i = 0; i < c.spans.size(); ++i) {
        const ClosestPoint cp = closestPoint(c.spans[i], p);
        if (cp.distance < best.distance) {
            best = {cp.point, i, cp.t, cp.distance};
        }
    }
    return best;
}

double distanceTo(const Contour& c, Vec2 p) {
    return closestPoint(c, p).distance;
}

}  // namespace carvetto::geometry
