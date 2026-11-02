#include "carvetto/geometry/span.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::geometry {

// ---------------------------------------------------------------- LineSpan

Vec2 LineSpan::tangentAt(double /*t*/) const {
    return (end - start).normalized();
}

// ----------------------------------------------------------------- ArcSpan

ArcSpan ArcSpan::fullCircle(Vec2 centre, double radius, ArcDirection dir) {
    return {centre, radius, 0.0, 0.0, dir};
}

ArcSpan ArcSpan::fromCentre(Vec2 centre, Vec2 start, Vec2 end, ArcDirection dir) {
    return {centre, distance(centre, start), (start - centre).angle(), (end - centre).angle(), dir};
}

double ArcSpan::sweepAngle() const {
    // Normalise the travelled angle into (0, 2*pi]; coincident angles mean a full circle.
    const double raw = direction == ArcDirection::CCW ? endAngle - startAngle : startAngle - endAngle;
    double sweep = normalizeAngle(raw);
    if (sweep <= 0.0) {
        sweep = kTwoPi;
    }
    return direction == ArcDirection::CCW ? sweep : -sweep;
}

bool ArcSpan::isFullCircle(double angleTol) const {
    return std::fabs(sweepAngle()) >= kTwoPi - angleTol;
}

Vec2 ArcSpan::tangentAt(double t) const {
    const Vec2 radial = Vec2::fromAngle(angleAt(t));
    return direction == ArcDirection::CCW ? radial.perp() : -radial.perp();
}

bool ArcSpan::containsAngle(double angle, double angleTol) const {
    const double sweep = std::fabs(sweepAngle());
    if (sweep >= kTwoPi - angleTol) {
        return true;
    }
    const double delta = direction == ArcDirection::CCW ? normalizeAngle(angle - startAngle)
                                                        : normalizeAngle(startAngle - angle);
    return delta <= sweep + angleTol || delta >= kTwoPi - angleTol;
}

Box2 ArcSpan::bounds() const {
    Box2 box{startPoint(), endPoint()};
    // Axis extremes occur at multiples of pi/2.
    for (int q = 0; q < 4; ++q) {
        const double angle = static_cast<double>(q) * (kPi * 0.5);
        if (containsAngle(angle)) {
            box.expand(centre + Vec2::fromAngle(angle, radius));
        }
    }
    return box;
}

ArcSpan ArcSpan::reversed() const {
    return {centre, radius, endAngle, startAngle,
            direction == ArcDirection::CCW ? ArcDirection::CW : ArcDirection::CCW};
}

std::vector<BezierSpan> ArcSpan::toBeziers() const {
    // Each piece covers at most 90 degrees; control distance k = 4/3 * tan(theta/4) * r.
    const double sweep = sweepAngle();
    const std::size_t pieces =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(std::fabs(sweep) / (kPi * 0.5) - 1e-12)));
    const double theta = sweep / static_cast<double>(pieces);
    const double k = (4.0 / 3.0) * std::tan(std::fabs(theta) * 0.25) * radius;
    std::vector<BezierSpan> out;
    out.reserve(pieces);
    for (std::size_t i = 0; i < pieces; ++i) {
        const double a0 = startAngle + static_cast<double>(i) * theta;
        const double a1 = a0 + theta;
        const Vec2 p0 = centre + Vec2::fromAngle(a0, radius);
        const Vec2 p3 = centre + Vec2::fromAngle(a1, radius);
        const double sign = theta >= 0.0 ? 1.0 : -1.0;
        const Vec2 t0 = Vec2::fromAngle(a0).perp() * sign;
        const Vec2 t1 = Vec2::fromAngle(a1).perp() * sign;
        out.push_back({p0, p0 + t0 * k, p3 - t1 * k, p3});
    }
    return out;
}

std::size_t ArcSpan::segmentCountFor(double chordalTol) const {
    const double sweep = std::fabs(sweepAngle());
    if (radius <= 0.0) {
        return 1;
    }
    if (chordalTol <= 0.0 || chordalTol >= radius) {
        // Degenerate tolerance: fall back to a coarse but sane subdivision.
        return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(sweep / (kPi * 0.5))));
    }
    // Chordal deviation of a chord spanning angle a is r*(1 - cos(a/2)).
    const double maxStep = 2.0 * std::acos(1.0 - chordalTol / radius);
    const auto n = static_cast<std::size_t>(std::ceil(sweep / maxStep - 1e-9));
    return std::max<std::size_t>(1, n);
}

// -------------------------------------------------------------- BezierSpan

namespace {

double bezierLengthRecursive(const BezierSpan& b, double absTol, int depth) {
    // Gravesen's estimate: L ~ (2*chord + polygon)/3, with error bounded by (polygon - chord).
    // Subdivide while the bound exceeds the budget for this piece.
    const double chord = distance(b.p0, b.p3);
    const double poly = distance(b.p0, b.p1) + distance(b.p1, b.p2) + distance(b.p2, b.p3);
    if (poly - chord <= absTol || depth >= 40) {
        return (2.0 * chord + poly) / 3.0;
    }
    const auto [l, r] = b.split(0.5);
    return bezierLengthRecursive(l, absTol * 0.5, depth + 1) +
           bezierLengthRecursive(r, absTol * 0.5, depth + 1);
}

void bezierPolygoniseRecursive(const BezierSpan& b, double chordalTol, int depth, std::vector<Vec2>& out) {
    if (b.flatness() <= chordalTol || depth >= 24) {
        out.push_back(b.p3);
        return;
    }
    const auto [l, r] = b.split(0.5);
    bezierPolygoniseRecursive(l, chordalTol, depth + 1, out);
    bezierPolygoniseRecursive(r, chordalTol, depth + 1, out);
}

// Distance from point p to the segment a-b (so control points overshooting the chord count too).
double distanceToChord(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double len2 = ab.lengthSquared();
    if (len2 <= 0.0) {
        return distance(p, a);
    }
    const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
    return distance(p, a + ab * t);
}

}  // namespace

double BezierSpan::length(double relTol) const {
    const double poly = distance(p0, p1) + distance(p1, p2) + distance(p2, p3);
    if (poly <= 0.0) {
        return 0.0;
    }
    return bezierLengthRecursive(*this, relTol * poly, 0);
}

Vec2 BezierSpan::pointAt(double t) const {
    const double u = 1.0 - t;
    const double b0 = u * u * u;
    const double b1 = 3.0 * u * u * t;
    const double b2 = 3.0 * u * t * t;
    const double b3 = t * t * t;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

Vec2 BezierSpan::derivativeAt(double t) const {
    const double u = 1.0 - t;
    return (p1 - p0) * (3.0 * u * u) + (p2 - p1) * (6.0 * u * t) + (p3 - p2) * (3.0 * t * t);
}

Vec2 BezierSpan::tangentAt(double t) const {
    Vec2 d = derivativeAt(t);
    if (d.lengthSquared() > 0.0) {
        return d.normalized();
    }
    // Degenerate derivative (coincident control points): use the next distinct control point.
    if (t <= 0.5) {
        for (const Vec2 q : {p1, p2, p3}) {
            if (q != p0) {
                return (q - p0).normalized();
            }
        }
    } else {
        for (const Vec2 q : {p2, p1, p0}) {
            if (q != p3) {
                return (p3 - q).normalized();
            }
        }
    }
    return {};
}

Box2 BezierSpan::bounds() const {
    Box2 box{p0, p3};
    // x(t), y(t) are cubics; their derivatives are quadratics a*t^2 + b*t + c.
    const auto addExtrema = [&](double c0, double c1, double c2, double c3) {
        const double a = -c0 + 3.0 * c1 - 3.0 * c2 + c3;
        const double b = 2.0 * (c0 - 2.0 * c1 + c2);
        const double c = c1 - c0;
        const auto consider = [&](double t) {
            if (t > 0.0 && t < 1.0) {
                box.expand(pointAt(t));
            }
        };
        if (std::fabs(a) < 1e-14) {
            if (std::fabs(b) > 1e-14) {
                consider(-c / b);
            }
            return;
        }
        const double disc = b * b - 4.0 * a * c;
        if (disc < 0.0) {
            return;
        }
        const double sq = std::sqrt(disc);
        consider((-b + sq) / (2.0 * a));
        consider((-b - sq) / (2.0 * a));
    };
    addExtrema(p0.x, p1.x, p2.x, p3.x);
    addExtrema(p0.y, p1.y, p2.y, p3.y);
    return box;
}

std::pair<BezierSpan, BezierSpan> BezierSpan::split(double t) const {
    const Vec2 p01 = lerp(p0, p1, t);
    const Vec2 p12 = lerp(p1, p2, t);
    const Vec2 p23 = lerp(p2, p3, t);
    const Vec2 p012 = lerp(p01, p12, t);
    const Vec2 p123 = lerp(p12, p23, t);
    const Vec2 mid = lerp(p012, p123, t);
    return {BezierSpan{p0, p01, p012, mid}, BezierSpan{mid, p123, p23, p3}};
}

double BezierSpan::flatness() const {
    return std::max(distanceToChord(p1, p0, p3), distanceToChord(p2, p0, p3));
}

// ------------------------------------------------------------ Span (variant)

double length(const Span& s) {
    return std::visit([](const auto& v) { return v.length(); }, s);
}

Vec2 startPoint(const Span& s) {
    return std::visit(
        [](const auto& v) -> Vec2 {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return v.start;
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                return v.startPoint();
            } else {
                return v.p0;
            }
        },
        s);
}

Vec2 endPoint(const Span& s) {
    return std::visit(
        [](const auto& v) -> Vec2 {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return v.end;
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                return v.endPoint();
            } else {
                return v.p3;
            }
        },
        s);
}

Vec2 pointAt(const Span& s, double t) {
    return std::visit([t](const auto& v) { return v.pointAt(t); }, s);
}

Vec2 tangentAt(const Span& s, double t) {
    return std::visit([t](const auto& v) { return v.tangentAt(t); }, s);
}

Box2 bounds(const Span& s) {
    return std::visit([](const auto& v) { return v.bounds(); }, s);
}

Span reversed(const Span& s) {
    return std::visit([](const auto& v) -> Span { return v.reversed(); }, s);
}

std::vector<Span> transformed(const Span& s, const Transform2& m) {
    return std::visit(
        [&m](const auto& v) -> std::vector<Span> {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, ArcSpan>) {
                if (m.isConformal()) {
                    ArcSpan a;
                    a.centre = m.apply(v.centre);
                    a.radius = v.radius * m.uniformScale();
                    a.startAngle = m.applyVector(Vec2::fromAngle(v.startAngle)).angle();
                    a.endAngle = m.applyVector(Vec2::fromAngle(v.endAngle)).angle();
                    const bool flip = m.isMirroring();
                    a.direction =
                        (v.direction == ArcDirection::CCW) != flip ? ArcDirection::CCW : ArcDirection::CW;
                    if (v.isFullCircle()) {
                        a.endAngle = a.startAngle;
                    }
                    return {a};
                }
                std::vector<Span> out;
                for (const BezierSpan& b : v.toBeziers()) {
                    out.emplace_back(b.transformed(m));
                }
                return out;
            } else {
                return {v.transformed(m)};
            }
        },
        s);
}

void polygoniseInto(const Span& s, const Tolerance& tol, std::vector<Vec2>& out) {
    std::visit(
        [&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                out.push_back(v.end);
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                const std::size_t n = v.segmentCountFor(tol.chordal);
                for (std::size_t i = 1; i <= n; ++i) {
                    out.push_back(v.pointAt(static_cast<double>(i) / static_cast<double>(n)));
                }
            } else {
                bezierPolygoniseRecursive(v, tol.chordal, 0, out);
            }
        },
        s);
}

bool isDegenerate(const Span& s, const Tolerance& tol) {
    return length(s) <= tol.distance;
}

}  // namespace carvetto::geometry
