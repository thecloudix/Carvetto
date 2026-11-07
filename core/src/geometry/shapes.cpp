#include "carvetto/geometry/shapes.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

namespace carvetto::geometry {

namespace {

constexpr double kMaxMagnitude = 1e9;  // mm; beyond this squared sizes lose all precision

bool finite(double v) { return std::isfinite(v) && std::fabs(v) <= kMaxMagnitude; }
bool finite(Vec2 v) { return finite(v.x) && finite(v.y); }

void addLine(Contour& c, Vec2 a, Vec2 b, double minLength) {
    if (distance(a, b) > minLength) {
        c.spans.emplace_back(LineSpan{a, b});
    }
}

void addArc(Contour& c, Vec2 centre, double r, double a0, double a1) {
    c.spans.emplace_back(ArcSpan{centre, r, a0, a1, ArcDirection::CCW});
}

}  // namespace

ShapeResult<Contour> buildRectangle(Vec2 centre, double width, double height, RectCorner corner,
                                    double cornerSize) {
    if (!finite(centre) || !finite(width) || !finite(height) || !std::isfinite(cornerSize)) {
        return std::string("rectangle: parameters must be finite");
    }
    if (!(width > 0.0) || !(height > 0.0)) {
        return std::string("rectangle: width and height must be positive");
    }
    double r = 0.0;
    if (corner != RectCorner::Square) {
        if (cornerSize < 0.0) {
            return std::string("rectangle: corner size must not be negative");
        }
        if (cornerSize > 0.5 * std::min(width, height)) {
            return std::string("rectangle: corner size exceeds half the shorter side");
        }
        r = cornerSize;
    }
    const double x0 = centre.x - 0.5 * width;
    const double x1 = centre.x + 0.5 * width;
    const double y0 = centre.y - 0.5 * height;
    const double y1 = centre.y + 0.5 * height;
    Contour c;
    c.closed = true;
    if (r == 0.0) {
        c.spans.emplace_back(LineSpan{{x0, y0}, {x1, y0}});
        c.spans.emplace_back(LineSpan{{x1, y0}, {x1, y1}});
        c.spans.emplace_back(LineSpan{{x1, y1}, {x0, y1}});
        c.spans.emplace_back(LineSpan{{x0, y1}, {x0, y0}});
        return c;
    }
    // A side consumed completely (r == half the side, up to rounding) emits no line span.
    const double eps = 1e-9 * std::max(width, height);
    const double half = 0.5 * kPi;
    const Vec2 bl{x0 + r, y0 + r};
    const Vec2 br{x1 - r, y0 + r};
    const Vec2 tr{x1 - r, y1 - r};
    const Vec2 tl{x0 + r, y1 - r};
    const bool round = corner == RectCorner::Rounded;
    auto cornerSpan = [&](Vec2 cc, double a0, Vec2 from, Vec2 to) {
        if (round) {
            addArc(c, cc, r, a0, a0 + half);
        } else {
            c.spans.emplace_back(LineSpan{from, to});
        }
    };
    addLine(c, {x0 + r, y0}, {x1 - r, y0}, eps);
    cornerSpan(br, -half, {x1 - r, y0}, {x1, y0 + r});
    addLine(c, {x1, y0 + r}, {x1, y1 - r}, eps);
    cornerSpan(tr, 0.0, {x1, y1 - r}, {x1 - r, y1});
    addLine(c, {x1 - r, y1}, {x0 + r, y1}, eps);
    cornerSpan(tl, half, {x0 + r, y1}, {x0, y1 - r});
    addLine(c, {x0, y1 - r}, {x0, y0 + r}, eps);
    cornerSpan(bl, kPi, {x0, y0 + r}, {x0 + r, y0});
    return c;
}

ShapeResult<Contour> buildCircle(Vec2 centre, double radius) {
    if (!finite(centre) || !finite(radius)) {
        return std::string("circle: parameters must be finite");
    }
    if (!(radius > 0.0)) {
        return std::string("circle: radius must be positive");
    }
    return Contour::circle(centre, radius);
}

ShapeResult<Contour> buildEllipse(Vec2 centre, double width, double height, double angle,
                                  const Tolerance& tol) {
    if (!finite(centre) || !finite(width) || !finite(height) || !std::isfinite(angle)) {
        return std::string("ellipse: parameters must be finite");
    }
    if (!(width > 0.0) || !(height > 0.0)) {
        return std::string("ellipse: width and height must be positive");
    }
    if (width == height) {
        return Contour::circle(centre, 0.5 * width);
    }
    const double a = 0.5 * width;
    const double b = 0.5 * height;
    // A cubic fitted to a circular arc of step t deviates by about 2.7e-4 * r * (t / 90 deg)^6;
    // the ellipse is an affine image of the circle, so the radial error is bounded by that
    // times the longer semi-axis. 3e-4 keeps a margin.
    const double longest = std::max(a, b);
    std::size_t n = 4;
    while (n < 1024 && 3e-4 * longest * std::pow(4.0 / static_cast<double>(n), 6.0) > tol.distance) {
        n *= 2;
    }
    const double step = kTwoPi / static_cast<double>(n);
    const double k = 4.0 / 3.0 * std::tan(0.25 * step);
    const double ca = std::cos(angle);
    const double sa = std::sin(angle);
    auto place = [&](double lx, double ly) {
        return Vec2{centre.x + lx * ca - ly * sa, centre.y + lx * sa + ly * ca};
    };
    Contour c;
    c.closed = true;
    c.spans.reserve(n);
    Vec2 prev = place(a, 0.0);
    const Vec2 first = prev;
    for (std::size_t i = 0; i < n; ++i) {
        const double t0 = step * static_cast<double>(i);
        const double t1 = step * static_cast<double>(i + 1);
        const double c0 = std::cos(t0);
        const double s0 = std::sin(t0);
        const double c1 = std::cos(t1);
        const double s1 = std::sin(t1);
        Vec2 end = i + 1 == n ? first : place(a * c1, b * s1);
        const Vec2 p1 = place(a * (c0 - k * s0), b * (s0 + k * c0));
        const Vec2 p2 = place(a * (c1 + k * s1), b * (s1 - k * c1));
        c.spans.emplace_back(BezierSpan{prev, p1, p2, end});
        prev = end;
    }
    return c;
}

ShapeResult<Contour> buildPolygon(Vec2 centre, int sides, double radius, double angle) {
    if (!finite(centre) || !finite(radius) || !std::isfinite(angle)) {
        return std::string("polygon: parameters must be finite");
    }
    if (sides < 3) {
        return std::string("polygon: needs at least 3 sides");
    }
    if (sides > 100000) {
        return std::string("polygon: too many sides");
    }
    if (!(radius > 0.0)) {
        return std::string("polygon: radius must be positive");
    }
    const std::size_t n = static_cast<std::size_t>(sides);
    Contour c;
    c.closed = true;
    c.spans.reserve(n);
    const Vec2 first = centre + Vec2::fromAngle(angle, radius);
    Vec2 prev = first;
    for (std::size_t k = 1; k <= n; ++k) {
        const Vec2 p = k == n ? first
                              : centre + Vec2::fromAngle(angle + kTwoPi * static_cast<double>(k) /
                                                                     static_cast<double>(n),
                                                         radius);
        c.spans.emplace_back(LineSpan{prev, p});
        prev = p;
    }
    return c;
}

ShapeResult<Contour> buildStar(Vec2 centre, int points, double outerRadius, double innerRadius,
                               double angle) {
    if (!finite(centre) || !finite(outerRadius) || !finite(innerRadius) || !std::isfinite(angle)) {
        return std::string("star: parameters must be finite");
    }
    if (points < 3) {
        return std::string("star: needs at least 3 points");
    }
    if (points > 100000) {
        return std::string("star: too many points");
    }
    if (!(outerRadius > 0.0) || !(innerRadius > 0.0)) {
        return std::string("star: radii must be positive");
    }
    if (!(innerRadius < outerRadius)) {
        return std::string("star: inner radius must be smaller than the outer radius");
    }
    const std::size_t n = static_cast<std::size_t>(points);
    const double step = kTwoPi / static_cast<double>(n);
    Contour c;
    c.closed = true;
    c.spans.reserve(2 * n);
    const Vec2 first = centre + Vec2::fromAngle(angle, outerRadius);
    Vec2 prev = first;
    for (std::size_t k = 0; k < n; ++k) {
        const Vec2 valley =
            centre + Vec2::fromAngle(angle + step * (static_cast<double>(k) + 0.5), innerRadius);
        const Vec2 tip = k + 1 == n ? first
                                    : centre + Vec2::fromAngle(angle + step * static_cast<double>(k + 1),
                                                               outerRadius);
        c.spans.emplace_back(LineSpan{prev, valley});
        c.spans.emplace_back(LineSpan{valley, tip});
        prev = tip;
    }
    return c;
}

ShapeResult<Contour> buildArcThreePoint(Vec2 a, Vec2 b, Vec2 c, const Tolerance& tol) {
    if (!finite(a) || !finite(b) || !finite(c)) {
        return std::string("arc: parameters must be finite");
    }
    if (distance(a, b) <= tol.distance || distance(b, c) <= tol.distance ||
        distance(a, c) <= tol.distance) {
        return std::string("arc: the three points must be distinct");
    }
    const Vec2 ab = b - a;
    const Vec2 ac = c - a;
    const double d = 2.0 * ab.cross(ac);
    // |cross| = twice the triangle area; collinear when the height of b over chord ac is
    // below the distance tolerance.
    if (std::fabs(ab.cross(ac)) <= tol.distance * ac.length()) {
        return std::string("arc: the three points are collinear");
    }
    const double ab2 = ab.lengthSquared();
    const double ac2 = ac.lengthSquared();
    const Vec2 centre{a.x + (ac.y * ab2 - ab.y * ac2) / d, a.y + (ab.x * ac2 - ac.x * ab2) / d};
    const ArcDirection dir = ab.cross(c - b) > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
    Contour out;
    out.closed = false;
    out.spans.emplace_back(ArcSpan::fromCentre(centre, a, c, dir));
    return out;
}

ShapeResult<Contour> buildArcCentreStartEnd(Vec2 centre, Vec2 start, Vec2 end, ArcDirection dir,
                                            const Tolerance& tol) {
    if (!finite(centre) || !finite(start) || !finite(end)) {
        return std::string("arc: parameters must be finite");
    }
    const Vec2 s = start - centre;
    const Vec2 e = end - centre;
    if (s.length() <= tol.distance || e.length() <= tol.distance) {
        return std::string("arc: start and end must differ from the centre");
    }
    if (std::fabs(s.cross(e)) <= tol.angle * s.length() * e.length() && s.dot(e) > 0.0) {
        return std::string("arc: start and end lie on the same ray");
    }
    Contour out;
    out.closed = false;
    out.spans.emplace_back(ArcSpan::fromCentre(centre, start, end, dir));
    return out;
}

ShapeResult<Contour> buildArcStartEndRadius(Vec2 start, Vec2 end, double radius, ArcDirection dir,
                                            bool large, const Tolerance& tol) {
    if (!finite(start) || !finite(end) || !finite(radius)) {
        return std::string("arc: parameters must be finite");
    }
    if (!(radius > 0.0)) {
        return std::string("arc: radius must be positive");
    }
    const Vec2 chord = end - start;
    const double len = chord.length();
    if (len <= tol.distance) {
        return std::string("arc: start and end must be distinct");
    }
    const double halfChord = 0.5 * len;
    if (halfChord > radius + tol.distance) {
        return std::string("arc: radius is smaller than half the chord");
    }
    const double h = halfChord >= radius ? 0.0 : std::sqrt(radius * radius - halfChord * halfChord);
    // CCW travel keeps the centre on the left of start->end for the shorter arc; the longer arc,
    // and CW travel, each flip the side.
    const bool left = (dir == ArcDirection::CCW) != large;
    const Vec2 normal = chord.perp() / len;
    const Vec2 mid = start + chord * 0.5;
    const Vec2 centre = left ? mid + normal * h : mid - normal * h;
    Contour out;
    out.closed = false;
    ArcSpan arc = ArcSpan::fromCentre(centre, start, end, dir);
    arc.radius = radius;
    out.spans.emplace_back(arc);
    return out;
}

}  // namespace carvetto::geometry
