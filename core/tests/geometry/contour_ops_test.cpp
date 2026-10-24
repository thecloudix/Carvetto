#include "carvetto/geometry/contour_ops.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <vector>

#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;
using Catch::Matchers::WithinAbs;

namespace {
const Tolerance tol{};

bool near(Vec2 a, Vec2 b, double eps = 1e-9) {
    return approxEqual(a, b, eps);
}

Contour square10(bool ccw) {
    Contour c = Contour::rectangle(Box2{{0.0, 0.0}, {10.0, 10.0}});
    if (!ccw) {
        c.reverse();
    }
    return c;
}

// Closed "D": line, half-circle arc, line back. Lines + arc only.
Contour dShape() {
    Contour c;
    c.closed = true;
    c.append(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    c.append(ArcSpan::fromCentre({10.0, 5.0}, {10.0, 0.0}, {10.0, 10.0}, ArcDirection::CCW));
    c.append(LineSpan{{10.0, 10.0}, {0.0, 10.0}});
    c.append(LineSpan{{0.0, 10.0}, {0.0, 0.0}});
    return c;
}

// Figure-eight (bow tie): two triangles meeting at (5,5), traced as one loop.
Contour bowTie() {
    Contour c;
    c.closed = true;
    c.append(LineSpan{{0.0, 0.0}, {10.0, 10.0}});
    c.append(LineSpan{{10.0, 10.0}, {10.0, 0.0}});
    c.append(LineSpan{{10.0, 0.0}, {0.0, 10.0}});
    c.append(LineSpan{{0.0, 10.0}, {0.0, 0.0}});
    return c;
}
}  // namespace

// ------------------------------------------------------------- area / orientation

TEST_CASE("Signed area of a 10x10 square is +100 CCW and -100 CW (AC3)", "[geometry][contour_ops]") {
    CHECK_THAT(signedArea(square10(true)), WithinAbs(100.0, 1e-12));
    CHECK_THAT(signedArea(square10(false)), WithinAbs(-100.0, 1e-12));
    CHECK_THAT(area(square10(false)), WithinAbs(100.0, 1e-12));
    CHECK(orientation(square10(true), tol) == Orientation::CCW);
    CHECK(orientation(square10(false), tol) == Orientation::CW);
}

TEST_CASE("Signed area is exact for arcs", "[geometry][contour_ops]") {
    const Contour ccw = Contour::circle({3.0, -2.0}, 5.0, ArcDirection::CCW);
    const Contour cw = Contour::circle({3.0, -2.0}, 5.0, ArcDirection::CW);
    CHECK_THAT(signedArea(ccw), WithinAbs(kPi * 25.0, 1e-9));
    CHECK_THAT(signedArea(cw), WithinAbs(-kPi * 25.0, 1e-9));
    // D shape: 10x10 square plus a half disc of radius 5 on the right.
    CHECK_THAT(signedArea(dShape()), WithinAbs(100.0 + 0.5 * kPi * 25.0, 1e-9));
    CHECK_THAT(signedArea(dShape().reversed()), WithinAbs(-(100.0 + 0.5 * kPi * 25.0), 1e-9));
}

TEST_CASE("Signed area of Bezier contours matches a fine polygonisation", "[geometry][contour_ops]") {
    Contour c;
    c.closed = true;
    c.append(LineSpan{{0.0, 0.0}, {20.0, 0.0}});
    c.append(BezierSpan{{20.0, 0.0}, {25.0, 10.0}, {15.0, 20.0}, {20.0, 25.0}});
    c.append(BezierSpan{{20.0, 25.0}, {10.0, 35.0}, {-5.0, 15.0}, {0.0, 0.0}});
    Tolerance fine;
    fine.chordal = 1e-5;
    const double expected = c.polygonise(fine).signedArea();
    CHECK_THAT(signedArea(c), WithinAbs(expected, 1e-3));
    // A Bezier that is really a straight line contributes like the line it is.
    Contour tri;
    tri.closed = true;
    tri.append(BezierSpan{{0.0, 0.0}, {3.0, 0.0}, {6.0, 0.0}, {9.0, 0.0}});
    tri.append(LineSpan{{9.0, 0.0}, {0.0, 6.0}});
    tri.append(LineSpan{{0.0, 6.0}, {0.0, 0.0}});
    CHECK_THAT(signedArea(tri), WithinAbs(27.0, 1e-12));
}

TEST_CASE("Area of open and degenerate contours", "[geometry][contour_ops]") {
    Contour open;
    open.append(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    open.append(LineSpan{{10.0, 0.0}, {10.0, 10.0}});
    CHECK(signedArea(open) == 0.0);
    CHECK(orientation(open, tol) == Orientation::Degenerate);

    Contour flat;
    flat.closed = true;
    flat.append(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    flat.append(LineSpan{{10.0, 0.0}, {0.0, 0.0}});
    CHECK_THAT(signedArea(flat), WithinAbs(0.0, 1e-12));
    CHECK(orientation(flat, tol) == Orientation::Degenerate);
    CHECK(signedArea(Contour{}) == 0.0);
}

TEST_CASE("Signed area is translation invariant", "[geometry][contour_ops]") {
    const Contour d = dShape();
    const Contour moved = d.transformed(Transform2::translation({-123.4, 567.8}));
    CHECK_THAT(signedArea(moved), WithinAbs(signedArea(d), 1e-7));
    const Contour rotated = d.transformed(Transform2::rotation(0.7, {4.0, 4.0}));
    CHECK_THAT(signedArea(rotated), WithinAbs(signedArea(d), 1e-9));
}

TEST_CASE("setOrientation reverses only when needed", "[geometry][contour_ops]") {
    Contour c = square10(false);
    setOrientation(c, Orientation::CCW, tol);
    CHECK(orientation(c, tol) == Orientation::CCW);
    const Contour before = c;
    setOrientation(c, Orientation::CCW, tol);
    CHECK(c == before);
    setOrientation(c, Orientation::CW, tol);
    CHECK(orientation(c, tol) == Orientation::CW);
    Contour open;
    open.append(LineSpan{{0.0, 0.0}, {1.0, 0.0}});
    const Contour openBefore = open;
    setOrientation(open, Orientation::CW, tol);
    CHECK(open == openBefore);
}

// ----------------------------------------------------------------- point in contour

TEST_CASE("isInside: (5,5) inside and (15,5) outside a square of either orientation (AC3)",
          "[geometry][contour_ops]") {
    for (const bool ccw : {true, false}) {
        const Contour sq = square10(ccw);
        for (const FillRule rule : {FillRule::EvenOdd, FillRule::NonZero}) {
            CHECK(isInside(sq, {5.0, 5.0}, rule, tol));
            CHECK_FALSE(isInside(sq, {15.0, 5.0}, rule, tol));
            CHECK_FALSE(isInside(sq, {-1.0, 5.0}, rule, tol));
            CHECK_FALSE(isInside(sq, {5.0, 10.5}, rule, tol));
        }
    }
}

TEST_CASE("Winding number sign follows orientation", "[geometry][contour_ops]") {
    CHECK(windingNumber(square10(true), {5.0, 5.0}, tol) == 1);
    CHECK(windingNumber(square10(false), {5.0, 5.0}, tol) == -1);
    CHECK(windingNumber(square10(true), {15.0, 5.0}, tol) == 0);
    CHECK(windingNumber(Contour::circle({0.0, 0.0}, 5.0, ArcDirection::CW), {1.0, -1.0}, tol) == -1);
}

TEST_CASE("Point location against arcs and Beziers", "[geometry][contour_ops]") {
    const Contour circle = Contour::circle({0.0, 0.0}, 5.0);
    CHECK(locatePoint(circle, {0.0, 0.0}, FillRule::NonZero, tol) == PointLocation::Inside);
    CHECK(locatePoint(circle, {4.9, 0.0}, FillRule::NonZero, tol) == PointLocation::Inside);
    CHECK(locatePoint(circle, {5.1, 0.0}, FillRule::NonZero, tol) == PointLocation::Outside);
    CHECK(locatePoint(circle, {5.0, 0.0}, FillRule::NonZero, tol) == PointLocation::OnBoundary);
    CHECK(locatePoint(circle, {-5.0, 0.0}, FillRule::NonZero, tol) == PointLocation::OnBoundary);
    // Ray from a point exactly level with the centre passes through the right-most point.
    CHECK(locatePoint(circle, {-7.0, 0.0}, FillRule::NonZero, tol) == PointLocation::Outside);
    // Ray from a point level with the top passes tangentially: no crossing.
    CHECK(locatePoint(circle, {-7.0, 5.0}, FillRule::NonZero, tol) == PointLocation::Outside);
    CHECK(locatePoint(circle, {-2.0, 4.0}, FillRule::EvenOdd, tol) == PointLocation::Inside);

    const Contour d = dShape();
    CHECK(isInside(d, {12.0, 5.0}, FillRule::EvenOdd, tol));  // in the half disc
    CHECK_FALSE(isInside(d, {14.0, 1.0}, FillRule::EvenOdd, tol));
    CHECK_FALSE(isInside(d, {15.1, 5.0}, FillRule::EvenOdd, tol));
    CHECK(locatePoint(d, {15.0, 5.0}, FillRule::EvenOdd, tol) == PointLocation::OnBoundary);

    Contour blob;
    blob.closed = true;
    blob.append(BezierSpan{{0.0, 0.0}, {10.0, -5.0}, {10.0, 15.0}, {0.0, 10.0}});
    blob.append(BezierSpan{{0.0, 10.0}, {-10.0, 15.0}, {-10.0, -5.0}, {0.0, 0.0}});
    CHECK(isInside(blob, {0.0, 5.0}, FillRule::EvenOdd, tol));
    CHECK(isInside(blob, {4.0, 5.0}, FillRule::NonZero, tol));
    CHECK_FALSE(isInside(blob, {8.0, 5.0}, FillRule::NonZero, tol));
    CHECK_FALSE(isInside(blob, {0.0, 12.0}, FillRule::NonZero, tol));
}

TEST_CASE("Ray through vertices is handled by the half-open rule", "[geometry][contour_ops]") {
    // Diamond: the ray from (0,0) along +x passes exactly through the vertex (5,0).
    Contour diamond;
    diamond.closed = true;
    diamond.append(LineSpan{{5.0, 0.0}, {0.0, 5.0}});
    diamond.append(LineSpan{{0.0, 5.0}, {-5.0, 0.0}});
    diamond.append(LineSpan{{-5.0, 0.0}, {0.0, -5.0}});
    diamond.append(LineSpan{{0.0, -5.0}, {5.0, 0.0}});
    CHECK(windingNumber(diamond, {0.0, 0.0}, tol) == 1);
    CHECK(windingNumber(diamond, {-7.0, 0.0}, tol) == 0);
    // Level with the top vertex, to its left: touching, not crossing.
    CHECK(windingNumber(diamond, {-7.0, 5.0}, tol) == 0);
    // Level with the bottom vertex (y = -5) and left of it.
    CHECK(windingNumber(diamond, {-7.0, -5.0}, tol) == 0);
    // Square's bottom edge is level with the ray: inside point on that level counts correctly.
    CHECK(windingNumber(square10(true), {5.0, 0.0}, tol) + windingNumber(square10(true), {5.0, 10.0}, tol) ==
          1);
}

TEST_CASE("Points on the boundary are OnBoundary and count as inside", "[geometry][contour_ops]") {
    const Contour sq = square10(true);
    CHECK(locatePoint(sq, {10.0, 5.0}, FillRule::EvenOdd, tol) == PointLocation::OnBoundary);
    CHECK(locatePoint(sq, {0.0, 0.0}, FillRule::EvenOdd, tol) == PointLocation::OnBoundary);
    CHECK(locatePoint(sq, {10.0005, 5.0}, FillRule::EvenOdd, tol) == PointLocation::OnBoundary);
    CHECK(locatePoint(sq, {10.002, 5.0}, FillRule::EvenOdd, tol) == PointLocation::Outside);
    CHECK(isInside(sq, {10.0, 5.0}, FillRule::EvenOdd, tol));
}

TEST_CASE("Fill rules differ only for self-overlapping regions", "[geometry][contour_ops]") {
    // Bow tie: both lobes have winding +-1 under either rule.
    const Contour tie = bowTie();
    CHECK(isInside(tie, {7.5, 5.0}, FillRule::EvenOdd, tol));
    CHECK(isInside(tie, {7.5, 5.0}, FillRule::NonZero, tol));
    CHECK_FALSE(isInside(tie, {5.0, 1.0}, FillRule::EvenOdd, tol));
    CHECK_FALSE(isInside(tie, {5.0, 1.0}, FillRule::NonZero, tol));
    CHECK(windingNumber(tie, {2.5, 5.0}, tol) == -windingNumber(tie, {7.5, 5.0}, tol));

    // Two nested CCW squares: the inner region has winding 2.
    const std::vector<Contour> nested{square10(true), Contour::rectangle(Box2{{3.0, 3.0}, {7.0, 7.0}})};
    CHECK_FALSE(isInside(nested, {5.0, 5.0}, FillRule::EvenOdd, tol));
    CHECK(isInside(nested, {5.0, 5.0}, FillRule::NonZero, tol));
    CHECK(isInside(nested, {1.0, 1.0}, FillRule::EvenOdd, tol));
    CHECK(isInside(nested, {1.0, 1.0}, FillRule::NonZero, tol));

    // Outer CCW with a CW hole: a true hole under both rules.
    std::vector<Contour> withHole{square10(true),
                                  Contour::rectangle(Box2{{3.0, 3.0}, {7.0, 7.0}}).reversed()};
    CHECK(locatePoint(withHole, {5.0, 5.0}, FillRule::EvenOdd, tol) == PointLocation::Outside);
    CHECK(locatePoint(withHole, {5.0, 5.0}, FillRule::NonZero, tol) == PointLocation::Outside);
    CHECK(locatePoint(withHole, {1.0, 1.0}, FillRule::NonZero, tol) == PointLocation::Inside);
    CHECK(locatePoint(withHole, {3.0, 5.0}, FillRule::NonZero, tol) == PointLocation::OnBoundary);
    CHECK(locatePoint(withHole, {50.0, 5.0}, FillRule::NonZero, tol) == PointLocation::Outside);
}

TEST_CASE("Open contours are implicitly closed for point tests; empty contours contain nothing",
          "[geometry][contour_ops]") {
    Contour open = square10(true);
    open.spans.pop_back();  // three sides, open
    open.closed = false;
    CHECK(isInside(open, {5.0, 5.0}, FillRule::EvenOdd, tol));
    CHECK_FALSE(isInside(open, {-1.0, 5.0}, FillRule::EvenOdd, tol));
    CHECK_FALSE(isInside(Contour{}, {0.0, 0.0}, FillRule::EvenOdd, tol));
    CHECK(locatePoint(Contour{}, {0.0, 0.0}, FillRule::EvenOdd, tol) == PointLocation::Outside);
}

// -------------------------------------------------------------------- closest point

TEST_CASE("Closest point on a half-circle arc from an external point is analytic (AC4)",
          "[geometry][contour_ops]") {
    // Upper half circle r=5 about the origin, CCW from (5,0) to (-5,0).
    const ArcSpan arc = ArcSpan::fromCentre({0.0, 0.0}, {5.0, 0.0}, {-5.0, 0.0}, ArcDirection::CCW);
    {
        // Radial direction inside the sweep: the answer is the point on the circle along it.
        const Vec2 p{3.0, 4.0 + 10.0};  // direction (3,14)
        const ClosestPoint cp = closestPoint(arc, p);
        const Vec2 expected = p.normalized() * 5.0;
        CHECK(near(cp.point, expected));
        CHECK_THAT(cp.distance, WithinAbs(p.length() - 5.0, 1e-9));
        CHECK_THAT(cp.t, WithinAbs(std::atan2(14.0, 3.0) / kPi, 1e-12));
    }
    {
        // Straight above the centre: apex (0,5), t = 0.5.
        const ClosestPoint cp = closestPoint(arc, {0.0, 20.0});
        CHECK(near(cp.point, {0.0, 5.0}));
        CHECK_THAT(cp.t, WithinAbs(0.5, 1e-12));
        CHECK_THAT(cp.distance, WithinAbs(15.0, 1e-9));
    }
    {
        // Below the arc, outside its sweep: nearer end point wins.
        const ClosestPoint cp = closestPoint(arc, {4.0, -3.0});
        CHECK(near(cp.point, {5.0, 0.0}));
        CHECK_THAT(cp.t, WithinAbs(0.0, 1e-12));
        CHECK_THAT(cp.distance, WithinAbs(std::sqrt(10.0), 1e-9));
        const ClosestPoint cp2 = closestPoint(arc, {-4.0, -3.0});
        CHECK(near(cp2.point, {-5.0, 0.0}));
        CHECK_THAT(cp2.t, WithinAbs(1.0, 1e-12));
    }
    {
        // Inside the circle but within the sweep: still the radial projection.
        const ClosestPoint cp = closestPoint(arc, {0.0, 1.0});
        CHECK(near(cp.point, {0.0, 5.0}));
        CHECK_THAT(cp.distance, WithinAbs(4.0, 1e-9));
    }
    // Centre query is well defined.
    const ClosestPoint centre = closestPoint(arc, {0.0, 0.0});
    CHECK_THAT(centre.distance, WithinAbs(5.0, 1e-9));
    CHECK(near(centre.point, {5.0, 0.0}));
}

TEST_CASE("Closest point on lines", "[geometry][contour_ops]") {
    const LineSpan s{{0.0, 0.0}, {10.0, 0.0}};
    const ClosestPoint mid = closestPoint(s, {3.0, 4.0});
    CHECK(near(mid.point, {3.0, 0.0}));
    CHECK_THAT(mid.t, WithinAbs(0.3, 1e-12));
    CHECK_THAT(mid.distance, WithinAbs(4.0, 1e-12));
    const ClosestPoint past = closestPoint(s, {14.0, 3.0});
    CHECK(near(past.point, {10.0, 0.0}));
    CHECK_THAT(past.t, WithinAbs(1.0, 1e-12));
    CHECK_THAT(past.distance, WithinAbs(5.0, 1e-12));
    const ClosestPoint before = closestPoint(s, {-1.0, 0.0});
    CHECK_THAT(before.t, WithinAbs(0.0, 1e-12));
    // Degenerate line.
    const ClosestPoint deg = closestPoint(LineSpan{{2.0, 2.0}, {2.0, 2.0}}, {5.0, 6.0});
    CHECK(near(deg.point, {2.0, 2.0}));
    CHECK_THAT(deg.distance, WithinAbs(5.0, 1e-12));
}

TEST_CASE("Closest point on Beziers", "[geometry][contour_ops]") {
    // Symmetric arch: closest point to a point on the axis of symmetry is the apex (5,3).
    const BezierSpan arch{{0.0, 0.0}, {3.0, 4.0}, {7.0, 4.0}, {10.0, 0.0}};
    const ClosestPoint apex = closestPoint(arch, {5.0, 10.0});
    CHECK(near(apex.point, {5.0, 3.0}, 1e-9));
    CHECK_THAT(apex.t, WithinAbs(0.5, 1e-9));
    CHECK_THAT(apex.distance, WithinAbs(7.0, 1e-9));
    // Query far past one end: the end point wins.
    const ClosestPoint end = closestPoint(arch, {20.0, -1.0});
    CHECK(near(end.point, {10.0, 0.0}));
    CHECK_THAT(end.t, WithinAbs(1.0, 1e-12));
    // A Bezier that is really a straight line matches the line result.
    const BezierSpan straight{{0.0, 0.0}, {3.0, 3.0}, {6.0, 6.0}, {9.0, 9.0}};
    const ClosestPoint cp = closestPoint(straight, {9.0, 0.0});
    CHECK(near(cp.point, {4.5, 4.5}, 1e-9));
    CHECK_THAT(cp.distance, WithinAbs(4.5 * std::sqrt(2.0), 1e-9));
    // Points on the curve itself have zero distance.
    const Vec2 on = arch.pointAt(0.3);
    CHECK_THAT(closestPoint(arch, on).distance, WithinAbs(0.0, 1e-9));
    CHECK_THAT(closestPoint(arch, on).t, WithinAbs(0.3, 1e-7));
}

TEST_CASE("Closest point on a contour picks the right span", "[geometry][contour_ops]") {
    const Contour d = dShape();
    const ContourClosestPoint right = closestPoint(d, {20.0, 5.0});
    CHECK(right.spanIndex == 1);
    CHECK(near(right.point, {15.0, 5.0}));
    CHECK_THAT(right.distance, WithinAbs(5.0, 1e-9));
    CHECK_THAT(right.t, WithinAbs(0.5, 1e-12));

    const ContourClosestPoint left = closestPoint(d, {-3.0, 4.0});
    CHECK(left.spanIndex == 3);
    CHECK(near(left.point, {0.0, 4.0}));
    CHECK_THAT(distanceTo(d, {-3.0, 4.0}), WithinAbs(3.0, 1e-12));

    // Tie between two spans at a corner: lower index wins.
    const ContourClosestPoint corner = closestPoint(d, {-1.0, -1.0});
    CHECK(corner.spanIndex == 0);
    CHECK(near(corner.point, {0.0, 0.0}));

    const ContourClosestPoint none = closestPoint(Contour{}, {1.0, 2.0});
    CHECK(std::isinf(none.distance));
    CHECK(std::isinf(distanceTo(Contour{}, {0.0, 0.0})));
}

// ------------------------------------------------- round-3 regressions (generative)

namespace {

// Brute-force reference: 1e5 uniform samples in power form, then golden-section refinement of
// the best sample's neighbourhood.
double bruteForceDistance(const BezierSpan& b, Vec2 q) {
    const Vec2 c3 = b.p3 - b.p0 + (b.p1 - b.p2) * 3.0;
    const Vec2 c2 = (b.p0 - b.p1 * 2.0 + b.p2) * 3.0;
    const Vec2 c1 = (b.p1 - b.p0) * 3.0;
    const Vec2 c0 = b.p0 - q;
    const auto d2 = [&](double t) {
        const double x = ((c3.x * t + c2.x) * t + c1.x) * t + c0.x;
        const double y = ((c3.y * t + c2.y) * t + c1.y) * t + c0.y;
        return x * x + y * y;
    };
    constexpr int n = 100000;
    int bestI = 0;
    double best = d2(0.0);
    for (int i = 1; i <= n; ++i) {
        const double v = d2(static_cast<double>(i) / n);
        if (v < best) {
            best = v;
            bestI = i;
        }
    }
    double lo = std::max(0, bestI - 1) / static_cast<double>(n);
    double hi = std::min(n, bestI + 1) / static_cast<double>(n);
    constexpr double invPhi = 0.6180339887498949;
    for (int iter = 0; iter < 100; ++iter) {
        const double x1 = hi - invPhi * (hi - lo);
        const double x2 = lo + invPhi * (hi - lo);
        if (d2(x1) < d2(x2)) {
            hi = x2;
        } else {
            lo = x1;
        }
    }
    best = std::min({best, d2(lo), d2(hi), d2(0.5 * (lo + hi))});
    return std::sqrt(best);
}

// A random cubic from one of several families: generic, loop, cusp, near-degenerate, and
// coincident control points, at a random scale, rotation and position.
BezierSpan randomBezier(SplitMix& rng, int family) {
    std::array<Vec2, 4> p{};
    switch (family) {
        case 0:  // generic
            for (Vec2& v : p) {
                v = {rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)};
            }
            break;
        case 1:  // self-intersecting loop, jittered
            p = {Vec2{0.0, 0.0}, Vec2{3.0, 1.0}, Vec2{-2.0, 1.0}, Vec2{1.0, 0.0}};
            for (Vec2& v : p) {
                v += Vec2{rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2)};
            }
            break;
        case 2:  // exact cusp (B'(0.5) = 0)
            p = {Vec2{0.0, 0.0}, Vec2{1.0, 1.0}, Vec2{0.0, 1.0}, Vec2{1.0, 0.0}};
            break;
        case 3: {  // near-degenerate: collinear controls that backtrack, plus a tiny wobble
            const double w = rng.uniform(0.0, 1e-7);
            p = {Vec2{0.0, 0.0}, Vec2{rng.uniform(-1.0, 2.0), w}, Vec2{rng.uniform(-1.0, 2.0), -w},
                 Vec2{rng.uniform(-1.0, 1.0), 0.0}};
            break;
        }
        default: {  // coincident controls (p0 = p1 and/or p2 = p3, or a point-like curve)
            const Vec2 a{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)};
            const Vec2 b{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)};
            const int sub = static_cast<int>(rng.next() % 3);
            if (sub == 0) {
                p = {a, a, b, b};
            } else if (sub == 1) {
                p = {a, b, b, a};
            } else {
                const Vec2 e{rng.uniform(-1e-9, 1e-9), rng.uniform(-1e-9, 1e-9)};
                p = {a, a + e, a - e, a};
            }
            break;
        }
    }
    const double scale = rng.logUniform(1e-2, 1e3);
    const Transform2 m = Transform2::translation({rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0)}) *
                         Transform2::rotation(rng.uniform(0.0, kTwoPi)) * Transform2::scaling(scale);
    return BezierSpan{p[0], p[1], p[2], p[3]}.transformed(m);
}

double curveScale(const BezierSpan& b) {
    const Box2 box = Box2{b.p0, b.p3}.united(Box2{b.p1, b.p2});
    return std::max({1.0, box.width(), box.height()});
}

}  // namespace

TEST_CASE("Closest point on a Bezier finds the global minimum (verifier repro)", "[geometry][contour_ops]") {
    const BezierSpan b{{6.1469989400297038, 5.3710241012533224},
                       {2.1595777605659254, -3.3024230383689979},
                       {8.2890946364353155, -5.1259080156549466},
                       {3.5442058110897925, 2.9709007947767123}};
    const Vec2 q = b.pointAt(0.202122019);
    const ClosestPoint cp = closestPoint(b, q);
    CHECK(cp.distance <= 1e-9);
    CHECK_THAT(cp.t, WithinAbs(0.202122019, 1e-7));
}

TEST_CASE("Closest point on Beziers matches brute force (generative)", "[geometry][contour_ops][property]") {
    SplitMix rng{0xB321E4ULL};
    double worstExcess = 0.0;
    double worstOnCurve = 0.0;
    int failures = 0;
    for (int i = 0; i < 20000; ++i) {
        const BezierSpan b = randomBezier(rng, i % 5);
        const double scale = curveScale(b);

        // On-curve query: distance must vanish.
        const double tq = rng.uniform(0.0, 1.0);
        const ClosestPoint on = closestPoint(b, b.pointAt(tq));
        worstOnCurve = std::max(worstOnCurve, on.distance / scale);

        // Off-curve query: never worse than brute force.
        const Vec2 q{rng.uniform(b.p0.x - scale, b.p0.x + scale),
                     rng.uniform(b.p0.y - scale, b.p0.y + scale)};
        const ClosestPoint cp = closestPoint(b, q);
        const double ref = bruteForceDistance(b, q);
        const double excess = cp.distance - ref;
        worstExcess = std::max(worstExcess, excess);
        // Self-consistency of the result.
        const bool consistent = std::fabs(distance(cp.point, q) - cp.distance) <= 1e-12 * scale &&
                                distance(cp.point, b.pointAt(cp.t)) <= 1e-9 * scale && cp.t >= 0.0 &&
                                cp.t <= 1.0;
        if (excess > 1e-9 || !consistent || on.distance > 1e-9 * scale) {
            ++failures;
            UNSCOPED_INFO("case " << i << " excess=" << excess << " onCurve=" << on.distance);
        }
    }
    INFO("worst excess over brute force " << worstExcess << ", worst on-curve distance/scale "
                                          << worstOnCurve);
    CHECK(failures == 0);
    CHECK(worstExcess <= 1e-9);
    CHECK(worstOnCurve <= 1e-9);
}

TEST_CASE("Closest point on a contour and distanceTo inherit the global Bezier search",
          "[geometry][contour_ops][property]") {
    SplitMix rng{0xC047002ULL};
    for (int i = 0; i < 500; ++i) {
        Contour c;
        std::vector<BezierSpan> parts;
        for (int k = 0; k < 4; ++k) {
            parts.push_back(randomBezier(rng, (i + k) % 5));
            c.append(parts.back());
        }
        const Vec2 q{rng.uniform(-150.0, 150.0), rng.uniform(-150.0, 150.0)};
        double ref = std::numeric_limits<double>::infinity();
        for (const BezierSpan& b : parts) {
            ref = std::min(ref, bruteForceDistance(b, q));
        }
        INFO("case " << i);
        const ContourClosestPoint cp = closestPoint(c, q);
        CHECK(cp.distance <= ref + 1e-9);
        CHECK(distanceTo(c, q) <= ref + 1e-9);
        CHECK_THAT(distance(cp.point, q), WithinAbs(cp.distance, 1e-9));
    }
}
