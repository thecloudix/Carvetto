#include "carvetto/geometry/contour.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <variant>

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;

namespace {
bool near(Vec2 a, Vec2 b, double tol = 1e-9) {
    return approxEqual(a, b, tol);
}

// Largest distance from any point of the circle to the polyline's nearest segment,
// sampled densely around the circumference.
double maxChordalDeviationFromCircle(const Polyline& pl, Vec2 centre, double radius) {
    double worst = 0.0;
    const int samples = 20000;
    for (int i = 0; i < samples; ++i) {
        const Vec2 c = centre + Vec2::fromAngle(kTwoPi * i / samples, radius);
        double best = 1e300;
        for (std::size_t k = 0; k < pl.segmentCount(); ++k) {
            const Vec2 a = pl.segmentStart(k);
            const Vec2 b = pl.segmentEnd(k);
            const Vec2 ab = b - a;
            const double t = std::clamp((c - a).dot(ab) / ab.lengthSquared(), 0.0, 1.0);
            best = std::min(best, distance(c, a + ab * t));
        }
        worst = std::max(worst, best);
    }
    return worst;
}

// A closed contour mixing all three span types: a rounded "D" with a Bezier top.
Contour mixedShape() {
    Contour c;
    c.closed = true;
    c.spans.emplace_back(LineSpan{{0.0, 0.0}, {20.0, 0.0}});
    c.spans.emplace_back(ArcSpan::fromCentre({20.0, 10.0}, {20.0, 0.0}, {20.0, 20.0}, ArcDirection::CCW));
    c.spans.emplace_back(BezierSpan{{20.0, 20.0}, {15.0, 30.0}, {5.0, 30.0}, {0.0, 20.0}});
    c.spans.emplace_back(LineSpan{{0.0, 20.0}, {0.0, 0.0}});
    return c;
}
}  // namespace

TEST_CASE("Contour factories", "[geometry][contour]") {
    const Contour circ = Contour::circle({1.0, 2.0}, 5.0);
    CHECK(circ.closed);
    CHECK(circ.spanCount() == 1);
    CHECK_THAT(circ.length(), WithinAbs(kTwoPi * 5.0, 1e-9));
    CHECK(near(circ.startPoint(), {6.0, 2.0}));

    const Contour rect = Contour::rectangle(Box2{{0.0, 0.0}, {10.0, 5.0}});
    CHECK(rect.closed);
    CHECK(rect.spanCount() == 4);
    CHECK_THAT(rect.length(), WithinAbs(30.0, 1e-12));
    CHECK(rect.isContinuous(Tolerance{}));
    CHECK_THAT(rect.polygonise(Tolerance{}).signedArea(), WithinAbs(50.0, 1e-12));

    Polyline pl;
    pl.points = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    pl.closed = false;
    const Contour fromPl = Contour::fromPolyline(pl);
    CHECK(fromPl.spanCount() == 2);
    CHECK_FALSE(fromPl.closed);
    CHECK(fromPl.endPoint() == Vec2{1.0, 1.0});
    pl.closed = true;
    CHECK(Contour::fromPolyline(pl).spanCount() == 3);

    const Contour empty;
    CHECK(empty.isEmpty());
    CHECK(empty.startPoint() == Vec2{});
    CHECK(empty.length() == 0.0);
    CHECK(empty.bounds().isEmpty());
    CHECK(empty.polygonise(Tolerance{}).isEmpty());
    CHECK(empty.pointAt(0.5) == Vec2{});
}

TEST_CASE("Contour length, pointAt and continuity", "[geometry][contour]") {
    const Contour c = mixedShape();
    CHECK(c.isContinuous(Tolerance{}));
    const double expected = 20.0 + kPi * 10.0 + std::get<BezierSpan>(c.spans[2]).length() + 20.0;
    CHECK_THAT(c.length(), WithinAbs(expected, 1e-9));
    CHECK(near(c.pointAt(0.0), {0.0, 0.0}));
    CHECK(near(c.pointAt(1.0), {0.0, 0.0}));
    CHECK(near(c.pointAt(10.0 / c.length()), {10.0, 0.0}));
    CHECK(near(c.pointAt((20.0 + kPi * 5.0) / c.length()), {30.0, 10.0}));

    Contour broken = c;
    broken.spans[1] = LineSpan{{20.0, 0.0}, {20.0, 19.0}};
    CHECK_FALSE(broken.isContinuous(Tolerance{}));
    Contour notClosing = c;
    notClosing.spans.pop_back();
    CHECK_FALSE(notClosing.isContinuous(Tolerance{}));
    notClosing.closed = false;
    CHECK(notClosing.isContinuous(Tolerance{}));
}

// Acceptance criterion 4.
TEST_CASE("Polygonising a circle r=10 at 0.01 keeps deviation <= 0.01 with < 200 vertices",
          "[geometry][contour][acceptance]") {
    const Vec2 centre{3.0, -4.0};
    const Contour circ = Contour::circle(centre, 10.0);
    Tolerance tol;
    tol.chordal = 0.01;
    const Polyline pl = circ.polygonise(tol);
    CHECK(pl.closed);
    CHECK(pl.vertexCount() < 200);
    CHECK(pl.vertexCount() >= 3);
    // No duplicated closing vertex.
    CHECK_FALSE(near(pl.points.front(), pl.points.back(), 1e-6));
    for (const Vec2 p : pl.points) {
        CHECK_THAT(distance(p, centre), WithinAbs(10.0, 1e-9));
    }
    CHECK(maxChordalDeviationFromCircle(pl, centre, 10.0) <= 0.01 + 1e-9);
}

// Acceptance criterion 5.
TEST_CASE("Contour reverse of a closed contour yields the reversed vertex set",
          "[geometry][contour][acceptance]") {
    Tolerance tol;
    tol.chordal = 0.05;
    const Contour c = mixedShape();
    const Contour r = c.reversed();
    REQUIRE(r.spanCount() == c.spanCount());
    CHECK(r.closed);
    CHECK(r.isContinuous(tol));
    CHECK_THAT(r.length(), WithinAbs(c.length(), 1e-9));
    // The first span of the reverse is the reverse of the last span of the original.
    CHECK(near(startPoint(r.spans.front()), endPoint(c.spans.back())));
    CHECK(near(endPoint(r.spans.back()), startPoint(c.spans.front())));

    // Compare vertex sets of the polygonisations: every forward vertex has a match in the
    // reverse polygonisation and vice versa (the Bezier subdivision is symmetric at t=0.5,
    // so the vertex sets agree exactly up to floating point).
    const Polyline fwd = c.polygonise(tol);
    const Polyline rev = r.polygonise(tol);
    REQUIRE(fwd.vertexCount() == rev.vertexCount());
    const auto hasMatch = [](const Polyline& in, Vec2 p) {
        return std::any_of(in.points.begin(), in.points.end(), [&](Vec2 q) { return near(p, q, 1e-9); });
    };
    for (const Vec2 p : fwd.points) {
        CHECK(hasMatch(rev, p));
    }
    for (const Vec2 p : rev.points) {
        CHECK(hasMatch(fwd, p));
    }
    // Walking the reverse polyline backwards from its start reproduces the forward order (cyclically).
    const std::size_t n = fwd.vertexCount();
    std::size_t start = 0;
    while (start < n && !near(rev.points[start], fwd.points[0], 1e-9)) {
        ++start;
    }
    REQUIRE(start < n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 expected = fwd.points[i];
        const Vec2 actual = rev.points[(start + n - i) % n];
        CHECK(near(actual, expected, 1e-9));
    }
    CHECK_THAT(fwd.signedArea(), WithinAbs(-rev.signedArea(), 1e-9));
    CHECK(fwd.signedArea() > 0.0);

    // Open contour reverse swaps the end points.
    Contour open = c;
    open.closed = false;
    open.spans.pop_back();
    const Contour ro = open.reversed();
    CHECK_FALSE(ro.closed);
    CHECK(near(ro.startPoint(), open.endPoint()));
    CHECK(near(ro.endPoint(), open.startPoint()));
    CHECK(ro.reversed() == open);
}

// Acceptance criterion 5 (bounds).
TEST_CASE("Contour bounds match the polygonised bounds within tolerance", "[geometry][contour][acceptance]") {
    Tolerance tol;
    tol.chordal = 0.01;
    const Contour shapes[] = {mixedShape(), Contour::circle({3.0, -4.0}, 10.0),
                              Contour::rectangle(Box2{{-1.0, -2.0}, {5.0, 7.0}}),
                              mixedShape().transformed(Transform2::rotation(0.6, {7.0, 7.0}))};
    for (const Contour& c : shapes) {
        const Box2 exact = c.bounds();
        const Box2 poly = c.polygonise(tol).bounds();
        CHECK_THAT(poly.min.x, WithinAbs(exact.min.x, tol.chordal));
        CHECK_THAT(poly.min.y, WithinAbs(exact.min.y, tol.chordal));
        CHECK_THAT(poly.max.x, WithinAbs(exact.max.x, tol.chordal));
        CHECK_THAT(poly.max.y, WithinAbs(exact.max.y, tol.chordal));
        // The exact box always contains the polygon (vertices lie on the curves).
        CHECK(exact.inflated(1e-9).contains(poly));
    }
    const Box2 b = mixedShape().bounds();
    CHECK_THAT(b.min.x, WithinAbs(0.0, 1e-12));
    CHECK_THAT(b.min.y, WithinAbs(0.0, 1e-12));
    CHECK_THAT(b.max.x, WithinAbs(30.0, 1e-12));
    CHECK_THAT(b.max.y, WithinAbs(27.5, 1e-12));  // Bezier peak at t=0.5: 0.125*20+0.375*30*2+0.125*20
}

TEST_CASE("Contour transform", "[geometry][contour]") {
    const Contour c = mixedShape();
    const Transform2 rigid = Transform2::translation({5.0, 6.0}) * Transform2::rotation(0.3);
    const Contour t = c.transformed(rigid);
    REQUIRE(t.spanCount() == 4);
    CHECK(std::holds_alternative<ArcSpan>(t.spans[1]));
    CHECK(t.closed);
    CHECK(t.isContinuous(Tolerance{}));
    CHECK_THAT(t.length(), WithinAbs(c.length(), 1e-9));
    for (int i = 0; i <= 10; ++i) {
        const double u = i / 10.0;
        CHECK(near(t.pointAt(u), rigid.apply(c.pointAt(u)), 1e-9));
    }

    // Non-uniform scale turns the half-circle into two Beziers (3 + 2 = 5 spans).
    Contour s = c;
    s.transform(Transform2::scaling(1.0, 2.0));
    CHECK(s.spanCount() == 5);
    CHECK(s.isContinuous(Tolerance{}));
    CHECK(approxEqual(s.bounds(), Box2{{0.0, 0.0}, {30.0, 55.0}}, 1e-6));
}
