#include "carvetto/geometry/boolean.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

#include "carvetto/geometry/fixed_point.hpp"

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

const Tolerance kTol{};

double area(const Contour& c) {
    return c.polygonise(kTol).signedArea();
}

double totalArea(const std::vector<Contour>& cs) {
    double a = 0.0;
    for (const auto& c : cs) {
        a += area(c);
    }
    return a;
}

Contour rect(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2{{x0, y0}, {x1, y1}});
}

}  // namespace

// --- Acceptance criterion 3 -----------------------------------------------------------------

TEST_CASE("Union of two overlapping squares is one contour", "[geometry][boolean]") {
    const auto r = booleanOp(BooleanOp::Union, {rect(0, 0, 10, 10)}, {rect(5, 5, 15, 15)}, kTol);
    REQUIRE(r.closed.size() == 1);
    CHECK(r.open.empty());
    CHECK(r.closed[0].closed);
    CHECK(r.closed[0].isContinuous(kTol));
    CHECK_THAT(area(r.closed[0]), WithinAbs(175.0, 1e-6));
    CHECK(r.closed[0].spanCount() == 8);
}

TEST_CASE("Difference with a contained square yields an outer and a hole", "[geometry][boolean]") {
    const auto r = booleanOp(BooleanOp::Difference, {rect(0, 0, 10, 10)}, {rect(3, 3, 7, 7)}, kTol);
    REQUIRE(r.closed.size() == 2);
    const double a0 = area(r.closed[0]);
    const double a1 = area(r.closed[1]);
    CHECK(a0 * a1 < 0.0);  // opposite orientation
    CHECK_THAT(std::fabs(a0) + std::fabs(a1), WithinAbs(116.0, 1e-6));
    CHECK_THAT(a0 + a1, WithinAbs(84.0, 1e-6));
    const Contour& outer = a0 > 0.0 ? r.closed[0] : r.closed[1];
    const Contour& hole = a0 > 0.0 ? r.closed[1] : r.closed[0];
    CHECK_THAT(area(outer), WithinAbs(100.0, 1e-6));  // outer is CCW
    CHECK_THAT(area(hole), WithinAbs(-16.0, 1e-6));   // hole is CW
    CHECK(outer.bounds().contains(hole.bounds()));
}

TEST_CASE("Intersection and Xor of overlapping squares", "[geometry][boolean]") {
    const std::vector<Contour> a{rect(0, 0, 10, 10)};
    const std::vector<Contour> b{rect(5, 5, 15, 15)};
    SECTION("intersection is the overlap") {
        const auto r = booleanOp(BooleanOp::Intersection, a, b, kTol);
        REQUIRE(r.closed.size() == 1);
        CHECK_THAT(area(r.closed[0]), WithinAbs(25.0, 1e-6));
        CHECK(approxEqual(r.closed[0].bounds(), Box2{{5.0, 5.0}, {10.0, 10.0}}, 1e-6));
    }
    SECTION("xor is both minus the overlap") {
        const auto r = booleanOp(BooleanOp::Xor, a, b, kTol);
        CHECK_THAT(totalArea(r.closed), WithinAbs(150.0, 1e-6));
    }
    SECTION("difference removes the overlap from the subject") {
        const auto r = booleanOp(BooleanOp::Difference, a, b, kTol);
        REQUIRE(r.closed.size() == 1);
        CHECK_THAT(area(r.closed[0]), WithinAbs(75.0, 1e-6));
    }
    SECTION("intersection of disjoint shapes is empty") {
        const auto r = booleanOp(BooleanOp::Intersection, a, {rect(20, 20, 30, 30)}, kTol);
        CHECK(r.isEmpty());
    }
}

TEST_CASE("Union with circles respects the chordal tolerance", "[geometry][boolean]") {
    const auto r = booleanOp(BooleanOp::Union, {Contour::circle({0.0, 0.0}, 10.0)},
                             {Contour::circle({10.0, 0.0}, 10.0)}, kTol);
    REQUIRE(r.closed.size() == 1);
    // Two circles r=10 with centres 10 apart: lens area = 2 r^2 acos(1/2) - (d/2) sqrt(4r^2 - d^2)
    const double lens = 2.0 * 100.0 * std::acos(0.5) - 5.0 * std::sqrt(300.0);
    const double expected = 2.0 * kPi * 100.0 - lens;
    CHECK_THAT(area(r.closed[0]), WithinRel(expected, 0.002));
}

TEST_CASE("Fill rules", "[geometry][boolean]") {
    // Outer CCW square with an inner square of the SAME orientation.
    const std::vector<Contour> shapes{rect(0, 0, 10, 10), rect(3, 3, 7, 7)};
    SECTION("NonZero fills the inner square") {
        const auto u = unionContours(shapes, kTol, FillRule::NonZero);
        REQUIRE(u.size() == 1);
        CHECK_THAT(area(u[0]), WithinAbs(100.0, 1e-6));
    }
    SECTION("EvenOdd keeps it as a hole") {
        const auto u = unionContours(shapes, kTol, FillRule::EvenOdd);
        REQUIRE(u.size() == 2);
        CHECK_THAT(totalArea(u), WithinAbs(84.0, 1e-6));
    }
    SECTION("NonZero with an opposite-oriented inner square keeps the hole") {
        const auto u = unionContours({rect(0, 0, 10, 10), rect(3, 3, 7, 7).reversed()}, kTol);
        REQUIRE(u.size() == 2);
        CHECK_THAT(totalArea(u), WithinAbs(84.0, 1e-6));
    }
}

TEST_CASE("Union of text-like glyphs keeps holes (weld behaviour)", "[geometry][boolean]") {
    // Two "O" glyphs (outer CCW, hole CW) overlapping slightly; the two holes must survive.
    const std::vector<Contour> glyphs{rect(0, 0, 10, 10), rect(2, 2, 8, 8).reversed(), rect(9, 0, 19, 10),
                                      rect(11, 2, 17, 8).reversed()};
    const auto u = unionContours(glyphs, kTol);
    REQUIRE(u.size() == 3);
    const auto holes = std::count_if(u.begin(), u.end(), [](const Contour& c) { return area(c) < 0.0; });
    CHECK(holes == 2);
    CHECK_THAT(totalArea(u), WithinAbs(190.0 - 72.0, 1e-6));
}

// --- Edge cases -----------------------------------------------------------------------------

TEST_CASE("Boolean edge cases", "[geometry][boolean]") {
    SECTION("empty inputs") {
        CHECK(booleanOp(BooleanOp::Union, {}, {}, kTol).isEmpty());
        CHECK(booleanOp(BooleanOp::Intersection, {rect(0, 0, 1, 1)}, {}, kTol).isEmpty());
        const auto d = booleanOp(BooleanOp::Difference, {rect(0, 0, 1, 1)}, {}, kTol);
        REQUIRE(d.closed.size() == 1);
        CHECK_THAT(area(d.closed[0]), WithinAbs(1.0, 1e-9));
        CHECK(booleanOp(BooleanOp::Difference, {}, {rect(0, 0, 1, 1)}, kTol).isEmpty());
        CHECK(unionContours({Contour{}}, kTol).empty());
    }
    SECTION("degenerate closed contours are ignored") {
        Contour sliver;
        sliver.closed = true;
        sliver.spans.emplace_back(LineSpan{{0.0, 0.0}, {5.0, 0.0}});
        sliver.spans.emplace_back(LineSpan{{5.0, 0.0}, {0.0, 0.0}});
        CHECK(unionContours({sliver}, kTol).empty());
        const auto u = unionContours({sliver, rect(0, 0, 2, 2)}, kTol);
        REQUIRE(u.size() == 1);
        CHECK_THAT(area(u[0]), WithinAbs(4.0, 1e-9));
    }
    SECTION("self-touching squares sharing an edge are welded into one") {
        const auto u = unionContours({rect(0, 0, 10, 10), rect(10, 0, 20, 10)}, kTol);
        REQUIRE(u.size() == 1);
        CHECK_THAT(area(u[0]), WithinAbs(200.0, 1e-6));
        CHECK(u[0].spanCount() == 4);
    }
    SECTION("squares touching at one vertex stay separate") {
        const auto u = unionContours({rect(0, 0, 10, 10), rect(10, 10, 20, 20)}, kTol);
        CHECK(u.size() == 2);
        CHECK_THAT(totalArea(u), WithinAbs(200.0, 1e-6));
    }
    SECTION("a shape unioned with itself is itself") {
        const auto u = unionContours({rect(0, 0, 10, 10), rect(0, 0, 10, 10)}, kTol);
        REQUIRE(u.size() == 1);
        CHECK_THAT(area(u[0]), WithinAbs(100.0, 1e-9));
    }
    SECTION("difference of a shape from itself is empty") {
        CHECK(booleanOp(BooleanOp::Difference, {rect(0, 0, 10, 10)}, {rect(0, 0, 10, 10)}, kTol).isEmpty());
    }
    SECTION("subtracting a hole-carrying shape re-exposes the hole region") {
        // Big square minus (ring = square with hole): the hole region remains filled.
        const auto r = booleanOp(BooleanOp::Difference, {rect(0, 0, 10, 10)},
                                 {rect(2, 2, 8, 8), rect(4, 4, 6, 6).reversed()}, kTol);
        REQUIRE(r.closed.size() == 3);
        CHECK_THAT(totalArea(r.closed), WithinAbs(100.0 - 36.0 + 4.0, 1e-6));
    }
    SECTION("fixed-point grid rounding stays below the distance tolerance") {
        const Contour tiny = rect(0.1234567, 0.1234567, 0.7654321, 0.7654321);
        const auto u = unionContours({tiny}, kTol);
        REQUIRE(u.size() == 1);
        for (const Span& s : u[0].spans) {
            const Vec2 p = startPoint(s);
            const double gridX = std::round(p.x * kFixedPointScale) / kFixedPointScale;
            CHECK_THAT(p.x, WithinAbs(gridX, 1e-12));
            CHECK((std::fabs(p.x - 0.1234567) < 1e-6 || std::fabs(p.x - 0.7654321) < 1e-6));
        }
    }
}

TEST_CASE("Open subject contours are clipped as open paths", "[geometry][boolean]") {
    Contour line;
    line.spans.emplace_back(LineSpan{{-5.0, 5.0}, {15.0, 5.0}});
    SECTION("intersection keeps the part inside the clip") {
        const auto r = booleanOp(BooleanOp::Intersection, {line}, {rect(0, 0, 10, 10)}, kTol);
        CHECK(r.closed.empty());
        REQUIRE(r.open.size() == 1);
        CHECK_FALSE(r.open[0].closed);
        CHECK_THAT(r.open[0].length(), WithinAbs(10.0, 1e-6));
    }
    SECTION("difference keeps the parts outside the clip") {
        const auto r = booleanOp(BooleanOp::Difference, {line}, {rect(0, 0, 10, 10)}, kTol);
        REQUIRE(r.open.size() == 2);
        CHECK_THAT(r.open[0].length() + r.open[1].length(), WithinAbs(10.0, 1e-6));
    }
    SECTION("open contours in the clip set are ignored") {
        const auto r = booleanOp(BooleanOp::Union, {rect(0, 0, 10, 10)}, {line}, kTol);
        REQUIRE(r.closed.size() == 1);
        CHECK(r.open.empty());
    }
}
