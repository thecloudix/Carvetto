#include "carvetto/geometry/intersect.hpp"

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>

#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;
using Catch::Matchers::WithinAbs;

namespace {
const Tolerance tol{};

bool near(Vec2 a, Vec2 b, double eps = 1e-9) {
    return approxEqual(a, b, eps);
}

bool hasPointNear(const IntersectionResult& r, Vec2 p, double eps = 1e-9) {
    for (const Intersection& hit : r.points) {
        if (near(hit.point, p, eps)) {
            return true;
        }
    }
    return false;
}

bool hasPointNear(const ContourIntersections& r, Vec2 p, double eps = 1e-9) {
    for (const ContourIntersection& hit : r.points) {
        if (near(hit.hit.point, p, eps)) {
            return true;
        }
    }
    return false;
}

Contour squareAt(Vec2 origin, double size) {
    return Contour::rectangle(Box2{origin, origin + Vec2{size, size}});
}
}  // namespace

// ------------------------------------------------------------------- line / line

TEST_CASE("Line-line crossing intersection", "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 10.0}};
    const LineSpan b{{0.0, 10.0}, {10.0, 0.0}};
    const IntersectionResult r = intersect(a, b, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(r.overlaps.empty());
    CHECK(near(r.points[0].point, {5.0, 5.0}));
    CHECK_THAT(r.points[0].tA, WithinAbs(0.5, 1e-12));
    CHECK_THAT(r.points[0].tB, WithinAbs(0.5, 1e-12));
    CHECK_FALSE(r.points[0].tangent);
}

TEST_CASE("Line-line: segments whose infinite lines cross outside them do not intersect",
          "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {1.0, 1.0}};
    const LineSpan b{{5.0, 0.0}, {0.0, 5.0}};
    CHECK(intersect(a, b, tol).isEmpty());
}

TEST_CASE("Line-line: hit at a shared end point is reported once with clamped parameters",
          "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 0.0}};
    const LineSpan b{{10.0, 0.0}, {10.0, 5.0}};
    const IntersectionResult r = intersect(a, b, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(near(r.points[0].point, {10.0, 0.0}));
    CHECK_THAT(r.points[0].tA, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.points[0].tB, WithinAbs(0.0, 1e-12));
}

TEST_CASE("Line-line: parallel offset segments do not intersect", "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 0.0}};
    const LineSpan b{{0.0, 1.0}, {10.0, 1.0}};
    CHECK(intersect(a, b, tol).isEmpty());
}

TEST_CASE("Line-line: collinear overlapping segments report an overlap", "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 0.0}};
    const LineSpan b{{4.0, 0.0}, {20.0, 0.0}};
    const IntersectionResult r = intersect(a, b, tol);
    CHECK(r.points.empty());
    REQUIRE(r.overlaps.size() == 1);
    CHECK_THAT(r.overlaps[0].tA0, WithinAbs(0.4, 1e-12));
    CHECK_THAT(r.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB1, WithinAbs(6.0 / 16.0, 1e-12));
}

TEST_CASE("Line-line: collinear overlap with opposite direction keeps B parameters in B's order",
          "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 0.0}};
    const LineSpan b{{8.0, 0.0}, {2.0, 0.0}};
    const IntersectionResult r = intersect(a, b, tol);
    REQUIRE(r.overlaps.size() == 1);
    CHECK_THAT(r.overlaps[0].tA0, WithinAbs(0.2, 1e-12));
    CHECK_THAT(r.overlaps[0].tA1, WithinAbs(0.8, 1e-12));
    CHECK_THAT(r.overlaps[0].tB0, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB1, WithinAbs(0.0, 1e-12));
}

TEST_CASE("Line-line: identical segments overlap fully", "[geometry][intersect]") {
    const LineSpan a{{1.0, 1.0}, {4.0, 5.0}};
    const IntersectionResult r = intersect(a, a, tol);
    REQUIRE(r.overlaps.size() == 1);
    CHECK_THAT(r.overlaps[0].tA0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
}

TEST_CASE("Line-line: collinear segments touching end to end give one tangent point",
          "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 0.0}};
    const LineSpan b{{10.0, 0.0}, {20.0, 0.0}};
    const IntersectionResult r = intersect(a, b, tol);
    CHECK(r.overlaps.empty());
    REQUIRE(r.points.size() == 1);
    CHECK(near(r.points[0].point, {10.0, 0.0}));
    CHECK(r.points[0].tangent);
}

TEST_CASE("Line-line: collinear disjoint segments do not intersect", "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 0.0}};
    const LineSpan b{{11.0, 0.0}, {20.0, 0.0}};
    CHECK(intersect(a, b, tol).isEmpty());
}

TEST_CASE("Degenerate (zero-length) spans behave as points", "[geometry][intersect]") {
    const Span point = LineSpan{{5.0, 0.0}, {5.0, 0.0}};
    const Span line = LineSpan{{0.0, 0.0}, {10.0, 0.0}};
    const Span arc = ArcSpan::fullCircle({0.0, 0.0}, 5.0);

    const IntersectionResult onLine = intersect(point, line, tol);
    REQUIRE(onLine.points.size() == 1);
    CHECK(near(onLine.points[0].point, {5.0, 0.0}));
    CHECK_THAT(onLine.points[0].tB, WithinAbs(0.5, 1e-12));

    const IntersectionResult onArc = intersect(arc, point, tol);
    REQUIRE(onArc.points.size() == 1);
    CHECK_THAT(onArc.points[0].tA, WithinAbs(0.0, 1e-12));

    const Span offPoint = LineSpan{{5.0, 1.0}, {5.0, 1.0}};
    CHECK(intersect(offPoint, line, tol).isEmpty());
    CHECK(intersect(point, point, tol).points.size() == 1);
    CHECK(intersect(point, offPoint, tol).isEmpty());
}

namespace {
// Asserts intersect(a,b) and intersect(b,a) describe the same contact set with parameters swapped.
void checkSymmetric(const Span& a, const Span& b, const Tolerance& t) {
    const IntersectionResult ab = intersect(a, b, t);
    const IntersectionResult ba = intersect(b, a, t);
    REQUIRE(ab.points.size() == ba.points.size());
    REQUIRE(ab.overlaps.size() == ba.overlaps.size());
    // Every reported point agrees with both spans at its parameters.
    for (const Intersection& p : ab.points) {
        CHECK(distance(p.point, pointAt(a, p.tA)) <= t.distance);
        CHECK(distance(p.point, pointAt(b, p.tB)) <= t.distance);
    }
    for (const Intersection& p : ab.points) {
        bool matched = false;
        for (const Intersection& q : ba.points) {
            if (approxEqual(p.point, q.point, 1e-9) && std::fabs(p.tA - q.tB) <= 1e-9 &&
                std::fabs(p.tB - q.tA) <= 1e-9 && p.tangent == q.tangent) {
                matched = true;
            }
        }
        CHECK(matched);
    }
    for (const Overlap& o : ab.overlaps) {
        // The same stretch seen from B: B's interval ascending, A's following it.
        Overlap e{o.tB0, o.tB1, o.tA0, o.tA1};
        if (e.tA0 > e.tA1) {
            std::swap(e.tA0, e.tA1);
            std::swap(e.tB0, e.tB1);
        }
        bool matched = false;
        for (const Overlap& q : ba.overlaps) {
            if (std::fabs(e.tA0 - q.tA0) <= 1e-9 && std::fabs(e.tA1 - q.tA1) <= 1e-9 &&
                std::fabs(e.tB0 - q.tB0) <= 1e-9 && std::fabs(e.tB1 - q.tB1) <= 1e-9) {
                matched = true;
            }
        }
        CHECK(matched);
    }
}

}  // namespace

TEST_CASE("Line-line: near-parallel crossing is found in both argument orders", "[geometry][intersect]") {
    // b crosses y = 0 near x = 666.7 and stays within ~1e-8 of a over a's extent.
    Tolerance fine;
    fine.distance = 1e-6;
    const LineSpan a{{660.0, 0.0}, {670.0, 0.0}};
    const LineSpan b{{-1000.0, -1.5e-6}, {1000.0, 3e-7}};
    const IntersectionResult ab = intersect(a, b, fine);
    const IntersectionResult ba = intersect(b, a, fine);
    CHECK_FALSE(ab.isEmpty());
    CHECK_FALSE(ba.isEmpty());
    REQUIRE(ab.overlaps.size() == 1);
    REQUIRE(ba.overlaps.size() == 1);
    CHECK_THAT(ab.overlaps[0].tA0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(ab.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
    CHECK_THAT(ba.overlaps[0].tB0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(ba.overlaps[0].tB1, WithinAbs(1.0, 1e-12));
    checkSymmetric(a, b, fine);

    // The same geometry at kilometre scale with the default tolerance.
    const LineSpan ak{{666660.0, 0.0}, {666670.0, 0.0}};
    const LineSpan bk{{-1e6, -1.5e-3}, {1e6, 3e-4}};
    CHECK_FALSE(intersect(ak, bk, tol).isEmpty());
    CHECK_FALSE(intersect(bk, ak, tol).isEmpty());
    REQUIRE(intersect(ak, bk, tol).overlaps.size() == 1);
    checkSymmetric(ak, bk, tol);
}

TEST_CASE("Line-line: near-parallel lines that are not collinear still cross", "[geometry][intersect]") {
    // Angle ~1e-10 rad: below the old "parallel" cut-off, but the shorter segment's ends are far
    // from the longer's line, so this is a genuine single crossing at (0, 0).
    Tolerance fine;
    fine.distance = 1e-9;
    const LineSpan a{{-1e3, -1e-7}, {1e3, 1e-7}};
    const LineSpan b{{-1e3, 1e-7}, {1e3, -1e-7}};
    for (const bool swap : {false, true}) {
        const IntersectionResult r = swap ? intersect(b, a, fine) : intersect(a, b, fine);
        CHECK(r.overlaps.empty());
        REQUIRE(r.points.size() == 1);
        CHECK(near(r.points[0].point, {0.0, 0.0}, 1e-9));
        CHECK_THAT(r.points[0].tA, WithinAbs(0.5, 1e-9));
        CHECK_THAT(r.points[0].tB, WithinAbs(0.5, 1e-9));
    }
    checkSymmetric(a, b, fine);
}

TEST_CASE("Line-line: end point within tolerance of the other segment is a contact",
          "[geometry][intersect]") {
    const LineSpan a{{0.0, 0.0}, {10.0, 0.0}};
    const LineSpan b{{5.0, 0.0005}, {5.0, 8.0}};  // stops 0.5 um above a
    const IntersectionResult r = intersect(a, b, tol);
    REQUIRE(r.points.size() == 1);
    CHECK_THAT(r.points[0].tA, WithinAbs(0.5, 1e-12));
    CHECK_THAT(r.points[0].tB, WithinAbs(0.0, 1e-12));
    checkSymmetric(a, b, tol);
    const LineSpan far{{5.0, 0.002}, {5.0, 8.0}};
    CHECK(intersect(a, far, tol).isEmpty());
}

TEST_CASE("Line-line: intersect(a,b) and intersect(b,a) agree for random pairs",
          "[geometry][intersect][property]") {
    SplitMix rng{0xC0FFEEULL};
    for (int i = 0; i < 2000; ++i) {
        const LineSpan a{{rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)},
                         {rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)}};
        LineSpan b;
        switch (i % 4) {
            case 0:  // fully random
                b = {{rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)},
                     {rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)}};
                break;
            case 1: {  // near-collinear with a: points on a's line, nudged by ~tolerance
                const Vec2 n = (a.end - a.start).normalized().perp();
                b = {a.pointAt(rng.uniform(-0.5, 1.5)) + n * rng.uniform(-2e-3, 2e-3),
                     a.pointAt(rng.uniform(-0.5, 1.5)) + n * rng.uniform(-2e-3, 2e-3)};
                break;
            }
            case 2: {  // nearly parallel, offset across a
                const Vec2 dir = (a.end - a.start).normalized();
                const Vec2 n = dir.perp();
                const Vec2 mid = a.pointAt(rng.uniform(0.0, 1.0));
                const double len = rng.uniform(1.0, 200.0);
                const double tilt = rng.uniform(-1e-6, 1e-6);
                b = {mid - (dir + n * tilt) * len, mid + (dir - n * tilt) * len};
                break;
            }
            default: {  // shares an end point with a
                b = {a.end, {rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)}};
                break;
            }
        }
        if (isDegenerate(Span{a}, tol) || isDegenerate(Span{b}, tol)) {
            continue;
        }
        INFO("pair " << i);
        checkSymmetric(a, b, tol);
    }
}

// -------------------------------------------------------------------- line / arc

TEST_CASE("Line-arc: secant through a circle gives two points", "[geometry][intersect]") {
    const LineSpan line{{-10.0, 0.0}, {10.0, 0.0}};
    const ArcSpan circle = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    const IntersectionResult r = intersect(line, circle, tol);
    REQUIRE(r.points.size() == 2);
    CHECK(hasPointNear(r, {-5.0, 0.0}));
    CHECK(hasPointNear(r, {5.0, 0.0}));
    CHECK(r.points[0].tA < r.points[1].tA);
    CHECK_FALSE(r.points[0].tangent);
    CHECK_FALSE(r.points[1].tangent);
}

TEST_CASE("Line-arc: only the hit inside the arc's sweep is reported", "[geometry][intersect]") {
    // Upper half circle from (5,0) through (0,5) to (-5,0).
    const ArcSpan upper = ArcSpan::fromCentre({0.0, 0.0}, {5.0, 0.0}, {-5.0, 0.0}, ArcDirection::CCW);
    const LineSpan vertical{{0.0, -10.0}, {0.0, 10.0}};
    const IntersectionResult r = intersect(vertical, upper, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(near(r.points[0].point, {0.0, 5.0}));
    CHECK_THAT(r.points[0].tA, WithinAbs(0.75, 1e-12));
    CHECK_THAT(r.points[0].tB, WithinAbs(0.5, 1e-12));
}

TEST_CASE("Line-arc: tangent contact returns one intersection with the tangent flag (AC2)",
          "[geometry][intersect]") {
    const LineSpan line{{-10.0, 5.0}, {10.0, 5.0}};
    const ArcSpan circle = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    const IntersectionResult r = intersect(line, circle, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(r.points[0].tangent);
    CHECK(near(r.points[0].point, {0.0, 5.0}));
    CHECK_THAT(r.points[0].tA, WithinAbs(0.5, 1e-12));
    CHECK_THAT(r.points[0].tB, WithinAbs(0.25, 1e-12));

    // Arc first, line second: parameters swap.
    const IntersectionResult rs = intersect(circle, line, tol);
    REQUIRE(rs.points.size() == 1);
    CHECK(rs.points[0].tangent);
    CHECK_THAT(rs.points[0].tA, WithinAbs(0.25, 1e-12));
    CHECK_THAT(rs.points[0].tB, WithinAbs(0.5, 1e-12));
}

TEST_CASE("Line-arc: a line that misses the circle by more than tolerance does not intersect",
          "[geometry][intersect]") {
    const LineSpan line{{-10.0, 5.01}, {10.0, 5.01}};
    const ArcSpan circle = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    CHECK(intersect(line, circle, tol).isEmpty());
}

TEST_CASE("Line-arc: a line ending just short of the circle still hits within tolerance",
          "[geometry][intersect]") {
    const LineSpan line{{0.0, 0.0}, {4.9995, 0.0}};
    const ArcSpan circle = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    const IntersectionResult r = intersect(line, circle, tol);
    REQUIRE(r.points.size() == 1);
    CHECK_THAT(r.points[0].tA, WithinAbs(1.0, 1e-12));
}

TEST_CASE("Line-arc: long lines through small circles stay precise", "[geometry][intersect]") {
    const ArcSpan unit = ArcSpan::fullCircle({0.0, 0.0}, 1.0);
    const LineSpan through{{-1e6, 0.0}, {1e6, 0.0}};
    const IntersectionResult r = intersect(through, unit, tol);
    REQUIRE(r.points.size() == 2);
    CHECK(near(r.points[0].point, {-1.0, 0.0}, 1e-9));
    CHECK(near(r.points[1].point, {1.0, 0.0}, 1e-9));

    const LineSpan offset{{-1e6, 0.5}, {1e6, 0.5}};
    const IntersectionResult ro = intersect(offset, unit, tol);
    REQUIRE(ro.points.size() == 2);
    CHECK(near(ro.points[0].point, {-std::sqrt(0.75), 0.5}, 1e-9));
    CHECK(near(ro.points[1].point, {std::sqrt(0.75), 0.5}, 1e-9));
    CHECK_THAT(ro.points[1].tA, WithinAbs(0.5 + std::sqrt(0.75) / 2e6, 1e-12));
    checkSymmetric(offset, unit, tol);
}

// --------------------------------------------------------------------- arc / arc

TEST_CASE("Arc-arc: two overlapping circles meet in two points", "[geometry][intersect]") {
    const ArcSpan a = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    const ArcSpan b = ArcSpan::fullCircle({6.0, 0.0}, 5.0);
    const IntersectionResult r = intersect(a, b, tol);
    REQUIRE(r.points.size() == 2);
    CHECK(hasPointNear(r, {3.0, 4.0}));
    CHECK(hasPointNear(r, {3.0, -4.0}));
    CHECK_FALSE(r.points[0].tangent);
}

TEST_CASE("Arc-arc: externally and internally tangent circles give one tangent point",
          "[geometry][intersect]") {
    const ArcSpan a = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    const ArcSpan external = ArcSpan::fullCircle({8.0, 0.0}, 3.0);
    const IntersectionResult re = intersect(a, external, tol);
    REQUIRE(re.points.size() == 1);
    CHECK(re.points[0].tangent);
    CHECK(near(re.points[0].point, {5.0, 0.0}));

    const ArcSpan internal = ArcSpan::fullCircle({2.0, 0.0}, 3.0);
    const IntersectionResult ri = intersect(a, internal, tol);
    REQUIRE(ri.points.size() == 1);
    CHECK(ri.points[0].tangent);
    CHECK(near(ri.points[0].point, {5.0, 0.0}));
}

TEST_CASE("Arc-arc: separated or nested circles do not intersect", "[geometry][intersect]") {
    const ArcSpan a = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    CHECK(intersect(a, ArcSpan::fullCircle({20.0, 0.0}, 5.0), tol).isEmpty());
    CHECK(intersect(a, ArcSpan::fullCircle({1.0, 0.0}, 1.0), tol).isEmpty());
    // Concentric, different radii.
    CHECK(intersect(a, ArcSpan::fullCircle({0.0, 0.0}, 3.0), tol).isEmpty());
}

TEST_CASE("Arc-arc: sweep filtering keeps only hits on both arcs", "[geometry][intersect]") {
    const ArcSpan a = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    // Upper half of the second circle only: contains (3,4) but not (3,-4).
    const ArcSpan upper = ArcSpan::fromCentre({6.0, 0.0}, {11.0, 0.0}, {1.0, 0.0}, ArcDirection::CCW);
    const IntersectionResult r = intersect(a, upper, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(near(r.points[0].point, {3.0, 4.0}));
    CHECK_THAT(r.points[0].tA, WithinAbs(std::atan2(4.0, 3.0) / kTwoPi, 1e-12));
    CHECK_THAT(r.points[0].tB, WithinAbs(std::atan2(4.0, -3.0) / kPi, 1e-12));
}

TEST_CASE("Arc-arc: coincident arcs on the same circle report overlaps", "[geometry][intersect]") {
    const Vec2 c{0.0, 0.0};
    // A: 0 -> 180 degrees CCW. B: 90 -> 270 degrees CCW. Overlap 90..180 = A's [0.5, 1].
    const ArcSpan a{c, 5.0, 0.0, kPi, ArcDirection::CCW};
    const ArcSpan b{c, 5.0, kPi * 0.5, kPi * 1.5, ArcDirection::CCW};
    const IntersectionResult r = intersect(a, b, tol);
    CHECK(r.points.empty());
    REQUIRE(r.overlaps.size() == 1);
    CHECK_THAT(r.overlaps[0].tA0, WithinAbs(0.5, 1e-12));
    CHECK_THAT(r.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB1, WithinAbs(0.5, 1e-12));
}

TEST_CASE("Arc-arc: coincident arcs running opposite ways keep B parameters in B's order",
          "[geometry][intersect]") {
    const Vec2 c{0.0, 0.0};
    const ArcSpan a{c, 5.0, 0.0, kPi, ArcDirection::CCW};
    const ArcSpan b{c, 5.0, kPi * 1.5, kPi * 0.5, ArcDirection::CW};  // 270 -> 90 clockwise
    const IntersectionResult r = intersect(a, b, tol);
    REQUIRE(r.overlaps.size() == 1);
    CHECK_THAT(r.overlaps[0].tA0, WithinAbs(0.5, 1e-12));
    CHECK_THAT(r.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB0, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB1, WithinAbs(0.5, 1e-12));
}

TEST_CASE("Arc-arc: coincident arcs that only touch end to end give one tangent point",
          "[geometry][intersect]") {
    const Vec2 c{0.0, 0.0};
    const ArcSpan a{c, 5.0, 0.0, kPi, ArcDirection::CCW};
    const ArcSpan b{c, 5.0, kPi, kPi * 1.5, ArcDirection::CCW};
    const IntersectionResult r = intersect(a, b, tol);
    CHECK(r.overlaps.empty());
    REQUIRE(r.points.size() == 1);
    CHECK(r.points[0].tangent);
    CHECK(near(r.points[0].point, {-5.0, 0.0}));
}

TEST_CASE("Arc-arc: identical full circles overlap everywhere", "[geometry][intersect]") {
    const ArcSpan a = ArcSpan::fullCircle({1.0, 2.0}, 5.0);
    const IntersectionResult r = intersect(a, a, tol);
    CHECK(r.points.empty());
    REQUIRE(!r.overlaps.empty());
    REQUIRE(r.overlaps.size() == 1);
    CHECK_THAT(r.overlaps[0].tA0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB1, WithinAbs(1.0, 1e-12));

    // Same circle traced the other way: B runs 1 -> 0 over A's 0 -> 1.
    const ArcSpan cw = ArcSpan::fullCircle({1.0, 2.0}, 5.0, ArcDirection::CW);
    const IntersectionResult rc = intersect(a, cw, tol);
    CHECK(rc.points.empty());
    REQUIRE(rc.overlaps.size() == 1);
    CHECK_THAT(rc.overlaps[0].tA0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(rc.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
    CHECK_THAT(rc.overlaps[0].tB0, WithinAbs(1.0, 1e-12));
    CHECK_THAT(rc.overlaps[0].tB1, WithinAbs(0.0, 1e-12));
}

TEST_CASE("Arc-arc: full circle against a partial arc on it", "[geometry][intersect]") {
    const ArcSpan circle = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    const ArcSpan quarter{{0.0, 0.0}, 5.0, kPi * 0.5, kPi, ArcDirection::CCW};
    const IntersectionResult r = intersect(quarter, circle, tol);
    REQUIRE(r.overlaps.size() == 1);
    CHECK_THAT(r.overlaps[0].tA0, WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tA1, WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.overlaps[0].tB0, WithinAbs(0.25, 1e-12));
    CHECK_THAT(r.overlaps[0].tB1, WithinAbs(0.5, 1e-12));
}

// ------------------------------------------------------------------- Bezier cases

TEST_CASE("Bezier-line: crossing is refined to within tolerance", "[geometry][intersect]") {
    const BezierSpan bz{{0.0, 0.0}, {3.0, 10.0}, {7.0, -10.0}, {10.0, 0.0}};  // S-curve, symmetric
    const LineSpan line{{5.0, -5.0}, {5.0, 5.0}};
    const IntersectionResult r = intersect(Span{bz}, Span{line}, tol);
    REQUIRE(r.points.size() == 1);
    // By symmetry the curve passes through (5,0) at t = 0.5.
    CHECK(near(r.points[0].point, {5.0, 0.0}, 1e-9));
    CHECK_THAT(r.points[0].tA, WithinAbs(0.5, 1e-9));
    CHECK_THAT(r.points[0].tB, WithinAbs(0.5, 1e-9));
    CHECK_FALSE(r.points[0].tangent);
}

TEST_CASE("Bezier-line: an S-curve crossing its chord three times", "[geometry][intersect]") {
    const BezierSpan bz{{0.0, 0.0}, {3.0, 10.0}, {7.0, -10.0}, {10.0, 0.0}};
    const LineSpan chord{{-1.0, 0.0}, {11.0, 0.0}};
    const IntersectionResult r = intersect(Span{bz}, Span{chord}, tol);
    REQUIRE(r.points.size() == 3);
    CHECK(near(r.points[0].point, {0.0, 0.0}));
    CHECK(near(r.points[1].point, {5.0, 0.0}));
    CHECK(near(r.points[2].point, {10.0, 0.0}));
}

TEST_CASE("Bezier-arc: a Bezier quarter circle meets a true circle along the whole piece or at ends",
          "[geometry][intersect]") {
    // The Bezier approximation of a quarter circle (r=10) deviates by ~2.7e-3 mm from the true arc,
    // which is above the default tolerance, so only the exact end points coincide.
    const ArcSpan quarter{{0.0, 0.0}, 10.0, 0.0, kPi * 0.5, ArcDirection::CCW};
    const BezierSpan bz = quarter.toBeziers().front();
    const ArcSpan circle = ArcSpan::fullCircle({0.0, 0.0}, 10.0);
    const IntersectionResult r = intersect(Span{bz}, Span{circle}, tol);
    CHECK(hasPointNear(r, {10.0, 0.0}, 1e-6));
    CHECK(hasPointNear(r, {0.0, 10.0}, 1e-6));
    for (const Intersection& p : r.points) {
        CHECK_THAT(distance(bz.pointAt(p.tA), circle.pointAt(p.tB)), WithinAbs(0.0, tol.distance));
    }
}

TEST_CASE("Bezier-Bezier: two crossing curves", "[geometry][intersect]") {
    const BezierSpan a{{0.0, 0.0}, {3.0, 4.0}, {7.0, 4.0}, {10.0, 0.0}};
    const BezierSpan b{{0.0, 3.0}, {3.0, -1.0}, {7.0, -1.0}, {10.0, 3.0}};
    const IntersectionResult r = intersect(Span{a}, Span{b}, tol);
    REQUIRE(r.points.size() == 2);
    for (const Intersection& p : r.points) {
        CHECK_THAT(distance(a.pointAt(p.tA), b.pointAt(p.tB)), WithinAbs(0.0, 1e-9));
        CHECK_THAT(distance(a.pointAt(p.tA), p.point), WithinAbs(0.0, 1e-9));
    }
    // Mirror symmetry about x = 5.
    CHECK_THAT(r.points[0].point.x + r.points[1].point.x, WithinAbs(10.0, 1e-9));
}

TEST_CASE("Bezier-line: tangent contact is found and flagged", "[geometry][intersect]") {
    // Symmetric arch with apex at (5, 3): y(0.5) = 0.75*4 = 3.
    const BezierSpan arch{{0.0, 0.0}, {3.0, 4.0}, {7.0, 4.0}, {10.0, 0.0}};
    const LineSpan line{{-5.0, 3.0}, {15.0, 3.0}};
    const IntersectionResult r = intersect(Span{arch}, Span{line}, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(r.points[0].tangent);
    CHECK(near(r.points[0].point, {5.0, 3.0}, 1e-6));
    CHECK_THAT(r.points[0].tA, WithinAbs(0.5, 1e-4));

    // A line just above the apex (beyond tolerance) misses.
    const LineSpan miss{{-5.0, 3.01}, {15.0, 3.01}};
    CHECK(intersect(Span{arch}, Span{miss}, tol).isEmpty());
}

TEST_CASE("Bezier: coincident spans are reported as sampled tangent points, not overlaps",
          "[geometry][intersect]") {
    // Documented limitation (ADR-0009): overlap detection for Beziers is future work.
    const BezierSpan bz{{0.0, 0.0}, {3.0, 4.0}, {7.0, 4.0}, {10.0, 0.0}};
    const IntersectionResult r = intersect(Span{bz}, Span{bz}, tol);
    CHECK(r.overlaps.empty());
    REQUIRE_FALSE(r.points.empty());
    for (const Intersection& p : r.points) {
        CHECK(p.tangent);
        CHECK_THAT(p.tA, WithinAbs(p.tB, 1e-6));
    }
}

TEST_CASE("Bezier-line: no false positives for a nearby but separate line", "[geometry][intersect]") {
    const BezierSpan arch{{0.0, 0.0}, {3.0, 4.0}, {7.0, 4.0}, {10.0, 0.0}};
    const LineSpan line{{0.0, -0.5}, {10.0, -0.5}};
    CHECK(intersect(Span{arch}, Span{line}, tol).isEmpty());
}

// ----------------------------------------------------------- contour intersection

TEST_CASE("Two unit squares offset by (0.5,0.5) intersect in exactly 2 points (AC1)",
          "[geometry][intersect]") {
    const Contour a = squareAt({0.0, 0.0}, 1.0);
    const Contour b = squareAt({0.5, 0.5}, 1.0);
    const ContourIntersections r = intersect(a, b, tol);
    REQUIRE(r.points.size() == 2);
    CHECK(r.overlaps.empty());
    CHECK(hasPointNear(r, {1.0, 0.5}, 1e-9));
    CHECK(hasPointNear(r, {0.5, 1.0}, 1e-9));
    for (const ContourIntersection& hit : r.points) {
        CHECK_FALSE(hit.hit.tangent);
        CHECK(hit.spanA < 4);
        CHECK(hit.spanB < 4);
    }
}

TEST_CASE("Contour intersection: hits at a shared vertex are reported once", "[geometry][intersect]") {
    // Square B's left edge passes exactly through A's corner (1,1)... use a diagonal line contour.
    const Contour a = squareAt({0.0, 0.0}, 1.0);
    Contour diag;
    diag.append(LineSpan{{0.5, 1.5}, {1.5, 0.5}});  // through corner (1,1)
    const ContourIntersections r = intersect(a, diag, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(near(r.points[0].hit.point, {1.0, 1.0}));
}

TEST_CASE("Contour intersection: circle through a square", "[geometry][intersect]") {
    const Contour square = squareAt({-5.0, -5.0}, 10.0);
    const Contour circle = Contour::circle({0.0, 0.0}, 6.0);
    const ContourIntersections r = intersect(square, circle, tol);
    REQUIRE(r.points.size() == 8);
    const double off = std::sqrt(36.0 - 25.0);
    CHECK(hasPointNear(r, {5.0, off}));
    CHECK(hasPointNear(r, {-5.0, -off}));
    CHECK(hasPointNear(r, {off, 5.0}));
}

TEST_CASE("Contour intersection: shared edge yields an overlap", "[geometry][intersect]") {
    const Contour a = squareAt({0.0, 0.0}, 1.0);
    const Contour b = squareAt({1.0, 0.0}, 1.0);
    const ContourIntersections r = intersect(a, b, tol);
    REQUIRE(r.overlaps.size() == 1);
    CHECK(r.overlaps[0].spanA == 1);  // A's right edge
    CHECK(r.overlaps[0].spanB == 3);  // B's left edge
    // The corner touches at (1,0) and (1,1) are end points of the overlapping edge and are
    // reported by the neighbouring edges only.
    for (const ContourIntersection& p : r.points) {
        CHECK((near(p.hit.point, {1.0, 0.0}) || near(p.hit.point, {1.0, 1.0})));
    }
}

TEST_CASE("Disjoint contours have no intersections", "[geometry][intersect]") {
    const Contour a = squareAt({0.0, 0.0}, 1.0);
    const Contour b = squareAt({5.0, 5.0}, 1.0);
    CHECK(intersect(a, b, tol).isEmpty());
}

// ------------------------------------------------------------- self intersection

TEST_CASE("A figure-eight reports self-intersection; a square does not (AC5)", "[geometry][intersect]") {
    Contour eight;
    eight.closed = true;
    eight.append(LineSpan{{0.0, 0.0}, {10.0, 10.0}});
    eight.append(LineSpan{{10.0, 10.0}, {10.0, 0.0}});
    eight.append(LineSpan{{10.0, 0.0}, {0.0, 10.0}});
    eight.append(LineSpan{{0.0, 10.0}, {0.0, 0.0}});
    CHECK(hasSelfIntersections(eight, tol));
    const ContourIntersections r = selfIntersections(eight, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(near(r.points[0].hit.point, {5.0, 5.0}));
    CHECK(r.points[0].spanA == 0);
    CHECK(r.points[0].spanB == 2);

    const Contour square = squareAt({0.0, 0.0}, 10.0);
    CHECK_FALSE(hasSelfIntersections(square, tol));
    CHECK(selfIntersections(square, tol).isEmpty());
}

TEST_CASE("Self-intersection ignores junctions but catches fold-backs and open-contour crossings",
          "[geometry][intersect]") {
    // Mixed shape with lines, an arc and a Bezier: continuous and simple.
    Contour simple;
    simple.closed = true;
    simple.append(LineSpan{{0.0, 0.0}, {20.0, 0.0}});
    simple.append(ArcSpan::fromCentre({20.0, 10.0}, {20.0, 0.0}, {20.0, 20.0}, ArcDirection::CCW));
    simple.append(BezierSpan{{20.0, 20.0}, {15.0, 30.0}, {5.0, 30.0}, {0.0, 20.0}});
    simple.append(LineSpan{{0.0, 20.0}, {0.0, 0.0}});
    CHECK_FALSE(hasSelfIntersections(simple, tol));
    CHECK_FALSE(hasSelfIntersections(Contour::circle({0.0, 0.0}, 4.0), tol));

    // Open zig-zag whose last segment crosses the first.
    Contour zig;
    zig.append(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    zig.append(LineSpan{{10.0, 0.0}, {10.0, 5.0}});
    zig.append(LineSpan{{10.0, 5.0}, {5.0, -5.0}});
    CHECK(hasSelfIntersections(zig, tol));
    const ContourIntersections rz = selfIntersections(zig, tol);
    REQUIRE(rz.points.size() == 1);
    CHECK(near(rz.points[0].hit.point, {7.5, 0.0}));

    // Fold-back: a collinear span retracing the previous one is an overlap, not a junction.
    Contour fold;
    fold.append(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    fold.append(LineSpan{{10.0, 0.0}, {4.0, 0.0}});
    const ContourIntersections rf = selfIntersections(fold, tol);
    REQUIRE(rf.overlaps.size() == 1);
    CHECK(hasSelfIntersections(fold, tol));

    // Collinear continuation is fine.
    Contour straight;
    straight.append(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    straight.append(LineSpan{{10.0, 0.0}, {20.0, 0.0}});
    CHECK_FALSE(hasSelfIntersections(straight, tol));
}

TEST_CASE("Self-intersection: an open contour whose ends meet is not self-intersecting",
          "[geometry][intersect]") {
    Contour c = squareAt({0.0, 0.0}, 10.0);
    c.closed = false;  // same four spans, first and last still share the (0,0) vertex
    // Open contours never close the loop, so first/last touching at (0,0) is a genuine contact
    // of non-consecutive spans.
    const ContourIntersections r = selfIntersections(c, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(near(r.points[0].hit.point, {0.0, 0.0}));
    c.closed = true;
    CHECK(selfIntersections(c, tol).isEmpty());
}

TEST_CASE("Self-intersection: a looping cubic Bezier crosses itself", "[geometry][intersect]") {
    Contour c;
    c.append(BezierSpan{{0.0, 0.0}, {30.0, 10.0}, {-20.0, 10.0}, {10.0, 0.0}});
    CHECK(hasSelfIntersections(c, tol));
    const ContourIntersections r = selfIntersections(c, tol);
    REQUIRE(r.points.size() == 1);
    CHECK(r.points[0].spanA == 0);
    CHECK(r.points[0].spanB == 0);
    const BezierSpan& bz = std::get<BezierSpan>(c.spans[0]);
    CHECK_THAT(distance(bz.pointAt(r.points[0].hit.tA), bz.pointAt(r.points[0].hit.tB)),
               WithinAbs(0.0, 1e-9));
    CHECK(std::fabs(r.points[0].hit.tA - r.points[0].hit.tB) > 0.1);
    // By symmetry the crossing lies on x = 5.
    CHECK_THAT(r.points[0].hit.point.x, WithinAbs(5.0, 1e-9));

    Contour plain;
    plain.append(BezierSpan{{0.0, 0.0}, {3.0, 4.0}, {7.0, 4.0}, {10.0, 0.0}});
    CHECK_FALSE(hasSelfIntersections(plain, tol));
}

// ------------------------------------------------- round-3 regressions (generative)

namespace {
// Checks every reported point of intersect(a, b) against both arcs: it must agree with A(tA) and
// B(tB) and lie on both circles, all within tolerance. Returns the number of points.
std::size_t checkArcPoints(const ArcSpan& a, const ArcSpan& b, const Tolerance& t) {
    const IntersectionResult r = intersect(a, b, t);
    for (const Intersection& p : r.points) {
        INFO("P=(" << p.point.x << ", " << p.point.y << ") tA=" << p.tA << " tB=" << p.tB);
        REQUIRE(distance(p.point, a.pointAt(p.tA)) <= t.distance);
        REQUIRE(distance(p.point, b.pointAt(p.tB)) <= t.distance);
        REQUIRE(std::fabs(distance(p.point, a.centre) - a.radius) <= t.distance);
        REQUIRE(std::fabs(distance(p.point, b.centre) - b.radius) <= t.distance);
    }
    return r.points.size();
}
}  // namespace

TEST_CASE("Arc-arc: near-tangent internal circles with close centres (verifier repro)",
          "[geometry][intersect]") {
    const ArcSpan a = ArcSpan::fullCircle({0.0, 0.0}, 5.0);
    const ArcSpan b = ArcSpan::fullCircle({0.01, 0.0}, 4.9895);
    for (const bool swapArgs : {false, true}) {
        const IntersectionResult r = swapArgs ? intersect(b, a, tol) : intersect(a, b, tol);
        REQUIRE(r.points.size() == 1);
        CHECK(r.points[0].tangent);
        CHECK(near(r.points[0].point, {5.0, 0.0}, tol.distance));
    }
    CHECK(checkArcPoints(a, b, tol) == 1);
    CHECK(checkArcPoints(b, a, tol) == 1);
    // A inside B, touching on the far side.
    const ArcSpan big = ArcSpan::fullCircle({0.01, 0.0}, 5.0105);
    const IntersectionResult rb = intersect(a, big, tol);
    REQUIRE(rb.points.size() == 1);
    CHECK(near(rb.points[0].point, {-5.0, 0.0}, tol.distance));
}

TEST_CASE("Arc-arc: nearly concentric circles meet only when their closest approach is within tolerance",
          "[geometry][intersect]") {
    // Closest approach of nested circles is |r1 - r2| - d (T-0005 AC2: no concentric special case).
    const ArcSpan a = ArcSpan::fullCircle({1.0, 1.0}, 5.0);
    for (const double dr : {0.0011, 0.01, 1.0}) {
        for (const double d : {0.0, 1e-9, 5e-4}) {
            const ArcSpan b = ArcSpan::fullCircle({1.0 + d, 1.0}, 5.0 + dr);
            const bool touches = dr - d <= tol.distance;
            INFO("dr=" << dr << " d=" << d);
            CHECK(intersect(a, b, tol).points.size() == (touches ? 1u : 0u));
            CHECK(intersect(b, a, tol).points.size() == (touches ? 1u : 0u));
            CHECK(intersect(a, b, tol).overlaps.empty());
        }
    }
}

TEST_CASE("Arc-arc: generative near-tangent internal and external circle pairs",
          "[geometry][intersect][property]") {
    SplitMix rng{0xA2CA2CULL};
    std::size_t tested = 0;
    for (int i = 0; i < 30000; ++i) {
        const double d = rng.logUniform(1e-4, 1e3);
        const double gap = rng.uniform(-0.9, 0.9) * tol.distance;
        const Vec2 ca{rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0)};
        const Vec2 cb = ca + Vec2::fromAngle(rng.uniform(0.0, kTwoPi), d);
        double r1 = 0.0;
        double r2 = 0.0;
        switch (i % 3) {
            case 0:  // external: r1 + r2 = d - gap
                r1 = (d - gap) * rng.uniform(0.01, 0.99);
                r2 = d - gap - r1;
                break;
            case 1:  // B inside A: r1 - r2 = d + gap
                r1 = rng.logUniform(std::max(1e-3, d + gap + 1e-3), 1e4);
                r2 = r1 - d - gap;
                break;
            default:  // A inside B: r2 - r1 = d + gap
                r2 = rng.logUniform(std::max(1e-3, d + gap + 1e-3), 1e4);
                r1 = r2 - d - gap;
                break;
        }
        if (r1 < 1e-3 || r2 < 1e-3 || r1 > 1e4 || r2 > 1e4) {
            continue;
        }
        const ArcSpan a{ca, r1, rng.uniform(-kPi, kPi), 0.0, ArcDirection::CCW};
        ArcSpan full = a;
        full.endAngle = full.startAngle;  // full circle, random seam
        const ArcSpan b{cb, r2, rng.uniform(-kPi, kPi), 0.0,
                        (i % 2) != 0 ? ArcDirection::CW : ArcDirection::CCW};
        ArcSpan fullB = b;
        fullB.endAngle = fullB.startAngle;
        INFO("case " << i << " d=" << d << " r1=" << r1 << " r2=" << r2 << " gap=" << gap);
        const std::size_t nab = checkArcPoints(full, fullB, tol);
        const std::size_t nba = checkArcPoints(fullB, full, tol);
        CHECK(nab == nba);
        if (d > tol.distance) {
            // The circles are within tolerance of touching: one tangent contact, unless they cross
            // at two points further apart than the tolerance (T-0008 AC3), which are both reported.
            // The common chord from the triangle of the centres and radii (Kahan's stable Heron).
            std::array<double, 3> s{d, r1, r2};
            std::sort(s.begin(), s.end(), std::greater<>());
            const double area4 =
                std::sqrt(std::max(0.0, (s[0] + (s[1] + s[2])) * (s[2] - (s[0] - s[1])) *
                                            (s[2] + (s[0] - s[1])) * (s[0] + (s[1] - s[2]))));
            const double chord = gap < 0.0 ? area4 / d : 0.0;  // 2 * (2 * area / d)
            if (chord < 0.99 * tol.distance) {
                CHECK(nab == 1);
            } else if (chord > 1.01 * tol.distance) {
                CHECK(nab == 2);
            }
        }
        ++tested;
    }
    CHECK(tested > 15000);
}

TEST_CASE("Arc-arc: generative random arcs report points on both arcs", "[geometry][intersect][property]") {
    SplitMix rng{0xBADA2CULL};
    for (int i = 0; i < 20000; ++i) {
        const ArcSpan a{{rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)},
                        rng.logUniform(1e-2, 50.0),
                        rng.uniform(-kPi, kPi),
                        rng.uniform(-kPi, kPi),
                        (i % 2) != 0 ? ArcDirection::CW : ArcDirection::CCW};
        const ArcSpan b{{rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)},
                        rng.logUniform(1e-2, 50.0),
                        rng.uniform(-kPi, kPi),
                        rng.uniform(-kPi, kPi),
                        (i % 3) != 0 ? ArcDirection::CW : ArcDirection::CCW};
        INFO("case " << i);
        CHECK(checkArcPoints(a, b, tol) == checkArcPoints(b, a, tol));
        checkSymmetric(a, b, tol);
    }
}

TEST_CASE("Line-line: reported point agrees with A(tA) for ill-conditioned crossings at large coordinates",
          "[geometry][intersect][property]") {
    SplitMix rng{0x11C0DEULL};
    for (int i = 0; i < 5000; ++i) {
        const Vec2 centre{rng.uniform(-1e6, 1e6), rng.uniform(-1e6, 1e6)};
        const Vec2 dir = Vec2::fromAngle(rng.uniform(0.0, kTwoPi));
        const double tilt = rng.logUniform(1e-12, 1e-4);
        const double len = rng.logUniform(1.0, 1e4);
        const Vec2 n = dir.perp();
        const LineSpan a{centre - dir * len, centre + dir * len};
        const LineSpan b{centre - (dir + n * tilt) * (len * rng.uniform(0.2, 1.0)),
                         centre + (dir + n * tilt) * (len * rng.uniform(0.2, 1.0))};
        INFO("case " << i);
        checkSymmetric(a, b, tol);
        CHECK_FALSE(intersect(a, b, tol).isEmpty());
    }
}
