#include "carvetto/toolpath/move.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "carvetto/geometry/vec3.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::kPi;
using carvetto::geometry::kTwoPi;
using Catch::Matchers::WithinAbs;

TEST_CASE("Vec3 minimal arithmetic", "[toolpath][vec3]") {
    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{4.0, 5.0, 6.0};
    CHECK(a + b == Vec3{5.0, 7.0, 9.0});
    CHECK(b - a == Vec3{3.0, 3.0, 3.0});
    CHECK(a * 2.0 == Vec3{2.0, 4.0, 6.0});
    CHECK(2.0 * a == a * 2.0);
    CHECK(a.dot(b) == 32.0);
    CHECK((Vec3{1, 0, 0}.cross(Vec3{0, 1, 0}) == Vec3{0, 0, 1}));
    CHECK(a.xy() == Vec2{1.0, 2.0});
    CHECK((Vec3{Vec2{7.0, 8.0}, -1.0} == Vec3{7.0, 8.0, -1.0}));
    CHECK_THAT((distance(Vec3{}, Vec3{3, 4, 12})), WithinAbs(13.0, 1e-12));
    CHECK(Vec3{}.normalized() == Vec3{});
    CHECK_THAT((Vec3{0, 0, 5}.normalized().z), WithinAbs(1.0, 1e-12));
    CHECK((carvetto::geometry::approxEqual(lerp(a, b, 0.5), Vec3{2.5, 3.5, 4.5})));
}

TEST_CASE("Move factories set kind and payload", "[toolpath][move]") {
    const Vec3 p{1.0, 2.0, 3.0};
    CHECK(Move::rapid(p).kind == MoveKind::Rapid);
    CHECK(Move::rapid(p).to == p);
    CHECK_FALSE(Move::rapid(p).feedOverride.has_value());
    CHECK(Move::feed(p, 800.0).feedOverride == 800.0);
    CHECK(Move::plunge(p).kind == MoveKind::Plunge);
    CHECK(Move::retract(p).kind == MoveKind::Retract);
    CHECK(Move::arcCW(p, Vec2{0, 0}).kind == MoveKind::ArcCW);
    CHECK(Move::arcCCW(p, Vec2{5, 5}).centre == Vec2{5, 5});
    const Move d = Move::dwell(p, 2.5);
    CHECK(d.kind == MoveKind::Dwell);
    CHECK(d.dwellSeconds == 2.5);
    CHECK(d.to == p);

    CHECK(isCutting(MoveKind::Feed));
    CHECK(isCutting(MoveKind::Plunge));
    CHECK(isCutting(MoveKind::Retract));
    CHECK(isCutting(MoveKind::ArcCW));
    CHECK_FALSE(isCutting(MoveKind::Rapid));
    CHECK_FALSE(isCutting(MoveKind::Dwell));
    CHECK(isArc(MoveKind::ArcCCW));
    CHECK_FALSE(isArc(MoveKind::Feed));
}

TEST_CASE("Linear move lengths", "[toolpath][move]") {
    CHECK_THAT(moveLength(Move::feed({3, 4, 0}), Vec3{}), WithinAbs(5.0, 1e-12));
    CHECK_THAT(moveLength(Move::plunge({0, 0, -3}), Vec3{0, 0, 5}), WithinAbs(8.0, 1e-12));
    CHECK_THAT(moveLength(Move::rapid({1, 1, 1}), Vec3{1, 1, 1}), WithinAbs(0.0, 0.0));
    CHECK(moveLength(Move::dwell({1, 1, 1}, 10.0), Vec3{1, 1, 1}) == 0.0);
    CHECK(moveLength(Move::dwell({1, 1, 1}, 10.0), Vec3{}) == 0.0);
}

TEST_CASE("Arc move sweep, radius and length", "[toolpath][move]") {
    // CCW quarter from (10,0) to (0,10) about origin.
    const Move q = Move::arcCCW({0, 10, 0}, Vec2{0, 0});
    CHECK_THAT(arcRadius(q, Vec2{10, 0}), WithinAbs(10.0, 1e-12));
    CHECK_THAT(arcSweep(q, Vec2{10, 0}), WithinAbs(kPi / 2, 1e-12));
    CHECK_THAT(moveLength(q, Vec3{10, 0, 0}), WithinAbs(5.0 * kPi, 1e-12));

    // The same end points clockwise is the three-quarter arc.
    const Move q3 = Move::arcCW({0, 10, 0}, Vec2{0, 0});
    CHECK_THAT(arcSweep(q3, Vec2{10, 0}), WithinAbs(-1.5 * kPi, 1e-12));
    CHECK_THAT(moveLength(q3, Vec3{10, 0, 0}), WithinAbs(15.0 * kPi, 1e-12));

    SECTION("coincident start and end is a full circle") {
        const Move full = Move::arcCW({10, 0, 0}, Vec2{0, 0});
        CHECK_THAT(arcSweep(full, Vec2{10, 0}), WithinAbs(-kTwoPi, 1e-12));
        CHECK_THAT(moveLength(full, Vec3{10, 0, 0}), WithinAbs(20.0 * kPi, 1e-12));
    }
    SECTION("helical arc adds the Z travel in quadrature") {
        const Move h = Move::arcCCW({10, 0, -4}, Vec2{0, 0});
        const double planar = 20.0 * kPi;
        CHECK_THAT(moveLength(h, Vec3{10, 0, -1}), WithinAbs(std::sqrt(planar * planar + 9.0), 1e-12));
    }
    SECTION("non-arc moves have zero sweep") {
        CHECK(arcSweep(Move::feed({1, 1, 1}), Vec2{}) == 0.0);
    }
}

TEST_CASE("Move bounds", "[toolpath][move]") {
    const Box2 line = moveBounds(Move::feed({10, -2, 0}), Vec3{0, 3, 0});
    CHECK(line.min == Vec2{0, -2});
    CHECK(line.max == Vec2{10, 3});

    // CCW half circle from (10,0) to (-10,0) bulges to y = 10.
    const Box2 arc = moveBounds(Move::arcCCW({-10, 0, 0}, Vec2{0, 0}), Vec3{10, 0, 0});
    CHECK_THAT(arc.max.y, WithinAbs(10.0, 1e-9));
    CHECK_THAT(arc.min.y, WithinAbs(0.0, 1e-9));
    CHECK_THAT(arc.min.x, WithinAbs(-10.0, 1e-9));
    CHECK_THAT(arc.max.x, WithinAbs(10.0, 1e-9));

    const Box2 dwell = moveBounds(Move::dwell({2, 2, 0}, 1.0), Vec3{2, 2, 0});
    CHECK(dwell.min == Vec2{2, 2});
    CHECK(dwell.max == Vec2{2, 2});
}
