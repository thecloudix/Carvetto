#include "carvetto/geometry/box2.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using Catch::Matchers::WithinAbs;

TEST_CASE("Box2 starts empty and absorbs points", "[geometry][box2]") {
    Box2 b;
    CHECK(b.isEmpty());
    CHECK(b.width() == 0.0);
    CHECK(b.height() == 0.0);
    b.expand({3.0, -1.0});
    CHECK_FALSE(b.isEmpty());
    CHECK(b.min == Vec2{3.0, -1.0});
    CHECK(b.max == Vec2{3.0, -1.0});
    b.expand({-2.0, 4.0});
    CHECK(b.min == Vec2{-2.0, -1.0});
    CHECK(b.max == Vec2{3.0, 4.0});
    CHECK_THAT(b.width(), WithinAbs(5.0, 1e-12));
    CHECK_THAT(b.height(), WithinAbs(5.0, 1e-12));
    CHECK(b.centre() == Vec2{0.5, 1.5});
    CHECK_THAT(b.area(), WithinAbs(25.0, 1e-12));
}

TEST_CASE("Box2 corner constructor orders corners", "[geometry][box2]") {
    const Box2 b{{5.0, 1.0}, {-1.0, 7.0}};
    CHECK(b.min == Vec2{-1.0, 1.0});
    CHECK(b.max == Vec2{5.0, 7.0});
    const Box2 c = Box2::fromCentre({0.0, 0.0}, 4.0, 2.0);
    CHECK(c.min == Vec2{-2.0, -1.0});
    CHECK(c.max == Vec2{2.0, 1.0});
}

TEST_CASE("Box2 containment and intersection", "[geometry][box2]") {
    const Box2 a{{0.0, 0.0}, {10.0, 10.0}};
    const Box2 b{{5.0, 5.0}, {15.0, 15.0}};
    const Box2 c{{20.0, 20.0}, {30.0, 30.0}};
    CHECK(a.contains(Vec2{5.0, 5.0}));
    CHECK(a.contains(Vec2{10.0, 10.0}));
    CHECK_FALSE(a.contains(Vec2{10.1, 5.0}));
    CHECK(a.contains(Vec2{10.1, 5.0}, 0.2));
    CHECK(a.contains(Box2{{1.0, 1.0}, {2.0, 2.0}}));
    CHECK_FALSE(a.contains(b));
    CHECK(a.intersects(b));
    CHECK_FALSE(a.intersects(c));
    CHECK_FALSE(Box2{}.intersects(a));
    CHECK_FALSE(Box2{}.contains(Vec2{0.0, 0.0}));

    const Box2 u = a.united(b);
    CHECK(u.min == Vec2{0.0, 0.0});
    CHECK(u.max == Vec2{15.0, 15.0});
    const Box2 i = a.intersected(b);
    CHECK(i.min == Vec2{5.0, 5.0});
    CHECK(i.max == Vec2{10.0, 10.0});
    CHECK(a.intersected(c).isEmpty());
    CHECK(a.united(Box2{}) == a);
}

TEST_CASE("Box2 inflate and approxEqual", "[geometry][box2]") {
    const Box2 a{{0.0, 0.0}, {10.0, 10.0}};
    const Box2 g = a.inflated(1.0);
    CHECK(g.min == Vec2{-1.0, -1.0});
    CHECK(g.max == Vec2{11.0, 11.0});
    CHECK(a.inflated(-6.0).isEmpty());
    CHECK(Box2{}.inflated(5.0).isEmpty());
    CHECK(carvetto::geometry::approxEqual(a, Box2{{1e-12, 0.0}, {10.0, 10.0}}));
    CHECK(carvetto::geometry::approxEqual(Box2{}, Box2{}));
    CHECK_FALSE(carvetto::geometry::approxEqual(Box2{}, a));
}
