#include "carvetto/geometry/box2.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstddef>
#include <vector>

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

// T-0364: MSVC 14.52 /O2 once compiled `Box2 box; box.expand(a); box.expand(b);` so that the result kept
// one field of four (std::min on the fields of the just-updated local read the stale memory slot). These
// cases sweep every ordering of the coordinates (equal, less, greater) through the same shapes the core
// uses (a segment's box stored in a vector, a ring's extent accumulated in a loop) against plain scalar
// references. They also run on Linux, where they are plain behaviour checks.
namespace {

Box2 segmentBoxOf(Vec2 a, Vec2 b) {
    Box2 box;
    box.expand(a);
    box.expand(b);
    return box;
}

double lowOf(double a, double b) { return a < b ? a : b; }
double highOf(double a, double b) { return a < b ? b : a; }

}  // namespace

TEST_CASE("T-0364: a box from two segment ends is exact for every ordering of the coordinates", "[geometry][box2][T-0364]") {
    const double v[] = {-2.5, 0.0, 1.016667, 18.983333};
    std::vector<Box2> stored;
    std::vector<Box2> expected;
    for (const double ax : v) {
        for (const double ay : v) {
            for (const double bx : v) {
                for (const double by : v) {
                    const Vec2 a{ax, ay};
                    const Vec2 b{bx, by};
                    stored.push_back(segmentBoxOf(a, b));
                    Box2 want;
                    want.min = Vec2{lowOf(ax, bx), lowOf(ay, by)};
                    want.max = Vec2{highOf(ax, bx), highOf(ay, by)};
                    expected.push_back(want);
                    CHECK(Box2{a, b} == want);
                }
            }
        }
    }
    REQUIRE(stored.size() == expected.size());
    std::size_t wrong = 0;
    for (std::size_t i = 0; i < stored.size(); ++i) {
        if (!(stored[i] == expected[i])) {
            ++wrong;
        }
    }
    CHECK(wrong == 0);
}

TEST_CASE("T-0364: the boxes of a square ring's edges and the ring's extent are exact", "[geometry][box2][T-0364]") {
    const std::vector<Vec2> ring = {{18.983333, 1.016667}, {18.983333, 18.983333}, {1.016667, 18.983333}, {1.016667, 1.016667}};
    std::vector<Box2> boxes;
    Box2 bounds;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const Vec2 a = ring[i];
        const Vec2 b = ring[(i + 1) % ring.size()];
        boxes.push_back(segmentBoxOf(a, b));
        bounds.expand(a);
    }
    REQUIRE(boxes.size() == 4);
    CHECK(boxes[0] == Box2{Vec2{18.983333, 1.016667}, Vec2{18.983333, 18.983333}});
    CHECK(boxes[1] == Box2{Vec2{1.016667, 18.983333}, Vec2{18.983333, 18.983333}});
    CHECK(boxes[2] == Box2{Vec2{1.016667, 1.016667}, Vec2{1.016667, 18.983333}});
    CHECK(boxes[3] == Box2{Vec2{1.016667, 1.016667}, Vec2{18.983333, 1.016667}});
    CHECK(bounds == Box2{Vec2{1.016667, 1.016667}, Vec2{18.983333, 18.983333}});
    for (const Box2& b : boxes) {
        CHECK_FALSE(b.isEmpty());
        CHECK(b.intersects(Box2{Vec2{8.8, 8.8}, Vec2{20.0, 8.8}}) == (b.max.x >= 8.8 && b.min.y <= 8.8 && b.max.y >= 8.8));
    }
}
