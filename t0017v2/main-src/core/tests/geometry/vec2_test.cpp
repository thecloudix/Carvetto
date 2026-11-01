#include "carvetto/geometry/vec2.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using carvetto::geometry::Vec2;
using Catch::Matchers::WithinAbs;

TEST_CASE("Vec2 arithmetic", "[geometry]") {
    const Vec2 a{1.0, 2.0};
    const Vec2 b{3.0, -4.0};
    CHECK((a + b) == Vec2{4.0, -2.0});
    CHECK((a - b) == Vec2{-2.0, 6.0});
    CHECK((a * 2.0) == Vec2{2.0, 4.0});
    CHECK((2.0 * a) == Vec2{2.0, 4.0});
    CHECK_THAT(b.length(), WithinAbs(5.0, 1e-12));
    CHECK_THAT(a.dot(b), WithinAbs(-5.0, 1e-12));
    CHECK_THAT(a.cross(b), WithinAbs(-10.0, 1e-12));
}

TEST_CASE("Vec2 normalized and perp", "[geometry]") {
    CHECK(Vec2{0.0, 0.0}.normalized() == Vec2{0.0, 0.0});
    const Vec2 n = Vec2{3.0, 4.0}.normalized();
    CHECK_THAT(n.length(), WithinAbs(1.0, 1e-12));
    CHECK(Vec2{1.0, 0.0}.perp() == Vec2{0.0, 1.0});
    CHECK(carvetto::geometry::approxEqual(Vec2{1.0, 1.0}, Vec2{1.0 + 1e-10, 1.0}, 1e-9));
}
