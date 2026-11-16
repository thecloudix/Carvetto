// T-0320: the flat-to-cylinder map used by the Auto-Wrapping View (ADR-0103).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "carvetto/geometry/cylinder_map.hpp"

using namespace carvetto::geometry;
using Catch::Approx;

TEST_CASE("cylinder map: along X puts the circumference on Y", "[geometry][rotary]") {
    CylinderMap m;
    m.alongX = true;
    m.radius = 10.0;
    m.axisZ = 0.0;
    SECTION("angle 0 is straight up") {
        const Vec3 p = m.wrap({5, 0, 10});
        CHECK(p.x == Approx(5));
        CHECK(p.y == Approx(0).margin(1e-12));
        CHECK(p.z == Approx(10));
    }
    SECTION("a quarter turn is arc length pi r / 2 and moves the top towards +Y") {
        const Vec3 p = m.wrap({0, kPi * 10.0 / 2.0, 10});
        CHECK(p.y == Approx(10));
        CHECK(p.z == Approx(0).margin(1e-9));
    }
    SECTION("a half turn is at the bottom") {
        const Vec3 p = m.wrap({0, kPi * 10.0, 10});
        CHECK(p.z == Approx(-10));
    }
    SECTION("flip runs the other way") {
        m.flip = true;
        const Vec3 p = m.wrap({0, kPi * 10.0 / 2.0, 10});
        CHECK(p.y == Approx(-10));
    }
    SECTION("the radius is the height above the axis") {
        m.axisZ = -10.0;  // Z zero at the surface
        const Vec3 p = m.wrap({0, 0, 0});
        CHECK(p.z == Approx(10));
        CHECK(m.wrap({0, 0, -10}).z == Approx(0).margin(1e-12));
    }
}

TEST_CASE("cylinder map: along Y puts the circumference on X", "[geometry][rotary]") {
    CylinderMap m;
    m.alongX = false;
    m.radius = 10.0;
    const Vec3 p = m.wrap({kPi * 10.0 / 2.0, 7, 10});
    CHECK(p.x == Approx(10));
    CHECK(p.y == Approx(7));
    CHECK(p.z == Approx(0).margin(1e-9));
}

TEST_CASE("cylinder map: unwrap inverts wrap near a reference", "[geometry][rotary]") {
    for (const bool alongX : {true, false}) {
        for (const bool flip : {false, true}) {
            CylinderMap m;
            m.alongX = alongX;
            m.flip = flip;
            m.radius = 12.5;
            m.axisZ = -3.0;
            for (const double c : {-70.0, -1.0, 0.0, 20.0, 60.0, 130.0}) {
                const Vec3 flat = alongX ? Vec3{4, c, 5} : Vec3{c, 4, 5};
                const Vec3 back = m.unwrap(m.wrap(flat), c);
                CHECK(back.x == Approx(flat.x).margin(1e-9));
                CHECK(back.y == Approx(flat.y).margin(1e-9));
                CHECK(back.z == Approx(flat.z).margin(1e-9));
            }
        }
    }
}

TEST_CASE("cylinder map: wrapPolyline splits segments by angle", "[geometry][rotary]") {
    CylinderMap m;
    m.radius = 10.0;
    const std::vector<Vec3> flat{{0, 0, 10}, {0, kPi * 10.0, 10}};  // half a turn
    const auto out = wrapPolyline(m, flat, kPi / 4.0);
    REQUIRE(out.size() == 5);  // 4 parts of 45 degrees
    CHECK(out.front().z == Approx(10));
    CHECK(out.back().z == Approx(-10));
    for (const Vec3& p : out) {
        CHECK(std::hypot(p.y, p.z) == Approx(10));  // stays on the cylinder
    }
    CHECK(wrapPolyline(m, {}, 0.1).empty());
    CHECK(wrapPolyline(m, {{1, 2, 10}}, 0.1).size() == 1);
}

TEST_CASE("cylinder map: wrapPolyline steps stay within the limit for a non-multiple turn", "[geometry][rotary]") {
    CylinderMap m;
    m.radius = 10.0;
    const double step = kPi / 4.0;
    const double turn = step * 2.01;  // 3 parts needed, not 2
    const std::vector<Vec3> flat{{0, 0, 10}, {0, turn * 10.0, 10}};
    const auto out = wrapPolyline(m, flat, step);
    REQUIRE(out.size() == 4);
    for (std::size_t i = 1; i < out.size(); ++i) {
        const double a0 = std::atan2(out[i - 1].y, out[i - 1].z);
        const double a1 = std::atan2(out[i].y, out[i].z);
        CHECK(std::fabs(a1 - a0) <= step + 1e-9);
    }
    const Vec3 end = m.wrap(flat.back());
    CHECK(out.back().y == Approx(end.y));
    CHECK(out.back().z == Approx(end.z));
}
