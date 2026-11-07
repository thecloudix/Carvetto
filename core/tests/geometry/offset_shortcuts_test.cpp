// T-0170: the hypot-free shortcuts of the offset (offset_shortcuts.hpp) decide exactly as the
// expressions they replace, also for absurdly small or large directions whose squares would be
// denormal (the square of a ~1e-160 direction has only a few correct digits).

#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "carvetto/geometry/vec2.hpp"
#include "offset_shortcuts.hpp"

using namespace carvetto::geometry;

namespace {

// u = (m, 0) and v of length lv at an angle whose sine is `sine` (cosine > 0 or < 0 by `flip`).
Vec2 turned(double lv, double sine, bool flip) {
    const double c = std::sqrt(1.0 - sine * sine);
    return Vec2{(flip ? -c : c) * lv, sine * lv};
}

}  // namespace

TEST_CASE("T-0170: parallel test matches |u x v| <= 1e-12 |u| |v| near the bound at extreme magnitudes",
          "[geometry][offset][t0170]") {
    int checked = 0;
    for (const double m : {1e-150, 1e-148, 3.3e-148, 1e-160, 1.3e-159, 2.7e-158, 5e-157, 9e-156, 1e-154, 1e-3, 1.0, 1e3}) {
        for (const double lv : {1e100, 3e60, 1e25, 1.0, 1e-3}) {
            for (const double off : {-1e-3, -1e-4, -5e-5, -1e-5, -1e-6, 1e-6, 1e-5, 5e-5, 1e-4, 1e-3}) {
                for (const bool flip : {false, true}) {
                    const Vec2 u{m, 0.0};
                    const Vec2 v = turned(lv, 1e-12 * (1.0 + off), flip);
                    const bool exact = std::fabs(u.cross(v)) <= 1e-12 * u.length() * v.length();
                    CAPTURE(m, lv, off, flip);
                    CHECK(detail::directionsParallel(u, v) == exact);
                    ++checked;
                }
            }
        }
    }
    CHECK(checked > 0);
}

TEST_CASE("T-0170: cosine side matches the unit-vector dot product at extreme magnitudes",
          "[geometry][offset][t0170]") {
    for (const double level : {0.0, -0.99}) {
        for (const double m : {1e-160, 1.3e-159, 2.7e-158, 5e-157, 9e-156, 1e-154, 1e-3, 1.0, 1e3}) {
            for (const double lv : {1e100, 1e60, 1e25, 1e10, 1.0}) {
                for (const double off : {-3e-9, -1.5e-9, -5e-10, 5e-10, 1.5e-9, 3e-9}) {
                    const double cosine = level + off;
                    const double angle = std::acos(cosine);
                    const Vec2 u{m, 0.0};
                    const Vec2 v{lv * std::cos(angle), lv * std::sin(angle)};
                    const double dot = u.normalized().dot(v.normalized());
                    const int side = detail::cosineSideOf(u, v, level);
                    CAPTURE(level, m, lv, off, dot, side);
                    if (side != 0) {
                        CHECK((side > 0) == (dot > level));
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0170: shortcuts decide ordinary geometry", "[geometry][offset][t0170]") {
    CHECK(detail::directionsParallel(Vec2{1, 0}, Vec2{-2, 0}));
    CHECK_FALSE(detail::directionsParallel(Vec2{1, 0}, Vec2{0, 2}));
    CHECK(detail::cosineSideOf(Vec2{1, 0}, Vec2{1, 1}, 0.0) == 1);
    CHECK(detail::cosineSideOf(Vec2{1, 0}, Vec2{-1, 1}, 0.0) == -1);
    CHECK(detail::cosineSideOf(Vec2{1, 0}, Vec2{0, 1}, 0.0) == 0);  // too close to decide on squares
}
