// T-0170: the hypot-free shortcuts of the offset (offset_shortcuts.hpp) decide exactly as the
// expressions they replace, also for absurdly small or large directions whose squares would be
// denormal (the square of a ~1e-160 direction has only a few correct digits).

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "carvetto/geometry/span.hpp"
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

// ------------------------------------------------------------------ T-0176: arcs

namespace {

// The distance the offset's exact test uses for a point and an arc (distanceToSpan, restated).
double exactArcDistance(Vec2 p, const ArcSpan& arc) {
    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
        return std::fabs(distance(p, arc.centre) - arc.radius);
    }
    return std::min(distance(p, arc.startPoint()), distance(p, arc.endPoint()));
}

}  // namespace

TEST_CASE("T-0176: arcClearlyFarther never says yes for a point nearer than the limit", "[geometry][offset][t0176]") {
    int yes = 0;
    int no = 0;
    for (const double mag : {1e-120, 1e-80, 1e-9, 1e-3, 1.0, 50.0, 1e4, 1e9, 1e30, 1e80}) {
        for (const double cx : {0.0, 1.0, -7.5, 1e4}) {
            const Vec2 centre{cx * mag, -0.5 * cx * mag};
            for (int ia = 0; ia < 12; ++ia) {
                const double a0 = 0.7 * ia;
                for (const ArcDirection dir : {ArcDirection::CCW, ArcDirection::CW}) {
                    const ArcSpan arc{centre, mag, a0, a0 + 1.9, dir};
                    for (int k = -12; k <= 12; ++k) {
                        for (int ip = 0; ip < 8; ++ip) {
                            const double ang = 0.9 * ip + 0.1;
                            const double rho = mag * (1.0 + 0.0625 * k);
                            const Vec2 p = centre + Vec2::fromAngle(ang, rho);
                            for (const double lim : {1e-3 * mag, 0.03 * mag, 0.1 * mag, 0.5 * mag, 2.0 * mag}) {
                                const double dist = exactArcDistance(p, arc);
                                if (detail::arcClearlyFarther(p, centre, mag, lim)) {
                                    CAPTURE(mag, cx, ia, k, ip, lim, dist);
                                    REQUIRE(!(dist < lim));
                                    ++yes;
                                } else {
                                    ++no;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    CHECK(yes > 1000);
    CHECK(no > 1000);
}

TEST_CASE("T-0176: arcClearlyFarther is not clear on the bound, on denormal or huge values and on non-finite input",
          "[geometry][offset][t0176]") {
    const Vec2 o{0.0, 0.0};
    // |rho - r| = 1 + a few ulp against limit 1: inside the margin, so no.
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3.0, 0.0}, o, 1.0, 2.0));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3.0, 0.0}, o, 1.0, 2.0 * (1.0 + 1e-10)));
    CHECK(detail::arcClearlyFarther(Vec2{3.0, 0.0}, o, 1.0, 1.9));
    // The point on the circle: distance 0, never farther than a positive limit.
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{1.0, 0.0}, o, 1.0, 1e-3));
    // Squares that would be denormal or overflow: not clear (the exact path decides).
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3e-160, 0.0}, o, 1e-160, 1e-170));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{5e-324, 0.0}, o, 1.0, 0.1));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3e160, 0.0}, o, 1e160, 1.0));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3.0, 0.0}, o, 1e300, 1.0));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3.0, 0.0}, o, 1.0, 1e300));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3.0, 0.0}, o, 1.0, -1.0));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{3.0, 0.0}, o, 1.0, std::nan("")));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{std::nan(""), 0.0}, o, 1.0, 0.1));
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{HUGE_VAL, 0.0}, o, 1.0, 0.1));
    // A tiny limit next to a large coordinate scale: the margin swallows it.
    CHECK_FALSE(detail::arcClearlyFarther(Vec2{1e6 + 3.0, 0.0}, Vec2{1e6, 0.0}, 1.0, 2.0 - 1e-9));
}

TEST_CASE("T-0176: normalizeAngle is the fmod formula for every kind of argument", "[geometry][t0176]") {
    const auto reference = [](double angle) {
        double a = std::fmod(angle, kTwoPi);
        if (a < 0.0) {
            a += kTwoPi;
        }
        if (a >= kTwoPi) {
            a = 0.0;
        }
        return a;
    };
    const auto same = [](double x, double y) {
        return std::memcmp(&x, &y, sizeof x) == 0 || (std::isnan(x) && std::isnan(y));
    };
    std::vector<double> values = {0.0,    -0.0,    1.0,     -1.0,          kPi,           -kPi,
                                  kTwoPi, -kTwoPi, 3 * kPi, -3 * kPi,      1e-300,        -1e-300,
                                  5e-324, -5e-324, 1e-17,   -1e-17,        1e300,         -1e300,
                                  HUGE_VAL, -HUGE_VAL, std::nan(""), std::nextafter(kTwoPi, 0.0),
                                  std::nextafter(kTwoPi, 10.0), -std::nextafter(kTwoPi, 0.0),
                                  -std::nextafter(kTwoPi, 10.0), std::nextafter(0.0, 1.0), -std::nextafter(0.0, 1.0)};
    for (int i = -400; i <= 400; ++i) {
        values.push_back(0.0173 * i);
        values.push_back(kPi * 0.5 * i);
        values.push_back(std::nextafter(kPi * 0.5 * i, 100.0));
        values.push_back(std::nextafter(kPi * 0.5 * i, -100.0));
    }
    for (const double v : values) {
        CAPTURE(v);
        CHECK(same(normalizeAngle(v), reference(v)));
    }
}

TEST_CASE("T-0176: ArcSpan::bounds matches the box of the endpoints and the axis extremes it contains",
          "[geometry][t0176]") {
    for (int ia = -9; ia < 9; ++ia) {
        for (const double sweep : {1e-4, 0.5, kPi * 0.5, 2.0, kPi, 4.0, kTwoPi - 1e-10, kTwoPi, 7.5}) {
            for (const ArcDirection dir : {ArcDirection::CCW, ArcDirection::CW}) {
                const double a0 = 0.5 * ia;
                const double a1 = dir == ArcDirection::CCW ? a0 + sweep : a0 - sweep;
                const ArcSpan arc{Vec2{3.0, -2.0}, 1.5, a0, a1, dir};
                Box2 want{arc.startPoint(), arc.endPoint()};
                for (int q = 0; q < 4; ++q) {
                    const double angle = static_cast<double>(q) * (kPi * 0.5);
                    if (arc.containsAngle(angle)) {
                        want.expand(arc.centre + Vec2::fromAngle(angle, arc.radius));
                    }
                }
                const Box2 got = arc.bounds();
                CAPTURE(ia, sweep);
                CHECK(std::memcmp(&got.min.x, &want.min.x, sizeof(double)) == 0);
                CHECK(std::memcmp(&got.min.y, &want.min.y, sizeof(double)) == 0);
                CHECK(std::memcmp(&got.max.x, &want.max.x, sizeof(double)) == 0);
                CHECK(std::memcmp(&got.max.y, &want.max.y, sizeof(double)) == 0);
            }
        }
    }
}

TEST_CASE("T-0176: arcClearlyFarther stays safe when the coordinate scale dwarfs the radius and the limit",
          "[geometry][offset][t0176]") {
    // The endpoints of an arc are computed from angles at absolute coordinates of size `scale`, so
    // they are off by a few ulp of the scale; a margin made of the limit alone would then say
    // "farther" for points whose true distance is just below the limit.
    std::uint64_t state = 0x9E3779B97F4A7C15ull;
    const auto next = [&state]() {
        state += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    };
    const auto unit = [&next]() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); };
    int yes = 0;
    int tried = 0;
    for (const double scale : {1e4, 1e6, 1e8, 1e12, 1e20, 1.8e34}) {
        for (int it = 0; it < 4000; ++it) {
            const double radius = (it % 3 == 0) ? 1e-3 : (it % 3 == 1) ? 1.0 : 0.1 * scale;
            const Vec2 centre{scale * (0.5 + unit()), -scale * (0.5 + unit())};
            const double a0 = 6.0 * unit();
            const ArcDirection dir = (it % 2 == 0) ? ArcDirection::CCW : ArcDirection::CW;
            const double sweep = 0.2 + 2.0 * unit();
            const ArcSpan arc{centre, radius, a0, dir == ArcDirection::CCW ? a0 + sweep : a0 - sweep, dir};
            // Limits from far below to far above the rounding of the endpoints.
            const double limits[] = {1e-12 * scale * (0.3 + 3.0 * unit()), 1e-13 * scale, 1e-9, 1e-3 * unit(),
                                     1e-14 * scale * unit()};
            const double lim = limits[it % 5];
            // The point: beyond the end of the arc (so the endpoint decides), at a radius that makes
            // |rho - r| about the limit, within a few 1e-10 of it; or the endpoint itself.
            Vec2 p = arc.endPoint();
            if (it % 4 != 0) {
                // at the end angle itself or a hair beyond/inside it: the endpoint and the circle agree
                const double beyond = (dir == ArcDirection::CCW ? 1.0 : -1.0) * ((it % 3) - 1) * 1e-13 * unit();
                const double rho = radius + lim * (1.0 + 4e-10 * (unit() - 0.5)) * ((it % 8 < 4) ? 1.0 : -1.0);
                if (rho <= 0.0) {
                    continue;
                }
                p = centre + Vec2::fromAngle(arc.endAngle + beyond, rho);
            }
            const double exact = exactArcDistance(p, arc);
            ++tried;
            if (detail::arcClearlyFarther(p, centre, radius, lim)) {
                CAPTURE(scale, radius, lim, exact, it);
                REQUIRE(!(exact < lim));
                ++yes;
            }
        }
    }
    CHECK(tried > 10000);
    static_cast<void>(yes);
}

TEST_CASE("T-0176: arcClearlyFarther on the verifier's repro and for tiny limits against distance 0",
          "[geometry][offset][t0176]") {
    // Exact distance 0 (the endpoint itself) against tiny limits at large coordinates.
    for (const double scale : {1e4, 1e8, 1e12}) {
        const Vec2 centre{scale, -scale};
        for (const double radius : {1e-3, 1.0, 50.0}) {
            for (int ia = 0; ia < 20; ++ia) {
                const ArcSpan arc{centre, radius, 0.37 * ia, 0.37 * ia + 1.3, ArcDirection::CCW};
                for (const Vec2 p : {arc.startPoint(), arc.endPoint()}) {
                    for (const double lim : {1e-15, 1e-12, 1e-10, 1e-9, 1e-8, 1e-6}) {
                        CAPTURE(scale, radius, ia, lim);
                        CHECK_FALSE(detail::arcClearlyFarther(p, centre, radius, lim));
                    }
                }
            }
        }
    }
    // m = 1.8e34, r = 4.2e35, L = 9.7767743590660e22: the margin is the scale term here.
    const double m = 1.8e34;
    const double r = 4.2e35;
    const double lim = 9.7767743590660e22;
    const Vec2 centre{m, -m};
    for (const double f : {0.9, 0.99, 1.0 - 1e-9, 1.0, 1.0 + 1e-9, 1.01, 1.1}) {
        const Vec2 p = centre + Vec2{r + f * lim, 0.0};
        const ArcSpan arc{centre, r, 0.0, 1.0, ArcDirection::CCW};
        const double exact = exactArcDistance(p, arc);
        if (detail::arcClearlyFarther(p, centre, r, lim)) {
            CAPTURE(f, exact);
            CHECK(!(exact < lim));
        }
    }
}

TEST_CASE("T-0176: ArcSpan::bounds at the tolerance windows of containsAngle", "[geometry][t0176]") {
    constexpr double kTol = 1e-9;
    const auto reference = [](const ArcSpan& arc) {
        Box2 box{arc.startPoint(), arc.endPoint()};
        for (int q = 0; q < 4; ++q) {
            const double angle = static_cast<double>(q) * (kPi * 0.5);
            if (arc.containsAngle(angle)) {
                box.expand(arc.centre + Vec2::fromAngle(angle, arc.radius));
            }
        }
        return box;
    };
    int cases = 0;
    for (int q = 0; q < 4; ++q) {
        const double axis = static_cast<double>(q) * (kPi * 0.5);
        for (const double dv : {0.0, 1e-10, -1e-10, 2e-9, -2e-9, 0.9e-9, -0.9e-9, 1.1e-9, -1.1e-9, 5e-10}) {
            for (const ArcDirection dir : {ArcDirection::CCW, ArcDirection::CW}) {
                const double sgn = dir == ArcDirection::CCW ? 1.0 : -1.0;
                // Arcs that start or end at the axis angle +- dv, and the other end 1.0 away either side.
                const double edges[][2] = {{axis + dv, axis + dv + sgn * 1.0}, {axis + dv - sgn * 1.0, axis + dv},
                                           {axis + dv + sgn * 0.3, axis + dv + sgn * 0.3 + sgn * (kTwoPi - 1e-9) + dv},
                                           {axis + dv, axis + dv + sgn * (kTwoPi - 1e-9)},
                                           {axis, axis + sgn * (kTwoPi - 1e-9 + dv)},
                                           {axis + dv, axis + dv + sgn * (kTwoPi - 2e-9)}};
                for (const auto& e : edges) {
                    const ArcSpan arc{Vec2{2.0, -3.0}, 1.25, e[0], e[1], dir};
                    const Box2 got = arc.bounds();
                    const Box2 want = reference(arc);
                    CAPTURE(q, dv, e[0], e[1]);
                    CHECK(std::memcmp(&got.min.x, &want.min.x, sizeof(double)) == 0);
                    CHECK(std::memcmp(&got.min.y, &want.min.y, sizeof(double)) == 0);
                    CHECK(std::memcmp(&got.max.x, &want.max.x, sizeof(double)) == 0);
                    CHECK(std::memcmp(&got.max.y, &want.max.y, sizeof(double)) == 0);
                    ++cases;
                }
            }
        }
    }
    CHECK(cases > 400);
    CHECK(kTol > 0.0);
}
