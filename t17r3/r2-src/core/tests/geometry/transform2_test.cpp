#include "carvetto/geometry/transform2.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;

namespace {
bool near(Vec2 a, Vec2 b, double tol = 1e-12) {
    return approxEqual(a, b, tol);
}
}  // namespace

TEST_CASE("Transform2 identity and translation", "[geometry][transform2]") {
    const Vec2 p{3.0, -2.0};
    CHECK(Transform2::identity().apply(p) == p);
    CHECK(Transform2::translation({1.0, 2.0}).apply(p) == Vec2{4.0, 0.0});
    // Translation does not affect direction vectors.
    CHECK(Transform2::translation({1.0, 2.0}).applyVector(p) == p);
}

// Acceptance criterion 1.
TEST_CASE("Transform2 rotation by pi/2 maps (1,0) to (0,1)", "[geometry][transform2][acceptance]") {
    const Vec2 r = Transform2::rotation(kPi / 2.0).apply({1.0, 0.0});
    CHECK_THAT(r.x, WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.y, WithinAbs(1.0, 1e-12));
}

// Acceptance criterion 1 (second half).
TEST_CASE("Transform2 compose(A,B).apply(p) == A.apply(B.apply(p))", "[geometry][transform2][acceptance]") {
    const Transform2 A = Transform2::rotation(0.7, {2.0, 3.0}) * Transform2::scaling(2.0, 0.5);
    const Transform2 B = Transform2::translation({-4.0, 1.5}) * Transform2::rotation(-1.3);
    const Vec2 pts[] = {{0.0, 0.0}, {1.0, 0.0}, {-3.5, 7.25}, {100.0, -0.001}};
    for (const Vec2 p : pts) {
        const Vec2 composed = compose(A, B).apply(p);
        const Vec2 nested = A.apply(B.apply(p));
        CHECK(near(composed, nested, 1e-9));
        CHECK(near((A * B).apply(p), nested, 1e-9));
    }
    // Composition is not commutative in general.
    CHECK_FALSE(near(compose(A, B).apply({1.0, 1.0}), compose(B, A).apply({1.0, 1.0}), 1e-6));
}

TEST_CASE("Transform2 rotation about a centre keeps the centre fixed", "[geometry][transform2]") {
    const Vec2 c{5.0, -2.0};
    const Transform2 r = Transform2::rotation(1.234, c);
    CHECK(near(r.apply(c), c));
    CHECK(near(r.apply(c + Vec2{1.0, 0.0}), c + Vec2::fromAngle(1.234)));
}

TEST_CASE("Transform2 scaling", "[geometry][transform2]") {
    CHECK(Transform2::scaling(2.0).apply({1.0, -3.0}) == Vec2{2.0, -6.0});
    CHECK(Transform2::scaling(2.0, 3.0).apply({1.0, -3.0}) == Vec2{2.0, -9.0});
    const Transform2 s = Transform2::scaling(2.0, 3.0, {1.0, 1.0});
    CHECK(near(s.apply({1.0, 1.0}), {1.0, 1.0}));
    CHECK(near(s.apply({2.0, 2.0}), {3.0, 4.0}));
}

TEST_CASE("Transform2 mirrors", "[geometry][transform2]") {
    CHECK(Transform2::mirrorX().apply({3.0, 4.0}) == Vec2{-3.0, 4.0});
    CHECK(Transform2::mirrorX(1.0).apply({3.0, 4.0}) == Vec2{-1.0, 4.0});
    CHECK(Transform2::mirrorY().apply({3.0, 4.0}) == Vec2{3.0, -4.0});
    CHECK(Transform2::mirrorY(1.0).apply({3.0, 4.0}) == Vec2{3.0, -2.0});
    // Mirror across the diagonal y = x swaps coordinates.
    const Transform2 d = Transform2::mirror({0.0, 0.0}, {1.0, 1.0});
    CHECK(near(d.apply({3.0, 4.0}), {4.0, 3.0}));
    CHECK(d.isMirroring());
    CHECK_FALSE(Transform2::rotation(1.0).isMirroring());
    CHECK(Transform2::mirror({0.0, 0.0}, {0.0, 0.0}) == Transform2::identity());
}

TEST_CASE("Transform2 inverse", "[geometry][transform2]") {
    const Transform2 m =
        Transform2::translation({3.0, -1.0}) * Transform2::rotation(0.4) * Transform2::scaling(2.0, 0.25);
    const auto inv = m.inverse();
    REQUIRE(inv.has_value());
    const Vec2 p{7.0, 11.0};
    CHECK(near(inv->apply(m.apply(p)), p, 1e-12));
    CHECK(approxEqual(*inv * m, Transform2::identity(), 1e-12));
    CHECK(approxEqual(m * *inv, Transform2::identity(), 1e-12));
    CHECK_FALSE(Transform2::scaling(0.0).inverse().has_value());
    CHECK_FALSE(Transform2::scaling(1.0, 0.0).isInvertible());
}

TEST_CASE("Transform2 conformality", "[geometry][transform2]") {
    CHECK(Transform2::identity().isConformal());
    CHECK(Transform2::rotation(0.3).isConformal());
    CHECK((Transform2::rotation(0.3) * Transform2::scaling(2.5)).isConformal());
    CHECK((Transform2::mirrorX() * Transform2::rotation(0.3)).isConformal());
    CHECK_FALSE(Transform2::scaling(1.0, 2.0).isConformal());
    CHECK_FALSE(Transform2{1.0, 0.5, 0.0, 1.0, 0.0, 0.0}.isConformal());  // shear
    CHECK_FALSE(Transform2::scaling(0.0).isConformal());
    CHECK_THAT((Transform2::rotation(1.0) * Transform2::scaling(3.0)).uniformScale(), WithinAbs(3.0, 1e-12));
    CHECK_THAT(Transform2::rotation(1.0).determinant(), WithinAbs(1.0, 1e-12));
}
