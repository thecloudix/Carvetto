// T-0152 internals: exact orientation / in-circle predicates and the incremental Delaunay
// triangulation the medial axis is built on (core/src/geometry/robust_predicates.hpp,
// delaunay.hpp).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "delaunay.hpp"
#include "robust_predicates.hpp"
#include "test_rng.hpp"

using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using carvetto::geometry::detail::Delaunay;
using carvetto::geometry::detail::incircleSign;
using carvetto::geometry::detail::orient2dSign;

TEST_CASE("robust predicates: orientation is exact near collinearity", "[geometry][medial][predicates]") {
    // Points on a grid of one unit in the last place around (0.5, 0.5): the sign of the
    // orientation against the line y = x is the sign of (y - x), which a plain double
    // evaluation gets wrong for many of them.
    const double ulp = std::ldexp(1.0, -53);
    const Vec2 a{12.0, 12.0};
    const Vec2 b{24.0, 24.0};
    for (int i = 0; i < 64; ++i) {
        for (int j = 0; j < 64; ++j) {
            const Vec2 c{0.5 + i * ulp, 0.5 + j * ulp};
            const int expected = j > i ? 1 : (j < i ? -1 : 0);
            CHECK(orient2dSign(a, b, c) == expected);
        }
    }
    CHECK(orient2dSign({0, 0}, {1e6, 1e6}, {2e6, 2e6}) == 0);
    CHECK(orient2dSign({1e6, -1e6}, {1e6 + 1e-9, -1e6}, {1e6, -1e6 + 1e-9}) == 1);
}

TEST_CASE("robust predicates: in-circle is exact for co-circular points", "[geometry][medial][predicates]") {
    CHECK(incircleSign({1, 0}, {0, 1}, {-1, 0}, {0, -1}) == 0);
    CHECK(incircleSign({1, 0}, {0, 1}, {-1, 0}, {0, -0.999999999999}) == 1);
    CHECK(incircleSign({1, 0}, {0, 1}, {-1, 0}, {0, -1.000000000001}) == -1);
    // Large offsets: the same circle translated far from the origin.
    const Vec2 o{1e6, -3e5};
    CHECK(incircleSign(o + Vec2{3, 0}, o + Vec2{0, 3}, o + Vec2{-3, 0}, o + Vec2{0, -3}) == 0);
    // Clockwise triangle flips the sign.
    CHECK(incircleSign({1, 0}, {-1, 0}, {0, 1}, {0, 0}) == -1);
}

namespace {

/// Checks the triangulation's invariants: counter-clockwise triangles, symmetric adjacency,
/// and the empty-circle property against every input vertex (brute force).
void checkDelaunay(const Delaunay& dt, const std::vector<int>& ids) {
    std::set<int> vertices(ids.begin(), ids.end());
    std::size_t live = 0;
    for (std::size_t t = 0; t < dt.triangleSlots(); ++t) {
        const int ti = static_cast<int>(t);
        if (!dt.alive(ti)) {
            continue;
        }
        ++live;
        const auto& tr = dt.triangle(ti);
        REQUIRE(orient2dSign(dt.point(tr.v[0]), dt.point(tr.v[1]), dt.point(tr.v[2])) > 0);
        for (int k = 0; k < 3; ++k) {
            const int nb = tr.n[static_cast<std::size_t>(k)];
            if (nb < 0) {
                continue;
            }
            REQUIRE(dt.alive(nb));
            bool back = false;
            for (int m : dt.triangle(nb).n) {
                back = back || m == ti;
            }
            REQUIRE(back);
        }
        bool super = false;
        for (int v : tr.v) {
            super = super || Delaunay::isSuper(v);
        }
        if (super) {
            continue;
        }
        for (int v : vertices) {
            if (v == tr.v[0] || v == tr.v[1] || v == tr.v[2]) {
                continue;
            }
            CHECK(incircleSign(dt.point(tr.v[0]), dt.point(tr.v[1]), dt.point(tr.v[2]), dt.point(v)) <= 0);
        }
    }
    // Euler: a triangulation of n points inside the super triangle has 2n + 1 triangles.
    CHECK(live == 2 * vertices.size() + 1);
}

}  // namespace

TEST_CASE("delaunay: random, grid-snapped, co-circular and duplicate points", "[geometry][medial][delaunay]") {
    carvetto::test::SplitMix rng{0xDE1A};
    for (int round = 0; round < 6; ++round) {
        Box2 box{{-10, -10}, {10, 10}};
        Delaunay dt(box);
        std::vector<int> ids;
        for (int i = 0; i < 300; ++i) {
            Vec2 p{rng.uniform(-10, 10), rng.uniform(-10, 10)};
            if (round % 3 == 1) {
                p = {std::round(p.x), std::round(p.y)};  // many collinear and duplicate points
            } else if (round % 3 == 2) {
                const double a = rng.uniform(0, 6.283185307179586);
                p = {8 * std::cos(a), 8 * std::sin(a)};  // co-circular up to rounding
            }
            ids.push_back(dt.insert(p, ids.empty() ? -1 : ids.back()));
        }
        INFO("round " << round);
        checkDelaunay(dt, ids);
    }
}

TEST_CASE("delaunay: duplicates return the existing vertex; edge lookup", "[geometry][medial][delaunay]") {
    Delaunay dt(Box2{{0, 0}, {4, 4}});
    const int a = dt.insert({0, 0});
    const int b = dt.insert({4, 0});
    const int c = dt.insert({2, 3});
    CHECK(dt.insert({4, 0}) == b);
    CHECK(dt.vertexCount() == 6);  // three super vertices + three points
    CHECK(dt.triangleWithEdge(a, b) >= 0);
    CHECK(dt.triangleWithEdge(b, c) >= 0);
    CHECK(dt.triangleWithEdge(c, a) >= 0);
    // The reverse directions belong to outside (super-vertex) triangles.
    CHECK(dt.triangleWithEdge(b, a) >= 0);
    // A point on the edge a-b removes it.
    const int m = dt.insert({2, 0});
    CHECK(dt.triangleWithEdge(a, b) < 0);
    CHECK(dt.triangleWithEdge(a, m) >= 0);
    CHECK(dt.triangleWithEdge(m, b) >= 0);
}
