// T-0178: detail::HitIndex returns exactly what a linear first-match scan over insertion order does.
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "hit_index.hpp"

using namespace carvetto::geometry;

namespace {
std::size_t linearFirst(const std::vector<Vec2>& pts, Vec2 p, const Tolerance& tol) {
    for (std::size_t i = 0; i < pts.size(); ++i) {
        if (tol.equalPoints(pts[i], p)) {
            return i;
        }
    }
    return detail::HitIndex::npos;
}
}  // namespace

TEST_CASE("HitIndex agrees with a linear first-match scan", "[hitindex][geometry]") {
    std::mt19937_64 g(99);
    for (const double extent : {0.01, 0.1, 5.0, 1e3}) {
        for (const double base : {0.0, -3.5e2, 1e7}) {
            const Tolerance tol = Tolerance::defaults();
            std::uniform_real_distribution<double> u(-extent, extent);
            detail::HitIndex index(tol);
            std::vector<Vec2> pts;
            for (int i = 0; i < 3000; ++i) {
                const Vec2 p{base + u(g), base + u(g)};
                const auto at = index.first(p, [&](std::size_t k) { return pts[k]; });
                REQUIRE(at == linearFirst(pts, p, tol));
                if (at == detail::HitIndex::npos) {
                    index.add(pts.size(), p);
                    pts.push_back(p);
                }
            }
        }
    }
}

TEST_CASE("HitIndex finds neighbours across every cell border and at exactly the tolerance", "[hitindex][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const double d = tol.distance;
    // Stored points at several offsets around a cell border (cells are 2 d wide).
    for (const double dx : {-d, -0.999 * d, -0.5 * d, 0.0, 0.5 * d, d}) {
        for (const double dy : {-d, 0.0, d}) {
            for (const double bx : {0.0, 2.0 * d, -2.0 * d, 7.0 * d}) {
                std::vector<Vec2> pts{{bx + dx, 0.0 + dy}};
                detail::HitIndex index(tol);
                index.add(0, pts[0]);
                for (const Vec2 q : {Vec2{bx, 0.0}, Vec2{bx - 1e-9, 0.0}, Vec2{bx + 1e-9, 1e-9}}) {
                    const auto at = index.first(q, [&](std::size_t k) { return pts[k]; });
                    REQUIRE(at == linearFirst(pts, q, tol));
                }
            }
        }
    }
}

TEST_CASE("HitIndex handles non-finite and absurd coordinates like the linear scan", "[hitindex][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<Vec2> pts{{1e300, 1e300}, {0.0, 0.0}, {inf, 0.0}, {1e300, 1e300}};
    detail::HitIndex index(tol);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        index.add(i, pts[i]);
    }
    for (const Vec2 q : {Vec2{1e300, 1e300}, Vec2{0.0005, 0.0}, Vec2{inf, 0.0}, Vec2{5.0, 5.0}, Vec2{-1e300, 0.0}}) {
        const auto at = index.first(q, [&](std::size_t k) { return pts[k]; });
        REQUIRE(at == linearFirst(pts, q, tol));
    }
    // Zero tolerance: only identical points match.
    detail::HitIndex exact(Tolerance{0.0, 1e-9, 0.01});
    exact.add(0, {1.0, 1.0});
    const std::vector<Vec2> one{{1.0, 1.0}};
    REQUIRE(exact.first({1.0, 1.0}, [&](std::size_t k) { return one[k]; }) == 0);
    REQUIRE(exact.first({1.0, 1.0000001}, [&](std::size_t k) { return one[k]; }) == detail::HitIndex::npos);
}
