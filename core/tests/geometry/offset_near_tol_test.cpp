// T-0168: open offsets of paths whose steps are barely longer than the tolerance (a "walk" of
// 1.5 um steps against tol 1 um), offset by a distance thousands of times the step, so the whole
// path lies within d of itself. (a) Cost used to grow with the square of the number of spans for
// every join (every raw element overlaps hundreds of others and nearly all of each is inside the
// neighbourhood); (b) a closed polygon approximating a small circle lost its whole inner offset
// when every offset piece was shorter than the tolerance.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/offset.hpp"
#include "cpu_scaling.hpp"
#include "offset_chain_checks.hpp"

using namespace carvetto::geometry;

namespace {

const Tolerance kTol{};  // distance 0.001

// Open random walk of `steps` straight steps of length `step` (a fixed linear congruential
// generator picks the headings, so the path is the same everywhere).
Contour walk(int steps, double step, std::uint32_t seed) {
    Contour c;
    std::uint32_t state = seed;
    Vec2 p{0.0, 0.0};
    for (int i = 0; i < steps; ++i) {
        state = state * 1664525U + 1013904223U;
        const double angle = static_cast<double>(state >> 8) / 16777216.0 * 6.283185307179586;
        const Vec2 q{p.x + step * std::cos(angle), p.y + step * std::sin(angle)};
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
    }
    return c;
}

// Closed counter-clockwise polygon with `n` equal sides inscribed in a circle about the origin.
Contour finePolygon(int n, double radius) {
    Contour c;
    c.closed = true;
    for (int i = 0; i < n; ++i) {
        const double a0 = 6.283185307179586 * i / n;
        const double a1 = 6.283185307179586 * (i + 1) / n;
        c.spans.emplace_back(LineSpan{Vec2{radius * std::cos(a0), radius * std::sin(a0)},
                                      Vec2{radius * std::cos(a1), radius * std::sin(a1)}});
    }
    return c;
}

double distanceToInput(Vec2 p, const Contour& input) {
    double best = 1e300;
    for (const Span& s : input.spans) {
        best = std::min(best, carvetto::geometry::testing::distanceToSpanExact(p, s));
    }
    return best;
}

double distanceToChains(Vec2 p, const std::vector<Contour>& chains) {
    double best = 1e300;
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            best = std::min(best, carvetto::geometry::testing::distanceToSpanExact(p, s));
        }
    }
    return best;
}

// Every chain stays at least d (less rounding) from the input and is continuous within `gap`.
void checkChains(const std::vector<Contour>& chains, const Contour& input, double d, double gap) {
    for (const Contour& c : chains) {
        for (std::size_t k = 0; k < c.spans.size(); ++k) {
            for (const double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                const Vec2 p = pointAt(c.spans[k], t);
                REQUIRE(distanceToInput(p, input) >= d - 1e-6);
            }
            if (k > 0) {
                REQUIRE(distance(endPoint(c.spans[k - 1]), startPoint(c.spans[k])) <= gap);
            }
        }
    }
}

}  // namespace

TEST_CASE("T-0168: a walk inside its own neighbourhood: same offset as the definition, every join",
          "[geometry][offset][open][t0168]") {
    const int steps = 600;
    const double d = 1.0;
    const Contour in = walk(steps, 0.0015, 12345U);
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol);
            REQUIRE_FALSE(chains.empty());
            checkChains(chains, in, d, 1e-9);
            if (join != JoinType::Round) {
                continue;
            }
            // Round: the chains are the whole boundary, so every point of a span's offset that is
            // d from the whole walk lies on one (within the tolerance: stretches no longer than
            // that may be missing), and none of them is farther from the walk than d.
            const double sign = side == OffsetSide::Left ? 1.0 : -1.0;
            for (const Span& s : chains.front().spans) {
                const Vec2 p = pointAt(s, 0.5);
                CHECK(distanceToInput(p, in) <= d + 1e-6);
            }
            std::size_t missing = 0;
            for (const Span& s : in.spans) {
                const auto& line = std::get<LineSpan>(s);
                const Vec2 dir = (line.end - line.start).normalized();
                const Vec2 p = (line.start + line.end) * 0.5 + dir.perp() * (sign * d);
                if (distanceToInput(p, in) >= d - 1e-9 && distanceToChains(p, chains) > kTol.distance) {
                    ++missing;
                }
            }
            CHECK(missing == 0);
        }
    }
}

TEST_CASE("T-0168 (a): the cost of a walk inside its own neighbourhood grows about linearly",
          "[geometry][offset][open][t0168][perf]") {
    // Before: 1.5 um steps, d = 1: Round 4-7 s at 2000 steps and 17-43 s at 4000, Sharp 12 s and
    // 60-130 s. Now ~0.04 s and ~0.09 s in Release at 4000 and 8000 steps. The small run is offset twice, so the
    // per-offset ratio is twice the measured one; the card's bound is 2.5 per doubling.
#ifdef NDEBUG
    const int steps = 4000;
#else
    const int steps = 400;
#endif
    const Contour shortWalk = walk(steps, 0.0015, 99U);
    const Contour longWalk = walk(2 * steps, 0.0015, 99U);
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        CAPTURE(static_cast<int>(join));
        std::size_t spans = 0;
        const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
            3,
            [&] {
                for (int r = 0; r < 2; ++r) {
                    for (const Contour& c : offsetOpenContour(shortWalk, 1.0, OffsetSide::Left, join, kTol)) {
                        spans += c.spans.size();
                    }
                }
            },
            [&] {
                for (const Contour& c : offsetOpenContour(longWalk, 1.0, OffsetSide::Left, join, kTol)) {
                    spans += c.spans.size();
                }
            });
        CHECK(spans > 0);
        const double perOffsetRatio = t.ratio() * 2.0;
        CAPTURE(t.smallSeconds, t.largeSeconds, perOffsetRatio);
        CHECK(perOffsetRatio < 2.5);
    }
}

TEST_CASE("T-0168 (b): the inner offset of a fine polygon circle is present when its pieces are below the tolerance",
          "[geometry][offset][closed][t0168]") {
    // Radius 1.9, d = 1, sides of 2 pi 1.9 / n; the inner offset (radius 0.9) has pieces of
    // 2 pi 0.9 / n. With tol 0.01 they are shorter than the tolerance from n = 566 (the card's
    // case is n = 8000 at tol 0.001, run below in optimised builds); every one used to be dropped
    // as degenerate and the offset came back empty.
    const Tolerance coarse{0.01, 1e-9, 0.01};
    const Contour in = finePolygon(600, 1.9);
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        CAPTURE(static_cast<int>(join));
        const std::vector<Contour> chains = offsetOpenContour(in, 1.0, OffsetSide::Left, join, coarse);
        REQUIRE(chains.size() == 1);
        REQUIRE(chains.front().closed);
        checkChains(chains, in, 1.0, 1e-9);
        for (int k = 0; k < 36; ++k) {
            const double a = 6.283185307179586 * k / 36.0 + 0.01;
            const Vec2 p{0.9 * std::cos(a), 0.9 * std::sin(a)};
            CHECK(distanceToChains(p, chains) <= coarse.distance);
        }
    }
}

#ifdef NDEBUG
TEST_CASE("T-0168 (b): the card's case: 8000 sides of 1.5 um, R 1.9, d 1, inner offset kept",
          "[geometry][offset][closed][t0168]") {
    const Contour in = finePolygon(8000, 1.9);
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        CAPTURE(static_cast<int>(join));
        const std::vector<Contour> chains = offsetOpenContour(in, 1.0, OffsetSide::Left, join, kTol);
        REQUIRE(chains.size() == 1);
        REQUIRE(chains.front().closed);
        checkChains(chains, in, 1.0, 1e-9);
        for (int k = 0; k < 36; ++k) {
            const double a = 6.283185307179586 * k / 36.0 + 0.01;
            CHECK(distanceToChains(Vec2{0.9 * std::cos(a), 0.9 * std::sin(a)}, chains) <= kTol.distance);
        }
    }
}
#endif
