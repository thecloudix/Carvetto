// T-0142 round 3: the nesting check behind outsets of unnested holes (ADR-0006 §10) finds
// candidate rings in a box hierarchy and tests big rings through an edge index
// (core/src/geometry/hole_nesting.hpp). Its decisions must be exactly those of the plain scan
// over all pairs with Clipper2's PointInPolygon, including every degenerate case (points on edges
// and vertices, vertices and horizontal edges on the test line, rounding of the cross product).
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "hole_nesting.hpp"
#include "test_rng.hpp"

using carvetto::geometry::detail::clipperPointInRing;
using carvetto::geometry::detail::everyHoleNested;
using carvetto::geometry::detail::GridPoint;
using carvetto::geometry::detail::GridRing;
using carvetto::geometry::detail::kRingIndexMinVertices;
using carvetto::geometry::detail::RingLocator;
using carvetto::geometry::detail::RingSide;
using carvetto::test::SplitMix;

namespace {

std::int64_t pick(SplitMix& rng, std::int64_t lo, std::int64_t hi) {  // in [lo, hi]
    return lo + static_cast<std::int64_t>(rng.next() % static_cast<std::uint64_t>(hi - lo + 1));
}

// A random ring: free vertices, a rectilinear staircase (horizontal edges, vertices on shared
// lines) or a comb, on a coarse grid of `step` units around `base`.
GridRing randomRing(SplitMix& rng, std::int64_t base, std::int64_t step, std::size_t n, int style) {
    GridRing r;
    const std::int64_t span = 12;
    if (style == 0) {
        for (std::size_t i = 0; i < n; ++i) {
            r.push_back({base + step * pick(rng, 0, span), base + step * pick(rng, 0, span)});
        }
    } else if (style == 1) {
        GridPoint p{base + step * pick(rng, 0, span), base + step * pick(rng, 0, span)};
        for (std::size_t i = 0; i < n; ++i) {
            r.push_back(p);
            (i % 2 == 0 ? p.x : p.y) = base + step * pick(rng, 0, span);
        }
    } else {
        // Comb: spine at x = 0, teeth to x = 3 at y = 2i .. 2i + 1, in units of `step`.
        const std::size_t teeth = n / 4 + 1;
        const auto t = static_cast<std::int64_t>(teeth);
        r.push_back({base, base});
        for (std::int64_t i = 0; i < t; ++i) {
            r.push_back({base + 3 * step, base + 2 * i * step});
            r.push_back({base + 3 * step, base + (2 * i + 1) * step});
            r.push_back({base + step, base + (2 * i + 1) * step});
            r.push_back({base + step, base + (2 * i + 2) * step});
        }
        r.push_back({base, base + (2 * t + 2) * step});
    }
    return r;
}

// Query points that hit the degenerate cases: vertices, edge points, vertex rows, random points.
GridPoint randomQuery(SplitMix& rng, const GridRing& r, std::int64_t base, std::int64_t step) {
    const GridPoint a = r[rng.next() % r.size()];
    const GridPoint b = r[rng.next() % r.size()];
    switch (rng.next() % 5) {
        case 0:
            return a;
        case 1:
            return {a.x + (b.x - a.x) / 2, a.y + (b.y - a.y) / 2};
        case 2:
            return {base + step * pick(rng, -1, 13) + pick(rng, -1, 1), a.y};
        case 3:
            return {a.x, base + step * pick(rng, -1, 13) + pick(rng, -1, 1)};
        default:
            return {base + pick(rng, -step, 14 * step), base + pick(rng, -step, 14 * step)};
    }
}

struct Box {
    std::int64_t x0, y0, x1, y1;
};

Box boxOf(const GridRing& r) {
    Box b{r[0].x, r[0].y, r[0].x, r[0].y};
    for (const GridPoint& p : r) {
        b.x0 = std::min(b.x0, p.x);
        b.y0 = std::min(b.y0, p.y);
        b.x1 = std::max(b.x1, p.x);
        b.y1 = std::max(b.y1, p.y);
    }
    return b;
}

// The plain scan: every hole against every ring wound like the lowest.
bool bruteEveryHoleNested(const std::vector<GridRing>& rings, const std::vector<int>& kind) {
    for (std::size_t h = 0; h < rings.size(); ++h) {
        if (kind[h] != 2) {
            continue;
        }
        const Box hb = boxOf(rings[h]);
        bool nested = false;
        for (std::size_t w = 0; w < rings.size() && !nested; ++w) {
            if (kind[w] != 1) {
                continue;
            }
            const Box wb = boxOf(rings[w]);
            if (wb.x0 <= hb.x0 && wb.x1 >= hb.x1 && wb.y0 <= hb.y0 && wb.y1 >= hb.y1) {
                nested = clipperPointInRing(rings[h].front(), rings[w]) == RingSide::Inside;
            }
        }
        if (!nested) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("T-0142 round 3: the indexed point-in-ring test equals Clipper2's PointInPolygon",
          "[geometry][offset][T-0142]") {
    SplitMix rng{0x5EED0142};
    std::size_t indexed = 0;
    std::size_t counts[3] = {0, 0, 0};
    for (int ringNo = 0; ringNo < 1500; ++ringNo) {
        const int style = static_cast<int>(rng.next() % 3);
        const std::size_t n = ringNo % 4 == 0 ? static_cast<std::size_t>(pick(rng, 3, 8))
                                              : static_cast<std::size_t>(pick(rng, 3, 260));
        // Small units (exact arithmetic) or huge coordinates, where the double cross product rounds.
        const bool huge = ringNo % 3 == 2;
        const std::int64_t base = huge ? pick(rng, -4000000000000000000LL, 3000000000000000000LL) / 4 : 0;
        const std::int64_t step = huge ? pick(rng, 1, 1000000000000000LL) : pick(rng, 1, 3);
        const GridRing ring = randomRing(rng, base, step, n, style);
        RingLocator<GridRing> loc(ring);
        for (int q = 0; q < 60; ++q) {
            const GridPoint p = randomQuery(rng, ring, base, step);
            const RingSide expected = clipperPointInRing(p, ring);
            INFO("ring " << ringNo << " style " << style << " size " << ring.size() << " query " << q);
            REQUIRE(loc.locate(p.x, p.y) == expected);
            ++counts[static_cast<int>(expected)];
        }
        if (ring.size() >= kRingIndexMinVertices) {
            ++indexed;
        }
    }
    INFO("outside " << counts[0] << " inside " << counts[1] << " on " << counts[2]);
    CHECK(indexed > 500);
    CHECK(counts[0] > 5000);
    CHECK(counts[1] > 5000);
    CHECK(counts[2] > 5000);
}

TEST_CASE("T-0142 round 3: nesting decisions equal the plain scan over all pairs", "[geometry][offset][T-0142]") {
    SplitMix rng{0x0142AB3};
    std::size_t nestedHoles = 0;
    std::size_t lonelyHoles = 0;
    for (int scene = 0; scene < 600; ++scene) {
        std::vector<GridRing> rings;
        std::vector<int> kind;
        const std::int64_t step = pick(rng, 1, 2);
        // Parents: rectangles, staircases and combs (some big enough to be indexed), touching and
        // overlapping on a coarse grid; holes: small rings placed at and around them.
        const int parents = static_cast<int>(pick(rng, 1, 12));
        for (int i = 0; i < parents; ++i) {
            const std::int64_t base = step * pick(rng, 0, 30);
            const int style = static_cast<int>(rng.next() % 3);
            const std::size_t n = style == 0 ? 4 : static_cast<std::size_t>(pick(rng, 4, 120));
            GridRing r = style == 0 ? GridRing{{base, base}, {base + 12 * step, base}, {base + 12 * step, base + 9 * step},
                                               {base, base + 9 * step}}
                                    : randomRing(rng, base, step, n, style);
            rings.push_back(std::move(r));
            kind.push_back(rng.next() % 6 == 0 ? 0 : 1);
        }
        const int holes = static_cast<int>(pick(rng, 1, 40));
        for (int i = 0; i < holes; ++i) {
            // Mostly anchored inside (or on the border of) a parent's box, else anywhere.
            GridPoint a{step * pick(rng, -2, 60), step * pick(rng, -2, 60)};
            if (rng.next() % 5 != 0) {
                const GridRing& parent = rings[rng.next() % static_cast<std::size_t>(parents)];
                const Box pb = boxOf(parent);
                a = {pick(rng, pb.x0, pb.x1), pick(rng, pb.y0, pb.y1)};
            }
            const std::int64_t w = pick(rng, 0, 3);
            const std::int64_t h = pick(rng, 0, 3);
            GridRing r{a, {a.x, a.y + h}, {a.x + w, a.y + h}, {a.x + w, a.y}};
            if (rng.next() % 4 == 0) {
                r = randomRing(rng, a.x, 1, static_cast<std::size_t>(pick(rng, 3, 7)), 0);
            }
            rings.push_back(std::move(r));
            kind.push_back(rng.next() % 8 == 0 ? 0 : 2);
        }
        INFO("scene " << scene);
        REQUIRE(everyHoleNested(rings, kind) == bruteEveryHoleNested(rings, kind));
        // Each hole on its own.
        for (std::size_t h = 0; h < rings.size(); ++h) {
            if (kind[h] != 2) {
                continue;
            }
            std::vector<int> one = kind;
            for (std::size_t j = 0; j < one.size(); ++j) {
                if (j != h && one[j] == 2) {
                    one[j] = 0;
                }
            }
            const bool expected = bruteEveryHoleNested(rings, one);
            INFO("hole " << h);
            REQUIRE(everyHoleNested(rings, one) == expected);
            (expected ? nestedHoles : lonelyHoles) += 1;
        }
    }
    INFO("nested " << nestedHoles << " not nested " << lonelyHoles);
    CHECK(nestedHoles > 1000);
    CHECK(lonelyHoles > 1000);
}

TEST_CASE("T-0142 round 3: nesting checks edge cases", "[geometry][offset][T-0142]") {
    const GridRing outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const GridRing hole{{2, 2}, {2, 4}, {4, 4}, {4, 2}};
    const GridRing onEdge{{0, 5}, {1, 6}, {1, 5}};  // first vertex on the outer's edge
    CHECK(everyHoleNested(std::vector<GridRing>{outer, hole}, {1, 2}));
    CHECK_FALSE(everyHoleNested(std::vector<GridRing>{outer, hole}, {0, 2}));  // no ring wound like the lowest
    CHECK_FALSE(everyHoleNested(std::vector<GridRing>{outer, onEdge}, {1, 2}));
    CHECK(everyHoleNested(std::vector<GridRing>{outer, onEdge}, {1, 0}));  // no hole
    CHECK(everyHoleNested(std::vector<GridRing>{}, {}));
}
