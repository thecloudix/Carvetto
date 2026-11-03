// T-0141: offsetContours offsets far-apart groups of closed contours in separate Clipper2 calls
// when many of them sit side by side (ADR-0006 section 9), so a row of thousands of small shapes
// costs about linear time instead of quadratic.
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "carvetto/geometry/offset.hpp"
#include "cpu_time.hpp"
#include "offset_batch.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using detail::GridBox;

namespace {

constexpr std::size_t kNever = std::numeric_limits<std::size_t>::max();

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.spans.emplace_back(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    return c;
}

Contour rect(Vec2 lo, Vec2 hi) {
    return polygon({lo, {hi.x, lo.y}, hi, {lo.x, hi.y}});
}

// The verifier's repro (T-0138): a row of n rectangles 0.5 wide, alternately 0.3 and 0.2 high,
// 0.7 apart.
std::vector<Contour> rectangleRow(int n) {
    std::vector<Contour> row;
    row.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double x = 0.7 * i;
        row.push_back(rect({x, 0.0}, {x + 0.5, i % 2 != 0 ? 0.2 : 0.3}));
    }
    return row;
}

using Ring = std::vector<Vec2>;

std::vector<Ring> ringsOf(const std::vector<Contour>& cs) {
    std::vector<Ring> out;
    for (const Contour& c : cs) {
        Ring r;
        for (const Span& s : c.spans) {
            r.push_back(startPoint(s));
        }
        out.push_back(r);
    }
    return out;
}

double distanceToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 d = b - a;
    const double len2 = d.dot(d);
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(d) / len2, 0.0, 1.0) : 0.0;
    return distance(p, a + d * t);
}

// Largest distance from a vertex of `a` to the edges of `b`.
double oneSidedDistance(const std::vector<Ring>& a, const std::vector<Ring>& b) {
    double worst = 0.0;
    for (const Ring& ra : a) {
        for (const Vec2 p : ra) {
            double best = std::numeric_limits<double>::infinity();
            for (const Ring& rb : b) {
                for (std::size_t i = 0, n = rb.size(); i < n; ++i) {
                    best = std::min(best, distanceToSegment(p, rb[i], rb[(i + 1) % n]));
                }
            }
            worst = std::max(worst, best);
        }
    }
    return worst;
}

bool boxesMeet(const GridBox& a, const GridBox& b) {
    return a.left <= b.right && b.left <= a.right && a.top <= b.bottom && b.top <= a.bottom;
}

}  // namespace

TEST_CASE("T-0141: separableBatches splits at gaps along x and y and keeps touching boxes together",
          "[geometry][offset][batch]") {
    // 0, 1 touch (shared edge x = 10); 2 is apart along x; 3 is below 0..1 (apart along y only
    // after the x cut); 4 is inside 2.
    const std::vector<GridBox> boxes{
        {0, 0, 10, 10}, {10, 0, 20, 10}, {30, 0, 40, 10}, {0, 20, 20, 30}, {32, 2, 38, 8},
    };
    const auto batches = detail::separableBatches(boxes);
    const std::vector<std::vector<std::size_t>> expect{{0, 1}, {2, 4}, {3}};
    CHECK(batches == expect);
    CHECK(detail::batchDepth(boxes, batches) == 2);  // {0, 1} and {2, 4} share y in [0, 10]
    CHECK(detail::separableBatches({}).empty());
    CHECK(detail::separableBatches({{5, 5, 5, 5}}) == std::vector<std::vector<std::size_t>>{{0}});
}

TEST_CASE("T-0141: separableBatches is a partition whose batches never meet", "[geometry][offset][batch]") {
    carvetto::test::SplitMix rng{141};
    for (int trial = 0; trial < 200; ++trial) {
        const std::size_t n = 1 + static_cast<std::size_t>(rng.uniform(0.0, 80.0));
        const double spread = trial % 2 == 0 ? 100.0 : 1000.0;
        std::vector<GridBox> boxes(n);
        for (GridBox& b : boxes) {
            b.left = static_cast<std::int64_t>(rng.uniform(0.0, spread));
            b.top = static_cast<std::int64_t>(rng.uniform(0.0, spread));
            b.right = b.left + static_cast<std::int64_t>(rng.uniform(0.0, 40.0));
            b.bottom = b.top + static_cast<std::int64_t>(rng.uniform(0.0, 40.0));
        }
        const auto batches = detail::separableBatches(boxes);
        std::vector<int> batchOf(n, -1);
        std::size_t last = 0;
        for (std::size_t k = 0; k < batches.size(); ++k) {
            REQUIRE_FALSE(batches[k].empty());
            CHECK(std::is_sorted(batches[k].begin(), batches[k].end()));
            if (k > 0) {
                CHECK(batches[k].front() > last);
            }
            last = batches[k].front();
            for (const std::size_t i : batches[k]) {
                REQUIRE(i < n);
                CHECK(batchOf[i] == -1);
                batchOf[i] = static_cast<int>(k);
            }
        }
        INFO("trial " << trial);
        for (std::size_t i = 0; i < n; ++i) {
            REQUIRE(batchOf[i] >= 0);
            for (std::size_t j = i + 1; j < n; ++j) {
                if (batchOf[i] != batchOf[j]) {
                    CHECK_FALSE(boxesMeet(boxes[i], boxes[j]));
                }
            }
        }
        // Brute-force depth: batches whose y-ranges contain one y.
        std::size_t depth = 0;
        for (const auto& probe : batches) {
            for (const std::size_t pi : probe) {
                const std::int64_t y = boxes[pi].top;
                std::size_t count = 0;
                for (const auto& b : batches) {
                    std::int64_t top = boxes[b.front()].top;
                    std::int64_t bottom = boxes[b.front()].bottom;
                    for (const std::size_t i : b) {
                        top = std::min(top, boxes[i].top);
                        bottom = std::max(bottom, boxes[i].bottom);
                    }
                    count += (top <= y && y <= bottom) ? 1 : 0;
                }
                depth = std::max(depth, count);
            }
        }
        CHECK(detail::batchDepth(boxes, batches) == depth);
    }
}

TEST_CASE("T-0141: fewer than 32 batches side by side take one Clipper2 call, more are batched",
          "[geometry][offset][batch]") {
    const Tolerance tol{0.001, 1e-9, 0.001};
    const double d = -(0.1 - 3e-6);
    // 31 rectangles in a row (and a gap of 0.2 > 2 |d|, so each is its own batch).
    const std::vector<Contour> narrow = rectangleRow(static_cast<int>(detail::kOffsetBatchMinDepth) - 1);
    CHECK(offsetContours(narrow, d, JoinType::Round, tol) ==
          detail::offsetContoursBatched(narrow, d, JoinType::Round, tol, 2.0, kNever));
    const std::vector<Contour> wide = rectangleRow(static_cast<int>(detail::kOffsetBatchMinDepth));
    const std::vector<Contour> batched = offsetContours(wide, d, JoinType::Round, tol);
    CHECK(batched == detail::offsetContoursBatched(wide, d, JoinType::Round, tol, 2.0, 0));
    // The 0.2 high rectangles collapse (inradius 0.1 > |d| by 3 nm = dust); the others remain.
    CHECK(batched.size() == wide.size() / 2);
    // A column never batches: one batch per line.
    std::vector<Contour> column;
    for (int i = 0; i < 64; ++i) {
        column.push_back(rect({0.0, 0.7 * i}, {0.5, 0.7 * i + 0.3}));
    }
    CHECK(offsetContours(column, d, JoinType::Round, tol) ==
          detail::offsetContoursBatched(column, d, JoinType::Round, tol, 2.0, kNever));
}

TEST_CASE("T-0141: batched offsets match one Clipper2 call up to grid rounding",
          "[geometry][offset][batch]") {
    // Clipper2 rounds intersections within its scanbeams, which other shapes at the same height
    // subdivide, so a batch can differ from one call by a few grid units along nearly parallel
    // edges (measured up to 11 units over ~18,000 random cases). Ring counts are the same.
    carvetto::test::SplitMix rng{1410};
    const double kSlack = 2e-5;  // mm, 20 grid units
    int compared = 0;
    int nonEmpty = 0;
    for (int scene = 0; scene < 60; ++scene) {
        const double size = rng.uniform(0.2, 3.0);
        const double pitch = size * rng.uniform(0.8, 3.0);
        const int nx = 2 + static_cast<int>(rng.uniform(0.0, 10.0));
        const int ny = 1 + static_cast<int>(rng.uniform(0.0, 3.0));
        const int orientation = scene % 4;  // 0: CCW, 1: mixed, 2: all CW, 3: CCW with holes
        const Vec2 origin{rng.uniform(-500.0, 500.0), rng.uniform(-500.0, 500.0)};
        std::vector<Contour> cs;
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                const Vec2 c = origin + Vec2{pitch * i, pitch * j} +
                               Vec2{rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2)} * size;
                const double s = size * rng.uniform(0.3, 0.6);
                Contour k;
                switch (static_cast<int>(rng.uniform(0.0, 4.0))) {
                    case 0:
                        k = rect(c - Vec2{s, 0.6 * s}, c + Vec2{s, 0.6 * s});
                        break;
                    case 1:
                        k = Contour::circle(c, s);
                        break;
                    case 2:
                        k = polygon({c + Vec2{-s, -s}, c + Vec2{s, -0.9 * s}, c + Vec2{-0.7 * s, s}});
                        break;
                    default:
                        k = rect(c - Vec2{s, 0.1 * s}, c + Vec2{s, 0.1 * s});
                        break;
                }
                const bool flip = orientation == 2 || (orientation == 1 && rng.uniform(0.0, 1.0) < 0.5);
                cs.push_back(flip ? k.reversed() : k);
                if (orientation == 3 && rng.uniform(0.0, 1.0) < 0.5) {
                    cs.push_back(Contour::circle(c, 0.25 * s, ArcDirection::CW));
                }
            }
        }
        const Tolerance tol{0.001, 1e-9, scene % 2 == 0 ? 0.001 : 0.01};
        for (int q = 0; q < 3; ++q) {
            const double d = q == 0 ? -size * rng.uniform(0.05, 0.35) : (q == 1 ? size * rng.uniform(0.01, 1.0) : 0.0);
            const JoinType join = scene % 3 == 0 ? JoinType::Round : (scene % 3 == 1 ? JoinType::Square : JoinType::Sharp);
            const auto one = ringsOf(detail::offsetContoursBatched(cs, d, join, tol, 3.0, kNever));
            const auto split = ringsOf(detail::offsetContoursBatched(cs, d, join, tol, 3.0, 0));
            INFO("scene " << scene << " q " << q);
            REQUIRE(one.size() == split.size());
            CHECK(oneSidedDistance(one, split) <= kSlack);
            CHECK(oneSidedDistance(split, one) <= kSlack);
            ++compared;
            nonEmpty += one.empty() ? 0 : 1;
        }
    }
    CHECK(nonEmpty > compared / 2);
}

TEST_CASE("T-0141: a group wound against the lowest contour is offset as in one call",
          "[geometry][offset][batch]") {
    // Clipper2's lowest contour (largest y: the CCW square on top) decides that CW contours are
    // holes: a lone CW circle is a hole without an outer and vanishes from an inset (outset, it
    // leaves Clipper2's join debris, the same in both). Batched on its own, the circle group
    // would flip to "outer" and come back as 40 rings; it joins the square's batch instead.
    const Tolerance tol{0.001, 1e-9, 0.01};
    std::vector<Contour> cs{rect({0.0, 15.0}, {5.0, 20.0})};
    for (int i = 0; i < 40; ++i) {
        cs.push_back(Contour::circle({3.0 * i, 0.0}, 1.0, ArcDirection::CW));
    }
    for (const double d : {-0.3, 0.3}) {
        const auto one = detail::offsetContoursBatched(cs, d, JoinType::Round, tol, 2.0, kNever);
        const auto split = detail::offsetContoursBatched(cs, d, JoinType::Round, tol, 2.0, 0);
        INFO("d " << d);
        if (d < 0.0) {
            CHECK(one.size() == 1);
        }
        REQUIRE(one.size() == split.size());
        CHECK(oneSidedDistance(ringsOf(one), ringsOf(split)) <= 2e-5);
    }
    // With the lowest contour wound CW the convention flips for everything, batched or not.
    std::vector<Contour> flipped;
    for (const Contour& c : cs) {
        flipped.push_back(c.reversed());
    }
    const auto one = detail::offsetContoursBatched(flipped, -0.3, JoinType::Round, tol, 2.0, kNever);
    const auto split = detail::offsetContoursBatched(flipped, -0.3, JoinType::Round, tol, 2.0, 0);
    CHECK(one.size() == 1);
    REQUIRE(one.size() == split.size());
    CHECK(oneSidedDistance(ringsOf(one), ringsOf(split)) <= 2e-5);
}

TEST_CASE("T-0141 (a): a row of rectangles offsets in about linear time (CPU-time ratio)",
          "[geometry][offset][batch][perf]") {
    // Before T-0141 one Clipper2 call held the whole row in its active edge list: about 4x per
    // doubling (20,000 rectangles took ~45 s in Release). Linear cost gives a ratio of ~4 here,
    // quadratic ~16; 2.5x per doubling allows 6.25.
    const Tolerance tol{0.001, 1e-9, 0.001};
    const std::vector<Contour> small = rectangleRow(5000);
    const std::vector<Contour> large = rectangleRow(20000);
    auto cost = [&tol](const std::vector<Contour>& row, std::size_t expect) {
        double best = std::numeric_limits<double>::infinity();
        for (int rep = 0; rep < 2; ++rep) {
            const double t0 = carvetto::test::processCpuSeconds();
            const std::size_t rings = offsetContours(row, -(0.1 - 3e-6), JoinType::Round, tol).size();
            best = std::min(best, carvetto::test::processCpuSeconds() - t0);
            CHECK(rings == expect);
        }
        return best;
    };
    const double smallCost = std::max(cost(small, 2500), 1e-3);
    const double largeCost = cost(large, 10000);
    CAPTURE(smallCost, largeCost);
    CHECK(largeCost < 6.25 * smallCost);
#ifdef NDEBUG
    CHECK(largeCost < 2.0);
#endif
}

TEST_CASE("T-0141 (a): a column of dust-thin rectangles offsets in about linear time",
          "[geometry][offset][batch][perf]") {
    // Half the rectangles collapse to dust; the dust nesting pass used to visit every ring of the
    // column (one shared x-range) for every dust ring.
    const Tolerance tol{0.001, 1e-9, 0.001};
    auto column = [](int n) {
        std::vector<Contour> cs;
        for (int i = 0; i < n; ++i) {
            const double y = 0.7 * i;
            cs.push_back(rect({0.0, y}, {0.5, y + (i % 2 != 0 ? 0.2 : 0.3)}));
        }
        return cs;
    };
    const std::vector<Contour> small = column(5000);
    const std::vector<Contour> large = column(20000);
    auto cost = [&tol](const std::vector<Contour>& cs, std::size_t expect) {
        double best = std::numeric_limits<double>::infinity();
        for (int rep = 0; rep < 2; ++rep) {
            const double t0 = carvetto::test::processCpuSeconds();
            const std::size_t rings = offsetContours(cs, -(0.1 - 3e-6), JoinType::Round, tol).size();
            best = std::min(best, carvetto::test::processCpuSeconds() - t0);
            CHECK(rings == expect);
        }
        return best;
    };
    const double smallCost = std::max(cost(small, 2500), 1e-3);
    const double largeCost = cost(large, 10000);
    CAPTURE(smallCost, largeCost);
    CHECK(largeCost < 6.25 * smallCost);
}
