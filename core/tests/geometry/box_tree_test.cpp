// T-0172: detail::BoxTree (core/src/geometry/box_tree.hpp), the box set behind the self-crossing
// cuts, and detail::BoxIndex (box_index.hpp), a grid plus a BoxTree behind offsetOpenContour step
// 7's cover queries. Round 1's clustering grid kept re-filing boxes that overlap a cell into finer
// grids, so nested, concentric or collinear boxes (a spiral slot's arcs, a back-and-forth strand)
// took gigabytes; round 2's BoxTree alone cost ~3.4 sqrt(n) per query on a hatch with varied line
// ends. These tests pin the query results against brute force and bound memory and query work on
// such box sets.

#include "box_index.hpp"
#include "box_tree.hpp"

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using carvetto::geometry::detail::BoxIndex;
using carvetto::geometry::detail::BoxTree;

namespace {

// Deterministic on every platform (no <random> distributions).
struct Lcg {
    std::uint64_t state;
    double uniform(double lo, double hi) {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return lo + (hi - lo) * static_cast<double>(state >> 11) * (1.0 / 9007199254740992.0);
    }
    double logUniform(double lo, double hi) { return std::exp(uniform(std::log(lo), std::log(hi))); }
    std::size_t below(std::size_t n) {
        return static_cast<std::size_t>(uniform(0.0, static_cast<double>(n))) % n;
    }
};

Box2 around(Vec2 c, double w, double h) {
    return Box2{c - Vec2{0.5 * w, 0.5 * h}, c + Vec2{0.5 * w, 0.5 * h}};
}

constexpr int kFamilies = 13;

// Box families that defeat uniform or clustering grids.
std::vector<Box2> family(int kind, std::size_t n, Lcg& rng) {
    std::vector<Box2> b;
    double x = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double k = static_cast<double>(i);
        switch (kind) {
            case 0:  // concentric squares
                b.push_back(around({0, 0}, 0.01 * (k + 1), 0.01 * (k + 1)));
                break;
            case 1: {  // the semicircles of a spiral (nested, alternating halves)
                const double r = 0.01 * static_cast<double>(i / 2 + 1);
                b.push_back(around(i % 2 ? Vec2{0, 0.5 * r} : Vec2{0, -0.5 * r}, 2 * r, r));
                break;
            }
            case 2:  // collinear nested segments with sub-nm jitter across
                b.push_back(around({rng.uniform(0, 1e-9), 3.0}, 0.0, rng.logUniform(1e-12, 1.0)));
                break;
            case 3:  // clustered, sizes over three decades
                b.push_back(around({rng.uniform(0, 1e-3), rng.uniform(0, 1e-3)}, rng.logUniform(1e-6, 1e-3),
                                   rng.logUniform(1e-6, 1e-3)));
                break;
            case 4:  // all huge (each covers all the others)
                b.push_back(around({rng.uniform(0, 1), rng.uniform(0, 1)}, 2, 2));
                break;
            case 5:  // all tiny
                b.push_back(around({rng.uniform(0, 100), rng.uniform(0, 100)}, 1e-9, 1e-9));
                break;
            case 6:  // tiny and huge mixed
                b.push_back(i % 2 ? around({rng.uniform(0, 100), rng.uniform(0, 100)}, 1e-6, 1e-6)
                                  : around({rng.uniform(0, 100), rng.uniform(0, 100)}, rng.logUniform(1, 200),
                                           rng.logUniform(1, 200)));
                break;
            case 7:  // a hatch at pitch 0.004 (one far box is added below)
                b.push_back(around({5, 0.004 * k}, 10, 0));
                break;
            case 8: {  // a back-and-forth strand along one line, lengths over seven decades
                const double len = rng.logUniform(1e-6, 10.0);
                const double next = i % 2 ? x - len : x + len;
                b.push_back(Box2{Vec2{x, 0}, Vec2{next, 0}});
                x = next;
                break;
            }
            case 9:  // flat boxes spread over 1e8 mm, sizes over twelve decades
                b.push_back(around({rng.uniform(-1, 1) * 1e8, rng.uniform(-1, 1) * 1e-8},
                                   rng.logUniform(1e-6, 1e6), 0));
                break;
            case 10:  // identical points
                b.push_back(around({1, 1}, 0, 0));
                break;
            case 11: {  // clusters within clusters
                double s = 1.0;
                Vec2 c{0, 0};
                for (int l = 0; l < 5; ++l) {
                    c = c + Vec2{rng.uniform(0, 1) * s, rng.uniform(0, 1) * s};
                    s *= 1e-3;
                }
                b.push_back(around(c, s * rng.uniform(0, 2), s * rng.uniform(0, 2)));
                break;
            }
            default:  // uniform scatter
                b.push_back(around({rng.uniform(0, 100), rng.uniform(0, 100)}, rng.logUniform(1e-3, 5),
                                   rng.logUniform(1e-3, 5)));
                break;
        }
    }
    if (kind == 7 || kind == 10) {
        b.push_back(around({1e5, 1e5}, 1, 1));
    }
    return b;
}

std::vector<std::size_t> reported(BoxTree& tree, const Box2& q) {
    std::vector<std::size_t> got;
    tree.query(q, [&](std::size_t i) { got.push_back(i); });
    std::sort(got.begin(), got.end());
    return got;
}

}  // namespace

TEST_CASE("T-0172: BoxTree reports exactly the boxes that meet the query, once each",
          "[geometry][offset][t0172]") {
    Lcg rng{0xB0C5u};
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::size_t checked = 0;
    for (int trial = 0; trial < 3 * kFamilies; ++trial) {
        const int kind = trial % kFamilies;
        const std::size_t n = 1 + rng.below(trial < kFamilies ? 40 : 1500);
        std::vector<Box2> items = family(kind, n, rng);
        // Boxes no query meets (empty, NaN) and boxes reaching infinity, anywhere in the order.
        items.insert(items.begin() + static_cast<std::ptrdiff_t>(rng.below(items.size())), Box2{});
        items.insert(items.begin() + static_cast<std::ptrdiff_t>(rng.below(items.size())),
                     Box2{Vec2{nan, 0}, Vec2{1, 1}});
        items.insert(items.begin() + static_cast<std::ptrdiff_t>(rng.below(items.size())),
                     Box2{Vec2{-inf, 2}, Vec2{inf, 2}});
        // Bulk-built on a prefix (or empty), then grown one box at a time with queries in between,
        // some of them for a range of ids only.
        const std::size_t bulk = trial % 3 == 0 ? 0 : rng.below(items.size());
        BoxTree tree(std::vector<Box2>(items.begin(), items.begin() + static_cast<std::ptrdiff_t>(bulk)));
        CAPTURE(trial, kind, n, bulk);
        for (std::size_t i = bulk; i <= items.size(); ++i) {
            const bool lastRound = i == items.size();
            for (int q = 0; q < (lastRound ? 300 : 2); ++q) {
                const Box2& base = items[rng.below(items.size())];
                Box2 query = base.inflated(rng.uniform(0, 1) < 0.2 ? 0.0 : rng.logUniform(1e-15, 1e3));
                switch (q % 7) {
                    case 1:
                        query = Box2{base.min, base.min};  // a corner point
                        break;
                    case 2:
                        query = Box2{Vec2{-1e12, -1e12}, Vec2{1e12, 1e12}};
                        break;
                    case 3:
                        query = Box2{};  // meets nothing
                        break;
                    default:
                        break;
                }
                std::vector<std::size_t> want;
                for (std::size_t j = 0; j < i; ++j) {
                    if (items[j].intersects(query)) {
                        want.push_back(j);
                    }
                }
                CHECK(reported(tree, query) == want);  // sorted, so a duplicate would differ too
                CHECK(tree.queryUntil(query, [](std::size_t) { return true; }) == !want.empty());
                // queryUntil stops at the first item for which f is true.
                if (!want.empty()) {
                    const std::size_t target = want[want.size() / 2];
                    std::size_t calls = 0;
                    CHECK(tree.queryUntil(query, [&](std::size_t j) {
                        ++calls;
                        return j == target;
                    }));
                    CHECK(calls <= want.size());
                }
                // Ids in [lo, hi] only.
                const std::size_t lo = rng.below(i + 1);
                const std::size_t hi = q % 5 == 0 ? lo : lo + rng.below(i + 2 - lo);
                std::vector<std::size_t> inRange;
                for (const std::size_t j : want) {
                    if (j >= lo && j <= hi) {
                        inRange.push_back(j);
                    }
                }
                std::vector<std::size_t> got;
                CHECK_FALSE(tree.queryUntil(query, lo, hi, [&](std::size_t j) {
                    got.push_back(j);
                    return false;
                }));
                std::sort(got.begin(), got.end());
                CHECK(got == inRange);
                CHECK_FALSE(tree.queryUntil(query, hi + 1, hi, [](std::size_t) { return true; }));
                ++checked;
            }
            if (i < items.size()) {
                CHECK(tree.add(items[i]) == i);
            }
        }
        CHECK(tree.box(items.size() - 1) == items.back());
        CHECK(tree.heldCount() == items.size() - 2);  // the empty and the NaN box are not held
    }
    CHECK(checked > 10000);
}

TEST_CASE("T-0172: BoxTree memory and query work stay near-linear on nested, collinear and clustered boxes",
          "[geometry][offset][t0172]") {
    // Each family: n boxes added one at a time, each first queried with its own box, then every
    // box's corner queried. Memory: every box is held once and the trees have fewer than n / 2
    // nodes (round 1's grid filed 107 M entries for 4000 spiral semicircles and over 8 GB for 120
    // nested collinear boxes). Work (nodes and boxes looked at): within a constant of
    // n log2 n plus the pairs reported, so no family costs quadratic time unless it reports a
    // quadratic number of pairs (a k-d tree visits O(n^(3/4)) nodes per query). The constant is
    // per family, ~1.5x the work measured (the collinear nested family is the worst, 34x): a
    // tree that split along one coordinate only (round-2 verifier's mutant BA) does 15.9x on the
    // clustered family, 9.2x on tiny and huge mixed and 20.5x on the uniform scatter.
    const std::array<double, kFamilies> factor{2, 2, 50, 10, 2.5, 18, 6, 5, 6, 18, 2, 18, 17};
    Lcg rng{0x5EEDu};
    const std::size_t n = 3000;
    for (int kind = 0; kind < kFamilies; ++kind) {
        const std::vector<Box2> boxes = family(kind, n, rng);
        BoxTree tree;
        std::size_t pairs = 0;
        for (const Box2& b : boxes) {
            tree.query(b, [&](std::size_t) { ++pairs; });
            tree.add(b);
        }
        for (const Box2& b : boxes) {
            tree.query(Box2{b.min, b.min}, [&](std::size_t) { ++pairs; });
        }
        const double m = static_cast<double>(boxes.size());
        const double budget =
            factor[static_cast<std::size_t>(kind)] * (m * std::log2(m) + static_cast<double>(pairs));
        CAPTURE(kind, pairs, tree.work(), tree.nodeCount(), tree.treeCount());
        CHECK(tree.heldCount() == boxes.size());
        CHECK(tree.nodeCount() < boxes.size() / 2);
        CHECK(static_cast<double>(tree.treeCount()) <= std::log2(m) + 1.0);
        CHECK(static_cast<double>(tree.work()) <= budget);
        // Uniform-grid and quadratic costs would be ~n^2 = 9e6 looks without reporting as many.
        if (pairs < 20 * boxes.size()) {
            CHECK(static_cast<double>(tree.work()) < 0.25 * m * m);
        }
    }
    // A bulk-built tree holds each box once too.
    const std::vector<Box2> nested = family(1, 20000, rng);
    const BoxTree bulk(nested);
    CHECK(bulk.heldCount() == nested.size());
    CHECK(bulk.treeCount() == 1);
    CHECK(bulk.nodeCount() < nested.size() / 2);
}

namespace {

std::vector<std::size_t> reported(BoxIndex& index, const Box2& q) {
    std::vector<std::size_t> got;
    index.query(q, [&](std::size_t i) { got.push_back(i); });
    std::sort(got.begin(), got.end());
    return got;
}

// Lines of a hatch at pitch 0.1 whose ends vary along x (a in [0, 4], b in [6, 10]): boxes that
// overlap along x and differ only in y, a k-d tree's weak case (round 2's step 7 cost up to 1.44x
// main on such strays, T-0172 round-2 verifier's rhatch); `across`: the same turned 90 degrees.
std::vector<Box2> variedHatch(std::size_t n, bool across, Lcg& rng) {
    std::vector<Box2> b;
    for (std::size_t i = 0; i < n; ++i) {
        const double y = 0.1 * static_cast<double>(i);
        const double x0 = rng.uniform(0, 4);
        const double x1 = rng.uniform(6, 10);
        b.push_back(across ? Box2{Vec2{-y, x0}, Vec2{-y, x1}} : Box2{Vec2{x0, y}, Vec2{x1, y}});
    }
    return b;
}

}  // namespace

TEST_CASE("T-0172: BoxIndex reports exactly the boxes that meet the query, once each, whatever its layout",
          "[geometry][offset][t0172]") {
    Lcg rng{0x1DE7u};
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::size_t checked = 0;
    std::size_t inGrid = 0;
    std::size_t inTree = 0;
    for (int trial = 0; trial < 4 * kFamilies; ++trial) {
        const int kind = trial % kFamilies;
        const std::size_t n = 1 + rng.below(trial < kFamilies ? 40 : 1500);
        std::vector<Box2> items = family(kind, n, rng);
        items.insert(items.begin() + static_cast<std::ptrdiff_t>(rng.below(items.size())), Box2{});
        items.insert(items.begin() + static_cast<std::ptrdiff_t>(rng.below(items.size())),
                     Box2{Vec2{nan, 0}, Vec2{1, 1}});
        items.insert(items.begin() + static_cast<std::ptrdiff_t>(rng.below(items.size())),
                     Box2{Vec2{-inf, 2}, Vec2{inf, 2}});
        items.insert(items.begin() + static_cast<std::ptrdiff_t>(rng.below(items.size())),
                     Box2{Vec2{0, 0}, Vec2{1e300, 1e300}});
        // Laid out over all the boxes, over a few of them (the rest lie outside), over nothing, or
        // over a non-finite box; for about as many boxes or far fewer.
        Box2 layout;
        const int variant = trial / kFamilies;
        const std::size_t few = std::min<std::size_t>(items.size(), 5);
        for (std::size_t i = 0; i < (variant == 1 ? few : items.size()); ++i) {
            if (std::isfinite(items[i].min.x) && std::isfinite(items[i].max.y) && items[i].max.x < 1e299) {
                layout.expand(items[i]);
            }
        }
        if (variant == 2) {
            layout = Box2{};
        } else if (variant == 3) {
            layout = Box2{Vec2{-inf, 0}, Vec2{inf, 1}};
        }
        BoxIndex index(layout, variant == 1 ? few : items.size());
        CAPTURE(trial, kind, n, variant);
        for (std::size_t i = 0; i <= items.size(); ++i) {
            const bool lastRound = i == items.size();
            for (int q = 0; q < (lastRound ? 300 : 2); ++q) {
                const Box2& base = items[rng.below(items.size())];
                Box2 query = base.inflated(rng.uniform(0, 1) < 0.2 ? 0.0 : rng.logUniform(1e-15, 1e3));
                switch (q % 7) {
                    case 1:
                        query = Box2{base.min, base.min};
                        break;
                    case 2:
                        query = Box2{Vec2{-1e12, -1e12}, Vec2{1e12, 1e12}};
                        break;
                    case 3:
                        query = Box2{};
                        break;
                    case 4:
                        query = Box2{Vec2{-inf, -inf}, Vec2{inf, inf}};
                        break;
                    default:
                        break;
                }
                std::vector<std::size_t> want;
                for (std::size_t j = 0; j < i; ++j) {
                    if (items[j].intersects(query)) {
                        want.push_back(j);
                    }
                }
                CHECK(reported(index, query) == want);  // sorted, so a duplicate would differ too
                // The grid and the tree report disjoint parts of it.
                std::vector<std::size_t> parts;
                index.queryGrid(query, [&](std::size_t j) { parts.push_back(j); });
                const std::size_t fromGrid = parts.size();
                index.queryTree(query, [&](std::size_t j) { parts.push_back(j); });
                std::sort(parts.begin(), parts.end());
                CHECK(parts == want);
                inGrid += fromGrid;
                inTree += parts.size() - fromGrid;
                ++checked;
            }
            if (i < items.size()) {
                CHECK(index.add(items[i]) == i);
                CHECK((index.box(i) == items[i] || std::isnan(items[i].min.x)));
            }
        }
        CHECK(index.gridCount() + index.treeCount() == items.size());
        CHECK(index.fullestCell() <= BoxIndex::kMaxPerCell);
    }
    // Built at once over a set: the same answers.
    const std::vector<Box2> scatter = family(kFamilies - 1, 2000, rng);
    BoxIndex bulk(scatter);
    for (int q = 0; q < 200; ++q) {
        const Box2 query = scatter[rng.below(scatter.size())].inflated(rng.logUniform(1e-6, 10));
        std::vector<std::size_t> want;
        for (std::size_t j = 0; j < scatter.size(); ++j) {
            if (scatter[j].intersects(query)) {
                want.push_back(j);
            }
        }
        CHECK(reported(bulk, query) == want);
    }
    CHECK(checked > 15000);
    CHECK(inGrid > 1000);
    CHECK(inTree > 1000);
}

TEST_CASE("T-0172: BoxIndex memory stays linear and its grid cells small on every box family",
          "[geometry][offset][t0172]") {
    // Each cell takes at most kMaxPerCell boxes and each box in the grid at most kMaxCellsPerBox
    // cells, and the grid has at most 3 max(1024, count) + 1 cells, so the entries are linear in
    // the boxes whatever their layout; boxes the grid does not take go to the tree. Nested boxes
    // (concentric squares, a spiral's semicircles) cover many cells: most go to the tree. A hatch
    // at pitch 0.004 next to one far box crowds one cell: all but kMaxPerCell go to the tree.
    Lcg rng{0x3E3Du};
    const std::size_t n = 3000;
    for (int kind = 0; kind < kFamilies; ++kind) {
        const std::vector<Box2> boxes = family(kind, n, rng);
        const BoxIndex index(boxes);
        const std::size_t m = boxes.size();
        CAPTURE(kind, index.gridCount(), index.treeCount(), index.entryCount(), index.cellCount());
        CHECK(index.fullestCell() <= BoxIndex::kMaxPerCell);
        CHECK(index.entryCount() <= BoxIndex::kMaxCellsPerBox * index.gridCount());
        CHECK(index.entryCount() <= BoxIndex::kMaxPerCell * index.cellCount());
        CHECK(index.cellCount() <= 3 * std::max<std::size_t>(1024, m) + 1);
        if (kind == 0 || kind == 1 || kind == 4) {
            CHECK(index.treeCount() > m / 2);
        }
        if (kind == 7 || kind == 10) {
            CHECK(index.gridCount() <= BoxIndex::kMaxPerCell + 1);
        }
        if (kind == 5 || kind == 12) {
            CHECK(index.treeCount() == 0);  // spread evenly: all in the grid
        }
    }
}

TEST_CASE("T-0172: BoxIndex queries cost O(1) on hatches with varied line ends, O(log n) when crowded",
          "[geometry][offset][t0172]") {
    // Small square queries (16 tolerances, step 7's first reach) at random points of a hatch at
    // pitch 0.1 with varied line ends: the grid looks at ~10 cells and boxes per query at any size
    // (BoxTree: ~3.4 sqrt(n), 860 at n = 64 000). Crowded into one cell by a far box (the round-1
    // clustered repro): the tree takes them, O(log n) per query.
    Lcg rng{0x8A7Cu};
    double perQuery[2][2] = {};
    const std::size_t sizes[2] = {2000, 64000};
    for (int across = 0; across < 2; ++across) {
        for (int s = 0; s < 2; ++s) {
            const std::size_t n = sizes[s];
            const std::vector<Box2> boxes = variedHatch(n, across == 1, rng);
            BoxIndex index(boxes);
            const double h = 0.1 * static_cast<double>(n);
            const int queries = 20000;
            std::size_t hits = 0;
            for (int q = 0; q < queries; ++q) {
                const double u = rng.uniform(0, 10);
                const double v = rng.uniform(0, h);
                const Vec2 p = across == 1 ? Vec2{-v, u} : Vec2{u, v};
                index.query(Box2{p - Vec2{0.0165, 0.0165}, p + Vec2{0.0165, 0.0165}},
                            [&](std::size_t) { ++hits; });
            }
            perQuery[across][s] = static_cast<double>(index.work()) / queries;
            CAPTURE(across, n, perQuery[across][s], hits, index.treeCount());
            CHECK(perQuery[across][s] < 30.0);
            CHECK(index.treeCount() == 0);
        }
        CHECK(perQuery[across][1] < 1.5 * perQuery[across][0]);
    }
    // Crowded: the hatch of family 7 (pitch 0.004) with one box at 1e5.
    for (const std::size_t n : {std::size_t{2000}, std::size_t{32000}}) {
        const std::vector<Box2> boxes = family(7, n, rng);
        BoxIndex index(boxes);
        const int queries = 20000;
        for (int q = 0; q < queries; ++q) {
            const Vec2 p{rng.uniform(0, 10), rng.uniform(0, 0.004 * static_cast<double>(n))};
            index.query(Box2{p - Vec2{0.0165, 0.0165}, p + Vec2{0.0165, 0.0165}}, [](std::size_t) {});
        }
        const double work = static_cast<double>(index.work()) / queries;
        CAPTURE(n, work, index.treeCount());
        CHECK(work < 20.0 * std::log2(static_cast<double>(n)));
    }
}
