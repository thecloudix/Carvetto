// Coverage and clearance tests for the pocket toolpath (T-0121 AC1, AC2, AC4). Every check
// rasterises the swept tool disc at 0.1 mm with the independent code in pocket_oracle.hpp and
// compares it with the analytically known reachable area.
//
// Tolerance band: cells whose centre is within kBand of the boundary of the reachable area are
// not judged either way. The pocket keeps the tool centre tolerance.chordal (0.01 mm) further
// from the walls than radius + allowance (ADR-0017), so a skin of at most ~0.015 mm stays on
// the walls; kBand (0.05 mm, half a cell) covers that and the polygon error.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_oracle.hpp"
#include "pocket_test_util.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::kPi;
using carvetto::geometry::Vec2;
namespace orc = carvetto::test::pocket;

namespace {

constexpr double kBand = 0.05;

PocketTool tool(double diameter, double stepoverFraction, double passDepth, int number = 1) {
    return orc::endMill("End Mill (" + std::to_string(diameter) + " mm)", number, diameter,
                        stepoverFraction * diameter, passDepth);
}

/// Signed reach depth of p in a W×H rectangle at the origin for a tool of radius r: >= 0 where
/// the tool can reach (inside, outside the four corner zones), negative elsewhere.
double rectReach(Vec2 p, double w, double h, double r) {
    const double cx = std::clamp(p.x, r, w - r);
    const double cy = std::clamp(p.y, r, h - r);
    if (p.x < 0 || p.y < 0 || p.x > w || p.y > h) {
        return -orc::boundaryDist({orc::rectPoly(0, 0, w, h)}, p);
    }
    if ((p.x < r || p.x > w - r) && (p.y < r || p.y > h - r)) {
        return r - std::hypot(p.x - cx, p.y - cy);  // corner zone: distance to the corner centre
    }
    return std::min({p.x, p.y, w - p.x, h - p.y});
}

struct Coverage {
    std::size_t mustCover{0};
    std::size_t missed{0};
    std::size_t mustNot{0};
    std::size_t gouged{0};
};

Coverage rectCoverage(const Toolpath& tp, double w, double h, double r, double z) {
    orc::Grid g(-2, -2, w + 2, h + 2, 0.1);
    orc::sweepFloor(g, tp, r, z);
    Coverage c;
    for (int j = 0; j < g.ny; ++j) {
        for (int i = 0; i < g.nx; ++i) {
            const double reach = rectReach(g.centre(i, j), w, h, r);
            if (reach >= kBand) {
                ++c.mustCover;
                c.missed += g.at(i, j) ? 0u : 1u;
            } else if (reach <= -kBand) {
                ++c.mustNot;
                c.gouged += g.at(i, j) ? 1u : 0u;
            }
        }
    }
    return c;
}

PocketParams baseParams() {
    PocketParams p;
    p.cutDepth = 6.0;
    p.safeZ = 5.0;
    return p;
}

}  // namespace

TEST_CASE("AC1: 50x30 rectangle, 6 mm tool, 40% stepover: full coverage, nothing outside",
          "[toolpath][pocket][coverage]") {
    const std::vector<Contour> rect{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{50, 30}})};
    struct Case {
        const char* name;
        PocketStrategy strategy;
        CutDirection dir;
        ProfilePass profile;
        double angle;
        bool ramp;
    };
    const Case cases[] = {
        {"offset climb", PocketStrategy::Offset, CutDirection::Climb, ProfilePass::Last, 0.0, false},
        {"offset conventional + ramp", PocketStrategy::Offset, CutDirection::Conventional, ProfilePass::None,
         0.0, true},
        {"offset climb, boundary first + ramp", PocketStrategy::Offset, CutDirection::Climb,
         ProfilePass::First, 0.0, true},
        {"raster 0 + last profile", PocketStrategy::Raster, CutDirection::Climb, ProfilePass::Last, 0.0,
         false},
        {"raster 45 + first profile + ramp", PocketStrategy::Raster, CutDirection::Climb, ProfilePass::First,
         kPi / 4, true},
        {"raster 90 + last profile", PocketStrategy::Raster, CutDirection::Conventional, ProfilePass::Last,
         kPi / 2, false},
    };
    for (const Case& cs : cases) {
        INFO(cs.name);
        PocketParams p = baseParams();
        p.strategy = cs.strategy;
        p.direction = cs.dir;
        p.profilePass = cs.profile;
        p.rasterAngle = cs.angle;
        p.rampPlunge = cs.ramp;
        const PocketResult res = pocketToolpath(rect, {tool(6.0, 0.4, 3.0)}, p);
        REQUIRE(res.toolpaths.size() == 1);
        const Toolpath& tp = res.toolpaths[0];
        REQUIRE_FALSE(tp.isEmpty());
        for (double z : {-3.0, -6.0}) {
            INFO("level " << z);
            const Coverage c = rectCoverage(tp, 50, 30, 3.0, z);
            CHECK(c.mustCover > 130000);
            CHECK(c.missed == 0);
            CHECK(c.mustNot > 0);
            CHECK(c.gouged == 0);
        }
    }
}

TEST_CASE("AC1 (stepover > radius): extra rings cover the middle of a narrow pocket",
          "[toolpath][pocket][coverage]") {
    // Centre region half-height 7.5 - 3.01 = 4.49: the second ring (inset 6) does not exist and
    // points 3..4.49 deep stay uncovered unless the residual ring is added.
    const std::vector<Contour> rect{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{40, 15}})};
    for (double frac : {0.8, 1.0}) {
        INFO("stepover fraction " << frac);
        PocketParams p = baseParams();
        p.cutDepth = 3.0;
        const PocketResult res = pocketToolpath(rect, {tool(6.0, frac, 3.0)}, p);
        const Coverage c = rectCoverage(res.toolpaths[0], 40, 15, 3.0, -3.0);
        CHECK(c.missed == 0);
        CHECK(c.gouged == 0);
    }
}

TEST_CASE("AC1 regression: a thin uncut strip in a narrow pocket is not discarded as a sliver",
          "[toolpath][pocket][coverage]") {
    // Verifier repro (T-0121 round 2): 60 x 12.08, 6 mm tool, stepover 6. The part of the centre
    // region deeper than r is 0.06 mm tall, which the old 5 x chordal opening threw away, leaving
    // a 0.06 mm strip along y = 6.04 (480 cells at 0.1 mm).
    for (double w : {12.0, 12.05, 12.08, 12.12, 12.15, 12.3, 13.0}) {
        for (double frac : {0.6, 0.9, 1.0}) {
            INFO("width " << w << " stepover fraction " << frac);
            PocketParams p = baseParams();
            p.cutDepth = 1.0;
            const PocketResult res = pocketToolpath({Contour::rectangle(Box2{Vec2{0, 0}, Vec2{60, w}})},
                                                    {tool(6.0, frac, 1.0)}, p);
            const Coverage c = rectCoverage(res.toolpaths[0], 60, w, 3.0, -1.0);
            CHECK(c.mustCover > 0);
            CHECK(c.missed == 0);
            CHECK(c.gouged == 0);
        }
    }
}

TEST_CASE("AC4 regression: thin rest material at a coarse chordal tolerance is still cut",
          "[toolpath][pocket][coverage][rest]") {
    // Verifier repro (seed 31493, chordal 0.049): a rest tool only slightly smaller than the
    // clearance tool leaves a ~0.08 mm band in each corner, which the old 5 x chordal opening
    // (0.245 mm) discarded. With the band kept the two tools together cover everything the
    // smaller one can reach.
    PocketParams p = baseParams();
    p.cutDepth = 2.0;
    p.tolerance.chordal = 0.049;
    const double r1 = 2.8;
    const PocketResult res = pocketToolpath({Contour::rectangle(Box2{Vec2{0, 0}, Vec2{40, 25}})},
                                            {tool(6.0, 0.4, 2.0, 1), tool(2 * r1, 0.4, 2.0, 2)}, p);
    REQUIRE(res.toolpaths.size() == 2);
    CHECK_FALSE(res.plans[1].restMaterial.empty());
    REQUIRE_FALSE(res.toolpaths[1].isEmpty());
    // Reach depth of the rest tool: r1 minus the distance to its centre region, the rectangle
    // inset by r1 + margin (the margin is one chordal tolerance; the region is exact for lines).
    const double m = p.tolerance.chordal;
    auto restReach = [&](Vec2 q) {
        const double cx = std::clamp(q.x, r1 + m, 40 - r1 - m);
        const double cy = std::clamp(q.y, r1 + m, 25 - r1 - m);
        return r1 - std::hypot(q.x - cx, q.y - cy);
    };
    const double band = 0.02;
    auto missedCells = [&](bool withRest) {
        orc::Grid g(-1, -1, 41, 26, 0.05);
        orc::sweepFloor(g, res.toolpaths[0], 3.0, -2.0);
        if (withRest) {
            orc::sweepFloor(g, res.toolpaths[1], r1, -2.0);
        }
        std::size_t must = 0, missed = 0;
        for (int j = 0; j < g.ny; ++j) {
            for (int i = 0; i < g.nx; ++i) {
                if (restReach(g.centre(i, j)) >= band) {
                    ++must;
                    missed += g.at(i, j) ? 0u : 1u;
                }
            }
        }
        CHECK(must > 350000);
        return missed;
    };
    CHECK(missedCells(true) == 0);
    CHECK(missedCells(false) > 20);  // the band is real: the clearance tool alone leaves it
}

TEST_CASE("AC2: a 20x10 island stays uncut with at least the tool radius of clearance",
          "[toolpath][pocket][coverage][island]") {
    const std::vector<Contour> shape{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{50, 30}}),
                                     Contour::rectangle(Box2{Vec2{15, 10}, Vec2{35, 20}})};
    const orc::Poly island = orc::rectPoly(15, 10, 35, 20);
    const std::vector<orc::Poly> boundary{orc::rectPoly(0, 0, 50, 30), island};
    for (PocketStrategy strategy : {PocketStrategy::Offset, PocketStrategy::Raster}) {
        for (double allowance : {0.0, 0.5}) {
            INFO("strategy " << static_cast<int>(strategy) << " allowance " << allowance);
            PocketParams p = baseParams();
            p.strategy = strategy;
            p.rasterAngle = kPi / 6;
            p.allowance = allowance;
            p.rampPlunge = true;
            const PocketResult res = pocketToolpath(shape, {tool(6.0, 0.4, 3.0)}, p);
            const Toolpath& tp = res.toolpaths[0];
            REQUIRE_FALSE(tp.isEmpty());

            // Exact: every move below the surface keeps >= radius + allowance from every wall.
            const orc::GougeReport g = orc::gougeCheck(tp, boundary, 0.0, p.safeZ);
            CHECK(g.allInside);
            CHECK(g.rapidsAtSafeZ);
            CHECK(g.minDistance >= 3.0 + allowance);

            // Rasterised: island cells never covered; everything reachable covered.
            orc::Grid grid(-2, -2, 52, 32, 0.1);
            orc::sweepFloor(grid, tp, 3.0, -6.0);
            std::size_t islandCells = 0, islandCut = 0, missed = 0, must = 0;
            for (int j = 0; j < grid.ny; ++j) {
                for (int i = 0; i < grid.nx; ++i) {
                    const Vec2 c = grid.centre(i, j);
                    const bool inIsland = orc::insideEvenOdd({island}, c);
                    const double islandDist = orc::boundaryDist({island}, c);
                    if (inIsland || islandDist < allowance - kBand) {
                        ++islandCells;
                        islandCut += grid.at(i, j);
                        continue;
                    }
                    // The island is convex and every channel is wider than the tool, so every
                    // point outside it (by more than the allowance) is reachable.
                    if (rectReach(c - Vec2{allowance, allowance}, 50 - 2 * allowance, 30 - 2 * allowance,
                                  3.0) >= kBand &&
                        islandDist >= allowance + kBand) {
                        ++must;
                        missed += grid.at(i, j) ? 0u : 1u;
                    }
                }
            }
            CHECK(islandCells >= 20000);
            CHECK(islandCut == 0);
            CHECK(must > 100000);
            CHECK(missed == 0);
        }
    }
}

TEST_CASE("AC4: rest machining (12 mm then 3 mm) touches only the rest areas",
          "[toolpath][pocket][coverage][rest]") {
    // 50x30 rectangle with an 8 mm wide, 20 mm long slot on the right that the 12 mm tool
    // cannot enter.
    const orc::Poly poly{{0, 0}, {50, 0}, {50, 11}, {70, 11}, {70, 19}, {50, 19}, {50, 30}, {0, 30}};
    Contour shape = Contour::fromPolyline({poly, true});
    const std::vector<orc::Poly> boundary{poly};
    for (PocketStrategy strategy : {PocketStrategy::Offset, PocketStrategy::Raster}) {
        INFO("strategy " << static_cast<int>(strategy));
        PocketParams p = baseParams();
        p.strategy = strategy;
        p.rasterAngle = 0.3;
        const PocketResult res =
            pocketToolpath({shape}, {tool(12.0, 0.4, 6.0, 1), tool(3.0, 0.4, 6.0, 2)}, p);
        REQUIRE(res.toolpaths.size() == 2);
        const Toolpath& clear = res.toolpaths[0];
        const Toolpath& rest = res.toolpaths[1];
        REQUIRE_FALSE(clear.isEmpty());
        REQUIRE_FALSE(rest.isEmpty());
        CHECK(clear.name == "Pocket [" + clear.tool.name + "]");
        CHECK_FALSE(res.plans[1].restMaterial.empty());

        // Rasterised difference: material inside the pocket the clearance tool did not sweep.
        orc::Grid g0(-1, -1, 71, 31, 0.1);
        orc::sweepFloor(g0, clear, 6.0, -6.0);
        std::vector<Vec2> uncut;
        for (int j = 0; j < g0.ny; ++j) {
            for (int i = 0; i < g0.nx; ++i) {
                const Vec2 c = g0.centre(i, j);
                if (!g0.at(i, j) && orc::insideEvenOdd(boundary, c) &&
                    orc::boundaryDist(boundary, c) >= kBand) {
                    uncut.push_back(c);
                }
            }
        }
        REQUIRE(uncut.size() > 1000);

        // Every point of every rest cutting move below the surface lies within 6 mm (the
        // clearance tool radius) of that difference; in fact within the rest radius plus one link.
        // Uncut cells bucketed in 1 mm squares; nearest-cell search ring by ring.
        const double bx0 = -1.0, by0 = -1.0;
        const int bnx = 72, bny = 32;
        std::vector<std::vector<Vec2>> buckets(static_cast<std::size_t>(bnx * bny));
        auto bucketOf = [&](Vec2 q, int& bi, int& bj) {
            bi = std::clamp(static_cast<int>(std::floor(q.x - bx0)), 0, bnx - 1);
            bj = std::clamp(static_cast<int>(std::floor(q.y - by0)), 0, bny - 1);
        };
        for (const Vec2 u : uncut) {
            int bi = 0, bj = 0;
            bucketOf(u, bi, bj);
            buckets[static_cast<std::size_t>(bj * bnx + bi)].push_back(u);
        }
        auto nearestUncut = [&](Vec2 q) {
            int bi = 0, bj = 0;
            bucketOf(q, bi, bj);
            double best = 1e9;
            for (int k = 0; k < 80; ++k) {
                for (int j = bj - k; j <= bj + k; ++j) {
                    for (int i = bi - k; i <= bi + k; ++i) {
                        if (std::max(std::abs(i - bi), std::abs(j - bj)) != k || i < 0 || j < 0 || i >= bnx ||
                            j >= bny) {
                            continue;
                        }
                        for (const Vec2 u : buckets[static_cast<std::size_t>(j * bnx + i)]) {
                            best = std::min(best, std::hypot(u.x - q.x, u.y - q.y));
                        }
                    }
                }
                if (best <= static_cast<double>(k)) {
                    break;  // every bucket further out is at least k mm away
                }
            }
            return best;
        };
        double worst = 0.0;
        for (const orc::Seg3& s : orc::segments(rest)) {
            if (!isCutting(s.kind) || std::min(s.a.z, s.b.z) >= 0.0) {
                continue;
            }
            const double len = distance(s.a.xy(), s.b.xy());
            const int n = std::max(1, static_cast<int>(std::ceil(len / 0.1)));
            for (int k = 0; k <= n; ++k) {
                worst = std::max(worst, nearestUncut(lerp(s.a.xy(), s.b.xy(), static_cast<double>(k) / n)));
            }
        }
        INFO("worst distance of the rest path from uncut material: " << worst);
        CHECK(worst <= 6.0);
        CHECK(worst <= 1.5 + 6.0 / 2.0 + 0.2);

        // Together the two tools cover everything the 3 mm tool can reach (with margin), and
        // neither tool comes closer than its radius to a wall. Valid rest-tool centres (inside,
        // >= 1.5 + kBand from every wall) are precomputed on a 0.05 mm grid.
        orc::Grid valid(-1, -1, 71, 31, 0.05);
        for (int j = 0; j < valid.ny; ++j) {
            for (int i = 0; i < valid.nx; ++i) {
                const Vec2 c = valid.centre(i, j);
                valid.at(i, j) = static_cast<std::uint8_t>(orc::insideEvenOdd(boundary, c) &&
                                                           orc::boundaryDist(boundary, c) >= 1.5 + kBand);
            }
        }
        orc::Grid g1(-1, -1, 71, 31, 0.1);
        orc::sweepFloor(g1, rest, 1.5, -6.0);
        std::size_t judged = 0, missed = 0;
        for (const Vec2 u : uncut) {
            // Reachable with kBand to spare: a valid centre within 1.5 - kBand of u.
            bool reachable = false;
            const int ci = static_cast<int>(std::floor((u.x - valid.x0) / valid.h));
            const int cj = static_cast<int>(std::floor((u.y - valid.y0) / valid.h));
            for (int j = std::max(0, cj - 30); j <= std::min(valid.ny - 1, cj + 30) && !reachable; ++j) {
                for (int i = std::max(0, ci - 30); i <= std::min(valid.nx - 1, ci + 30) && !reachable; ++i) {
                    const Vec2 c = valid.centre(i, j);
                    reachable = valid.at(i, j) && std::hypot(c.x - u.x, c.y - u.y) <= 1.5 - kBand;
                }
            }
            if (!reachable) {
                continue;
            }
            ++judged;
            const int i = static_cast<int>(std::floor((u.x - g1.x0) / g1.h));
            const int j = static_cast<int>(std::floor((u.y - g1.y0) / g1.h));
            missed += g1.at(i, j) ? 0u : 1u;
        }
        CHECK(judged > 1000);
        CHECK(missed == 0);
        CHECK(orc::gougeCheck(clear, boundary, 0.0, p.safeZ).minDistance >= 6.0);
        CHECK(orc::gougeCheck(rest, boundary, 0.0, p.safeZ).minDistance >= 1.5);
    }
}

namespace {

/// Cells inside `polys` that a tool of radius r1 can reach (some centre inside, at least
/// r1 + margin + band from every wall, within r1 - band of the cell) but that neither toolpath
/// swept at depth z (`validCentres` counts the rest tool's valid centre cells). Independent of the pocket
/// code: grid sweep, even-odd test, edge distances.
std::size_t unreachedRest(const std::vector<orc::Poly>& polys, const Toolpath& t0, double r0,
                          const Toolpath& t1, double r1, double z, double margin, std::size_t& validCentres) {
    const double h = 0.1, band = 0.02;
    Box2 box;
    for (const orc::Poly& poly : polys) {
        for (const Vec2 q : poly) {
            box.expand(q);
        }
    }
    orc::Grid g(box.min.x - 1, box.min.y - 1, box.max.x + 1, box.max.y + 1, h);
    orc::sweepFloor(g, t0, r0, z);
    orc::sweepFloor(g, t1, r1, z);
    orc::Grid valid(box.min.x - 1, box.min.y - 1, box.max.x + 1, box.max.y + 1, h);
    for (int j = 0; j < valid.ny; ++j) {
        for (int i = 0; i < valid.nx; ++i) {
            const Vec2 c = valid.centre(i, j);
            valid.at(i, j) = static_cast<std::uint8_t>(orc::insideEvenOdd(polys, c) &&
                                                       orc::boundaryDist(polys, c) >= r1 + margin + band);
        }
    }
    const int win = static_cast<int>(std::ceil(r1 / h));
    std::size_t missed = 0;
    validCentres = 0;
    for (int j = 0; j < g.ny; ++j) {
        for (int i = 0; i < g.nx; ++i) {
            if (g.at(i, j)) {
                continue;  // swept by one of the tools
            }
            const Vec2 p = g.centre(i, j);
            if (!orc::insideEvenOdd(polys, p)) {
                continue;  // outside the pocket: nothing to cut
            }
            bool reachable = false;
            for (int dj = -win; dj <= win && !reachable; ++dj) {
                for (int di = -win; di <= win && !reachable; ++di) {
                    const int ii = i + di, jj = j + dj;
                    if (ii < 0 || jj < 0 || ii >= valid.nx || jj >= valid.ny || !valid.at(ii, jj)) {
                        continue;
                    }
                    const Vec2 c = valid.centre(ii, jj);
                    reachable = std::hypot(c.x - p.x, c.y - p.y) <= r1 - band;
                }
            }
            if (reachable) {
                ++missed;
            }
        }
    }
    for (int j = 0; j < valid.ny; ++j) {
        for (int i = 0; i < valid.nx; ++i) {
            validCentres += valid.at(i, j);
        }
    }
    return missed;
}

orc::Poly dumbbell(double w) {
    return {{0, 0},   {25, 0},  {25, 15 - w / 2}, {40, 15 - w / 2}, {40, 5},  {60, 5},
            {60, 25}, {40, 25}, {40, 15 + w / 2}, {25, 15 + w / 2}, {25, 30}, {0, 30}};
}

}  // namespace

TEST_CASE("AC4 regression: the 15 x 6 mm neck of a dumbbell is rest material and is cut",
          "[toolpath][pocket][coverage][rest]") {
    // Round-3 verifier repro: the opening of the rest material insets it slightly, and the
    // inside-out guard dropped the whole neck because one vertex near a cusp came out a little
    // too close; only the eight corners were left.
    for (double w : {6.0, 6.03, 6.05}) {
        for (PocketStrategy strategy : {PocketStrategy::Offset, PocketStrategy::Raster}) {
            INFO("neck width " << w << " strategy " << static_cast<int>(strategy));
            PocketParams p = baseParams();
            p.strategy = strategy;
            p.rasterAngle = 0.4;
            const orc::Poly poly = dumbbell(w);
            const PocketResult res = pocketToolpath({Contour::fromPolyline({poly, true})},
                                                    {tool(12.0, 0.4, 6.0, 1), tool(3.0, 0.4, 6.0, 2)}, p);
            bool neck = false;
            for (const Contour& c : res.plans[1].restMaterial) {
                const Box2 b = c.bounds();
                neck = neck || (b.min.x <= 25.5 && b.max.x >= 39.5 && b.height() > 5.0);
            }
            CHECK(neck);
            std::size_t judged = 0;
            CHECK(unreachedRest({poly}, res.toolpaths[0], 6.0, res.toolpaths[1], 1.5, -6.0,
                                p.tolerance.chordal, judged) == 0);
            CHECK(judged > 1000);
        }
    }
}

TEST_CASE("AC4 property: rest material the rest tool can reach is always cut",
          "[toolpath][pocket][coverage][rest]") {
    // Random dumbbells and slotted rectangles, both strategies, every profile pass except
    // Raster + None (see below), fixed seeds.
    carvetto::test::SplitMix rng{0x7121'0300ULL};
    for (int k = 0; k < 10; ++k) {
        orc::Poly poly;
        if (k % 2 == 0) {
            poly = dumbbell(rng.uniform(4.0, 9.0));
        } else {
            const double sw = rng.uniform(3.5, 8.0), sl = rng.uniform(5.0, 15.0), sy = rng.uniform(4.0, 20.0);
            poly = {{0, 0},        {40, 0},  {40, sy}, {40 + sl, sy}, {40 + sl, sy + sw},
                    {40, sy + sw}, {40, 30}, {0, 30}};
        }
        PocketParams p = baseParams();
        p.cutDepth = 3.0;
        p.strategy = rng.next() % 2 == 0 ? PocketStrategy::Offset : PocketStrategy::Raster;
        p.profilePass = static_cast<ProfilePass>(rng.next() % 3);
        if (p.strategy == PocketStrategy::Raster && p.profilePass == ProfilePass::None) {
            // Raster without a boundary pass leaves wall scallops for every tool by design (the
            // rest tool's own raster has no boundary pass either), so the property does not hold
            // there; its rest material is still taken from the actual sweep (pocket_test).
            p.profilePass = ProfilePass::Last;
        }
        p.rasterAngle = rng.uniform(0.0, 3.0);
        const double d0 = rng.uniform(8.0, 12.0), d1 = rng.uniform(2.0, 3.5);
        INFO("case " << k << " strategy " << static_cast<int>(p.strategy) << " pass "
                     << static_cast<int>(p.profilePass) << " d0 " << d0 << " d1 " << d1);
        const PocketResult res = pocketToolpath({Contour::fromPolyline({poly, true})},
                                                {tool(d0, 0.4, 3.0, 1), tool(d1, 0.4, 3.0, 2)}, p);
        REQUIRE(res.toolpaths.size() == 2);
        std::size_t judged = 0;
        CHECK(unreachedRest({poly}, res.toolpaths[0], d0 / 2, res.toolpaths[1], d1 / 2, -3.0,
                            p.tolerance.chordal, judged) == 0);
        for (std::size_t i = 0; i < 2; ++i) {
            const orc::GougeReport g = orc::gougeCheck(res.toolpaths[i], {poly}, 1e-9, p.safeZ);
            CHECK(g.allInside);
            CHECK(g.minDistance >= 0.5 * res.toolpaths[i].tool.diameter);
        }
    }
}
