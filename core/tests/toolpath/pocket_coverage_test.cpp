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

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_oracle.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::kPi;
using carvetto::geometry::Vec2;
namespace orc = carvetto::test::pocket;

namespace {

constexpr double kBand = 0.05;

PocketTool tool(double diameter, double stepoverFraction, double passDepth, int number = 1) {
    PocketTool t;
    t.tool.name = "End Mill (" + std::to_string(diameter) + " mm)";
    t.tool.toolNumber = number;
    t.tool.diameter = diameter;
    t.tool.feedRate = 1000.0;
    t.tool.plungeRate = 300.0;
    t.tool.spindleSpeed = 18000.0;
    t.stepover = stepoverFraction * diameter;
    t.passDepth = passDepth;
    return t;
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
