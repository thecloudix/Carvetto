// Randomized gouge-free property test for the pocket toolpath (T-0121 AC6, "as in T-0120"): no
// cutting move below the surface ever comes closer than tool radius + allowance to the uncut
// side of any input contour (outer walls and island walls), with exact segment/segment
// distances from pocket_oracle.hpp. Random star-shaped pockets with up to three star-shaped
// islands, random tools (one clearance + optional rest tool), strategies, angles, stepovers,
// allowances, ramps and depths; fixed seeds (SplitMix64) so every platform draws the same cases.
//
// Inputs are kept to straight-line polygons whose vertices are at least 2 mm from each other's
// edges, away from the sub-tolerance cases of the offset code (T-0009/T-0017 territory).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_oracle.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Contour;
using carvetto::geometry::kTwoPi;
using carvetto::geometry::Polyline;
using carvetto::geometry::Vec2;
using carvetto::test::SplitMix;
namespace orc = carvetto::test::pocket;

namespace {

orc::Poly star(SplitMix& rng, Vec2 c, double rMin, double rMax, int n) {
    orc::Poly p;
    const double phase = rng.uniform(0.0, kTwoPi);
    for (int i = 0; i < n; ++i) {
        const double a = phase + kTwoPi * (i + rng.uniform(-0.3, 0.3)) / n;
        const double r = rng.uniform(rMin, rMax);
        p.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
    }
    return p;
}

bool clearOf(const std::vector<orc::Poly>& polys, const orc::Poly& cand, double gap) {
    for (const Vec2 v : cand) {
        if (orc::boundaryDist(polys, v) < gap) {
            return false;
        }
    }
    for (const orc::Poly& q : polys) {
        for (const Vec2 v : q) {
            if (orc::boundaryDist({cand}, v) < gap) {
                return false;
            }
        }
    }
    return true;
}

struct Case {
    std::vector<orc::Poly> polys;  // outer first, then islands
    std::vector<PocketTool> tools;
    PocketParams params;
};

Case makeCase(std::uint64_t seed) {
    SplitMix rng{seed};
    Case c;
    const orc::Poly outer = star(rng, {50, 50}, 15.0, 40.0, 5 + static_cast<int>(rng.next() % 8));
    c.polys.push_back(outer);
    const int islands = static_cast<int>(rng.next() % 4);
    for (int k = 0, tries = 0; k < islands && tries < 50; ++tries) {
        const Vec2 at{rng.uniform(15, 85), rng.uniform(15, 85)};
        if (!orc::insideEvenOdd({outer}, at)) {
            continue;
        }
        orc::Poly isl = star(rng, at, 1.5, 6.0, 3 + static_cast<int>(rng.next() % 5));
        bool inside = true;
        for (const Vec2 v : isl) {
            inside = inside && orc::insideEvenOdd({outer}, v);
        }
        if (inside && clearOf(c.polys, isl, 2.0)) {
            c.polys.push_back(isl);
            ++k;
        }
    }
    auto makeTool = [&](double d, int number) {
        PocketTool t;
        t.tool.name = "T" + std::to_string(number);
        t.tool.toolNumber = number;
        t.tool.diameter = d;
        t.tool.feedRate = 1200;
        t.tool.plungeRate = 400;
        t.stepover = rng.uniform(0.1, 1.0) * d;
        t.passDepth = rng.uniform(1.0, 6.0);
        return t;
    };
    const double d1 = rng.uniform(3.0, 12.0);
    c.tools.push_back(makeTool(d1, 1));
    if (rng.next() % 2 == 0) {
        c.tools.push_back(makeTool(rng.uniform(1.0, 0.9 * d1), 2));
    }
    PocketParams& p = c.params;
    p.strategy = rng.next() % 2 == 0 ? PocketStrategy::Offset : PocketStrategy::Raster;
    p.direction = rng.next() % 2 == 0 ? CutDirection::Climb : CutDirection::Conventional;
    p.rasterAngle = rng.uniform(0.0, 3.2);
    p.profilePass = static_cast<ProfilePass>(rng.next() % 3);
    p.allowance = rng.next() % 2 == 0 ? 0.0 : rng.uniform(0.0, 1.0);
    p.rampPlunge = rng.next() % 2 == 0;
    p.rampDistance = rng.uniform(1.0, 15.0);
    p.startDepth = rng.next() % 3 == 0 ? rng.uniform(0.0, 2.0) : 0.0;
    p.cutDepth = rng.uniform(1.0, 10.0);
    p.maintainExactStep = rng.next() % 2 == 0;
    return c;
}

}  // namespace

TEST_CASE("AC6: randomized pockets never gouge walls or islands", "[toolpath][pocket][gouge]") {
    std::size_t nonEmpty = 0, withIslands = 0, withRest = 0;
    for (std::uint64_t seed = 1; seed <= 60; ++seed) {
        const Case c = makeCase(0x7121'0000ULL + seed);
        std::vector<Contour> input;
        for (std::size_t i = 0; i < c.polys.size(); ++i) {
            Polyline pl{c.polys[i], true};
            if (i % 2 == 1) {
                pl.reverse();  // orientation must not matter (even-odd region)
            }
            input.push_back(Contour::fromPolyline(pl));
        }
        const PocketResult res = pocketToolpath(input, c.tools, c.params);
        withIslands += c.polys.size() > 1 ? 1u : 0u;
        for (std::size_t ti = 0; ti < res.toolpaths.size(); ++ti) {
            INFO("seed " << seed << " tool " << ti << " d=" << c.tools[ti].tool.diameter << " strategy "
                         << static_cast<int>(c.params.strategy) << " allowance " << c.params.allowance);
            const Toolpath& tp = res.toolpaths[ti];
            if (tp.isEmpty()) {
                continue;
            }
            ++nonEmpty;
            withRest += ti > 0 ? 1u : 0u;
            const double surface = -c.params.startDepth;
            const orc::GougeReport g = orc::gougeCheck(tp, c.polys, surface, c.params.safeZ);
            CHECK(g.checkedMoves > 0);
            CHECK(g.allInside);
            CHECK(g.rapidsAtSafeZ);
            CHECK(g.minDistance >= 0.5 * c.tools[ti].tool.diameter + c.params.allowance - 1e-9);
            // Shape of the move list: starts with a rapid at safe Z, ends retracted, plunges and
            // retracts are vertical, nothing goes below the bottom of the pocket.
            CHECK(tp.moves.front().kind == MoveKind::Rapid);
            CHECK(tp.moves.back().kind == MoveKind::Retract);
            CHECK(tp.moves.back().to.z == c.params.safeZ);
            for (std::size_t i = 0; i < tp.moves.size(); ++i) {
                const Move& mv = tp.moves[i];
                CHECK(mv.to.z >= -(c.params.startDepth + c.params.cutDepth) - 1e-9);
                CHECK_FALSE(isArc(mv.kind));
                if (mv.kind == MoveKind::Plunge || mv.kind == MoveKind::Retract) {
                    CHECK(distance(mv.to.xy(), tp.startOf(i).xy()) == 0.0);
                }
            }
        }
    }
    // The generator must actually exercise islands, rest tools and many toolpaths.
    CHECK(nonEmpty >= 60);
    CHECK(withIslands >= 20);
    CHECK(withRest >= 10);
}
