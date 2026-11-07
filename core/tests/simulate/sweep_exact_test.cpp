// T-0222 AC1: exact oracles for the swept surface. Level segments, plunges and planar arcs are
// compared with the closed-form distance from a cell centre to the path (the surface is
// z + h(distance)); sloped segments and helices with the brute-force sampled oracle in oracle.hpp.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;

namespace {

constexpr double kTol = 1e-6;  // float storage of heights up to ~10 mm

struct Worst {
    double dev{0.0};
    std::size_t cells{0};
};

/// Largest |cell - (z + h(d))| over cells whose distance d is not within 1e-9 of the tool radius.
template <class DistFn>
Worst compareAnalytic(const Heightfield& f, const Profile& tool, double z, DistFn dist) {
    Worst w;
    for (std::size_t j = 0; j < f.ny(); ++j) {
        for (std::size_t i = 0; i < f.nx(); ++i) {
            const geometry::Vec2 c = f.centreOf(i, j);
            const double d = dist(c.x, c.y);
            if (std::fabs(d - tool.radius) < 1e-9) {
                continue;
            }
            double expect = f.topZ();
            if (d < tool.radius) {
                expect = std::min(f.topZ(), std::max(f.bottomZ(), z + tool.at(d)));
            }
            w.dev = std::max(w.dev, std::fabs(f.height(i, j) - expect));
            ++w.cells;
        }
    }
    return w;
}

double distToArc(double px, double py, double cx, double cy, double rho, double a0, double sweep) {
    // sweep signed; angle of the point relative to the start, measured in the arc's direction
    const double ang = std::atan2(py - cy, px - cx);
    double rel = (sweep >= 0 ? ang - a0 : a0 - ang);
    while (rel < 0) rel += 2 * kPi;
    while (rel >= 2 * kPi) rel -= 2 * kPi;
    const double absSweep = std::fabs(sweep);
    if (rel <= absSweep || absSweep >= 2 * kPi - 1e-12) {
        return std::fabs(std::hypot(px - cx, py - cy) - rho);
    }
    const double ea = a0 + sweep;
    return std::min(std::hypot(px - (cx + rho * std::cos(a0)), py - (cy + rho * std::sin(a0))),
                    std::hypot(px - (cx + rho * std::cos(ea)), py - (cy + rho * std::sin(ea))));
}

}  // namespace

TEST_CASE("simulate: level feed move equals the analytic swept surface for every tool", "[simulate][oracle][segment]") {
    const Stock stock = stockOf(24, 16, 0.25);
    double worstAll = 0.0;
    for (const NamedTool& t : allTools()) {
        INFO(t.name);
        const double z = -1.5;
        const Simulator sim = run(stock, t.shape, {4, 6, 5}, {Move::plunge({4, 6, z}), Move::feed({19.3, 9.1, z})});
        REQUIRE(sim.ok());
        const Heightfield& f = sim.heightfield();
        // The plunge leg is a vertical stamp at (4, 6); the swept surface is the union of both.
        const Worst w = compareAnalytic(f, t.profile, z, [&](double x, double y) {
            return distToSegment(x, y, 4, 6, 19.3, 9.1);
        });
        INFO("cells " << w.cells << " max deviation " << w.dev);
        CHECK(w.dev <= kTol);
        CHECK(f.cutCells() > 0);
        worstAll = std::max(worstAll, w.dev);
    }
    std::cout << "[oracle] level feed (8 tools): max |cell - analytic| " << worstAll << " mm\n";
}

TEST_CASE("simulate: plunge into the stock stamps the tool profile", "[simulate][oracle][plunge]") {
    const Stock stock = stockOf(16, 16, 0.2);
    double worstAll = 0.0;
    for (const NamedTool& t : allTools()) {
        INFO(t.name);
        const Simulator sim = run(stock, t.shape, {8.05, 7.9, 5}, {Move::plunge({8.05, 7.9, -3.0})});
        const Worst w = compareAnalytic(sim.heightfield(), t.profile, -3.0,
                                        [&](double x, double y) { return std::hypot(x - 8.05, y - 7.9); });
        INFO("max deviation " << w.dev);
        CHECK(w.dev <= kTol);
        worstAll = std::max(worstAll, w.dev);
        // A retract after it leaves the same hole.
        const Simulator sim2 = run(stock, t.shape, {8.05, 7.9, 5},
                                   {Move::plunge({8.05, 7.9, -3.0}), Move::retract({8.05, 7.9, 5})});
        CHECK(sim2.heightfield().heights() == sim.heightfield().heights());
    }
    std::cout << "[oracle] plunge (8 tools): max |cell - analytic| " << worstAll << " mm\n";
}

TEST_CASE("simulate: planar arcs equal the analytic swept surface", "[simulate][oracle][arc]") {
    const Stock stock = stockOf(30, 30, 0.25);
    struct ArcCase {
        const char* name;
        bool ccw;
        double cx, cy, rho, a0Deg, sweepDeg;  // sweepDeg magnitude; 360 = full circle
    };
    const ArcCase cases[] = {
        {"quarter ccw", true, 15, 15, 8, 0, 90},       {"quarter cw", false, 15, 15, 8, 0, 90},
        {"half ccw", true, 15, 15, 9, 30, 180},        {"half cw", false, 15, 15, 9, 30, 180},
        {"three quarters ccw", true, 14, 16, 7, 200, 270}, {"three quarters cw", false, 14, 16, 7, 200, 270},
        {"full ccw", true, 15, 15, 6, 77, 360},        {"full cw", false, 15, 15, 6, 77, 360},
        {"small ccw", true, 15, 15, 0.4, 10, 300},     {"tiny sweep cw", false, 15, 15, 8, 100, 4},
        {"just over a quarter", true, 15, 15, 8, 89, 91}, {"exact quarters cw", false, 15, 15, 10, 270, 180},
    };
    double worstAll = 0.0;
    for (const NamedTool& t : allTools()) {
        for (const ArcCase& a : cases) {
            INFO(t.name << " / " << a.name);
            const double z = -1.25;
            const double a0 = a.a0Deg * kDeg;
            const double sweep = (a.ccw ? 1 : -1) * a.sweepDeg * kDeg;
            const Vec3 s{a.cx + a.rho * std::cos(a0), a.cy + a.rho * std::sin(a0), z};
            Vec3 e{a.cx + a.rho * std::cos(a0 + sweep), a.cy + a.rho * std::sin(a0 + sweep), z};
            if (a.sweepDeg == 360) {
                e = s;
            }
            const Move m = a.ccw ? Move::arcCCW(e, {a.cx, a.cy}) : Move::arcCW(e, {a.cx, a.cy});
            const Simulator sim = run(stock, t.shape, {s.x, s.y, 5}, {Move::plunge(s), m});
            REQUIRE(sim.ok());
            const Worst w = compareAnalytic(sim.heightfield(), t.profile, z, [&](double x, double y) {
                double d = distToArc(x, y, a.cx, a.cy, a.rho, a0, sweep);
                // the plunge at the start is inside the swept region of the arc already
                return d;
            });
            INFO("max deviation " << w.dev);
            CHECK(w.dev <= kTol);
            worstAll = std::max(worstAll, w.dev);
        }
    }
    std::cout << "[oracle] planar arcs (8 tools x 12 arcs, CW/CCW, full circles): max |cell - analytic| " << worstAll
              << " mm\n";
}

TEST_CASE("simulate: sloped segments and helices stay inside the sampled oracle's bounds", "[simulate][oracle][helix]") {
    // Random ramps (shallow to steeper than the V-bit cone), arcs and helices with CW/CCW, compared
    // with the brute-force sampled envelope (hi >= exact >= lo, see oracle.hpp). Helices are swept
    // as inscribed chords (sagitta <= cell / 64), so the envelope is widened by that.
    Rng rng(20260607);
    const Stock stock = stockOf(24, 24, 0.25);
    const auto tools = allTools();
    double worstBelow = 0, worstAbove = 0, worstGap = 0;
    std::size_t cases = 0;
    for (int iter = 0; iter < 48; ++iter) {
        const NamedTool& t = tools[static_cast<std::size_t>(iter) % tools.size()];
        const double sx0 = 5 + 14 * rng.uniform();
        const double sy0 = 5 + 14 * rng.uniform();
        const double z0 = -4.0 * rng.uniform();
        std::vector<Move> moves;
        std::vector<Move> cuts;
        Vec3 start{sx0, sy0, z0};
        const int kind = iter % 3;
        if (kind == 0) {
            const double ang = 2 * kPi * rng.uniform();
            const double len = 2 + 8 * rng.uniform();
            const double slopes[] = {0.05, 0.4, 1.0, 3.0, 8.0};
            const double slope = slopes[(iter / 3) % 5] * (rng.uniform() < 0.5 ? -1 : 1);
            Vec3 e{sx0 + len * std::cos(ang), sy0 + len * std::sin(ang), z0 + slope * len};
            e.z = std::max(-9.0, std::min(1.0, e.z));
            cuts.push_back(Move::feed(e));
        } else {
            const double rho = 1.0 + 5 * rng.uniform();
            const double a0 = 2 * kPi * rng.uniform();
            const bool ccw = (iter / 3) % 2 == 0;
            const double sw = (0.3 + 1.9 * rng.uniform()) * kPi * (ccw ? 1 : -1);
            const double cx = sx0 - rho * std::cos(a0);
            const double cy = sy0 - rho * std::sin(a0);
            const double dz = kind == 1 ? 0.0 : -3.0 * rng.uniform();
            const Vec3 e{cx + rho * std::cos(a0 + sw), cy + rho * std::sin(a0 + sw), z0 + dz};
            cuts.push_back(ccw ? Move::arcCCW(e, {cx, cy}) : Move::arcCW(e, {cx, cy}));
        }
        moves.push_back(Move::plunge(start));
        for (const Move& c : cuts) moves.push_back(c);
        const Simulator sim = run(stock, t.shape, {sx0, sy0, 5}, moves);
        REQUIRE(sim.ok());
        Samples smp = sampleMoves(moves, {sx0, sy0, 5}, 0.25 / 24);
        smp.buildIndex(t.profile.radius + smp.spacingXY);
        // chord approximation of helices: widen by the sagitta bound (cell / 64) in the tolerance
        const double slack = kind == 2 ? 0.25 / 64 * 6.0 : 0.0;  // x steepest cone slope used here (V 20: 5.7)
        const Deviation d = compare(sim.heightfield(), smp, t.profile, kTol + slack, 1);
        INFO(t.name << " iter " << iter << " below " << d.belowLo << " above " << d.aboveHi << " gap " << d.gap);
        CHECK(d.violations == 0);
        worstBelow = std::max(worstBelow, d.belowLo);
        worstAbove = std::max(worstAbove, d.aboveHi);
        worstGap = std::max(worstGap, d.gap);
        ++cases;
    }
    std::cout << "[oracle] sloped/arc/helix cases " << cases << ": max below lo " << worstBelow << ", max above hi "
              << worstAbove << ", oracle band " << worstGap << " mm\n";
}

TEST_CASE("simulate: overlapping passes give the minimum of the heights", "[simulate][oracle][min]") {
    const Stock stock = stockOf(24, 16, 0.25);
    for (const NamedTool& t : allTools()) {
        INFO(t.name);
        const std::vector<Move> a{Move::plunge({3, 5, -1.0}), Move::feed({20, 9, -2.0})};
        const std::vector<Move> b{Move::plunge({6, 12, -3.0}), Move::feed({17, 2, -0.5})};
        const Simulator sa = run(stock, t.shape, {3, 5, 5}, a);
        const Simulator sb = run(stock, t.shape, {6, 12, 5}, b);
        std::vector<Move> both = a;
        both.push_back(Move::retract({20, 9, 5}));
        both.push_back(Move::rapid({6, 12, 5}));
        both.insert(both.end(), b.begin(), b.end());
        const Simulator sab = run(stock, t.shape, {3, 5, 5}, both);
        std::vector<Move> rev = b;
        rev.push_back(Move::retract({17, 2, 5}));
        rev.push_back(Move::rapid({3, 5, 5}));
        rev.insert(rev.end(), a.begin(), a.end());
        const Simulator sba = run(stock, t.shape, {6, 12, 5}, rev);
        const auto& ha = sa.heightfield().heights();
        const auto& hb = sb.heightfield().heights();
        const auto& hab = sab.heightfield().heights();
        const auto& hba = sba.heightfield().heights();
        REQUIRE(ha.size() == hab.size());
        std::size_t differing = 0;
        for (std::size_t k = 0; k < ha.size(); ++k) {
            if (hab[k] != std::min(ha[k], hb[k]) || hba[k] != hab[k]) {
                ++differing;
            }
        }
        CHECK(differing == 0);
    }
}

TEST_CASE("simulate: cut state, pass ids and the table floor", "[simulate][ids]") {
    const Stock stock = stockOf(20, 12, 0.25, 4.0);
    const ToolShape flat = *ToolShape::flatEndMill(2.0);
    Simulator sim(stock);
    Toolpath a;
    a.append(Move::rapid({4, 6, 5}));
    a.append(Move::plunge({4, 6, -1}));
    a.append(Move::feed({12, 6, -1}));
    Toolpath b;
    b.append(Move::rapid({8, 6, 5}));
    b.append(Move::plunge({8, 6, -3}));
    b.append(Move::feed({16, 6, -3}));
    b.append(Move::plunge({16, 6, -7}));  // through the 4 mm stock: floors at the table
    REQUIRE(sim.addToolpath(a, flat, 7));
    REQUIRE(sim.addToolpath(b, flat, 9));
    REQUIRE(sim.advanceAll());
    const Heightfield& f = sim.heightfield();
    std::size_t i = 0, j = 0;
    REQUIRE(f.cellAt({5.1, 6.1}, i, j));
    CHECK(f.isCut(i, j));
    CHECK(f.passId(i, j) == 7);
    CHECK(f.height(i, j) == -1.0f);
    REQUIRE(f.cellAt({10.1, 6.1}, i, j));  // overlap: deeper second pass wins and takes the id
    CHECK(f.height(i, j) == -3.0f);
    CHECK(f.passId(i, j) == 9);
    REQUIRE(f.cellAt({16.1, 6.1}, i, j));
    CHECK(f.height(i, j) == -4.0f);  // table at -4
    REQUIRE(f.cellAt({1.1, 1.1}, i, j));
    CHECK_FALSE(f.isCut(i, j));
    CHECK(f.passId(i, j) == 0);
    CHECK(f.height(i, j) == 0.0f);
    std::size_t cut = 0;
    for (std::size_t q = 0; q < f.nx() * f.ny(); ++q) cut += f.cutStates()[q];
    CHECK(cut == f.cutCells());
}
