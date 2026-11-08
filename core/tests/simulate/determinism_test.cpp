// T-0222 AC3: determinism. The heightfield of a fixed job has a fixed byte-for-byte hash; the same
// constant must hold at -O0 (Debug) and -O2 (Release) and on every compiler in CI. Only
// IEEE + - * / and sqrt are used on the path to a cell (det_math.hpp replaces sin/cos/atan2), the
// core is built without floating-point contraction, and no container with unspecified iteration
// order takes part.

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <iostream>

#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;

namespace {

/// Every move kind and every tool, from a fixed seed (SplitMix64 is portable).
unsigned long long jobHash(std::uint64_t seed, bool stepwise, bool timeWise) {
    const Stock stock = stockOf(26, 20, 0.25, 8.0);
    Rng rng(seed);
    const auto tools = allTools();
    Simulator sim(stock);
    std::uint32_t id = 1;
    for (const NamedTool& nt : tools) {
        Toolpath t;
        t.tool.feedRate = 1200.0;
        t.tool.plungeRate = 400.0;
        Vec3 cur{rng.uniform(3, 23), rng.uniform(3, 17), 5};
        t.append(Move::rapid(cur));
        for (int k = 0; k < 30; ++k) {
            const double r = rng.uniform();
            Move m;
            if (r < 0.2) {
                m = Move::plunge({cur.x, cur.y, rng.uniform(-6, -0.2)});
            } else if (r < 0.55) {
                m = Move::feed({rng.uniform(3, 23), rng.uniform(3, 17), rng.uniform(-6, -0.2)});
            } else if (r < 0.9) {
                // Directions from exact literals: no libm call may take part in building the input.
                static const double dirs[8][2] = {{1, 0},   {0.6, 0.8},   {0, 1},  {-0.8, 0.6},
                                                  {-1, 0}, {-0.6, -0.8}, {0, -1}, {0.8, -0.6}};
                const double rho = rng.uniform(0.8, 6);
                const std::size_t i0 = rng.next() % 8;
                const std::size_t i1 = (i0 + 1 + rng.next() % 7) % 8;
                const bool ccw = rng.uniform() < 0.5;
                const Vec2 c{cur.x - rho * dirs[i0][0], cur.y - rho * dirs[i0][1]};
                const Vec3 e{c.x + rho * dirs[i1][0], c.y + rho * dirs[i1][1],
                             rng.uniform() < 0.5 ? cur.z : rng.uniform(-6, -0.2)};
                m = ccw ? Move::arcCCW(e, c) : Move::arcCW(e, c);
            } else {
                m = Move::retract({cur.x, cur.y, 5});
            }
            cur = m.to;
            t.append(m);
        }
        sim.addToolpath(t, nt.shape, id++);
    }
    if (stepwise) {
        for (std::size_t n = 0; n <= sim.moveCount(); n += 7) sim.advanceToMove(n);
    }
    if (timeWise) {
        const double total = sim.totalSeconds();
        for (int k = 1; k <= 40; ++k) sim.advanceToTime(total * k / 40.0 * 0.999);
    }
    sim.advanceAll();
    return hashHeights(sim.heightfield());
}

}  // namespace

TEST_CASE("simulate: the heightfield of a fixed job is byte-identical across runs, compilers and optimisation levels",
          "[simulate][determinism]") {
    // Hashes of the float heights (FNV-1a over the bytes, row-major) of three fixed jobs. They were
    // produced by a Debug (-O0) GCC build and are asserted unchanged in Release (-O2) builds and on
    // MSVC in CI; a change means the cut surface changed.
    constexpr unsigned long long kExpected[] = {17424907614780743214ULL, 15272308804137981140ULL, 2073923921758280580ULL};
    const std::uint64_t seeds[] = {101, 202, 303};
    for (int i = 0; i < 3; ++i) {
        const unsigned long long h = jobHash(seeds[i], false, false);
        std::cout << "[determinism] seed " << seeds[i] << " hash " << h << "ULL\n";
        CHECK(jobHash(seeds[i], false, false) == h);
        CHECK(jobHash(seeds[i], true, false) == h);   // stepped playback
        CHECK(jobHash(seeds[i], true, true) == h);    // time playback, then the rest
        CHECK(h == kExpected[i]);
    }
}
