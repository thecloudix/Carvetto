// T-0222 AC2: performance. Cost model (ADR-0032 §8): per move O(cells in the clipped swept
// footprint), so the total is near-linear in moves x footprint cells; doubling the number of moves
// costs at most 2.5x (interleaved CPU-time runs of cpu_scaling.hpp). Absolute Release bounds are
// asserted only in optimised builds.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "../geometry/cpu_scaling.hpp"
#include "../geometry/medial_axis_glyphs.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

namespace {

/// `count` short cutting moves (1-3 mm, random directions and depths, with arcs every 8th move) on
/// a square of `side` mm; every move starts where the previous one ended.
Toolpath scatterJob(std::size_t count, double side, std::uint64_t seed) {
    Rng rng(seed);
    Toolpath t;
    t.tool.feedRate = 1500.0;
    t.tool.plungeRate = 500.0;
    Vec3 cur{side / 2, side / 2, 5.0};
    t.append(Move::rapid(cur));
    for (std::size_t k = 0; k < count; ++k) {
        const double ax = rng.uniform(-1.0, 1.0);
        const double ay = rng.uniform(-1.0, 1.0);
        const double len = rng.uniform(1.0, 3.0) / std::max(0.2, std::hypot(ax, ay));
        Vec3 to{cur.x + ax * len, cur.y + ay * len, rng.uniform(-4.0, -0.5)};
        to.x = std::min(side - 2, std::max(2.0, to.x));
        to.y = std::min(side - 2, std::max(2.0, to.y));
        if (k % 8 == 7 && cur.x + 1.0 <= side - 2 && cur.y + 1.0 <= side - 2) {
            const double h = 1.0;  // quarter-turn arc to a point at distance h in x and y
            to = {cur.x + h, cur.y + h, cur.z};
            t.append(Move::arcCCW(to, {cur.x + h, cur.y}));
        } else {
            t.append(Move::feed(to));
        }
        cur = to;
    }
    return t;
}

double cpuOf(const Stock& stock, const Toolpath& job, const ToolShape& tool) {
    const double t0 = test::processCpuSeconds();
    Simulator sim(stock);
    REQUIRE(sim.addToolpath(job, tool, 1));
    REQUIRE(sim.advanceAll());
    return test::processCpuSeconds() - t0;
}

}  // namespace

TEST_CASE("simulate: simulation cost grows linearly with the number of moves", "[simulate][perf][scaling]") {
    const Stock stock = stockOf(200, 200, 0.5, 10.0);  // 400 x 400 cells
    const std::size_t n = kRelease ? 20000 : 3000;
    const Toolpath smallJob = scatterJob(n, 200, 5);
    const Toolpath largeJob = scatterJob(2 * n, 200, 5);
    const ToolShape tool = *ToolShape::ballNose(3.0);
    std::size_t smallCut = 0, largeCut = 0;
    const test::ScalingTimes times = test::interleavedCpuSeconds(
        3,
        [&] {
            Simulator sim(stock);
            sim.addToolpath(smallJob, tool, 1);
            sim.advanceAll();
            smallCut = sim.heightfield().cutCells();
        },
        [&] {
            Simulator sim(stock);
            sim.addToolpath(largeJob, tool, 1);
            sim.advanceAll();
            largeCut = sim.heightfield().cutCells();
        });
    INFO(n << " moves: " << times.smallSeconds << " s CPU, " << 2 * n << " moves: " << times.largeSeconds
         << " s CPU, ratio " << times.ratio());
    std::cout << "[perf] " << n << " -> " << 2 * n << " moves: " << times.smallSeconds << " s -> " << times.largeSeconds
              << " s, ratio " << times.ratio() << "\n";
    CHECK(smallCut > 1000);
    CHECK(largeCut >= smallCut);
    CHECK(times.ratio() <= 2.5);
}

TEST_CASE("simulate: one million moves on a 2k x 2k stock within the documented bound", "[simulate][perf][million]") {
    // ADR-0032 §8: Release, 1,000,000 short moves (ball nose 3 mm) on 1000 x 1000 mm at 0.5 mm
    // (2000 x 2000 cells) in at most 10 s of CPU time. Debug builds run 20,000 moves on the same stock.
    const std::size_t count = kRelease ? 1'000'000 : 20'000;
    const Stock stock = stockOf(1000, 1000, 0.5, 10.0);
    const Toolpath job = scatterJob(count, 1000, 11);
    const ToolShape tool = *ToolShape::ballNose(3.0);
    const double t = cpuOf(stock, job, tool);
    INFO(count << " moves: " << t << " s CPU");
    std::cout << "[perf] " << count << " moves on 2000 x 2000 cells: " << t << " s CPU\n";
    if (kRelease) {
        CHECK(t <= 10.0);
    }
}

TEST_CASE("simulate: 5000 V-carved glyph regions at medium quality within the documented bound", "[simulate][perf][glyphs]") {
    // ADR-0032 §8: the V-carve record of 5,000 glyph-like regions (T-0153's page) simulates on a
    // 700 x 450 mm stock at medium quality (0.25 mm, ~5 M cells) in at most 10 s of CPU time in
    // Release. Debug builds simulate 150 regions.
    const std::size_t regions = kRelease ? 5000 : 150;
    std::vector<geometry::Contour> page;
    for (std::size_t k = 0; k < regions; ++k) {
        const Vec2 o{static_cast<double>(k % 100) * 7.0, static_cast<double>(k / 100) * 9.0};
        test::medial::appendGlyph(page, static_cast<int>(k % 5), o);
    }
    tooling::Tool tool;
    tool.id = "v60";
    tool.name = "v60";
    tool.type = tooling::ToolType::VBit;
    tool.diameter = 12.7;
    tool.includedAngle = 60 * kDeg;
    tool.defaultCuttingData.toolNumber = 1;
    tool.defaultCuttingData.feedRate = 1500;
    tool.defaultCuttingData.plungeRate = 500;
    tool.defaultCuttingData.spindleSpeed = 18000;
    const toolpath::VCarvingResult r = toolpath::vCarvingToolpath(page, tool, std::nullopt, {}, toolpath::VCarvingParams{});
    REQUIRE(r.ok());
    geometry::Box2 area;
    for (const auto& c : page) {
        area.expand(c.bounds().min);
        area.expand(c.bounds().max);
    }
    Stock stock = Stock::withQuality(area.max.x + 10.0, area.max.y + 10.0, 12.0, simulate::Quality::Medium);
    stock.origin = {-5.0, -5.0};
    const double t0 = test::processCpuSeconds();
    Simulator sim(stock);
    REQUIRE(sim.ok());
    REQUIRE(sim.addToolpath(r.toolpath, *ToolShape::fromTool(tool), 1));
    REQUIRE(sim.advanceAll());
    const double t = test::processCpuSeconds() - t0;
    INFO(regions << " regions, " << r.toolpath.moveCount() << " moves, " << sim.heightfield().cellCount() << " cells: "
         << t << " s CPU");
    std::cout << "[perf] V-carve " << regions << " glyph regions: " << r.toolpath.moveCount() << " moves, "
              << sim.heightfield().cellCount() << " cells (medium), " << t << " s CPU\n";
    CHECK(sim.diagnostics().empty());
    CHECK(sim.heightfield().cutCells() > regions * 10);
    if (kRelease) {
        CHECK(t <= 10.0);
    }
}
