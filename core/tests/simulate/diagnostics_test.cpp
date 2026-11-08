// T-0222 AC3: diagnostics - rapid in material, below the table, tool deeper than its flutes,
// tool change without retract.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <vector>

#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;
using simulate::DiagnosticKind;

namespace {

std::size_t count(const Simulator& s, DiagnosticKind k) {
    std::size_t n = 0;
    for (const auto& d : s.diagnostics()) n += d.kind == k;
    return n;
}

}  // namespace

TEST_CASE("simulate: rapid in material: only rapids through uncut stock are flagged", "[simulate][diagnostics]") {
    const Stock stock = stockOf(30, 12, 0.25, 8.0);
    const ToolShape flat = *ToolShape::flatEndMill(3.0);
    // A slot 3 deep from x = 5 to 20 at y = 6; rapids then run along it, across it, and above.
    const Simulator along = run(stock, flat, {5, 6, 5},
                                {Move::plunge({5, 6, -3}), Move::feed({20, 6, -3}), Move::rapid({6, 6, -3}),
                                 Move::rapid({19, 6, -2.5})});
    CHECK(count(along, DiagnosticKind::RapidInMaterial) == 0);
    const Simulator safe = run(stock, flat, {5, 6, 5}, {Move::rapid({25, 2, 5}), Move::rapid({25, 2, 0.0})});
    CHECK(safe.diagnostics().empty());  // at or above the top surface is not "in material"
    const Simulator across = run(stock, flat, {5, 6, 5},
                                 {Move::plunge({5, 6, -3}), Move::feed({20, 6, -3}), Move::rapid({12, 10, -3}),
                                  Move::rapid({12, 1, -3})});
    // Move indices: 0 rapid start, 1 plunge, 2 feed, 3 rapid to (12,10,-3): from (20,6,-3) through
    // uncut stock; 4 rapid along x = 12 from y = 10 to 1: crosses the slot but leaves uncut stock on both sides.
    REQUIRE(count(across, DiagnosticKind::RapidInMaterial) == 2);
    CHECK(across.diagnostics()[0].move == 3);
    CHECK(across.diagnostics()[0].toolpathId == 1);
    CHECK(across.diagnostics()[0].position == Vec3{12, 10, -3});
    CHECK(across.diagnostics()[1].move == 4);
    CHECK_FALSE(across.diagnostics()[0].message.empty());
    // Rapids do not cut: the heights are those of the feed alone.
    const Simulator feedOnly = run(stock, flat, {5, 6, 5}, {Move::plunge({5, 6, -3}), Move::feed({20, 6, -3})});
    CHECK(across.heightfield().heights() == feedOnly.heightfield().heights());
    // A rapid diving through the top into uncut stock is flagged, even straight down.
    const Simulator dive = run(stock, flat, {5, 6, 5}, {Move::rapid({5, 6, -1})});
    CHECK(count(dive, DiagnosticKind::RapidInMaterial) == 1);
    // ... but a feed move through material is ordinary cutting.
    CHECK(count(run(stock, flat, {5, 6, 5}, {Move::feed({5, 6, -1}), Move::feed({9, 6, -1})}),
                DiagnosticKind::RapidInMaterial) == 0);
}

TEST_CASE("simulate: below the table", "[simulate][diagnostics]") {
    const ToolShape ball = *ToolShape::ballNose(3.0);
    Stock stock = stockOf(20, 12, 0.5, 4.0);  // table at -4
    Simulator exact = run(stock, ball, {5, 6, 5}, {Move::plunge({5, 6, -4.0})});
    CHECK(count(exact, DiagnosticKind::BelowTable) == 0);  // touching the table is fine
    CHECK(count(run(stock, ball, {5, 6, 5}, {Move::plunge({5, 6, -4.0005})}), DiagnosticKind::BelowTable) == 0);  // within tolerance
    const Simulator below = run(stock, ball, {5, 6, 5}, {Move::plunge({5, 6, -4.2}), Move::feed({9, 6, -4.2})});
    REQUIRE(count(below, DiagnosticKind::BelowTable) == 2);
    CHECK(below.diagnostics()[0].move == 1);
    CHECK(below.diagnostics()[0].position.z == -4.2);
    // the floor of the heightfield is the table
    std::size_t i = 0, j = 0;
    REQUIRE(below.heightfield().cellAt({5.1, 6.1}, i, j));
    CHECK(below.heightfield().height(i, j) == -4.0f);
    // Rapid moves below the table are flagged too; arcs count their end points.
    CHECK(count(run(stock, ball, {5, 6, 5}, {Move::rapid({5, 6, -5})}), DiagnosticKind::BelowTable) == 1);
    CHECK(count(run(stock, ball, {5, 6, 5},
                    {Move::plunge({8, 6, -1}), Move::arcCCW({8, 6, -4.5}, {10, 6})}),
                DiagnosticKind::BelowTable) == 1);
    // Z zero at the machine bed: the table is at Z = 0, the top at the thickness.
    stock.topZ = 4.0;
    CHECK(count(run(stock, ball, {5, 6, 8}, {Move::plunge({5, 6, 0.0})}), DiagnosticKind::BelowTable) == 0);
    CHECK(count(run(stock, ball, {5, 6, 8}, {Move::plunge({5, 6, -0.3})}), DiagnosticKind::BelowTable) == 1);
}

TEST_CASE("simulate: tool deeper than its flute length", "[simulate][diagnostics]") {
    const Stock stock = stockOf(30, 12, 0.25, 30.0);
    const ToolShape flute = ToolShape::flatEndMill(4.0)->withFluteLength(8.0);
    // tip at -8: the shank starts exactly at the surface -> fine; -8.5: shank in material.
    CHECK(count(run(stock, flute, {5, 6, 5}, {Move::plunge({5, 6, -8.0})}), DiagnosticKind::ToolTooDeep) == 0);
    const Simulator deep = run(stock, flute, {5, 6, 5}, {Move::plunge({5, 6, -8.5})});
    REQUIRE(count(deep, DiagnosticKind::ToolTooDeep) == 1);
    CHECK(deep.diagnostics()[0].move == 1);
    // Already-cleared stock does not count: a 6 mm pocket first, then a plunge to 8.5 mm in the pocket
    // (2.5 mm below its floor) leaves nothing above the flute top.
    Toolpath pocket;
    pocket.append(Move::rapid({5, 6, 5}));
    pocket.append(Move::plunge({5, 6, -6}));
    pocket.append(Move::feed({25, 6, -6}));
    pocket.append(Move::feed({25, 6, -8.5}));
    Simulator sim(stock);
    REQUIRE(sim.addToolpath(pocket, flute, 1));
    sim.advanceAll();
    CHECK(count(sim, DiagnosticKind::ToolTooDeep) == 0);  // the plunge to -6: tip 6 deep < 8; later one is in the pocket
    // An unlimited flute length never reports.
    CHECK(count(run(stock, *ToolShape::flatEndMill(4.0), {5, 6, 5}, {Move::plunge({5, 6, -25.0})}),
                DiagnosticKind::ToolTooDeep) == 0);
    // A sloped move whose end is deep: flagged at that move only.
    const Simulator ramp = run(stock, flute, {5, 6, 5}, {Move::feed({15, 6, -3}), Move::feed({20, 6, -12})});
    REQUIRE(count(ramp, DiagnosticKind::ToolTooDeep) == 1);
    CHECK(ramp.diagnostics()[0].move == 2);
    // An arc/helix too.
    const Simulator helix = run(stock, flute, {10, 6, 5},
                                {Move::arcCCW({10, 6, -10}, {12, 6})});
    CHECK(count(helix, DiagnosticKind::ToolTooDeep) == 1);
}

TEST_CASE("simulate: tool change without retract", "[simulate][diagnostics]") {
    const Stock stock = stockOf(30, 12, 0.25, 12.0);
    const ToolShape flat = *ToolShape::flatEndMill(3.0);
    const ToolShape ball = *ToolShape::ballNose(3.0);
    auto path = [](int toolNumber, double endZ) {
        Toolpath t;
        t.tool.toolNumber = toolNumber;
        t.append(Move::rapid({5, 6, 5}));
        t.append(Move::plunge({5, 6, -2}));
        t.append(Move::feed({9, 6, -2}));
        if (endZ > 0) t.append(Move::retract({9, 6, endZ}));
        return t;
    };
    {  // different tool, previous one left in the cut
        Simulator sim(stock);
        sim.addToolpath(path(1, -1), flat, 1);   // no retract: ends at (9,6,-2)
        sim.addToolpath(path(2, 5), ball, 2);
        sim.advanceAll();
        REQUIRE(count(sim, DiagnosticKind::ToolChangeWithoutRetract) == 1);
        const auto& d = sim.diagnostics()[0];
        CHECK(d.move == 3);
        CHECK(d.toolpathId == 2);
        CHECK(d.position == Vec3{9, 6, -2});
    }
    {  // retracted before the change: fine
        Simulator sim(stock);
        sim.addToolpath(path(1, 5), flat, 1);
        sim.addToolpath(path(2, 5), ball, 2);
        sim.advanceAll();
        CHECK(count(sim, DiagnosticKind::ToolChangeWithoutRetract) == 0);
    }
    {  // same tool, not retracted: not a tool change
        Simulator sim(stock);
        sim.addToolpath(path(1, -1), flat, 1);
        sim.addToolpath(path(1, 5), flat, 2);
        sim.advanceAll();
        CHECK(count(sim, DiagnosticKind::ToolChangeWithoutRetract) == 0);
    }
    {  // same shape but another tool number is a change
        Simulator sim(stock);
        sim.addToolpath(path(1, -1), flat, 1);
        sim.addToolpath(path(2, 5), flat, 2);
        sim.advanceAll();
        CHECK(count(sim, DiagnosticKind::ToolChangeWithoutRetract) == 1);
    }
}

TEST_CASE("simulate: diagnostics are capped and counted", "[simulate][diagnostics]") {
    const Stock stock = stockOf(30, 12, 0.5, 8.0);
    simulate::SimOptions opt;
    opt.maxDiagnostics = 5;
    Simulator sim(stock, opt);
    Toolpath t;
    t.append(Move::rapid({5, 6, 5}));
    for (int k = 0; k < 20; ++k) {
        t.append(Move::rapid({5, 6, -1}));
        t.append(Move::rapid({5, 6, 5}));  // leaving upward through uncut stock is a rapid in material too
    }
    sim.addToolpath(t, *ToolShape::flatEndMill(2.0), 1);
    sim.advanceAll();
    CHECK(sim.diagnostics().size() == 5);
    CHECK(sim.droppedDiagnostics() == 35);
}
