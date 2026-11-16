// T-0325 (ADR-0107): laser burn-rate simulation. Counter-based and exact where the model is exact: the beam is a
// disc, the dose at a cell is power/100 * burnRate / speed * (path length inside the disc around the cell centre) /
// diameter, summed over moves; darkness is min(1, dose).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <vector>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/toolpath/toolpath.hpp"

using namespace carvetto;
using geometry::Vec3;
using simulate::SimStatus;
using simulate::Simulator;
using simulate::Stock;
using toolpath::Move;
using toolpath::Toolpath;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kBeam = 1.0;  // mm
constexpr double kRate = 1000.0;
constexpr double kFeed = 1000.0;

Stock stock() {
    Stock s;
    s.width = 20.0;
    s.height = 10.0;
    s.thickness = 5.0;
    s.cellSize = 0.25;
    return s;
}

Toolpath laserPath(std::vector<Move> moves, double feed = kFeed) {
    Toolpath tp;
    tp.name = "Laser";
    tp.tool.name = "Laser";
    tp.tool.toolNumber = 1;
    tp.tool.diameter = kBeam;
    tp.tool.feedRate = feed;
    tp.tool.plungeRate = 200.0;
    tp.tool.laser = true;
    tp.moves = std::move(moves);
    return tp;
}

/// Chord factor of a cell whose centre is `d` from the centre line of an infinitely long path.
double chord(double d) {
    const double r = kBeam / 2.0;
    return 2.0 * std::sqrt(r * r - d * d) / kBeam;
}

// Cell (20, 19) has its centre at (5.125, 4.875): 0.125 below the line y = 5.
constexpr std::size_t kI = 20;
constexpr std::size_t kJ = 19;

std::vector<Move> line(double power, double x0 = 2.0, double x1 = 12.0, double y = 5.0) {
    return {Move::rapid({x0, y, 1.0}), Move::feed({x0, y, 0.0}), Move::feed({x1, y, 0.0}).withJet(power),
            Move::retract({x1, y, 1.0})};
}

Simulator runLine(double power, double feed = kFeed) {
    Simulator sim(stock());
    REQUIRE(sim.addLaserToolpath(laserPath(line(power), feed), kRate, 1));
    REQUIRE(sim.advanceAll());
    return sim;
}

}  // namespace

TEST_CASE("T-0325: a pass at full power and the burn rate gives dose 1 on the centre line (chord factor off it)",
          "[simulate][laser]") {
    Simulator sim = runLine(100.0);
    CHECK(sim.hasBurn());
    CHECK_THAT(static_cast<double>(sim.burnDoses()[sim.heightfield().indexOf(kI, kJ)]), WithinAbs(chord(0.125), 1e-5));
    CHECK_THAT(static_cast<double>(sim.darkness(kI, kJ)), WithinAbs(chord(0.125), 1e-5));
    // The row above the line is symmetric.
    CHECK_THAT(static_cast<double>(sim.darkness(kI, kJ + 1)), WithinAbs(chord(0.125), 1e-5));
    // The next rows are 0.375 away: still inside the 0.5 mm radius.
    CHECK_THAT(static_cast<double>(sim.darkness(kI, kJ - 1)), WithinAbs(chord(0.375), 1e-5));
    // 0.625 away: outside the beam, unmarked.
    CHECK(sim.darkness(kI, kJ - 2) == 0.0f);
    CHECK(sim.darkness(kI, kJ + 3) == 0.0f);
    // Before and past the ends of the line: a cell 1 mm before the start is unmarked.
    CHECK(sim.darkness(4, kJ) == 0.0f);
    CHECK(sim.darkness(52, kJ) == 0.0f);
    // Marked band: rows j = 18..21 (four rows within 0.5 mm), columns about 10 mm / 0.25 mm wide plus the end caps.
    CHECK(sim.markedCells() > 150);
    CHECK(sim.markedCells() < 200);
}

TEST_CASE("T-0325: dose scales with power and inversely with speed", "[simulate][laser]") {
    const double full = chord(0.125);
    CHECK_THAT(static_cast<double>(runLine(50.0).darkness(kI, kJ)), WithinAbs(0.5 * full, 1e-5));
    CHECK_THAT(static_cast<double>(runLine(100.0, 2000.0).darkness(kI, kJ)), WithinAbs(0.5 * full, 1e-5));
    CHECK_THAT(static_cast<double>(runLine(100.0, 4000.0).darkness(kI, kJ)), WithinAbs(0.25 * full, 1e-5));
    // Slower than the burn rate darkens beyond 1: the raw dose keeps the excess, the darkness saturates.
    Simulator slow = runLine(100.0, 250.0);
    CHECK_THAT(static_cast<double>(slow.burnDoses()[slow.heightfield().indexOf(kI, kJ)]), WithinAbs(4.0 * full, 1e-4));
    CHECK(slow.darkness(kI, kJ) == 1.0f);
}

TEST_CASE("T-0325: a feed override and the plunge rate set the speed of their move", "[simulate][laser]") {
    Simulator sim(stock());
    Toolpath tp = laserPath({Move::rapid({2, 5, 1}), Move::feed({2, 5, 0}), Move::feed({12, 5, 0}, 2000.0).withJet(100.0)});
    REQUIRE(sim.addLaserToolpath(tp, kRate, 1));
    REQUIRE(sim.advanceAll());
    CHECK_THAT(static_cast<double>(sim.darkness(kI, kJ)), WithinAbs(0.5 * chord(0.125), 1e-5));
    // A Plunge move uses the plunge rate (200 mm/min): five times the dose of a feed at 1000.
    Simulator sim2(stock());
    Toolpath tp2 = laserPath({Move::rapid({2, 5, 1}), Move::feed({2, 5, 0}),
                              Move::plunge({12, 5, 0}).withJet(10.0)});
    REQUIRE(sim2.addLaserToolpath(tp2, kRate, 1));
    REQUIRE(sim2.advanceAll());
    CHECK_THAT(static_cast<double>(sim2.darkness(kI, kJ)), WithinAbs(0.1 * 5.0 * chord(0.125), 1e-5));
}

TEST_CASE("T-0325: passes add up, a polyline equals the same path in one move, rapids and beam-off moves mark nothing",
          "[simulate][laser]") {
    // Two passes on the same line.
    std::vector<Move> twice = line(40.0);
    const std::vector<Move> again = line(40.0);
    twice.insert(twice.end(), again.begin(), again.end());
    Simulator two(stock());
    REQUIRE(two.addLaserToolpath(laserPath(twice), kRate, 1));
    REQUIRE(two.advanceAll());
    CHECK_THAT(static_cast<double>(two.darkness(kI, kJ)), WithinAbs(0.8 * chord(0.125), 1e-5));

    // The line split in two at a joint gives the same doses everywhere (the doses of the halves add exactly).
    Simulator whole = runLine(100.0);
    Simulator split(stock());
    REQUIRE(split.addLaserToolpath(laserPath({Move::rapid({2, 5, 1}), Move::feed({2, 5, 0}),
                                              Move::feed({7.1, 5, 0}).withJet(100.0),
                                              Move::feed({12, 5, 0}).withJet(100.0), Move::retract({12, 5, 1})}),
                                   kRate, 1));
    REQUIRE(split.advanceAll());
    REQUIRE(whole.burnDoses().size() == split.burnDoses().size());
    for (std::size_t k = 0; k < whole.burnDoses().size(); ++k) {
        REQUIRE_THAT(static_cast<double>(split.burnDoses()[k]), WithinAbs(static_cast<double>(whole.burnDoses()[k]), 1e-5));
    }

    // Beam-off feeds, rapids and dwells mark nothing.
    Simulator off(stock());
    REQUIRE(off.addLaserToolpath(laserPath({Move::rapid({2, 5, 1}), Move::feed({2, 5, 0}), Move::feed({12, 5, 0}),
                                            Move::dwell({12, 5, 0}, 1.0), Move::rapid({2, 6, 1})}),
                                 kRate, 1));
    REQUIRE(off.advanceAll());
    CHECK(off.markedCells() == 0);
    CHECK(off.hasBurn());
}

TEST_CASE("T-0325: the laser removes no material and leaves the heightfield and diagnostics alone", "[simulate][laser]") {
    Simulator sim = runLine(100.0);
    CHECK(sim.heightfield().cutCells() == 0);
    CHECK(sim.diagnostics().empty());
    for (const float z : sim.heightfield().heights()) {
        REQUIRE(z == 0.0f);
    }
    // A beam-on move below the table does not raise a collision diagnostic either (no tool body).
    Simulator deep(stock());
    REQUIRE(deep.addLaserToolpath(laserPath({Move::rapid({2, 5, -30}), Move::feed({12, 5, -30}).withJet(50.0)}), kRate, 1));
    REQUIRE(deep.advanceAll());
    CHECK(deep.diagnostics().empty());
    CHECK(deep.markedCells() > 0);
}

TEST_CASE("T-0325: an arc is marked along its circle", "[simulate][laser]") {
    Simulator sim(stock());
    // Full circle of radius 4 around (10, 5), counter-clockwise, from (14, 5).
    Toolpath tp = laserPath({Move::rapid({14, 5, 1}), Move::feed({14, 5, 0}),
                             Move::arcCCW({14, 5, 0}, {10, 5}).withJet(100.0)});
    REQUIRE(sim.addLaserToolpath(tp, kRate, 1));
    REQUIRE(sim.advanceAll());
    // Independent oracle (the chord approximation, sagitta cell/64, moves the ring by up to 0.004 mm: tolerance 0.01): the circle sampled finely, its length inside the beam disc around each cell centre.
    const auto& hf = sim.heightfield();
    const auto oracle = [&](std::size_t i, std::size_t j) {
        const geometry::Vec2 c = hf.centreOf(i, j);
        constexpr int kSteps = 200000;
        const double step = 2.0 * 3.14159265358979323846 * 4.0 / kSteps;
        double length = 0.0;
        for (int k = 0; k < kSteps; ++k) {
            const double a = 2.0 * 3.14159265358979323846 * (k + 0.5) / kSteps;
            const double x = 10.0 + 4.0 * std::cos(a) - c.x;
            const double y = 5.0 + 4.0 * std::sin(a) - c.y;
            if (x * x + y * y <= kBeam * kBeam / 4.0) {
                length += step;
            }
        }
        return length / kBeam;
    };
    std::size_t checked = 0;
    for (const geometry::Vec2 p : {geometry::Vec2{13.875, 5.125}, geometry::Vec2{6.125, 4.875}, geometry::Vec2{10.125, 8.875},
                                   geometry::Vec2{10.125, 0.875}, geometry::Vec2{12.875, 7.875}, geometry::Vec2{10.125, 8.4},
                                   geometry::Vec2{10.125, 9.4}, geometry::Vec2{10.125, 5.125}}) {
        std::size_t i = 0;
        std::size_t j = 0;
        REQUIRE(hf.cellAt(p, i, j));
        CHECK_THAT(static_cast<double>(sim.burnDoses()[hf.indexOf(i, j)]), WithinAbs(oracle(i, j), 0.01));
        ++checked;
    }
    CHECK(checked == 8);
    CHECK(sim.darkness(40, 20) == 0.0f);  // the centre of the circle
    CHECK(sim.darkness(2, 2) == 0.0f);
    // A clockwise quarter arc marks only its quarter.
    Simulator q(stock());
    Toolpath tq = laserPath({Move::rapid({14, 5, 1}), Move::feed({14, 5, 0}),
                             Move::arcCW({10, 1, 0}, {10, 5}).withJet(100.0)});
    REQUIRE(q.addLaserToolpath(tq, kRate, 1));
    REQUIRE(q.advanceAll());
    std::size_t iq = 0;
    std::size_t jq = 0;
    REQUIRE(hf.cellAt({12.85, 2.15}, iq, jq));  // on the quarter (south-east)
    CHECK(q.darkness(iq, jq) > 0.5f);
    REQUIRE(hf.cellAt({7.15, 7.85}, iq, jq));  // north-west, not on the quarter
    CHECK(q.darkness(iq, jq) == 0.0f);
}

TEST_CASE("T-0325: playback marks whole moves; seeking back clears; time playback has no partial dose", "[simulate][laser]") {
    Simulator sim(stock());
    REQUIRE(sim.addLaserToolpath(laserPath(line(100.0)), kRate, 1));  // moves: rapid, feed, burn, retract
    REQUIRE(sim.moveCount() == 4);
    REQUIRE(sim.advanceToMove(2));
    CHECK(sim.markedCells() == 0);
    REQUIRE(sim.advanceToMove(3));
    const std::size_t marked = sim.markedCells();
    CHECK(marked > 0);
    // Mid-move time playback: nothing of the burn move yet, same as before it started.
    Simulator t(stock());
    REQUIRE(t.addLaserToolpath(laserPath(line(100.0)), kRate, 1));
    const double t2 = t.moveEndTimes()[1];
    const double t3 = t.moveEndTimes()[2];
    REQUIRE(t.advanceToTime(0.5 * (t2 + t3)));
    CHECK(t.movesDone() == 2);
    CHECK(t.markedCells() == 0);
    REQUIRE(t.advanceToTime(t3 + 1e-9));
    CHECK(t.markedCells() == marked);
    // Seek back resets the layer; seeking forward again reproduces it exactly.
    REQUIRE(t.seekMove(1));
    CHECK(t.markedCells() == 0);
    REQUIRE(t.seekMove(3));
    CHECK(t.burnDoses() == sim.burnDoses());
    sim.reset();
    CHECK(sim.markedCells() == 0);
    CHECK(sim.hasBurn());
}

TEST_CASE("T-0325: a laser toolpath mixes with a cutting one; each acts on its own layer", "[simulate][laser]") {
    Simulator sim(stock());
    Toolpath mill;
    mill.name = "Mill";
    mill.tool.toolNumber = 2;
    mill.tool.diameter = 2.0;
    mill.tool.feedRate = 1000.0;
    mill.tool.plungeRate = 300.0;
    mill.tool.spindleSpeed = 10000.0;
    mill.moves = {Move::rapid({3, 8, 1}), Move::plunge({3, 8, -1}), Move::feed({8, 8, -1}), Move::retract({8, 8, 1})};
    REQUIRE(sim.addToolpath(mill, *simulate::ToolShape::flatEndMill(2.0), 1));
    REQUIRE(sim.addLaserToolpath(laserPath(line(100.0)), kRate, 2));
    REQUIRE(sim.advanceAll());
    CHECK(sim.heightfield().cutCells() > 0);
    CHECK(sim.markedCells() > 0);
    // Cells of the milled slot are not burnt and the burnt cells are not cut.
    std::size_t i = 0;
    std::size_t j = 0;
    REQUIRE(sim.heightfield().cellAt({5.0, 8.0}, i, j));
    CHECK(sim.darkness(i, j) == 0.0f);
    CHECK(sim.heightfield().isCut(i, j));
    CHECK_FALSE(sim.heightfield().isCut(kI, kJ));
}

TEST_CASE("T-0325: refusals, budgets and allocation failure", "[simulate][laser]") {
    using Catch::Matchers::ContainsSubstring;
    Simulator sim(stock());
    CHECK_FALSE(sim.addLaserToolpath(laserPath(line(100.0)), 0.0, 1));
    CHECK(sim.error().status == SimStatus::InvalidTool);
    CHECK_THAT(sim.error().message, ContainsSubstring("burn rate"));
    CHECK_FALSE(sim.addLaserToolpath(laserPath(line(100.0)), std::nan(""), 1));
    CHECK_FALSE(sim.addLaserToolpath(laserPath(line(100.0)), -5.0, 1));
    CHECK_FALSE(sim.hasBurn());
    CHECK(sim.moveCount() == 0);

    Toolpath notLaser = laserPath(line(100.0));
    notLaser.tool.laser = false;
    CHECK_FALSE(sim.addLaserToolpath(notLaser, kRate, 1));
    CHECK_THAT(sim.error().message, ContainsSubstring("not a laser"));

    Toolpath noBeam = laserPath(line(100.0));
    noBeam.tool.diameter = 0.0;
    CHECK_FALSE(sim.addLaserToolpath(noBeam, kRate, 1));
    CHECK(sim.error().status == SimStatus::InvalidTool);

    // Invalid move coordinates are refused like any toolpath's.
    Toolpath bad = laserPath({Move::rapid({std::nan(""), 0, 0})});
    CHECK_FALSE(sim.addLaserToolpath(bad, kRate, 1));
    CHECK(sim.error().status == SimStatus::InvalidMove);

    // The work budget counts burn cells.
    simulate::SimOptions tiny;
    tiny.maxCellWork = 10.0;
    Simulator limited(stock(), tiny);
    REQUIRE(limited.addLaserToolpath(laserPath(line(100.0)), kRate, 1));
    CHECK_FALSE(limited.advanceAll());
    CHECK(limited.error().status == SimStatus::WorkBudgetExceeded);
    CHECK(limited.cellWork() > 10.0);

    // A failed burn layer allocation is an OutOfMemory error, nothing queued.
    Simulator oom(stock());
    simulate::detail::failNextAllocation(4);
    CHECK_FALSE(oom.addLaserToolpath(laserPath(line(100.0)), kRate, 1));
    CHECK(oom.error().status == SimStatus::OutOfMemory);
    CHECK(oom.moveCount() == 0);
    CHECK(oom.addLaserToolpath(laserPath(line(100.0)), kRate, 1));  // the failure was consumed
}

TEST_CASE("T-0325: darkness outside the stock and without a layer is zero", "[simulate][laser]") {
    Simulator sim(stock());
    CHECK_FALSE(sim.hasBurn());
    CHECK(sim.darkness(0, 0) == 0.0f);
    REQUIRE(sim.addLaserToolpath(laserPath(line(100.0)), kRate, 1));
    CHECK(sim.darkness(100000, 0) == 0.0f);
    CHECK(sim.darkness(0, 100000) == 0.0f);
    // A beam partly outside the stock marks the cells inside only (no crash, no wrap).
    Simulator edge(stock());
    REQUIRE(edge.addLaserToolpath(laserPath({Move::rapid({-5, 0.1, 1}), Move::feed({-5, 0.1, 0}),
                                             Move::feed({25, 0.1, 0}).withJet(100.0)}),
                                  kRate, 1));
    REQUIRE(edge.advanceAll());
    CHECK(edge.darkness(0, 0) > 0.0f);
    CHECK(edge.darkness(79, 0) > 0.0f);
    CHECK(edge.darkness(0, 5) == 0.0f);
}
