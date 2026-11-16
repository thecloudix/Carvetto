// T-0335: the simulator drills one hole per spindle of a drill bank stroke (ADR-0111): positions, per-spindle depth
// (a shallower spindle is stopped by the head's stroke limit), per-spindle drill diameter and shape, a switched-off
// reference spindle, partial playback, and the refusals.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <vector>

#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;
using Catch::Approx;
using toolpath::BankSpindleSpec;

namespace {

/// Height of the cut stock at (x, y).
double heightAt(const Simulator& sim, double x, double y) {
    std::size_t i = 0;
    std::size_t j = 0;
    REQUIRE(sim.heightfield().cellAt({x, y}, i, j));
    return static_cast<double>(sim.heightfield().height(i, j));
}

Move banked(Move m, std::uint64_t set) {
    m.bank = set;
    return m;
}

/// Three spindles 20 mm apart (reference = the middle one), depths 4, 6 and 3, 3 mm drills.
std::vector<BankSpindleSpec> head() {
    return {{{-20.0, 0.0}, 3.0, 4.0}, {{0.0, 0.0}, 3.0, 6.0}, {{20.0, 0.0}, 3.0, 3.0}};
}

Toolpath stroke(std::uint64_t set, double bottom = -6.0, double x = 50.0) {
    Toolpath tp;
    tp.tool.feedRate = 1000.0;
    tp.tool.plungeRate = 300.0;
    tp.bankHead = head();
    tp.moves = {banked(Move::rapid({x, 20, 5}), set), banked(Move::plunge({x, 20, bottom}), set),
                banked(Move::retract({x, 20, 5}), set)};
    return tp;
}

Simulator cutStroke(const Toolpath& tp, const ToolShape& tool = *ToolShape::flatEndMill(3.0)) {
    Simulator sim(stockOf(100, 40, 0.25, 10.0));
    REQUIRE(sim.addToolpath(tp, tool, 1));
    sim.advanceAll();
    return sim;
}

}  // namespace

TEST_CASE("simulate: a drill bank stroke drills one hole per spindle, each to its own depth",
          "[simulate][drill_bank][spindles]") {
    const Simulator sim = cutStroke(stroke(0b111));
    // Spindles at x = 30, 50, 70; the deepest (spindle 2) sets the stroke to -6, the others stop at their depths.
    CHECK(heightAt(sim, 30, 20) == Approx(-4.0).margin(1e-6));
    CHECK(heightAt(sim, 50, 20) == Approx(-6.0).margin(1e-6));
    CHECK(heightAt(sim, 70, 20) == Approx(-3.0).margin(1e-6));
    // Nothing between the holes, nothing at other places.
    CHECK(heightAt(sim, 40, 20) == Approx(0.0).margin(1e-6));
    CHECK(heightAt(sim, 60, 20) == Approx(0.0).margin(1e-6));
    CHECK(heightAt(sim, 30, 30) == Approx(0.0).margin(1e-6));
    // Three holes of the 3 mm drill: the cut area is three discs (about 7.07 mm2 each).
    const double cell = 0.25;
    const double area = static_cast<double>(sim.heightfield().cutCells()) * cell * cell;
    CHECK(area == Approx(3 * 3.14159265358979 * 1.5 * 1.5).epsilon(0.05));
    CHECK(sim.diagnostics().empty());
}

TEST_CASE("simulate: a switched-off or unused spindle drills nothing, the reference position is not a hole",
          "[simulate][drill_bank][spindles]") {
    // Spindles 1 and 3 only: no hole at the reference spindle's place.
    const Simulator sim = cutStroke(stroke(0b101, -4.0));
    CHECK(heightAt(sim, 30, 20) == Approx(-4.0).margin(1e-6));
    CHECK(heightAt(sim, 70, 20) == Approx(-3.0).margin(1e-6));
    CHECK(heightAt(sim, 50, 20) == Approx(0.0).margin(1e-6));
    // Spindle 2 alone: only the middle hole.
    const Simulator mid = cutStroke(stroke(0b010));
    CHECK(heightAt(mid, 50, 20) == Approx(-6.0).margin(1e-6));
    CHECK(heightAt(mid, 30, 20) == Approx(0.0).margin(1e-6));
    CHECK(mid.heightfield().cutCells() < sim.heightfield().cutCells());
}

TEST_CASE("simulate: a stroke's depth limit comes from its own moves", "[simulate][drill_bank][spindles]") {
    // Stroke A drills to -6 with all spindles; stroke B (spindles 1 and 3, deepest 4) only to -4: spindle 3 stops at -3.
    Toolpath tp = stroke(0b111);
    const Toolpath b = stroke(0b101, -4.0, 50.0);
    tp.moves.insert(tp.moves.end(), b.moves.begin(), b.moves.end());
    // Move stroke B sideways (a second row) by giving it another Y.
    for (std::size_t i = 3; i < tp.moves.size(); ++i) {
        tp.moves[i].to.y = 30.0;
    }
    const Simulator sim = cutStroke(tp);
    CHECK(heightAt(sim, 30, 30) == Approx(-4.0).margin(1e-6));
    CHECK(heightAt(sim, 70, 30) == Approx(-3.0).margin(1e-6));  // stopped at its own depth, not at the stroke bottom
    CHECK(heightAt(sim, 50, 30) == Approx(0.0).margin(1e-6));
}

TEST_CASE("simulate: each spindle's drill has its own diameter; the toolpath's tool is used when it fits",
          "[simulate][drill_bank][spindles]") {
    Toolpath tp = stroke(0b111);
    tp.bankHead[0].diameter = 7.0;  // a wider drill on spindle 1
    const Simulator sim = cutStroke(tp);
    CHECK(heightAt(sim, 30 + 3.2, 20) == Approx(-4.0).margin(1e-6));  // inside the 7 mm hole (radius 3.5)
    CHECK(heightAt(sim, 30 + 3.8, 20) == Approx(0.0).margin(1e-6));
    CHECK(heightAt(sim, 50 + 1.8, 20) == Approx(0.0).margin(1e-6));  // spindle 2 is still 3 mm
    // A spindle whose diameter is the tool's uses the tool's shape (a V-bit: a cone), another diameter a flat drill.
    const ToolShape vbit = *ToolShape::vBit(6.0, 90 * kDeg);
    Toolpath cone = stroke(0b011);
    cone.bankHead[0].diameter = 6.0;  // the tool's own
    cone.bankHead[1].diameter = 7.0;  // flat
    cone.bankHead[0].depth = 6.0;
    const Simulator cs = cutStroke(cone, vbit);
    CHECK(heightAt(cs, 30, 20) == Approx(-6.0).margin(0.25));  // the cone's tip (cell centre within 0.18 mm)
    CHECK(heightAt(cs, 30 + 2.5, 20) > -6.0 + 2.0);  // the cone rises with the radius
    CHECK(heightAt(cs, 50 + 2.5, 20) == Approx(-6.0).margin(1e-6));  // a flat bottom out to the radius
}

TEST_CASE("simulate: partial playback drills every spindle's hole in step", "[simulate][drill_bank][spindles]") {
    Simulator sim(stockOf(100, 40, 0.25, 10.0));
    REQUIRE(sim.addToolpath(stroke(0b111), *ToolShape::flatEndMill(3.0), 1));
    // The plunge from z 5 to -6 takes 11 / 300 * 60 = 2.2 s; at 1.1 s the tools are at z -0.5.
    REQUIRE(sim.advanceToTime(1.1));
    CHECK(heightAt(sim, 30, 20) == Approx(-0.5).margin(0.02));
    CHECK(heightAt(sim, 50, 20) == Approx(-0.5).margin(0.02));
    CHECK(heightAt(sim, 70, 20) == Approx(-0.5).margin(0.02));
    REQUIRE(sim.advanceToTime(2.0));
    CHECK(heightAt(sim, 30, 20) == Approx(-4.0).margin(1e-6));  // already at its limit
    CHECK(heightAt(sim, 50, 20) < -4.0);
    CHECK(heightAt(sim, 70, 20) == Approx(-3.0).margin(1e-6));
    sim.advanceAll();
    CHECK(heightAt(sim, 50, 20) == Approx(-6.0).margin(1e-6));
    // Replaying from the start gives the same stock.
    const std::vector<float> done = sim.heightfield().heights();
    REQUIRE(sim.seekMove(0));
    sim.advanceAll();
    CHECK(sim.heightfield().heights() == done);
}

TEST_CASE("simulate: drill bank toolpaths are refused when the head cannot be used", "[simulate][drill_bank][spindles]") {
    const ToolShape flat = *ToolShape::flatEndMill(3.0);
    Simulator sim(stockOf(100, 40, 0.25, 10.0));
    Toolpath beyond = stroke(0b1111);  // spindle 4 does not exist
    CHECK_FALSE(sim.addToolpath(beyond, flat, 1));
    CHECK(sim.error().status == simulate::SimStatus::InvalidMove);
    CHECK(sim.moveCount() == 0);
    Toolpath noHead = stroke(0b001);
    noHead.bankHead.clear();
    CHECK_FALSE(sim.addToolpath(noHead, flat, 1));
    CHECK(sim.error().status == simulate::SimStatus::InvalidMove);
    Toolpath zero = stroke(0b001);
    zero.bankHead[0].diameter = 0.0;
    CHECK_FALSE(sim.addToolpath(zero, flat, 1));
    CHECK(sim.error().status == simulate::SimStatus::InvalidTool);
    Toolpath far = stroke(0b001);
    far.bankHead[0].offset.x = 1e9;
    CHECK_FALSE(sim.addToolpath(far, flat, 1));
    CHECK(sim.error().status == simulate::SimStatus::InvalidMove);
    Toolpath negative = stroke(0b001);
    negative.bankHead[0].depth = -1.0;
    CHECK_FALSE(sim.addToolpath(negative, flat, 1));
    // An unused spindle with a bad diameter does not matter; a good toolpath is accepted afterwards.
    Toolpath unused = stroke(0b001);
    unused.bankHead[2].diameter = 0.0;
    CHECK(sim.addToolpath(unused, flat, 1));
    Toolpath arc = stroke(0b001);
    arc.moves.push_back(banked(Move::arcCW({60, 20, -2}, {55, 20}), 0b001));
    CHECK_FALSE(sim.addToolpath(arc, flat, 1));
    CHECK(sim.error().status == simulate::SimStatus::InvalidMove);
    // A toolpath with no spindle sets ignores any head.
    Toolpath plain;
    plain.tool.feedRate = 1000.0;
    plain.tool.plungeRate = 300.0;
    plain.bankHead = head();
    plain.moves = {Move::rapid({10, 10, 5}), Move::plunge({10, 10, -2})};
    Simulator other(stockOf(100, 40, 0.25, 10.0));
    REQUIRE(other.addToolpath(plain, flat, 1));
    other.advanceAll();
    CHECK(heightAt(other, 10, 10) == Approx(-2.0).margin(1e-6));
    CHECK(heightAt(other, 30, 10) == Approx(0.0).margin(1e-6));  // no spindle at the head offset
}
