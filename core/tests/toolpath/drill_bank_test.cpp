// T-0328: drill bank toolpath (drill_bank.hpp, ADR-0106). Bank layouts and parameter checks, matching of holes to
// the spindle pattern (tolerances, diameters, switched-off spindles, reference drill, row/column offsets, fewest holes
// per stroke), the greedy choice of strokes, refusal and reporting of unmatched holes, per-spindle depth, reuse of
// the Drilling moves (peck, dwell), the order of the strokes, and the work counters.

#include <algorithm>
#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/drill_bank.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;
using Catch::Approx;

namespace {

tooling::Tool drillTool(double diameter = 3.0, double passDepth = 2.0) {
    tooling::Tool t;
    t.id = "drill";
    t.name = "Drill";
    t.type = tooling::ToolType::Drill;
    t.diameter = diameter;
    t.includedAngle = 118.0 * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = 7;
    t.defaultCuttingData.feedRate = 500.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 10000.0;
    t.defaultCuttingData.passDepth = passDepth;
    return t;
}

DrillBankParams bank(int horizontal = 4, int vertical = 0) {
    DrillBankParams p;
    p.horizontalDrills = horizontal;
    p.verticalDrills = vertical;
    p.pitch = 32.0;
    p.cutDepth = 10.0;
    p.safeZ = 5.0;
    return p;
}

Contour hole(double x, double y, double radius = 1.5) {
    return Contour::circle({x, y}, radius);
}

/// Holes of a row: `count` holes from (x0, y) at `step`.
std::vector<Contour> row(double x0, double y, int count, double step = 32.0, double radius = 1.5) {
    std::vector<Contour> out;
    for (int i = 0; i < count; ++i) {
        out.push_back(hole(x0 + step * i, y, radius));
    }
    return out;
}

void append(std::vector<Contour>& to, const std::vector<Contour>& more) {
    to.insert(to.end(), more.begin(), more.end());
}

DrillBankResult run(const std::vector<Contour>& cs, const DrillBankParams& p, const tooling::Tool& t = drillTool()) {
    return drillBankToolpath(cs, t, std::nullopt, p);
}

bool hasMessage(const DrillBankResult& r, ToolpathIssue issue, const std::string& part) {
    for (const ToolpathDiagnostic& d : r.diagnostics) {
        if (d.issue == issue && d.message.find(part) != std::string::npos) {
            return true;
        }
    }
    return false;
}

std::size_t count(const DrillBankResult& r, ToolpathIssue issue) {
    std::size_t n = 0;
    for (const ToolpathDiagnostic& d : r.diagnostics) {
        n += d.issue == issue ? 1 : 0;
    }
    return n;
}

std::size_t drilledHoles(const DrillBankResult& r) {
    std::size_t n = 0;
    for (const DrillBankPlacement& pl : r.placements) {
        n += pl.holes.size();
    }
    return n;
}

}  // namespace

TEST_CASE("drill bank: spindle layout", "[toolpath][drill_bank]") {
    DrillBankParams p = bank(4, 3);
    p.rowOffset = 10.0;
    p.columnOffset = -5.0;
    const std::vector<DrillBankSpindle> s = drillBankSpindles(p);
    REQUIRE(s.size() == 7);
    const double rowX[] = {-48.0, -16.0, 16.0, 48.0};
    for (int i = 0; i < 4; ++i) {
        CHECK(s[static_cast<std::size_t>(i)].number == i + 1);
        CHECK(s[static_cast<std::size_t>(i)].offset.x == Approx(rowX[i]));
        CHECK(s[static_cast<std::size_t>(i)].offset.y == Approx(10.0));
    }
    const double colY[] = {-32.0, 0.0, 32.0};
    for (int j = 0; j < 3; ++j) {
        const DrillBankSpindle& c = s[static_cast<std::size_t>(4 + j)];
        CHECK(c.number == 5 + j);
        CHECK(c.offset.x == Approx(-5.0));
        CHECK(c.offset.y == Approx(colY[j]));
    }
    // A single spindle sits at the axis (plus the offset); the settings follow the numbers.
    p = bank(1, 0);
    CHECK(drillBankSpindles(p).at(0).offset == Vec2{0.0, 0.0});
    p = bank(2, 2);
    p.inactiveSpindles = {2, 4};
    p.spindleDiameters = {3, 4, 5, 6};
    p.spindleDepths = {0, 7, 8, 9};
    const std::vector<DrillBankSpindle> t = drillBankSpindles(p);
    CHECK(t[0].active);
    CHECK_FALSE(t[1].active);
    CHECK(t[2].active);
    CHECK_FALSE(t[3].active);
    CHECK(t[0].diameter == 3.0);
    CHECK(t[3].diameter == 6.0);
    CHECK(t[0].depth == 0.0);
    CHECK(t[2].depth == 8.0);
    // Short lists leave the rest at the defaults.
    p.spindleDiameters = {3};
    p.spindleDepths = {};
    CHECK(drillBankSpindles(p)[1].diameter == 0.0);
    CHECK(drillBankSpindles(p)[1].depth == 0.0);
}

TEST_CASE("drill bank: parameter checks", "[toolpath][drill_bank]") {
    CHECK(drillBankParamProblem(bank()).empty());
    const auto bad = [](DrillBankParams p, const std::string& part) {
        INFO(part);
        const std::string problem = drillBankParamProblem(p);
        CHECK(problem.find(part) != std::string::npos);
        // The toolpath itself refuses the same way.
        const DrillBankResult r = run(row(0, 0, 4), p);
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::InvalidParameters));
        CHECK(r.toolpath.moves.empty());
    };
    DrillBankParams p = bank();
    p.horizontalDrills = -1;
    bad(p, "drills on its row");
    p = bank();
    p.horizontalDrills = 33;
    bad(p, "drills on its row");
    p = bank();
    p.verticalDrills = 33;
    bad(p, "drills on its row");
    p = bank(0, 0);
    bad(p, "no drills");
    for (const double v : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), 2.0e6}) {
        p = bank();
        p.pitch = v;
        bad(p, "pitch must be");
    }
    p = bank();
    p.rowOffset = std::numeric_limits<double>::infinity();
    bad(p, "offsets");
    p = bank();
    p.columnOffset = 2.0e6;
    bad(p, "offsets");
    p = bank();
    p.referenceDrill = 0;
    bad(p, "reference drill");
    p.referenceDrill = 5;
    bad(p, "reference drill");
    p = bank();
    p.referenceDrill = 4;  // the last spindle is fine
    CHECK(drillBankParamProblem(p).empty());
    p = bank();
    p.inactiveSpindles = {0};
    bad(p, "does not exist");
    p.inactiveSpindles = {5};
    bad(p, "does not exist");
    p.inactiveSpindles = {2, 2};
    bad(p, "twice");
    p.inactiveSpindles = {1, 2, 3, 4};
    bad(p, "every drill is switched off");
    p = bank();
    p.spindleDiameters = {3, 3};
    bad(p, "diameter for every drill");
    p.spindleDiameters = {3, 3, 3, 0};
    bad(p, "diameters must be");
    p.spindleDiameters = {3, 3, 3, std::numeric_limits<double>::quiet_NaN()};
    bad(p, "diameters must be");
    p = bank();
    p.spindleDepths = {1, 1, 1};
    bad(p, "depth for every drill");
    p.spindleDepths = {1, 1, 1, -1};
    bad(p, "depths must be");
    p.spindleDepths = {1, 1, 1, 2.0e5};
    bad(p, "depths must be");
    p = bank();
    p.pitchTolerance = 0.0;
    bad(p, "pitch tolerance must be at least");
    p.pitchTolerance = std::numeric_limits<double>::quiet_NaN();
    bad(p, "pitch tolerance must be at least");
    p = bank();
    p.diameterTolerance = -0.1;
    bad(p, "diameter tolerance");
    p = bank();
    p.minHoles = 0;
    bad(p, "minimum number of holes");
    p.minHoles = 5;
    bad(p, "minimum number of holes");
    p = bank();
    p.inactiveSpindles = {1};
    p.minHoles = 4;  // only three spindles are active
    bad(p, "minimum number of holes");
    p.minHoles = 3;
    CHECK(drillBankParamProblem(p).empty());
    // Closest spindles: tolerance zones must not overlap (pitch 32: the bound is 16).
    p = bank();
    p.pitchTolerance = 16.0;
    bad(p, "less than half the distance");
    p.pitchTolerance = 15.999;
    CHECK(drillBankParamProblem(p).empty());
    // The column's offset can bring spindles closer than the pitch.
    p = bank(2, 2);
    p.pitch = 10.0;
    p.rowOffset = 0.0;
    p.columnOffset = 4.0;  // column at x = 4, row at x = +-5: distance between (5, 0) and (4, +-5)...
    p.pitchTolerance = 2.0;
    CHECK(drillBankParamProblem(p).empty());
    p.pitchTolerance = 3.0;  // closest pair: (5,0) and (4,5): sqrt(26) = 5.099 -> bound 2.549
    bad(p, "less than half the distance");
    // Spindles at the same place are refused whatever the tolerance.
    p = bank(3, 3);
    p.pitchTolerance = 1e-6;
    bad(p, "drills at the same place");
}

TEST_CASE("drill bank: parameter limits are inclusive", "[toolpath][drill_bank]") {
    CHECK(drillBankParamProblem(bank(32, 0)).empty());
    CHECK(drillBankParamProblem(bank(0, 32)).empty());
    DrillBankParams p = bank(1, 0);
    CHECK_FALSE(drillBankParamProblem(p).empty());  // the default of two holes per stroke needs two drills
    p.minHoles = 1;
    CHECK(drillBankParamProblem(p).empty());
    p = bank();
    p.pitch = 1.0e6;
    CHECK(drillBankParamProblem(p).empty());
    p.pitch = 1.0e6 * 1.0000001;
    CHECK_FALSE(drillBankParamProblem(p).empty());
    p = bank();
    p.rowOffset = 1.0e6;
    p.columnOffset = -1.0e6;
    CHECK(drillBankParamProblem(p).empty());
    p.rowOffset = 1.0000001e6;
    CHECK_FALSE(drillBankParamProblem(p).empty());
    p = bank();
    p.spindleDepths = {0.0, 1.0e5, 1.0, 1.0};
    p.spindleDiameters = {1.0e6, 1.0e-9, 1.0, 1.0};
    CHECK(drillBankParamProblem(p).empty());
    p.spindleDepths = {0.0, 1.0000001e5, 1.0, 1.0};
    CHECK_FALSE(drillBankParamProblem(p).empty());
    p = bank();
    p.spindleDiameters = {1.0000001e6, 1.0, 1.0, 1.0};
    CHECK_FALSE(drillBankParamProblem(p).empty());
    p = bank();
    p.pitchTolerance = 1e-6;
    CHECK(drillBankParamProblem(p).empty());
    p.pitchTolerance = 0.9e-6;
    CHECK_FALSE(drillBankParamProblem(p).empty());
    p = bank();
    p.diameterTolerance = 0.0;
    CHECK(drillBankParamProblem(p).empty());
    p.minHoles = 4;
    CHECK(drillBankParamProblem(p).empty());
}

TEST_CASE("drill bank: the diameter of a hole is that of the circle of equal area", "[toolpath][drill_bank]") {
    CHECK(drillBankHoleDiameter(hole(5, 5, 2.5)) == Approx(5.0).margin(1e-9));
    CHECK(drillBankHoleDiameter(Contour::rectangle({{0, 0}, {10, 10}})) == Approx(2.0 * std::sqrt(100.0 / geometry::kPi)));
    CHECK(drillBankHoleDiameter(Contour{}) == 0.0);
}

TEST_CASE("drill bank: a row of holes at the pitch is one stroke", "[toolpath][drill_bank]") {
    const DrillBankResult r = run(row(10, 20, 4), bank());
    REQUIRE(r.ok());
    REQUIRE(r.placements.size() == 1);
    const DrillBankPlacement& pl = r.placements[0];
    CHECK(pl.reference.x == Approx(10.0));
    CHECK(pl.reference.y == Approx(20.0));
    REQUIRE(pl.holes.size() == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(pl.holes[i].spindle == static_cast<int>(i) + 1);
        CHECK(pl.holes[i].contour == i);
        CHECK(pl.holes[i].centre.x == Approx(10.0 + 32.0 * static_cast<double>(i)));
        CHECK(pl.holes[i].deviation == Approx(0.0).margin(1e-9));
        CHECK(pl.holes[i].depth == 10.0);
    }
    CHECK(pl.depth == 10.0);
    CHECK(r.unmatched.empty());
    CHECK(r.diagnostics.empty());
    // One stroke: rapid at safe Z, plunge to the bottom, retract.
    REQUIRE(r.toolpath.moves.size() == 3);
    CHECK(r.toolpath.moves[0].kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves[0].to == geometry::Vec3{10.0, 20.0, 5.0});
    CHECK(r.toolpath.moves[1].kind == MoveKind::Plunge);
    CHECK(r.toolpath.moves[1].to == geometry::Vec3{10.0, 20.0, -10.0});
    CHECK(r.toolpath.moves[2].kind == MoveKind::Retract);
    CHECK(r.toolpath.moves[2].to == geometry::Vec3{10.0, 20.0, 5.0});
    CHECK(r.toolpath.name == "Drill bank");
    CHECK(r.toolpath.tool.name == "Drill");
}

TEST_CASE("drill bank: strokes are the greedy cover, ties to the lower anchor hole", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    p.order.selectionOrder = true;  // placements in the order they were chosen
    SECTION("six holes in a row: four, then the last two") {
        const DrillBankResult r = run(row(0, 0, 6), p);
        REQUIRE(r.ok());
        REQUIRE(r.placements.size() == 2);
        CHECK(r.placements[0].reference.x == Approx(0.0));
        CHECK(r.placements[0].holes.size() == 4);
        CHECK(r.placements[1].reference.x == Approx(128.0));
        CHECK(r.placements[1].holes.size() == 2);
        CHECK(r.unmatched.empty());
    }
    SECTION("five holes: the fifth cannot be drilled together with another one") {
        const DrillBankResult r = run(row(0, 0, 5), p);
        CHECK_FALSE(r.ok());
        CHECK(r.unmatched.size() == 1);
        CHECK(r.unmatched[0].contour == 4);
        CHECK(r.toolpath.moves.empty());
        DrillBankParams skip = p;
        skip.unmatched = BankUnmatched::Skip;
        const DrillBankResult s = run(row(0, 0, 5), skip);
        REQUIRE(s.ok());
        REQUIRE(s.placements.size() == 1);
        CHECK(s.placements[0].holes.size() == 4);
        CHECK(s.unmatched.size() == 1);
        // minHoles 1: a stroke of one hole is made.
        DrillBankParams one = p;
        one.minHoles = 1;
        const DrillBankResult o = run(row(0, 0, 5), one);
        REQUIRE(o.ok());
        CHECK(o.placements.size() == 2);
        CHECK(o.placements[1].holes.size() == 1);
        CHECK(o.unmatched.empty());
    }
    SECTION("the better stroke wins over the earlier one") {
        // A pair first in the selection, then a row of four: the row is chosen first.
        std::vector<Contour> cs = row(0, 100, 2);
        append(cs, row(300, 0, 4));
        const DrillBankResult r = run(cs, p);
        REQUIRE(r.ok());
        REQUIRE(r.placements.size() == 2);
        CHECK(r.placements[0].holes.size() == 4);
        CHECK(r.placements[1].holes.size() == 2);
    }
    SECTION("a candidate whose gain fell is queued again behind better ones (lazy evaluation)") {
        // Three drills, pitch 32, minHoles 1, holes at x = 144, 176, 80 and 48. The first stroke (reference 144) takes
        // 144 and 176. A candidate queued with a larger gain than it has now (one that also counted 144 or 176) is
        // put back behind the better candidates; if it were taken at once, the holes 48 and 80 would be split into
        // two single-hole strokes instead of one stroke of two.
        DrillBankParams three = bank(3, 0);
        three.order.selectionOrder = true;
        three.minHoles = 1;
        three.unmatched = BankUnmatched::Skip;
        std::vector<Contour> cs{hole(144, 0), hole(176, 0), hole(80, 0), hole(48, 0)};
        const DrillBankResult r = run(cs, three);
        REQUIRE(r.ok());
        REQUIRE(r.placements.size() == 2);
        CHECK(r.placements[0].holes.size() == 2);
        CHECK(r.placements[1].holes.size() == 2);
        CHECK(drilledHoles(r) == 4);
    }
    SECTION("each hole is drilled once; an overlapping candidate is re-evaluated") {
        // Five holes in a row, minHoles 2: the candidate anchored on the second hole had gain 4 before the first
        // stroke took four holes; afterwards it would drill one hole, so it is dropped.
        DrillBankParams skip = p;
        skip.unmatched = BankUnmatched::Skip;
        const DrillBankResult r = run(row(0, 0, 5), skip);
        REQUIRE(r.ok());
        CHECK(drilledHoles(r) == 4);
        std::vector<std::size_t> seen;
        for (const DrillBankPlacement& pl : r.placements) {
            for (const DrillBankHole& h : pl.holes) {
                CHECK(std::find(seen.begin(), seen.end(), h.contour) == seen.end());
                seen.push_back(h.contour);
            }
        }
    }
}

TEST_CASE("drill bank: pitch tolerance", "[toolpath][drill_bank]") {
    // Holes 32.0625 apart (exactly representable): the second one is 0.0625 from its spindle.
    std::vector<Contour> cs{hole(0, 0), hole(32.0625, 0)};
    DrillBankParams p = bank();
    p.pitchTolerance = 0.0625;
    const DrillBankResult r = run(cs, p);
    REQUIRE(r.ok());
    REQUIRE(r.placements.size() == 1);
    REQUIRE(r.placements[0].holes.size() == 2);
    const DrillBankHole& moved = r.placements[0].holes[1];
    CHECK(moved.deviation == Approx(0.0625));
    CHECK(moved.centre.x == Approx(32.0625));
    CHECK(moved.drilledAt.x == Approx(32.0));
    // The moved hole is reported once, with the largest deviation.
    CHECK(count(r, ToolpathIssue::BankHoleMoved) == 1);
    CHECK(hasMessage(r, ToolpathIssue::BankHoleMoved, "1 hole(s)"));
    CHECK(hasMessage(r, ToolpathIssue::BankHoleMoved, "0.0625"));
    // A hair less tolerance: no stroke drills both.
    p.pitchTolerance = 0.0624;
    const DrillBankResult s = run(cs, p);
    CHECK_FALSE(s.ok());
    CHECK(s.unmatched.size() == 2);
    // A deviation within the merge distance is not worth a report.
    DrillBankParams exact = bank();
    exact.tolerance.distance = 0.0625;
    exact.tolerance.chordal = 0.0625;
    exact.pitchTolerance = 0.0625;
    const DrillBankResult q = run(cs, exact);
    REQUIRE(q.ok());
    CHECK(count(q, ToolpathIssue::BankHoleMoved) == 0);
    // Two candidate holes for one spindle: the nearer one is drilled.
    std::vector<Contour> two{hole(0, 0), hole(32.0625, 0), hole(32.03125, 0)};
    DrillBankParams near = bank();
    near.unmatched = BankUnmatched::Skip;
    near.pitchTolerance = 0.0625;
    near.tolerance.distance = 0.01;
    const DrillBankResult n = run(two, near);
    REQUIRE(n.ok());
    REQUIRE(n.placements[0].holes.size() == 2);
    CHECK(n.placements[0].holes[1].contour == 2);
    CHECK(n.unmatched.size() == 1);
}

TEST_CASE("drill bank: two holes equally near one spindle, the lower vector index is drilled", "[toolpath][drill_bank]") {
    // Holes 0.03125 either side of the second spindle's position (all exactly representable): equal distances.
    DrillBankParams p = bank();
    p.unmatched = BankUnmatched::Skip;
    p.pitchTolerance = 0.03125;
    p.tolerance.distance = 0.01;
    SECTION("the lower index first") {
        const std::vector<Contour> cs{hole(0, 0), hole(32.03125, 0), hole(31.96875, 0)};
        const DrillBankResult r = run(cs, p);
        REQUIRE(r.ok());
        REQUIRE(r.placements.size() == 1);
        REQUIRE(r.placements[0].holes.size() == 2);
        CHECK(r.placements[0].holes[1].contour == 1);
        CHECK(r.unmatched.size() == 1);
        CHECK(r.unmatched[0].contour == 2);
    }
    SECTION("the lower index last in position") {
        const std::vector<Contour> cs{hole(0, 0), hole(31.96875, 0), hole(32.03125, 0)};
        const DrillBankResult r = run(cs, p);
        REQUIRE(r.ok());
        REQUIRE(r.placements.size() == 1);
        REQUIRE(r.placements[0].holes.size() == 2);
        CHECK(r.placements[0].holes[1].contour == 1);
        CHECK(r.unmatched.size() == 1);
        CHECK(r.unmatched[0].contour == 2);
    }
}

TEST_CASE("drill bank: a deviation larger than the merge distance on the worst hole is the one named",
          "[toolpath][drill_bank]") {
    std::vector<Contour> cs{hole(0, 0), hole(32.03125, 0), hole(64.0625, 0)};
    DrillBankParams p = bank();
    p.pitchTolerance = 0.25;
    const DrillBankResult r = run(cs, p);
    REQUIRE(r.ok());
    CHECK(count(r, ToolpathIssue::BankHoleMoved) == 1);
    CHECK(hasMessage(r, ToolpathIssue::BankHoleMoved, "2 hole(s)"));
    CHECK(hasMessage(r, ToolpathIssue::BankHoleMoved, "vector 2"));
    for (const ToolpathDiagnostic& d : r.diagnostics) {
        if (d.issue == ToolpathIssue::BankHoleMoved) {
            CHECK(d.contour == std::optional<std::size_t>(2));
        }
    }
}

TEST_CASE("drill bank: diameters", "[toolpath][drill_bank]") {
    // Four spindles of 5, 5, 8, 8 mm; holes of those diameters at the pitch.
    DrillBankParams p = bank();
    p.matchDiameters = true;
    p.spindleDiameters = {5, 5, 8, 8};
    std::vector<Contour> cs{hole(0, 0, 2.5), hole(32, 0, 2.5), hole(64, 0, 4), hole(96, 0, 4)};
    const DrillBankResult r = run(cs, p);
    REQUIRE(r.ok());
    REQUIRE(r.placements.size() == 1);
    CHECK(r.placements[0].holes.size() == 4);

    SECTION("a hole of the wrong size is not drilled by that spindle") {
        std::vector<Contour> swapped{hole(0, 0, 2.5), hole(32, 0, 2.5), hole(64, 0, 2.5), hole(96, 0, 4)};
        DrillBankParams skip = p;
        skip.unmatched = BankUnmatched::Skip;
        const DrillBankResult s = run(swapped, skip);
        REQUIRE(s.ok());
        CHECK(s.placements[0].holes.size() == 3);
        REQUIRE(s.unmatched.size() == 1);
        CHECK(s.unmatched[0].contour == 2);
        CHECK_FALSE(s.unmatched[0].diameter);  // a spindle of 8 mm exists; the pattern does not match
    }
    SECTION("tolerance") {
        std::vector<Contour> off{hole(0, 0, 2.5 + 0.075), hole(32, 0, 2.5 - 0.075), hole(64, 0, 4), hole(96, 0, 4)};
        p.diameterTolerance = 0.2;  // the holes differ by 0.15
        CHECK(run(off, p).ok());
        p.diameterTolerance = 0.1;
        const DrillBankResult s = run(off, p);
        CHECK_FALSE(s.ok());
        CHECK(s.unmatched.size() == 2);
        CHECK(s.unmatched[0].diameter);
        CHECK(hasMessage(s, ToolpathIssue::BankHoleUnmatched, "fits no active drill"));
    }
    SECTION("off: the diameters are not compared") {
        p.matchDiameters = false;
        std::vector<Contour> any{hole(0, 0, 1), hole(32, 0, 9), hole(64, 0, 2), hole(96, 0, 3)};
        const DrillBankResult s = run(any, p);
        REQUIRE(s.ok());
        CHECK(s.placements[0].holes.size() == 4);
    }
    SECTION("without spindle diameters the tool's diameter is every spindle's") {
        DrillBankParams q = bank();
        q.matchDiameters = true;
        const DrillBankResult three = run(row(0, 0, 4, 32.0, 1.5), q, drillTool(3.0));
        CHECK(three.ok());
        const DrillBankResult five = run(row(0, 0, 4, 32.0, 2.5), q, drillTool(3.0));
        CHECK_FALSE(five.ok());
        CHECK(five.unmatched.size() == 4);
        CHECK(five.unmatched[0].diameter);
        CHECK(run(row(0, 0, 4, 32.0, 2.5), q, drillTool(5.0)).ok());
    }
}

TEST_CASE("drill bank: switched-off spindles and the reference drill", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    p.inactiveSpindles = {2};
    SECTION("a stroke skips the switched-off spindle") {
        // Holes under spindles 1, 3 and 4 only.
        std::vector<Contour> cs{hole(0, 0), hole(64, 0), hole(96, 0)};
        const DrillBankResult r = run(cs, p);
        REQUIRE(r.ok());
        REQUIRE(r.placements.size() == 1);
        REQUIRE(r.placements[0].holes.size() == 3);
        CHECK(r.placements[0].holes[0].spindle == 1);
        CHECK(r.placements[0].holes[1].spindle == 3);
        CHECK(r.placements[0].holes[2].spindle == 4);
    }
    SECTION("a hole where only the switched-off spindle would be is unmatched") {
        std::vector<Contour> cs = row(0, 0, 4);
        const DrillBankResult r = run(cs, p);
        CHECK_FALSE(r.ok());
        REQUIRE(r.unmatched.size() == 1);
        CHECK(r.unmatched[0].contour == 1);
        CHECK(hasMessage(r, ToolpathIssue::BankHoleUnmatched, "together with at least 1 other"));
        CHECK(hasMessage(r, ToolpathIssue::BankHoleUnmatched, "nothing is cut"));
    }
    SECTION("the reference drill can be switched off; the moves are its") {
        p.referenceDrill = 2;
        std::vector<Contour> cs{hole(0, 0), hole(64, 0), hole(96, 0)};
        const DrillBankResult r = run(cs, p);
        REQUIRE(r.ok());
        // Spindle 2 is one pitch right of spindle 1.
        CHECK(r.placements[0].reference.x == Approx(32.0));
        CHECK(r.toolpath.moves[0].to.x == Approx(32.0));
    }
    SECTION("reference drill 3 of an active bank") {
        DrillBankParams q = bank();
        q.referenceDrill = 3;
        const DrillBankResult r = run(row(10, 20, 4), q);
        REQUIRE(r.ok());
        CHECK(r.placements[0].reference.x == Approx(74.0));
        CHECK(r.placements[0].reference.y == Approx(20.0));
        CHECK(r.placements[0].holes[0].spindle == 1);
        CHECK(r.toolpath.moves[1].to.x == Approx(74.0));
    }
}

TEST_CASE("drill bank: row and column offsets, horizontal and vertical drills", "[toolpath][drill_bank]") {
    DrillBankParams p = bank(2, 2);
    p.pitch = 20.0;
    p.rowOffset = 10.0;
    p.columnOffset = -5.0;
    p.minHoles = 4;  // only the whole head's pattern counts
    // The head at (100, 100): row spindles at (90, 110), (110, 110); column at (95, 90), (95, 110).
    std::vector<Contour> cs{hole(90, 110), hole(110, 110), hole(95, 90), hole(95, 110)};
    const DrillBankResult r = run(cs, p);
    REQUIRE(r.ok());
    REQUIRE(r.placements.size() == 1);
    REQUIRE(r.placements[0].holes.size() == 4);
    CHECK(r.placements[0].reference.x == Approx(90.0));
    CHECK(r.placements[0].reference.y == Approx(110.0));
    CHECK(r.placements[0].holes[0].spindle == 1);
    CHECK(r.placements[0].holes[1].spindle == 2);
    CHECK(r.placements[0].holes[2].spindle == 3);
    CHECK(r.placements[0].holes[3].spindle == 4);
    // The same holes with the signs of the offsets swapped are not a pattern of this head.
    DrillBankParams flipped = p;
    flipped.rowOffset = -10.0;
    CHECK_FALSE(run(cs, flipped).ok());
    flipped = p;
    flipped.columnOffset = 5.0;
    CHECK_FALSE(run(cs, flipped).ok());
    // A vertical-only head: a column of three at the pitch.
    DrillBankParams column = bank(0, 3);
    const DrillBankResult c = run({hole(7, 0), hole(7, 32), hole(7, 64)}, column);
    REQUIRE(c.ok());
    REQUIRE(c.placements.size() == 1);
    CHECK(c.placements[0].holes.size() == 3);
    CHECK(c.placements[0].reference.x == Approx(7.0));
    CHECK(c.placements[0].reference.y == Approx(0.0));
    // The row is horizontal: holes in a column do not fit a horizontal-only head.
    CHECK_FALSE(run({hole(7, 0), hole(7, 32), hole(7, 64)}, bank(3, 0)).ok());
}

TEST_CASE("drill bank: per-spindle depth", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    p.cutDepth = 4.0;
    p.spindleDepths = {5.0, 8.0, 0.0, 0.0};
    const DrillBankResult r = run(row(0, 0, 4), p);
    REQUIRE(r.ok());
    const DrillBankPlacement& pl = r.placements[0];
    CHECK(pl.holes[0].depth == 5.0);
    CHECK(pl.holes[1].depth == 8.0);
    CHECK(pl.holes[2].depth == 4.0);  // 0 = the toolpath's cut depth
    CHECK(pl.holes[3].depth == 4.0);
    CHECK(pl.depth == 8.0);
    REQUIRE(r.toolpath.moves.size() == 3);
    CHECK(r.toolpath.moves[1].to.z == Approx(-8.0));

    SECTION("a stroke of shallow spindles only goes as deep as they do") {
        // Two strokes: spindles 3, 4 (depth 4) and spindles 1, 2 (depths 5, 8) in separate places.
        DrillBankParams q = p;
        q.inactiveSpindles = {1, 2};
        std::vector<Contour> shallow{hole(64, 100), hole(96, 100)};
        const DrillBankResult s = run(shallow, q);
        REQUIRE(s.ok());
        CHECK(s.placements[0].depth == 4.0);
        CHECK(s.toolpath.moves[1].to.z == Approx(-4.0));
    }
    SECTION("start depth and peck levels follow each stroke's own depth") {
        DrillBankParams q = bank(2, 0);
        q.startDepth = 1.0;
        q.cutDepth = 4.0;
        q.peck = true;
        q.retractGap = 1.0;
        q.spindleDepths = {0.0, 7.0};
        q.order.selectionOrder = true;
        // Two strokes of a two-spindle head: the first drills with spindle 1 and 2 (depth 7), the second with 1 only
        // is not possible (minHoles 2), so use minHoles 1 and holes far apart.
        q.minHoles = 1;
        std::vector<Contour> cs{hole(0, 0), hole(32, 0), hole(200, 0)};
        const DrillBankResult s = run(cs, q);  // pass depth 2
        REQUIRE(s.ok());
        REQUIRE(s.placements.size() == 2);
        CHECK(s.placements[0].depth == 7.0);
        CHECK(s.placements[1].depth == 4.0);
        // Stroke 1: levels -3, -5, -7, -8 (start depth 1, depth 7, step 2, the last the remainder -8).
        std::vector<double> plunges1;
        std::vector<double> plunges2;
        std::size_t strokes = 0;
        for (const Move& m : s.toolpath.moves) {
            if (m.kind == MoveKind::Rapid) {
                ++strokes;
            }
            if (m.kind == MoveKind::Plunge) {
                (strokes == 1 ? plunges1 : plunges2).push_back(m.to.z);
            }
        }
        CHECK(plunges1 == std::vector<double>{-3.0, -5.0, -7.0, -8.0});
        CHECK(plunges2 == std::vector<double>{-3.0, -5.0});
    }
}

TEST_CASE("drill bank: a one-spindle bank makes the Drilling moves", "[toolpath][drill_bank]") {
    DrillBankParams p = bank(1, 0);
    p.minHoles = 1;
    p.startDepth = 0.5;
    p.cutDepth = 7.0;
    p.peck = true;
    p.retractGap = 1.5;
    p.dwellTime = 0.25;
    p.startPosition = {5.0, -5.0};
    for (const PeckRetract retract : {PeckRetract::AboveStart, PeckRetract::AbovePreviousPass}) {
        p.peckRetract = retract;
        std::vector<Contour> cs{hole(10, 10), hole(60, 0), hole(30, 40), hole(0, 0)};
        const DrillBankResult r = run(cs, p);
        const DrillResult d = drillToolpath(cs, drillTool(), std::nullopt, p);  // slices to the DrillParams
        REQUIRE(r.ok());
        REQUIRE(d.ok());
        // T-0335: the bank's moves also carry the spindle set (here spindle 1); the moves are otherwise Drilling's.
        std::vector<Move> bankMoves = r.toolpath.moves;
        for (Move& m : bankMoves) {
            CHECK(m.bank == 1);
            m.bank = 0;
        }
        CHECK(bankMoves == d.toolpath.moves);
        CHECK(r.toolpath.tool == d.toolpath.tool);
        REQUIRE(r.placements.size() == d.holes.size());
        for (std::size_t i = 0; i < d.holes.size(); ++i) {
            CHECK(r.placements[i].reference == d.holes[i].centre);
        }
    }
}

TEST_CASE("drill bank: order of the strokes", "[toolpath][drill_bank]") {
    DrillBankParams p = bank(2, 0);
    p.minHoles = 2;
    // Three pairs; vector order is far, near, middle.
    std::vector<Contour> cs{hole(300, 0), hole(332, 0), hole(0, 0), hole(32, 0), hole(150, 0), hole(182, 0)};
    SECTION("nearest first from the start position") {
        p.startPosition = {0, 0};
        const DrillBankResult r = run(cs, p);
        REQUIRE(r.placements.size() == 3);
        CHECK(r.placements[0].reference.x == Approx(0.0));
        CHECK(r.placements[1].reference.x == Approx(150.0));
        CHECK(r.placements[2].reference.x == Approx(300.0));
        CHECK(r.toolpath.moves[0].to.x == Approx(0.0));
        // The start position matters.
        p.startPosition = {400, 0};
        const DrillBankResult f = run(cs, p);
        REQUIRE(f.placements.size() == 3);
        CHECK(f.placements[0].reference.x == Approx(300.0));
        CHECK(f.placements[1].reference.x == Approx(150.0));
        CHECK(f.placements[2].reference.x == Approx(0.0));
    }
    SECTION("the shared order options apply to the strokes") {
        p.order.selectionOrder = true;  // the order in which the strokes were chosen: by anchor hole
        const DrillBankResult s = run(cs, p);
        REQUIRE(s.placements.size() == 3);
        CHECK(s.placements[0].reference.x == Approx(300.0));
        CHECK(s.placements[1].reference.x == Approx(0.0));
        CHECK(s.placements[2].reference.x == Approx(150.0));
        p.order.selectionOrder = false;
        p.order.leftToRight = true;
        const DrillBankResult l = run(cs, p);
        REQUIRE(l.placements.size() == 3);
        CHECK(l.placements[0].reference.x == Approx(0.0));
        CHECK(l.placements[1].reference.x == Approx(150.0));
        CHECK(l.placements[2].reference.x == Approx(300.0));
    }
    SECTION("the nearest order travels less than the order of selection") {
        p.startPosition = {0, 0};
        const DrillBankResult near = run(cs, p);
        p.order.selectionOrder = true;
        const DrillBankResult sel = run(cs, p);
        const auto travel = [](const DrillBankResult& r) {
            double len = 0.0;
            Vec2 at{0, 0};
            for (const DrillBankPlacement& pl : r.placements) {
                len += geometry::distance(at, pl.reference);
                at = pl.reference;
            }
            return len;
        };
        CHECK(travel(near) < travel(sel));
    }
}

TEST_CASE("drill bank: unmatched holes are reported by name", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    std::vector<Contour> cs = row(0, 0, 4);
    cs.push_back(hole(1000, 1000));
    cs.push_back(hole(2000, 1000));
    const DrillBankResult r = run(cs, p);
    CHECK_FALSE(r.ok());
    CHECK(r.toolpath.moves.empty());
    CHECK(r.placements.empty());
    REQUIRE(r.unmatched.size() == 2);
    CHECK(r.unmatched[0].contour == 4);
    CHECK(r.unmatched[0].centre.x == Approx(1000.0));
    CHECK_FALSE(r.unmatched[0].diameter);
    CHECK(count(r, ToolpathIssue::BankHoleUnmatched) == 3);  // two holes and the summary
    CHECK(hasMessage(r, ToolpathIssue::BankHoleUnmatched, "vector 4 (hole at 1000, 1000)"));
    CHECK(hasMessage(r, ToolpathIssue::BankHoleUnmatched, "2 hole(s) fit no stroke of the drill bank; nothing is cut"));
    for (const ToolpathDiagnostic& d : r.diagnostics) {
        if (d.issue == ToolpathIssue::BankHoleUnmatched && d.contour) {
            CHECK((*d.contour == 4 || *d.contour == 5));
        }
    }

    p.unmatched = BankUnmatched::Skip;
    const DrillBankResult s = run(cs, p);
    REQUIRE(s.ok());
    CHECK(s.placements.size() == 1);
    CHECK(s.unmatched.size() == 2);
    CHECK(hasMessage(s, ToolpathIssue::BankHoleUnmatched, "they are not drilled"));
    CHECK_FALSE(hasMessage(s, ToolpathIssue::BankHoleUnmatched, "nothing is cut"));

    // Nothing matches and the unmatched holes are skipped: an empty toolpath with the reasons.
    const DrillBankResult none = run({hole(0, 0), hole(100, 100)}, p);
    CHECK_FALSE(none.ok());
    CHECK(none.unmatched.size() == 2);
    CHECK(none.has(ToolpathIssue::BankHoleUnmatched));
}

TEST_CASE("drill bank: the per-hole reports are capped", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    std::vector<Contour> cs;
    for (int i = 0; i < 150; ++i) {
        cs.push_back(hole(1000.0 * i, 0));
    }
    const DrillBankResult r = run(cs, p);
    CHECK_FALSE(r.ok());
    CHECK(r.unmatched.size() == 150);
    CHECK(count(r, ToolpathIssue::BankHoleUnmatched) == 101);  // 100 holes and the summary
    CHECK(hasMessage(r, ToolpathIssue::BankHoleUnmatched, "150 hole(s) fit no stroke"));
    // Isolated holes cost one look at every spindle each and are never looked at again.
    CHECK(r.stats.placementEvaluations == 150 * 4);
    CHECK(r.stats.spindleQueries == 150 * 4 * 4);
}

TEST_CASE("drill bank: the move cap counts the dwell moves", "[toolpath][drill_bank]") {
    // 10000 passes per stroke, 800 strokes: 800 x (2 + 10000 x 3) moves with a dwell is over the 20 million cap; it is
    // refused before any move is made (without a dwell it would be 16 million and pass).
    DrillBankParams p = bank(1, 0);
    p.minHoles = 1;
    p.cutDepth = 5.0;
    p.peck = true;
    p.dwellTime = 0.5;
    std::vector<Contour> cs;
    for (int i = 0; i < 800; ++i) {
        cs.push_back(hole(10.0 * i, 0));
    }
    const DrillBankResult r = run(cs, p, drillTool(3.0, 0.0005));
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    CHECK(hasMessage(r, ToolpathIssue::WorkLimitExceeded, "too many strokes or passes"));
    CHECK(r.toolpath.moves.empty());
}

TEST_CASE("drill bank: the pass cap is exactly 10000 passes per hole", "[toolpath][drill_bank]") {
    // One hole, one spindle, pass depth 1 mm: a cut depth of N mm makes N passes. 10000 passes are allowed, 10001
    // are refused (with the work limit message, before any move is made).
    DrillBankParams p = bank(1, 0);
    p.minHoles = 1;
    p.peck = true;
    const std::vector<Contour> cs{hole(0, 0)};
    SECTION("exactly the cap") {
        p.cutDepth = 10000.0;
        const DrillBankResult r = run(cs, p, drillTool(3.0, 1.0));
        REQUIRE(r.ok());
        std::size_t plunges = 0;
        for (const Move& m : r.toolpath.moves) {
            plunges += m.kind == MoveKind::Plunge ? 1 : 0;
        }
        CHECK(plunges == 10000);
    }
    SECTION("one pass over the cap") {
        p.cutDepth = 10001.0;
        const DrillBankResult r = run(cs, p, drillTool(3.0, 1.0));
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
        CHECK(hasMessage(r, ToolpathIssue::WorkLimitExceeded, "more than 10000 passes per hole"));
        CHECK(r.toolpath.moves.empty());
    }
}

TEST_CASE("drill bank: holes come from the vectors as in Drilling", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    std::vector<Contour> cs = row(0, 0, 4);
    Contour open;
    open.append(geometry::LineSpan{{0, 50}, {10, 50}});
    cs.push_back(open);
    cs.push_back(hole(32, 0));  // the same centre as vector 1
    cs.push_back(Contour{});
    const DrillBankResult r = run(cs, p);
    REQUIRE(r.ok());
    CHECK(r.placements[0].holes.size() == 4);
    CHECK(r.has(ToolpathIssue::OpenContourIgnored));
    CHECK(r.has(ToolpathIssue::DuplicateHole));
    CHECK(r.has(ToolpathIssue::EmptyContour));
    // A marker circle smaller than the drills is no reason to warn: the spindles' drills differ.
    const DrillBankResult tiny = run(row(0, 0, 4, 32.0, 0.1), p);
    CHECK(tiny.ok());
    CHECK_FALSE(tiny.has(ToolpathIssue::HoleWiderThanVector));
    // No vectors, or none usable.
    const DrillBankResult empty = run({}, p);
    CHECK_FALSE(empty.ok());
    CHECK(empty.has(ToolpathIssue::NoRegion));
    const DrillBankResult onlyOpen = run({open}, p);
    CHECK_FALSE(onlyOpen.ok());
    CHECK(onlyOpen.has(ToolpathIssue::OpenContourIgnored));
}

TEST_CASE("drill bank: tool and Drilling parameters are checked as in Drilling", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    tooling::Tool vbit = drillTool();
    vbit.type = tooling::ToolType::VBit;
    const DrillBankResult v = run(row(0, 0, 4), p, vbit);
    CHECK_FALSE(v.ok());
    CHECK(v.has(ToolpathIssue::InvalidTool));
    p.cutDepth = 0.0;
    CHECK(run(row(0, 0, 4), p).has(ToolpathIssue::InvalidParameters));
    p = bank();
    p.safeZ = -1.0;
    CHECK(run(row(0, 0, 4), p).has(ToolpathIssue::InvalidParameters));
    p = bank();
    p.peck = true;
    p.retractGap = 0.0;
    CHECK(run(row(0, 0, 4), p).has(ToolpathIssue::InvalidParameters));
    // Peck with a too small pass depth for the depth: refused, nothing cut.
    p = bank();
    p.peck = true;
    p.cutDepth = 100.0;
    const DrillBankResult many = run(row(0, 0, 4), p, drillTool(3.0, 0.001));
    CHECK_FALSE(many.ok());
    CHECK(many.has(ToolpathIssue::WorkLimitExceeded));
    // The per-spindle depth counts for the pass limit as well.
    p = bank();
    p.peck = true;
    p.cutDepth = 1.0;
    p.spindleDepths = {0, 0, 0, 50000.0};
    CHECK(run(row(0, 0, 4), p, drillTool(3.0, 0.001)).has(ToolpathIssue::WorkLimitExceeded));
}

TEST_CASE("drill bank: work counters and limits", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    // A 5 x 8 grid of holes: 5 rows of 8 at the pitch.
    std::vector<Contour> cs;
    for (int r = 0; r < 5; ++r) {
        append(cs, row(0, 100.0 * r, 8));
    }
    const DrillBankResult r = run(cs, p);
    REQUIRE(r.ok());
    CHECK(drilledHoles(r) == 40);
    CHECK(r.placements.size() == 10);
    CHECK(r.stats.spindleQueries == r.stats.placementEvaluations * 4);
    // Every (hole, spindle) pair is evaluated once to start with; the lazy greedy needs few more.
    CHECK(r.stats.placementEvaluations >= 40 * 4);
    CHECK(r.stats.placementEvaluations <= 40 * 4 * 3);

    // The limit counts spindle queries and stops the run within one evaluation of it.
    ToolpathRunControl control;
    control.workLimit = 200;
    const DrillBankResult limited = drillBankToolpath(cs, drillTool(), std::nullopt, p, control);
    CHECK_FALSE(limited.ok());
    CHECK(limited.has(ToolpathIssue::WorkLimitExceeded));
    // 40 holes x 4 spindles x 4 spindles = 640 > 200 is refused before any work.
    CHECK(limited.stats.spindleQueries == 0);
    control.workLimit = 640;
    const DrillBankResult exact = drillBankToolpath(cs, drillTool(), std::nullopt, p, control);
    // The estimate fits, but the greedy pass needs more queries than that: stopped after at most one evaluation.
    CHECK_FALSE(exact.ok());
    CHECK(exact.has(ToolpathIssue::WorkLimitExceeded));
    CHECK(exact.stats.spindleQueries <= 640 + 4);
    CHECK(exact.stats.spindleQueries > 640);
    control.workLimit = 100000;
    CHECK(drillBankToolpath(cs, drillTool(), std::nullopt, p, control).ok());
}

TEST_CASE("drill bank: cancelled before any work, deterministic output", "[toolpath][drill_bank]") {
    DrillBankParams p = bank();
    std::atomic<bool> flag{true};
    ToolpathRunControl control;
    control.cancel = &flag;
    const DrillBankResult c = drillBankToolpath(row(0, 0, 4), drillTool(), std::nullopt, p, control);
    CHECK_FALSE(c.ok());
    CHECK(c.has(ToolpathIssue::Cancelled));
    CHECK(c.placements.empty());
    flag = false;
    const DrillBankResult a = drillBankToolpath(row(0, 0, 4), drillTool(), std::nullopt, p, control);
    const DrillBankResult b = drillBankToolpath(row(0, 0, 4), drillTool(), std::nullopt, p);
    REQUIRE(a.ok());
    CHECK(a.toolpath == b.toolpath);
    CHECK(a.placements == b.placements);
    CHECK(a.diagnostics == b.diagnostics);
}
