// T-0323: Laser Cut / Fill toolpath (laser.hpp, ADR-0099): kerf side of the cut strategies, hatch spacing,
// angle, cross hatch, fill order, holes, overscan moves, pass and repeat counts, beam power on the moves,
// tools, limits and hostile input.

#include <algorithm>
#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/laser.hpp"
#include "carvetto/toolpath/move.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

tooling::Tool beam(double diameter = 0.2, tooling::ToolType type = tooling::ToolType::EndMill) {
    tooling::Tool t;
    t.id = "laser";
    t.name = "Laser";
    t.type = type;
    t.diameter = diameter;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 900.0;
    t.defaultCuttingData.plungeRate = 900.0;
    t.defaultCuttingData.spindleSpeed = 0.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.maxBurnRate = 2000.0;
    return t;
}

Contour square(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2({x0, y0}, {x1, y1}));
}

Contour line(Vec2 a, Vec2 b) {
    Contour c;
    c.append(LineSpan{a, b});
    return c;
}

LaserCutFillParams hatchParams() {
    LaserCutFillParams p;
    p.strategy = LaserStrategy::HatchFill;
    p.stepover = 1.0;
    p.power = 40.0;
    return p;
}

LaserCutFillResult run(const std::vector<Contour>& cs, const LaserCutFillParams& p, double dia = 0.2) {
    return laserCutFillToolpath(cs, beam(dia), std::nullopt, p);
}

/// Cutting moves (beam on) of the result.
std::vector<Move> beamMoves(const LaserCutFillResult& r) {
    std::vector<Move> out;
    for (const Move& m : r.toolpath.moves) {
        if (m.jetPower > 0.0) {
            out.push_back(m);
        }
    }
    return out;
}

Box2 boundsOf(const std::vector<Move>& ms) {
    Box2 b;
    for (const Move& m : ms) {
        b.expand({m.to.x, m.to.y});
    }
    return b;
}

}  // namespace

TEST_CASE("T-0323: pass levels follow cut depth and pass depth", "[toolpath][laser]") {
    CHECK(laserPassLevels(0.0, 1.0) == std::vector<double>{0.0});
    CHECK(laserPassLevels(3.0, 1.0) == std::vector<double>{-1.0, -2.0, -3.0});
    CHECK(laserPassLevels(2.5, 1.0) == std::vector<double>{-1.0, -2.0, -2.5});
    CHECK(laserPassLevels(3.0, 0.0) == std::vector<double>{-3.0});
    CHECK(laserPassLevels(3.0, 5.0) == std::vector<double>{-3.0});
}

TEST_CASE("T-0323: cut on follows the vector with the beam on and rises at the end", "[toolpath][laser]") {
    LaserCutFillParams p;
    p.strategy = LaserStrategy::CutOn;
    p.power = 65.0;
    p.moveSpeed = 750.0;
    const LaserCutFillResult r = run({line({0, 0}, {10, 0})}, p);
    REQUIRE(r.ok());
    CHECK(r.passes == 1);
    CHECK(r.strokes == 1);
    CHECK(r.toolpath.tool.laser);
    CHECK(r.toolpath.tool.spindleSpeed == 0.0);
    CHECK(r.toolpath.tool.feedRate == 750.0);
    CHECK(r.toolpath.tool.plungeRate == 750.0);
    REQUIRE(r.toolpath.moves.size() == 3);
    CHECK(r.toolpath.moves[0].kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves[0].to.x == Approx(0.0));
    CHECK(r.toolpath.moves[0].to.z == 0.0);
    CHECK(r.toolpath.moves[1].kind == MoveKind::Feed);
    CHECK(r.toolpath.moves[1].to.x == Approx(10.0));
    CHECK(r.toolpath.moves[1].jetPower == 65.0);
    CHECK(r.toolpath.moves[2].kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves[2].to.z == p.safeZ);
    CHECK(r.toolpath.moves[2].jetPower == 0.0);
}

TEST_CASE("T-0323: the move speed defaults to the tool's feed rate", "[toolpath][laser]") {
    LaserCutFillParams p;
    p.strategy = LaserStrategy::CutOn;
    const LaserCutFillResult r = run({line({0, 0}, {10, 0})}, p);
    REQUIRE(r.ok());
    CHECK(r.toolpath.tool.feedRate == 900.0);
}

TEST_CASE("T-0323: cut outside and inside are kerf compensated on closed vectors", "[toolpath][laser]") {
    const std::vector<Contour> sq{square(0, 0, 20, 20)};
    LaserCutFillParams p;
    p.strategy = LaserStrategy::CutOutside;
    LaserCutFillResult out = run(sq, p, 0.4);
    REQUIRE(out.ok());
    Box2 b = boundsOf(beamMoves(out));
    CHECK(b.min.x == Approx(-0.2).margin(1e-6));
    CHECK(b.max.y == Approx(20.2).margin(1e-6));
    p.strategy = LaserStrategy::CutInside;
    LaserCutFillResult in = run(sq, p, 0.4);
    REQUIRE(in.ok());
    b = boundsOf(beamMoves(in));
    CHECK(b.min.x == Approx(0.2).margin(1e-6));
    CHECK(b.max.x == Approx(19.8).margin(1e-6));
    p.strategy = LaserStrategy::CutOn;
    LaserCutFillResult on = run(sq, p, 0.4);
    REQUIRE(on.ok());
    b = boundsOf(beamMoves(on));
    CHECK(b.min.x == Approx(0.0).margin(1e-6));
    CHECK(b.max.x == Approx(20.0).margin(1e-6));
}

TEST_CASE("T-0323: the allowance moves the cut away from the vector", "[toolpath][laser]") {
    LaserCutFillParams p;
    p.strategy = LaserStrategy::CutOutside;
    p.allowance = 0.5;
    const LaserCutFillResult out = run({square(0, 0, 20, 20)}, p, 0.4);
    REQUIRE(out.ok());
    const Box2 b = boundsOf(beamMoves(out));
    CHECK(b.min.x == Approx(-0.7).margin(1e-6));
}

TEST_CASE("T-0323: a hole is cut on the side that keeps the region", "[toolpath][laser]") {
    // Outside keeps the part: inside the hole the beam runs inside the hole outline.
    LaserCutFillParams p;
    p.strategy = LaserStrategy::CutOutside;
    const LaserCutFillResult r = run({square(0, 0, 20, 20), square(8, 8, 12, 12)}, p, 0.4);
    REQUIRE(r.ok());
    CHECK(r.strokes == 2);
    // The hole is cut first (inner cuts first); its cut lies within the hole outline.
    const std::vector<Move> ms = beamMoves(r);
    const Move& first = ms.front();
    CHECK(first.to.x > 8.0);
    CHECK(first.to.x < 12.0);
}

TEST_CASE("T-0323: an open vector is cut on its right (outside) or left (inside)", "[toolpath][laser]") {
    LaserCutFillParams p;
    p.strategy = LaserStrategy::CutOutside;
    LaserCutFillResult r = run({line({0, 0}, {10, 0})}, p, 0.4);
    REQUIRE(r.ok());
    CHECK(beamMoves(r).front().to.y == Approx(-0.2).margin(1e-6));
    p.strategy = LaserStrategy::CutInside;
    r = run({line({0, 0}, {10, 0})}, p, 0.4);
    REQUIRE(r.ok());
    CHECK(beamMoves(r).front().to.y == Approx(0.2).margin(1e-6));
}

TEST_CASE("T-0323: hatch lines are spaced by the stepover, aligned to the origin and serpentine", "[toolpath][laser]") {
    const LaserCutFillResult r = run({square(0, 0, 10, 10)}, hatchParams());
    REQUIRE(r.ok());
    // Region inset by the beam radius 0.1: lines at y = 1..9.
    CHECK(r.strokes == 9);
    const std::vector<Move> ms = beamMoves(r);
    REQUIRE(ms.size() == 9);
    for (std::size_t i = 0; i < ms.size(); ++i) {
        CHECK(ms[i].to.y == Approx(static_cast<double>(i + 1)).margin(1e-9));
        CHECK(ms[i].jetPower == 40.0);
        // Alternating direction: odd lines end on the right, even on the left.
        CHECK(ms[i].to.x == Approx(i % 2 == 0 ? 9.9 : 0.1).margin(1e-6));
    }
}

TEST_CASE("T-0323: hatch angle turns the lines", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.hatchAngle = 1.5707963267948966;
    const LaserCutFillResult r = run({square(0, 0, 10, 10)}, p);
    REQUIRE(r.ok());
    const std::vector<Move> ms = beamMoves(r);
    REQUIRE(!ms.empty());
    // Vertical lines: the beam moves in Y only.
    for (std::size_t i = 0; i < ms.size(); ++i) {
        const Move& rap = r.toolpath.moves[2 * i];
        REQUIRE(rap.kind == MoveKind::Rapid);
        CHECK(rap.to.x == Approx(ms[i].to.x).margin(1e-6));
    }
    CHECK(ms.size() == 9);
    // 45 degrees: lines x - y = const, none longer than the diagonal.
    p.hatchAngle = 0.7853981633974483;
    const LaserCutFillResult d = run({square(0, 0, 10, 10)}, p);
    REQUIRE(d.ok());
    for (std::size_t i = 0; i + 1 < d.toolpath.moves.size(); i += 2) {
        const Move& a = d.toolpath.moves[i];
        const Move& b = d.toolpath.moves[i + 1];
        CHECK(std::fabs((b.to.x - a.to.x) - (b.to.y - a.to.y)) < 1e-6);
    }
}

TEST_CASE("T-0323: cross hatch adds a second set at right angles", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.crossHatch = true;
    const LaserCutFillResult r = run({square(0, 0, 10, 10)}, p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 18);
    const std::vector<Move> ms = beamMoves(r);
    // First nine horizontal, then nine vertical.
    for (std::size_t i = 0; i < 9; ++i) {
        CHECK(r.toolpath.moves[2 * i].to.y == Approx(r.toolpath.moves[2 * i + 1].to.y).margin(1e-9));
    }
    for (std::size_t i = 9; i < 18; ++i) {
        CHECK(r.toolpath.moves[2 * i].to.x == Approx(r.toolpath.moves[2 * i + 1].to.x).margin(1e-9));
    }
}

TEST_CASE("T-0323: fill together runs a line across every region; regionally fills one region at a time",
          "[toolpath][laser]") {
    const std::vector<Contour> two{square(0, 0, 10, 10), square(20, 0, 30, 10)};
    LaserCutFillParams p = hatchParams();
    p.fillMode = LaserFillMode::Together;
    const LaserCutFillResult t = run(two, p);
    REQUIRE(t.ok());
    CHECK(t.strokes == 18);
    // Together: the second stroke is in the other region (same line y = 1).
    CHECK(t.toolpath.moves[0].to.y == Approx(t.toolpath.moves[2].to.y).margin(1e-9));
    CHECK(std::fabs(t.toolpath.moves[0].to.x - t.toolpath.moves[2].to.x) > 10.0);
    p.fillMode = LaserFillMode::Regionally;
    const LaserCutFillResult g = run(two, p);
    REQUIRE(g.ok());
    CHECK(g.strokes == 18);
    // Regionally: the first nine strokes all lie in one region.
    const bool left = g.toolpath.moves[0].to.x < 15.0;
    for (std::size_t i = 0; i < 9; ++i) {
        CHECK((g.toolpath.moves[2 * i].to.x < 15.0) == left);
    }
    CHECK((g.toolpath.moves[2 * 9].to.x < 15.0) != left);
}

TEST_CASE("T-0323: hatch lines skip holes", "[toolpath][laser]") {
    const LaserCutFillResult r = run({square(0, 0, 10, 10), square(3, 3, 7, 7)}, hatchParams());
    REQUIRE(r.ok());
    // Lines y = 1, 2 and 8, 9 cross once; y = 3..7 cross the region twice (the hole is inset outwards by 0.1).
    // y = 3 and 7 touch the grown hole edge at 2.9 / 7.1 only: lines 3..7 are all split.
    CHECK(r.strokes == 4 + 5 * 2);
    for (const Move& m : beamMoves(r)) {
        const bool inHole = m.to.x > 2.95 && m.to.x < 7.05 && m.to.y > 2.95 && m.to.y < 7.05;
        CHECK_FALSE(inHole);
    }
}

TEST_CASE("T-0323: overscan runs the beam off before and after each hatch line", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.overscan = 2.0;
    const LaserCutFillResult r = run({square(0, 0, 10, 10)}, p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 9);
    // Per stroke: rapid to the overscan point, off-feed to the start, the cut, off-feed past the end.
    const std::vector<Move>& m = r.toolpath.moves;
    REQUIRE(m.size() == 9 * 4 + 1);
    CHECK(m[0].kind == MoveKind::Rapid);
    CHECK(m[0].to.x == Approx(0.1 - 2.0).margin(1e-6));
    CHECK(m[1].kind == MoveKind::Feed);
    CHECK(m[1].jetPower == 0.0);
    CHECK(m[1].to.x == Approx(0.1).margin(1e-6));
    CHECK(m[2].jetPower == 40.0);
    CHECK(m[2].to.x == Approx(9.9).margin(1e-6));
    CHECK(m[3].jetPower == 0.0);
    CHECK(m[3].to.x == Approx(11.9).margin(1e-6));
    // The next line starts from the right: its overscan point lies right of the region.
    CHECK(m[4].to.x == Approx(11.9).margin(1e-6));
    CHECK(m[5].to.x == Approx(9.9).margin(1e-6));
    CHECK(m[7].to.x == Approx(-1.9).margin(1e-6));
    // The cut strategies ignore overscan.
    LaserCutFillParams c;
    c.strategy = LaserStrategy::CutOn;
    c.overscan = 2.0;
    CHECK(run({line({0, 0}, {10, 0})}, c).toolpath.moves.size() == 3);
}

TEST_CASE("T-0323: a hatch allowance insets the filled region", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.allowance = 1.4;
    const LaserCutFillResult r = run({square(0, 0, 10, 10)}, p);
    REQUIRE(r.ok());
    // Inset by beam radius 0.1 plus allowance 1.4 = 1.5: region 1.5..8.5, lines at y = 2..8.
    CHECK(r.strokes == 7);
    const std::vector<Move> ms = beamMoves(r);
    REQUIRE(ms.size() == 7);
    for (std::size_t i = 0; i < ms.size(); ++i) {
        CHECK(ms[i].to.y == Approx(static_cast<double>(i + 2)).margin(1e-9));
        CHECK(ms[i].to.x == Approx(i % 2 == 0 ? 8.5 : 1.5).margin(1e-6));
    }
    const Box2 b = boundsOf(ms);
    CHECK(b.min.x == Approx(1.5).margin(1e-6));
    CHECK(b.max.x == Approx(8.5).margin(1e-6));
}

TEST_CASE("T-0323: a small overscan still adds the overscan moves", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.overscan = 0.2;
    const LaserCutFillResult r = run({square(0, 0, 10, 10)}, p);
    REQUIRE(r.ok());
    const std::vector<Move>& m = r.toolpath.moves;
    REQUIRE(m.size() == 9 * 4 + 1);
    CHECK(m[0].kind == MoveKind::Rapid);
    CHECK(m[0].to.x == Approx(0.1 - 0.2).margin(1e-6));
    CHECK(m[1].jetPower == 0.0);
    CHECK(m[1].to.x == Approx(0.1).margin(1e-6));
    CHECK(m[2].jetPower == 40.0);
    CHECK(m[2].to.x == Approx(9.9).margin(1e-6));
    CHECK(m[3].jetPower == 0.0);
    CHECK(m[3].to.x == Approx(10.1).margin(1e-6));
}

TEST_CASE("T-0323: cut depth sets the number of passes and repeats rerun each pass", "[toolpath][laser]") {
    LaserCutFillParams p;
    p.strategy = LaserStrategy::CutOn;
    p.cutDepth = 3.0;
    p.passDepth = 1.0;
    LaserCutFillResult r = run({line({0, 0}, {10, 0})}, p);
    REQUIRE(r.ok());
    CHECK(r.passes == 3);
    std::vector<double> zs;
    for (const Move& m : r.toolpath.moves) {
        if (m.jetPower > 0.0) {
            zs.push_back(m.to.z);
        }
    }
    CHECK(zs == std::vector<double>{-1.0, -2.0, -3.0});
    p.repeats = 2;
    r = run({line({0, 0}, {10, 0})}, p);
    zs.clear();
    for (const Move& m : r.toolpath.moves) {
        if (m.jetPower > 0.0) {
            zs.push_back(m.to.z);
        }
    }
    CHECK(zs == std::vector<double>{-1.0, -1.0, -2.0, -2.0, -3.0, -3.0});
    // The pass depth defaults to the tool's cutting data (1 mm here).
    p.passDepth = 0.0;
    p.repeats = 1;
    CHECK(run({line({0, 0}, {10, 0})}, p).passes == 3);
}

TEST_CASE("T-0323: hatch outline cuts the inset region boundary after the fill", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.outline = true;
    const LaserCutFillResult r = run({square(0, 0, 10, 10)}, p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 10);
    // The last cut is the outline: a closed rectangle inset by the beam radius.
    const std::vector<Move> ms = beamMoves(r);
    const std::vector<Move> outline(ms.end() - 4, ms.end());
    const Box2 b = boundsOf(outline);
    CHECK(b.min.x == Approx(0.1).margin(1e-6));
    CHECK(b.max.y == Approx(9.9).margin(1e-6));
    CHECK(outline.back().to.x == Approx(outline.front().to.x - 0.0).margin(20.0));  // closed: returns to its start
}

TEST_CASE("T-0323: hatch fill ignores open vectors with a diagnostic", "[toolpath][laser]") {
    const LaserCutFillResult r = run({square(0, 0, 10, 10), line({0, 0}, {5, 5})}, hatchParams());
    REQUIRE(r.ok());
    CHECK(r.has(ToolpathIssue::OpenContourIgnored));
    const LaserCutFillResult none = run({line({0, 0}, {5, 5})}, hatchParams());
    CHECK_FALSE(none.ok());
    CHECK(none.has(ToolpathIssue::OpenContourIgnored));
}

TEST_CASE("T-0323: bad parameters and tools are refused", "[toolpath][laser]") {
    const std::vector<Contour> sq{square(0, 0, 10, 10)};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto refused = [&](auto&& edit, ToolpathIssue why = ToolpathIssue::InvalidParameters) {
        LaserCutFillParams p = hatchParams();
        edit(p);
        const LaserCutFillResult r = run(sq, p);
        CHECK_FALSE(r.ok());
        CHECK(r.has(why));
    };
    refused([](LaserCutFillParams& p) { p.power = 0.0; });
    refused([](LaserCutFillParams& p) { p.power = 100.5; });
    refused([&](LaserCutFillParams& p) { p.power = nan; });
    refused([](LaserCutFillParams& p) { p.stepover = 0.0; });
    refused([](LaserCutFillParams& p) { p.repeats = 0; });
    refused([](LaserCutFillParams& p) { p.overscan = -1.0; });
    refused([](LaserCutFillParams& p) { p.cutDepth = -1.0; });
    refused([&](LaserCutFillParams& p) { p.hatchAngle = nan; });
    refused([](LaserCutFillParams& p) { p.safeZ = 0.0; });
    const LaserCutFillResult v = laserCutFillToolpath(sq, beam(0.2, tooling::ToolType::VBit), std::nullopt, hatchParams());
    CHECK_FALSE(v.ok());
    CHECK(v.has(ToolpathIssue::InvalidTool));
    tooling::Tool noFeed = beam();
    noFeed.defaultCuttingData.feedRate = 0.0;
    CHECK(laserCutFillToolpath(sq, noFeed, std::nullopt, hatchParams()).has(ToolpathIssue::InvalidTool));
    LaserCutFillParams withSpeed = hatchParams();
    withSpeed.moveSpeed = 500.0;
    CHECK(laserCutFillToolpath(sq, noFeed, std::nullopt, withSpeed).ok());
}

TEST_CASE("T-0323: work limits count moves and lines, not time", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.stepover = 0.01;
    ToolpathRunControl control;
    control.moveLimit = 100;
    const LaserCutFillResult r = laserCutFillToolpath({square(0, 0, 10, 10)}, beam(), std::nullopt, p, control);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    // More than the line limit across a huge region.
    p.stepover = 1e-4;
    const LaserCutFillResult huge = run({square(0, 0, 1000, 1000)}, p);
    CHECK_FALSE(huge.ok());
    CHECK(huge.has(ToolpathIssue::WorkLimitExceeded));
    // Too many passes.
    LaserCutFillParams q;
    q.strategy = LaserStrategy::CutOn;
    q.cutDepth = 5000.0;
    q.passDepth = 1.0;
    CHECK(run({line({0, 0}, {1, 0})}, q).has(ToolpathIssue::WorkLimitExceeded));
}

TEST_CASE("T-0323: a cancelled hatch gives nothing", "[toolpath][laser]") {
    std::atomic<bool> stop{true};
    ToolpathRunControl control;
    control.cancel = &stop;
    LaserCutFillParams p = hatchParams();
    p.stepover = 0.05;
    const LaserCutFillResult r = laserCutFillToolpath({square(0, 0, 200, 200)}, beam(), std::nullopt, p, control);
    CHECK_FALSE(r.ok());
}

TEST_CASE("T-0323: the same input gives the same toolpath", "[toolpath][laser]") {
    LaserCutFillParams p = hatchParams();
    p.crossHatch = true;
    p.hatchAngle = 0.3;
    p.outline = true;
    p.overscan = 1.0;
    const std::vector<Contour> in{square(0, 0, 30, 20), square(5, 5, 10, 10), square(40, 0, 50, 10)};
    const LaserCutFillResult a = run(in, p);
    const LaserCutFillResult b = run(in, p);
    REQUIRE(a.ok());
    CHECK(a.toolpath == b.toolpath);
}
