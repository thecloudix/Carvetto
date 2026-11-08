// T-0222: tool shapes (flat, ball, V, engraving, tapered ball) and the stock/heightfield container.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <string>

#include "carvetto/simulate/stock.hpp"
#include "carvetto/simulate/tool_shape.hpp"
#include "carvetto/tooling/tool.hpp"
#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;

namespace {

/// Half width of the cut a shape leaves at `depth`: the largest r with h(r) <= depth.
double halfWidthAt(const ToolShape& s, double depth) {
    double lo = 0.0;
    double hi = s.radius();
    if (s.heightAt(hi) <= depth) {
        return hi;
    }
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        (s.heightAt(mid) <= depth ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

tooling::Tool toolOf(tooling::ToolType type, double d, double angleDeg = 0, double tip = 0) {
    tooling::Tool t;
    t.id = "t";
    t.name = "t";
    t.type = type;
    t.diameter = d;
    t.includedAngle = angleDeg * kDeg;
    t.tipDiameter = tip;
    return t;
}

}  // namespace

TEST_CASE("simulate: tool shape profiles match the independent height functions", "[simulate][tool]") {
    for (const NamedTool& t : allTools()) {
        INFO(t.name);
        REQUIRE(t.shape.valid());
        CHECK(std::fabs(t.shape.radius() - t.profile.radius) < 1e-15);
        for (int i = 0; i <= 200; ++i) {
            const double r = t.profile.radius * i / 200.0;
            CHECK(std::fabs(t.shape.heightAt(r) - t.profile.height(r)) < 1e-9);
        }
        CHECK(std::isinf(t.shape.heightAt(t.profile.radius * 1.0001)));
        // convex and non-decreasing: the exact sweep formulas depend on it
        double prev = t.shape.heightAt(0.0);
        double prevSlope = -1.0;
        for (int i = 1; i <= 400; ++i) {
            const double r0 = t.profile.radius * (i - 1) / 400.0;
            const double r1 = t.profile.radius * i / 400.0;
            const double h = t.shape.heightAt(r1);
            CHECK(h >= prev - 1e-12);
            const double slope = (h - prev) / (r1 - r0);
            CHECK(slope >= prevSlope - 1e-6);
            prev = h;
            prevSlope = slope;
        }
    }
}

TEST_CASE("simulate: tool shapes agree with the tool database cutting widths", "[simulate][tool]") {
    using tooling::ToolType;
    struct Case {
        tooling::Tool tool;
    };
    const Case cases[] = {{toolOf(ToolType::EndMill, 6)},
                          {toolOf(ToolType::BallNose, 6)},
                          {toolOf(ToolType::VBit, 12, 60)},
                          {toolOf(ToolType::VBit, 12, 90)},
                          {toolOf(ToolType::Engraving, 6, 40, 0.5)},
                          {toolOf(ToolType::TaperedBallNose, 8, 20, 3)},
                          {toolOf(ToolType::TaperedBallNose, 10, 45, 4)},
                          {toolOf(ToolType::Drill, 6, 118)},
                          {toolOf(ToolType::DiamondDrag, 4)}};
    for (const Case& c : cases) {
        INFO(std::string(tooling::toString(c.tool.type)));
        const auto shape = ToolShape::fromTool(c.tool);
        REQUIRE(shape.has_value());
        for (double depth : {0.01, 0.1, 0.5, 1.0, 2.5, 4.0, 9.0}) {
            const double want = tooling::cuttingWidthAtDepth(c.tool, depth) / 2.0;
            CHECK(std::fabs(halfWidthAt(*shape, depth) - want) < 1e-6);
        }
    }
}

TEST_CASE("simulate: fromTool covers the supported types and refuses the others", "[simulate][tool]") {
    using tooling::ToolType;
    CHECK(ToolShape::fromTool(toolOf(ToolType::EndMill, 6))->kind() == simulate::ShapeKind::FlatEndMill);
    CHECK(ToolShape::fromTool(toolOf(ToolType::BallNose, 6))->kind() == simulate::ShapeKind::BallNose);
    CHECK(ToolShape::fromTool(toolOf(ToolType::VBit, 6, 60))->kind() == simulate::ShapeKind::VBit);
    CHECK(ToolShape::fromTool(toolOf(ToolType::TaperedBallNose, 6, 30, 2))->kind() ==
          simulate::ShapeKind::TaperedBallNose);
    tooling::Tool radiused = toolOf(ToolType::RadiusedEndMill, 6);
    radiused.cornerRadius = 1;
    CHECK_FALSE(ToolShape::fromTool(radiused).has_value());
    CHECK_FALSE(ToolShape::fromTool(toolOf(ToolType::FormTool, 6)).has_value());
    CHECK_FALSE(ToolShape::fromTool(toolOf(ToolType::VBit, 6, 0)).has_value());      // no angle: fails validate
    CHECK_FALSE(ToolShape::fromTool(toolOf(ToolType::EndMill, -2)).has_value());
    const auto withFlute = ToolShape::fromTool(toolOf(ToolType::EndMill, 6), 12.0);
    REQUIRE(withFlute.has_value());
    CHECK(withFlute->fluteLength() == 12.0);
    CHECK_FALSE(ToolShape::fromTool(toolOf(ToolType::EndMill, 6), 0.0).has_value());
    CHECK_FALSE(ToolShape::fromTool(toolOf(ToolType::EndMill, 6), std::nan("")).has_value());
    CHECK(std::isinf(ToolShape::fromTool(toolOf(ToolType::EndMill, 6))->fluteLength()));
}

TEST_CASE("simulate: factories refuse unusable dimensions", "[simulate][tool]") {
    const double quietNaN = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_FALSE(ToolShape::flatEndMill(0).has_value());
    CHECK_FALSE(ToolShape::flatEndMill(-1).has_value());
    CHECK_FALSE(ToolShape::flatEndMill(quietNaN).has_value());
    CHECK_FALSE(ToolShape::flatEndMill(inf).has_value());
    CHECK_FALSE(ToolShape::flatEndMill(1e9).has_value());
    CHECK_FALSE(ToolShape::ballNose(0).has_value());
    CHECK_FALSE(ToolShape::vBit(6, 0).has_value());
    CHECK_FALSE(ToolShape::vBit(6, kPi).has_value());
    CHECK_FALSE(ToolShape::vBit(6, quietNaN).has_value());
    CHECK_FALSE(ToolShape::vBit(6, 1.0, 6.0).has_value());   // tip as wide as the tool
    CHECK_FALSE(ToolShape::vBit(6, 1.0, -1.0).has_value());
    CHECK_FALSE(ToolShape::taperedBallNose(6, 0.5, 7.0).has_value());  // ball wider than the tool
    CHECK_FALSE(ToolShape::taperedBallNose(6, 0.5, 0.0).has_value());
    CHECK_FALSE(ToolShape().valid());
    CHECK(ToolShape::vBit(6, 1.0, 0.0).has_value());
}

TEST_CASE("simulate: the shank is a flat cylinder at the flute length", "[simulate][tool]") {
    const ToolShape t = ToolShape::ballNose(4)->withFluteLength(7.5);
    const ToolShape shank = t.shank();
    CHECK(shank.radius() == t.radius());
    CHECK(shank.heightAt(0.0) == 7.5);
    CHECK(shank.heightAt(2.0) == 7.5);
    CHECK(std::isinf(shank.heightAt(2.1)));
}

TEST_CASE("simulate: stock checks: dimensions, cell counts and the cell cap", "[simulate][stock]") {
    using simulate::StockIssue;
    simulate::Stock s;
    s.width = 100;
    s.height = 50;
    s.cellSize = 0.5;
    simulate::StockCheck c = simulate::checkStock(s);
    REQUIRE(c.ok());
    CHECK(c.nx == 200);
    CHECK(c.ny == 100);
    CHECK(c.cells() == 20000);
    s.width = 100.1;  // partial cell rounds up
    CHECK(simulate::checkStock(s).nx == 201);

    simulate::Stock bad = s;
    bad.width = 0;
    CHECK(simulate::checkStock(bad).issue == StockIssue::NotPositive);
    bad = s;
    bad.cellSize = -1;
    CHECK(simulate::checkStock(bad).issue == StockIssue::NotPositive);
    bad = s;
    bad.thickness = std::nan("");
    CHECK(simulate::checkStock(bad).issue == StockIssue::NotFinite);
    bad = s;
    bad.origin.x = std::numeric_limits<double>::infinity();
    CHECK(simulate::checkStock(bad).issue == StockIssue::NotFinite);
    bad = s;
    bad.width = 1e300;
    CHECK(simulate::checkStock(bad).issue == StockIssue::NotFinite);
    bad = s;
    bad.cellSize = 1e-300;
    CHECK(simulate::checkStock(bad).issue == StockIssue::TooManyCells);
    bad = s;
    bad.cellSize = 0.01;
    bad.maxCells = 1'000'000;
    c = simulate::checkStock(bad);
    CHECK(c.issue == StockIssue::TooManyCells);
    CHECK(c.message.find("cells") != std::string::npos);
    bad.maxCells = std::numeric_limits<std::size_t>::max();  // never above the absolute limit
    bad.width = 1e5;
    bad.height = 1e5;
    bad.cellSize = 1.0;
    CHECK(simulate::checkStock(bad).issue == StockIssue::TooManyCells);
}

TEST_CASE("simulate: fitCellSize grows the cell until the stock fits", "[simulate][stock]") {
    simulate::Stock s;
    s.width = 1000;
    s.height = 1000;
    const double fitted = simulate::fitCellSize(s, 0.1, 1'000'000);
    CHECK(fitted >= 1.0);
    s.cellSize = fitted;
    s.maxCells = 1'000'000;
    CHECK(simulate::checkStock(s).ok());
    CHECK(simulate::fitCellSize(s, 5.0, 1'000'000) == 5.0);
    CHECK(simulate::fitCellSize(s, -1.0, 100) == 0.0);
    s.width = std::nan("");
    CHECK(simulate::fitCellSize(s, 1.0, 100) == 0.0);
}

TEST_CASE("simulate: quality levels halve the cell size", "[simulate][stock]") {
    CHECK(simulate::cellSizeFor(simulate::Quality::Coarse) == 0.5);
    CHECK(simulate::cellSizeFor(simulate::Quality::Medium) == 0.25);
    CHECK(simulate::cellSizeFor(simulate::Quality::Fine) == 0.125);
    const simulate::Stock s = simulate::Stock::withQuality(10, 8, 3, simulate::Quality::Medium);
    CHECK(s.cellSize == 0.25);
    CHECK(s.thickness == 3);
}

TEST_CASE("simulate: heightfield layout and the stock cap error", "[simulate][stock]") {
    simulate::Stock s;
    s.width = 10;
    s.height = 6;
    s.thickness = 4;
    s.cellSize = 0.5;
    s.topZ = 4;  // Z zero at the machine bed
    simulate::Simulator sim(s);
    REQUIRE(sim.ok());
    const simulate::Heightfield& f = sim.heightfield();
    CHECK(f.nx() == 20);
    CHECK(f.ny() == 12);
    CHECK(f.cellCount() == 240);
    CHECK(f.topZ() == 4.0);
    CHECK(f.bottomZ() == 0.0);
    CHECK(f.height(3, 3) == 4.0f);
    CHECK(f.centreOf(0, 0).x == 0.25);
    CHECK(f.centreOf(19, 11).y == 5.75);
    CHECK(f.bytes() == 240 * (4 + 1 + 4));
    std::size_t i = 99, j = 99;
    CHECK(f.cellAt({9.99, 5.99}, i, j));
    CHECK(i == 19);
    CHECK(j == 11);
    CHECK_FALSE(f.cellAt({10.0, 3.0}, i, j));
    CHECK_FALSE(f.cellAt({-0.01, 3.0}, i, j));
    CHECK_FALSE(f.cellAt({std::nan(""), 3.0}, i, j));

    simulate::Stock big = s;
    big.width = 5000;
    big.height = 5000;
    big.cellSize = 0.1;
    simulate::Simulator refused(big);
    CHECK_FALSE(refused.ok());
    CHECK(refused.error().status == simulate::SimStatus::TooManyCells);
    CHECK(refused.heightfield().cellCount() == 0);
    CHECK_FALSE(refused.advanceAll());
    CHECK_FALSE(refused.addToolpath(toolpath::Toolpath{}, *ToolShape::flatEndMill(3), 1));

    simulate::Stock noIds = s;
    noIds.trackPassIds = false;
    simulate::Simulator plain(noIds);
    CHECK_FALSE(plain.heightfield().tracksPassIds());
    CHECK(plain.heightfield().bytes() == 240 * 5);
}
