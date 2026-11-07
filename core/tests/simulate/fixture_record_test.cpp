// T-0222 AC1: the heightfield of real Pocket / Profile / V-Carve / Engrave records. The records
// are made from the CLI's DXF fixtures (app/cli/tests/data: part.dxf, sign.dxf, tools.json) with
// the parameters of the CLI's example params files, cut by the simulator at medium-ish quality,
// and every cell is compared with the brute-force sampled oracle of oracle.hpp (tool positions
// every cell/8 along the record, each cell centre taking the lowest cutting height).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/io/dxf.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;
using geometry::Contour;

namespace {

std::string dataPath(const std::string& name) {
    return std::string(CARVETTO_CLI_DATA_DIR) + "/" + name;
}

std::vector<std::uint8_t> readBytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in);
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::vector<Contour> layerContours(const std::string& file, const std::string& layer) {
    const auto dxf = io::importDxf(readBytes(dataPath(file)));
    REQUIRE(dxf.ok());
    std::vector<Contour> out;
    for (const io::DxfContour& c : dxf.value().contours) {
        if (dxf.value().layers[c.layer].name == layer) out.push_back(c.contour);
    }
    REQUIRE_FALSE(out.empty());
    return out;
}

tooling::Tool toolById(const std::string& id) {
    std::ifstream in(dataPath("tools.json"), std::ios::binary);
    REQUIRE(in);
    std::stringstream ss;
    ss << in.rdbuf();
    const tooling::ImportResult r = tooling::importToolDatabaseJson(ss.str());
    REQUIRE(r.ok);
    const tooling::Tool* t = r.database.findTool(id);
    REQUIRE(t != nullptr);
    return *t;
}

geometry::Box2 boundsOf(const std::vector<Contour>& cs) {
    geometry::Box2 b;
    for (const Contour& c : cs) {
        b.expand(c.bounds().min);
        b.expand(c.bounds().max);
    }
    return b;
}

struct Record {
    std::vector<Toolpath> paths;
    std::vector<tooling::Tool> tools;
    geometry::Box2 area;
};

struct Outcome {
    Deviation dev;
    std::size_t moves{0};
    std::size_t cut{0};
    std::size_t diagnostics{0};
};

/// Simulates a record on a stock around `area` and checks every cell against the oracle.
Outcome simulateAndCompare(const Record& rec, double cell, double stockThickness, const char* label, double slack = 0.0) {
    Stock stock;
    stock.cellSize = cell;
    stock.thickness = stockThickness;
    stock.origin = {rec.area.min.x - 8.0, rec.area.min.y - 8.0};
    stock.width = rec.area.max.x - rec.area.min.x + 16.0;
    stock.height = rec.area.max.y - rec.area.min.y + 16.0;
    Simulator sim(stock);
    REQUIRE(sim.ok());
    Outcome out;
    std::uint32_t id = 1;
    for (std::size_t k = 0; k < rec.paths.size(); ++k) {
        const auto shape = ToolShape::fromTool(rec.tools[k]);
        REQUIRE(shape.has_value());
        REQUIRE(sim.addToolpath(rec.paths[k], *shape, id++));
    }
    REQUIRE(sim.advanceAll());
    out.moves = sim.moveCount();
    out.cut = sim.heightfield().cutCells();
    out.diagnostics = sim.diagnostics().size();
    // The oracle: the same tool profiles written independently, applied toolpath after toolpath
    // (the lowest value of all of them, since every toolpath only lowers the surface).
    for (const auto& d : sim.diagnostics()) {
        INFO(label << ": diagnostic kind " << static_cast<int>(d.kind) << " at move " << d.move);
    }
    Heightfield oracleMin = sim.heightfield();  // only for its geometry
    std::vector<Samples> sets;
    std::vector<Profile> profiles;
    for (std::size_t k = 0; k < rec.paths.size(); ++k) {
        const tooling::Tool& t = rec.tools[k];
        Profile p;
        switch (t.type) {
            case tooling::ToolType::EndMill: p = flatProfile(t.diameter); break;
            case tooling::ToolType::BallNose: p = ballProfile(t.diameter); break;
            case tooling::ToolType::VBit: p = vProfile(t.diameter, t.includedAngle); break;
            default: FAIL("unexpected tool type");
        }
        profiles.push_back(p);
        Vec3 start = rec.paths[k].moves.empty() ? Vec3{} : rec.paths[k].moves.front().to;
        Samples s = sampleMoves(rec.paths[k].moves, start, cell / 8.0);
        s.buildIndex(p.radius + s.spacingXY);
        sets.push_back(std::move(s));
    }
    const Heightfield& f = sim.heightfield();
    for (std::size_t j = 0; j < f.ny(); ++j) {
        for (std::size_t i = 0; i < f.nx(); ++i) {
            const geometry::Vec2 c = f.centreOf(i, j);
            double lo = f.topZ();
            double hi = f.topZ();
            for (std::size_t k = 0; k < sets.size(); ++k) {
                const Bounds b = bounds(sets[k], profiles[k], c.x, c.y, f.topZ(), f.bottomZ());
                lo = std::min(lo, b.lo - slack);
                hi = std::min(hi, b.hi);
            }
            const double v = f.height(i, j);
            ++out.dev.cells;
            out.dev.belowLo = std::max(out.dev.belowLo, lo - v);
            out.dev.aboveHi = std::max(out.dev.aboveHi, v - hi);
            out.dev.maxHiMinusSim = std::max(out.dev.maxHiMinusSim, hi - v);
            if (v < lo - 1e-5 || v > hi + 1e-5) ++out.dev.violations;
        }
    }
    std::cout << "[fixture] " << label << ": " << out.moves << " moves, " << out.dev.cells << " cells ("
              << cell << " mm), " << out.cut << " cut; below lower bound " << out.dev.belowLo
              << " mm, above upper bound " << out.dev.aboveHi << " mm, oracle sampling error (oracle - cell, at most) " << out.dev.maxHiMinusSim << " mm, violations " << out.dev.violations
              << ", diagnostics " << out.diagnostics << "\n";
    return out;
}

}  // namespace

TEST_CASE("simulate: Pocket record of part.dxf against the brute-force oracle", "[simulate][fixture][pocket]") {
    const std::vector<Contour> pockets = layerContours("part.dxf", "Pockets");
    const tooling::Tool tool = toolById("end-mill-6mm");
    toolpath::PocketParams p;
    p.name = "Pockets";
    p.startDepth = 0;
    p.cutDepth = 4;
    p.safeZ = 5;
    p.strategy = toolpath::PocketStrategy::Offset;
    p.profilePass = toolpath::ProfilePass::Last;
    const toolpath::PocketResult r = toolpath::pocketToolpath(pockets, {toolpath::PocketTool{tool, std::nullopt}}, p);
    REQUIRE(r.ok());
    Record rec;
    rec.area = boundsOf(pockets);
    for (const Toolpath& t : r.toolpaths) {
        rec.paths.push_back(t);
        rec.tools.push_back(tool);
    }
    const Outcome o = simulateAndCompare(rec, 0.5, 12.0, "Pocket");
    CHECK(o.dev.violations == 0);
    CHECK(o.diagnostics == 0);
    CHECK(o.cut > 500);
}

TEST_CASE("simulate: Profile record of part.dxf against the brute-force oracle", "[simulate][fixture][profile]") {
    const std::vector<Contour> outline = layerContours("part.dxf", "Outline");
    const tooling::Tool tool = toolById("end-mill-3mm");
    toolpath::ProfileParams p;
    p.name = "Cut out";
    p.side = toolpath::ProfileSide::Outside;
    p.passes.cutDepth = 6;
    p.passes.passDepth = 3;
    p.safeZ = 5;
    p.tabs.enabled = true;
    p.tabs.count = 4;
    p.tabs.length = 8;
    p.tabs.thickness = 2;
    const toolpath::ProfileResult r = toolpath::profileToolpath(outline, tool, p);
    REQUIRE(r.ok());
    Record rec;
    rec.area = boundsOf(outline);
    rec.paths.push_back(r.toolpath);
    rec.tools.push_back(tool);
    const Outcome o = simulateAndCompare(rec, 0.5, 12.0, "Profile with tabs");
    CHECK(o.dev.violations == 0);
    CHECK(o.diagnostics == 0);
    CHECK(o.cut > 500);
}

TEST_CASE("simulate: V-Carve record of sign.dxf against the brute-force oracle", "[simulate][fixture][v_carve]") {
    const std::vector<Contour> letters = layerContours("sign.dxf", "Letters");
    const tooling::Tool tool = toolById("v-bit-90");
    toolpath::VCarvingParams p;
    p.name = "Letters";
    p.safeZ = 5;
    p.order.leftToRight = true;
    p.tolerance.chordal = 0.05;
    const toolpath::VCarvingResult r = toolpath::vCarvingToolpath(letters, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    Record rec;
    rec.area = boundsOf(letters);
    for (const Toolpath& t : r.toolpaths()) {
        rec.paths.push_back(t);
        rec.tools.push_back(tool);
    }
    const Outcome o = simulateAndCompare(rec, 0.5, 12.0, "V-Carve");
    CHECK(o.dev.violations == 0);
    CHECK(o.diagnostics == 0);
    CHECK(o.cut > 200);
}

TEST_CASE("simulate: Engrave record of part.dxf against the brute-force oracle", "[simulate][fixture][engrave]") {
    const std::vector<Contour> outline = layerContours("part.dxf", "Outline");
    const tooling::Tool tool = toolById("v-bit-90");
    toolpath::EngraveParams p;
    p.name = "Border";
    p.depth = 0.5;
    p.safeZ = 5;
    p.strategy = toolpath::EngraveStrategy::Outline;
    const toolpath::EngraveResult r = toolpath::engraveToolpath(outline, tool, std::nullopt, p);
    REQUIRE(r.ok());
    Record rec;
    rec.area = boundsOf(outline);
    rec.paths.push_back(r.toolpath);
    rec.tools.push_back(tool);
    const Outcome o = simulateAndCompare(rec, 0.25, 12.0, "Engrave outline");
    CHECK(o.dev.violations == 0);
    CHECK(o.diagnostics == 0);
    CHECK(o.cut > 200);
    // Engrave fill with the V-bit at shallow depth too (different move mix).
    toolpath::EngraveParams f = p;
    f.strategy = toolpath::EngraveStrategy::Fill;
    f.stepover = 1.0;
    const toolpath::EngraveResult rf = toolpath::engraveToolpath(layerContours("part.dxf", "Pockets"), tool, std::nullopt, f);
    REQUIRE(rf.ok());
    Record recFill;
    recFill.area = boundsOf(layerContours("part.dxf", "Pockets"));
    recFill.paths.push_back(rf.toolpath);
    recFill.tools.push_back(tool);
    const Outcome of = simulateAndCompare(recFill, 0.5, 12.0, "Engrave fill");
    CHECK(of.dev.violations == 0);
    CHECK(of.diagnostics == 0);
}

TEST_CASE("simulate: V-carve widths and depths follow the medial-axis radius formula", "[simulate][fixture][v_carve][medial]") {
    // A 6 x 30 slot with a 90 degree V-bit: the medial axis is the centre line with inscribed
    // radius 3 (less near the ends), the carve depth at radius r is r / tan(45 deg), and the cut
    // across the slot is the V groove z = -(r - |x|) / tan(half angle). With a 60 degree bit the
    // depth is r / tan(30 deg) = 1.732 r.
    for (double angleDeg : {90.0, 60.0}) {
        INFO("angle " << angleDeg);
        const double halfSlot = 3.0;
        const std::vector<Contour> slot{Contour::rectangle(geometry::Box2{{5, 5}, {35, 11}})};
        tooling::Tool tool;
        tool.id = "v";
        tool.name = "v";
        tool.type = tooling::ToolType::VBit;
        tool.diameter = 12.7;
        tool.includedAngle = angleDeg * kDeg;
        tool.defaultCuttingData.toolNumber = 1;
        tool.defaultCuttingData.feedRate = 1500;
        tool.defaultCuttingData.plungeRate = 500;
        tool.defaultCuttingData.spindleSpeed = 18000;
        toolpath::VCarvingParams p;
        p.safeZ = 5;
        p.tolerance.chordal = 0.01;
        const toolpath::VCarvingResult r = toolpath::vCarvingToolpath(slot, tool, std::nullopt, {}, p);
        REQUIRE(r.ok());
        Stock stock = stockOf(40, 16, 0.1, 12.0);
        Simulator sim(stock);
        REQUIRE(sim.addToolpath(r.toolpath, *ToolShape::fromTool(tool), 1));
        REQUIRE(sim.advanceAll());
        const Heightfield& f = sim.heightfield();
        const double k = 1.0 / std::tan(angleDeg * kDeg / 2);
        double worstDepth = 0;
        double widthAtSurface = 0;
        std::size_t rows = 0;
        for (std::size_t j = 0; j < f.ny(); ++j) {
            const double y = f.centreOf(0, j).y;
            const double x = y - 8.0;  // across the slot, centre line at y = 8
            if (std::fabs(x) >= halfSlot - 0.05) continue;
            const double expect = -(halfSlot - std::fabs(x)) * k;
            for (std::size_t i = 0; i < f.nx(); ++i) {
                const double cx = f.centreOf(i, 0).x;
                if (cx < 15 || cx > 25) continue;  // away from the ends
                worstDepth = std::max(worstDepth, std::fabs(f.height(i, j) - expect));
            }
            ++rows;
        }
        // width at the surface: cells lowered across the middle column of the slot
        std::size_t ic = 0, jc = 0;
        REQUIRE(f.cellAt({20.0, 8.0}, ic, jc));
        for (std::size_t j = 0; j < f.ny(); ++j) {
            if (f.height(ic, j) < -1e-4f) widthAtSurface += f.cellSize();
        }
        std::cout << "[v-carve] " << angleDeg << " deg slot: max depth deviation " << worstDepth
                  << " mm over " << rows << " rows, width at surface " << widthAtSurface << " mm (slot 6)\n";
        CHECK(rows > 40);
        CHECK(worstDepth <= 0.02 * k + 1e-5);  // 2 x chordal, as a footprint radius, times the cone slope
        CHECK(std::fabs(widthAtSurface - 2 * halfSlot) <= 0.25);
        CHECK(sim.diagnostics().empty());
    }
    // A disc of radius 5: one plunge at the centre to depth r / tan(half angle), cone around it.
    const std::vector<Contour> disc{Contour::circle({20, 8}, 5.0)};
    tooling::Tool tool;
    tool.id = "v";
    tool.name = "v";
    tool.type = tooling::ToolType::VBit;
    tool.diameter = 12.7;
    tool.includedAngle = 90 * kDeg;
    tool.defaultCuttingData.toolNumber = 1;
    tool.defaultCuttingData.feedRate = 1500;
    tool.defaultCuttingData.plungeRate = 500;
    tool.defaultCuttingData.spindleSpeed = 18000;
    toolpath::VCarvingParams p;
    p.safeZ = 5;
    const toolpath::VCarvingResult r = toolpath::vCarvingToolpath(disc, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    Simulator sim(stockOf(40, 16, 0.1, 12.0));
    REQUIRE(sim.addToolpath(r.toolpath, *ToolShape::fromTool(tool), 1));
    REQUIRE(sim.advanceAll());
    const Heightfield& f = sim.heightfield();
    double worst = 0;
    for (std::size_t j = 0; j < f.ny(); ++j) {
        for (std::size_t i = 0; i < f.nx(); ++i) {
            const geometry::Vec2 c = f.centreOf(i, j);
            const double rr = std::hypot(c.x - 20, c.y - 8);
            if (rr >= 4.9) continue;
            worst = std::max(worst, std::fabs(f.height(i, j) - (-(5.0 - rr))));
        }
    }
    std::cout << "[v-carve] disc: max cone deviation " << worst << " mm\n";
    CHECK(worst <= 0.03);
}
