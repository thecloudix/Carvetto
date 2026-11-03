// T-0151: Quick Engrave toolpath (engrave.hpp, ADR-0024). AC1: Outline (the tool centre follows
// open and closed vectors at the depth) and Fill (closed regions cleared with a pattern and
// stepover), equal passes, tools, diagnostics, every rapid at safe Z. AC2: Fill is Pocket's
// machinery (identical moves to pocketToolpath with the mapped parameters) and gouge-free by the
// independent pocket oracle (pocket_oracle.hpp) for V-bits, end mills and ball noses.

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_oracle.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;
using geometry::Vec3;
namespace oracle = carvetto::test::pocket;

namespace {

constexpr double kSafeZ = 5.0;

tooling::Tool vBit(double angleDeg = 60.0, double diameter = 6.35) {
    tooling::Tool t;
    t.id = "vbit";
    t.name = "V-bit";
    t.type = tooling::ToolType::VBit;
    t.diameter = diameter;
    t.includedAngle = angleDeg * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = 3;
    t.defaultCuttingData.feedRate = 1200.0;
    t.defaultCuttingData.plungeRate = 400.0;
    t.defaultCuttingData.spindleSpeed = 16000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 0.2;
    return t;
}

tooling::Tool endMill(double diameter = 1.0) {
    tooling::Tool t = vBit();
    t.id = "em";
    t.name = "End mill";
    t.type = tooling::ToolType::EndMill;
    t.diameter = diameter;
    t.includedAngle = 0.0;
    t.defaultCuttingData.stepover = 0.4 * diameter;
    return t;
}

tooling::Tool ballNose(double diameter = 2.0) {
    tooling::Tool t = endMill(diameter);
    t.id = "bn";
    t.name = "Ball nose";
    t.type = tooling::ToolType::BallNose;
    return t;
}

tooling::Tool diamondDrag(double diameter = 0.5) {
    tooling::Tool t = endMill(diameter);
    t.id = "dd";
    t.name = "Diamond drag";
    t.type = tooling::ToolType::DiamondDrag;
    return t;
}

Contour polygon(const std::vector<Vec2>& pts, bool closed = true) {
    Contour c;
    const std::size_t n = closed ? pts.size() : pts.size() - 1;
    for (std::size_t i = 0; i < n; ++i) {
        c.append(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    c.closed = closed;
    return c;
}

EngraveParams outlineParams(double depth = 0.5, int passes = 1) {
    EngraveParams p;
    p.depth = depth;
    p.passes = passes;
    p.safeZ = kSafeZ;
    return p;
}

EngraveParams fillParams(double depth, int passes, PocketStrategy pattern, double stepover = 0.0) {
    EngraveParams p = outlineParams(depth, passes);
    p.strategy = EngraveStrategy::Fill;
    p.fillPattern = pattern;
    p.stepover = stepover;
    return p;
}

/// Start point of every move.
std::vector<Vec3> starts(const Toolpath& tp) {
    std::vector<Vec3> out;
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        out.push_back(tp.startOf(i));
    }
    return out;
}

/// The move rules every engrave toolpath obeys: the first move is a rapid at safe Z, every rapid
/// is at safe Z (start and end), plunges and retracts are vertical, nothing goes below −depth,
/// and the toolpath ends at safe Z.
void checkMoveRules(const Toolpath& tp, double depth) {
    REQUIRE_FALSE(tp.isEmpty());
    CHECK(tp.moves.front().kind == MoveKind::Rapid);
    const std::vector<Vec3> from = starts(tp);
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        INFO("move " << i);
        CHECK(m.to.z >= -depth - 1e-12);
        if (m.kind == MoveKind::Rapid) {
            CHECK(m.to.z == kSafeZ);
            CHECK(from[i].z == kSafeZ);
        }
        if (m.kind == MoveKind::Plunge || m.kind == MoveKind::Retract) {
            CHECK(std::fabs(m.to.x - from[i].x) <= 1e-12);
            CHECK(std::fabs(m.to.y - from[i].y) <= 1e-12);
        }
        if (m.kind == MoveKind::Plunge) {
            CHECK(m.to.z < from[i].z);
        }
    }
    CHECK(tp.moves.back().to.z == kSafeZ);
}

std::vector<double> distinctCutZ(const Toolpath& tp) {
    std::vector<double> zs;
    for (const Move& m : tp.moves) {
        if ((m.kind == MoveKind::Feed || isArc(m.kind)) && m.to.z < 0.0) {
            zs.push_back(m.to.z);
        }
    }
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    std::reverse(zs.begin(), zs.end());
    return zs;
}

}  // namespace

TEST_CASE("engraveLevels: equal passes down to the depth", "[toolpath][engrave]") {
    CHECK(engraveLevels(0.5, 1) == std::vector<double>{-0.5});
    const std::vector<double> three = engraveLevels(1.5, 3);
    REQUIRE(three.size() == 3);
    CHECK(three[0] == Catch::Approx(-0.5));
    CHECK(three[1] == Catch::Approx(-1.0));
    CHECK(three[2] == -1.5);
    for (int n = 1; n <= 1000; n += 37) {
        const std::vector<double> lv = engraveLevels(0.3, n);
        REQUIRE(lv.size() == static_cast<std::size_t>(n));
        CHECK(lv.back() == -0.3);
        for (std::size_t i = 0; i < lv.size(); ++i) {
            CHECK(lv[i] == Catch::Approx(-0.3 * static_cast<double>(i + 1) / n).margin(1e-12));
        }
    }
    CHECK(engraveLevels(0.0, 1).empty());
    CHECK(engraveLevels(1.0, 0).empty());
    CHECK(engraveLevels(1.0, 1001).empty());
    CHECK(engraveLevels(std::nan(""), 1).empty());
}

TEST_CASE("AC1: Outline follows a closed vector at the depth", "[toolpath][engrave][outline]") {
    const Contour square = Contour::rectangle(Box2{Vec2{10, 10}, Vec2{30, 20}});
    const EngraveResult r = engraveToolpath({square}, vBit(), std::nullopt, outlineParams(0.4));
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    const Toolpath& tp = r.toolpath;
    checkMoveRules(tp, 0.4);
    CHECK(tp.name == "Quick Engrave");
    CHECK(tp.tool.name == "V-bit");
    CHECK(tp.tool.feedRate == 1200.0);
    REQUIRE(tp.moves.size() == 7);  // rapid, plunge, 4 sides, retract
    CHECK(tp.moves[0].to == Vec3{10, 10, kSafeZ});
    CHECK(tp.moves[1].kind == MoveKind::Plunge);
    CHECK(tp.moves[1].to == Vec3{10, 10, -0.4});
    for (std::size_t i = 2; i < 6; ++i) {
        CHECK(tp.moves[i].kind == MoveKind::Feed);
        CHECK(tp.moves[i].to.z == -0.4);
        CHECK(geometry::distanceTo(square, tp.moves[i].to.xy()) <= 1e-12);
    }
    CHECK(tp.moves[5].to.xy() == Vec2{10, 10});
    CHECK(tp.moves[6].kind == MoveKind::Retract);
    CHECK(tp.count(MoveKind::Retract) == 1);
}

TEST_CASE("AC1: Outline in several passes steps down at the start of a closed vector",
          "[toolpath][engrave][outline][passes]") {
    const Contour tri = polygon({{0, 0}, {10, 0}, {5, 8}});
    const EngraveResult r = engraveToolpath({tri}, vBit(), std::nullopt, outlineParams(1.5, 3));
    REQUIRE(r.ok());
    checkMoveRules(r.toolpath, 1.5);
    const std::vector<double> zs = distinctCutZ(r.toolpath);
    REQUIRE(zs.size() == 3);
    CHECK(zs[0] == Catch::Approx(-0.5));
    CHECK(zs[1] == Catch::Approx(-1.0));
    CHECK(zs[2] == -1.5);
    CHECK(r.toolpath.count(MoveKind::Plunge) == 3);
    CHECK(r.toolpath.count(MoveKind::Retract) == 1);
    CHECK(r.toolpath.count(MoveKind::Rapid) == 1);
    for (std::size_t i = 0; i < r.toolpath.moves.size(); ++i) {
        if (r.toolpath.moves[i].kind == MoveKind::Plunge) {
            CHECK(r.toolpath.moves[i].to.xy() == Vec2{0, 0});
        }
    }
}

TEST_CASE("AC1: Outline cuts an open vector back and forth", "[toolpath][engrave][outline][passes]") {
    const Contour zig = polygon({{0, 0}, {10, 0}, {10, 5}, {20, 5}}, false);
    const EngraveResult r = engraveToolpath({zig}, endMill(), std::nullopt, outlineParams(0.9, 3));
    REQUIRE(r.ok());
    checkMoveRules(r.toolpath, 0.9);
    const Toolpath& tp = r.toolpath;
    // rapid, (plunge + 3 feeds) x 3, retract
    REQUIRE(tp.moves.size() == 14);
    CHECK(tp.moves[1].to == Vec3{0, 0, -0.3});
    CHECK(tp.moves[4].to.xy() == Vec2{20, 5});
    CHECK(tp.moves[5].kind == MoveKind::Plunge);
    CHECK(tp.moves[5].to.xy() == Vec2{20, 5});
    CHECK(tp.moves[5].to.z == Catch::Approx(-0.6));
    CHECK(tp.moves[8].to.xy() == Vec2{0, 0});
    CHECK(tp.moves[9].to == Vec3{0, 0, -0.9});
    CHECK(tp.moves[12].to == Vec3{20, 5, -0.9});
    CHECK(tp.moves[13] == Move::retract({20, 5, kSafeZ}));
    CHECK(tp.count(MoveKind::Rapid) == 1);
}

TEST_CASE("AC1: Outline keeps arcs and flattens Beziers within the tolerance", "[toolpath][engrave][outline]") {
    const Contour circle = Contour::circle({50, 50}, 8.0);
    Contour curve;
    curve.append(BezierSpan{{0, 0}, {5, 10}, {15, -10}, {20, 0}});
    curve.closed = false;
    EngraveParams p = outlineParams(0.3);
    p.tolerance.chordal = 0.005;
    const EngraveResult r = engraveToolpath({circle, curve}, vBit(), std::nullopt, p);
    REQUIRE(r.ok());
    checkMoveRules(r.toolpath, 0.3);
    std::size_t arcs = 0;
    std::size_t feeds = 0;
    const std::vector<Vec3> from = starts(r.toolpath);
    for (std::size_t i = 0; i < r.toolpath.moves.size(); ++i) {
        const Move& m = r.toolpath.moves[i];
        if (isArc(m.kind)) {
            ++arcs;
            CHECK(m.centre == Vec2{50, 50});
            CHECK(geometry::distance(m.to.xy(), Vec2{50, 50}) == Catch::Approx(8.0));
        } else if (m.kind == MoveKind::Feed) {
            ++feeds;
            CHECK(geometry::distanceTo(curve, m.to.xy()) <= 1e-9);
            const Vec2 mid = 0.5 * (from[i].xy() + m.to.xy());
            CHECK(geometry::distanceTo(curve, mid) <= p.tolerance.chordal + 1e-9);
        }
    }
    CHECK(arcs >= 1);
    CHECK(feeds >= 8);
}

TEST_CASE("AC1: Outline skips empty, gapped and zero-length vectors", "[toolpath][engrave][outline][diagnostics]") {
    Contour gapped;
    gapped.append(LineSpan{{0, 0}, {10, 0}});
    gapped.append(LineSpan{{10, 1}, {0, 1}});
    gapped.closed = false;
    Contour dot;
    dot.append(LineSpan{{3, 3}, {3, 3}});
    const Contour good = polygon({{20, 0}, {30, 0}}, false);
    const EngraveResult r = engraveToolpath({Contour{}, gapped, dot, good}, vBit(), std::nullopt, outlineParams());
    REQUIRE(r.ok());
    REQUIRE(r.diagnostics.size() == 3);
    CHECK(r.diagnostics[0].issue == ToolpathIssue::EmptyContour);
    CHECK(r.diagnostics[0].contour == std::size_t{0});
    CHECK(r.diagnostics[1].issue == ToolpathIssue::DiscontinuousContour);
    CHECK(r.diagnostics[1].contour == std::size_t{1});
    CHECK(r.diagnostics[2].issue == ToolpathIssue::EmptyContour);
    CHECK(r.diagnostics[2].contour == std::size_t{2});

    const EngraveResult none = engraveToolpath({}, vBit(), std::nullopt, outlineParams());
    CHECK_FALSE(none.ok());
    CHECK(none.has(ToolpathIssue::NoRegion));
}

TEST_CASE("AC1: Outline cuts are ordered nearest first, inner vectors before outer", "[toolpath][engrave][order]") {
    const Contour outer = Contour::rectangle(Box2{Vec2{0, 0}, Vec2{40, 40}});
    const Contour inner = Contour::rectangle(Box2{Vec2{10, 10}, Vec2{20, 20}});
    const Contour farLine = polygon({{200, 0}, {210, 0}}, false);
    const Contour nearLine = polygon({{50, 0}, {60, 0}}, false);
    EngraveParams p = outlineParams();
    p.startPosition = {45, 0};
    const EngraveResult r = engraveToolpath({outer, farLine, inner, nearLine}, vBit(), std::nullopt, p);
    REQUIRE(r.ok());
    std::vector<Vec2> rapids;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Rapid) {
            rapids.push_back(m.to.xy());
        }
    }
    // Nearest from (45, 0): the near line; then the outer square's family, inner first; then the far line.
    REQUIRE(rapids.size() == 4);
    CHECK(rapids[0] == Vec2{50, 0});
    CHECK(rapids[1] == Vec2{10, 10});
    CHECK(rapids[2] == Vec2{0, 0});
    CHECK(rapids[3] == Vec2{200, 0});
}

TEST_CASE("AC2: Fill is Pocket's raster/offset machinery with the engrave depth and passes",
          "[toolpath][engrave][fill][reuse]") {
    const std::vector<Contour> region{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{30, 20}}),
                                      Contour::circle({15, 10}, 4.0)};
    const tooling::Tool tool = vBit(90.0);
    for (const PocketStrategy pattern : {PocketStrategy::Offset, PocketStrategy::Raster}) {
        INFO("pattern " << static_cast<int>(pattern));
        EngraveParams p = fillParams(0.6, 2, pattern, 0.4);
        p.rasterAngle = 0.3;
        const EngraveResult r = engraveToolpath(region, tool, std::nullopt, p);
        REQUIRE(r.ok());
        CHECK(r.diagnostics.empty());

        // The same pocket, asked for directly.
        PocketTool pt{tool, tool.defaultCuttingData};
        pt.cuttingData->stepover = 0.4;
        pt.cuttingData->passDepth = 0.3;
        PocketParams pp;
        pp.name = p.name;
        pp.cutDepth = 0.6;
        pp.safeZ = kSafeZ;
        pp.strategy = pattern;
        pp.rasterAngle = 0.3;
        pp.profilePass = ProfilePass::Last;
        const PocketResult direct = pocketToolpath(region, {pt}, pp);
        REQUIRE(direct.ok());
        CHECK(r.toolpath == direct.toolpaths.front());
        CHECK(distinctCutZ(r.toolpath).size() == 2);
        checkMoveRules(r.toolpath, 0.6);
    }
}

TEST_CASE("AC1: Fill stepover from the parameters or the cutting data, at most the width at depth",
          "[toolpath][engrave][fill][tool]") {
    const std::vector<Contour> region{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 10}})};
    // A 90-degree V-bit 0.5 mm deep cuts 1 mm wide.
    const tooling::Tool tool = vBit(90.0);
    const EngraveResult fromData = engraveToolpath(region, tool, std::nullopt, fillParams(0.5, 1, PocketStrategy::Raster));
    REQUIRE(fromData.ok());  // the cutting data's 0.2 mm
    const EngraveResult own = engraveToolpath(region, tool, std::nullopt, fillParams(0.5, 1, PocketStrategy::Raster, 1.0));
    REQUIRE(own.ok());
    CHECK(own.toolpath.moveCount() < fromData.toolpath.moveCount());

    const EngraveResult tooWide =
        engraveToolpath(region, tool, std::nullopt, fillParams(0.5, 1, PocketStrategy::Raster, 1.01));
    CHECK_FALSE(tooWide.ok());
    REQUIRE(tooWide.diagnostics.size() == 1);
    CHECK(tooWide.diagnostics[0].issue == ToolpathIssue::InvalidTool);
    CHECK(tooWide.diagnostics[0].message.find("the stepover 1.01 mm is wider than the") != std::string::npos);
    CHECK(tooWide.diagnostics[0].message.find("the tool cuts at the engrave depth of 0.5 mm") != std::string::npos);

    tooling::CuttingData noStep = tool.defaultCuttingData;
    noStep.stepover = 0.0;
    const EngraveResult none = engraveToolpath(region, tool, noStep, fillParams(0.5, 1, PocketStrategy::Offset));
    CHECK_FALSE(none.ok());
    CHECK(none.has(ToolpathIssue::InvalidTool));
}

TEST_CASE("AC1: Fill ignores open vectors and fills nested vectors even-odd", "[toolpath][engrave][fill]") {
    const Contour outer = Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 20}});
    const Contour island = Contour::rectangle(Box2{Vec2{5, 5}, Vec2{15, 15}});
    const Contour open = polygon({{30, 0}, {40, 0}}, false);
    const tooling::Tool tool = endMill(1.0);
    const EngraveResult r =
        engraveToolpath({outer, open, island}, tool, std::nullopt, fillParams(0.2, 1, PocketStrategy::Offset));
    REQUIRE(r.ok());
    REQUIRE(r.diagnostics.size() == 1);
    CHECK(r.diagnostics[0].issue == ToolpathIssue::OpenContourIgnored);
    CHECK(r.diagnostics[0].contour == std::size_t{1});
    const std::vector<oracle::Poly> walls{oracle::rectPoly(0, 0, 20, 20), oracle::rectPoly(5, 5, 15, 15)};
    const oracle::GougeReport g = oracle::gougeCheck(r.toolpath, walls, 0.0, kSafeZ);
    CHECK(g.allInside);
    CHECK(g.rapidsAtSafeZ);
    CHECK(g.minDistance >= 0.5 - 1e-9);

    const EngraveResult onlyOpen = engraveToolpath({open}, tool, std::nullopt, fillParams(0.2, 1, PocketStrategy::Offset));
    CHECK_FALSE(onlyOpen.ok());
    CHECK(onlyOpen.has(ToolpathIssue::OpenContourIgnored));
    CHECK(onlyOpen.has(ToolpathIssue::NoRegion));
}

TEST_CASE("AC2: Fill is gouge-free by the pocket oracle for V-bits, end mills and ball noses (generative)",
          "[toolpath][engrave][fill][safety]") {
    std::mt19937_64 rng(151);
    auto U = [&rng](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    auto I = [&rng](int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); };
    std::size_t checked = 0;
    for (int scene = 0; scene < 40; ++scene) {
        INFO("scene " << scene);
        std::vector<oracle::Poly> polys;
        if (scene % 2 == 0) {
            // A star-like letter shape.
            oracle::Poly star;
            const int k = I(5, 14) * 2;
            for (int i = 0; i < k; ++i) {
                const double t = 2.0 * geometry::kPi * (i + U(-0.2, 0.2)) / k;
                const double rad = i % 2 ? U(2.0, 5.0) : U(6.0, 12.0);
                star.push_back({rad * std::cos(t), rad * std::sin(t)});
            }
            polys.push_back(star);
        } else {
            // A plate with islands.
            const double w = U(15, 40);
            const double h = U(10, 25);
            polys.push_back(oracle::rectPoly(0, 0, w, h));
            const int n = I(0, 3);
            for (int i = 0; i < n; ++i) {
                const double cx = w * (i + 0.5) / (n + 0.5);
                polys.push_back(oracle::rectPoly(cx - 1.5, h * 0.3, cx + 1.5, h * 0.3 + U(1.0, h * 0.5)));
            }
        }
        std::vector<Contour> contours;
        for (const oracle::Poly& poly : polys) {
            contours.push_back(Contour::fromPolyline(geometry::Polyline{poly, true}));
        }
        tooling::Tool tool;
        switch (scene % 3) {
            case 0: tool = vBit(I(0, 1) ? 60.0 : 90.0); break;
            case 1: tool = endMill(U(0.5, 2.0)); break;
            default: tool = ballNose(U(1.0, 3.0)); break;
        }
        const double depth = U(0.1, 1.5);
        const int passes = I(1, 3);
        const double width = tooling::cuttingWidthAtDepth(tool, depth);
        EngraveParams p = fillParams(depth, passes, I(0, 1) ? PocketStrategy::Raster : PocketStrategy::Offset,
                                     width * U(0.2, 0.9));
        p.rasterAngle = U(0.0, geometry::kPi);
        const EngraveResult r = engraveToolpath(contours, tool, std::nullopt, p);
        if (!r.ok()) {
            CHECK(r.has(ToolpathIssue::ToolTooLarge));  // a narrow shape may hold no tool
            continue;
        }
        ++checked;
        checkMoveRules(r.toolpath, depth);
        const oracle::GougeReport g = oracle::gougeCheck(r.toolpath, polys, 0.0, kSafeZ);
        CHECK(g.allInside);
        CHECK(g.rapidsAtSafeZ);
        // The tool's width at the engrave depth stays inside every wall (a V-bit's cone at depth).
        CHECK(g.minDistance >= 0.5 * width - 1e-7);
        CHECK(distinctCutZ(r.toolpath).size() == static_cast<std::size_t>(passes));
    }
    CHECK(checked >= 30);
}

TEST_CASE("AC1: V-bit, engraving tool, end mill and ball nose are accepted",
          "[toolpath][engrave][tool]") {
    CHECK(engraveToolAllowed(tooling::ToolType::VBit));
    CHECK(engraveToolAllowed(tooling::ToolType::Engraving));
    CHECK(engraveToolAllowed(tooling::ToolType::EndMill));
    CHECK(engraveToolAllowed(tooling::ToolType::BallNose));
    CHECK_FALSE(engraveToolAllowed(tooling::ToolType::DiamondDrag));  // owner decision 2026-11-03
    CHECK_FALSE(engraveToolAllowed(tooling::ToolType::Drill));
    CHECK_FALSE(engraveToolAllowed(tooling::ToolType::FormTool));
    CHECK_FALSE(engraveToolAllowed(tooling::ToolType::TaperedBallNose));
    CHECK_FALSE(engraveToolAllowed(tooling::ToolType::RadiusedEndMill));

    const std::vector<Contour> square{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}})};
    tooling::Tool engraving = vBit(30.0);
    engraving.type = tooling::ToolType::Engraving;
    engraving.tipDiameter = 0.2;
    for (const tooling::Tool& t : {vBit(), engraving, endMill(), ballNose()}) {
        INFO(t.name);
        const EngraveResult r = engraveToolpath(square, t, std::nullopt, outlineParams(0.2));
        CHECK(r.ok());
        CHECK(r.diagnostics.empty());
    }
    tooling::Tool drill = endMill(3.0);
    drill.type = tooling::ToolType::Drill;
    drill.includedAngle = 2.0;
    const EngraveResult refused = engraveToolpath(square, drill, std::nullopt, outlineParams(0.2));
    CHECK_FALSE(refused.ok());
    REQUIRE(refused.diagnostics.size() == 1);
    CHECK(refused.diagnostics[0].issue == ToolpathIssue::InvalidTool);
    CHECK(refused.diagnostics[0].message.find("is a drill") != std::string::npos);
}

TEST_CASE("AC1: diamond-drag tools are refused until spindle-off support exists", "[toolpath][engrave][tool]") {
    // Owner decision 2026-11-03 (ADR-0024 section 6): a drag tool must not run with the spindle on,
    // and toolpaths cannot yet leave it off.
    const Contour line = polygon({{0, 0}, {25, 0}, {25, 10}}, false);
    for (const EngraveStrategy s : {EngraveStrategy::Outline, EngraveStrategy::Fill}) {
        EngraveParams p = outlineParams(0.05);
        p.strategy = s;
        const EngraveResult r =
            engraveToolpath({line, Contour::rectangle(Box2{Vec2{0, 20}, Vec2{10, 30}})}, diamondDrag(), std::nullopt, p);
        CHECK_FALSE(r.ok());
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics[0].issue == ToolpathIssue::InvalidTool);
        CHECK(r.diagnostics[0].message ==
              "tool 'Diamond drag' is a diamond drag tool; diamond-drag tools need spindle-off support, not yet available");
    }
}

TEST_CASE("AC2: Fill words Pocket's diagnostics for the engraving", "[toolpath][engrave][fill][diagnostics]") {
    // A 2 mm square holds no 3 mm end mill: Pocket's "too large for the pocket" reads as a fill.
    const std::vector<Contour> small{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{2, 2}})};
    const EngraveResult r = engraveToolpath(small, endMill(3.0), std::nullopt, fillParams(0.2, 1, PocketStrategy::Offset));
    CHECK_FALSE(r.ok());
    REQUIRE(r.diagnostics.size() == 1);
    CHECK(r.diagnostics[0].issue == ToolpathIssue::ToolTooLarge);
    CHECK_FALSE(r.diagnostics[0].tool.has_value());
    CHECK(r.diagnostics[0].message == "tool 'End mill': too large for the fill area, nothing to cut");
    const EngraveResult open =
        engraveToolpath({polygon({{0, 0}, {5, 0}}, false)}, endMill(), std::nullopt, fillParams(0.2, 1, PocketStrategy::Raster));
    REQUIRE_FALSE(open.diagnostics.empty());
    for (const ToolpathDiagnostic& d : open.diagnostics) {
        INFO(d.message);
        CHECK(d.message.find("pocket") == std::string::npos);
    }
}

TEST_CASE("AC1: unusable tools and cutting data are refused", "[toolpath][engrave][tool]") {
    const std::vector<Contour> square{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}})};
    tooling::Tool noAngle = vBit();
    noAngle.includedAngle = 0.0;
    tooling::CuttingData noFeed = vBit().defaultCuttingData;
    noFeed.feedRate = 0.0;
    tooling::CuttingData noPlunge = vBit().defaultCuttingData;
    noPlunge.plungeRate = std::nan("");
    CHECK(engraveToolpath(square, noAngle, std::nullopt, outlineParams()).has(ToolpathIssue::InvalidTool));
    CHECK(engraveToolpath(square, vBit(), noFeed, outlineParams()).has(ToolpathIssue::InvalidTool));
    CHECK(engraveToolpath(square, vBit(), noPlunge, outlineParams()).has(ToolpathIssue::InvalidTool));
    // The chosen cutting data overrides the tool's default data.
    tooling::CuttingData chosen = vBit().defaultCuttingData;
    chosen.feedRate = 777.0;
    chosen.toolNumber = 9;
    const EngraveResult r = engraveToolpath(square, vBit(), chosen, outlineParams());
    REQUIRE(r.ok());
    CHECK(r.toolpath.tool.feedRate == 777.0);
    CHECK(r.toolpath.tool.toolNumber == 9);
}

TEST_CASE("AC1: invalid parameters are refused", "[toolpath][engrave][params]") {
    const std::vector<Contour> square{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}})};
    std::vector<EngraveParams> bad;
    auto with = [&bad](auto change) {
        EngraveParams p = outlineParams();
        change(p);
        bad.push_back(p);
    };
    with([](EngraveParams& p) { p.depth = 0.0; });
    with([](EngraveParams& p) { p.depth = -1.0; });
    with([](EngraveParams& p) { p.depth = std::nan(""); });
    with([](EngraveParams& p) { p.passes = 0; });
    with([](EngraveParams& p) { p.passes = 1001; });
    with([](EngraveParams& p) { p.safeZ = 0.0; });
    with([](EngraveParams& p) { p.safeZ = std::numeric_limits<double>::infinity(); });
    with([](EngraveParams& p) { p.stepover = -0.1; });
    with([](EngraveParams& p) { p.rasterAngle = std::nan(""); });
    with([](EngraveParams& p) { p.startPosition.y = std::nan(""); });
    with([](EngraveParams& p) { p.tolerance.chordal = 0.0; });
    with([](EngraveParams& p) { p.tolerance.distance = 2.0 * p.tolerance.chordal; });
    for (std::size_t i = 0; i < bad.size(); ++i) {
        INFO("case " << i);
        for (const EngraveStrategy s : {EngraveStrategy::Outline, EngraveStrategy::Fill}) {
            EngraveParams p = bad[i];
            p.strategy = s;
            const EngraveResult r = engraveToolpath(square, vBit(), std::nullopt, p);
            CHECK_FALSE(r.ok());
            REQUIRE(r.diagnostics.size() == 1);
            CHECK(r.diagnostics[0].issue == ToolpathIssue::InvalidParameters);
        }
    }
}

TEST_CASE("AC1: engraving is deterministic and refuses runaway work", "[toolpath][engrave][limits]") {
    std::vector<Contour> many;
    for (int i = 0; i < 50; ++i) {
        many.push_back(Contour::circle({static_cast<double>(i % 10) * 7.0, static_cast<double>(i / 10) * 7.0}, 3.0));
    }
    const EngraveResult a = engraveToolpath(many, vBit(), std::nullopt, outlineParams(0.4, 2));
    const EngraveResult b = engraveToolpath(many, vBit(), std::nullopt, outlineParams(0.4, 2));
    REQUIRE(a.ok());
    CHECK(a.toolpath == b.toolpath);
    CHECK(a.toolpath.count(MoveKind::Rapid) == 50);

    // 1000 passes over a long, finely flattened curve: far above the 20,000,000-move cap.
    Contour wiggle;
    for (int i = 0; i < 25000; ++i) {
        const double x = static_cast<double>(i);
        wiggle.append(LineSpan{{x, 0}, {x + 1.0, (i % 2) ? 0.0 : 1.0}});
        wiggle.append(LineSpan{{x + 1.0, (i % 2) ? 0.0 : 1.0}, {x + 1.0, 0}});
    }
    wiggle.closed = false;
    const EngraveResult huge = engraveToolpath({wiggle}, vBit(), std::nullopt, outlineParams(1.0, 1000));
    CHECK_FALSE(huge.ok());
    CHECK(huge.has(ToolpathIssue::WorkLimitExceeded));
}

// T-0155 (a): only Pocket's fixed phrases are reworded; a quoted tool name stays as it is.
TEST_CASE("T-0155: Fill rewording leaves the tool name untouched", "[toolpath][engrave][fill][diagnostics]") {
    const std::vector<Contour> small{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{2, 2}})};
    for (const char* name : {"my pocket cutter", "the pocket", "tool 1 (x)", "pocket boundary too large for the pocket"}) {
        INFO(name);
        tooling::Tool tool = endMill(3.0);
        tool.name = name;
        const EngraveResult r = engraveToolpath(small, tool, std::nullopt, fillParams(0.2, 1, PocketStrategy::Offset));
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics[0].issue == ToolpathIssue::ToolTooLarge);
        CHECK(r.diagnostics[0].message == "tool '" + std::string(name) + "': too large for the fill area, nothing to cut");
    }
    // Other labelled Pocket messages: the label is rewritten, the name kept, no stray "pocket".
    tooling::Tool tool = endMill(0.5);
    tool.name = "pocket";
    EngraveParams tiny = fillParams(0.2, 1, PocketStrategy::Raster, 0.005);
    const EngraveResult r = engraveToolpath(small, tool, std::nullopt, tiny);
    REQUIRE_FALSE(r.diagnostics.empty());
    for (const ToolpathDiagnostic& d : r.diagnostics) {
        INFO(d.message);
        CHECK(d.message.rfind("tool 'pocket'", 0) == 0);
        CHECK(d.message.find("pocket", std::string("tool 'pocket'").size()) == std::string::npos);
    }
}

// T-0155 (b): depth / passes has a floor (kEngraveMinPassDepth); at or above it the levels are
// exactly `passes`, below it they are refused, never silently fewer.
TEST_CASE("T-0155: engraveLevels yields exactly passes levels down to the pass-depth floor",
          "[toolpath][engrave][passes]") {
    for (const double depth : {1e-6, 1e-5, 1e-4, 1e-3, 0.01, 0.3, 7.0}) {
        for (const int n : {1, 2, 3, 7, 10, 99, 100, 333, 999, 1000}) {
            const bool allowed = depth / static_cast<double>(n) >= kEngraveMinPassDepth;
            const std::vector<double> lv = engraveLevels(depth, n);
            INFO("depth " << depth << " passes " << n);
            if (!allowed) {
                CHECK(lv.empty());
                continue;
            }
            REQUIRE(lv.size() == static_cast<std::size_t>(n));
            CHECK(lv.back() == -depth);
            for (std::size_t i = 0; i < lv.size(); ++i) {
                CHECK(lv[i] == Catch::Approx(-depth * static_cast<double>(i + 1) / n).margin(depth * 1e-12));
            }
        }
    }
    // Exactly at the floor: 1000 passes of 0.000001 mm.
    CHECK(engraveLevels(1000.0 * kEngraveMinPassDepth, 1000).size() == 1000);
    // The harness case from T-0151 verification: once 999 levels, now refused.
    CHECK(engraveLevels(1e-6, 1000).empty());
    CHECK(engraveLevels(0.5e-6, 1).empty());
}

TEST_CASE("T-0155: a pass shallower than the floor is refused with a clear message", "[toolpath][engrave][params]") {
    const std::vector<Contour> square{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}})};
    for (const EngraveStrategy s : {EngraveStrategy::Outline, EngraveStrategy::Fill}) {
        EngraveParams p = s == EngraveStrategy::Fill ? fillParams(1e-6, 1000, PocketStrategy::Offset)
                                                     : outlineParams(1e-6, 1000);
        const EngraveResult r = engraveToolpath(square, endMill(), std::nullopt, p);
        CHECK_FALSE(r.ok());
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics[0].issue == ToolpathIssue::InvalidParameters);
        CHECK(r.diagnostics[0].message ==
              "each pass must be at least 0.000001 mm deep (engrave depth / number of passes)");
        CHECK(r.toolpath.isEmpty());
    }
    // At the floor the Outline cuts every level: one plunge per pass on a closed vector.
    const EngraveResult atFloor =
        engraveToolpath({Contour::circle({0, 0}, 1)}, endMill(), std::nullopt, outlineParams(1e-3, 1000));
    REQUIRE(atFloor.ok());
    CHECK(distinctCutZ(atFloor.toolpath).size() == 1000);
}

// T-0155 (c): numbers in messages are rounded to 6 significant digits.
TEST_CASE("T-0155: stepover refusal prints widths rounded", "[toolpath][engrave][fill][diagnostics]") {
    const std::vector<Contour> region{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 10}})};
    // A 90-degree V-bit 0.5 mm deep cuts 2 * 0.5 * tan(45 deg) = 0.9999999999999999 mm in doubles.
    const EngraveResult r =
        engraveToolpath(region, vBit(90.0), std::nullopt, fillParams(0.5, 1, PocketStrategy::Raster, 1.01));
    REQUIRE(r.diagnostics.size() == 1);
    CHECK(r.diagnostics[0].message ==
          "tool 'V-bit': the stepover 1.01 mm is wider than the 1 mm the tool cuts at the engrave depth of 0.5 mm");
    const EngraveResult third =
        engraveToolpath(region, vBit(90.0), std::nullopt, fillParams(1.0 / 3.0, 1, PocketStrategy::Raster, 2.54));
    REQUIRE(third.diagnostics.size() == 1);
    CHECK(third.diagnostics[0].message ==
          "tool 'V-bit': the stepover 2.54 mm is wider than the 0.666667 mm the tool cuts at the engrave depth of "
          "0.333333 mm");
}
