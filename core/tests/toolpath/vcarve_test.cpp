// T-0153: V-carve toolpath (ADR-0025). AC1: every move of the carving tool stays inside the
// region - its cross-section at the start plane never crosses a wall (exact cone/wall check of
// vcarve_oracle.hpp over a generative sweep) -, the depth follows radius / tan(angle / 2) within
// the tolerance, flat depths are honoured with clearance pocketing (gouge-free by the pocket
// oracle). AC2: text-heavy jobs (5,000 glyph-like regions) within a documented CPU time bound.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/vcarve.hpp"
#include "pocket_oracle.hpp"
#include "vcarve_oracle.hpp"
#include "vcarve_test_util.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far).
#include "../geometry/cpu_time.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::test::vcarve::ballNose;
using carvetto::test::vcarve::endMill;
using carvetto::test::vcarve::polygon;
using carvetto::test::vcarve::vBit;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
namespace orc = carvetto::test::vcarve;

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

constexpr double kSafeZ = 5.0;

double deepest(const Toolpath& tp) {
    return tp.isEmpty() ? 0.0 : -*tp.minZ();
}

/// Every Z of the toolpath's cutting moves, in order.
std::size_t countKind(const Toolpath& tp, MoveKind k) {
    return tp.count(k);
}

/// The structural rules every V-carve toolpath follows: rapids only at safe Z, every cut starts
/// with a rapid over its start and a plunge, and ends with a retract to safe Z.
void checkStructure(const Toolpath& tp) {
    REQUIRE_FALSE(tp.isEmpty());
    CHECK(tp.moves.front().kind == MoveKind::Rapid);
    CHECK(tp.moves.back().kind == MoveKind::Retract);
    CHECK(tp.moves.back().to.z == kSafeZ);
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        if (m.kind == MoveKind::Rapid) {
            CHECK(m.to.z == kSafeZ);
            CHECK(tp.startOf(i).z == kSafeZ);
        }
        if (m.kind == MoveKind::Plunge || m.kind == MoveKind::Retract) {
            CHECK(m.to.x == tp.startOf(i).x);
            CHECK(m.to.y == tp.startOf(i).y);
        }
    }
}

/// Region as polygons for the pocket oracle (the fixtures here are polygonal).
std::vector<test::pocket::Poly> polys(const std::vector<Contour>& cs) {
    std::vector<test::pocket::Poly> out;
    for (const Contour& c : cs) {
        out.push_back(c.polygonise(geometry::Tolerance{0.001, 1e-9, 0.0001}).points);
    }
    return out;
}

/// The pocket parameters vcarveToolpath hands to pocketToolpath for the flat areas.
PocketParams flatPocket(const VCarveParams& p, double limit, double allowance) {
    PocketParams pocket;
    pocket.name = p.name;
    pocket.startDepth = 0.0;
    pocket.cutDepth = limit;
    pocket.maintainExactStep = true;
    pocket.lastPassThickness = 0.0;
    pocket.safeZ = p.safeZ + p.startDepth;
    pocket.strategy = p.clearanceStrategy;
    pocket.direction = p.clearanceDirection;
    pocket.rasterAngle = p.clearanceRasterAngle;
    pocket.profilePass = ProfilePass::Last;
    pocket.allowance = allowance;
    pocket.rampPlunge = p.rampPlunge;
    pocket.rampDistance = p.rampDistance;
    pocket.tolerance = p.tolerance;
    return pocket;
}

}  // namespace

TEST_CASE("V-carve depth rule: radius / tan(half angle) for V-bits and the ball chord for ball noses",
          "[toolpath][vcarve]") {
    const tooling::Tool v90 = vBit(90.0, 12.7);
    const tooling::Tool v60 = vBit(60.0, 6.35);
    const tooling::Tool ball = ballNose(6.0);
    CHECK(vcarveToolAllowed(tooling::ToolType::VBit));
    CHECK(vcarveToolAllowed(tooling::ToolType::BallNose));
    CHECK_FALSE(vcarveToolAllowed(tooling::ToolType::EndMill));
    CHECK_FALSE(vcarveToolAllowed(tooling::ToolType::Engraving));
    CHECK(vcarveClearanceToolAllowed(tooling::ToolType::EndMill));
    CHECK_FALSE(vcarveClearanceToolAllowed(tooling::ToolType::VBit));

    CHECK(std::fabs(vcarveFullWidthDepth(v90) - 6.35) < 1e-12);
    CHECK(std::fabs(vcarveFullWidthDepth(v60) - 3.175 / std::tan(geometry::kPi / 6.0)) < 1e-12);
    CHECK(vcarveFullWidthDepth(ball) == 3.0);
    CHECK(vcarveFullWidthDepth(endMill(3.0)) == 0.0);
    for (const double r : {0.0, 0.01, 0.5, 1.5, 3.0, 3.1749}) {
        INFO("r = " << r);
        CHECK(std::fabs(vcarveDepthForRadius(v60, r) - r / std::tan(geometry::kPi / 6.0)) < 1e-12);
        CHECK(std::fabs(vcarveDepthForRadius(v90, r) - r) < 1e-12);
        // Inverse of half the cutting width.
        CHECK(std::fabs(0.5 * tooling::cuttingWidthAtDepth(v60, vcarveDepthForRadius(v60, r)) - r) < 1e-12);
        CHECK(std::fabs(0.5 * tooling::cuttingWidthAtDepth(ball, vcarveDepthForRadius(ball, r)) - std::min(r, 3.0)) <
              1e-9);
    }
    CHECK(vcarveDepthForRadius(v60, 10.0) == vcarveFullWidthDepth(v60));  // clamped at the full width
    CHECK(vcarveDepthForRadius(ball, 2.0) == 3.0 - std::sqrt(5.0));
    CHECK(vcarveDepthForRadius(v60, -1.0) == 0.0);
}

TEST_CASE("V-carve of a rectangle follows its skeleton at radius / tan(half angle)", "[toolpath][vcarve]") {
    const std::vector<Contour> rect{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 6}})};
    const tooling::Tool tool = vBit(90.0, 12.7);
    VCarveParams p;
    const VCarveResult r = vcarveToolpath(rect, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    CHECK(r.regions == 1);
    CHECK(r.clearance.empty());
    CHECK(r.toolpaths().size() == 1);
    CHECK(r.toolpath.name == "V-Carve");
    CHECK(r.toolpath.tool.name == "V-bit");
    checkStructure(r.toolpath);
    // The middle line is 3 mm from both long walls: depth (3 - chordal/2) / tan(45 deg).
    const double margin = 0.5 * p.tolerance.chordal;
    CHECK(std::fabs(deepest(r.toolpath) - (3.0 - margin)) < 1e-9);
    bool middle = false;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Feed && std::fabs(m.to.y - 3.0) < 1e-9 && m.to.x > 3.5 && m.to.x < 16.5) {
            middle = true;
            CHECK(std::fabs(m.to.z + (3.0 - margin)) < 1e-9);
        }
    }
    CHECK(middle);
    // The four corner branches reach the corners' neighbourhood: the shallowest cut point of each
    // is within a few tolerances of a corner and at depth ~0.
    std::size_t nearCorners = 0;
    for (const Vec2 corner : {Vec2{0, 0}, Vec2{20, 0}, Vec2{20, 6}, Vec2{0, 6}}) {
        for (const Move& m : r.toolpath.moves) {
            if (geometry::distance(Vec2{m.to.x, m.to.y}, corner) < 0.05 && m.to.z <= 0.0 && m.to.z > -0.05) {
                ++nearCorners;
                break;
            }
        }
    }
    CHECK(nearCorners == 4);
    // Plunges at the plunge rate, cuts at the feed rate (the toolpath's defaults).
    CHECK(r.toolpath.tool.feedRate == 1500.0);
    CHECK(r.toolpath.tool.plungeRate == 500.0);
    CHECK(countKind(r.toolpath, MoveKind::Plunge) == countKind(r.toolpath, MoveKind::Retract));

    const orc::ConeReport cone = orc::coneCheck(r.toolpath, rect, tool, 0.0, kSafeZ, 1e-5);
    CHECK(cone.minClearance >= 0.0);
    CHECK(cone.allInside);
    CHECK(cone.rapidsAtSafeZ);
    // Deterministic.
    CHECK(vcarveToolpath(rect, tool, std::nullopt, {}, p).toolpath == r.toolpath);
}

TEST_CASE("V-carve of a circle is one plunge at its centre", "[toolpath][vcarve]") {
    const std::vector<Contour> circle{Contour::circle({10, 5}, 3.0)};
    const tooling::Tool tool = vBit(60.0, 12.7);
    VCarveParams p;
    const VCarveResult r = vcarveToolpath(circle, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    REQUIRE(r.toolpath.moves.size() == 3);
    CHECK(r.toolpath.moves[0].kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves[0].to.z == kSafeZ);
    CHECK(r.toolpath.moves[1].kind == MoveKind::Plunge);
    CHECK(std::fabs(r.toolpath.moves[1].to.x - 10.0) < 1e-6);
    CHECK(std::fabs(r.toolpath.moves[1].to.y - 5.0) < 1e-6);
    const double expected = (3.0 - 0.5 * p.tolerance.chordal) / std::tan(geometry::kPi / 6.0);
    CHECK(std::fabs(-r.toolpath.moves[1].to.z - expected) < 1e-5);
    CHECK(r.toolpath.moves[2].kind == MoveKind::Retract);
}

TEST_CASE("V-carve with a ball nose: depth from the ball's chord", "[toolpath][vcarve]") {
    const std::vector<Contour> circle{Contour::circle({0, 0}, 2.0)};
    const tooling::Tool ball = ballNose(6.0);
    const VCarveResult r = vcarveToolpath(circle, ball, std::nullopt, {}, VCarveParams{});
    REQUIRE(r.ok());
    REQUIRE(r.toolpath.moves.size() == 3);
    const double s = 2.0 - 0.005;
    CHECK(std::fabs(-r.toolpath.moves[1].to.z - (3.0 - std::sqrt(9.0 - s * s))) < 1e-5);

    // A glyph with a ball nose passes the cone/wall check.
    std::vector<Contour> glyph;
    test::medial::appendGlyph(glyph, 3, {0, 0});
    const VCarveResult g = vcarveToolpath(glyph, ballNose(3.0), std::nullopt, {}, VCarveParams{});
    REQUIRE(g.ok());
    const orc::ConeReport cone = orc::coneCheck(g.toolpath, glyph, ballNose(3.0), 0.0, kSafeZ, 1e-5);
    CHECK(cone.minClearance >= 0.0);
    CHECK(cone.allInside);
}

TEST_CASE("V-carve start depth moves the whole carve down", "[toolpath][vcarve]") {
    std::vector<Contour> glyph;
    test::medial::appendGlyph(glyph, 1, {0, 0});
    const tooling::Tool tool = vBit(60.0, 12.7);
    VCarveParams p;
    const VCarveResult flat = vcarveToolpath(glyph, tool, std::nullopt, {}, p);
    p.startDepth = 1.25;
    const VCarveResult deep = vcarveToolpath(glyph, tool, std::nullopt, {}, p);
    REQUIRE(flat.ok());
    REQUIRE(deep.toolpath.moves.size() == flat.toolpath.moves.size());
    for (std::size_t i = 0; i < flat.toolpath.moves.size(); ++i) {
        const Move& a = flat.toolpath.moves[i];
        const Move& b = deep.toolpath.moves[i];
        CHECK(a.kind == b.kind);
        CHECK(a.to.x == b.to.x);
        CHECK(a.to.y == b.to.y);
        CHECK(b.to.z == (a.to.z == kSafeZ ? kSafeZ : a.to.z - 1.25));
    }
}

TEST_CASE("V-carve flat depth without clearance tools: the V-bit clears the flat area with Pocket",
          "[toolpath][vcarve][flat]") {
    const std::vector<Contour> plate{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{30, 20}}),
                                     polygon({{8, 8}, {12, 8}, {10, 12}})};
    tooling::Tool tool = vBit(90.0, 12.7);
    tool.defaultCuttingData.stepover = 0.8;
    VCarveParams p;
    p.flatDepth = 1.0;
    const VCarveResult r = vcarveToolpath(plate, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    CHECK(r.clearance.empty());
    checkStructure(r.toolpath);
    CHECK(std::fabs(deepest(r.toolpath) - 1.0) < 1e-12);  // the flat depth exactly

    // The flat area is Pocket's own work, move for move (no fill code of its own).
    const PocketResult pocket = pocketToolpath(plate, {PocketTool{tool, std::nullopt}}, flatPocket(p, 1.0, 0.0));
    REQUIRE(pocket.ok());
    const std::vector<Move>& pm = pocket.toolpaths.front().moves;
    REQUIRE(r.toolpath.moves.size() > pm.size());
    CHECK(std::equal(pm.begin(), pm.end(), r.toolpath.moves.begin()));

    const orc::ConeReport cone = orc::coneCheck(r.toolpath, plate, tool, 0.0, kSafeZ, 1e-5);
    CHECK(cone.minClearance >= 0.0);
    CHECK(cone.allInside);
}

TEST_CASE("V-carve flat depth with clearance tools: Pocket with the V-bit's width as allowance",
          "[toolpath][vcarve][flat]") {
    const std::vector<Contour> plate{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{30, 20}}),
                                     polygon({{8, 8}, {12, 8}, {10, 12}})};
    const tooling::Tool tool = vBit(90.0, 12.7, "V-bit 90");
    const std::vector<PocketTool> clear{{endMill(3.0, "EM 3", 4), std::nullopt}, {endMill(1.0, "EM 1", 5), std::nullopt}};
    for (const double start : {0.0, 2.0}) {
        for (const PocketStrategy strategy : {PocketStrategy::Offset, PocketStrategy::Raster}) {
            INFO("start depth " << start << ", strategy " << static_cast<int>(strategy));
            VCarveParams p;
            p.name = "Sign";
            p.startDepth = start;
            p.flatDepth = 1.5;
            p.clearanceStrategy = strategy;
            p.clearanceRasterAngle = 0.3;
            const VCarveResult r = vcarveToolpath(plate, tool, std::nullopt, clear, p);
            REQUIRE(r.ok());
            CHECK(r.diagnostics.empty());
            REQUIRE(r.clearance.size() == 2);
            CHECK(r.clearance[0].name == "Sign [EM 3]");
            CHECK(r.clearance[1].name == "Sign [EM 1]");
            CHECK(r.toolpath.name == "Sign [V-bit 90]");
            CHECK(r.clearance[0].tool.toolNumber == 4);
            const std::vector<Toolpath> all = r.toolpaths();
            REQUIRE(all.size() == 3);
            CHECK(all[0].name == "Sign [EM 3]");
            CHECK(all[2].name == "Sign [V-bit 90]");

            // The clearance is Pocket's, move for move, moved down by the start depth.
            const double allowance = 1.5;  // the 90-degree V-bit's footprint at 1.5 mm
            PocketResult pocket = pocketToolpath(plate, clear, flatPocket(p, 1.5, allowance));
            REQUIRE(pocket.toolpaths.size() == 2);
            for (std::size_t t = 0; t < 2; ++t) {
                std::vector<Move> moved = pocket.toolpaths[t].moves;
                for (Move& m : moved) {
                    m.to.z -= start;
                }
                CHECK(r.clearance[t].moves == moved);
                // Gouge-free by the pocket oracle: the end mill keeps the V-bit's footprint off
                // every wall, at every depth.
                const test::pocket::GougeReport g =
                    test::pocket::gougeCheck(r.clearance[t], polys(plate), -start, kSafeZ);
                CHECK(g.allInside);
                CHECK(g.rapidsAtSafeZ);
                CHECK(g.minDistance >= 0.5 * clear[t].tool.diameter + allowance - 1e-6);
                CHECK(std::fabs(deepest(r.clearance[t]) - (start + 1.5)) < 1e-12);
            }
            // The V-bit cuts the walls of the flat area at the flat depth and the skeleton above it.
            checkStructure(r.toolpath);
            CHECK(std::fabs(deepest(r.toolpath) - (start + 1.5)) < 1e-12);
            const orc::ConeReport cone = orc::coneCheck(r.toolpath, plate, tool, start, kSafeZ, 1e-5);
            CHECK(cone.minClearance >= 0.0);
            CHECK(cone.allInside);
        }
    }
}

TEST_CASE("V-carve limits the depth to the tool's full width and reports it", "[toolpath][vcarve][flat]") {
    const std::vector<Contour> wide{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{30, 20}})};
    const tooling::Tool small = vBit(90.0, 6.0, "Small V");
    VCarveParams p;
    VCarveResult r = vcarveToolpath(wide, small, std::nullopt, {}, p);
    REQUIRE(r.ok());
    CHECK(r.has(ToolpathIssue::CarveDepthLimited));
    CHECK(std::fabs(deepest(r.toolpath) - 3.0) < 1e-12);
    CHECK(orc::coneCheck(r.toolpath, wide, small, 0.0, kSafeZ, 1e-5).minClearance >= 0.0);

    p.flatDepth = 5.0;  // deeper than the 3 mm the bit reaches at its full width
    r = vcarveToolpath(wide, small, std::nullopt, {}, p);
    REQUIRE(r.ok());
    REQUIRE(r.has(ToolpathIssue::CarveDepthLimited));
    CHECK(r.diagnostics.front().message.find("the flat depth 5 mm is deeper") != std::string::npos);
    CHECK(std::fabs(deepest(r.toolpath) - 3.0) < 1e-12);

    // A flat depth that no region reaches changes nothing.
    std::vector<Contour> glyph;
    test::medial::appendGlyph(glyph, 1, {0, 0});
    p.flatDepth = 50.0;
    const VCarveResult a = vcarveToolpath(glyph, vBit(60.0, 12.7), std::nullopt, {}, p);
    p.flatDepth = 0.0;
    const VCarveResult b = vcarveToolpath(glyph, vBit(60.0, 12.7), std::nullopt, {}, p);
    CHECK(a.toolpath == b.toolpath);
    CHECK(b.diagnostics.empty());
}

TEST_CASE("V-carve groups regions and orders them and reports skipped vectors", "[toolpath][vcarve]") {
    std::vector<Contour> page;
    for (int k = 0; k < 5; ++k) {
        test::medial::appendGlyph(page, k, Vec2{7.0 * (4 - k), 0.0});  // right to left
    }
    const tooling::Tool tool = vBit(60.0, 12.7);
    VCarveParams p;
    p.order.leftToRight = true;
    VCarveResult r = vcarveToolpath(page, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    CHECK(r.regions == 5);  // the O, D and their counters stay one region each
    // Left to right: the plunges' x never jumps back by a whole glyph pitch.
    double lastX = -1e9;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Plunge) {
            CHECK(m.to.x > lastX - 6.0);
            lastX = std::max(lastX, m.to.x);
        }
    }
    p.order = OrderOptions{};
    p.order.selectionOrder = true;
    r = vcarveToolpath(page, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    double lastSel = 1e9;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Plunge) {
            CHECK(m.to.x < lastSel + 6.0);  // selection order is right to left here
            lastSel = std::min(lastSel, m.to.x);
        }
    }

    // Open, empty and gapped vectors are skipped and named; the rest is carved.
    Contour open = polygon({{0, 20}, {5, 20}, {5, 25}});
    open.closed = false;
    Contour gapped;
    gapped.closed = true;
    gapped.append(geometry::LineSpan{{10, 20}, {15, 20}});
    gapped.append(geometry::LineSpan{{15, 21}, {10, 25}});
    std::vector<Contour> mixed{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 4}}), open, Contour{}, gapped};
    const VCarveResult m = vcarveToolpath(mixed, tool, std::nullopt, {}, VCarveParams{});
    REQUIRE(m.ok());
    CHECK(m.has(ToolpathIssue::OpenContourIgnored));
    CHECK(m.has(ToolpathIssue::EmptyContour));
    CHECK(m.has(ToolpathIssue::DiscontinuousContour));
    CHECK(m.regions == 1);

    // Nothing usable: an empty result with a diagnostic.
    const VCarveResult none = vcarveToolpath({open}, tool, std::nullopt, {}, VCarveParams{});
    CHECK_FALSE(none.ok());
    CHECK_FALSE(none.diagnostics.empty());
    // A strip narrower than the chordal tolerance: no inscribed circle wider than the margin,
    // nothing to carve.
    const VCarveResult thin = vcarveToolpath({polygon({{0, 0}, {10, 0}, {10, 0.004}, {0, 0.004}})}, tool,
                                             std::nullopt, {}, VCarveParams{});
    CHECK_FALSE(thin.ok());
    CHECK(thin.has(ToolpathIssue::NoRegion));
}

TEST_CASE("V-carve refuses bad tools and parameters", "[toolpath][vcarve][errors]") {
    const std::vector<Contour> rect{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 6}})};
    const tooling::Tool tool = vBit(90.0, 12.7);
    auto refused = [&](const tooling::Tool& t, const std::vector<PocketTool>& clear, const VCarveParams& p,
                       ToolpathIssue issue, const std::string& words) {
        const VCarveResult r = vcarveToolpath(rect, t, std::nullopt, clear, p);
        CHECK_FALSE(r.ok());
        CHECK(r.toolpaths().empty());
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics.front().issue == issue);
        INFO(r.diagnostics.front().message);
        CHECK(r.diagnostics.front().message.find(words) != std::string::npos);
    };
    VCarveParams p;
    refused(endMill(3.0, "EM"), {}, p, ToolpathIssue::InvalidTool, "tool 'EM' is an end mill; V-carving needs");
    refused(tool, {{vBit(60.0, 6.0, "V2"), std::nullopt}}, p, ToolpathIssue::InvalidTool,
            "clearance tool 1 ('V2') is a V-bit; clearance tools must be end mills");
    tooling::Tool noFeed = tool;
    noFeed.defaultCuttingData.feedRate = 0.0;
    refused(noFeed, {}, p, ToolpathIssue::InvalidTool, "feed rate");
    tooling::Tool badAngle = tool;
    badAngle.includedAngle = 0.0;
    refused(badAngle, {}, p, ToolpathIssue::InvalidTool, "tool 'V-bit'");

    VCarveParams bad = p;
    bad.startDepth = -1.0;
    refused(tool, {}, bad, ToolpathIssue::InvalidParameters, "start depth");
    bad = p;
    bad.flatDepth = -0.5;
    refused(tool, {}, bad, ToolpathIssue::InvalidParameters, "flat depth");
    bad = p;
    bad.safeZ = 0.0;
    refused(tool, {}, bad, ToolpathIssue::InvalidParameters, "safe Z");
    bad = p;
    bad.flatDepth = std::numeric_limits<double>::quiet_NaN();
    refused(tool, {}, bad, ToolpathIssue::InvalidParameters, "finite");
    bad = p;
    bad.tolerance.chordal = 0.0;
    refused(tool, {}, bad, ToolpathIssue::InvalidParameters, "tolerance");
    bad = p;
    bad.rampPlunge = true;
    bad.rampDistance = 0.0;
    refused(tool, {}, bad, ToolpathIssue::InvalidParameters, "ramp distance");

    // The V-bit cannot clear a flat area with a stepover wider than it cuts there: Pocket refuses,
    // and so does the V-carve (no half-done flat area).
    tooling::Tool wideStep = tool;
    wideStep.defaultCuttingData.stepover = 5.0;
    VCarveParams flat = p;
    flat.flatDepth = 1.0;
    const VCarveResult r = vcarveToolpath(rect, wideStep, std::nullopt, {}, flat);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::InvalidTool));
    CHECK(r.diagnostics.front().message.find("flat area clearance: ") == 0);
}

TEST_CASE("V-carve refuses to carve when the medial axis is not accurate", "[toolpath][vcarve][errors]") {
    // Coordinates beyond what the default tolerance resolves (ADR-0023: 1e-12 of the largest
    // coordinate must stay below chordal / 2): the medial axis is flagged accuracyCapped.
    const double distant = 1e10;
    const std::vector<Contour> both{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 4}}),
                                    Contour::rectangle(Box2{Vec2{distant, distant}, Vec2{distant + 10, distant + 4}})};
    const tooling::Tool tool = vBit(90.0, 12.7);
    const VCarveResult r = vcarveToolpath(both, tool, std::nullopt, {}, VCarveParams{});
    CHECK_FALSE(r.ok());
    CHECK(r.toolpaths().empty());
    REQUIRE(r.has(ToolpathIssue::MedialAxisCapped));
    CHECK(r.diagnostics.size() == 1);
    CHECK(r.diagnostics.front().contour == std::optional<std::size_t>{1});
    CHECK(r.diagnostics.front().message.find("vector(s) 1 ") != std::string::npos);

    // The region near the origin alone carves.
    CHECK(vcarveToolpath({both[0]}, tool, std::nullopt, {}, VCarveParams{}).ok());
}

TEST_CASE("Medial axis of an isosceles triangle with an obtuse apex is not empty (T-0153)",
          "[geometry][medial][vcarve]") {
    // The base split at its middle under the apex puts both circumcentres exactly on the legs:
    // neither inside nor outside. Before T-0153 the skeleton stayed empty.
    for (const double h : {8.0, 3.0, 1.0}) {
        INFO("height " << h);
        const std::vector<Contour> tri{polygon({{0, 0}, {20, 0}, {10, h}})};
        const geometry::MedialAxis ax = geometry::medialAxis(tri, geometry::Tolerance{});
        CHECK_FALSE(ax.accuracyCapped);
        // Very obtuse apexes lose their branch (ADR-0023); the rest of the skeleton is there.
        REQUIRE_FALSE(ax.branches.empty());
        if (h >= 3.0) {
            CHECK(ax.branches.size() == 3);
        }
        double rMax = 0.0;
        for (const geometry::MedialBranch& b : ax.branches) {
            for (const geometry::MedialPoint& m : b.points) {
                rMax = std::max(rMax, m.radius);
                CHECK(std::fabs(m.radius - orc::boundaryDistance(tri, m.point)) < 1e-9);
            }
        }
        // The incircle radius: area / semi-perimeter.
        const double leg = std::hypot(10.0, h);
        CHECK(std::fabs(rMax - 10.0 * h / (10.0 + leg)) < 0.01);
    }
}

TEST_CASE("AC1: generative sweep - the carving tool never crosses a wall and the depth is within the tolerance",
          "[toolpath][vcarve][ac1][sweep]") {
    const std::uint64_t cases = kRelease ? 800 : 120;
    double worstCone = std::numeric_limits<double>::infinity();
    double worstLateral = 0.0;
    double worstCoverage = 0.0;
    std::size_t carved = 0;
    for (std::uint64_t k = 0; k < cases; ++k) {
        const orc::SweepCase sc = orc::sweepCase(k);
        INFO("case " << k << ": " << sc.what);
        const VCarveParams& p = sc.params;
        const VCarveResult r = vcarveToolpath(sc.region, sc.tool, std::nullopt, sc.clearance, p);
        if (!r.ok()) {
            // Only regions too narrow to carve at the tolerance may come out empty.
            REQUIRE_FALSE(r.diagnostics.empty());
            CHECK(r.has(ToolpathIssue::NoRegion));
            CHECK(sc.what.find("thin strip") == 0);
            continue;
        }
        ++carved;
        const double chordal = p.tolerance.chordal;
        const double full = vcarveFullWidthDepth(sc.tool);
        const double limit = p.flatDepth > 0.0 ? std::min(p.flatDepth, full) : full;
        const double limitRadius = orc::footprintAt(sc.tool, limit);

        // Exact cone/wall check of the carving toolpath.
        const orc::ConeReport cone = orc::coneCheck(r.toolpath, sc.region, sc.tool, p.startDepth, p.safeZ, 1e-5);
        INFO("worst at (" << cone.worstAt.x << ", " << cone.worstAt.y << ", " << cone.worstAt.z << ")");
        CHECK(cone.minClearance >= 0.0);
        CHECK(cone.allInside);
        CHECK(cone.rapidsAtSafeZ);
        worstCone = std::min(worstCone, cone.minClearance);
        CHECK(std::fabs(deepest(r.toolpath) - p.startDepth) <= limit + 1e-12);

        // Clearance tools keep the carving tool's footprint at the flat depth off every wall.
        REQUIRE(r.clearance.size() == sc.clearance.size());
        for (std::size_t t = 0; t < r.clearance.size(); ++t) {
            const orc::ConeReport c =
                orc::coneCheck(r.clearance[t], sc.region, sc.clearance[t].tool, p.startDepth, p.safeZ, 1e-5);
            CHECK(c.minClearance >= limitRadius - 1e-6);
            CHECK(c.allInside);
            CHECK(c.rapidsAtSafeZ);
        }

        // Depth = radius / tan(angle / 2) within the tolerance: at every cut point above the limit
        // the footprint is within the chordal tolerance of the boundary distance (skipped where
        // the carving tool clears flat areas itself: its pocket passes are not skeleton cuts).
        const bool selfClear = sc.clearance.empty() && (p.flatDepth > 0.0 || r.has(ToolpathIssue::CarveDepthLimited));
        if (!selfClear) {
            for (const Move& m : r.toolpath.moves) {
                const double depth = -p.startDepth - m.to.z;
                if (m.kind != MoveKind::Feed || depth >= limit - 1e-9) {
                    continue;
                }
                const double lateral =
                    orc::boundaryDistance(sc.region, {m.to.x, m.to.y}) - orc::footprintAt(sc.tool, depth);
                CHECK(lateral <= chordal);
                worstLateral = std::max(worstLateral, lateral / chordal);
            }
        }
        // Coverage (every third case without flat areas): every inside point is carved to its
        // depth within 1.5 x the chordal tolerance (as a footprint radius).
        if (!selfClear && p.flatDepth == 0.0 && k % 3 == 0) {
            test::SplitMix rng{k * 7 + 1};
            Box2 box;
            for (const Contour& c : sc.region) {
                box.expand(c.bounds());
            }
            int got = 0;
            for (int t = 0; t < 2000 && got < 40; ++t) {
                const Vec2 q{rng.uniform(box.min.x, box.max.x), rng.uniform(box.min.y, box.max.y)};
                if (!orc::insideRegion(sc.region, q)) {
                    continue;
                }
                ++got;
                const double dist = orc::boundaryDistance(sc.region, q);
                const double want = std::min(limit, vcarveDepthForRadius(sc.tool, dist));
                const double c = orc::carvedDepth(r.toolpath, sc.tool, p.startDepth, q, 0.1 * chordal);
                if (c < want) {
                    const double shortfall = dist - orc::footprintAt(sc.tool, c);
                    CHECK(shortfall <= 1.5 * chordal);
                    worstCoverage = std::max(worstCoverage, shortfall / chordal);
                }
            }
        }
    }
    INFO("carved " << carved << " of " << cases << "; worst clearance " << worstCone << " mm, worst lateral "
                   << worstLateral << " x chordal, worst coverage shortfall " << worstCoverage << " x chordal");
    CHECK(carved >= cases * 9 / 10);
    CHECK(worstCone >= 0.0);
}

TEST_CASE("AC2: 5000 glyph-like regions within the documented CPU time", "[toolpath][vcarve][ac2][perf]") {
    // ADR-0025: in a Release build, 5,000 glyph-like regions (45,000 boundary segments) V-carve
    // in at most 60 s of CPU time (measured ~11 s on the T-0153 machine); the cost grows
    // linearly with the number of regions (each region's medial axis is computed on its own).
    // Debug builds run a smaller ladder with the same scaling rule.
    const std::vector<std::size_t> sizes =
        kRelease ? std::vector<std::size_t>{1250, 2500, 5000} : std::vector<std::size_t>{125, 250};
    const tooling::Tool tool = vBit(60.0, 12.7);
    double prev = 0.0;
    std::string log;
    for (std::size_t i = 0; i < sizes.size(); ++i) {
        std::vector<Contour> page;
        std::size_t segments = 0;
        for (std::size_t k = 0; k < sizes[i]; ++k) {
            const Vec2 o{static_cast<double>(k % 100) * 7.0, static_cast<double>(k / 100) * 9.0};
            segments += test::medial::appendGlyph(page, static_cast<int>(k % 5), o);
        }
        const double t0 = test::processCpuSeconds();
        const VCarveResult r = vcarveToolpath(page, tool, std::nullopt, {}, VCarveParams{});
        const double t = test::processCpuSeconds() - t0;
        log += std::to_string(sizes[i]) + " regions, " + std::to_string(segments) + " segments: " +
               std::to_string(t) + " s CPU, " + std::to_string(r.toolpath.moveCount()) + " moves\n";
        INFO(log);
        REQUIRE(r.ok());
        CHECK(r.regions == sizes[i]);
        CHECK(r.diagnostics.empty());
        if (i > 0) {
            CHECK(t <= 2.5 * prev + 0.25);  // + scheduler-tick noise on small runs
        }
        if (kRelease && sizes[i] >= 5000) {
            CHECK(t <= 60.0);
        }
        prev = t;
    }
}
