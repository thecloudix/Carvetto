// T-0240: Prism carving toolpath (prism.hpp, ADR-0042): peak depth, facet geometry, full prism,
// tool limit, holes, direction, hostile input.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/prism.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

tooling::Tool vBit(double angleDeg = 90.0, double diameter = 12.7) {
    tooling::Tool t;
    t.id = "vbit";
    t.name = "V-bit";
    t.type = tooling::ToolType::VBit;
    t.diameter = diameter;
    t.includedAngle = angleDeg * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = 2;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 16000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 0.5;
    return t;
}

Contour square(double side) {
    return Contour::rectangle(Box2{Vec2{0, 0}, Vec2{side, side}});
}

bool isCut(const Move& m) {
    return m.kind == MoveKind::Feed || m.kind == MoveKind::ArcCW || m.kind == MoveKind::ArcCCW;
}

double lowestCutZ(const Toolpath& tp) {
    double z = 1e300;
    for (const Move& m : tp.moves) {
        if (isCut(m)) {
            z = std::min(z, m.to.z);
        }
    }
    return z;
}

/// Every cut lies on the 45 degree facet: its depth equals its distance from the square's edge.
void checkFacet(const Toolpath& tp, double side) {
    for (const Move& m : tp.moves) {
        if (isCut(m)) {
            const double edge = std::min(std::min(m.to.x, side - m.to.x), std::min(m.to.y, side - m.to.y));
            CHECK(-m.to.z == Approx(edge).margin(1e-5));
        }
    }
}

PrismParams params(double depth) {
    PrismParams p;
    p.depth = depth;
    p.stepover = 0.5;
    return p;
}

}  // namespace

TEST_CASE("prism: peak depth equals the stated depth on a symmetric shape", "[prism]") {
    const PrismResult r = prismToolpath({square(20.0)}, vBit(), std::nullopt, params(3.0));
    REQUIRE(r.ok());
    CHECK(lowestCutZ(r.toolpath) == Approx(-3.0).margin(1e-9));
    CHECK(r.reachedDepth == Approx(3.0).margin(1e-9));
    CHECK(*r.toolpath.maxZ() == Approx(5.0));  // rapids at safe Z, cuts all below the surface
    CHECK(r.toolpath.tool.toolNumber == 2);
    checkFacet(r.toolpath, 20.0);
    // The deepest cut is the ring 3 mm in from the vector (tool centre), nothing nearer the middle.
    for (const Move& m : r.toolpath.moves) {
        if (isCut(m)) {
            CHECK(m.to.x >= -1e-6);
            CHECK(m.to.x <= 20.0 + 1e-6);
            CHECK(std::min(std::min(m.to.x, 20.0 - m.to.x), std::min(m.to.y, 20.0 - m.to.y)) <= 3.0 + 1e-6);
        }
    }
}

TEST_CASE("prism: the facet slope follows the V angle", "[prism]") {
    // 60 degree V: half angle 30, so at depth z the tool is z tan 30 from the edge.
    const PrismResult r = prismToolpath({square(20.0)}, vBit(60.0), std::nullopt, params(4.0));
    REQUIRE(r.ok());
    CHECK(lowestCutZ(r.toolpath) == Approx(-4.0).margin(1e-9));
    const double t = std::tan(geometry::kPi / 6.0);
    for (const Move& m : r.toolpath.moves) {
        if (isCut(m)) {
            const double edge = std::min(std::min(m.to.x, 20.0 - m.to.x), std::min(m.to.y, 20.0 - m.to.y));
            CHECK(edge == Approx(-m.to.z * t).margin(1e-5));
        }
    }
}

TEST_CASE("prism: full prism reaches the tool-limited depth, or the shape's peak", "[prism]") {
    const tooling::Tool tool = vBit();
    REQUIRE(prismMaxDepth(tool) == Approx(6.35));
    SECTION("a shape wider than the tool's cone: the tool limit") {
        PrismParams p = params(1.0);  // the depth is ignored
        p.fullPrism = true;
        const PrismResult r = prismToolpath({square(40.0)}, tool, std::nullopt, p);
        REQUIRE(r.ok());
        CHECK(lowestCutZ(r.toolpath) == Approx(-6.35).margin(1e-9));
        CHECK_FALSE(r.has(ToolpathIssue::CarveDepthLimited));
    }
    SECTION("a small shape: the facets meet at the inradius") {
        PrismParams p = params(1.0);
        p.fullPrism = true;
        const PrismResult r = prismToolpath({square(10.0)}, tool, std::nullopt, p);
        REQUIRE(r.ok());
        CHECK(r.reachedDepth == Approx(5.0).margin(2e-3));
        CHECK(lowestCutZ(r.toolpath) == Approx(-5.0).margin(2e-3));
        CHECK(lowestCutZ(r.toolpath) >= -5.0);
        checkFacet(r.toolpath, 10.0);
    }
    SECTION("a depth beyond the shape without full prism stops at the peak too") {
        const PrismResult r = prismToolpath({square(10.0)}, tool, std::nullopt, params(6.0));
        REQUIRE(r.ok());
        CHECK(r.reachedDepth == Approx(5.0).margin(2e-3));
    }
}

TEST_CASE("prism: a depth beyond the tool is limited", "[prism]") {
    const PrismResult r = prismToolpath({square(40.0)}, vBit(), std::nullopt, params(9.0));
    REQUIRE(r.ok());
    CHECK(r.has(ToolpathIssue::CarveDepthLimited));
    CHECK(lowestCutZ(r.toolpath) == Approx(-6.35).margin(1e-9));
}

TEST_CASE("prism: holes stay raised and direction reverses the cuts", "[prism]") {
    std::vector<Contour> v{square(40.0), Contour::rectangle(Box2{Vec2{15, 15}, Vec2{25, 25}})};
    const PrismResult r = prismToolpath(v, vBit(), std::nullopt, params(2.0));
    REQUIRE(r.ok());
    for (const Move& m : r.toolpath.moves) {
        if (isCut(m)) {
            // Never inside the hole's own facets' tool-free zone: depth is the distance to the nearer edge.
            const double outer = std::min(std::min(m.to.x, 40.0 - m.to.x), std::min(m.to.y, 40.0 - m.to.y));
            const double dx = std::max(std::max(15.0 - m.to.x, m.to.x - 25.0), 0.0);
            const double dy = std::max(std::max(15.0 - m.to.y, m.to.y - 25.0), 0.0);
            const double hole = std::hypot(dx, dy);
            CHECK(-m.to.z <= std::min(outer, hole) + 1e-4);
        }
    }
    CHECK(lowestCutZ(r.toolpath) == Approx(-2.0).margin(1e-9));

    PrismParams a = params(2.0);
    PrismParams b = a;
    b.direction = CutDirection::Conventional;
    const PrismResult ra = prismToolpath({square(20.0)}, vBit(), std::nullopt, a);
    const PrismResult rb = prismToolpath({square(20.0)}, vBit(), std::nullopt, b);
    REQUIRE(ra.ok());
    REQUIRE(rb.ok());
    CHECK_FALSE(ra.toolpath == rb.toolpath);
    CHECK(ra.toolpath.moveCount() == rb.toolpath.moveCount());
}

TEST_CASE("prism: deterministic, open vectors ignored", "[prism]") {
    Contour open;
    open.closed = false;
    open.spans.push_back(LineSpan{Vec2{0, 0}, Vec2{5, 5}});
    const PrismResult a = prismToolpath({square(20.0), open}, vBit(), std::nullopt, params(2.0));
    const PrismResult b = prismToolpath({square(20.0), open}, vBit(), std::nullopt, params(2.0));
    REQUIRE(a.ok());
    CHECK(a.toolpath == b.toolpath);
    CHECK(a.has(ToolpathIssue::OpenContourIgnored));
}

TEST_CASE("prism: hostile input", "[prism]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<Contour> sq{square(20.0)};

    auto refused = [&](const std::vector<Contour>& c, const tooling::Tool& t, const PrismParams& p, ToolpathIssue why) {
        const PrismResult r = prismToolpath(c, t, std::nullopt, p);
        CHECK_FALSE(r.ok());
        CHECK(r.has(why));
        CHECK(r.reachedDepth == 0.0);
    };
    for (const double bad : {nan, inf, -inf, 0.0, -1.0}) {
        PrismParams p = params(2.0);
        p.depth = bad;
        refused(sq, vBit(), p, ToolpathIssue::InvalidParameters);
    }
    for (const double bad : {nan, inf, 0.0, -1.0, 1e-9, 101.0}) {
        PrismParams p = params(2.0);
        p.stepover = bad;
        refused(sq, vBit(), p, ToolpathIssue::InvalidParameters);
    }
    for (const double bad : {nan, inf, 0.0, -2.0}) {
        PrismParams p = params(2.0);
        p.safeZ = bad;
        refused(sq, vBit(), p, ToolpathIssue::InvalidParameters);
    }
    {
        PrismParams p = params(2.0);
        p.startPosition = Vec2{nan, 0.0};
        refused(sq, vBit(), p, ToolpathIssue::InvalidParameters);
    }
    {
        PrismParams p = params(nan);  // full prism ignores the depth, but not a non-finite one
        p.fullPrism = true;
        refused(sq, vBit(), p, ToolpathIssue::InvalidParameters);
    }
    {
        tooling::Tool endMill = vBit();
        endMill.type = tooling::ToolType::EndMill;
        refused(sq, endMill, params(2.0), ToolpathIssue::InvalidTool);
        tooling::Tool bad = vBit();
        bad.includedAngle = nan;
        refused(sq, bad, params(2.0), ToolpathIssue::InvalidParameters);
    }
    refused({}, vBit(), params(2.0), ToolpathIssue::NoRegion);
    {
        Contour open;
        open.closed = false;
        open.spans.push_back(LineSpan{Vec2{0, 0}, Vec2{5, 5}});
        refused({open}, vBit(), params(2.0), ToolpathIssue::NoRegion);
        refused({Contour{}}, vBit(), params(2.0), ToolpathIssue::NoRegion);
    }
    {
        // A sliver too narrow to carve.
        refused({Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20.0, 1e-7}})}, vBit(), params(2.0), ToolpathIssue::ToolTooLarge);
    }
    {
        // More cuts than the limit allows.
        PrismParams p = params(6.0);
        p.stepover = 0.001;
        refused({square(40.0)}, vBit(), p, ToolpathIssue::WorkLimitExceeded);
    }
    {
        // Non-finite coordinates do not crash or produce moves.
        Contour c = square(20.0);
        c.spans[0] = LineSpan{Vec2{0, 0}, Vec2{nan, 0}};
        const PrismResult r = prismToolpath({c}, vBit(), std::nullopt, params(2.0));
        for (const Move& m : r.toolpath.moves) {
            CHECK(std::isfinite(m.to.x));
            CHECK(std::isfinite(m.to.y));
            CHECK(std::isfinite(m.to.z));
        }
    }
}
