// T-0236: Chamfer toolpath (chamfer.hpp, ADR-0035): tool-geometry exact depth and offset for a
// V-bit on a 20 x 20 square, type / slope / overcut variants, round nose, depth limit, linked
// width and depth, hostile input.

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
#include "carvetto/toolpath/chamfer.hpp"
#include "carvetto/toolpath/move.hpp"

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

tooling::Tool ballNose(double diameter = 3.0) {
    tooling::Tool t = vBit();
    t.id = "bn";
    t.name = "Ball nose";
    t.type = tooling::ToolType::BallNose;
    t.diameter = diameter;
    t.includedAngle = 0.0;
    return t;
}

Contour square() {
    return Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 20}});
}

/// XY extent and lowest Z of the cutting moves (those below the surface).
struct Cut {
    double minX{1e300}, maxX{-1e300}, minY{1e300}, maxY{-1e300}, minZ{1e300}, maxZ{-1e300};
    int count{0};
};

Cut cutOf(const Toolpath& tp) {
    Cut c;
    for (const Move& m : tp.moves) {
        if (m.kind != MoveKind::Feed && m.kind != MoveKind::ArcCW && m.kind != MoveKind::ArcCCW) {
            continue;
        }
        c.minX = std::min(c.minX, m.to.x);
        c.maxX = std::max(c.maxX, m.to.x);
        c.minY = std::min(c.minY, m.to.y);
        c.maxY = std::max(c.maxY, m.to.y);
        c.minZ = std::min(c.minZ, m.to.z);
        c.maxZ = std::max(c.maxZ, m.to.z);
        ++c.count;
    }
    return c;
}

ChamferParams params(ChamferType type = ChamferType::Inside, ChamferSlope slope = ChamferSlope::Down) {
    ChamferParams p;
    p.type = type;
    p.slope = slope;
    p.width = 2.0;
    return p;
}

}  // namespace

TEST_CASE("chamfer: linked width and cut depth", "[chamfer]") {
    CHECK(chamferCutDepth(geometry::kPi / 4.0, 2.0) == Approx(1.0));
    CHECK(chamferWidth(geometry::kPi / 4.0, 1.0) == Approx(2.0));
    CHECK(chamferCutDepth(geometry::kPi / 6.0, 2.0) == Approx(1.0 / std::tan(geometry::kPi / 6.0)));
    CHECK(chamferWidth(chamferCutDepth(0.4, 3.3) * 0 + 0.4, chamferCutDepth(0.4, 3.3)) == Approx(3.3));
    CHECK(chamferCutDepth(0.0, 2.0) == 0.0);
    CHECK(chamferCutDepth(geometry::kPi / 2.0, 2.0) == 0.0);
    CHECK(chamferCutDepth(0.5, -1.0) == 0.0);
    CHECK(chamferCutDepth(std::nan(""), 1.0) == 0.0);
    CHECK(chamferWidth(0.5, 0.0) == 0.0);
    CHECK(chamferWidth(std::nan(""), 1.0) == 0.0);
    CHECK(chamferWidth(1.6, 1.0) == 0.0);
}

TEST_CASE("chamfer: 90 degree V-bit, width 2 on a 20 x 20 square cuts 1 deep", "[chamfer]") {
    const tooling::Tool tool = vBit();
    const ChamferParams p = params();
    REQUIRE(chamferTipDepth(tool, p).has_value());
    CHECK(*chamferTipDepth(tool, p) == Approx(1.0).margin(1e-12));
    CHECK(chamferMaxCutDepth(tool) == Approx(6.35));

    SECTION("inside, slope down: the tip runs on the vector") {
        const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, p);
        REQUIRE(r.ok());
        const Cut c = cutOf(r.toolpath);
        CHECK(c.minZ == Approx(-1.0).margin(1e-9));
        CHECK(c.maxZ == Approx(-1.0).margin(1e-9));
        CHECK(c.minX == Approx(0.0).margin(1e-6));
        CHECK(c.maxX == Approx(20.0).margin(1e-6));
        CHECK(c.minY == Approx(0.0).margin(1e-6));
        CHECK(c.maxY == Approx(20.0).margin(1e-6));
        CHECK(r.toolpath.tool.toolNumber == 2);
        // Never deeper than the tool can cut, every rapid at safe Z.
        CHECK(-*r.toolpath.minZ() <= chamferMaxCutDepth(tool));
        for (const Move& m : r.toolpath.moves) {
            if (m.kind == MoveKind::Rapid) {
                CHECK(m.to.z == Approx(5.0));
            }
        }
    }
    SECTION("inside, slope up: 1 mm inside the vector") {
        const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, params(ChamferType::Inside, ChamferSlope::Up));
        REQUIRE(r.ok());
        const Cut c = cutOf(r.toolpath);
        CHECK(c.minZ == Approx(-1.0).margin(1e-9));
        CHECK(c.minX == Approx(1.0).margin(1e-6));
        CHECK(c.maxX == Approx(19.0).margin(1e-6));
        CHECK(c.minY == Approx(1.0).margin(1e-6));
        CHECK(c.maxY == Approx(19.0).margin(1e-6));
    }
    SECTION("outside, slope up: 1 mm outside the vector") {
        const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, params(ChamferType::Outside, ChamferSlope::Up));
        REQUIRE(r.ok());
        const Cut c = cutOf(r.toolpath);
        CHECK(c.minX == Approx(-1.0).margin(1e-6));
        CHECK(c.maxX == Approx(21.0).margin(1e-6));
        CHECK(c.minY == Approx(-1.0).margin(1e-6));
        CHECK(c.maxY == Approx(21.0).margin(1e-6));
    }
    SECTION("outside, slope down: on the vector") {
        const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, params(ChamferType::Outside));
        REQUIRE(r.ok());
        const Cut c = cutOf(r.toolpath);
        CHECK(c.minX == Approx(0.0).margin(1e-6));
        CHECK(c.maxX == Approx(20.0).margin(1e-6));
    }
}

TEST_CASE("chamfer: overcut carries the face along its plane", "[chamfer]") {
    const tooling::Tool tool = vBit();
    ChamferParams p = params();
    p.overcut = 0.5;
    SECTION("slope down inside: 0.5 mm away from the face, 0.5 deeper") {
        const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, p);
        REQUIRE(r.ok());
        const Cut c = cutOf(r.toolpath);
        CHECK(c.minZ == Approx(-1.5).margin(1e-9));
        // Face side is inside, the tool moves out of it: 0.5 outside the vector.
        CHECK(c.minX == Approx(-0.5).margin(1e-6));
        CHECK(c.maxX == Approx(20.5).margin(1e-6));
    }
    SECTION("slope up outside: 1.5 out and 1.5 deep") {
        p.type = ChamferType::Outside;
        p.slope = ChamferSlope::Up;
        const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, p);
        REQUIRE(r.ok());
        const Cut c = cutOf(r.toolpath);
        CHECK(c.minZ == Approx(-1.5).margin(1e-9));
        CHECK(c.maxX == Approx(21.5).margin(1e-6));
    }
}

TEST_CASE("chamfer: other angles", "[chamfer]") {
    const tooling::Tool tool = vBit(60.0);  // half angle 30 degrees
    const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, params());
    REQUIRE(r.ok());
    CHECK(cutOf(r.toolpath).minZ == Approx(-1.0 / std::tan(geometry::kPi / 6.0)).margin(1e-9));
    // Params angle is ignored for a V-bit.
    ChamferParams p = params();
    p.angle = 0.1;
    CHECK(*chamferTipDepth(tool, p) == Approx(1.0 / std::tan(geometry::kPi / 6.0)));
}

TEST_CASE("chamfer: the depth never exceeds the tool's maximum cut depth", "[chamfer]") {
    const tooling::Tool tool = vBit();  // cone height 6.35
    ChamferParams p = params();
    p.width = 12.7;  // C = 6.35: at the limit
    CHECK(chamferToolpath({square()}, tool, std::nullopt, p).ok());
    p.width = 12.8;
    ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, p);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::InvalidParameters));
    p.width = 12.0;
    p.overcut = 1.0;  // 6 + 1 > 6.35
    r = chamferToolpath({square()}, tool, std::nullopt, p);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::InvalidParameters));
    CHECK_FALSE(chamferTipDepth(tool, p).has_value());
}

TEST_CASE("chamfer: round-nosed tool", "[chamfer]") {
    const tooling::Tool tool = ballNose(3.0);  // R = 1.5
    ChamferParams p = params();
    p.angle = geometry::kPi / 4.0;
    p.width = 3.0;  // C = 1.5
    const double s = std::sin(p.angle);
    const double expected = 1.5 - 1.5 * (1.0 / s - 1.0);
    REQUIRE(chamferTipDepth(tool, p).has_value());
    CHECK(*chamferTipDepth(tool, p) == Approx(expected));
    const ChamferResult r = chamferToolpath({square()}, tool, std::nullopt, p);
    CHECK(r.ok());
    CHECK(-*r.toolpath.minZ() <= chamferMaxCutDepth(tool));
    CHECK(chamferMaxCutDepth(tool) == Approx(1.5));

    SECTION("angle required") {
        p.angle = 0.0;
        CHECK(chamferToolpath({square()}, tool, std::nullopt, p).has(ToolpathIssue::InvalidParameters));
    }
    SECTION("too shallow for the ball to form the face") {
        p.width = 1.0;
        CHECK(chamferToolpath({square()}, tool, std::nullopt, p).has(ToolpathIssue::InvalidParameters));
    }
    SECTION("deeper than the ball") {
        p.width = 20.0;
        CHECK(chamferToolpath({square()}, tool, std::nullopt, p).has(ToolpathIssue::InvalidParameters));
    }
}

TEST_CASE("chamfer: cut direction and open vectors", "[chamfer]") {
    const tooling::Tool tool = vBit();
    ChamferParams p = params();
    const ChamferResult climb = chamferToolpath({square()}, tool, std::nullopt, p);
    p.direction = CutDirection::Conventional;
    const ChamferResult conv = chamferToolpath({square()}, tool, std::nullopt, p);
    REQUIRE(climb.ok());
    REQUIRE(conv.ok());
    CHECK_FALSE(climb.toolpath.moves == conv.toolpath.moves);
    CHECK(cutOf(climb.toolpath).count == cutOf(conv.toolpath).count);

    Contour line;
    line.append(LineSpan{{0, 0}, {10, 0}});
    p = params(ChamferType::Outside, ChamferSlope::Up);  // right-hand side of travel: y < 0
    const ChamferResult open = chamferToolpath({line}, tool, std::nullopt, p);
    REQUIRE(open.ok());
    const Cut c = cutOf(open.toolpath);
    CHECK(c.maxY == Approx(-1.0).margin(1e-6));
    CHECK(c.minY == Approx(-1.0).margin(1e-6));
}

TEST_CASE("chamfer: unusable tools and hostile parameters", "[chamfer]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    tooling::Tool endMill = vBit();
    endMill.type = tooling::ToolType::EndMill;
    ChamferResult r = chamferToolpath({square()}, endMill, std::nullopt, params());
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::InvalidTool));
    CHECK_FALSE(chamferToolAllowed(tooling::ToolType::EndMill));
    CHECK(chamferToolAllowed(tooling::ToolType::BallNose));
    CHECK(chamferMaxCutDepth(endMill) == 0.0);

    tooling::Tool bad = vBit();
    bad.includedAngle = 0.0;
    CHECK(chamferToolpath({square()}, bad, std::nullopt, params()).has(ToolpathIssue::InvalidParameters));

    const tooling::Tool tool = vBit();
    auto invalid = [&](auto mutate) {
        ChamferParams p = params();
        mutate(p);
        const ChamferResult res = chamferToolpath({square()}, tool, std::nullopt, p);
        CHECK_FALSE(res.ok());
        CHECK(res.has(ToolpathIssue::InvalidParameters));
        CHECK_FALSE(chamferTipDepth(tool, p).has_value());
    };
    invalid([&](ChamferParams& p) { p.width = nan; });
    invalid([&](ChamferParams& p) { p.width = inf; });
    invalid([&](ChamferParams& p) { p.width = 0.0; });
    invalid([&](ChamferParams& p) { p.width = -1.0; });
    invalid([&](ChamferParams& p) { p.width = 1e-9; });
    invalid([&](ChamferParams& p) { p.width = 1e9; });
    invalid([&](ChamferParams& p) { p.overcut = -0.1; });
    invalid([&](ChamferParams& p) { p.overcut = nan; });
    invalid([&](ChamferParams& p) { p.overcut = 1e9; });
    invalid([&](ChamferParams& p) { p.safeZ = 0.0; });
    invalid([&](ChamferParams& p) { p.safeZ = inf; });
    invalid([&](ChamferParams& p) { p.angle = nan; });
    invalid([&](ChamferParams& p) { p.startPosition.x = inf; });
    invalid([&](ChamferParams& p) { p.tolerance.chordal = nan; });

    CHECK_FALSE(chamferToolpath({}, tool, std::nullopt, params()).ok());
    CHECK_FALSE(chamferToolpath({}, tool, std::nullopt, params()).diagnostics.empty());
}
