#include "carvetto/tooling/tool.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

using namespace carvetto::tooling;
using carvetto::geometry::kPi;
using carvetto::geometry::Vec2;
using Catch::Matchers::WithinAbs;

namespace {
Tool makeTool(ToolType type, double diameter) {
    Tool t;
    t.id = "t";
    t.name = "tool";
    t.type = type;
    t.diameter = diameter;
    return t;
}
}  // namespace

// --------------------------------------------------------------- ToolType

TEST_CASE("ToolType names round-trip", "[tooling][tool]") {
    for (ToolType type : allToolTypes()) {
        const auto parsed = parseToolType(toString(type));
        REQUIRE(parsed.has_value());
        CHECK(*parsed == type);
    }
    CHECK(allToolTypes().size() == 9);
    CHECK(toString(ToolType::VBit) == "v_bit");
    CHECK_FALSE(parseToolType("laser").has_value());
    CHECK_FALSE(parseToolType("").has_value());
    CHECK_FALSE(parseToolType("V_Bit").has_value());
}

// ---------------------------------------------------------- cutting width

TEST_CASE("V-bit 90 deg diameter 12.7 cuts 6 mm wide at depth 3 (AC1)", "[tooling][tool]") {
    Tool t = makeTool(ToolType::VBit, 12.7);
    t.includedAngle = kPi / 2.0;
    CHECK_THAT(cuttingWidthAtDepth(t, 3.0), WithinAbs(6.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 0.0), WithinAbs(0.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, -1.0), WithinAbs(0.0, 1e-12));
    // Capped at the full diameter once the V is exhausted (12.7 / 2 = 6.35 deep).
    CHECK_THAT(cuttingWidthAtDepth(t, 6.35), WithinAbs(12.7, 1e-9));
    CHECK_THAT(cuttingWidthAtDepth(t, 20.0), WithinAbs(12.7, 1e-12));
}

TEST_CASE("V-bit 60 deg follows 2*d*tan(a/2)", "[tooling][tool]") {
    Tool t = makeTool(ToolType::VBit, 6.0);
    t.includedAngle = kPi / 3.0;
    CHECK_THAT(cuttingWidthAtDepth(t, 1.0), WithinAbs(2.0 * std::tan(kPi / 6.0), 1e-12));
}

TEST_CASE("End mill and diamond drag widths are constant", "[tooling][tool]") {
    for (ToolType type : {ToolType::EndMill, ToolType::DiamondDrag}) {
        const Tool t = makeTool(type, 6.35);
        CHECK(cuttingWidthAtDepth(t, 0.0) == 6.35);
        CHECK(cuttingWidthAtDepth(t, 2.0) == 6.35);
        CHECK(cuttingWidthAtDepth(t, 100.0) == 6.35);
    }
}

TEST_CASE("Ball nose width is the sphere chord", "[tooling][tool]") {
    const Tool t = makeTool(ToolType::BallNose, 10.0);  // r = 5
    CHECK_THAT(cuttingWidthAtDepth(t, 0.0), WithinAbs(0.0, 1e-12));
    // chord at depth d: 2*sqrt(2*r*d - d^2); d = 1 -> 2*sqrt(9) = 6
    CHECK_THAT(cuttingWidthAtDepth(t, 1.0), WithinAbs(6.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 5.0), WithinAbs(10.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 7.0), WithinAbs(10.0, 1e-12));
}

TEST_CASE("Tapered ball nose follows ball then tangent taper", "[tooling][tool]") {
    Tool t = makeTool(ToolType::TaperedBallNose, 6.0);
    t.tipDiameter = 2.0;          // r = 1
    t.includedAngle = kPi / 3.0;  // half angle 30 deg
    const double r = 1.0;
    const double a = kPi / 6.0;
    const double h0 = r * (1.0 - std::sin(a));  // 0.5
    CHECK_THAT(h0, WithinAbs(0.5, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 0.0), WithinAbs(0.0, 1e-12));
    // Inside the ball region.
    CHECK_THAT(cuttingWidthAtDepth(t, 0.25), WithinAbs(2.0 * std::sqrt(2.0 * r * 0.25 - 0.0625), 1e-12));
    // At the tangent point both formulas agree: 2 r cos a.
    CHECK_THAT(cuttingWidthAtDepth(t, h0), WithinAbs(2.0 * r * std::cos(a), 1e-12));
    // Above it: linear taper.
    const double d = 2.0;
    CHECK_THAT(cuttingWidthAtDepth(t, d),
               WithinAbs(2.0 * r * std::cos(a) + 2.0 * (d - h0) * std::tan(a), 1e-12));
    // Capped at the shank diameter.
    CHECK_THAT(cuttingWidthAtDepth(t, 100.0), WithinAbs(6.0, 1e-12));
}

TEST_CASE("Tapered ball nose with zero taper is a cylinder of the tip diameter", "[tooling][tool]") {
    Tool t = makeTool(ToolType::TaperedBallNose, 6.0);
    t.tipDiameter = 3.0;
    t.includedAngle = 0.0;
    CHECK_THAT(cuttingWidthAtDepth(t, 1.5), WithinAbs(3.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 50.0), WithinAbs(3.0, 1e-12));
}

TEST_CASE("Engraving cutter adds its flat tip to the V", "[tooling][tool]") {
    Tool t = makeTool(ToolType::Engraving, 6.0);
    t.includedAngle = kPi / 2.0;
    t.tipDiameter = 0.2;
    CHECK_THAT(cuttingWidthAtDepth(t, 0.0), WithinAbs(0.2, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 1.0), WithinAbs(2.2, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 10.0), WithinAbs(6.0, 1e-12));
}

TEST_CASE("Radiused end mill is flat core plus corner chord", "[tooling][tool]") {
    Tool t = makeTool(ToolType::RadiusedEndMill, 10.0);
    t.cornerRadius = 2.0;
    CHECK_THAT(cuttingWidthAtDepth(t, 0.0), WithinAbs(6.0, 1e-12));
    // d = 1: core 6 + 2*sqrt(2*2*1 - 1) = 6 + 2*sqrt(3)
    CHECK_THAT(cuttingWidthAtDepth(t, 1.0), WithinAbs(6.0 + 2.0 * std::sqrt(3.0), 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 2.0), WithinAbs(10.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 9.0), WithinAbs(10.0, 1e-12));
}

TEST_CASE("Radiused end mill with corner radius = D/2 behaves like a ball nose", "[tooling][tool]") {
    Tool r = makeTool(ToolType::RadiusedEndMill, 10.0);
    r.cornerRadius = 5.0;
    const Tool b = makeTool(ToolType::BallNose, 10.0);
    for (double d : {0.0, 0.5, 2.0, 5.0, 8.0}) {
        CHECK_THAT(cuttingWidthAtDepth(r, d), WithinAbs(cuttingWidthAtDepth(b, d), 1e-12));
    }
}

TEST_CASE("Form tool width interpolates its half profile", "[tooling][tool]") {
    Tool t = makeTool(ToolType::FormTool, 0.0);
    // Ogee-ish: radius 1 at the tip, 3 at height 2, 3 up to height 4, 5 at height 6.
    t.profile = {{1.0, 0.0}, {3.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};
    CHECK_THAT(cuttingWidthAtDepth(t, 0.0), WithinAbs(2.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 1.0), WithinAbs(4.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 2.0), WithinAbs(6.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 3.0), WithinAbs(6.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 5.0), WithinAbs(8.0, 1e-12));
    // Beyond the profile: last radius.
    CHECK_THAT(cuttingWidthAtDepth(t, 50.0), WithinAbs(10.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, -1.0), WithinAbs(2.0, 1e-12));
    CHECK(cuttingWidthAtDepth(makeTool(ToolType::FormTool, 0.0), 1.0) == 0.0);
}

TEST_CASE("Drill follows the point angle until full diameter", "[tooling][tool]") {
    Tool t = makeTool(ToolType::Drill, 6.0);
    t.includedAngle = kPi / 2.0;  // 90 deg point for easy numbers
    CHECK_THAT(cuttingWidthAtDepth(t, 0.0), WithinAbs(0.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 1.0), WithinAbs(2.0, 1e-12));
    CHECK_THAT(cuttingWidthAtDepth(t, 3.0), WithinAbs(6.0, 1e-9));
    CHECK_THAT(cuttingWidthAtDepth(t, 30.0), WithinAbs(6.0, 1e-12));
}

TEST_CASE("Cutting width never exceeds the diameter for any type", "[tooling][tool]") {
    for (ToolType type : allToolTypes()) {
        if (type == ToolType::FormTool) continue;
        Tool t = makeTool(type, 8.0);
        t.includedAngle = kPi * 0.75;
        t.tipDiameter = 1.0;
        t.cornerRadius = 1.0;
        for (double d = 0.0; d <= 40.0; d += 0.37) {
            const double w = cuttingWidthAtDepth(t, d);
            CHECK(w >= 0.0);
            CHECK(w <= 8.0 + 1e-12);
        }
    }
}

// ------------------------------------------------------------ cutting data

TEST_CASE("Cutting data lookup falls back to the default and says so (AC4)", "[tooling][tool]") {
    Tool t = makeTool(ToolType::EndMill, 6.0);
    t.defaultCuttingData.feedRate = 1000.0;
    CuttingData oakOnShapeoko;
    oakOnShapeoko.feedRate = 2500.0;
    t.setCuttingData("shapeoko", "oak", oakOnShapeoko);

    const CuttingDataLookup hit = t.cuttingDataFor("shapeoko", "oak");
    REQUIRE(hit.data != nullptr);
    CHECK_FALSE(hit.isFallback);
    CHECK(hit.data->feedRate == 2500.0);

    const CuttingDataLookup miss = t.cuttingDataFor("shapeoko", "mdf");
    REQUIRE(miss.data != nullptr);
    CHECK(miss.isFallback);
    CHECK(miss.data->feedRate == 1000.0);
    CHECK(miss.data == &t.defaultCuttingData);

    const CuttingDataLookup otherMachine = t.cuttingDataFor("nomad", "oak");
    CHECK(otherMachine.isFallback);

    CHECK(t.findCuttingData("shapeoko", "mdf") == nullptr);
    CHECK(t.findCuttingData("shapeoko", "oak") != nullptr);
}

TEST_CASE("Cutting data set/replace/remove", "[tooling][tool]") {
    Tool t = makeTool(ToolType::EndMill, 6.0);
    CuttingData a;
    a.passDepth = 1.0;
    CuttingData b;
    b.passDepth = 2.0;
    t.setCuttingData("m", "x", a);
    t.setCuttingData("m", "x", b);
    CHECK(t.cuttingData.size() == 1);
    CHECK(t.findCuttingData("m", "x")->passDepth == 2.0);
    CHECK(t.removeCuttingData("m", "x"));
    CHECK_FALSE(t.removeCuttingData("m", "x"));
    CHECK(t.cuttingData.empty());
}

TEST_CASE("Effective stepovers and chip load", "[tooling][tool]") {
    CuttingData d;
    d.stepover = 2.0;
    CHECK(d.effectiveFinalPassStepover() == 2.0);
    CHECK(d.effectiveClearancePassStepover() == 2.0);
    d.finalPassStepover = 0.5;
    d.clearancePassStepover = 3.0;
    CHECK(d.effectiveFinalPassStepover() == 0.5);
    CHECK(d.effectiveClearancePassStepover() == 3.0);

    d.feedRate = 2000.0;
    d.spindleSpeed = 10000.0;
    CHECK_THAT(chipLoad(d, 2), WithinAbs(0.1, 1e-12));
    CHECK(chipLoad(d, 0) == 0.0);
    d.spindleSpeed = 0.0;
    CHECK(chipLoad(d, 2) == 0.0);
}

// --------------------------------------------------------------- validate

TEST_CASE("validate accepts well-formed tools of every type", "[tooling][tool]") {
    Tool t = makeTool(ToolType::EndMill, 6.0);
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::BallNose;
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::DiamondDrag;
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::VBit;
    t.includedAngle = kPi / 2.0;
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::Drill;
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::Engraving;
    t.tipDiameter = 0.1;
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::TaperedBallNose;
    t.tipDiameter = 1.0;
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::RadiusedEndMill;
    t.cornerRadius = 1.0;
    CHECK_FALSE(validate(t).has_value());
    t.type = ToolType::FormTool;
    t.profile = {{1.0, 0.0}, {2.0, 1.0}};
    CHECK_FALSE(validate(t).has_value());
}

TEST_CASE("validate rejects bad geometry", "[tooling][tool]") {
    Tool t = makeTool(ToolType::EndMill, 0.0);
    CHECK(validate(t).has_value());
    t.diameter = 6.0;
    t.flutes = 0;
    CHECK(validate(t).has_value());
    t.flutes = 2;

    t.type = ToolType::VBit;
    t.includedAngle = 0.0;
    CHECK(validate(t).has_value());
    t.includedAngle = kPi;
    CHECK(validate(t).has_value());
    t.includedAngle = kPi / 2.0;

    t.type = ToolType::Engraving;
    t.tipDiameter = 6.0;
    CHECK(validate(t).has_value());
    t.tipDiameter = -0.1;
    CHECK(validate(t).has_value());

    t.type = ToolType::TaperedBallNose;
    t.tipDiameter = 0.0;
    CHECK(validate(t).has_value());
    t.tipDiameter = 7.0;
    CHECK(validate(t).has_value());

    t.type = ToolType::RadiusedEndMill;
    t.cornerRadius = 0.0;
    CHECK(validate(t).has_value());
    t.cornerRadius = 3.5;
    CHECK(validate(t).has_value());

    t.type = ToolType::FormTool;
    t.profile = {{1.0, 0.0}};
    CHECK(validate(t).has_value());
    t.profile = {{1.0, 0.5}, {2.0, 1.0}};
    CHECK(validate(t).has_value());
    t.profile = {{1.0, 0.0}, {2.0, 0.0}};
    CHECK(validate(t).has_value());
    t.profile = {{-1.0, 0.0}, {2.0, 1.0}};
    CHECK(validate(t).has_value());
    t.profile = {{0.0, 0.0}, {0.0, 1.0}};
    CHECK(validate(t).has_value());
}
