// T-0323: the post's JET_TOOL_POWER / JET_TOOL_ON / JET_TOOL_OFF blocks act (ADR-0099): the beam is switched
// from each move's power, a laser toolpath writes no spindle start and accepts spindle speed 0, and the
// refusals (no jet blocks, power on a non-laser toolpath, bad power).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/laser.hpp"
#include "carvetto/toolpath/toolpath.hpp"

using namespace carvetto;
using post::PostContext;
using post::PostProcessor;
using toolpath::Move;
using toolpath::Toolpath;
using Catch::Matchers::ContainsSubstring;

namespace {

const char* kLaserPost = R"PP(POST_NAME = "Test laser"
FILE_EXTENSION = "nc"
UNITS = "MM"
LINE_ENDING = "[10]"
LASER_SUPPORT = "YES"
VAR POWER = [JET_POWER|A| S|1.0|10]
VAR X_POSITION = [X|C| X|1.3]
VAR Y_POSITION = [Y|C| Y|1.3]
VAR Z_POSITION = [Z|C| Z|1.3]
VAR FEED_RATE = [F|C| F|1.0]
VAR SPINDLE_SPEED = [S|A| S|1.0]

begin HEADER
"G21 (header)"
"M3[S]"
end
begin SPINDLE_ON
"M3[S] (spindle on)"
end
begin TOOLCHANGE
"(toolchange [TOOLNAME])"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G1[X][Y][Z][F]"
end
begin JET_TOOL_POWER
"(power[JET_POWER])"
end
begin JET_TOOL_ON
"M4"
end
begin JET_TOOL_OFF
"M5"
end
begin FOOTER
"M2"
end
)PP";

PostProcessor parsePost(const std::string& text) {
    auto p = PostProcessor::parse(text);
    REQUIRE(p.ok());
    return p.value();
}

toolpath::ToolSnapshot laserTool(const std::string& name = "Laser") {
    toolpath::ToolSnapshot t;
    t.name = name;
    t.toolNumber = 1;
    t.diameter = 0.2;
    t.feedRate = 800.0;
    t.plungeRate = 800.0;
    t.spindleSpeed = 0.0;
    t.laser = true;
    return t;
}

Toolpath laserPath(std::vector<Move> moves) {
    Toolpath tp;
    tp.name = "Laser";
    tp.tool = laserTool();
    tp.moves = std::move(moves);
    return tp;
}

std::string text(const PostProcessor& p, const std::vector<Toolpath>& tps) {
    auto out = p.run(tps, PostContext{});
    REQUIRE(out.ok());
    return out.value().text;
}

}  // namespace

TEST_CASE("T-0323: the beam is switched on and off around the cutting moves, with the power set first",
          "[post][laser]") {
    const PostProcessor p = parsePost(kLaserPost);
    const Toolpath tp = laserPath({Move::rapid({0, 0, 0}), Move::feed({10, 0, 0}).withJet(50.0),
                                   Move::feed({10, 10, 0}).withJet(50.0), Move::feed({20, 10, 0}),
                                   Move::feed({20, 20, 0}).withJet(75.0), Move::rapid({0, 0, 5})});
    const std::string out = text(p, {tp});
    const std::string expected =
        "G21 (header)\n"
        "M3 S0\n"
        "G0 X0.000 Y0.000 Z0.000\n"
        "(power S500)\n"
        "M4\n"
        "G1 X10.000 F800\n"
        "G1 Y10.000\n"
        "M5\n"
        "G1 X20.000\n"
        "(power S750)\n"
        "M4\n"
        "G1 Y20.000\n"
        "M5\n"
        "G0 X0.000 Y0.000 Z5.000\n"
        "M2\n";
    CHECK(out == expected);
}

TEST_CASE("T-0323: a power change while the beam stays on sets the power without switching it on again",
          "[post][laser]") {
    const PostProcessor p = parsePost(kLaserPost);
    const Toolpath tp = laserPath({Move::rapid({0, 0, 0}), Move::feed({10, 0, 0}).withJet(20.0),
                                   Move::feed({20, 0, 0}).withJet(100.0)});
    const std::string out = text(p, {tp});
    CHECK(out.find("(power S200)") != std::string::npos);
    CHECK(out.find("(power S1000)") != std::string::npos);
    std::size_t on = 0;
    for (std::size_t at = out.find("M4"); at != std::string::npos; at = out.find("M4", at + 1)) {
        ++on;
    }
    CHECK(on == 1);
    // A toolpath that ends with the beam on switches it off before the footer.
    CHECK(out.find("M5\nM2") != std::string::npos);
}

TEST_CASE("T-0323: a laser toolpath writes no spindle start and accepts spindle speed 0", "[post][laser]") {
    const PostProcessor p = parsePost(kLaserPost);
    Toolpath a = laserPath({Move::rapid({0, 0, 0}), Move::feed({10, 0, 0}).withJet(10.0)});
    Toolpath b = a;
    b.tool.name = "Laser two";
    b.tool.toolNumber = 2;
    const std::string out = text(p, {a, b});
    CHECK(out.find("spindle on") == std::string::npos);
    CHECK(out.find("(toolchange Laser two)") != std::string::npos);
    // The beam is off between the toolpaths and power is set again for the second.
    CHECK(out.find("M5\n(toolchange Laser two)") != std::string::npos);
}

TEST_CASE("T-0323: a spinning toolpath still needs a spindle speed above zero", "[post][laser]") {
    const PostProcessor p = parsePost(kLaserPost);
    Toolpath tp = laserPath({Move::rapid({0, 0, 0}), Move::feed({10, 0, 0})});
    tp.tool.laser = false;
    auto out = p.run({tp}, PostContext{});
    CHECK_FALSE(out.ok());
}

TEST_CASE("T-0323: a laser toolpath needs the jet on and off blocks", "[post][laser]") {
    std::string noOff = kLaserPost;
    noOff.replace(noOff.find("begin JET_TOOL_OFF\n\"M5\"\nend\n"), 27, "");
    const PostProcessor p = parsePost(noOff);
    const Toolpath tp = laserPath({Move::rapid({0, 0, 0}), Move::feed({10, 0, 0}).withJet(10.0)});
    auto out = p.run({tp}, PostContext{});
    REQUIRE_FALSE(out.ok());
    CHECK_THAT(out.error().message, ContainsSubstring("JET_TOOL_OFF"));
}

TEST_CASE("T-0323: beam power on a toolpath whose tool is not a laser's is refused", "[post][laser]") {
    const PostProcessor p = parsePost(kLaserPost);
    Toolpath tp = laserPath({Move::rapid({0, 0, 0}), Move::feed({10, 0, 0}).withJet(10.0)});
    tp.tool.laser = false;
    tp.tool.spindleSpeed = 10000.0;
    auto out = p.run({tp}, PostContext{});
    REQUIRE_FALSE(out.ok());
    CHECK_THAT(out.error().message, ContainsSubstring("not a laser"));
}

TEST_CASE("T-0323: out-of-range beam power is refused", "[post][laser]") {
    const PostProcessor p = parsePost(kLaserPost);
    for (const double bad : {101.0, -5.0}) {
        Move m = Move::feed({10, 0, 0});
        m.jetPower = bad;
        const Toolpath tp = laserPath({Move::rapid({0, 0, 0}), m});
        CHECK_FALSE(p.run({tp}, PostContext{}).ok());
    }
    // Beam power on a rapid is refused too.
    Move r = Move::rapid({5, 0, 0});
    r.jetPower = 10.0;
    CHECK_FALSE(p.run({laserPath({r})}, PostContext{}).ok());
}

TEST_CASE("T-0323: a laser cut / fill toolpath posts with one on and off per stroke", "[post][laser]") {
    const PostProcessor p = parsePost(kLaserPost);
    tooling::Tool tool;
    tool.id = "l";
    tool.name = "Laser";
    tool.diameter = 0.2;
    tool.defaultCuttingData.feedRate = 600.0;
    tool.defaultCuttingData.plungeRate = 600.0;
    tool.defaultCuttingData.passDepth = 1.0;
    toolpath::LaserCutFillParams params;
    params.strategy = toolpath::LaserStrategy::HatchFill;
    params.stepover = 2.0;
    params.power = 35.0;
    const toolpath::LaserCutFillResult r = toolpath::laserCutFillToolpath(
        {geometry::Contour::rectangle(geometry::Box2({0, 0}, {10, 10}))}, tool, std::nullopt, params);
    REQUIRE(r.ok());
    const std::string out = text(p, {r.toolpath});
    std::size_t on = 0;
    std::size_t off = 0;
    for (std::size_t at = out.find("M4"); at != std::string::npos; at = out.find("M4", at + 1)) {
        ++on;
    }
    for (std::size_t at = out.find("M5"); at != std::string::npos; at = out.find("M5", at + 1)) {
        ++off;
    }
    CHECK(on == r.strokes);
    CHECK(off == r.strokes);
    // The power is set once (it never changes within the toolpath).
    CHECK(out.find("(power S350)") != std::string::npos);
    CHECK(out.find("(power S350)") == out.rfind("(power S350)"));
}
