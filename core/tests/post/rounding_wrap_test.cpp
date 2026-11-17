// T-0322: a Rounding toolpath through the wrapping post (ADR-0103, ADR-0115): every turn of the raster is a
// move that turns the rotary axis by exactly 360 degrees, and a spiral layout line is a helix.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/rounding.hpp"

using namespace carvetto;

namespace {

const char* kPost = R"PP(
ROTARY_WRAP_Y = "A"
MAXIMUM_ROTATION = 90
POST_NAME = "Rounding test (*.nc)"
FILE_EXTENSION = "nc"
LINE_ENDING = "[10]"
VAR TOOL_NUMBER = [T|A||1.0]
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|C| F|1.0]
VAR X_POSITION = [X|C| X|1.3]
VAR Y_POSITION = [Y|C| Y|1.3]
VAR Z_POSITION = [Z|C| Z|1.3]
VAR A_POSITION = [A|C| A|1.3]
VAR SAFE_Z = [SAFEZ|A| Z|1.3]
begin HEADER
"G90"
"M3[S]"
end
begin RAPID_MOVE
"G0[X][Y][Z][A]"
end
begin PLUNGE_MOVE
"G1[X][Y][Z][A][F]"
end
begin FEED_MOVE
"G1[X][Y][Z][A][F]"
end
begin RETRACT_MOVE
"G1[X][Y][Z][A][F]"
end
begin FOOTER
"M30"
end
)PP";

std::vector<std::string> lines(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream in(text);
    for (std::string l; std::getline(in, l);) {
        out.push_back(l);
    }
    return out;
}

}  // namespace

TEST_CASE("rounding through the wrapping post: each turn is 360 degrees of A", "[post][rotary][rounding]") {
    tooling::Tool t;
    t.id = "mill";
    t.name = "Mill";
    t.diameter = 12.0;
    t.defaultCuttingData = {5.0, 10.0, 0.0, 0.0, 12000.0, 1000.0, 300.0, 0.0, 1};
    toolpath::RoundingParams p;
    p.method = toolpath::RoundingMethod::Raster;
    p.blankRadius = 25.0;
    p.targetRadius = 20.0;
    p.surfaceZ = 25.0;  // axis zero
    p.alongStart = 0.0;
    p.alongEnd = 20.0;
    p.safeZ = 35.0;
    const toolpath::RoundingResult r = toolpath::roundingToolpath(t, std::nullopt, p);
    REQUIRE(r.ok());
    REQUIRE(r.strokes == 3);  // 20 mm in steps of 10: three turns, one pass

    auto parsed = post::PostProcessor::parse(kPost);
    REQUIRE(parsed.ok());
    post::PostContext ctx;
    ctx.fileName = "rounding";
    ctx.home = {0, 0, 35};
    ctx.safeZ = 35;
    ctx.rotaryDiameter = 50.0;
    ctx.rotaryAlong = 'X';
    ctx.rotaryAxisZ = 0.0;
    const auto out = parsed.value().run({r.toolpath}, ctx);
    REQUIRE(out.ok());
    const std::vector<std::string> text = lines(out.value().text);
    // MAXIMUM_ROTATION 90: each full turn is written as four cutting moves ending at A90, A180, A270 and A360;
    // the X of the turn is constant (the cylinder is along X). The lifted raster unwinds the axis by rapid moves.
    std::size_t a360 = 0;
    std::size_t a90 = 0;
    for (const std::string& l : text) {
        if (l.rfind("G1", 0) == 0 && l.find(" A360.000") != std::string::npos) {
            ++a360;
        }
        if (l.rfind("G1", 0) == 0 && l.find(" A90.000") != std::string::npos) {
            ++a90;
        }
        CHECK(l.find(" Y") == std::string::npos);  // the circumference is written as the angle only
    }
    CHECK(a360 == 3);
    CHECK(a90 == 3);
}
