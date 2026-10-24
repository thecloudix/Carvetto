// Golden-file tests for the shipped Grbl post (posts/grbl_mm.pp). Paths come from CMake
// (CARVETTO_POSTS_DIR, CARVETTO_GOLDEN_DIR) so the tests run from any build directory.

#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"

using namespace carvetto::post;
using carvetto::toolpath::Move;
using carvetto::toolpath::Toolpath;
using carvetto::toolpath::ToolSnapshot;

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

PostProcessor grbl() {
    auto r = PostProcessor::parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp"));
    if (!r.ok()) {
        FAIL("grbl_mm.pp failed to parse: " << r.error().describe());
    }
    return r.value();
}

ToolSnapshot endMill() {
    ToolSnapshot t;
    t.name = "End Mill (6 mm)";
    t.toolNumber = 1;
    t.diameter = 6.0;
    t.feedRate = 1000.0;
    t.plungeRate = 300.0;
    t.spindleSpeed = 18000.0;
    return t;
}

/// A 50 mm square profile at Z-3: one rapid, one plunge, four feeds and a retract.
Toolpath square() {
    Toolpath tp;
    tp.name = "Profile 1";
    tp.tool = endMill();
    tp.moves = {Move::rapid({10, 10, 5}),  Move::plunge({10, 10, -3}), Move::feed({60, 10, -3}),
                Move::feed({60, 60, -3}),  Move::feed({10, 60, -3}),   Move::feed({10, 10, -3}),
                Move::retract({10, 10, 5})};
    return tp;
}

PostContext squareContext() {
    PostContext ctx;
    ctx.fileName = "grbl_square";
    ctx.product = "Carvetto";
    ctx.home = {0, 0, 20};
    ctx.safeZ = 5.0;
    return ctx;
}

}  // namespace

TEST_CASE("Shipped Grbl post parses cleanly", "[post][grbl]") {
    const PostProcessor pp = grbl();
    CHECK(pp.warnings().empty());
    CHECK(pp.settings().postName == "Grbl (mm) (*.nc)");
    CHECK(pp.settings().fileExtension == "nc");
    CHECK(pp.settings().units == Units::Millimetres);
    CHECK(pp.settings().lineEnding == "\n");
    for (const char* b : {"HEADER", "FOOTER", "TOOLCHANGE", "NEW_SEGMENT", "RAPID_MOVE", "FEED_MOVE",
                          "PLUNGE_MOVE", "RETRACT_MOVE", "CW_ARC_MOVE", "CCW_ARC_MOVE", "DWELL_MOVE"}) {
        INFO(b);
        CHECK(pp.hasBlock(b));
    }
}

TEST_CASE("AC1: Grbl post + square profile matches golden grbl_square.nc exactly", "[post][grbl][golden]") {
    const std::string golden = readFile(std::string(CARVETTO_GOLDEN_DIR) + "/post/grbl_square.nc");
    const auto r = grbl().run({square()}, squareContext());
    REQUIRE(r.ok());
    CHECK(r.value().warnings.empty());
    CHECK(r.value().text == golden);
}

TEST_CASE("Grbl post: tool change, arcs and dwell", "[post][grbl]") {
    Toolpath drill;
    drill.name = "Drill";
    drill.tool = endMill();
    drill.tool.name = "Drill 3 mm";
    drill.tool.toolNumber = 2;
    drill.tool.spindleSpeed = 10000.0;
    drill.moves = {Move::rapid({30, 30, 5}), Move::plunge({30, 30, -5}), Move::dwell({30, 30, -5}, 0.5),
                   Move::retract({30, 30, 5})};
    Toolpath circle;
    circle.name = "Circle";
    circle.tool = drill.tool;
    circle.moves = {Move::rapid({40, 30, 5}), Move::plunge({40, 30, -1}), Move::arcCW({40, 30, -1}, {30, 30}),
                    Move::retract({40, 30, 5})};

    const auto r = grbl().run({square(), drill, circle}, squareContext());
    REQUIRE(r.ok());
    const std::string& out = r.value().text;
    const std::string expectedTail =
        "G0 Z5.000\n"  // end of the square
        "M5\n"
        "G0 Z5.000\n"
        "(Change to tool 2: Drill 3 mm)\n"
        "M0\n"
        "(Drill)\n"
        "M3 S10000\n"
        "G0 X30.000 Y30.000 Z5.000\n"
        "G1 Z-5.000 F300\n"
        "G4 P0.50\n"
        "G0 Z5.000\n"
        "(Circle)\n"
        "M3 S10000\n"
        "G0 X40.000 Y30.000 Z5.000\n"
        "G1 Z-1.000 F300\n"
        "G2 I-10.000 J0.000 F1000\n"  // full circle: modal X/Y/Z unchanged

        "G0 Z5.000\n"
        "G0 Z5.000\n"
        "M5\n"
        "G0 X0.000 Y0.000\n"
        "M30\n";
    REQUIRE(out.size() > expectedTail.size());
    CHECK(out.substr(out.size() - expectedTail.size()) == expectedTail);
}
