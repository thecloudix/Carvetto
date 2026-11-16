// T-0325 (ADR-0107): the shipped laser-capable post (posts/grbl_laser_mm.pp) against a golden file, the
// LASER_SUPPORT / machine-flag setup check, and the refusal of a laser toolpath by a post that does not declare it.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/post/laser_setup.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/machine.hpp"
#include "carvetto/toolpath/toolpath.hpp"

using namespace carvetto;
using post::PostContext;
using post::PostProcessor;
using toolpath::Move;
using toolpath::Toolpath;
using toolpath::ToolSnapshot;
using Catch::Matchers::ContainsSubstring;

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

PostProcessor laserPost() {
    auto r = PostProcessor::parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_laser_mm.pp"));
    if (!r.ok()) {
        FAIL("grbl_laser_mm.pp failed to parse: " << r.error().describe());
    }
    return r.value();
}

ToolSnapshot laserTool(int number, const char* name, double feed) {
    ToolSnapshot t;
    t.name = name;
    t.toolNumber = number;
    t.diameter = 0.2;
    t.feedRate = feed;
    t.plungeRate = feed;
    t.laser = true;
    return t;
}

/// Two laser toolpaths with one tool (cut with arc and a beam-off feed; a hatch with overscan) and a third with
/// another laser tool.
std::vector<Toolpath> laserFixture() {
    Toolpath cut;
    cut.name = "Laser cut";
    cut.tool = laserTool(1, "Laser 0.2 mm", 1200.0);
    cut.moves = {Move::rapid({10, 10, 5}),
                 Move::rapid({10, 10, 0}),
                 Move::feed({20, 10, 0}).withJet(60.0),
                 Move::arcCCW({30, 20, 0}, {20, 20}).withJet(60.0),
                 Move::feed({30, 30, 0}).withJet(80.0),
                 Move::feed({30, 40, 0}),
                 Move::feed({10, 40, 0}).withJet(80.0),
                 Move::rapid({10, 10, 5})};
    Toolpath hatch;
    hatch.name = "Laser fill";
    hatch.tool = cut.tool;
    hatch.moves = {Move::rapid({-2, 5, 0}),         Move::feed({0, 5, 0}),
                   Move::feed({20, 5, 0}).withJet(35.0), Move::feed({22, 5, 0}),
                   Move::rapid({22, 6, 0}),         Move::feed({20, 6, 0}),
                   Move::feed({0, 6, 0}).withJet(35.0),  Move::feed({-2, 6, 0}),
                   Move::rapid({-2, 6, 5})};
    Toolpath second;
    second.name = "Laser mark";
    second.tool = laserTool(2, "Laser 0.1 mm", 3000.0);
    second.moves = {Move::rapid({0, 0, 0}), Move::feed({5, 0, 0}).withJet(100.0), Move::feed({5, 5, 0}).withJet(7.5),
                    Move::rapid({5, 5, 5})};
    return {cut, hatch, second};
}

PostContext context() {
    PostContext ctx;
    ctx.fileName = "laser_fixture";
    ctx.product = "Carvetto";
    ctx.home = {0, 0, 20};
    ctx.safeZ = 5.0;
    return ctx;
}

}  // namespace

TEST_CASE("T-0325: the shipped laser post parses cleanly and declares laser support", "[post][laser]") {
    const PostProcessor pp = laserPost();
    CHECK(pp.warnings().empty());
    CHECK(pp.settings().laserSupport);
    CHECK(pp.settings().units == post::Units::Millimetres);
    for (const char* b : {"JET_TOOL_POWER", "JET_TOOL_ON", "JET_TOOL_OFF", "SPINDLE_ON"}) {
        INFO(b);
        CHECK(pp.hasBlock(b));
    }
    CHECK(post::laserSetupProblem(nullptr, pp).empty());
}

TEST_CASE("T-0325: laser fixture matches the golden file", "[post][laser][golden]") {
    const PostProcessor pp = laserPost();
    auto out = pp.run(laserFixture(), context());
    REQUIRE(out.ok());
    CHECK(out.value().warnings.empty());
    const std::string golden = readFile(std::string(CARVETTO_GOLDEN_DIR) + "/post/grbl_laser_mm_laser.nc");
    CHECK(out.value().text == golden);
}

TEST_CASE("T-0325: laser output obeys the beam safety rules", "[post][laser]") {
    const PostProcessor pp = laserPost();
    auto out = pp.run(laserFixture(), context());
    REQUIRE(out.ok());
    std::istringstream in(out.value().text);
    std::string line;
    bool on = false;
    int moves = 0;
    while (std::getline(in, line)) {
        if (line == "M4") {
            CHECK_FALSE(on);
            on = true;
        } else if (line == "M5") {
            on = false;
        } else if (line.rfind("G0", 0) == 0 && line.rfind("G0 ", 0) == 0) {
            CHECK_FALSE(on);  // never a rapid with the beam on
        }
        CHECK(line.rfind("M3 ", 0) != 0);  // a laser toolpath writes no spindle start (M30 is the program end)
        CHECK(line != "M3");
        moves += line.rfind("G1", 0) == 0 || line.rfind("G3", 0) == 0 ? 1 : 0;
    }
    CHECK_FALSE(on);  // the program ends with the beam off
    CHECK(moves > 5);
}

TEST_CASE("T-0325: a toolpath that is not a laser's still starts the spindle through the laser post", "[post][laser]") {
    const PostProcessor pp = laserPost();
    Toolpath mill;
    mill.name = "Mill";
    mill.tool.name = "End mill";
    mill.tool.toolNumber = 3;
    mill.tool.diameter = 3.0;
    mill.tool.feedRate = 800.0;
    mill.tool.plungeRate = 200.0;
    mill.tool.spindleSpeed = 12000.0;
    mill.moves = {Move::rapid({0, 0, 5}), Move::plunge({0, 0, -1}), Move::feed({10, 0, -1}), Move::retract({10, 0, 5})};
    std::vector<Toolpath> job = laserFixture();
    job.push_back(mill);
    auto out = pp.run(job, context());
    REQUIRE(out.ok());
    const std::string& t = out.value().text;
    const std::size_t m3 = t.find("M3 S12000");
    REQUIRE(m3 != std::string::npos);
    CHECK(t.find("M3 ") == m3);  // the only spindle start
    CHECK(t.find("G1", m3) != std::string::npos);
}

TEST_CASE("T-0325: laserSetupProblem checks machine flag, LASER_SUPPORT and the jet blocks", "[post][laser]") {
    const PostProcessor laser = laserPost();
    tooling::Machine plain;
    plain.id = "m";
    plain.name = "Router";
    tooling::Machine diode = plain;
    diode.name = "Diode";
    diode.laser = true;
    CHECK_THAT(post::laserSetupProblem(&plain, laser), ContainsSubstring("Router"));
    CHECK(post::laserSetupProblem(&diode, laser).empty());
    CHECK(post::laserSetupProblem(nullptr, laser).empty());

    // A post without the declaration, and one that declares it but lacks the blocks.
    const std::string base = "POST_NAME = \"x\"\nFILE_EXTENSION = \"nc\"\nUNITS = \"MM\"\nLINE_ENDING = \"[10]\"\n";
    auto noDecl = PostProcessor::parse(base + "begin JET_TOOL_ON\n\"M4\"\nend\nbegin JET_TOOL_OFF\n\"M5\"\nend\n");
    REQUIRE(noDecl.ok());
    CHECK_THAT(post::laserSetupProblem(&diode, noDecl.value()), ContainsSubstring("LASER_SUPPORT"));
    auto noOff = PostProcessor::parse(base + "LASER_SUPPORT = \"YES\"\nbegin JET_TOOL_ON\n\"M4\"\nend\n");
    REQUIRE(noOff.ok());
    CHECK_THAT(post::laserSetupProblem(&diode, noOff.value()), ContainsSubstring("JET_TOOL_OFF"));
    auto noOn = PostProcessor::parse(base + "LASER_SUPPORT = \"YES\"\nbegin JET_TOOL_OFF\n\"M5\"\nend\n");
    REQUIRE(noOn.ok());
    CHECK_THAT(post::laserSetupProblem(&diode, noOn.value()), ContainsSubstring("JET_TOOL_ON"));
}

TEST_CASE("T-0325: a post without LASER_SUPPORT refuses a laser toolpath even with the jet blocks", "[post][laser]") {
    std::string text = readFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_laser_mm.pp");
    const std::size_t at = text.find("LASER_SUPPORT = \"YES\"");
    REQUIRE(at != std::string::npos);
    text.replace(at, std::string("LASER_SUPPORT = \"YES\"").size(), "LASER_SUPPORT = \"NO\"");
    auto p = PostProcessor::parse(text);
    REQUIRE(p.ok());
    auto out = p.value().run(laserFixture(), context());
    REQUIRE_FALSE(out.ok());
    CHECK_THAT(out.error().describe(), ContainsSubstring("LASER_SUPPORT"));
    CHECK_THAT(out.error().describe(), ContainsSubstring("Laser cut"));
    // A job without a laser toolpath is unaffected by the flag.
    Toolpath mill;
    mill.name = "Mill";
    mill.tool.toolNumber = 1;
    mill.tool.diameter = 3.0;
    mill.tool.feedRate = 800.0;
    mill.tool.plungeRate = 200.0;
    mill.tool.spindleSpeed = 12000.0;
    mill.moves = {Move::rapid({0, 0, 5}), Move::plunge({0, 0, -1}), Move::retract({0, 0, 5})};
    CHECK(p.value().run({mill}, context()).ok());
}

TEST_CASE("T-0325: the machine laser flag round-trips in the tool database and is absent when unset", "[tooling][laser]") {
    tooling::ToolDatabase db;
    tooling::Machine router;
    router.id = "router";
    router.name = "Router";
    tooling::Machine diode;
    diode.id = "diode";
    diode.name = "Diode";
    diode.laser = true;
    REQUIRE(db.addMachine(router));
    REQUIRE(db.addMachine(diode));
    const std::string text = tooling::exportToolDatabaseJson(db);
    // Exactly one machine carries the key: a database without a laser machine is byte-identical to before.
    CHECK(text.find("\"laser\"") != std::string::npos);
    CHECK(text.find("\"laser\"") == text.rfind("\"laser\""));
    auto back = tooling::importToolDatabaseJson(text);
    REQUIRE(back.ok);
    REQUIRE(back.database.findMachine("diode") != nullptr);
    CHECK(back.database.findMachine("diode")->laser);
    CHECK_FALSE(back.database.findMachine("router")->laser);
    CHECK(tooling::exportToolDatabaseJson(back.database) == text);

    tooling::ToolDatabase none;
    REQUIRE(none.addMachine(router));
    CHECK(tooling::exportToolDatabaseJson(none).find("\"laser\"") == std::string::npos);

    // A non-boolean flag is refused, naming the machine.
    std::string bad = text;
    bad.replace(bad.find("\"laser\": true"), std::string("\"laser\": true").size(), "\"laser\": 1");
    auto refused = tooling::importToolDatabaseJson(bad);
    CHECK_FALSE(refused.ok);
    CHECK_THAT(refused.error, ContainsSubstring("laser"));
}
