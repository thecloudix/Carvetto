// T-0154: toolpaths that run with the spindle off (diamond-drag Quick Engrave, ADR-0020, ADR-0024).
//
// - A toolpath whose tool snapshot has `spindleOff` writes no spindle start: SPINDLE_ON is skipped,
//   the spindle start lines of HEADER / TOOLCHANGE / NEW_SEGMENT are left out, and the post's
//   SPINDLE_OFF_PAUSE (M5 + operator stop) is written right before it. A post without that block
//   refuses such toolpaths; a line that writes [S] together with other codes, an [S] anywhere else
//   and a literal M3/M4/S word are refused too.
// - The exemption from T-0134's spindle speed > 0 rule is exactly that flag: every toolpath kind
//   (Profile, Pocket, Drill, Quick Engrave with every other tool, also a diamond drag tool used by
//   Profile or Pocket) with spindle speed 0 is still refused by every shipped post.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"

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

const std::vector<std::string>& shippedStems() {
    static const std::vector<std::string> stems{"grbl_mm",  "grbl_inch",   "mach3_mm",      "mach3_inch",
                                                "mach4_mm", "linuxcnc_mm", "linuxcnc_inch", "marlin_mm"};
    return stems;
}

PostProcessor loadShipped(const std::string& stem) {
    auto r = PostProcessor::parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/" + stem + ".pp"));
    if (!r.ok()) {
        FAIL(stem << ".pp: " << r.error().describe());
    }
    return r.value();
}

PostProcessor parseText(const std::string& text) {
    auto r = PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("parse failed: " << r.error().describe());
    }
    return r.value();
}

/// The run's error message, or "" when it succeeded.
std::string runError(const PostProcessor& pp, const std::vector<Toolpath>& tps) {
    const auto r = pp.run(tps, test::fixtureContext());
    return r.ok() ? std::string() : r.error().describe();
}

std::string runOk(const PostProcessor& pp, const std::vector<Toolpath>& tps) {
    const auto r = pp.run(tps, test::fixtureContext());
    if (!r.ok()) {
        FAIL("run failed: " << r.error().describe());
    }
    return r.value().text;
}

ToolSnapshot spinning(int number, double rpm = 1000.0) {
    ToolSnapshot t;
    t.name = "Mill " + std::to_string(number);
    t.toolNumber = number;
    t.diameter = 3.0;
    t.feedRate = 600.0;
    t.plungeRate = 200.0;
    t.spindleSpeed = rpm;
    return t;
}

Toolpath cut(const std::string& name, const ToolSnapshot& tool) {
    Toolpath tp;
    tp.name = name;
    tp.tool = tool;
    tp.moves = {Move::rapid({1, 1, 5}), Move::plunge({1, 1, -0.1}), Move::feed({2, 1, -0.1}), Move::retract({2, 1, 5})};
    return tp;
}

Toolpath drag(const std::string& name = "Drag") {
    return cut(name, test::fixtureDragTool());
}

// A small post: every block on its own lines so the tests can see what was written.
const std::string kMoves =
    "begin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\n"
    "begin FEED_MOVE\n\"G1[X][Y][Z][F]\"\nend\n";
const std::string kPause = "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"(spindle off)\"\n\"M0\"\nend\n";
const std::string kSmall =
    "VAR SPINDLE_SPEED = [S|A| S|1.0]\n"
    "VAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\n"
    "VAR FEED_RATE = [F|C| F|1.0]\n"
    "begin HEADER\n\"(header [TOOLPATH_NAME])\"\n\"M3[S]\"\nend\n"
    "begin TOOLCHANGE\n\"M5\"\n\"(change to [T])\"\n\"M6 T[T]\"\nend\n"
    "begin SPINDLE_ON\n\"M3[S]\"\nend\n"
    "begin NEW_SEGMENT\n\"(segment [TOOLPATH_NAME])\"\n\"M3[S]\"\nend\n"
    "begin FOOTER\n\"M5\"\n\"M30\"\nend\n" +
    kMoves;

std::size_t count(const std::string& text, const std::string& what) {
    std::size_t n = 0;
    for (std::size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) {
        ++n;
    }
    return n;
}

/// Lines whose G-code outside comments (`( … )`, `;` to the line end) holds an M3 word.
int liveSpindleStarts(const std::string& text) {
    int n = 0;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        std::string code;
        int depth = 0;
        for (const char c : line) {
            if (c == ';' && depth == 0) break;
            if (c == '(') {
                ++depth;
            } else if (c == ')') {
                --depth;
            } else if (depth == 0) {
                code += c;
            }
        }
        if (code.find("M3") != std::string::npos && code.find("M30") == std::string::npos) {
            ++n;
        }
    }
    return n;
}

/// A tool for real toolpath generators.
tooling::Tool makeTool(tooling::ToolType type, double rpm) {
    tooling::Tool t;
    t.id = "t";
    t.name = "Tool";
    t.type = type;
    t.diameter = 3.0;
    t.includedAngle = type == tooling::ToolType::Drill ? 118.0 * geometry::kPi / 180.0 : 0.0;
    if (type == tooling::ToolType::VBit || type == tooling::ToolType::Engraving) {
        t.diameter = 6.35;
        t.includedAngle = geometry::kPi / 2.0;
        t.tipDiameter = type == tooling::ToolType::Engraving ? 0.2 : 0.0;
    }
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = rpm;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 0.5;
    return t;
}

}  // namespace

TEST_CASE("T-0154: a spindle-off toolpath skips the spindle start and gets the pause", "[post][spindle-off]") {
    const PostProcessor pp = parseText(kSmall + kPause);

    // First toolpath: the header's spindle start line is left out, the pause follows the header.
    const std::string first = runOk(pp, {drag("A"), drag("B")});
    CHECK(first.find("M3 ") == std::string::npos);
    CHECK(first.find(" S") == std::string::npos);
    CHECK(first ==
          "(header A)\nM5\n(spindle off)\nM0\n"
          "G0 X1.000 Y1.000 Z5.000\nG1 Z-0.100 F400\nG1 X2.000 F1500\nG1 Z5.000 F400\n"
          "(segment B)\nM5\n(spindle off)\nM0\n"
          "G0 X1.000 Y1.000 Z5.000\nG1 Z-0.100 F400\nG1 X2.000 F1500\nG1 Z5.000 F400\n"
          "M5\nM30\n");

    // After a spinning toolpath: TOOLCHANGE (no SPINDLE_ON), then the pause; back to the spinning
    // tool: TOOLCHANGE + SPINDLE_ON with its speed.
    const std::string mixed = runOk(pp, {cut("Mill", spinning(1)), drag(), cut("Mill again", spinning(1))});
    CHECK(count(mixed, "M3 S1000") == 2);
    CHECK(count(mixed, "M6 T") == 2);
    CHECK(count(mixed, "M0\n") == 1);
    const std::size_t pause = mixed.find("(spindle off)");
    REQUIRE(pause != std::string::npos);
    CHECK(mixed.find("M6 T3") < pause);
    CHECK(mixed.rfind("M3 S1000") > pause);
    CHECK(mixed.find("M3", mixed.find("M6 T3")) > mixed.find("M6 T1", pause));
}

TEST_CASE("T-0154: the same tool number and name running spindle off and on is a tool change",
          "[post][spindle-off]") {
    const PostProcessor pp = parseText(kSmall + kPause);
    ToolSnapshot same = test::fixtureDragTool();
    same.spindleOff = false;
    same.spindleSpeed = 5000.0;
    const std::string text = runOk(pp, {drag(), cut("Spun", same), drag("Drag 2")});
    // Spindle on only through TOOLCHANGE + SPINDLE_ON, never through NEW_SEGMENT.
    CHECK(text.find("(segment") == std::string::npos);
    CHECK(count(text, "M6 T3") == 2);
    CHECK(count(text, "M3 S5000") == 1);
    CHECK(count(text, "(spindle off)") == 2);
}

TEST_CASE("T-0154: a post without SPINDLE_OFF_PAUSE refuses spindle-off toolpaths", "[post][spindle-off]") {
    const PostProcessor pp = parseText(kSmall);
    const std::string msg = runError(pp, {cut("Mill", spinning(1)), drag("Drag text")});
    CHECK_THAT(msg, ContainsSubstring("toolpath 'Drag text' runs with the spindle off"));
    CHECK_THAT(msg, ContainsSubstring("no SPINDLE_OFF_PAUSE block"));
    // Spinning toolpaths do not need the block.
    CHECK(runError(pp, {cut("Mill", spinning(1))}).empty());
    // A spindle-off toolpath without moves is skipped like any other (T-0136).
    Toolpath empty = drag("Empty");
    empty.moves.clear();
    CHECK(runError(pp, {cut("Mill", spinning(1)), empty}).empty());
}

TEST_CASE("T-0154: posts cannot start the spindle of a spindle-off toolpath", "[post][spindle-off]") {
    // A spindle start sharing its line with another code is not left out but refused.
    const std::string shared = kSmall + kPause;
    std::string header = shared;
    header.replace(header.find("\"M3[S]\"\nend\nbegin TOOLCHANGE"), 7, "\"T1 M3[S]\"");
    CHECK_THAT(runError(parseText(header), {drag()}),
               ContainsSubstring("writes [S] together with other codes; start the spindle on a line of its own"));
    // A spinning first toolpath still writes that header line.
    CHECK(runOk(parseText(header), {cut("Mill", spinning(1))}).find("T1 M3 S1000") != std::string::npos);

    // Spindle start lines in their accepted forms are left out.
    for (const char* line : {"\"M3[S]\"", "\"M03 S[S]\"", "\"[S] M4\"", "\"M3[S] (start)\"", "\"S[S]\""}) {
        INFO(line);
        std::string post = std::string("VAR SPINDLE_SPEED = [S|A||1.0]\n") +
                           "begin HEADER\n" + line + "\nend\n" + kMoves + kPause;
        CHECK(runError(parseText(post), {drag()}).empty());
    }

    // [S] in any other block of a spindle-off toolpath is refused.
    const std::string inPause = kSmall + "begin SPINDLE_OFF_PAUSE\n\"M5 S[S]\"\n\"M0\"\nend\n";
    CHECK_THAT(runError(parseText(inPause), {drag()}), ContainsSubstring("the post writes [S] here"));
    std::string inMove = kPause + "begin HEADER\n\"G21\"\nend\nbegin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\n"
                                  "begin FEED_MOVE\n\"G1[X][Y][Z] S[S]\"\nend\n";
    CHECK_THAT(runError(parseText(inMove), {drag()}), ContainsSubstring("in FEED_MOVE of toolpath 'Drag'"));

    // Literal spindle starts and speeds (no [S] tag) are refused as well: the second guard.
    for (const char* literal : {"M3", "M03", "m4", "M3.0", "S12000", "G0 Z5 M4", "M3(start)"}) {
        INFO(literal);
        std::string post = "begin NEW_SEGMENT\n\"" + std::string(literal) + "\"\nend\n" + kMoves + kPause;
        const std::string msg = runError(parseText(post), {drag("A"), drag("B")});
        CHECK_THAT(msg, ContainsSubstring("runs with the spindle off"));
        CHECK_THAT(msg, ContainsSubstring("but the post writes"));
    }
    // ... but not words that merely contain the letters, comments, or Marlin's M0 message.
    for (const char* harmless : {"M0 Drag tool 3 spindle off then resume", "(M3 S1000)", "; M3 S1000", "M30", "M5",
                                 "G43 H3", "M6 T3"}) {
        INFO(harmless);
        std::string post = "begin NEW_SEGMENT\n\"" + std::string(harmless) + "\"\nend\n" + kMoves + kPause;
        CHECK(runError(parseText(post), {drag("A"), drag("B")}).empty());
    }
}

TEST_CASE("T-0154: only the spindle-off flag exempts a toolpath from the spindle speed rule",
          "[post][spindle-off][safety]") {
    for (const std::string& stem : shippedStems()) {
        INFO(stem);
        const PostProcessor pp = loadShipped(stem);
        // Spindle speed 0 without the flag: refused in the header, after a tool change and in a
        // new segment, although the post can write spindle-off toolpaths.
        for (std::size_t which : {std::size_t{0}, std::size_t{1}, std::size_t{2}}) {
            std::vector<Toolpath> tps{cut("A", spinning(1)), cut("B", spinning(2)), cut("C", spinning(2))};
            tps[which].tool.spindleSpeed = 0.0;  // C has B's tool number and name -> NEW_SEGMENT
            const std::string msg = runError(pp, tps);
            INFO(msg);
            CHECK_THAT(msg, ContainsSubstring("has spindle speed 0 rpm; it must be greater than zero"));
        }
        // The flagged toolpath passes with speed 0.
        CHECK(runError(pp, {cut("A", spinning(1)), drag()}).empty());
    }
}

TEST_CASE("T-0154: every other toolpath kind with spindle speed 0 is still refused by every shipped post",
          "[post][spindle-off][safety]") {
    using geometry::Box2;
    using geometry::Contour;
    using geometry::Vec2;
    const std::vector<Contour> square{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 20}})};
    std::vector<std::pair<std::string, Toolpath>> kinds;

    // Profile and Pocket, with an end mill and with a diamond drag tool (they accept any tool but
    // never mark it spindle-off).
    for (const tooling::ToolType type : {tooling::ToolType::EndMill, tooling::ToolType::DiamondDrag}) {
        const tooling::Tool tool = makeTool(type, 0.0);
        toolpath::ProfileParams pp;
        pp.passes.cutDepth = 1.0;
        const toolpath::ProfileResult pr = toolpath::profileToolpath(square, tool, pp);
        REQUIRE(pr.ok());
        kinds.emplace_back("profile " + std::to_string(static_cast<int>(type)), pr.toolpath);
        toolpath::PocketParams kp;
        kp.cutDepth = 1.0;
        const toolpath::PocketResult kr =
            toolpath::pocketToolpath(square, {toolpath::PocketTool{tool, std::nullopt}}, kp);
        REQUIRE_FALSE(kr.toolpaths.empty());
        REQUIRE_FALSE(kr.toolpaths.front().isEmpty());
        kinds.emplace_back("pocket " + std::to_string(static_cast<int>(type)), kr.toolpaths.front());
    }
    // Drill.
    {
        toolpath::DrillParams dp;
        dp.cutDepth = 2.0;
        const toolpath::DrillResult dr = toolpath::drillToolpath(square, makeTool(tooling::ToolType::Drill, 0.0),
                                                                 std::nullopt, dp);
        REQUIRE(dr.ok());
        kinds.emplace_back("drill", dr.toolpath);
    }
    // Quick Engrave with every tool but the diamond drag, Outline and Fill.
    for (const tooling::ToolType type : {tooling::ToolType::VBit, tooling::ToolType::Engraving,
                                         tooling::ToolType::EndMill, tooling::ToolType::BallNose}) {
        for (const toolpath::EngraveStrategy strategy :
             {toolpath::EngraveStrategy::Outline, toolpath::EngraveStrategy::Fill}) {
            toolpath::EngraveParams ep;
            ep.depth = 0.2;
            ep.strategy = strategy;
            ep.stepover = 0.1;
            const toolpath::EngraveResult er = toolpath::engraveToolpath(square, makeTool(type, 0.0), std::nullopt, ep);
            REQUIRE(er.ok());
            kinds.emplace_back("engrave " + std::to_string(static_cast<int>(type)) + "/" +
                                   std::to_string(static_cast<int>(strategy)),
                               er.toolpath);
        }
    }
    for (const auto& [what, tp] : kinds) {
        INFO(what);
        CHECK_FALSE(tp.tool.spindleOff);
        CHECK(tp.tool.spindleSpeed == 0.0);
    }

    for (const std::string& stem : shippedStems()) {
        INFO(stem);
        const PostProcessor pp = loadShipped(stem);
        for (const auto& [what, tp] : kinds) {
            INFO(what);
            Toolpath named = tp;
            named.name = "Kind";
            // First (HEADER), after a tool change (SPINDLE_ON) and after a drag toolpath.
            std::vector<std::vector<Toolpath>> jobs{{named}, {cut("Mill", spinning(9)), named}, {drag(), named}};
            for (const std::vector<Toolpath>& job : jobs) {
                const std::string msg = runError(pp, job);
                INFO(msg);
                CHECK_THAT(msg, ContainsSubstring("of toolpath 'Kind'"));
                CHECK_THAT(msg, ContainsSubstring("has spindle speed 0 rpm; it must be greater than zero"));
            }
        }
        // The same diamond drag tool through Quick Engrave: spindle off, posted.
        toolpath::EngraveParams ep;
        ep.depth = 0.1;
        const toolpath::EngraveResult er =
            toolpath::engraveToolpath(square, makeTool(tooling::ToolType::DiamondDrag, 0.0), std::nullopt, ep);
        REQUIRE(er.ok());
        CHECK(er.toolpath.tool.spindleOff);
        CHECK(runError(pp, {er.toolpath}).empty());
    }
}

TEST_CASE("T-0154: hostile tool names in the spindle-off pause stay inside the comment",
          "[post][spindle-off][text]") {
    for (const std::string& stem : shippedStems()) {
        INFO(stem);
        const PostProcessor pp = loadShipped(stem);
        for (const char* name : {"LOGOPEN,/tmp/x", "Evil\nG0 Z-50", "a)M3 S9000(", "MSG,hi", "x;M3 S1", "~!?"}) {
            INFO(name);
            std::vector<Toolpath> job = test::dragToolpaths();
            for (Toolpath& tp : job) {
                if (tp.tool.spindleOff) {
                    tp.tool.name = name;
                }
            }
            const std::string text = runOk(pp, job);
            // No injected block, no spindle start beyond the end mill's, no active comment.
            CHECK(text.find("\nG0 Z-50") == std::string::npos);
            CHECK(liveSpindleStarts(text) == 1);
            CHECK(text.find("(LOGOPEN,") == std::string::npos);
            CHECK(text.find("(MSG,") == std::string::npos);
            CHECK(text.find_first_of("~!?") == std::string::npos);
        }
    }
}
