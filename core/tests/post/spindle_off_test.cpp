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
    "VAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\n"
    "VAR FEED_RATE = [F|C| F|1.0]\n"
    "begin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\n"
    "begin FEED_MOVE\n\"G1[X][Y][Z][F]\"\nend\n";
const std::string kPause = "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"(spindle off)\"\n\"M0\"\nend\n";
const std::string kUnlock = "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n\"(unlock [T])\"\n\"M0\"\nend\n";
const std::string kSmall =
    "VAR SPINDLE_SPEED = [S|A| S|1.0]\n"
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
    const PostProcessor pp = parseText(kSmall + kPause + kUnlock);

    // First toolpath: the header's spindle start line is left out, the pause follows the header.
    const std::string first = runOk(pp, {drag("A"), drag("B")});
    CHECK(first.find("M3 ") == std::string::npos);
    CHECK(first.find(" S") == std::string::npos);
    CHECK(first ==
          "(header A)\nM5\n(spindle off)\nM0\n"
          "G0 X1.000 Y1.000 Z5.000\nG1 Z-0.100 F400\nG1 X2.000 F1500\nG1 Z5.000 F400\n"
          "(segment B)\nM5\n(spindle off)\nM0\n"
          "G0 X1.000 Y1.000 Z5.000\nG1 Z-0.100 F400\nG1 X2.000 F1500\nG1 Z5.000 F400\n"
          "M5\n(unlock 3)\nM0\nM5\nM30\n");

    // After a spinning toolpath: TOOLCHANGE (no SPINDLE_ON), then the pause; back to the spinning
    // tool: TOOLCHANGE + SPINDLE_ON with its speed.
    const std::string mixed = runOk(pp, {cut("Mill", spinning(1)), drag(), cut("Mill again", spinning(1))});
    CHECK(count(mixed, "M3 S1000") == 2);
    CHECK(count(mixed, "M6 T") == 2);
    CHECK(count(mixed, "M0\n") == 2);  // the spindle-off pause and the unlock pause
    const std::size_t pause = mixed.find("(spindle off)");
    REQUIRE(pause != std::string::npos);
    CHECK(mixed.find("M6 T3") < pause);
    CHECK(mixed.rfind("M3 S1000") > pause);
    CHECK(mixed.find("M3", mixed.find("M6 T3")) > mixed.find("M6 T1", pause));
}

TEST_CASE("T-0154: the same tool number and name running spindle off and on is a tool change",
          "[post][spindle-off]") {
    const PostProcessor pp = parseText(kSmall + kPause + kUnlock);
    ToolSnapshot same = test::fixtureDragTool();
    same.spindleOff = false;
    same.toolType.clear();
    same.spindleSpeed = 5000.0;
    const std::string text = runOk(pp, {drag(), cut("Spun", same), drag("Drag 2")});
    // Spindle on only through TOOLCHANGE + SPINDLE_ON, never through NEW_SEGMENT.
    CHECK(text.find("(segment") == std::string::npos);
    CHECK(count(text, "M6 T3") == 2);
    CHECK(count(text, "M3 S5000") == 1);
    CHECK(count(text, "(spindle off)") == 2);
}

TEST_CASE("T-0154: the unlock pause comes once per drag section, before the spindle restarts",
          "[post][spindle-off]") {
    const PostProcessor pp = parseText(kSmall + kPause + kUnlock);
    // Two drag toolpaths, then a spinning one: one unlock pause, after the second drag toolpath,
    // in the drag tool's context, before the tool change that starts the spindle.
    const std::string a = runOk(pp, {drag("A"), drag("B"), cut("Mill", spinning(1))});
    CHECK(count(a, "(unlock 3)") == 1);
    CHECK(a.find("(segment B)") < a.find("(unlock 3)"));
    CHECK(a.find("(unlock 3)") < a.find("M6 T1"));
    CHECK(a.find("M6 T1") < a.find("M3 S1000"));
    // Two drag sections each followed by a spinning toolpath: two unlock pauses.
    const std::string b =
        runOk(pp, {cut("M1", spinning(1)), drag("A"), cut("M2", spinning(2)), drag("B"), cut("M3", spinning(1))});
    CHECK(count(b, "(unlock 3)") == 2);
    // A job ending with drag toolpaths gets it before the footer (the next program could start
    // a locked spindle); a job without spindle-off toolpaths gets none.
    const std::string end = runOk(pp, {cut("M1", spinning(1)), drag("A"), drag("B")});
    CHECK(count(end, "(unlock 3)") == 1);
    CHECK(end.size() >= 21);
    CHECK(end.substr(end.size() - 21) == "(unlock 3)\nM0\nM5\nM30\n");
    CHECK(runOk(pp, {cut("M1", spinning(1)), cut("M2", spinning(2))}).find("(unlock") == std::string::npos);

    // A post without the block refuses jobs that restart the spindle after a drag section ...
    const PostProcessor noUnlock = parseText(kSmall + kPause);
    const std::string msg = runError(noUnlock, {drag("Drag"), cut("Mill", spinning(1))});
    CHECK_THAT(msg, ContainsSubstring("toolpath 'Mill' starts the spindle after the spindle-off toolpath 'Drag'"));
    CHECK_THAT(msg, ContainsSubstring("no SPINDLE_UNLOCK_PAUSE block"));
    // ... and jobs that end with a drag section.
    const std::string endMsg = runError(noUnlock, {cut("Mill", spinning(1)), drag("Drag")});
    CHECK_THAT(endMsg, ContainsSubstring("the job ends with the spindle-off toolpath 'Drag'"));
    CHECK_THAT(endMsg, ContainsSubstring("no SPINDLE_UNLOCK_PAUSE block"));
    CHECK(runError(noUnlock, {cut("Mill", spinning(1))}).empty());

    // The unlock pause is still part of the spindle-off section: no spindle word in it.
    for (const char* body : {"\"M3 S1000\"", "\"M5 S[S]\""}) {
        INFO(body);
        const PostProcessor bad =
            parseText(kSmall + kPause + "begin SPINDLE_UNLOCK_PAUSE\n" + body + "\n\"M0\"\nend\n");
        CHECK_THAT(runError(bad, {drag("Drag"), cut("Mill", spinning(1))}),
                   ContainsSubstring("SPINDLE_UNLOCK_PAUSE block line"));
    }
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
    const std::string shared = kSmall + kPause + kUnlock;
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
                           "begin HEADER\n" + line + "\nend\n" + kMoves + kPause + kUnlock;
        CHECK(runError(parseText(post), {drag()}).empty());
    }

    // [S] in any other block of a spindle-off toolpath is refused.
    const std::string inPause = kSmall + kUnlock + "begin SPINDLE_OFF_PAUSE\n\"M5 S[S]\"\n\"M0\"\nend\n";
    CHECK_THAT(runError(parseText(inPause), {drag()}), ContainsSubstring("holds an S followed by a tag"));
    std::string inMove = kPause + kUnlock + "begin HEADER\n\"G21\"\nend\nbegin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\n"
                                  "begin FEED_MOVE\n\"G1[X][Y][Z] S[S]\"\nend\n";
    CHECK_THAT(runError(parseText(inMove), {drag()}), ContainsSubstring("runs with the spindle off"));

    // Literal spindle starts and speeds (no [S] tag) are refused as well, in template comments too
    // (strict rule: the template text of a drag section has no reason to mention them).
    for (const char* literal :
         {"M3", "M03", "m4", "M3.0", "S12000", "G0 Z5 M4", "M3(start)", "(M3 S1000)", "; M3 S1000", "(note: S100)"}) {
        INFO(literal);
        std::string post = "begin NEW_SEGMENT\n\"" + std::string(literal) + "\"\nend\n" + kMoves + kPause + kUnlock;
        const std::string msg = runError(parseText(post), {drag("A"), drag("B")});
        CHECK_THAT(msg, ContainsSubstring("runs with the spindle off"));
        CHECK_THAT(msg, ContainsSubstring("holds the spindle word"));
    }
    // ... but not the allowed codes.
    for (const char* harmless : {"M30", "M5", "G43 H3", "M6 T3", "M05", "N10 G4 P0.5"}) {
        INFO(harmless);
        std::string post = "begin NEW_SEGMENT\n\"" + std::string(harmless) + "\"\nend\n" + kMoves + kPause + kUnlock;
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

    // Profile and Pocket with an end mill (they never mark a toolpath spindle-off).
    {
        const tooling::Tool tool = makeTool(tooling::ToolType::EndMill, 0.0);
        toolpath::ProfileParams pp;
        pp.passes.cutDepth = 1.0;
        const toolpath::ProfileResult pr = toolpath::profileToolpath(square, tool, pp);
        REQUIRE(pr.ok());
        kinds.emplace_back("profile", pr.toolpath);
        toolpath::PocketParams kp;
        kp.cutDepth = 1.0;
        const toolpath::PocketResult kr =
            toolpath::pocketToolpath(square, {toolpath::PocketTool{tool, std::nullopt}}, kp);
        REQUIRE_FALSE(kr.toolpaths.empty());
        REQUIRE_FALSE(kr.toolpaths.front().isEmpty());
        kinds.emplace_back("pocket", kr.toolpaths.front());
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

TEST_CASE("T-0154: Profile and Pocket refuse diamond-drag tools and point to Quick Engrave",
          "[toolpath][spindle-off][tool]") {
    using geometry::Box2;
    using geometry::Contour;
    using geometry::Vec2;
    const std::vector<Contour> square{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 20}})};
    tooling::Tool drag = makeTool(tooling::ToolType::DiamondDrag, 12000.0);
    drag.name = "Drag 90";
    toolpath::ProfileParams pp;
    pp.passes.cutDepth = 0.1;
    const toolpath::ProfileResult pr = toolpath::profileToolpath(square, drag, pp);
    CHECK_FALSE(pr.ok());
    REQUIRE(pr.diagnostics.size() == 1);
    CHECK(pr.diagnostics[0].issue == toolpath::ToolpathIssue::InvalidTool);
    CHECK(pr.diagnostics[0].message ==
          "tool 'Drag 90' is a diamond drag tool, which must not turn; use Quick Engrave for diamond-drag tools");

    toolpath::PocketParams kp;
    kp.cutDepth = 0.1;
    const toolpath::PocketResult kr = toolpath::pocketToolpath(square, {toolpath::PocketTool{drag, std::nullopt}}, kp);
    for (const Toolpath& tp : kr.toolpaths) {
        CHECK(tp.isEmpty());
    }
    REQUIRE_FALSE(kr.diagnostics.empty());
    CHECK(kr.diagnostics[0].issue == toolpath::ToolpathIssue::InvalidTool);
    CHECK(kr.diagnostics[0].message ==
          "tool 1 (Drag 90): skipped, a diamond drag tool must not turn; use Quick Engrave for diamond-drag tools");
    // In a multi-tool pocket only the drag tool is skipped.
    const toolpath::PocketResult mixed = toolpath::pocketToolpath(
        square, {toolpath::PocketTool{makeTool(tooling::ToolType::EndMill, 1000.0), std::nullopt},
                 toolpath::PocketTool{drag, std::nullopt}},
        kp);
    REQUIRE_FALSE(mixed.toolpaths.empty());
    CHECK_FALSE(mixed.toolpaths.front().isEmpty());
    CHECK(mixed.has(toolpath::ToolpathIssue::InvalidTool));

    // Quick Engrave Fill still fills with the drag tool (Pocket gets it as an end mill).
    toolpath::EngraveParams ep;
    ep.depth = 0.1;
    ep.strategy = toolpath::EngraveStrategy::Fill;
    ep.stepover = 0.2;
    const toolpath::EngraveResult er = toolpath::engraveToolpath(square, drag, std::nullopt, ep);
    REQUIRE(er.ok());
    CHECK(er.toolpath.tool.spindleOff);
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

TEST_CASE("T-0154: spindle words are found the way a controller reads them", "[post][spindle-off]") {
    // Spaces and tabs between letter and value, signs, a leading '.', lower case, words glued to
    // others, block delete and line numbers: all spindle starts or speeds, in any block of the
    // spindle-off section (TOOLCHANGE, the pause, NEW_SEGMENT).
    const std::vector<std::string> attacks{"M 3",    "M 03",   "M\t3",  "S 1000", "S.5",     "M+3",   "m 4",
                                           "s 1",    "M3.0",   "M004",   "G0M3",   "/M3",     "N10M3", "G0 X1M3",
                                           "M[T]",   "M 0 3",  "S-5",    "M + 3",  "S\t.25"};
    for (const std::string& attack : attacks) {
        INFO(attack);
        const std::string line = "\"" + attack + "\"\n";
        // In TOOLCHANGE before a drag toolpath (tool 3: "M[T]" renders M3).
        const std::string tc = "begin TOOLCHANGE\n" + line + "end\nbegin SPINDLE_ON\n\"M3[S]\"\nend\n" +
                               "VAR SPINDLE_SPEED = [S|A| S|1.0]\n" + kMoves + kPause + kUnlock;
        const std::string tcMsg = runError(parseText(tc), {cut("Mill", spinning(1)), drag()});
        CHECK_THAT(tcMsg, ContainsSubstring("in TOOLCHANGE of toolpath 'Drag'"));
        CHECK_THAT(tcMsg, ContainsSubstring("runs with the spindle off"));
        // In the spindle-off pause itself.
        const std::string pause =
            "begin SPINDLE_OFF_PAUSE\n\"M5\"\n" + line + "\"M0\"\nend\n" + kMoves + kUnlock;
        CHECK_THAT(runError(parseText(pause), {drag()}), ContainsSubstring("SPINDLE_OFF_PAUSE block line"));
        // In the unlock pause.
        const std::string unlock =
            "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n" + line + "\"M0\"\nend\n" + kMoves + kPause;
        CHECK_THAT(runError(parseText(unlock), {drag()}), ContainsSubstring("SPINDLE_UNLOCK_PAUSE block line"));
    }
    // Spindle start lines with spaces or signs are still recognised as spindle-only and left out.
    for (const char* line : {"\"M 3 [S]\"", "\"M\t03 S [S]\"", "\"[N] M+4[S]\""}) {
        INFO(line);
        const std::string post = std::string("VAR SPINDLE_SPEED = [S|A||1.0]\n") + "begin HEADER\n" + line +
                                 "\nend\n" + kMoves + kPause + kUnlock;
        CHECK(runError(parseText(post), {drag()}).empty());
    }
    // The allowlist is strict: a letter must be followed directly by its value, upper case only,
    // and message text after M0 is allowed only on ';'-style posts (Marlin).
    for (const char* refused : {"M0 Unlock spindle then resume", "M 30", "M 5", "M 6 T 3", "m5", "G0 x1"}) {
        INFO(refused);
        std::string post = "begin NEW_SEGMENT\n\"" + std::string(refused) + "\"\nend\n" + kMoves + kPause + kUnlock;
        CHECK_THAT(runError(parseText(post), {drag("A"), drag("B")}), ContainsSubstring("runs with the spindle off"));
    }
}

TEST_CASE("T-0154: the pause blocks must stop the spindle and pause", "[post][spindle-off]") {
    const std::string base = kSmall;
    struct Case {
        std::string pause;
        std::string unlock;
        const char* error;
    };
    const std::vector<Case> cases{
        {"begin SPINDLE_OFF_PAUSE\nend\n", kUnlock, "SPINDLE_OFF_PAUSE block writes no M5"},
        {"begin SPINDLE_OFF_PAUSE\n\"M0\"\nend\n", kUnlock, "SPINDLE_OFF_PAUSE block writes no M5"},
        {"begin SPINDLE_OFF_PAUSE\n\"M5\"\nend\n", kUnlock, "SPINDLE_OFF_PAUSE block writes no M0"},
        {"begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"(M0)\"\nend\n", kUnlock, "SPINDLE_OFF_PAUSE block writes no M0"},
        {"begin SPINDLE_OFF_PAUSE\n\"(M5) M0\"\nend\n", kUnlock, "SPINDLE_OFF_PAUSE block writes no M5"},
        {kPause, "begin SPINDLE_UNLOCK_PAUSE\nend\n", "SPINDLE_UNLOCK_PAUSE block writes no M5"},
        {kPause, "begin SPINDLE_UNLOCK_PAUSE\n\"M0\"\nend\n", "SPINDLE_UNLOCK_PAUSE block writes no M5"},
        // Block delete: a controller may skip lines starting with '/', so they do not count.
        {"begin SPINDLE_OFF_PAUSE\n\"/M5\"\n\"M0\"\nend\n", kUnlock, "SPINDLE_OFF_PAUSE block writes no M5"},
        {"begin SPINDLE_OFF_PAUSE\n\"M5\"\n\" / M0\"\nend\n", kUnlock, "SPINDLE_OFF_PAUSE block writes no M0"},
        {kPause, "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n\"/M0\"\nend\n", "SPINDLE_UNLOCK_PAUSE block writes no M0"},
        {kPause, "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n\"; M0\"\nend\n", "SPINDLE_UNLOCK_PAUSE block writes no M0"},
    };
    for (const Case& c : cases) {
        INFO(c.pause << c.unlock);
        const PostProcessor pp = parseText(base + c.pause + c.unlock);
        CHECK_THAT(runError(pp, {cut("Mill", spinning(1)), drag()}), ContainsSubstring(c.error));
        // Jobs without spindle-off toolpaths do not use the blocks.
        CHECK(runError(pp, {cut("Mill", spinning(1))}).empty());
    }
    // Zero-padded words are fine; spaced or lower-case ones count for the content check but the
    // allowlist then refuses writing them.
    const PostProcessor padded = parseText(base + "begin SPINDLE_OFF_PAUSE\n\"M05\"\n\"M00\"\nend\n" +
                                           "begin SPINDLE_UNLOCK_PAUSE\n\"M005\"\n\"M0\"\nend\n");
    CHECK(runError(padded, {cut("Mill", spinning(1)), drag()}).empty());
    const PostProcessor spaced = parseText(base + "begin SPINDLE_OFF_PAUSE\n\"M 05\"\n\"m 0\"\nend\n" + kUnlock);
    CHECK_THAT(runError(spaced, {cut("Mill", spinning(1)), drag()}), ContainsSubstring("in SPINDLE_OFF_PAUSE"));
}

TEST_CASE("T-0154: the post refuses a spindle-off toolpath whose tool is not a diamond drag tool",
          "[post][spindle-off][safety]") {
    for (const std::string& stem : shippedStems()) {
        INFO(stem);
        const PostProcessor pp = loadShipped(stem);
        for (const char* type : {"", "v_bit", "end_mill", "Diamond_Drag"}) {
            INFO(type);
            Toolpath crafted = drag("Crafted");
            crafted.tool.toolType = type;
            const std::string msg = runError(pp, {cut("Mill", spinning(1)), crafted});
            CHECK_THAT(msg, ContainsSubstring("toolpath 'Crafted' is marked to run with the spindle off"));
            CHECK_THAT(msg, ContainsSubstring("is not a diamond drag tool"));
        }
    }
}

TEST_CASE("T-0154: strict rule (a): output text must be printable ASCII", "[post][spindle-off][text]") {
    // Grbl drops bytes >= 0x80 (and acts on realtime ones), so these would run M3 / S1000.
    for (const char* attack : {"M\xC2\xA0" "3", "M\xC3\xA9" "3", "M\x9E" "3", "M\xFF" "3", "S\xC2\xA0" "1000",
                               "M\xE2\x80\x8B" "3", "(note \xC3\xA9)"}) {
        INFO(attack);
        const auto parsed = PostProcessor::parse("begin TOOLCHANGE\n\"" + std::string(attack) + "\"\nend\n" + kMoves);
        REQUIRE_FALSE(parsed.ok());
        CHECK_THAT(parsed.error().describe(), ContainsSubstring("output text must be printable ASCII"));
    }
    // Comment lines of the .pp file ('+') may still hold any text.
    CHECK(PostProcessor::parse("+ caf\xC3\xA9\n" + kMoves).ok());
}

TEST_CASE("T-0154: strict rule (b): comments end at the first ')' and may not nest", "[post][spindle-off][text]") {
    for (const char* attack : {"(a (b) M3 S1000", "((x)M3", "(open", "x) M5", "(a(b))"}) {
        INFO(attack);
        for (const char* block : {"TOOLCHANGE", "NEW_SEGMENT", "SPINDLE_OFF_PAUSE"}) {
            INFO(block);
            std::string post = kSmall + kUnlock;
            if (std::string(block) == "SPINDLE_OFF_PAUSE") {
                post += "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"" + std::string(attack) + "\"\n\"M0\"\nend\n";
            } else {
                post = "begin " + std::string(block) + "\n\"" + attack + "\"\nend\n" + kMoves + kPause + kUnlock;
            }
            const std::string msg =
                runError(parseText(post), {cut("Mill", spinning(1)), drag("A"), drag("B")});
            CHECK_THAT(msg, ContainsSubstring("runs with the spindle off"));
        }
    }
    // The same unbalanced text in a job without spindle-off toolpaths is the post author's business.
    CHECK(runError(parseText("begin NEW_SEGMENT\n\"(open\"\nend\n" + kMoves), {cut("A", spinning(1)), cut("B", spinning(1))})
              .empty());
}

TEST_CASE("T-0154: strict rule (c): no parameters or expressions in a drag section", "[post][spindle-off][text]") {
    for (const char* attack : {"#1=3", "M#1", "S#1", "M##1", "(#1)", "M [1+2]"}) {
        INFO(attack);
        std::string post = "begin NEW_SEGMENT\n\"" + std::string(attack) + "\"\nend\n" + kMoves + kPause + kUnlock;
        const auto parsed = PostProcessor::parse(post);
        if (!parsed.ok()) {
            // '[' starts a tag in the .pp grammar: "M [1+2]" is refused by the parser already.
            CHECK_THAT(parsed.error().describe(), ContainsSubstring("unknown variable tag"));
            continue;
        }
        const std::string msg = runError(parsed.value(), {drag("A"), drag("B")});
        CHECK_THAT(msg, ContainsSubstring("runs with the spindle off"));
    }
    // A value placed in live G-code cannot smuggle a parameter or expression either.
    std::string live = "begin NEW_SEGMENT\n\"G4 P1 [TOOLNAME]\"\nend\n" + kMoves + kPause + kUnlock;
    for (const char* name : {"#1", "[3]", "x M3"}) {
        INFO(name);
        std::vector<Toolpath> job{drag("A"), drag("B")};
        job[1].tool.name = name;
        job[0].tool.name = name;
        CHECK_THAT(runError(parseText(live), job), ContainsSubstring("runs with the spindle off"));
    }
}

TEST_CASE("T-0154: ';' is a comment only for ';' posts; values in comments may mention S", "[post][spindle-off][text]") {
    // A tool named "S12 drill" in a comment is fine on every shipped post (paren and ';' styles).
    for (const std::string& stem : shippedStems()) {
        INFO(stem);
        std::vector<Toolpath> job = test::dragToolpaths();
        for (Toolpath& tp : job) {
            tp.tool.name = tp.tool.spindleOff ? "S12 drill M3" : tp.tool.name;
        }
        CHECK(runError(loadShipped(stem), job).empty());
    }
    // A Marlin-style (';') post: a value after ';' is a comment.
    const std::string semi = "begin HEADER\n\"; [TOOLNAME]\"\nend\nbegin NEW_SEGMENT\n\"G4 P1 ; [TOOLNAME]\"\nend\n" +
                             kMoves + "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"; pause\"\n\"M0\"\nend\n" +
                             "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n\"; unlock\"\n\"M0\"\nend\n";
    std::vector<Toolpath> job{drag("A"), drag("B")};
    job[0].tool.name = job[1].tool.name = "x M3 S100";
    CHECK(runError(parseText(semi), job).empty());
    // The same lines in a paren-comment post: text after ';' counts as G-code for the check.
    const std::string paren = "begin HEADER\n\"(h)\"\nend\nbegin NEW_SEGMENT\n\"G4 P1 ; [TOOLNAME]\"\nend\n" + kMoves +
                              kPause + kUnlock;
    CHECK_THAT(runError(parseText(paren), job), ContainsSubstring("the character ';'"));
}

TEST_CASE("T-0154: drag-section allowlist refuses everything not on it", "[post][spindle-off][allowlist]") {
    const auto newSegment = [](const std::string& line) {
        return "begin NEW_SEGMENT\n\"" + line + "\"\nend\n" + kMoves + kPause + kUnlock;
    };
    // Round-3 repros: Grbl ignores '/' anywhere; fractional or out-of-range M codes that
    // controllers round, truncate or run by their integer part; and other characters.
    for (const char* attack :
         {"M/3", "S/1000", "G0 X1 /M3", "M3.004", "M4.004", "M3.00005", "M2.99995", "M3.5", "M259", "M0.5", "M5.0",
          "M+5", "M-0", "G0.004", "G28", "G54", "G92 X0", "G1 X1*", "X=1", "G0 %", "G0 $", "G0 @", "G0 ^", "G0 ~",
          "G0 !", "G0 ?", "(stop!)", "(why?)", "S100", "D1", "E5", "R2", "K1", "A90", "Q1", "L2", "O100", "M98 P1",
          "M100", "G0 X--1", "G0 X1.2.3", "G0 X.5", "T-1", "T1.0", "H+1", "N-10", "G 0", "G0X1Y2Z"}) {
        INFO(attack);
        const auto parsed = PostProcessor::parse(newSegment(attack));
        REQUIRE(parsed.ok());
        CHECK_THAT(runError(parsed.value(), {drag("A"), drag("B")}), ContainsSubstring("runs with the spindle off"));
    }
    // On the allowlist: the codes the shipped posts write in drag sections, also zero-padded.
    for (const char* allowed :
         {"G0 X1.5 Y-2 Z+0.100 F400", "G1 X1", "G2 X1 Y1 I0.5 J0", "G3 X0 Y0 I-0.5 J0", "G4 P0.25", "G17 G20 G21",
          "G40 G49 G80", "G90 G91.1 G94", "G91", "G43 H3", "T3 M6", "M5", "M0", "M00", "M1", "M2", "M30", "M05",
          "N10 G0 Z5", "  G0  Z5  ", "G01 X1", "G091.10", "M00005", "N99999", "X12345678.123456"}) {
        INFO(allowed);
        CHECK(runError(parseText(newSegment(allowed)), {drag("A"), drag("B")}).empty());
    }
    // A tool name rendered into live G-code falls under the same allowlist.
    const std::string live = "begin NEW_SEGMENT\n\"G1 Z-0.100 F200 [TOOLNAME]\"\nend\n" + kMoves + kPause + kUnlock;
    for (const char* name : {"M/3", "M3.004", "M259", "S1", "x"}) {
        INFO(name);
        std::vector<Toolpath> job{drag("A"), drag("B")};
        job[0].tool.name = job[1].tool.name = name;
        CHECK_THAT(runError(parseText(live), job), ContainsSubstring("runs with the spindle off"));
    }
}

TEST_CASE("T-0154: Marlin-style posts: one command per line, emergency-parser codes",
          "[post][spindle-off][allowlist]") {
    const std::string semiMoves = kMoves + "begin HEADER\n\"; [TOOLPATH_NAME]\"\nend\n";
    const std::string semiPause = "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"; Drag tool [T]: spindle off\"\n\"M0\"\nend\n";
    const std::string semiUnlock = "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n\"; Unlock the spindle\"\n\"M0\"\nend\n";
    CHECK(runError(parseText(semiMoves + semiPause + semiUnlock), {drag()}).empty());
    // Emergency-parser codes are refused anywhere in template text, comments included.
    for (const char* attack : {"; M410", "G4 P1 ; M112", "; resume with M 108", "; M0876"}) {
        INFO(attack);
        const std::string post =
            semiMoves + "begin NEW_SEGMENT\n\"" + std::string(attack) + "\"\nend\n" + semiPause + semiUnlock;
        CHECK_THAT(runError(parseText(post), {drag("A"), drag("B")}), ContainsSubstring("emergency-parser code"));
    }
    // Marlin runs one command per line: "M5 M0" on one line does not count as the pause's M0 (and
    // an M0 line must be bare anyway).
    const std::string oneLine = "begin SPINDLE_OFF_PAUSE\n\"M5 M0\"\nend\n";
    CHECK_THAT(runError(parseText(semiMoves + oneLine + semiUnlock), {drag()}),
               ContainsSubstring("SPINDLE_OFF_PAUSE block writes no M0"));
}

TEST_CASE("T-0154 bare M0: an M0/M1 line in a drag section holds only [N] M0 and one comment",
          "[post][spindle-off][allowlist]") {
    const std::string semiHeader = "begin HEADER\n\"; [TOOLPATH_NAME]\"\nend\n";
    const std::string semiUnlock = "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n\"M0\"\nend\n";
    const auto semiPost = [&](const std::string& pauseLine) {
        return kMoves + semiHeader + "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"" + pauseLine + "\"\nend\n" + semiUnlock;
    };
    const auto parenPost = [&](const std::string& pauseLine) {
        return kMoves + "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"" + pauseLine + "\"\nend\n" + kUnlock;
    };
    // postatk6: message text (Marlin's FASTER_GCODE_PARSER reads its upper-case letters as
    // parameters: "M0 SPINDLE STOP 1" is P1, "M0 TOOL UP [T]" is P3), digits ("M0 3" is M03 on
    // Grbl/LinuxCNC), and any other word ("M0 P1", "M0 S1", "M0 T[T]", "M5 M0").
    for (const char* line : {"M0 SPINDLE STOP 1", "M0 UNLOCK UP 5", "M0 TOOL UP [T]", "M0 3", "M0 Drag tool [T] off",
                             "M0 P1", "M0 S1", "M0 T[T]", "M0 H1", "M5 M0", "M0 M0", "G4 P1 M0", "M1 go", "M00 X",
                             "M0 [TOOLNAME]", "M0 Check [TOOLPATH_NAME]"}) {
        INFO(line);
        CHECK_THAT(runError(parseText(semiPost(line)), {drag()}), ContainsSubstring("runs with the spindle off"));
        CHECK_THAT(runError(parseText(parenPost(line)), {drag()}), ContainsSubstring("runs with the spindle off"));
    }
    // Bare forms, with one comment, are fine.
    for (const char* line : {"M0", "M00", "M0 ; pause"}) {
        INFO(line);
        CHECK(runError(parseText(semiPost(line)), {drag()}).empty());
    }
    for (const char* line : {"M0", "N10 M0", "M0 (pause)", "(pause) M0"}) {
        INFO(line);
        CHECK(runError(parseText(parenPost(line)), {drag()}).empty());
    }
    CHECK_THAT(runError(parseText(parenPost("(a) M0 (b)")), {drag()}), ContainsSubstring("more than one comment"));
    // A VAR prefix cannot add a parameter to the M0 line either (postatk6 "TOOLS " -> S3).
    const std::string prefixed = "VAR TOOL_NUMBER = [T|A|TOOLS |1.0]\n" + semiPost("M0 [T]");
    CHECK_THAT(runError(parseText(prefixed), {drag()}), ContainsSubstring("runs with the spindle off"));
    // A text tag in a ';' comment before M0 is fine (the shipped Marlin style).
    std::vector<Toolpath> job{drag()};
    job[0].tool.name = "M3 S18000 P1";
    CHECK(runError(parseText(kMoves + semiHeader + "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"; Load [TOOLNAME]\"\n\"M0\"\nend\n" +
                             semiUnlock),
                   job)
              .empty());
    // N5: on a ';' post a '(' in live code is refused (Marlin without PAREN_COMMENTS).
    std::vector<Toolpath> paren{drag()};
    paren[0].tool.name = "( S1000)";
    const std::string a10 = semiPost("M0") + "begin PLUNGE_MOVE\n\"G1[Z][F] [TOOLNAME]\"\nend\n";
    CHECK_THAT(runError(parseText(a10), paren), ContainsSubstring("parenthesis outside the ';' comment"));
}

TEST_CASE("T-0154: long Mach3 drag jobs pass (line numbers above 99999)", "[post][spindle-off][allowlist]") {
    // 12000 drag moves: Mach3 numbers lines N10, N20, ... beyond N100000.
    Toolpath big = drag("Long");
    big.moves = {Move::rapid({0, 0, 5}), Move::plunge({0, 0, -0.1})};
    for (int k = 1; k <= 12000; ++k) {
        big.moves.push_back(Move::feed({static_cast<double>(k % 100), static_cast<double>(k / 100), -0.1}));
    }
    big.moves.push_back(Move::retract({0, 120, 5}));
    for (const std::string& stem : shippedStems()) {
        INFO(stem);
        const std::string err = runError(loadShipped(stem), {cut("Mill", spinning(1)), big, cut("Mill", spinning(1))});
        CHECK(err.empty());
        INFO(err);
    }
    const auto r = loadShipped("mach3_mm").run({big}, test::fixtureContext());
    REQUIRE(r.ok());
    CHECK(r.value().text.find("N100000 ") != std::string::npos);
}

TEST_CASE("T-0154 round 4: LINE_ENDING may only be LF, CR LF or CR", "[post][spindle-off][allowlist]") {
    // A1-A1d: the line ending is written after every line.
    for (const char* ending : {" M3 S12000[13][10]", "[10]M3[10]", "~[10]", "[10][133]", "[10][10]", "x"}) {
        INFO(ending);
        const auto parsed = PostProcessor::parse("LINE_ENDING = \"" + std::string(ending) + "\"\n" + kMoves);
        REQUIRE_FALSE(parsed.ok());
        CHECK_THAT(parsed.error().describe(), ContainsSubstring("LF, CR LF or CR"));
    }
    for (const char* ending : {"[10]", "[13][10]", "[13]"}) {
        INFO(ending);
        CHECK(PostProcessor::parse("LINE_ENDING = \"" + std::string(ending) + "\"\n" + kMoves).ok());
    }
}

TEST_CASE("T-0154 round 4: pause content is judged on the written lines; no block delete",
          "[post][spindle-off][allowlist]") {
    // A3: a VAR prefix that makes a line start with '/'.
    const std::string slashN = "VAR LINE_NUMBER = [N|A|/N|1.0]\n" + kMoves +
                               "begin SPINDLE_OFF_PAUSE\n\"[N] M5\"\n\"[N] M0\"\nend\n" +
                               "begin SPINDLE_UNLOCK_PAUSE\n\"[N] M5\"\n\"[N] M0\"\nend\n";
    CHECK_THAT(runError(parseText(slashN), {drag()}), ContainsSubstring("the character '/'"));
    // A4/A4b: "M0[T]" writes M05 or M03.
    for (const int tool : {5, 3}) {
        INFO(tool);
        std::vector<Toolpath> job{drag()};
        job[0].tool.toolNumber = tool;
        const std::string post = "VAR TOOL_NUMBER = [T|A||1.0]\n" + kMoves +
                                 "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"M0[T]\"\nend\n" + kUnlock;
        CHECK_THAT(runError(parseText(post), job), ContainsSubstring("runs with the spindle off"));
    }
    // A5/A5b: digit caps.
    for (const char* line : {"M000000005", "G00000091.1", "T123456", "N1234567890", "X123456789", "X1.1234567"}) {
        INFO(line);
        const std::string post = "begin NEW_SEGMENT\n\"" + std::string(line) + "\"\nend\n" + kMoves + kPause + kUnlock;
        CHECK_THAT(runError(parseText(post), {drag("A"), drag("B")}), ContainsSubstring("too many digits"));
    }
    // A9: a format that pads coordinates past 8 integer digits.
    const std::string padded = "VAR SAFE_Z = [SAFEZ|A| Z|20.3]\n" + kMoves +
                               "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"G0[SAFEZ]\"\n\"M0\"\nend\n" + kUnlock;
    CHECK_THAT(runError(parseText(padded), {drag()}), ContainsSubstring("too many digits"));
}

TEST_CASE("T-0154: no tabs in drag sections; no N words on ';'-style posts", "[post][spindle-off][allowlist]") {
    // postatk7 S6/S7/T1b: Marlin keeps tabs, so "N0<TAB>M0" or "<TAB>M0" is an unknown command and
    // the pause (and in T1b the M5) would be skipped.
    const std::string semiHeader = "begin HEADER\n\"; [TOOLPATH_NAME]\"\nend\n";
    const std::string semiUnlock = "begin SPINDLE_UNLOCK_PAUSE\n\"M5\"\n\"M0\"\nend\n";
    const auto semiPause = [&](const std::string& lines) {
        return kMoves + semiHeader + "begin SPINDLE_OFF_PAUSE\n" + lines + "end\n" + semiUnlock;
    };
    for (const char* lines : {"\"M5\"\n\"N0\tM0\"\n", "\"M5\"\n\"\tM0\"\n", "\"\tM5\"\n\"\tM0\"\n"}) {
        INFO(lines);
        CHECK_THAT(runError(parseText(semiPause(lines)), {drag()}), ContainsSubstring("a tab"));
    }
    // Tabs are refused on paren posts too (no shipped post writes them).
    CHECK_THAT(runError(parseText(kMoves + "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"\tM0\"\nend\n" + kUnlock), {drag()}),
               ContainsSubstring("a tab"));
    // N words on a ';' post: hosts add their own N...*checksum and Marlin skips only one.
    for (const char* lines : {"\"M5\"\n\"N1 M0\"\n", "\"N1 M5\"\n\"M0\"\n"}) {
        INFO(lines);
        CHECK_THAT(runError(parseText(semiPause(lines)), {drag()}), ContainsSubstring("an N word"));
    }
    const std::string numbered = "VAR LINE_NUMBER = [N|A|N|1.0]\n" + kMoves + semiHeader +
                                 "begin RAPID_MOVE\n\"[N] G0[X][Y][Z]\"\nend\n" +
                                 "begin SPINDLE_OFF_PAUSE\n\"M5\"\n\"M0\"\nend\n" + semiUnlock;
    CHECK_THAT(runError(parseText(numbered), {drag()}), ContainsSubstring("an N word"));
    // N words stay fine on paren posts (Mach3 numbers its lines).
    CHECK(runError(parseText(kMoves + "begin SPINDLE_OFF_PAUSE\n\"N10 M5\"\n\"N20 M0\"\nend\n" + kUnlock), {drag()})
              .empty());
}
