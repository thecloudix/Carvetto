#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"

using namespace carvetto::post;
using carvetto::geometry::kPi;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;
using carvetto::toolpath::Move;
using carvetto::toolpath::Toolpath;
using carvetto::toolpath::ToolSnapshot;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

PostProcessor parsePost(const std::string& text) {
    auto r = PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("parse failed: " << r.error().describe());
    }
    return r.value();
}

ToolSnapshot tool(int number, std::string name = "") {
    ToolSnapshot t;
    t.name = name.empty() ? "Tool " + std::to_string(number) : name;
    t.toolNumber = number;
    t.diameter = 6.0;
    t.feedRate = 1000.0;
    t.plungeRate = 300.0;
    t.spindleSpeed = 18000.0;
    return t;
}

Toolpath toolpathOf(std::string name, ToolSnapshot t, std::vector<Move> moves) {
    Toolpath tp;
    tp.name = std::move(name);
    tp.tool = std::move(t);
    tp.moves = std::move(moves);
    return tp;
}

std::vector<std::string> lines(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream in(text);
    std::string l;
    while (std::getline(in, l)) {
        out.push_back(l);
    }
    return out;
}

std::string runOk(const PostProcessor& pp, const std::vector<Toolpath>& tps, const PostContext& ctx = {}) {
    auto r = pp.run(tps, ctx);
    if (!r.ok()) {
        FAIL("run failed: " << r.error().describe());
    }
    return r.value().text;
}

/// Reads the value after `letter` in a G-code line (NaN if absent).
double word(const std::string& line, char letter) {
    const std::size_t i = line.find(letter);
    if (i == std::string::npos) {
        return std::nan("");
    }
    return std::stod(line.substr(i + 1));
}

// A small all-absolute post: every coordinate written on every line, for easy parsing.
const std::string kAbsolute = R"PP(
VAR X = [X|A| X|1.4]
VAR Y = [Y|A| Y|1.4]
VAR Z = [Z|A| Z|1.4]
VAR F = [F|A| F|1.1]
VAR I = [I|A| I|1.4]
VAR J = [J|A| J|1.4]
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G1[X][Y][Z][F]"
end
)PP";

const std::string kArcBlocks = R"PP(
begin CW_ARC_MOVE
"G2[X][Y][Z][I][J][F]"
end
begin CCW_ARC_MOVE
"G3[X][Y][Z][I][J][F]"
end
)PP";

}  // namespace

TEST_CASE("AC2: [X|A|X|1.3] in a block writes X12.346", "[post][run]") {
    const PostProcessor pp = parsePost("VAR XP = [X|A|X|1.3]\nbegin RAPID_MOVE\n\"G0 [X]\"\nend\n");
    const std::string out = runOk(pp, {toolpathOf("t", tool(1), {Move::rapid({12.3456, 0, 5})})});
    CHECK(out == "G0 X12.346\n");
}

TEST_CASE("AC2: incremental mode renders deltas", "[post][run]") {
    const PostProcessor pp = parsePost(R"PP(
VAR XI = [X|AI| X|1.3]
VAR YI = [Y|CI| Y|1.3]
VAR ZI = [Z|AI| Z|1.3]
begin HEADER
"G91[X]"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G1[X][Y][Z]"
end
)PP");
    PostContext ctx;
    ctx.home = {1.0, 1.0, 10.0};
    const std::string out = runOk(pp,
                                  {toolpathOf("t", tool(1),
                                              {Move::rapid({11.0, 1.0, 5.0}), Move::feed({11.0, 4.5, 5.0}),
                                               Move::feed({8.25, 4.5, 2.0})})},
                                  ctx);
    // Header is stationary: delta 0. Y is CI, so a zero delta is omitted.
    CHECK(lines(out) == std::vector<std::string>{"G91 X0.000", "G0 X10.000 Z-5.000",
                                                 "G1 X0.000 Y3.500 Z0.000", "G1 X-2.750 Z-3.000"});
}

TEST_CASE("Incremental deltas telescope without rounding drift", "[post][run]") {
    const PostProcessor pp = parsePost("VAR XI = [X|AI|X|1.1]\nbegin FEED_MOVE\n\"[X]\"\nend\n");
    std::vector<Move> moves;
    for (int i = 1; i <= 10; ++i) {
        moves.push_back(Move::feed({0.14 * i, 0, 0}));  // 0.14 rounds to 0.1 each time if done naively
    }
    double sum = 0.0;
    for (const std::string& l : lines(runOk(pp, {toolpathOf("t", tool(1), moves)}))) {
        sum += std::stod(l.substr(1));
    }
    CHECK_THAT(sum, WithinAbs(1.4, 1e-9));
}

TEST_CASE("Modal (C) variables are suppressed when unchanged and reset per toolpath", "[post][run]") {
    const PostProcessor pp = parsePost(R"PP(
VAR X = [X|C| X|1.1]
VAR Y = [Y|C| Y|1.1]
VAR F = [F|C| F|1.0]
begin FEED_MOVE
"G1[X][Y][F]"
end
begin NEW_SEGMENT
"(next)"
end
)PP");
    const std::vector<Move> moves{Move::feed({1, 0, 0}), Move::feed({1, 2, 0}), Move::feed({1, 2, 0})};
    const std::string out =
        runOk(pp, {toolpathOf("a", tool(1), moves), toolpathOf("b", tool(1), {Move::feed({1, 2, 0})})});
    CHECK(lines(out) ==
          std::vector<std::string>{"G1 X1.0 Y0.0 F1000", "G1 Y2.0", "G1", "(next)", "G1 X1.0 Y2.0 F1000"});
}

TEST_CASE("A line whose variables are all suppressed is dropped; literal empty lines are kept",
          "[post][run]") {
    const PostProcessor pp = parsePost("VAR X = [X|C|X|1.1]\nbegin FEED_MOVE\n\"[X]\"\n\"\"\nend\n");
    const std::string out =
        runOk(pp, {toolpathOf("t", tool(1), {Move::feed({1, 0, 0}), Move::feed({1, 0, 0})})});
    CHECK(out == "X1.0\n\n\n");
}

TEST_CASE("AC3: arcs use CW/CCW_ARC_MOVE with I/J relative to the start", "[post][run][arc]") {
    const PostProcessor pp = parsePost(kAbsolute + kArcBlocks);
    const std::string out =
        runOk(pp, {toolpathOf("t", tool(1),
                              {Move::rapid({10, 0, -1}), Move::arcCCW({0, 10, -1}, {0, 0}),  // quarter CCW
                               Move::arcCW({10, 0, -1}, {0, 0}),                             // back CW
                               Move::arcCCW({20, 10, -1}, {15, 5})})});                      // offset centre
    const auto l = lines(out);
    REQUIRE(l.size() == 4);
    CHECK(l[1] == "G3 X0.0000 Y10.0000 Z-1.0000 I-10.0000 J0.0000 F1000.0");
    CHECK(l[2] == "G2 X10.0000 Y0.0000 Z-1.0000 I0.0000 J-10.0000 F1000.0");
    CHECK(l[3] == "G3 X20.0000 Y10.0000 Z-1.0000 I5.0000 J5.0000 F1000.0");
}

TEST_CASE("Arc tags: IA/JA, start, mid, signed radius, angle", "[post][run][arc]") {
    const PostProcessor pp = parsePost(kAbsolute + R"PP(
VAR R = [Radius|A| R|1.3]
begin CCW_ARC_MOVE
"[IA] [JA] [ArcStartX] [ArcStartY] [ArcMidX] [ArcMidY] [Radius] [Angle]"
end
begin CW_ARC_MOVE
"[Radius] [Angle]"
end
)PP");
    const std::string out = runOk(pp, {toolpathOf("t", tool(1),
                                                  {Move::rapid({12, 2, 0}), Move::arcCCW({2, 12, 0}, {2, 2}),
                                                   Move::arcCW({-8, 2, 0}, {2, 2})})});
    const auto l = lines(out);
    REQUIRE(l.size() == 3);
    // Quarter CCW about (2,2) from (12,2): mid at 45 degrees.
    CHECK(l[1] == "2.000 2.000 12.000 2.000 9.071 9.071  R10.000 90.000");
    // CW from (2,12) to (-8,2) about (2,2) is 270 degrees: R-form radius is negative.
    CHECK(l[2] == " R-10.000 270.000");
}

TEST_CASE("Helical arcs use the helical block when present, otherwise the arc block with Z",
          "[post][run][arc]") {
    const std::vector<Move> helix{Move::rapid({10, 0, 0}), Move::arcCCW({10, 0, -2}, {0, 0}),
                                  Move::arcCCW({10, 0, -4}, {0, 0}), Move::arcCCW({0, 10, -4}, {0, 0})};
    SECTION("with helical blocks") {
        const PostProcessor pp = parsePost(kAbsolute + kArcBlocks + R"PP(
begin FIRST_CCW_HELICAL_ARC_PLUNGE_MOVE
"HPLUNGE[Z]"
end
begin CCW_HELICAL_ARC_MOVE
"HELIX[Z]"
end
)PP");
        const auto l = lines(runOk(pp, {toolpathOf("t", tool(1), helix)}));
        REQUIRE(l.size() == 4);
        CHECK(l[1] == "HPLUNGE Z-2.0000");
        CHECK(l[2] == "HELIX Z-4.0000");
        CHECK(l[3] == "G3 X0.0000 Y10.0000 Z-4.0000 I-10.0000 J0.0000 F1000.0");
    }
    SECTION("without helical blocks: full-circle helix through the arc block") {
        const PostProcessor pp = parsePost(kAbsolute + kArcBlocks);
        const auto l = lines(runOk(pp, {toolpathOf("t", tool(1), helix)}));
        REQUIRE(l.size() == 4);
        CHECK(l[1] == "G3 X10.0000 Y0.0000 Z-2.0000 I-10.0000 J0.0000 F1000.0");
    }
}

TEST_CASE("AC3: without arc blocks arcs are linearised within the tolerance", "[post][run][arc]") {
    const PostProcessor pp = parsePost(kAbsolute);
    PostContext ctx;
    ctx.arcTolerance = 0.01;
    const double r = 20.0;
    const Vec2 c{5.0, -3.0};
    const std::string out =
        runOk(pp,
              {toolpathOf("t", tool(1),
                          {Move::rapid({c.x + r, c.y, 0}), Move::arcCW({c.x, c.y - r, -2}, c, 600.0)})},
              ctx);
    const auto l = lines(out);
    REQUIRE(l.size() > 10);
    double prevX = c.x + r;
    double prevY = c.y;
    double prevZ = 0.0;
    for (std::size_t i = 1; i < l.size(); ++i) {
        INFO(l[i]);
        REQUIRE(l[i].rfind("G1", 0) == 0);
        const double x = word(l[i], 'X');
        const double y = word(l[i], 'Y');
        const double z = word(l[i], 'Z');
        CHECK(word(l[i], 'F') == 600.0);
        CHECK_THAT(std::hypot(x - c.x, y - c.y), WithinAbs(r, 1e-4));  // vertices on the circle
        // Chord midpoint deviation from the arc stays within tolerance (+ print rounding).
        const double mx = (x + prevX) / 2.0 - c.x;
        const double my = (y + prevY) / 2.0 - c.y;
        CHECK(r - std::hypot(mx, my) <= 0.01 + 1e-4);
        CHECK(z <= prevZ + 1e-9);  // helix descends monotonically
        prevX = x;
        prevY = y;
        prevZ = z;
    }
    CHECK(l.back() == "G1 X5.0000 Y-23.0000 Z-2.0000 F600.0");  // exact end point
    // CW from 0 to -90 degrees sweeps a quarter turn.
    const double maxStep = 2.0 * std::acos(1.0 - 0.01 / r);
    CHECK(static_cast<double>(l.size() - 1) == std::ceil((kPi / 2.0) / maxStep));
}

TEST_CASE("Arcs outside MIN/MAX_ARC_RADIUS are linearised", "[post][run][arc]") {
    const PostProcessor pp = parsePost("MIN_ARC_RADIUS = 1\nMAX_ARC_RADIUS = 100\n" + kAbsolute + kArcBlocks);
    const auto small = lines(runOk(
        pp, {toolpathOf("t", tool(1), {Move::rapid({0.5, 0, 0}), Move::arcCCW({-0.5, 0, 0}, {0, 0})})}));
    CHECK(small.size() > 2);
    for (std::size_t i = 1; i < small.size(); ++i) {
        CHECK(small[i].rfind("G1", 0) == 0);
    }
    const auto big = lines(
        runOk(pp, {toolpathOf("t", tool(1), {Move::rapid({200, 0, 0}), Move::arcCCW({0, 200, 0}, {0, 0})})}));
    CHECK(big.size() > 2);
    CHECK(big[1].rfind("G1", 0) == 0);
    const auto ok = lines(
        runOk(pp, {toolpathOf("t", tool(1), {Move::rapid({50, 0, 0}), Move::arcCCW({0, 50, 0}, {0, 0})})}));
    REQUIRE(ok.size() == 2);
    CHECK(ok[1].rfind("G3", 0) == 0);
}

TEST_CASE("R-form posts write a full circle as two half circles", "[post][run][arc]") {
    const PostProcessor pp = parsePost(kAbsolute + R"PP(
VAR R = [Radius|A| R|1.3]
begin CW_ARC_MOVE
"G2[X][Y][Radius]"
end
)PP");
    const auto l = lines(
        runOk(pp, {toolpathOf("t", tool(1), {Move::rapid({10, 0, 0}), Move::arcCW({10, 0, 0}, {0, 0})})}));
    REQUIRE(l.size() == 3);
    CHECK(l[1] == "G2 X-10.0000 Y0.0000 R10.000");
    CHECK(l[2] == "G2 X10.0000 Y0.0000 R10.000");
}

TEST_CASE("AC5: ROTARY_WRAP_Y wraps Y into A (Y = pi*25 on a 50 mm cylinder is 180 degrees)",
          "[post][run][rotary]") {
    const std::string body = R"PP(
VAR X = [X|A| X|1.3]
VAR Y = [Y|A| Y|1.3]
VAR Z = [Z|A| Z|1.3]
VAR A = [A|A| A|1.3]
begin RAPID_MOVE
"G0[X][Y][Z][A]"
end
begin FEED_MOVE
"G1[X][Y][Z][A]"
end
begin CW_ARC_MOVE
"G2[X][Y][I][J]"
end
)PP";
    PostContext ctx;
    ctx.rotaryDiameter = 50.0;
    const std::vector<Toolpath> tps{toolpathOf("t", tool(1), {Move::rapid({10, kPi * 25.0, 3})})};

    SECTION("positive") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n" + body);
        CHECK(runOk(pp, tps, ctx) == "G0 X10.000 Z3.000 A180.000\n");
    }
    SECTION("negated") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"-A\"\n" + body);
        CHECK(runOk(pp, tps, ctx) == "G0 X10.000 Z3.000 A-180.000\n");
    }
    SECTION("ROTARY_WRAP_X wraps X instead") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_X = \"A\"\n" + body);
        const std::vector<Toolpath> xs{toolpathOf("t", tool(1), {Move::rapid({kPi * 50.0, 7, 3})})};
        CHECK(runOk(pp, xs, ctx) == "G0 Y7.000 Z3.000 A360.000\n");
    }
    SECTION("arcs are linearised when wrapping") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n" + body);
        const auto l = lines(
            runOk(pp, {toolpathOf("t", tool(1), {Move::rapid({10, 0, 0}), Move::arcCW({0, -10, 0}, {0, 0})})},
                  ctx));
        CHECK(l.size() > 2);
        for (std::size_t i = 1; i < l.size(); ++i) {
            CHECK(l[i].rfind("G1", 0) == 0);
        }
    }
    SECTION("a wrapping post without a rotary diameter is an error") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n" + body);
        const auto r = pp.run(tps, PostContext{});
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().message, ContainsSubstring("rotary diameter"));
    }
}

TEST_CASE("Without rotary wrap, [A] is 0", "[post][run][rotary]") {
    const PostProcessor pp = parsePost("begin RAPID_MOVE\n\"[A]\"\nend\n");
    CHECK(runOk(pp, {toolpathOf("t", tool(1), {Move::rapid({1, 2, 3})})}) == "0.000\n");
}

TEST_CASE("Line numbering increments and wraps at LINE_NUMBER_MAXIMUM", "[post][run][lines]") {
    const PostProcessor pp = parsePost(R"PP(
LINE_NUMBER_START = 10
LINE_NUMBER_INCREMENT = 10
LINE_NUMBER_MAXIMUM = 30
VAR N = [N|A|N|1.0]
begin HEADER
"[N] G21"
end
begin FEED_MOVE
"[N] G1 [X]"
end
begin FOOTER
"[N] M30"
end
)PP");
    const std::string out = runOk(pp, {toolpathOf("t", tool(1),
                                                  {Move::feed({1, 0, 0}), Move::feed({2, 0, 0}),
                                                   Move::feed({3, 0, 0}), Move::feed({4, 0, 0})})});
    CHECK(lines(out) == std::vector<std::string>{"N10 G21", "N20 G1 1.000", "N30 G1 2.000", "N10 G1 3.000",
                                                 "N20 G1 4.000", "N30 M30"});
}

TEST_CASE("Line numbering defaults start at 0 step 1; LINE_ENDING is applied", "[post][run][lines]") {
    const PostProcessor pp = parsePost(
        "LINE_ENDING = \"[13][10]\"\nbegin HEADER\n\"N[N]\"\n\"N[N]\"\nend\nbegin FOOTER\n\"N[N]\"\nend\n");
    CHECK(runOk(pp, {}) == "N0\r\nN1\r\nN2\r\n");
}

TEST_CASE("Inch output scales lengths and feeds but not speeds, angles or counts", "[post][run][units]") {
    const PostProcessor pp = parsePost(R"PP(
UNITS = "INCHES"
VAR X = [X|A|X|1.4]
VAR Z = [Z|A|Z|1.4]
VAR F = [F|A|F|1.1]
VAR S = [S|A|S|1.0]
VAR TD = [TDIA|A|D|1.3]
VAR SZ = [SAFEZ|A|Z|1.3]
VAR XM = [XLENGTH|A|L|1.3]
VAR MULT = [FC|A|C|1.3|2]
begin HEADER
"[TDIA] [SAFEZ] [XLENGTH] [S] [T] [FC]"
end
begin FEED_MOVE
"G1 [X] [Z] [F]"
end
begin PLUNGE_MOVE
"G1 [Z] [F] [FP]"
end
begin CW_ARC_MOVE
"G2 [I] [J] [Angle]"
end
begin DWELL_MOVE
"G4 [DWELL]"
end
)PP");
    PostContext ctx;
    ctx.safeZ = 12.7;
    ctx.materialMin = {0, 0, -19.05};
    ctx.materialMax = {254, 127, 0};
    ToolSnapshot t = tool(3);
    t.diameter = 6.35;
    t.feedRate = 2540.0;
    t.plungeRate = 254.0;
    const std::string out = runOk(pp,
                                  {toolpathOf("t", t,
                                              {Move::plunge({0, 0, -6.35}), Move::feed({25.4, 0, -6.35}),
                                               Move::arcCW({25.4, -50.8, -6.35}, {25.4, -25.4}),
                                               Move::dwell({25.4, -50.8, -6.35}, 1.5)})},
                                  ctx);
    // multiplier (2) applies after unit conversion: 2540 mm/min -> 100 in/min -> 200.
    CHECK(lines(out) == std::vector<std::string>{"D0.250 Z0.500 L10.000 S18000 3 C200.000",
                                                 "G1 Z-0.2500 F10.0 10.000", "G1 X1.0000 Z-0.2500 F100.0",
                                                 "G2 0.000 -1.000 180.000", "G4 1.500"});
}

TEST_CASE("Inverse-time mode: [F] and [FI] give 1/minutes for cutting moves", "[post][run][feed]") {
    const PostProcessor pp = parsePost(R"PP(
INVERSE_TIME_MODE = "YES"
UNITS = "INCHES"
VAR F = [F|A|F|1.3]
VAR FI = [FI|A|Q|1.3]
begin RAPID_MOVE
"G0 [X]"
end
begin FEED_MOVE
"G93 G1 [F] [FI]"
end
)PP");
    // 500 mm at 1000 mm/min = 0.5 min -> F2 (no inch scaling for inverse time).
    const std::string out =
        runOk(pp, {toolpathOf("t", tool(1), {Move::rapid({0, 0, 0}), Move::feed({500, 0, 0})})});
    CHECK(lines(out) == std::vector<std::string>{"G0 0.000", "G93 G1 F2.000 Q2.000"});
}

TEST_CASE("FEED_RATE_CHANGE is written only when the feed changes", "[post][run][feed]") {
    const PostProcessor pp = parsePost(R"PP(
begin FEED_RATE_CHANGE
"F[F]"
end
begin FEED_MOVE
"G1 [X]"
end
begin PLUNGE_MOVE
"G1 [Z]"
end
)PP");
    const std::string out = runOk(pp, {toolpathOf("t", tool(1),
                                                  {Move::plunge({0, 0, -1}), Move::feed({1, 0, -1}),
                                                   Move::feed({2, 0, -1}), Move::feed({3, 0, -1}, 250.0)})});
    CHECK(lines(out) == std::vector<std::string>{"F300.000", "G1 -1.000", "F1000.000", "G1 1.000", "G1 2.000",
                                                 "F250.000", "G1 3.000"});
}

TEST_CASE("Missing blocks fall back or are skipped", "[post][run][fallback]") {
    const std::vector<Move> moves{Move::rapid({0, 0, 5}), Move::plunge({0, 0, -1}), Move::feed({1, 0, -1}),
                                  Move::dwell({1, 0, -1}, 2.0), Move::retract({1, 0, 5})};
    SECTION("only FEED_MOVE: everything moves through it; dwell dropped with a warning") {
        const PostProcessor pp = parsePost("begin FEED_MOVE\n\"G1 [X] [Z] [F]\"\nend\n");
        auto r = pp.run({toolpathOf("t", tool(1), moves)}, PostContext{});
        REQUIRE(r.ok());
        CHECK(lines(r.value().text) ==
              std::vector<std::string>{"G1 0.000 5.000 1000.000", "G1 0.000 -1.000 300.000",
                                       "G1 1.000 -1.000 1000.000", "G1 1.000 5.000 300.000"});
        REQUIRE(r.value().warnings.size() == 2);
        CHECK_THAT(r.value().warnings[0].message, ContainsSubstring("RAPID_MOVE"));
        CHECK_THAT(r.value().warnings[1].message, ContainsSubstring("DWELL_MOVE"));
    }
    SECTION("no FEED_MOVE and a move that needs it is an error") {
        const PostProcessor pp = parsePost("begin RAPID_MOVE\n\"G0\"\nend\n");
        const auto r = pp.run({toolpathOf("t", tool(1), moves)}, PostContext{});
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().message, ContainsSubstring("plunge") && ContainsSubstring("FEED_MOVE"));
    }
    SECTION("no HEADER, FOOTER, TOOLCHANGE or NEW_SEGMENT: nothing extra is written") {
        const PostProcessor pp = parsePost("begin RAPID_MOVE\n\"G0 [X]\"\nend\n");
        const std::string out = runOk(pp, {toolpathOf("a", tool(1), {Move::rapid({1, 0, 0})}),
                                           toolpathOf("b", tool(2), {Move::rapid({2, 0, 0})}),
                                           toolpathOf("c", tool(2), {Move::rapid({3, 0, 0})})});
        CHECK(out == "G0 1.000\nG0 2.000\nG0 3.000\n");
    }
}

TEST_CASE("FIRST_* and INITIAL_* blocks apply to the first move of each toolpath", "[post][run][fallback]") {
    const PostProcessor pp = parsePost(R"PP(
begin INITIAL_RAPID_MOVE
"IR"
end
begin RAPID_MOVE
"R"
end
begin FIRST_FEED_MOVE
"FF"
end
begin FEED_MOVE
"F"
end
begin FIRST_PLUNGE_MOVE
"FP"
end
begin PLUNGE_MOVE
"P"
end
begin RETRACT_MOVE
"U"
end
begin NEW_SEGMENT
"--"
end
)PP");
    const std::vector<Move> moves{Move::rapid({0, 0, 5}),   Move::plunge({0, 0, -1}), Move::feed({1, 0, -1}),
                                  Move::feed({2, 0, -1}),   Move::retract({2, 0, 5}), Move::rapid({5, 0, 5}),
                                  Move::plunge({5, 0, -1}), Move::feed({6, 0, -1})};
    const std::string out = runOk(pp, {toolpathOf("a", tool(1), moves), toolpathOf("b", tool(1), moves)});
    const std::string one = "IR\nFP\nFF\nF\nU\nR\nP\nF\n";
    CHECK(out == one + "--\n" + one);
}

TEST_CASE("Empty toolpath list writes only HEADER and FOOTER", "[post][run][edge]") {
    const PostProcessor pp = parsePost(R"PP(
begin HEADER
"%"
"(T[T] [TOOLNAME])"
end
begin TOOLCHANGE
"M6"
end
begin RAPID_MOVE
"G0"
end
begin FOOTER
"M30"
end
)PP");
    CHECK(runOk(pp, {}) == "%\n(T0 )\nM30\n");
    // A toolpath without moves is skipped entirely, including its tool change.
    CHECK(runOk(pp, {toolpathOf("empty", tool(4), {})}) == "%\n(T0 )\nM30\n");
    CHECK(runOk(pp, {toolpathOf("a", tool(1), {Move::rapid({0, 0, 5})}), toolpathOf("empty", tool(4), {}),
                     toolpathOf("b", tool(1), {Move::rapid({0, 0, 5})})}) == "%\n(T1 Tool 1)\nG0\nG0\nM30\n");
}

TEST_CASE("Multiple toolpaths: TOOLCHANGE + SPINDLE_ON on a new tool, NEW_SEGMENT otherwise",
          "[post][run][multi]") {
    const PostProcessor pp = parsePost(R"PP(
FIRST_TOOLPATH_NEW_SEGMENT = "YES"
VAR X = [X|C| X|1.1]
begin HEADER
"(first tool T[T])"
end
begin TOOLCHANGE
"(change T[TP] -> T[T] [TOOLNAME] [TOOLPATH_NAME])"
"T[T] M6"
end
begin SPINDLE_ON
"M3 S[S]"
end
begin NEW_SEGMENT
"(segment [TOOLPATH_NAME] T[T])"
end
begin RAPID_MOVE
"G0[X]"
end
begin FOOTER
"(used: [TOOLS_USED])"
end
)PP");
    ToolSnapshot t2 = tool(2, "Ball Nose");
    t2.spindleSpeed = 12000.0;
    const std::string out = runOk(pp, {toolpathOf("Rough", tool(1), {Move::rapid({1, 0, 5})}),
                                       toolpathOf("Rough 2", tool(1), {Move::rapid({1, 0, 5})}),
                                       toolpathOf("Finish", t2, {Move::rapid({1, 0, 5})}),
                                       toolpathOf("Back", tool(1), {Move::rapid({1, 0, 5})})});
    CHECK(lines(out) == std::vector<std::string>{
                            "(first tool T1)",
                            "(segment Rough T1)",
                            "G0 X1.0",
                            "(segment Rough 2 T1)",
                            "G0 X1.0",  // modal memory reset: X restated
                            "(change T1 -> T2 Ball Nose Finish)",
                            "T2 M6",
                            "M3 S12000",
                            "G0 X1.0",
                            "(change T2 -> T1 Tool 1 Back)",
                            "T1 M6",
                            "M3 S18000",
                            "G0 X1.0",
                            "(used: Tool 1, Ball Nose)",
                        });
}

TEST_CASE("Text variables: SUBSTITUTE and context strings", "[post][run][text]") {
    const PostProcessor pp = parsePost(R"PP(
SUBSTITUTE = "([)]"
begin HEADER
"([TOOLNAME])"
"[PRODUCT] [DATE] [TIME] [TP_FILENAME] [TP_DIR] [TP_EXT] [PATHNAME] [FILE_NOTES]"
"[XH] [YH] [ZH] [XMIN] [YMAX] [ZLENGTH]"
end
begin RAPID_MOVE
end
)PP");
    PostContext ctx;
    ctx.product = "Carvetto 0.0.1";
    ctx.date = "2026-10-24";
    ctx.time = "12:00";
    ctx.fileName = "part";
    ctx.fileDir = "/out";
    ctx.fileExt = "nc";
    ctx.pathName = "/out/part.nc";
    ctx.fileNotes = "note (x)";
    ctx.home = {1, 2, 30};
    ctx.materialMin = {-5, -6, -18};
    ctx.materialMax = {100, 200, 0};
    const std::string out =
        runOk(pp, {toolpathOf("t", tool(1, "End Mill (1/4\")"), {Move::rapid({0, 0, 5})})}, ctx);
    CHECK(lines(out) ==
          std::vector<std::string>{"(End Mill [1/4\"])",
                                   "Carvetto 0.0.1 2026-10-24 12:00 part /out nc /out/part.nc note [x]",
                                   "1.000 2.000 30.000 -5.000 200.000 18.000"});
}

TEST_CASE("Dwell writes DWELL_MOVE with the pause in seconds", "[post][run]") {
    const PostProcessor pp = parsePost("VAR P = [DWELL|A|P|1.2]\nbegin DWELL_MOVE\n\"G4 [DWELL]\"\nend\n");
    CHECK(runOk(pp, {toolpathOf("t", tool(1), {Move::dwell({0, 0, 0}, 0.75)})}) == "G4 P0.75\n");
}

TEST_CASE("Output is deterministic", "[post][run]") {
    const PostProcessor pp = parsePost(kAbsolute + kArcBlocks);
    const std::vector<Toolpath> tps{toolpathOf(
        "t", tool(1), {Move::rapid({10, 0, 5}), Move::arcCCW({10, 0, -3}, {0, 0}), Move::feed({20, 0, -3})})};
    CHECK(runOk(pp, tps) == runOk(pp, tps));
}
