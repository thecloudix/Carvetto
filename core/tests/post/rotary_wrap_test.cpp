// T-0320: wrapped post output for rotary jobs (ADR-0103): angle = arc length / radius in degrees, sign
// (post -A and Flip Design), feed scaling, MAXIMUM_ROTATION splitting, the along-axis cross-check and
// the refusals. Golden files hold the exact wrapped programs for a cylinder along X and along Y.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"

using namespace carvetto;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using post::PostContext;
using post::PostProcessor;
using toolpath::Move;
using toolpath::Toolpath;

namespace {

constexpr double kPi = std::numbers::pi;

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// A small post: modal axis words and feed, A from the wrapped axis. `settings` go before the variables.
std::string postText(const std::string& settings) {
    return settings + R"PP(
POST_NAME = "Rotary test (*.nc)"
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
"(Tool [T])"
"G90 G94 G21"
"G0[SAFEZ]"
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
"G0[SAFEZ]"
"M30"
end
)PP";
}

PostProcessor parsePost(const std::string& settings) {
    auto r = PostProcessor::parse(postText(settings));
    if (!r.ok()) {
        FAIL("post failed to parse: " << r.error().describe());
    }
    return r.value();
}

toolpath::ToolSnapshot tool(double diameter = 6.0) {
    toolpath::ToolSnapshot t;
    t.name = "End Mill";
    t.toolNumber = 1;
    t.diameter = diameter;
    t.feedRate = 1000.0;
    t.plungeRate = 300.0;
    t.spindleSpeed = 18000.0;
    return t;
}

constexpr double kRadius = 25.0;  // 50 mm cylinder
constexpr double kQuarter = kPi * kRadius / 2.0;  // arc length of 90 degrees

/// Cylinder along X: the circumference is Y. Z is machine Z with the axis at 0 (surface at 25).
Toolpath alongXPath() {
    Toolpath tp;
    tp.name = "Wrapped";
    tp.tool = tool();
    tp.moves = {Move::rapid({0, 0, 30}),
                Move::plunge({0, 0, 23}),
                Move::feed({40, 0, 23}),
                Move::feed({40, kQuarter, 23}),
                Move::feed({60, 2 * kQuarter, 23}),
                Move::retract({60, 2 * kQuarter, 30})};
    return tp;
}

/// Cylinder along Y: the circumference is X.
Toolpath alongYPath() {
    Toolpath tp;
    tp.name = "Wrapped";
    tp.tool = tool();
    tp.moves = {Move::rapid({0, 0, 30}),
                Move::plunge({0, 0, 23}),
                Move::feed({0, 40, 23}),
                Move::feed({kQuarter, 40, 23}),
                Move::feed({2 * kQuarter, 60, 23}),
                Move::retract({2 * kQuarter, 60, 30})};
    return tp;
}

PostContext cylinderContext(char along, bool flip = false) {
    PostContext ctx;
    ctx.fileName = "rotary";
    ctx.home = {0, 0, 30};
    ctx.safeZ = 30;
    ctx.rotaryDiameter = 2.0 * kRadius;
    ctx.rotaryAlong = along;
    ctx.rotaryFlip = flip;
    ctx.rotaryAxisZ = 0.0;
    return ctx;
}

std::string run(const PostProcessor& pp, const Toolpath& tp, const PostContext& ctx) {
    const auto r = pp.run({tp}, ctx);
    if (!r.ok()) {
        FAIL("post run failed: " << r.error().describe());
    }
    return r.value().text;
}

void checkGolden(const std::string& name, const std::string& text) {
    CHECK(text == readFile(std::string(CARVETTO_GOLDEN_DIR) + "/post/" + name));
}

}  // namespace

TEST_CASE("golden: wrapped output for a cylinder along X (Y becomes A)", "[post][rotary][golden]") {
    const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
    checkGolden("rotary_x_wrapped.nc", run(pp, alongXPath(), cylinderContext('X')));
}

TEST_CASE("golden: wrapped output for a cylinder along Y (X becomes A)", "[post][rotary][golden]") {
    const PostProcessor pp = parsePost("ROTARY_WRAP_X = \"A\"\n");
    checkGolden("rotary_y_wrapped.nc", run(pp, alongYPath(), cylinderContext('Y')));
}

TEST_CASE("golden: -A and Flip Design cancel; either alone reverses the angle", "[post][rotary][golden]") {
    const PostProcessor neg = parsePost("ROTARY_WRAP_Y = \"-A\"\n");
    const PostProcessor pos = parsePost("ROTARY_WRAP_Y = \"A\"\n");
    const std::string plain = run(pos, alongXPath(), cylinderContext('X'));
    CHECK(run(neg, alongXPath(), cylinderContext('X', true)) == plain);
    checkGolden("rotary_x_flipped.nc", run(pos, alongXPath(), cylinderContext('X', true)));
    CHECK(run(neg, alongXPath(), cylinderContext('X')) == run(pos, alongXPath(), cylinderContext('X', true)));
}

TEST_CASE("angle is arc length over radius in degrees", "[post][rotary]") {
    const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
    const std::string text = run(pp, alongXPath(), cylinderContext('X'));
    CHECK_THAT(text, ContainsSubstring("A90.000"));
    CHECK_THAT(text, ContainsSubstring("A180.000"));
    // The flat circumference axis is not written.
    CHECK_THAT(text, !ContainsSubstring("Y"));
}

TEST_CASE("wrapped feed keeps the surface speed", "[post][rotary]") {
    const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
    const std::string text = run(pp, alongXPath(), cylinderContext('X'));
    // Along the axis only: unchanged. Round only: mm/min over the surface becomes deg/min = 1000 * 180 / (pi r).
    CHECK_THAT(text, ContainsSubstring("G1 X40.000 F1000"));
    const double roundOnly = 1000.0 * 180.0 / (kPi * kRadius);
    char buf[64];
    std::snprintf(buf, sizeof buf, "A90.000 F%.0f", roundOnly);
    CHECK_THAT(text, ContainsSubstring(buf));
    // Diagonal: 20 mm along and 90 degrees round; the controller length is sqrt(20^2 + 90^2).
    const double diagonal = 1000.0 * std::sqrt(20.0 * 20.0 + 90.0 * 90.0) / std::sqrt(20.0 * 20.0 + kQuarter * kQuarter);
    std::snprintf(buf, sizeof buf, "A180.000 F%.0f", diagonal);
    CHECK_THAT(text, ContainsSubstring(buf));
}

TEST_CASE("MAXIMUM_ROTATION splits a wrapped move into equal parts", "[post][rotary]") {
    const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\nMAXIMUM_ROTATION = 30\n");
    CHECK(pp.settings().maxRotation == Approx(30.0));
    const std::string text = run(pp, alongXPath(), cylinderContext('X'));
    for (const char* a : {"A30.000", "A60.000", "A90.000", "A120.000", "A150.000", "A180.000"}) {
        CHECK_THAT(text, ContainsSubstring(a));
    }
    // 90 degrees at the limit is not split further.
    const PostProcessor atLimit = parsePost("ROTARY_WRAP_Y = \"A\"\nMAXIMUM_ROTATION = 90\n");
    const std::string whole = run(atLimit, alongXPath(), cylinderContext('X'));
    CHECK_THAT(whole, !ContainsSubstring("A45.000"));
    CHECK_THAT(whole, ContainsSubstring("A90.000"));

    SECTION("a remainder that is not a whole multiple still never exceeds the limit") {
        // The A words written for one wrapped move of `degrees` under the 30 degree limit.
        const auto angles = [&](double degrees) {
            Toolpath tp;
            tp.name = "Wrapped";
            tp.tool = tool();
            tp.moves = {Move::rapid({0, 0, 30}), Move::plunge({0, 0, 23}),
                        Move::feed({0, degrees / 360.0 * 2.0 * kPi * kRadius, 23})};
            const std::string out = run(pp, tp, cylinderContext('X'));
            std::vector<double> v;
            std::istringstream in(out);
            std::string line;
            while (std::getline(in, line)) {
                if (line.rfind("G1", 0) == 0 && line.find('A') != std::string::npos) {
                    v.push_back(std::stod(line.substr(line.find('A') + 1)));
                }
            }
            return v;
        };
        const auto a = angles(90.0001);
        REQUIRE(a.size() == 4);  // 4 parts (A is modal, so the unchanged A0 of the plunge is not written)
        double prev = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i) {
            CHECK(a[i] - prev <= 30.0 + 1e-9);
            prev = a[i];
        }
        CHECK(a.back() == Approx(90.0001).margin(1e-3));
        const auto b = angles(30.00001);
        REQUIRE(b.size() == 2);  // 2 parts
        CHECK(b.back() == Approx(30.00001).margin(1e-3));
        CHECK(b[0] <= 30.0 + 1e-9);
        CHECK(angles(100.0).size() == 4);  // 4 parts of 25 degrees
    }
    SECTION("a limit that would need more than 100000 moves is refused") {
        const PostProcessor tiny = parsePost("ROTARY_WRAP_Y = \"A\"\nMAXIMUM_ROTATION = 0.0001\n");
        const auto r = tiny.run({alongXPath()}, cylinderContext('X'));
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().message, ContainsSubstring("MAXIMUM_ROTATION"));
    }
    SECTION("a bad value is a parse error") {
        CHECK_FALSE(PostProcessor::parse(postText("MAXIMUM_ROTATION = -5\n")).ok());
        CHECK_FALSE(PostProcessor::parse(postText("MAXIMUM_ROTATION = abc\n")).ok());
    }
}

TEST_CASE("rotary refusals and diagnostics", "[post][rotary]") {
    SECTION("the post wraps the axis the cylinder lies along") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_X = \"A\"\n");
        const auto r = pp.run({alongXPath()}, cylinderContext('X'));
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().message, ContainsSubstring("lies along X"));
        CHECK_THAT(r.error().message, ContainsSubstring("must wrap Y"));
    }
    SECTION("a rotary job with a post that does not wrap warns") {
        const PostProcessor pp = parsePost("");
        const auto r = pp.run({alongXPath()}, cylinderContext('X'));
        REQUIRE(r.ok());
        REQUIRE(r.value().warnings.size() == 1);
        CHECK_THAT(r.value().warnings[0].message, ContainsSubstring("does not wrap"));
    }
    SECTION("no cross-check when the job does not say which axis") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_X = \"A\"\n");
        PostContext ctx = cylinderContext('X');
        ctx.rotaryAlong = 0;
        CHECK(pp.run({alongXPath()}, ctx).ok());
    }
    SECTION("a move at or below the cylinder axis") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
        Toolpath tp = alongXPath();
        tp.moves[2] = Move::feed({40, 0, -1});
        const auto r = pp.run({tp}, cylinderContext('X'));
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().message, ContainsSubstring("cylinder axis"));
        CHECK_THAT(r.error().message, ContainsSubstring("'Wrapped'"));
        tp.moves[2] = Move::feed({40, 0, 0});
        CHECK_FALSE(pp.run({tp}, cylinderContext('X')).ok());
        tp.moves[2] = Move::feed({40, 0, 0.5});
        CHECK(pp.run({tp}, cylinderContext('X')).ok());
    }
    SECTION("a rapid below the cylinder axis is refused too") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
        Toolpath tp = alongXPath();
        tp.moves[0] = Move::rapid({0, 0, -2});
        const auto r = pp.run({tp}, cylinderContext('X'));
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().message, ContainsSubstring("cylinder axis"));
        CHECK_THAT(r.error().message, ContainsSubstring("'Wrapped'"));
        CHECK_THAT(r.error().message, ContainsSubstring("move 1"));
    }
    SECTION("a non-finite cylinder axis height is refused") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
        PostContext ctx = cylinderContext('X');
        ctx.rotaryAxisZ = std::numeric_limits<double>::quiet_NaN();
        const auto r = pp.run({alongXPath()}, ctx);
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().message, ContainsSubstring("cylinder axis"));
        ctx.rotaryAxisZ = std::numeric_limits<double>::infinity();
        CHECK_FALSE(pp.run({alongXPath()}, ctx).ok());
    }
    SECTION("a tool wider than the radius warns") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
        Toolpath tp = alongXPath();
        tp.tool = tool(30.0);
        const auto r = pp.run({tp}, cylinderContext('X'));
        REQUIRE(r.ok());
        REQUIRE(r.value().warnings.size() == 1);
        CHECK_THAT(r.value().warnings[0].message, ContainsSubstring("wider than the cylinder radius"));
    }
    SECTION("more than a turn warns") {
        const PostProcessor pp = parsePost("ROTARY_WRAP_Y = \"A\"\n");
        Toolpath tp = alongXPath();
        tp.moves.push_back(Move::feed({60, 4.5 * kQuarter, 23}));
        const auto r = pp.run({tp}, cylinderContext('X'));
        REQUIRE(r.ok());
        REQUIRE(r.value().warnings.size() == 1);
        CHECK_THAT(r.value().warnings[0].message, ContainsSubstring("more than once round"));
    }
}
