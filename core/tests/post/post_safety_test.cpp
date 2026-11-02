// Output-safety tests for the post interpreter (T-0133, ADR-0020).
//
// AC1: every text value rendered by a post has its control characters (0x00-0x1F, 0x7F) replaced
//      by spaces, and inside an open `( … )` comment `(`/`)` become `[`/`]`, so no name can start
//      a new block or close a comment early. Checked for every shipped post with hostile names
//      (newline, CR, tab, NUL, DEL, ")", "(", ";") in the toolpath name, tool name and file name.
// AC2: the post refuses non-finite or out-of-range numbers (±inf, NaN, 1e300) with an error
//      instead of output, for every shipped post.
// AC3: the shipped goldens are unchanged (shipped_posts_test.cpp); the shipped posts only place
//      text tags inside comments.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"

using namespace carvetto::post;
using carvetto::toolpath::Move;
using carvetto::toolpath::MoveKind;
using carvetto::toolpath::Toolpath;
using carvetto::toolpath::ToolSnapshot;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

struct Shipped {
    const char* stem;
    const char* lineEnding;
};

const std::vector<Shipped>& shipped() {
    static const std::vector<Shipped> posts{
        {"grbl_mm", "\n"},     {"grbl_inch", "\n"},   {"mach3_mm", "\r\n"},    {"mach3_inch", "\r\n"},
        {"mach4_mm", "\r\n"},  {"linuxcnc_mm", "\n"}, {"linuxcnc_inch", "\n"}, {"marlin_mm", "\n"},
    };
    return posts;
}

PostProcessor parseText(const std::string& text) {
    auto r = PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("parse failed: " << r.error().describe());
    }
    return r.value();
}

PostProcessor loadShipped(const Shipped& p) {
    return parseText(readFile(std::string(CARVETTO_POSTS_DIR) + "/" + p.stem + ".pp"));
}

std::string golden(const Shipped& p) {
    return readFile(std::string(CARVETTO_GOLDEN_DIR) + "/post/" + p.stem + "_fixture.nc");
}

std::vector<std::string> splitLines(const std::string& text, const std::string& ending) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = text.find(ending, start);
        REQUIRE(end != std::string::npos);
        lines.push_back(text.substr(start, end - start));
        start = end + ending.size();
    }
    return lines;
}

/// The executable part of a line: `( … )` comments and everything after a `;` outside them
/// removed. Fails the test when a line nests comments or leaves one unclosed.
std::string codeOf(const std::string& line) {
    std::string code;
    bool paren = false;
    for (char c : line) {
        if (paren) {
            REQUIRE(c != '(');  // nested comment: an error on LinuxCNC and Grbl
            paren = c != ')';
        } else if (c == '(') {
            paren = true;
        } else if (c == ';') {
            break;
        } else {
            code += c;
        }
    }
    REQUIRE_FALSE(paren);  // unclosed comment
    return code;
}

/// A name built to break out of a comment or start a new block in every comment style.
std::string hostile(const std::string& stem) {
    std::string s = stem + "\nG0 Z-50\r\nM3 S1\tX";
    s += '\0';
    s += "Y\x7f\x1b;) G0 Z-60 (";
    return s;
}

/// The fixture with hostile toolpath, tool and file names.
std::vector<Toolpath> hostileFixture() {
    std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
    for (Toolpath& tp : tps) {
        tp.name = hostile(tp.name);
        tp.tool.name = hostile(tp.tool.name);
    }
    return tps;
}

PostContext hostileContext() {
    PostContext ctx = carvetto::test::fixtureContext();
    ctx.fileName = hostile("fixture");
    return ctx;
}

std::string runOk(const PostProcessor& pp, const std::vector<Toolpath>& tps, const PostContext& ctx) {
    auto r = pp.run(tps, ctx);
    if (!r.ok()) {
        FAIL("run failed: " << r.error().describe());
    }
    return r.value().text;
}

Toolpath simple(std::vector<Move> moves) {
    Toolpath tp;
    tp.name = "t";
    tp.tool = carvetto::test::fixtureEndMill();
    tp.moves = std::move(moves);
    return tp;
}

}  // namespace

// ---- AC1: text sanitising ----------------------------------------------------------------------

TEST_CASE("AC1: hostile names cannot add blocks or close comments in any shipped post",
          "[post][safety][text]") {
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const PostProcessor pp = loadShipped(p);
        const std::string out = runOk(pp, hostileFixture(), hostileContext());
        const std::vector<std::string> got = splitLines(out, p.lineEnding);
        const std::vector<std::string> want = splitLines(golden(p), p.lineEnding);

        // Same number of lines: no name started a new block.
        REQUIRE(got.size() == want.size());
        for (std::size_t i = 0; i < got.size(); ++i) {
            INFO("line " << i + 1 << ": " << got[i]);
            // No control character survives anywhere in a line.
            for (char c : got[i]) {
                const auto u = static_cast<unsigned char>(c);
                CHECK((u >= 0x20 && u != 0x7F));
            }
            // The executable part of every line is exactly the golden's: the names changed
            // comment text only.
            CHECK(codeOf(got[i]) == codeOf(want[i]));
        }
        CHECK_THAT(out, ContainsSubstring("G0 Z-50"));  // the text is kept, as comment text
    }
}

TEST_CASE("AC1: expected rendering of a hostile toolpath name per comment style", "[post][safety][text]") {
    const auto nameLine = [](const Shipped& p) {
        std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
        std::string name = "A\nB\rC\tD";
        name += '\0';
        name += "E;F)G(H";
        tps[0].name = name;
        const std::string out = runOk(loadShipped(p), tps, carvetto::test::fixtureContext());
        for (const std::string& line : splitLines(out, p.lineEnding)) {
            if (line.find("A B C D") != std::string::npos) {
                return line;
            }
        }
        return std::string("<not found>");
    };
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const std::string line = nameLine(p);
        if (std::string(p.stem) == "marlin_mm") {
            // `;` comments run to the end of the line: only control characters need replacing.
            CHECK(line == "; A B C D E;F)G(H");
        } else {
            CHECK(line == "(A B C D E;F]G[H)");
        }
    }
}

TEST_CASE("AC1: control characters become spaces in every text tag and context string",
          "[post][safety][text]") {
    const PostProcessor pp = parseText(R"PP(
begin HEADER
"[TOOLNAME]|[TOOLPATH_NAME]|[TOOLS_USED]|[TIME]|[DATE]|[TP_FILENAME]|[TP_DIR]|[TP_EXT]|[PATHNAME]|[PRODUCT]|[FILE_NOTES]"
end
begin RAPID_MOVE
end
)PP");
    PostContext ctx;
    ctx.time = "1\n2";
    ctx.date = "3\r4";
    ctx.fileName = "5\t6";
    ctx.fileDir = std::string("7\0" "8", 3);
    ctx.fileExt = "9\x7f" "0";
    ctx.pathName = "a\x01" "b";
    ctx.product = "c\x1f" "d";
    ctx.fileNotes = "e\n\nf";
    Toolpath a = simple({Move::rapid({0, 0, 5})});
    a.name = "p\nq";
    a.tool.name = "r\ns";
    Toolpath b = a;
    b.tool.name = "u\rv";
    b.tool.toolNumber = 2;
    const std::string out = runOk(pp, {a, b}, ctx);
    CHECK(out == "r s|p q|r s; u v|1 2|3 4|5 6|7 8|9 0|a b|c d|e  f\n");
}

TEST_CASE("AC1: parentheses are mapped only inside an open ( ) comment, even without SUBSTITUTE",
          "[post][safety][text]") {
    const PostProcessor pp = parseText(R"PP(
begin HEADER
"([TOOLPATH_NAME])"
"[TOOLPATH_NAME]"
"; [TOOLPATH_NAME]"
"(closed) [TOOLPATH_NAME]"
end
begin RAPID_MOVE
end
)PP");
    Toolpath tp = simple({Move::rapid({0, 0, 5})});
    tp.name = "a) G0 Z-50 (b";
    const std::string out = runOk(pp, {tp}, {});
    CHECK(out ==
          "(a] G0 Z-50 [b)\n"  // inside a comment: cannot close it
          "a) G0 Z-50 (b\n"    // live text: the post author's choice, written as is
          "; a) G0 Z-50 (b\n"  // a ; comment runs to the end of the line
          "(closed) a) G0 Z-50 (b\n");

    // A VAR prefix that opens the comment counts as part of the line before the value.
    const PostProcessor prefixed = parseText(R"PP(
VAR NAME = [TOOLPATH_NAME|A|(|1.0]
begin HEADER
"[TOOLPATH_NAME])"
end
begin RAPID_MOVE
end
)PP");
    CHECK(runOk(prefixed, {tp}, {}) == "(a] G0 Z-50 [b)\n");
}

TEST_CASE("AC1: SUBSTITUTE cannot reintroduce a comment close or a control character",
          "[post][safety][text]") {
    const PostProcessor pp = parseText("SUBSTITUTE = \"a)b(c\tdz\"\nbegin HEADER\n\"([TOOLPATH_NAME])\"\nend\n"
                                       "begin RAPID_MOVE\nend\n");
    Toolpath tp = simple({Move::rapid({0, 0, 5})});
    tp.name = "abcdx";
    CHECK(runOk(pp, {tp}, {}) == "(][ zx)\n");
}

TEST_CASE("AC3: shipped posts place every text tag inside a comment", "[post][safety][text]") {
    const std::vector<std::string> textTags{"[TOOLNAME]",   "[TOOL_NOTES]", "[TOOLPATH_NAME]", "[TOOLS_USED]",
                                            "[TIME]",       "[DATE]",       "[TP_FILENAME]",   "[TP_DIR]",
                                            "[TP_EXT]",     "[PATHNAME]",   "[PRODUCT]",       "[FILE_NOTES]",
                                            "[CUTTER_COMP]"};
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const std::string source = readFile(std::string(CARVETTO_POSTS_DIR) + "/" + p.stem + ".pp");
        std::istringstream in(source);
        std::string line;
        while (std::getline(in, line)) {
            for (const std::string& tag : textTags) {
                const std::size_t at = line.find(tag);
                if (at == std::string::npos) {
                    continue;
                }
                INFO(line);
                const std::string before = line.substr(0, at);
                const bool inParen = before.rfind('(') != std::string::npos &&
                                     (before.rfind(')') == std::string::npos ||
                                      before.rfind(')') < before.rfind('('));
                const bool inLine = before.find(';') != std::string::npos;
                CHECK((inParen || inLine));
            }
        }
    }
}

// ---- AC2: numbers ------------------------------------------------------------------------------

namespace {

/// Each case breaks one value of the fixture.
struct BadValue {
    const char* what;
    std::function<void(std::vector<Toolpath>&, PostContext&)> apply;
};

std::vector<BadValue> badValues() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<BadValue> cases;
    for (double v : {kInf, -kInf, nan, 1e300, -1e300, 1e6 + 1.0}) {
        cases.push_back({"plunge Z", [v](std::vector<Toolpath>& t, PostContext&) { t[0].moves[1].to.z = v; }});
        cases.push_back({"feed X", [v](std::vector<Toolpath>& t, PostContext&) { t[0].moves[2].to.x = v; }});
        cases.push_back({"feed Y", [v](std::vector<Toolpath>& t, PostContext&) { t[0].moves[3].to.y = v; }});
        cases.push_back({"rapid X", [v](std::vector<Toolpath>& t, PostContext&) { t[0].moves[0].to.x = v; }});
        cases.push_back(
            {"arc centre", [v](std::vector<Toolpath>& t, PostContext&) { t[1].moves[2].centre.x = v; }});
        cases.push_back({"arc end Z", [v](std::vector<Toolpath>& t, PostContext&) { t[1].moves[2].to.z = v; }});
        cases.push_back(
            {"feed override", [v](std::vector<Toolpath>& t, PostContext&) { t[0].moves[2].feedOverride = v; }});
        cases.push_back({"dwell", [v](std::vector<Toolpath>& t, PostContext&) { t[2].moves[6].dwellSeconds = v; }});
        cases.push_back({"tool feed", [v](std::vector<Toolpath>& t, PostContext&) { t[0].tool.feedRate = v; }});
        cases.push_back(
            {"tool plunge", [v](std::vector<Toolpath>& t, PostContext&) { t[2].tool.plungeRate = v; }});
        cases.push_back(
            {"spindle", [v](std::vector<Toolpath>& t, PostContext&) { t[2].tool.spindleSpeed = v; }});
        cases.push_back({"safe Z", [v](std::vector<Toolpath>&, PostContext& c) { c.safeZ = v; }});
        cases.push_back({"home Z", [v](std::vector<Toolpath>&, PostContext& c) { c.home.z = v; }});
    }
    return cases;
}

}  // namespace

TEST_CASE("AC2: every shipped post refuses non-finite and out-of-range numbers", "[post][safety][number]") {
    const std::vector<BadValue> cases = badValues();
    for (const Shipped& p : shipped()) {
        const PostProcessor pp = loadShipped(p);
        for (const BadValue& bad : cases) {
            std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
            PostContext ctx = carvetto::test::fixtureContext();
            bad.apply(tps, ctx);
            INFO(p.stem << ": " << bad.what);
            const auto r = pp.run(tps, ctx);
            REQUIRE_FALSE(r.ok());
            const std::string msg = r.error().describe();
            INFO(msg);
            CHECK_THAT(msg, ContainsSubstring("cannot write G-code"));
            const bool finiteProblem = msg.find("is not a finite number") != std::string::npos;
            const bool rangeProblem = msg.find("is outside the supported range") != std::string::npos;
            // T-0134 AC3: a spindle speed that is not above zero (NaN included) gets the spindle
            // message, naming the tool.
            const bool spindleProblem = std::string(bad.what) == "spindle" &&
                                        msg.find("has spindle speed") != std::string::npos &&
                                        msg.find("must be greater than zero") != std::string::npos;
            CHECK((finiteProblem || rangeProblem || spindleProblem));
            // The message stays short: no 300-digit number.
            CHECK(msg.size() < 300);
        }
    }
}

TEST_CASE("AC2: error messages name the toolpath, move and value", "[post][safety][number]") {
    const PostProcessor pp = loadShipped(shipped()[0]);
    std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
    tps[0].moves[1].to.z = -kInf;
    tps[0].name = "Evil\nG0 Z-50";
    auto r = pp.run(tps, carvetto::test::fixtureContext());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().describe() ==
          "cannot write G-code: toolpath 'Evil G0 Z-50', move 2: Z -inf is not a finite number");

    tps = carvetto::test::fixtureToolpaths();
    tps[2].tool.spindleSpeed = 1e300;
    r = pp.run(tps, carvetto::test::fixtureContext());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().describe() ==
          "cannot write G-code in SPINDLE_ON of toolpath 'Drill': [S] 1e+300 rpm is outside the "
          "supported range -1e+06..1e+06 rpm");

    tps = carvetto::test::fixtureToolpaths();
    tps[0].name = std::string(500, 'n');
    tps[0].moves[2].feedOverride = std::numeric_limits<double>::quiet_NaN();
    r = pp.run(tps, carvetto::test::fixtureContext());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().describe() == "cannot write G-code: toolpath '" + std::string(64, 'n') +
                                      "...', move 3: feed NaN is not a finite number");
}

TEST_CASE("AC2: a bad coordinate is refused even on an axis the post never writes",
          "[post][safety][number]") {
    const PostProcessor pp = parseText("VAR X = [X|A| X|1.3]\nbegin RAPID_MOVE\n\"G0[X]\"\nend\n");
    const auto r = pp.run({simple({Move::rapid({1, 2, kInf})})}, {});
    REQUIRE_FALSE(r.ok());
    CHECK_THAT(r.error().describe(), ContainsSubstring("Z +inf is not a finite number"));
}

TEST_CASE("AC2: the machine range limit is 1e6 mm and inclusive", "[post][safety][number]") {
    const PostProcessor pp =
        parseText("VAR X = [X|A| X|1.3]\nbegin RAPID_MOVE\n\"G0[X]\"\nend\nbegin FEED_MOVE\n\"G1[X]\"\nend\n");
    CHECK(runOk(pp, {simple({Move::rapid({1e6, 0, 0}), Move::rapid({-1e6, 0, 0})})}, {}) ==
          "G0 X1000000.000\nG0 X-1000000.000\n");
    CHECK_FALSE(pp.run({simple({Move::rapid({1e6 + 0.001, 0, 0})})}, {}).ok());
    // Arcs: a non-finite centre never reaches the arc maths (linearised here: no arc blocks).
    const auto arc = pp.run({simple({Move::rapid({0, 0, 0}), Move::arcCW({1, 0, 0}, {kInf, 0})})}, {});
    REQUIRE_FALSE(arc.ok());
    CHECK_THAT(arc.error().describe(), ContainsSubstring("arc centre X +inf"));
}

TEST_CASE("AC2: incremental output and derived values are range checked too", "[post][safety][number]") {
    // Incremental Z: both ends are checked as written.
    const PostProcessor inc = parseText("VAR Z = [Z|AI| Z|1.3|1e4]\nbegin RAPID_MOVE\n\"G0[Z]\"\nend\n");
    CHECK(runOk(inc, {simple({Move::rapid({0, 0, 5})})}, {}) == "G0 Z50000.000\n");
    const auto big = inc.run({simple({Move::rapid({0, 0, 1e6})})}, {});
    REQUIRE_FALSE(big.ok());
    CHECK_THAT(big.error().describe(), ContainsSubstring("[Z] would be written as 1e+10"));

    // A rotary wrap onto a tiny cylinder turns a valid Y into an absurd angle.
    const PostProcessor wrap =
        parseText("ROTARY_WRAP_Y = \"A\"\nVAR A = [A|A| A|1.3]\nbegin RAPID_MOVE\n\"G0[A]\"\nend\n");
    PostContext rot;
    rot.rotaryDiameter = 1e-6;
    const auto r = wrap.run({simple({Move::rapid({0, 1000, 5})})}, rot);
    REQUIRE_FALSE(r.ok());
    CHECK_THAT(r.error().describe(), ContainsSubstring("[A]"));
    CHECK_THAT(r.error().describe(), ContainsSubstring("degrees"));

    // Material extents in the header.
    const PostProcessor hdr = parseText("begin HEADER\n\"([XLENGTH])\"\nend\nbegin RAPID_MOVE\nend\n");
    PostContext mat;
    mat.materialMax = {kInf, 0, 0};
    CHECK_FALSE(hdr.run({}, mat).ok());
}

TEST_CASE("AC2: a VAR multiplier cannot push a written value past 1e9", "[post][safety][number]") {
    const PostProcessor pp = parseText("VAR X = [X|A| X|1.3|1e300]\nbegin RAPID_MOVE\n\"G0[X]\"\nend\n");
    const auto r = pp.run({simple({Move::rapid({1, 0, 0})})}, {});
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().line == 1);
    CHECK_THAT(r.error().describe(), ContainsSubstring("VAR multiplier 1e+300"));
    // Without a multiplier the failure is not blamed on the VAR line.
    const PostProcessor tool = parseText("VAR T = [T|A|T|1.0]\nbegin HEADER\n\"[T]\"\nend\nbegin RAPID_MOVE\nend\n");
    Toolpath big = simple({Move::rapid({0, 0, 5})});
    big.tool.toolNumber = 2000000000;
    const auto t = tool.run({big}, {});
    REQUIRE_FALSE(t.ok());
    CHECK(t.error().line == 0);
    CHECK_THAT(t.error().describe(), ContainsSubstring("[T] would be written as 2e+09"));
    // A sane multiplier (Marlin's seconds -> milliseconds) still works at the limit.
    const PostProcessor ms = parseText("VAR P = [DWELL|A| P|1.0|1000]\nbegin DWELL_MOVE\n\"G4[DWELL]\"\nend\n");
    CHECK(runOk(ms, {simple({Move::dwell({0, 0, 0}, 1e6)})}, {}) == "G4 P1000000000\n");
}

TEST_CASE("AC2: non-finite VAR multipliers are rejected when parsing", "[post][safety][number]") {
    for (const char* m : {"inf", "-inf", "nan", "1e400"}) {
        INFO(m);
        const auto r = PostProcessor::parse(std::string("VAR X = [X|A| X|1.3|") + m + "]\n");
        CHECK_FALSE(r.ok());
    }
}

TEST_CASE("AC2: a refused run produces no partial output and stops early", "[post][safety][number]") {
    const PostProcessor pp = loadShipped(shipped()[0]);
    std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
    tps[2].moves.back().to.z = kInf;  // the very last move
    const auto r = pp.run(tps, carvetto::test::fixtureContext());
    REQUIRE_FALSE(r.ok());
    CHECK_THAT(r.error().describe(), ContainsSubstring("toolpath 'Drill', move 12"));
}

// ---- round 2: arc tolerance, rotary diameter, inverse time (verifier B1 and notes) -------------

namespace {

const char* const kLinearOnly =
    "VAR X = [X|A|X|1.3]\nVAR Y = [Y|A|Y|1.3]\nVAR Z = [Z|A|Z|1.3]\nbegin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\n"
    "begin FEED_MOVE\n\"G1[X][Y][Z]\"\nend\n";

std::size_t countLines(const std::string& text) {
    std::size_t n = 0;
    for (char c : text) {
        n += c == '\n' ? 1 : 0;
    }
    return n;
}

}  // namespace

TEST_CASE("B1: an arc tolerance outside 1e-6..1 mm is refused, never a single chord",
          "[post][safety][arc]") {
    const PostProcessor pp = parseText(kLinearOnly);
    const Toolpath semi = simple({Move::rapid({25, 0, -1}), Move::arcCCW({-25, 0, -1}, {0, 0})});
    for (double tol : {1e-12, 1e-15, 1e-17, 1e-20, 0.0, -0.01, std::numeric_limits<double>::quiet_NaN(), kInf,
                       -kInf, 1e300, 1.5, 9e-7}) {
        INFO("tolerance " << tol);
        PostContext ctx;
        ctx.arcTolerance = tol;
        const auto r = pp.run({semi}, ctx);
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().describe(), ContainsSubstring("arc tolerance"));
        CHECK_THAT(r.error().describe(), ContainsSubstring("outside the supported range 1e-06..1 mm"));
    }
    // The bounds themselves are accepted and give many chords.
    for (double tol : {1e-6, 1.0}) {
        PostContext ctx;
        ctx.arcTolerance = tol;
        const std::string out = runOk(pp, {semi}, ctx);
        CHECK(countLines(out) > 4);
        CHECK(out.substr(out.size() - 24) == "G1X-25.000Y0.000Z-1.000\n");
    }
}

TEST_CASE("B1: a huge arc at the finest tolerance is capped at 100000 chords, not collapsed",
          "[post][safety][arc]") {
    const PostProcessor pp = parseText(kLinearOnly);
    PostContext ctx;
    ctx.arcTolerance = 1e-6;
    // r = 1e6 mm full circle: needs ~2.2 million chords at 1e-6 mm; capped.
    const std::string out =
        runOk(pp, {simple({Move::rapid({1e6, 0, 0}), Move::arcCCW({1e6, 0, 0}, {0, 0})})}, ctx);
    CHECK(countLines(out) == 1 + 100000);
}

TEST_CASE("Rotary diameter must be finite and at most 1e6 mm when the post wraps", "[post][safety][rotary]") {
    const PostProcessor wrap =
        parseText("ROTARY_WRAP_Y = \"A\"\nVAR A = [A|A| A|1.3]\nbegin RAPID_MOVE\n\"G0[A]\"\nend\n");
    for (double d : {kInf, 1e300, 1e6 + 1.0, std::numeric_limits<double>::quiet_NaN()}) {
        INFO("diameter " << d);
        PostContext ctx;
        ctx.rotaryDiameter = d;
        const auto r = wrap.run({simple({Move::rapid({0, 10, 5})})}, ctx);
        REQUIRE_FALSE(r.ok());
        CHECK_THAT(r.error().describe(), ContainsSubstring("rotary diameter"));
    }
    PostContext ok;
    ok.rotaryDiameter = 1e6;
    CHECK(wrap.run({simple({Move::rapid({0, 10, 5})})}, ok).ok());
    // Without a wrap the diameter is not used and not checked.
    const PostProcessor plain = parseText(kLinearOnly);
    PostContext unused;
    unused.rotaryDiameter = kInf;
    CHECK(plain.run({simple({Move::rapid({0, 10, 5})})}, unused).ok());
}

TEST_CASE("Inverse-time [F] uses the inverse-time limit (1e9 1/min), not the feed limit",
          "[post][safety][number]") {
    const PostProcessor pp = parseText("INVERSE_TIME_MODE = YES\nVAR F = [F|A| F|1.1]\nbegin RAPID_MOVE\n"
                                       "\"G0\"\nend\nbegin FEED_MOVE\n\"G1[F]\"\nend\n");
    // 1000 mm/min over 1e-4 mm: 1e7 1/min, above the 1e6 feed limit but a valid inverse time.
    const std::string out = runOk(pp, {simple({Move::rapid({0, 0, 0}), Move::feed({1e-4, 0, 0})})}, {});
    CHECK(out == "G0\nG1 F10000000.0\n");
    // 1000 mm/min over 1e-8 mm: 1e11 1/min.
    const auto r = pp.run({simple({Move::rapid({0, 0, 0}), Move::feed({1e-8, 0, 0})})}, {});
    REQUIRE_FALSE(r.ok());
    CHECK_THAT(r.error().describe(), ContainsSubstring("1/min is outside the supported range -1e+09..1e+09"));
}

// ---- round 2: printable ASCII only (verifier B2, owner decision "ASCII-only, all posts") --------

TEST_CASE("B2: text values are printable ASCII; Grbl realtime bytes never reach the output",
          "[post][safety][text]") {
    // U+00D1 U+00D7 U+00DB (Grbl feed +10 %, rapid 25 %, spindle -10 % bytes), ~ ! ?, é, €, a
    // stray continuation byte, a truncated sequence, 0xFF and an overlong encoding.
    std::string name = "\xC3\x91\xC3\x97" "10 \xC3\x9B\xC3\x97" "9 ~!? \xC3\xA9 \xE2\x82\xAC \x80 \xE2\x82 \xFF \xC0\xAF";
    const std::string expected = "N_10 U_9 ___ e _ _ __ _ __";
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
        for (Toolpath& tp : tps) {
            tp.name = name;
            tp.tool.name = name;
        }
        PostContext ctx = carvetto::test::fixtureContext();
        ctx.fileName = name;
        ctx.product = name;
        const std::string out = runOk(loadShipped(p), tps, ctx);
        for (char c : out) {
            const auto u = static_cast<unsigned char>(c);
            CHECK((u < 0x80 && c != '~' && c != '!' && c != '?'));
        }
        CHECK_THAT(out, ContainsSubstring(expected));
    }
}

TEST_CASE("B2: Latin-1 letters fold to plain letters, everything else non-ASCII to one _",
          "[post][safety][text]") {
    const PostProcessor pp =
        parseText("begin HEADER\n\"[TOOLPATH_NAME]\"\nend\nbegin RAPID_MOVE\nend\n");
    Toolpath tp = simple({Move::rapid({0, 0, 5})});
    tp.name = "\xC3\x80\xC3\x85\xC3\x87\xC3\x89\xC3\x8F\xC3\x91\xC3\x96\xC3\x98\xC3\x9C\xC3\x9D"  // ÀÅÇÉÏÑÖØÜÝ
              "\xC3\xA0\xC3\xA7\xC3\xA8\xC3\xB1\xC3\xB6\xC3\xBC\xC3\xBF"                          // àçèñöüÿ
              " \xC3\x86\xC3\x9F\xC3\xB7\xC2\xB0 \xF0\x9F\x99\x82 \xE6\x97\xA5";  // Æß÷° emoji 日
    CHECK(runOk(pp, {tp}, {}) == "AACEINOOUYacenouy ____ _ _\n");
}

TEST_CASE("B2: error messages quote names as printable ASCII", "[post][safety][text]") {
    const PostProcessor pp = parseText(kLinearOnly);
    Toolpath tp = simple({Move::rapid({0, 0, kInf})});
    tp.name = "\xC3\x91~\n\xFF";
    const auto r = pp.run({tp}, {});
    REQUIRE_FALSE(r.ok());
    CHECK_THAT(r.error().describe(), ContainsSubstring("toolpath 'N_ _'"));
    for (char c : r.error().describe()) {
        CHECK(static_cast<unsigned char>(c) < 0x80);
    }
}

// ---- round 2: active comments (verifier B3, owner decision "neutralise in sanitiser") ----------

TEST_CASE("B3: a value at the start of a comment cannot form an active comment", "[post][safety][text]") {
    const PostProcessor pp = parseText(R"PP(
begin HEADER
"([TOOLPATH_NAME])"
"( [TOOLPATH_NAME])"
"; [TOOLPATH_NAME]"
"(Name: [TOOLPATH_NAME])"
"[TOOLPATH_NAME]"
end
begin RAPID_MOVE
end
)PP");
    const auto render = [&](const std::string& name) {
        Toolpath tp = simple({Move::rapid({0, 0, 5})});
        tp.name = name;
        return runOk(pp, {tp}, {});
    };
    for (const char* keyword : {"MSG", "DEBUG", "PRINT", "LOG", "LOGOPEN", "LOGAPPEND", "ABORT", "PY", "PYRUN"}) {
        INFO(keyword);
        const std::string k(keyword);
        CHECK(render(k + ",/home/cnc/mill.hal") == "(" + k + "_/home/cnc/mill.hal)\n( " + k +
                                                       "_/home/cnc/mill.hal)\n; " + k +
                                                       "_/home/cnc/mill.hal\n(Name: " + k +
                                                       ",/home/cnc/mill.hal)\n" + k + ",/home/cnc/mill.hal\n");
    }
    // Mixed case, leading spaces and spaces inside the keyword.
    CHECK(render("  lOgOpEn,x").substr(0, 15) == "(  lOgOpEn_x)\n(");
    CHECK(render("Log Open , x").substr(0, 15) == "(Log Open _ x)\n");
    CHECK(render("\tmsg,hi").substr(0, 9) == "( msg_hi)");
    // Any word, not just known keywords; every comma that continues the word run (a replaced
    // comma is a word character, so the run goes on).
    CHECK(render("Profile,1,2").substr(0, 14) == "(Profile_1_2)\n");
    CHECK(render("Profile,1-2,3").substr(0, 16) == "(Profile_1-2,3)\n");
    // No-comma keywords get a leading _.
    CHECK(render("PROBEOPEN /tmp/x").substr(0, 19) == "(_PROBEOPEN /tmp/x)");
    CHECK(render("probe close").substr(0, 14) == "(_probe close)");
    CHECK(render("LogClose").substr(0, 11) == "(_LogClose)");
    // Ordinary names are untouched.
    CHECK(render("Profile 1").substr(0, 12) == "(Profile 1)\n");
    CHECK(render("Pocket - 3 mm, rough").substr(0, 23) == "(Pocket - 3 mm, rough)\n");
}

TEST_CASE("B3: a VAR prefix that opens the comment counts as comment start", "[post][safety][text]") {
    const PostProcessor pp = parseText(R"PP(
VAR NAME = [TOOLPATH_NAME|A|(|1.0]
begin HEADER
"[TOOLPATH_NAME])"
end
begin RAPID_MOVE
end
)PP");
    Toolpath tp = simple({Move::rapid({0, 0, 5})});
    tp.name = "LOGOPEN,/etc/x";
    CHECK(runOk(pp, {tp}, {}) == "(LOGOPEN_/etc/x)\n");
}

TEST_CASE("B3: no shipped post writes an active comment for a hostile name", "[post][safety][text]") {
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        for (const char* name : {"LOGOPEN,/home/cnc/mill.hal", " msg ,hi", "PROBEOPEN /tmp/p", "py,import os"}) {
            INFO(name);
            std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
            for (Toolpath& tp : tps) {
                tp.name = name;
                tp.tool.name = name;
            }
            PostContext ctx = carvetto::test::fixtureContext();
            ctx.fileName = name;
            for (const std::string& line : splitLines(runOk(loadShipped(p), tps, ctx), p.lineEnding)) {
                // Every comment's text, spaces dropped and upper-cased, must not start with a
                // word followed by a comma, nor with an argument-less active keyword.
                std::size_t open = std::string::npos;
                for (std::size_t i = 0; i < line.size(); ++i) {
                    if (line[i] == '(' || (line[i] == ';' && open == std::string::npos)) {
                        open = i;
                        std::string squashed;
                        for (std::size_t k = i + 1; k < line.size() && line[k] != ')'; ++k) {
                            if (line[k] != ' ') {
                                squashed += static_cast<char>(std::toupper(static_cast<unsigned char>(line[k])));
                            }
                        }
                        INFO(line);
                        const std::size_t word = squashed.find_first_not_of(
                            "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_");
                        CHECK_FALSE((word != std::string::npos && squashed[word] == ','));
                        CHECK(squashed.rfind("PROBEOPEN", 0) == std::string::npos);
                    }
                }
            }
        }
    }
}

// ---- round 3: active comments assembled from several values (verifier B4) -----------------------

namespace {

/// True when a comment's text (after its `(` or `;`) would run as an active comment: spaces
/// dropped and upper-cased, a word run followed by a comma, or a leading PROBEOPEN / PROBECLOSE /
/// LOGCLOSE.
bool isActiveComment(std::string_view text) {
    std::string squashed;
    for (char c : text) {
        if (c != ' ' && c != '\t') {
            squashed += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
    }
    const std::size_t stop = squashed.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_");
    if (stop != std::string::npos && squashed[stop] == ',') {
        return true;
    }
    for (const char* keyword : {"PROBEOPEN", "PROBECLOSE", "LOGCLOSE"}) {
        if (squashed.rfind(keyword, 0) == 0) {
            return true;
        }
    }
    return false;
}

/// Every comment of every line of `out`.
std::vector<std::string> commentsOf(const std::string& out) {
    std::vector<std::string> comments;
    std::istringstream in(out);
    std::string line;
    while (std::getline(in, line)) {
        bool paren = false;
        std::string current;
        for (std::size_t i = 0; i < line.size(); ++i) {
            const char c = line[i];
            if (paren) {
                if (c == ')') {
                    comments.push_back(current);
                    paren = false;
                } else {
                    current += c;
                }
            } else if (c == '(') {
                paren = true;
                current.clear();
            } else if (c == ';') {
                comments.push_back(line.substr(i + 1));
                break;
            }
        }
        if (paren) {
            comments.push_back(current);
        }
    }
    return comments;
}

std::string runNames(const PostProcessor& pp, const std::string& tool, const std::string& toolpath,
                     const std::string& dir = "", const std::string& file = "") {
    Toolpath tp = simple({Move::rapid({0, 0, 5})});
    tp.tool.name = tool;
    tp.name = toolpath;
    PostContext ctx;
    ctx.fileDir = dir;
    ctx.fileName = file;
    return runOk(pp, {tp}, ctx);
}

PostProcessor headerPost(const std::string& line, const std::string& vars = "") {
    return parseText(vars + "begin HEADER\n\"" + line + "\"\nend\nbegin RAPID_MOVE\nend\n");
}

}  // namespace

TEST_CASE("B4: adjacent values cannot assemble an active comment", "[post][safety][text]") {
    const std::string hal = ",/home/cnc/linuxcnc/configs/mill/mill.hal";
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME])"), "LOGOPEN", hal) ==
          "(LOGOPEN_/home/cnc/linuxcnc/configs/mill/mill.hal)\n");
    CHECK(runNames(headerPost("([TP_DIR][TP_FILENAME])"), "", "", "LOGOPEN", ",/tmp/x") == "(LOGOPEN_/tmp/x)\n");
    CHECK(runNames(headerPost(";[TOOLNAME][TOOLPATH_NAME]"), "LOGOPEN", ",/tmp/x") == ";LOGOPEN_/tmp/x\n");
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME])"), "PROBE", "OPEN /tmp/p") == "(PROBE_OPEN /tmp/p)\n");
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME])"), "LOG", "CLOSE") == "(LOG_CLOSE)\n");
    CHECK(runNames(headerPost("([TOOLNAME] [TOOLPATH_NAME])"), "LOGOPEN", ",/tmp/x") == "(LOGOPEN _/tmp/x)\n");
    CHECK(runNames(headerPost("([TOOLNAME] [TOOLPATH_NAME])"), "probe", "close") == "(probe _close)\n");
    // Three values, the comma in the last.
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME][TP_FILENAME])"), "LOG", "OPEN", "", ",/tmp/x") ==
          "(LOGOPEN_/tmp/x)\n");
    // A template word between the values at comment start is part of the run.
    CHECK(runNames(headerPost("([TOOLNAME] and [TOOLPATH_NAME])"), "x", ",/tmp/x") == "(x and _/tmp/x)\n");
    CHECK(runNames(headerPost("(MSG[TOOLPATH_NAME])"), "", ",hi") == "(MSG_hi)\n");
    // A VAR prefix word between the values is part of the run.
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME])", "VAR TP = [TOOLPATH_NAME|A|LOG|1.0]\n"), "x",
                   "OPEN,/tmp/p") == "(xLOGOPEN_/tmp/p)\n");
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME])", "VAR TP = [TOOLPATH_NAME|A|PRO|1.0]\n"), "",
                   "BEOPEN /tmp/p") == "(PRO_BEOPEN /tmp/p)\n");
}

TEST_CASE("B4: only words and spaces keep the comment 'at its start'", "[post][safety][text]") {
    // A non-word character before the value ends the keyword run; commas are kept from then on.
    CHECK(runNames(headerPost("([TOOLNAME]-[TOOLPATH_NAME])"), "LOGOPEN", ",x") == "(LOGOPEN-,x)\n");
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME])"), "Tool: 1", ",x") == "(Tool: 1,x)\n");
    // The documented gap: template text written after a value completes the active comment
    // (here the comma, or the rest of the keyword from a VAR prefix). Values already written
    // cannot be changed; no shipped post has template text there.
    CHECK(runNames(headerPost("([TOOLNAME],x)"), "LOGOPEN", "") == "(LOGOPEN,x)\n");
    CHECK(runNames(headerPost("([TOOLNAME][TOOLPATH_NAME])", "VAR TP = [TOOLPATH_NAME|A|OPEN|1.0]\n"), "PROBE",
                   " /tmp/p") == "(PROBEOPEN /tmp/p)\n");
}

TEST_CASE("B4: randomized concatenations of hostile values never form an active comment",
          "[post][safety][text]") {
    const std::vector<std::string> pieces{"LOGOPEN", "LOG",  "OPEN",      ",",      ",/tmp/x", " ",       "PROBE",
                                          "OPEN /p", "msg",  ",hi",       "Py",     "run,",    "CLOSE",   "",
                                          " , ",     "Lo g", " Open ,x", "probe c", "lose",    "AbOrT,",  "\t,",
                                          "DEBUG",   "x)(",  "\xC3\x91,", "PRINT ",  "_,",      "logclose"};
    const std::vector<std::string> templates{
        "([TOOLNAME][TOOLPATH_NAME][TP_FILENAME])", "([TOOLNAME] [TOOLPATH_NAME] [TP_FILENAME])",
        ";[TOOLNAME][TOOLPATH_NAME][TP_FILENAME]",  "( [TOOLNAME][TOOLPATH_NAME])",
        "([TOOLNAME]x[TOOLPATH_NAME])",              "; [TOOLNAME] [TOOLPATH_NAME]",
        "(ok) ([TOOLNAME][TOOLPATH_NAME])"};
    std::mt19937 rng(20261102u);  // mt19937's sequence is fixed by the standard
    const auto pick = [&](std::size_t n) { return static_cast<std::size_t>(rng() % n); };
    for (const std::string& t : templates) {
        const PostProcessor pp = headerPost(t);
        for (int run = 0; run < 400; ++run) {
            std::string v[3];
            for (std::string& s : v) {
                const std::size_t parts = 1 + pick(2);
                for (std::size_t k = 0; k < parts; ++k) {
                    s += pieces[pick(pieces.size())];
                }
            }
            const std::string out = runNames(pp, v[0], v[1], "", v[2]);
            for (const std::string& comment : commentsOf(out)) {
                INFO(t << " -> " << out);
                CHECK_FALSE(isActiveComment(comment));
            }
        }
    }
}

TEST_CASE("AC3: shipped posts define no VAR prefix that could move a text tag out of a comment",
          "[post][safety][text]") {
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const PostProcessor pp = loadShipped(p);
        for (const char* tag : {"TOOLNAME", "TOOL_NOTES", "TOOLPATH_NAME", "TOOLS_USED", "TIME", "DATE",
                                "TP_FILENAME", "TP_DIR", "TP_EXT", "PATHNAME", "PRODUCT", "FILE_NOTES",
                                "CUTTER_COMP"}) {
            INFO(tag);
            const VarDef* def = pp.var(tag);
            if (def != nullptr) {
                CHECK(def->prefix.find_first_of("();") == std::string::npos);
            }
        }
    }
}

// ---- T-0134: residuals of the T-0133 verification ----------------------------------------------

namespace {

/// Every place in `source` (a .pp file) where a text tag in an output line is followed by
/// something other than `)`, the end of the line or another text tag without a VAR prefix:
/// template text right after a value, which the sanitiser cannot judge (ADR-0020 gap).
std::vector<std::string> textAfterValue(const std::string& source, const PostProcessor& pp) {
    const auto isText = [](std::string_view tag) {
        const TagInfo* info = findTag(tag);
        return info != nullptr && info->kind == TagKind::Text;
    };
    std::vector<std::string> found;
    std::istringstream in(source);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const std::size_t first = line.find_first_not_of(" \t");
        const std::size_t last = line.find_last_not_of(" \t");
        if (first == std::string::npos || line[first] != '"' || last <= first || line[last] != '"') {
            continue;  // not an output line
        }
        const std::string text = line.substr(first + 1, last - first - 1);
        std::size_t i = 0;
        while ((i = text.find('[', i)) != std::string::npos) {
            const std::size_t close = text.find(']', i);
            REQUIRE(close != std::string::npos);
            const std::string tag = text.substr(i + 1, close - i - 1);
            i = close + 1;
            if (!isText(tag) || i == text.size() || text[i] == ')') {
                continue;
            }
            bool ok = false;
            if (text[i] == '[') {
                const std::size_t nextClose = text.find(']', i);
                REQUIRE(nextClose != std::string::npos);
                const std::string next = text.substr(i + 1, nextClose - i - 1);
                const VarDef* def = pp.var(next);
                ok = isText(next) && (def == nullptr || def->prefix.empty());
            }
            if (!ok) {
                found.push_back(line);
            }
        }
    }
    return found;
}

std::string replaceAll(std::string s, const std::string& from, const std::string& to) {
    for (std::size_t at = s.find(from); at != std::string::npos; at = s.find(from, at + to.size())) {
        s.replace(at, from.size(), to);
    }
    return s;
}

}  // namespace

TEST_CASE("T-0134: shipped posts follow a text tag only by ')', the line end or another text tag",
          "[post][safety][text]") {
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const std::string source = readFile(std::string(CARVETTO_POSTS_DIR) + "/" + p.stem + ".pp");
        const PostProcessor pp = parseText(source);
        CHECK(textAfterValue(source, pp).empty());

        // The check catches the gap shapes: template commas and keyword completions after a value.
        for (const char* add : {",x", " ,x", "OPEN", "CLOSE", " x", "[T]"}) {
            INFO(add);
            const std::string mutated =
                replaceAll(replaceAll(source, "([TOOLPATH_NAME])", std::string("([TOOLPATH_NAME]") + add + ")"),
                           "; [TOOLPATH_NAME]\"", std::string("; [TOOLPATH_NAME]") + add + "\"");
            REQUIRE(mutated != source);
            CHECK_FALSE(textAfterValue(mutated, parseText(mutated)).empty());
        }
    }
    // Another text tag is fine, unless a VAR prefix puts template text between the two.
    const std::string adjacent = "begin HEADER\n\"([TOOLNAME][TOOLPATH_NAME])\"\nend\n";
    CHECK(textAfterValue(adjacent, parseText(adjacent)).empty());
    const std::string prefixed = "VAR NAME = [TOOLPATH_NAME|A|OPEN|1.0]\n" + adjacent;
    CHECK_FALSE(textAfterValue(prefixed, parseText(prefixed)).empty());
}

TEST_CASE("T-0134: the parser refuses control characters other than tab in .pp lines", "[post][safety][text]") {
    struct Bad {
        std::string ch;
        const char* code;
    };
    for (const Bad& b : {Bad{"\f", "0x0C"}, Bad{"\v", "0x0B"}, Bad{"\r", "0x0D"}, Bad{"\x01", "0x01"},
                         Bad{"\x1b", "0x1B"}, Bad{"\x7f", "0x7F"}, Bad{std::string(1, '\0'), "0x00"}}) {
        INFO(b.code);
        // In an output line between "(" and a value: LinuxCNC would skip it as whitespace and
        // run "(MSG,hi)" (verifier driver ff.cpp).
        auto r = PostProcessor::parse("begin HEADER\n\"(" + b.ch + "[TOOLPATH_NAME])\"\nend\n");
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().line == 2);
        CHECK_THAT(r.error().message, ContainsSubstring(std::string("control character ") + b.code));
        // In a VAR prefix and in a setting.
        auto v = PostProcessor::parse("+ c\nVAR NAME = [TOOLPATH_NAME|A|(" + b.ch + "|1.0]\n");
        REQUIRE_FALSE(v.ok());
        CHECK(v.error().line == 2);
        auto s = PostProcessor::parse("POST_NAME = \"a" + b.ch + "b\"\n");
        REQUIRE_FALSE(s.ok());
        CHECK(s.error().line == 1);
    }
    // Tab is allowed (and counts as a space in the word run); CR LF line ends and comment lines
    // are unaffected.
    const PostProcessor tab = parseText(
        "begin HEADER\r\n\"(\t[TOOLPATH_NAME])\"\r\nend\r\nbegin RAPID_MOVE\r\nend\r\n+ \f note\r\n");
    Toolpath tp = simple({Move::rapid({0, 0, 5})});
    tp.name = "MSG,hi";
    CHECK(runOk(tab, {tp}, {}) == "(\tMSG_hi)\n");
}

TEST_CASE("T-0134: [TOOLS_USED] joins names with '; ' so the comma rule does not touch them",
          "[post][safety][text]") {
    const PostProcessor pp = parseText(R"PP(
begin HEADER
"([TOOLS_USED])"
"; [TOOLS_USED]"
"(Tools: [TOOLS_USED])"
end
begin TOOLCHANGE
end
begin RAPID_MOVE
end
)PP");
    const auto run = [&](const std::string& a, const std::string& b) {
        Toolpath t1 = simple({Move::rapid({0, 0, 5})});
        t1.tool.name = a;
        Toolpath t2 = simple({Move::rapid({0, 0, 5})});
        t2.tool.toolNumber = t1.tool.toolNumber + 1;
        t2.tool.name = b;
        return runOk(pp, {t1, t2}, {});
    };
    CHECK(run("EndMill", "Drill") == "(EndMill; Drill)\n; EndMill; Drill\n(Tools: EndMill; Drill)\n");
    // The first name's own commas still continue its word run; the "; " ends it.
    CHECK(run("MSG,a", "b,c") == "(MSG_a; b,c)\n; MSG_a; b,c\n(Tools: MSG,a; b,c)\n");
}

// ---- T-0134 AC3 (owner decision 2026-11-02): spindle speed above zero, no rate printed as zero

namespace {

/// The run's error message ("" when it succeeds).
std::string runError(const PostProcessor& pp, const std::vector<Toolpath>& tps, const PostContext& ctx) {
    const auto r = pp.run(tps, ctx);
    return r.ok() ? std::string() : r.error().describe();
}

/// One unit of the last printed digit of `tag` in `pp`, in internal units (mm/min, rpm, s).
double printStep(const PostProcessor& pp, const char* tag, bool lengthUnits) {
    const VarDef* def = pp.var(tag);
    REQUIRE(def != nullptr);
    double step = std::pow(10.0, -def->format.decimals) / def->multiplier;
    if (lengthUnits && pp.settings().units == Units::Inches) {
        step *= 25.4;
    }
    return step;
}

}  // namespace

TEST_CASE("T-0134 AC3: every shipped post refuses a spindle speed that is not above zero",
          "[post][safety][number]") {
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const PostProcessor pp = loadShipped(p);
        const PostContext ctx = carvetto::test::fixtureContext();
        for (double bad : {0.0, -0.0, -5.0, std::numeric_limits<double>::quiet_NaN(), 0.4}) {
            INFO(bad);
            // The first tool (spindle started in the header) and the tool after a change (SPINDLE_ON).
            for (std::size_t which : {std::size_t{0}, std::size_t{2}}) {
                std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
                tps[which].tool.spindleSpeed = bad;
                const std::string msg = runError(pp, tps, ctx);
                INFO(msg);
                CHECK_THAT(msg, ContainsSubstring("cannot write G-code"));
                CHECK_THAT(msg, ContainsSubstring("of toolpath '" + tps[which].name + "'"));
                if (bad == 0.4) {
                    // Positive but printed as S0.
                    CHECK_THAT(msg, ContainsSubstring("[S] 0.4 rpm would be written as S0"));
                } else {
                    CHECK_THAT(msg, ContainsSubstring("tool '" + tps[which].tool.name + "' has spindle speed"));
                    CHECK_THAT(msg, ContainsSubstring("must be greater than zero"));
                }
            }
        }
        // Just above the print threshold: S1.
        std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
        tps[0].tool.spindleSpeed = 0.6;
        tps[2].tool.spindleSpeed = 0.6;
        CHECK(runError(pp, tps, ctx).empty());
    }
}

TEST_CASE("T-0134 AC3: the spindle check covers SPINDLE_ON without [S] and skips posts that never start it",
          "[post][safety][number]") {
    const std::string moves = "begin RAPID_MOVE\n\"G0[X]\"\nend\nbegin FEED_MOVE\n\"G1[X]\"\nend\n";
    Toolpath a = simple({Move::rapid({1, 0, 5})});
    a.tool.spindleSpeed = 1000.0;
    Toolpath b = a;
    b.name = "Second";
    b.tool.toolNumber = 2;
    b.tool.name = "Manual";
    b.tool.spindleSpeed = 0.0;

    const PostProcessor m3 = parseText("begin TOOLCHANGE\n\"M0\"\nend\nbegin SPINDLE_ON\n\"M3\"\nend\n" + moves);
    const std::string msg = runError(m3, {a, b}, {});
    CHECK_THAT(msg, ContainsSubstring("in SPINDLE_ON of toolpath 'Second': tool 'Manual' has spindle speed 0 rpm"));

    // A manual-spindle post that never writes [S] nor SPINDLE_ON writes any speed.
    const PostProcessor manual = parseText("begin TOOLCHANGE\n\"M0\"\nend\n" + moves);
    CHECK(runError(manual, {a, b}, {}).empty());

    // A job without toolpaths has no spindle to check.
    const PostProcessor header = parseText("begin HEADER\n\"M3 S[S]\"\nend\n" + moves);
    CHECK(runError(header, {}, {}).empty());
}

TEST_CASE("T-0134 AC3: every shipped post refuses a feed or dwell above zero that would print as zero",
          "[post][safety][number]") {
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const PostProcessor pp = loadShipped(p);
        const PostContext ctx = carvetto::test::fixtureContext();
        const double feedStep = printStep(pp, "F", true);
        const double dwellStep = printStep(pp, "DWELL", false);
        struct Case {
            const char* what;
            const char* tag;
            double step;
            std::function<void(std::vector<Toolpath>&, double)> apply;
        };
        const std::vector<Case> cases{
            {"feed rate", "[F]", feedStep, [](std::vector<Toolpath>& t, double v) { t[0].tool.feedRate = v; }},
            {"plunge rate", "[F]", feedStep, [](std::vector<Toolpath>& t, double v) { t[0].tool.plungeRate = v; }},
            {"feed override", "[F]", feedStep,
             [](std::vector<Toolpath>& t, double v) { t[0].moves[2].feedOverride = v; }},
            {"dwell", "[DWELL]", dwellStep, [](std::vector<Toolpath>& t, double v) { t[2].moves[6].dwellSeconds = v; }},
        };
        for (const Case& c : cases) {
            INFO(c.what);
            for (double tiny : {0.4 * c.step, 1e-6 * c.step, std::numeric_limits<double>::denorm_min()}) {
                INFO(tiny);
                std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
                c.apply(tps, tiny);
                const std::string msg = runError(pp, tps, ctx);
                INFO(msg);
                CHECK_THAT(msg, ContainsSubstring(std::string(c.tag) + " "));
                CHECK_THAT(msg, ContainsSubstring("would be written as "));
                CHECK_THAT(msg, ContainsSubstring("above zero but too small"));
            }
            // Just above the threshold, the value prints as the smallest non-zero number.
            std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
            c.apply(tps, 0.6 * c.step);
            CHECK(runError(pp, tps, ctx).empty());
        }
    }
}

TEST_CASE("T-0134 AC3: every shipped post refuses feeds of zero or less and dwells of zero or less",
          "[post][safety][number]") {
    for (const Shipped& p : shipped()) {
        INFO(p.stem);
        const PostProcessor pp = loadShipped(p);
        const PostContext ctx = carvetto::test::fixtureContext();
        struct Case {
            const char* what;
            std::function<void(std::vector<Toolpath>&, double)> apply;
        };
        const std::vector<Case> feeds{
            {"feed rate", [](std::vector<Toolpath>& t, double v) { t[0].tool.feedRate = v; }},
            {"plunge rate", [](std::vector<Toolpath>& t, double v) { t[0].tool.plungeRate = v; }},
            {"later tool plunge rate", [](std::vector<Toolpath>& t, double v) { t[2].tool.plungeRate = v; }},
            {"feed override", [](std::vector<Toolpath>& t, double v) { t[0].moves[2].feedOverride = v; }},
        };
        for (const Case& c : feeds) {
            INFO(c.what);
            for (double bad : {0.0, -0.0, -5.0, -1e-300}) {
                INFO(bad);
                std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
                c.apply(tps, bad);
                const std::string msg = runError(pp, tps, ctx);
                INFO(msg);
                CHECK_THAT(msg, ContainsSubstring("cannot write G-code"));
                CHECK_THAT(msg, ContainsSubstring("[F] feed "));
                CHECK_THAT(msg, ContainsSubstring("feeds must be greater than zero"));
            }
        }
        for (double bad : {0.0, -0.0, -0.5, -1e-300}) {
            INFO("dwell " << bad);
            std::vector<Toolpath> tps = carvetto::test::fixtureToolpaths();
            tps[2].moves[6].dwellSeconds = bad;
            const std::string msg = runError(pp, tps, ctx);
            CHECK_THAT(msg, ContainsSubstring("toolpath 'Drill', move 7: dwell "));
            CHECK_THAT(msg, ContainsSubstring(" s must be greater than zero"));
        }
    }
}

TEST_CASE("T-0134 AC3: a zero dwell is refused even when the post drops dwells", "[post][safety][number]") {
    const PostProcessor pp = parseText("begin RAPID_MOVE\n\"G0[X]\"\nend\nbegin FEED_MOVE\n\"G1[X]\"\nend\n");
    Toolpath tp = simple({Move::rapid({1, 0, 5}), Move::dwell({1, 0, 5}, 0.0)});
    CHECK_THAT(runError(pp, {tp}, {}), ContainsSubstring("move 2: dwell 0 s must be greater than zero"));
    tp.moves[1].dwellSeconds = 0.5;
    CHECK(runError(pp, {tp}, {}).empty());  // dropped with a warning, as before
}

TEST_CASE("T-0134 AC3: inverse-time feeds are judged by the feed they come from", "[post][safety][number]") {
    // Two custom posts (shipped posts do not use inverse time): INVERSE_TIME_MODE, where [F] writes
    // inverse time on cutting moves, and an inch post writing [FI] directly.
    const PostProcessor inverseMode = parseText(R"PP(
UNITS = "MM"
INVERSE_TIME_MODE = "YES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.1]
begin HEADER
"M3[S]"
end
begin RAPID_MOVE
"G94 G0[X][Y][Z]"
end
begin FEED_MOVE
"G93 G1[X][Y][Z][F]"
end
)PP");
    const PostProcessor fiTag = parseText(R"PP(
UNITS = "INCHES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR INV = [FI|A| F|1.2]
begin HEADER
"M3[S]"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G93 G1[X][Y][Z][FI]"
end
)PP");
    // Plunge 1 mm, cut 20 mm, retract 6 mm.
    const auto job = [](double feed, double plunge, std::optional<double> override) {
        Toolpath t = simple({Move::rapid({0, 0, 5}), Move::plunge({0, 0, -1}), Move::feed({20, 0, -1}),
                             Move::retract({20, 0, 5})});
        t.tool.feedRate = feed;
        t.tool.plungeRate = plunge;
        t.tool.spindleSpeed = 18000.0;
        t.moves[2].feedOverride = override;
        return std::vector<Toolpath>{t};
    };
    for (const PostProcessor* pp : {&inverseMode, &fiTag}) {
        INFO((pp == &inverseMode ? "INVERSE_TIME_MODE" : "[FI]"));
        CHECK(runError(*pp, job(1000.0, 300.0, std::nullopt), {}).empty());
        for (double bad : {0.0, -0.0, -5.0}) {
            INFO(bad);
            for (const auto& [what, tps] :
                 {std::pair{"feed", job(bad, 300.0, std::nullopt)}, std::pair{"plunge", job(1000.0, bad, std::nullopt)},
                  std::pair{"override", job(1000.0, 300.0, bad)}}) {
                INFO(what);
                const std::string msg = runError(*pp, tps, {});
                INFO(msg);
                CHECK_THAT(msg, ContainsSubstring(" mm/min (inverse time "));
                CHECK_THAT(msg, ContainsSubstring("feeds must be greater than zero"));
            }
        }
        // A feed above zero whose inverse time prints as zero (or underflows to exactly 0).
        for (double tiny : {1e-6, std::numeric_limits<double>::denorm_min()}) {
            INFO(tiny);
            const std::string msg = runError(*pp, job(tiny, 300.0, std::nullopt), {});
            INFO(msg);
            CHECK_THAT(msg, ContainsSubstring("above zero but too small"));
        }
        // A zero-length cutting move has inverse time 0 by definition and is written.
        std::vector<Toolpath> still = job(1000.0, 300.0, std::nullopt);
        still[0].moves.insert(still[0].moves.begin() + 3, Move::feed({20, 0, -1}));
        CHECK(runError(*pp, still, {}).empty());
    }
}

TEST_CASE("T-0134 AC3: a job without toolpaths checks neither spindle nor feed tags", "[post][safety][number]") {
    const PostProcessor pp = parseText(
        "begin HEADER\n\"M3 S[S] F[F] F[FC] F[FP]\"\nend\nbegin RAPID_MOVE\n\"G0[X]\"\nend\n");
    CHECK(runError(pp, {}, {}).empty());
}
