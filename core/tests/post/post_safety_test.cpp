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
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
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
    CHECK(out == "r s|p q|r s, u v|1 2|3 4|5 6|7 8|9 0|a b|c d|e  f\n");
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
            CHECK((finiteProblem || rangeProblem));
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
