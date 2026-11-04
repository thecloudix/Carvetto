// T-0162: no written line is longer than 250 characters (line ending excluded), on every post
// and for any tool, toolpath or file name length (ADR-0020). LinuxCNC refuses a line of 254
// characters or more with its line ending (LINELEN 255, NCE_COMMAND_TOO_LONG) and grblHAL keeps
// comments in its 256-character line buffer (error:overflow), so a long name in a comment used
// to abort the program. A text value in a comment is cut to fit, ending in "..."; a line that
// cannot be made to fit is refused.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"

using namespace carvetto;
using post::PostContext;
using post::PostProcessor;
using toolpath::Move;
using toolpath::Toolpath;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::EndsWith;
using Catch::Matchers::StartsWith;

namespace {

constexpr std::size_t kLimit = 250;

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

PostProcessor parseText(const std::string& text) {
    auto r = PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("parse failed: " << r.error().describe());
    }
    return r.value();
}

std::string runOk(const PostProcessor& pp, const std::vector<Toolpath>& tps, const PostContext& ctx) {
    const auto r = pp.run(tps, ctx);
    if (!r.ok()) {
        FAIL("run failed: " << r.error().describe());
    }
    return r.value().text;
}

std::string runError(const PostProcessor& pp, const std::vector<Toolpath>& tps) {
    const auto r = pp.run(tps, test::fixtureContext());
    return r.ok() ? std::string() : r.error().describe();
}

/// The lines of `text` without their endings.
std::vector<std::string> linesOf(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
    }
    return lines;
}

/// `n` bytes of a name built from `pattern`, repeated (may end inside a UTF-8 sequence).
std::string nameOf(std::size_t n, const std::string& pattern) {
    std::string s;
    while (s.size() < n) {
        s += pattern;
    }
    s.resize(n);
    return s;
}

Toolpath cut(const std::string& name) {
    Toolpath tp;
    tp.name = name;
    tp.tool = test::fixtureEndMill();
    tp.moves = {Move::rapid({1, 1, 5}), Move::plunge({1, 1, -0.1}), Move::feed({2, 1, -0.1}), Move::retract({2, 1, 5})};
    return tp;
}

const std::string kMoves =
    "VAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\n"
    "VAR FEED_RATE = [F|C| F|1.0]\n"
    "begin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\n"
    "begin FEED_MOVE\n\"G1[X][Y][Z][F]\"\nend\n";

std::string headerPost(const std::string& line) {
    return "begin HEADER\n\"" + line + "\"\nend\n" + kMoves;
}

}  // namespace

TEST_CASE("T-0162 AC2: shipped posts keep every line within 250 characters for any name length",
          "[post][line-length]") {
    // Names of every length, with characters the sanitiser changes (parentheses, UTF-8, a comma,
    // Grbl realtime characters), in every text tag the shipped posts write: toolpath names, tool
    // names (spinning and diamond drag), the file name and the product.
    const std::vector<std::size_t> lengths{0, 1, 40, 180, 230, 240, 245, 246, 247, 248, 249, 250, 251, 252, 260, 300, 1000, 5000};
    for (const std::string& stem : shippedStems()) {
        const PostProcessor pp = parseText(readFile(std::string(CARVETTO_POSTS_DIR) + "/" + stem + ".pp"));
        for (const std::size_t n : lengths) {
            INFO(stem << ", names of " << n << " bytes");
            std::vector<Toolpath> job = test::fixtureToolpaths();
            for (const Toolpath& d : test::dragToolpaths()) {
                job.push_back(d);
            }
            int k = 0;
            for (Toolpath& tp : job) {
                tp.name = nameOf(n, "Path" + std::to_string(k++) + " (r\xC3\xA9sum\xC3\xA9), ok~ ");
                tp.tool.name = nameOf(n, "Tool " + std::to_string(tp.tool.toolNumber) + " [x]\xE2\x82\xAC; ");
            }
            PostContext ctx = test::fixtureContext();
            ctx.fileName = nameOf(n, "file-name_");
            ctx.product = nameOf(n, "Product ");
            const std::string out = runOk(pp, job, ctx);
            bool cutSeen = false;
            for (const std::string& line : linesOf(out)) {
                CHECK(line.size() <= kLimit);
                if (line.find("...") != std::string::npos) {
                    cutSeen = true;
                    // The cut is at the end of the comment and the line uses the room it has.
                    CHECK(line.size() >= kLimit - 3);
                    if (stem == "marlin_mm") {
                        CHECK_THAT(line, EndsWith("..."));
                    } else {
                        CHECK_THAT(line, EndsWith("...)"));
                    }
                }
            }
            // Short names are written whole; long ones are cut visibly.
            if (n <= 180) {
                CHECK_FALSE(cutSeen);
            }
            if (n >= 300) {
                CHECK(cutSeen);
                CHECK_THAT(out, ContainsSubstring(stem == "marlin_mm" ? "Path0 (resume), ok_ Path0"
                                                                      : "Path0 [resume], ok_ Path0"));
            }
        }
    }
}

TEST_CASE("T-0162 AC2: a comment value is cut to the exact line limit with an ellipsis", "[post][line-length]") {
    const PostProcessor pp = parseText(headerPost("(T [TOOLPATH_NAME])"));
    // "(T " + name + ")" is 4 characters plus the name.
    const std::string fits = runOk(pp, {cut(std::string(246, 'a'))}, test::fixtureContext());
    CHECK(linesOf(fits).front() == "(T " + std::string(246, 'a') + ")");
    for (const std::size_t n : {std::size_t{247}, std::size_t{248}, std::size_t{400}, std::size_t{100000}}) {
        INFO(n);
        const std::string out = runOk(pp, {cut(std::string(n, 'a'))}, test::fixtureContext());
        CHECK(linesOf(out).front() == "(T " + std::string(243, 'a') + "...)");
    }
    // ';' comments too.
    const std::string semi = runOk(parseText(headerPost("; T [TOOLPATH_NAME]")), {cut(std::string(400, 'b'))},
                                   test::fixtureContext());
    CHECK(linesOf(semi).front() == "; T " + std::string(243, 'b') + "...");
    // CR LF line endings: the limit excludes the ending.
    const std::string crlf = runOk(parseText("LINE_ENDING = \"[13][10]\"\n" + headerPost("(T [TOOLPATH_NAME])")),
                                   {cut(std::string(400, 'c'))}, test::fixtureContext());
    CHECK(crlf.substr(0, crlf.find('\n') + 1) == "(T " + std::string(243, 'c') + "...)\r\n");
}

TEST_CASE("T-0162 AC2: several text values on one line share the room", "[post][line-length]") {
    const PostProcessor pp = parseText(headerPost("([TOOLNAME] / [TOOLPATH_NAME])"));
    Toolpath tp = cut(std::string(400, 'p'));
    tp.tool.name = "Short tool";
    std::string line = linesOf(runOk(pp, {tp}, test::fixtureContext())).front();
    CHECK(line.size() == kLimit);
    CHECK_THAT(line, StartsWith("(Short tool / ppp"));
    CHECK_THAT(line, EndsWith("ppp...)"));
    // Both long: both cut to the same length, the line within the limit.
    tp.tool.name = std::string(400, 't');
    line = linesOf(runOk(pp, {tp}, test::fixtureContext())).front();
    CHECK(line.size() <= kLimit);
    CHECK(line.size() >= kLimit - 1);
    const std::size_t half = (line.size() - 5) / 2;  // "(" + " / " + ")"
    CHECK(line == "(" + std::string(half - 3, 't') + "... / " + std::string(half - 3, 'p') + "...)");
}

TEST_CASE("T-0162 AC2: cutting a value keeps the sanitising and writes counters once", "[post][line-length]") {
    // An active-comment keyword stays neutralised in the cut value.
    const std::string active = linesOf(runOk(parseText(headerPost("([TOOLPATH_NAME])")),
                                             {cut("LOGOPEN,/home/cnc/x" + std::string(400, 'a'))},
                                             test::fixtureContext()))
                                   .front();
    CHECK_THAT(active, StartsWith("(LOGOPEN_/home/cnc/xaaa"));
    CHECK_THAT(active, EndsWith("...)"));
    CHECK(active.size() == kLimit);
    // A parenthesis in the name stays mapped; a cut never closes the comment early.
    const std::string paren = linesOf(runOk(parseText(headerPost("(T [TOOLPATH_NAME])")),
                                            {cut(nameOf(400, "a)b(c "))}, test::fixtureContext()))
                                  .front();
    CHECK(paren.find(')') == paren.size() - 1);
    CHECK(paren.find('(', 1) == std::string::npos);
    // Line numbers advance once per line, modal values are remembered once.
    const std::string numbered = "LINE_NUMBER_START = 10\nLINE_NUMBER_INCREMENT = 10\nVAR LINE_NUMBER = [N|A|N|1.0]\n"
                                 "begin HEADER\n\"[N] ([TOOLPATH_NAME])\"\n\"[N] (next)\"\nend\n" +
                                 kMoves;
    const std::vector<std::string> lines =
        linesOf(runOk(parseText(numbered), {cut(std::string(400, 'n'))}, test::fixtureContext()));
    REQUIRE(lines.size() >= 2);
    CHECK_THAT(lines[0], StartsWith("N10 (nnn"));
    CHECK(lines[0].size() == kLimit);
    CHECK(lines[1] == "N20 (next)");
}

TEST_CASE("T-0162 AC2: a line that cannot be made to fit is refused", "[post][line-length]") {
    // Template text alone.
    const std::string text250 = "(" + std::string(248, 'x') + ")";
    CHECK(runError(parseText(headerPost(text250)), {cut("A")}).empty());
    const std::string text251 = "(" + std::string(249, 'x') + ")";
    CHECK_THAT(runError(parseText(headerPost(text251)), {cut("A")}),
               ContainsSubstring("would be 251 characters long; lines are limited to 250"));
    // A text value in live G-code is not cut (it is not a comment), so a long one is refused.
    CHECK(runError(parseText(headerPost("M117 [TOOLPATH_NAME]")), {cut("Short")}).empty());
    CHECK_THAT(runError(parseText(headerPost("M117 [TOOLPATH_NAME]")), {cut(std::string(300, 'L'))}),
               ContainsSubstring("lines are limited to 250"));
    // So many text values that even "..." for each is too long.
    std::string many = "(";
    for (int i = 0; i < 90; ++i) {
        many += "[TOOLPATH_NAME]";
    }
    many += ")";
    CHECK(runError(parseText(headerPost(many)), {cut("ab")}).empty());  // 2 + 180 characters
    CHECK_THAT(runError(parseText(headerPost(many)), {cut("abcd")}), ContainsSubstring("lines are limited to 250"));
}
