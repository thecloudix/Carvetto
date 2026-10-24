#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <string>

#include "carvetto/post/post_processor.hpp"

using namespace carvetto::post;
using Catch::Matchers::ContainsSubstring;

namespace {
PostProcessor parseOk(const std::string& text) {
    auto r = PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("parse failed: " << r.error().describe());
    }
    return r.value();
}
Error parseFail(const std::string& text) {
    auto r = PostProcessor::parse(text);
    REQUIRE_FALSE(r.ok());
    return r.error();
}
}  // namespace

TEST_CASE("Parser: comments, settings, VARs and blocks", "[post][parse]") {
    const PostProcessor pp = parseOk(R"PP(+ a comment
   + indented comment

POST_NAME = "Test post (*.tap)"
FILE_EXTENSION = "tap"
UNITS = "INCHES"
LINE_ENDING = "[13][10]"
LINE_NUMBER_START = 100
LINE_NUMBER_INCREMENT = 5
LINE_NUMBER_MAXIMUM = 9999
RAPID_PLUNGE_TO_STARTZ = "YES"
INVERSE_TIME_MODE = "NO"
FIRST_TOOLPATH_NEW_SEGMENT = "YES"
MIN_ARC_RADIUS = 0.05
MAX_ARC_RADIUS = 5000
SUBSTITUTE = "({)}"
ROTARY_WRAP_Y = "-A"
RAPID_SPEED = 5000

VAR X_POSITION = [X|C| X|1.4]
begin HEADER
+ comment inside a block
"%"
"G20 [X]"
""
end
)PP");
    const PostSettings& s = pp.settings();
    CHECK(s.postName == "Test post (*.tap)");
    CHECK(s.fileExtension == "tap");
    CHECK(s.units == Units::Inches);
    CHECK(s.lineEnding == "\r\n");
    CHECK(s.lineNumberStart == 100);
    CHECK(s.lineNumberIncrement == 5);
    CHECK(s.lineNumberMaximum == 9999);
    CHECK(s.rapidPlungeToStartZ);
    CHECK_FALSE(s.inverseTimeMode);
    CHECK(s.firstToolpathNewSegment);
    CHECK(s.minArcRadius == 0.05);
    CHECK(s.maxArcRadius == 5000.0);
    REQUIRE(s.substitute.size() == 2);
    CHECK(s.substitute[0] == std::pair<char, char>{'(', '{'});
    CHECK(s.substitute[1] == std::pair<char, char>{')', '}'});
    REQUIRE(s.rotaryWrapY.has_value());
    CHECK(*s.rotaryWrapY == RotaryWrap{'A', true});
    CHECK_FALSE(s.rotaryWrapX.has_value());
    CHECK(s.raw.at("RAPID_SPEED") == "5000");
    CHECK(pp.warnings().empty());

    REQUIRE(pp.hasBlock("HEADER"));
    const PostBlock* h = pp.block("HEADER");
    REQUIRE(h->lines.size() == 3);
    CHECK(h->lines[0] == "%");
    CHECK(h->lines[1] == "G20 [X]");
    CHECK(h->lines[2].empty());
    CHECK(h->line == 21);
    CHECK_FALSE(pp.hasBlock("FOOTER"));

    REQUIRE(pp.var("X") != nullptr);
    CHECK(pp.var("X")->name == "X_POSITION");
    CHECK(pp.var("X")->format.decimals == 4);
    CHECK(pp.var("Y") == nullptr);
}

TEST_CASE("Parser: defaults when settings are absent", "[post][parse]") {
    const PostProcessor pp = parseOk("begin FOOTER\n\"M30\"\nend\n");
    const PostSettings& s = pp.settings();
    CHECK(s.units == Units::Millimetres);
    CHECK(s.lineEnding == "\n");
    CHECK(s.lineNumberStart == 0);
    CHECK(s.lineNumberIncrement == 1);
    CHECK(s.lineNumberMaximum == 999999);
    CHECK_FALSE(s.rotaryWrapX.has_value());
    CHECK(s.substitute.empty());
}

TEST_CASE("Parser: CRLF source files parse the same", "[post][parse]") {
    const PostProcessor pp = parseOk("UNITS = \"MM\"\r\nbegin FOOTER\r\n\"M30\"\r\nend\r\n");
    REQUIRE(pp.hasBlock("FOOTER"));
    CHECK(pp.block("FOOTER")->lines.at(0) == "M30");
}

TEST_CASE("AC4: unknown block names are a warning, not a failure", "[post][parse]") {
    const PostProcessor pp = parseOk("begin MADE_UP_BLOCK\n\"G0\"\nend\nbegin FOOTER\n\"M30\"\nend\n");
    REQUIRE(pp.warnings().size() == 1);
    CHECK(pp.warnings()[0].line == 1);
    CHECK_THAT(pp.warnings()[0].message, ContainsSubstring("MADE_UP_BLOCK"));
    CHECK(pp.hasBlock("FOOTER"));
    CHECK_FALSE(PostProcessor::isKnownBlock("MADE_UP_BLOCK"));
    CHECK(PostProcessor::isKnownBlock("CW_HELICAL_ARC_MOVE"));
}

TEST_CASE("AC4: unknown variable tags are an error naming the line", "[post][parse]") {
    SECTION("in a block line") {
        const Error e = parseFail("POST_NAME = \"x\"\nbegin FEED_MOVE\n\"G1 [X] [WIBBLE]\"\nend\n");
        CHECK(e.line == 3);
        CHECK_THAT(e.describe(), ContainsSubstring("line 3") && ContainsSubstring("WIBBLE"));
    }
    SECTION("in a VAR definition") {
        const Error e = parseFail("\n\nVAR FOO = [WIBBLE|A|W|1.0]\n");
        CHECK(e.line == 3);
        CHECK_THAT(e.describe(), ContainsSubstring("line 3") && ContainsSubstring("WIBBLE"));
    }
    SECTION("in an unknown block too") {
        CHECK(parseFail("begin MADE_UP\n\"[NOPE]\"\nend\n").line == 2);
    }
}

TEST_CASE("Parser: unknown settings and duplicates are warnings", "[post][parse]") {
    const PostProcessor pp = parseOk(R"PP(FANCY_OPTION = "YES"
VAR X1 = [X|A|X|1.3]
VAR X2 = [X|A|X|1.1]
begin FOOTER
"M2"
end
begin FOOTER
"M30"
end
)PP");
    REQUIRE(pp.warnings().size() == 3);
    CHECK(pp.warnings()[0].line == 1);
    CHECK(pp.warnings()[1].line == 3);
    CHECK(pp.warnings()[2].line == 7);
    CHECK(pp.var("X")->name == "X2");
    CHECK(pp.block("FOOTER")->lines.at(0) == "M30");
}

TEST_CASE("Parser: structural errors name the line", "[post][parse]") {
    CHECK(parseFail("begin HEADER\n\"G0\"\n").line == 1);             // missing end
    CHECK(parseFail("\"G0\"\nend\n").line == 1);                      // stray output line
    CHECK(parseFail("POST_NAME = \"a\"\nend\n").line == 2);           // end without begin
    CHECK(parseFail("begin HEADER\nbegin FOOTER\nend\n").line == 2);  // nested begin
    CHECK(parseFail("begin HEADER\nG0 X1\nend\n").line == 2);         // unquoted line
    CHECK(parseFail("begin HEADER\n\"G0 X1\nend\n").line == 2);       // missing quote
    CHECK(parseFail("begin HEADER\n\"G0 [X\"\nend\n").line == 2);     // unterminated [
    CHECK(parseFail("begin\n").line == 1);                            // no block name
    CHECK(parseFail("just words\n").line == 1);                       // unrecognised
    CHECK(parseFail("VAR = [X|A|X|1.3]\n").line == 1);                // VAR without name
}

TEST_CASE("Parser: invalid setting values are errors naming the line", "[post][parse]") {
    CHECK(parseFail("UNITS = \"FURLONGS\"\n").line == 1);
    CHECK(parseFail("\nLINE_NUMBER_START = ten\n").line == 2);
    CHECK(parseFail("MIN_ARC_RADIUS = big\n").line == 1);
    CHECK(parseFail("INVERSE_TIME_MODE = \"MAYBE\"\n").line == 1);
    CHECK(parseFail("SUBSTITUTE = \"abc\"\n").line == 1);
    CHECK(parseFail("LINE_ENDING = \"[13\"\n").line == 1);
    CHECK(parseFail("LINE_ENDING = \"[999]\"\n").line == 1);
    CHECK(parseFail("ROTARY_WRAP_X = \"AB\"\n").line == 1);
    const Error both = parseFail("ROTARY_WRAP_X = \"A\"\nROTARY_WRAP_Y = \"A\"\n");
    CHECK_THAT(both.message, ContainsSubstring("ROTARY_WRAP"));
}
