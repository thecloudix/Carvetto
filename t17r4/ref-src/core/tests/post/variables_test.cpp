#include "carvetto/post/variables.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

using namespace carvetto::post;
using Catch::Matchers::ContainsSubstring;

TEST_CASE("AC2: [X|A|X|1.3] renders 12.3456 as X12.346", "[post][vars]") {
    const auto def = parseVarSpec("[X|A|X|1.3]", 7);
    REQUIRE(def.ok());
    const VarDef& v = def.value();
    CHECK(v.tag == "X");
    CHECK(v.mode == OutputMode::Always);
    CHECK_FALSE(v.incremental);
    CHECK(v.prefix == "X");
    CHECK(v.format == NumberFormat{false, 1, 3});
    CHECK(v.multiplier == 1.0);
    CHECK(v.line == 7);
    CHECK(v.prefix + formatNumber(v.format, 12.3456) == "X12.346");
}

TEST_CASE("Number formats: decimals, padding, sign, rounding, no negative zero", "[post][vars]") {
    CHECK(formatNumber({false, 1, 0}, 18000.0) == "18000");
    CHECK(formatNumber({false, 1, 0}, 2.5) == "3");
    CHECK(formatNumber({false, 1, 0}, -2.5) == "-3");
    CHECK(formatNumber({false, 1, 4}, 0.0) == "0.0000");
    CHECK(formatNumber({false, 1, 3}, -0.0004) == "0.000");
    CHECK(formatNumber({false, 1, 3}, -1.2344) == "-1.234");
    CHECK(formatNumber({false, 3, 2}, 7.5) == "007.50");
    CHECK(formatNumber({false, 3, 2}, -7.5) == "-007.50");
    CHECK(formatNumber({true, 1, 3}, 1.0) == "+1.000");
    CHECK(formatNumber({true, 1, 3}, -1.0) == "-1.000");
    CHECK(formatNumber({true, 1, 1}, 0.0) == "+0.0");
    CHECK(formatNumber({false, 1, 3}, 123456.7891) == "123456.789");
}

TEST_CASE("parseNumberFormat accepts [+]W.D and rejects junk", "[post][vars]") {
    CHECK(parseNumberFormat("1.3").value() == NumberFormat{false, 1, 3});
    CHECK(parseNumberFormat("+2.4").value() == NumberFormat{true, 2, 4});
    CHECK(parseNumberFormat("1").value() == NumberFormat{false, 1, 0});
    CHECK(parseNumberFormat("1.0").value() == NumberFormat{false, 1, 0});
    for (const char* bad : {"", "x", "1.", ".3", "1.3.2", "-1.3", "1.99", "1.a"}) {
        INFO(bad);
        CHECK_FALSE(parseNumberFormat(bad, 4).ok());
    }
    CHECK(parseNumberFormat("q", 4).error().line == 4);
}

TEST_CASE("parseVarSpec: modes, prefix spaces, multiplier", "[post][vars]") {
    const auto c = parseVarSpec("[F|C| F|1.1]");
    REQUIRE(c.ok());
    CHECK(c.value().mode == OutputMode::Changed);
    CHECK(c.value().prefix == " F");
    CHECK(c.value().format.decimals == 1);

    const auto m = parseVarSpec("[ S | A | S | 1.0 | 0.001 ]");
    REQUIRE(m.ok());
    CHECK(m.value().tag == "S");
    CHECK(m.value().prefix == " S ");
    CHECK(m.value().multiplier == 0.001);

    const auto inc = parseVarSpec("[Z|CI|Z|1.3]");
    REQUIRE(inc.ok());
    CHECK(inc.value().incremental);
    CHECK(inc.value().mode == OutputMode::Changed);
    CHECK(parseVarSpec("[A|ai|A|1.3]").value().incremental);
}

TEST_CASE("parseVarSpec errors name the line", "[post][vars]") {
    const auto unknown = parseVarSpec("[XX|A|X|1.3]", 12);
    REQUIRE_FALSE(unknown.ok());
    CHECK(unknown.error().line == 12);
    CHECK_THAT(unknown.error().describe(), ContainsSubstring("line 12") && ContainsSubstring("'XX'"));

    CHECK_FALSE(parseVarSpec("X|A|X|1.3", 1).ok());        // no brackets
    CHECK_FALSE(parseVarSpec("[X|A|X]", 1).ok());          // too few fields
    CHECK_FALSE(parseVarSpec("[X|A|X|1.3|1|2]", 1).ok());  // too many fields
    CHECK_FALSE(parseVarSpec("[X|B|X|1.3]", 1).ok());      // bad mode
    CHECK_FALSE(parseVarSpec("[X|A|X|1.3|two]", 1).ok());  // bad multiplier
    CHECK_FALSE(parseVarSpec("[X|A|X|abc]", 1).ok());      // bad format
    const auto incF = parseVarSpec("[F|AI|F|1.0]", 3);     // incremental only on axes
    REQUIRE_FALSE(incF.ok());
    CHECK_THAT(incF.error().message, ContainsSubstring("incremental"));
}

TEST_CASE("Tag table and default definitions", "[post][vars]") {
    REQUIRE(findTag("X") != nullptr);
    CHECK(findTag("X")->axis);
    CHECK(findTag("A")->kind == TagKind::Angle);
    CHECK(findTag("F")->kind == TagKind::FeedRate);
    CHECK(findTag("TOOLNAME")->kind == TagKind::Text);
    CHECK(findTag("ArcMidX") != nullptr);
    CHECK(findTag("x") == nullptr);  // tags are case-sensitive
    CHECK(findTag("BOGUS") == nullptr);
    CHECK(defaultVarDef("N").format.decimals == 0);
    CHECK(defaultVarDef("X").format.decimals == 3);
    CHECK(defaultVarDef("X").prefix.empty());
    CHECK(defaultVarDef("X").mode == OutputMode::Always);
}
