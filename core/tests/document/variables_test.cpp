#include "carvetto/document/variables.hpp"

#include <limits>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace carvetto::document;
using Catch::Matchers::WithinAbs;

TEST_CASE("variable names must start with a letter and use letters, digits or underscore",
          "[document][variables]") {
    CHECK(DocumentVariables::isValidName("a"));
    CHECK(DocumentVariables::isValidName("Depth"));
    CHECK(DocumentVariables::isValidName("pocket_depth_2"));
    CHECK(DocumentVariables::isValidName("X1"));

    CHECK_FALSE(DocumentVariables::isValidName(""));
    CHECK_FALSE(DocumentVariables::isValidName("1depth"));
    CHECK_FALSE(DocumentVariables::isValidName("_depth"));
    CHECK_FALSE(DocumentVariables::isValidName("pocket depth"));
    CHECK_FALSE(DocumentVariables::isValidName("pocket-depth"));
    CHECK_FALSE(DocumentVariables::isValidName("depth."));
    CHECK_FALSE(DocumentVariables::isValidName("dépth"));
}

TEST_CASE("variables are set, read, overwritten and removed", "[document][variables]") {
    DocumentVariables vars;
    CHECK(vars.empty());
    CHECK(vars.set("Depth", 6.0));
    CHECK_FALSE(vars.set("2bad", 1.0));
    CHECK(vars.size() == 1);
    CHECK(vars.contains("Depth"));
    CHECK_FALSE(vars.contains("depth"));  // case-sensitive
    REQUIRE(vars.get("Depth").has_value());
    CHECK_THAT(*vars.get("Depth"), WithinAbs(6.0, 0.0));

    CHECK(vars.set("Depth", 7.5));
    CHECK_THAT(*vars.get("Depth"), WithinAbs(7.5, 0.0));

    CHECK(vars.set("Alpha", 1.0));
    CHECK(vars.names() == std::vector<std::string>{"Alpha", "Depth"});

    CHECK(vars.remove("Depth"));
    CHECK_FALSE(vars.remove("Depth"));
    CHECK_FALSE(vars.get("Depth").has_value());
}

TEST_CASE("a referenced variable cannot be removed until released", "[document][variables]") {
    DocumentVariables vars;
    vars.set("Depth", 6.0);
    vars.addReference("Depth");
    vars.addReference("Depth");
    vars.addReference("Missing");  // ignored
    CHECK(vars.referenceCount("Depth") == 2);
    CHECK(vars.referenceCount("Missing") == 0);

    CHECK_FALSE(vars.remove("Depth"));
    vars.removeReference("Depth");
    CHECK_FALSE(vars.remove("Depth"));
    vars.removeReference("Depth");
    vars.removeReference("Depth");  // never negative
    CHECK(vars.referenceCount("Depth") == 0);
    CHECK(vars.remove("Depth"));

    vars.set("Forced", 1.0);
    vars.addReference("Forced");
    vars.forceRemove("Forced");
    CHECK_FALSE(vars.contains("Forced"));
}

TEST_CASE("variables export to text and re-import overwriting same names", "[document][variables]") {
    DocumentVariables vars;
    vars.set("Depth", 6.35);
    vars.set("Count", 3.0);
    vars.set("Tiny", 0.1);
    const std::string text = vars.toText();
    CHECK(text == "Count = 3\nDepth = 6.35\nTiny = 0.1\n");

    DocumentVariables other;
    other.set("Depth", 99.0);
    other.set("Keep", 1.0);
    const auto result = other.importText(text);
    CHECK(result.imported == 3);
    CHECK(result.rejectedLines.empty());
    CHECK_THAT(*other.get("Depth"), WithinAbs(6.35, 0.0));
    CHECK_THAT(*other.get("Count"), WithinAbs(3.0, 0.0));
    CHECK_THAT(*other.get("Keep"), WithinAbs(1.0, 0.0));
    CHECK(other.size() == 4);

    // Round trip is exact.
    DocumentVariables again;
    again.importText(other.toText());
    CHECK(again == other);
}

TEST_CASE("variable import skips comments and reports bad lines", "[document][variables]") {
    DocumentVariables vars;
    const auto result = vars.importText(
        "# header\n"
        "\n"
        "Good = 1.5\r\n"
        "no equals sign\n"
        "9bad = 2\n"
        "Nan = abc\n"
        "  Spaced   =   -2.25  \n"
        "Last = 4");
    CHECK(result.imported == 3);
    CHECK(result.rejectedLines == std::vector<std::size_t>{4, 5, 6});
    CHECK_THAT(*vars.get("Good"), WithinAbs(1.5, 0.0));
    CHECK_THAT(*vars.get("Spaced"), WithinAbs(-2.25, 0.0));
    CHECK_THAT(*vars.get("Last"), WithinAbs(4.0, 0.0));
}

namespace {
double evalOk(std::string_view text, const DocumentVariables& v = {}, ExpressionContext c = {}) {
    const auto r = evaluateExpression(text, v, c);
    INFO(text << " -> " << r.error);
    REQUIRE(r.ok());
    return *r.value;
}
}  // namespace

TEST_CASE("expressions evaluate to hand-computed values", "[document][variables][expression]") {
    CHECK_THAT(evalOk("1+2*3"), WithinAbs(7.0, 1e-12));
    CHECK_THAT(evalOk("=(1+2)*3"), WithinAbs(9.0, 1e-12));
    CHECK_THAT(evalOk("10-4-3"), WithinAbs(3.0, 1e-12));
    CHECK_THAT(evalOk("100/5/2"), WithinAbs(10.0, 1e-12));
    CHECK_THAT(evalOk("2^3^2"), WithinAbs(512.0, 1e-9));   // right-associative
    CHECK_THAT(evalOk("-2^2"), WithinAbs(-4.0, 1e-12));    // power binds tighter than minus
    CHECK_THAT(evalOk("2^-1"), WithinAbs(0.5, 1e-12));
    CHECK_THAT(evalOk("--3"), WithinAbs(3.0, 1e-12));
    CHECK_THAT(evalOk(" 1.5e2 + .5 "), WithinAbs(150.5, 1e-12));
    CHECK_THAT(evalOk("2*-3"), WithinAbs(-6.0, 1e-12));
}

TEST_CASE("letter variables, feet and document variables in expressions",
          "[document][variables][expression]") {
    ExpressionContext ctx;
    ctx.width = 300.0;
    ctx.height = 200.0;
    ctx.thickness = 18.0;
    DocumentVariables vars;
    vars.set("Margin", 12.5);
    CHECK_THAT(evalOk("W/2", vars, ctx), WithinAbs(150.0, 1e-12));
    CHECK_THAT(evalOk("x+y", vars, ctx), WithinAbs(500.0, 1e-12));
    CHECK_THAT(evalOk("H-T", vars, ctx), WithinAbs(182.0, 1e-12));
    CHECK_THAT(evalOk("Z", vars, ctx), WithinAbs(18.0, 1e-12));
    CHECK_THAT(evalOk("P", vars, ctx), WithinAbs(3.14159265358979, 1e-12));
    CHECK_THAT(evalOk("25.4*I", vars, ctx), WithinAbs(1.0, 1e-12));
    CHECK_THAT(evalOk("2*M", vars, ctx), WithinAbs(50.8, 1e-12));
    CHECK_THAT(evalOk("2'", vars, ctx), WithinAbs(609.6, 1e-9));
    CHECK_THAT(evalOk("{Margin}*2+W", vars, ctx), WithinAbs(325.0, 1e-12));
    CHECK_THAT(evalOk("({Margin})'", vars, ctx), WithinAbs(3810.0, 1e-9));

    const auto r = evaluateExpression("{Margin}+{Margin}+{Other}", vars, ctx);
    CHECK_FALSE(r.ok());
    CHECK(r.usedVariables == std::vector<std::string>{"Margin", "Other"});
}

TEST_CASE("T-0271: a foot is 12 in inch display units", "[document][variables][expression]") {
    DocumentVariables vars;
    vars.set("Two", 2.0);
    ExpressionContext inches;
    inches.feet = 12.0;
    CHECK_THAT(evalOk("2'", vars, inches), WithinAbs(24.0, 1e-12));
    CHECK_THAT(evalOk("{Two}'+1", vars, inches), WithinAbs(25.0, 1e-12));
    CHECK_THAT(evalOk("1''", vars, inches), WithinAbs(144.0, 1e-12));
    CHECK_THAT(evalOk("2'", vars, ExpressionContext{}), WithinAbs(609.6, 1e-9));  // millimetres by default
}

TEST_CASE("malformed and hostile expressions fail cleanly and bounded",
          "[document][variables][expression]") {
    DocumentVariables vars;
    vars.set("Big", 1e308);
    vars.set("Bad", std::numeric_limits<double>::infinity());
    const char* bad[] = {"",       "   ",   "=",     "1+",     "*2",   "(1",     "1)",   "1 2",
                         "1/0",    "1/(2-2)", "{",   "{Nope}", "{1x}", "{}",     "Depth", "WW",
                         "W1",     "abc",   "1..2",  ".",      "1e",   "$",      "1+*2", "{Bad}",
                         "{Big}*10", "{Big}^2", "(-8)^0.5", "10^400", "1e999", "'", "2''x"};
    for (const char* b : bad) {
        INFO(b);
        CHECK_FALSE(evaluateExpression(b, vars).ok());
    }
    // 2'' is feet-of-feet by design and fine; just make sure it parses.
    CHECK(evaluateExpression("2''", vars).ok());
}

TEST_CASE("expression nesting and length are bounded", "[document][variables][expression]") {
    const DocumentVariables vars;
    const std::string shallow = std::string(20, '(') + "1" + std::string(20, ')');
    CHECK_THAT(evalOk(shallow), WithinAbs(1.0, 0.0));

    const std::string deep = std::string(100000, '(') + "1" + std::string(100000, ')');
    const auto d = evaluateExpression(deep, vars);
    CHECK_FALSE(d.ok());
    CHECK_FALSE(d.error.empty());

    const std::string within = std::string(200, '(') + "1" + std::string(200, ')');
    CHECK_FALSE(evaluateExpression(within, vars).ok());  // exceeds depth bound, no crash

    std::string powChain = "2";
    for (int i = 0; i < 500; ++i) powChain += "^1";
    CHECK_FALSE(evaluateExpression(powChain, vars).ok());

    CHECK_THAT(evalOk(std::string(500, '-') + "3"), WithinAbs(3.0, 0.0));  // iterative signs
    CHECK_FALSE(evaluateExpression(std::string(5000, '-') + "3", vars).ok());

    std::string longSum = "1";
    for (int i = 0; i < 300; ++i) longSum += "+1";  // 601 chars, long but flat
    CHECK_THAT(evalOk(longSum), WithinAbs(301.0, 0.0));
    CHECK_FALSE(evaluateExpression(std::string(kMaxExpressionLength + 1, '1'), vars).ok());
}

TEST_CASE("deleting a variable used by an expression is refused until released",
          "[document][variables][expression]") {
    DocumentVariables vars;
    vars.set("Depth", 6.0);
    const auto r = evaluateExpression("{Depth}*2", vars);
    REQUIRE(r.ok());
    for (const auto& n : r.usedVariables) vars.addReference(n);
    CHECK_FALSE(vars.remove("Depth"));
    for (const auto& n : r.usedVariables) vars.removeReference(n);
    CHECK(vars.remove("Depth"));
}
