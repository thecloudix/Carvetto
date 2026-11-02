#include "carvetto/document/variables.hpp"

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
