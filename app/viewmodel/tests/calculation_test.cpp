// T-0257: calculation edit boxes in the shared numeric parsers and the toolpath form.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/calculation.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using Catch::Approx;
using document::Units;

namespace {

struct Provider {
    document::Document doc;
    Provider() {
        doc.variables().set("Margin", 5.0);
        setCalculationProvider(this, [this]() {
            return std::optional<CalculationEnv>(environmentOf(doc));
        });
    }
    ~Provider() { clearCalculationProvider(this); }
};

}  // namespace

TEST_CASE("calculation: letter variables follow the job size in display units", "[calculation]") {
    Provider p;
    const double w = p.doc.job().sizeMm.x;
    const double t = p.doc.job().sizeMm.z;
    REQUIRE(w == 300.0);  // the expectations below are not vacuous
    REQUIRE(t == 18.0);
    const Calculation mm = calculate("=W/2 - {Margin}", Units::Millimetres);
    REQUIRE(mm.ok);
    CHECK(mm.value == Approx(w / 2 - 5.0));
    CHECK(mm.usedVariables == std::vector<std::string>{"Margin"});
    const Calculation in = calculate("=T", Units::Inches);
    REQUIRE(in.ok);
    CHECK(in.value == Approx(t / 25.4));
}

TEST_CASE("calculation: parseNumber evaluates = expressions and keeps plain parsing strict", "[calculation]") {
    Provider p;
    ParsedNumber n = parseNumber("= 2 * {Margin} + 1", FieldKind::Length, Units::Millimetres);
    REQUIRE(n.ok);
    CHECK(n.value == Approx(11.0));
    n = parseNumber("=1+1", FieldKind::Length, Units::Inches);  // display units in, millimetres out
    REQUIRE(n.ok);
    CHECK(n.value == Approx(50.8));
    n = parseNumber("=90/2", FieldKind::Angle, Units::Millimetres);
    CHECK(n.value == Approx(45.0));
    CHECK_FALSE(parseNumber("=5/2", FieldKind::Count, Units::Millimetres).ok);
    CHECK(parseNumber("=6/2", FieldKind::Count, Units::Millimetres).value == Approx(3.0));
    // Errors are clear and nothing else is accepted without the leading '='.
    n = parseNumber("={Nope}", FieldKind::Length, Units::Millimetres);
    CHECK_FALSE(n.ok);
    CHECK_FALSE(n.error.empty());
    n = parseNumber("=1/0", FieldKind::Length, Units::Millimetres);
    CHECK_FALSE(n.ok);
    CHECK_FALSE(parseNumber("W/2", FieldKind::Length, Units::Millimetres).ok);
    CHECK_FALSE(parseNumber("=1e308*10", FieldKind::Length, Units::Millimetres).ok);
    CHECK_FALSE(parseNumber("=(((((", FieldKind::Length, Units::Millimetres).ok);
}

TEST_CASE("calculation: the newest provider answers and the older one returns when it is cleared", "[calculation]") {
    Provider outer;
    outer.doc.variables().set("Margin", 1.0);
    {
        Provider inner;
        inner.doc.variables().set("Margin", 2.0);
        CHECK(calculate("={Margin}", Units::Millimetres).value == Approx(2.0));
    }
    CHECK(calculate("={Margin}", Units::Millimetres).value == Approx(1.0));
}

TEST_CASE("calculation: without a provider constants work and variables are errors", "[calculation]") {
    CHECK(calculate("=2^3", Units::Millimetres).value == Approx(8.0));
    CHECK_FALSE(calculate("={A}", Units::Millimetres).ok);
}

TEST_CASE("calculation: text substitution replaces known names only", "[calculation]") {
    document::DocumentVariables vars;
    vars.set("A", 2.5);
    CHECK(substituteVariables("w={A} x={B} {A", vars) == "w=2.5 x={B} {A");
    CHECK(substituteVariables("", vars).empty());
}

TEST_CASE("toolpath form: an expression field keeps its text and drives the value", "[calculation][toolpath]") {
    Provider p;
    const tooling::ToolDatabase db = testing::cliToolDatabase();
    ToolpathForm form(document::ToolpathOperation::Pocket, Units::Millimetres);
    REQUIRE(form.setTool(db, testing::source("end-mill-6mm")).ok);
    REQUIRE(form.setField("cutDepth", "=T/6").ok);
    CHECK(form.expression("cutDepth") == "=T/6");
    CHECK(std::get<toolpath::PocketParams>(form.definition().params).cutDepth ==
          Approx(p.doc.job().sizeMm.z / 6));
    const auto fields = form.fields();
    const auto it = std::find_if(fields.begin(), fields.end(), [](const ToolpathField& f) { return f.id == "cutDepth"; });
    REQUIRE(it != fields.end());
    CHECK(it->text == "=T/6");
    // A refused calculation changes nothing; a plain value replaces the expression.
    const FieldResult bad = form.setField("cutDepth", "={Nope}");
    CHECK_FALSE(bad.ok);
    CHECK(form.expression("cutDepth") == "=T/6");
    REQUIRE(form.setField("cutDepth", "4").ok);
    CHECK(form.expression("cutDepth").empty());
    // Variables named by expressions (also seconds/factor fields) are reported.
    REQUIRE(form.setField("safeZ", "={Margin}").ok);
    CHECK(form.usedVariables() == std::vector<std::string>{"Margin"});
    form.resetValues();
    CHECK(form.expressions().empty());
}
