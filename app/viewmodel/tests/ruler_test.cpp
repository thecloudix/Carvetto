#include "carvetto/viewmodel/ruler.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdlib>
#include <string>

using carvetto::document::Units;
using carvetto::geometry::Box2;
using carvetto::viewmodel::chooseRulerStep;
using carvetto::viewmodel::formatScaledDecimal;
using carvetto::viewmodel::RulerModel;
using carvetto::viewmodel::RulerOrientation;
using carvetto::viewmodel::RulerStep;
using carvetto::viewmodel::rulerTicks;
using carvetto::viewmodel::ViewTransform;

namespace {

/// Number of digits after the decimal point in a label.
int decimalsIn(const std::string& s) {
    const auto dot = s.find('.');
    return dot == std::string::npos ? 0 : static_cast<int>(s.size() - dot - 1);
}

void checkRuler(const RulerModel& m, Units units, double pxPerMm) {
    const int mant = m.step.mantissa;
    REQUIRE((mant == 1 || mant == 2 || mant == 5));
    const double stepPx = m.step.value() * pxPerMm * (units == Units::Inches ? 25.4 : 1.0);
    REQUIRE(stepPx >= 50.0);
    REQUIRE(stepPx <= 150.0);
    double lastMajor = std::nan("");
    int majors = 0;
    for (const auto& t : m.ticks) {
        if (!t.major) {
            REQUIRE(t.label.empty());
            continue;
        }
        ++majors;
        if (!std::isnan(lastMajor)) {
            const double spacing = std::fabs(t.positionPx - lastMajor);
            REQUIRE(spacing >= 50.0 - 1e-6);
            REQUIRE(spacing <= 150.0 + 1e-6);
        }
        lastMajor = t.positionPx;
        // Labels are exact: no exponent, no more decimals than the step has, and they parse to
        // a multiple of the step.
        REQUIRE(t.label.find('e') == std::string::npos);
        REQUIRE(decimalsIn(t.label) <= std::max(0, -m.step.exponent));
        const double v = std::strtod(t.label.c_str(), nullptr);
        const double k = v / m.step.value();
        REQUIRE(std::fabs(k - std::round(k)) < 1e-9);
    }
    REQUIRE(majors >= 2);
}

}  // namespace

TEST_CASE("AC2: major ticks stay 50-150 px apart for zoom 0.01..1000 px/mm", "[viewmodel][ruler]") {
    const Units units = GENERATE(Units::Millimetres, Units::Inches);
    for (int i = 0; i <= 600; ++i) {
        const double pxPerMm = std::pow(10.0, -2.0 + 5.0 * static_cast<double>(i) / 600.0);
        ViewTransform view;
        view.setViewport(1000.0, 800.0);
        view.set(pxPerMm, {137.25, 611.5});
        INFO("pxPerMm=" << pxPerMm << " units=" << (units == Units::Inches ? "in" : "mm"));
        checkRuler(rulerTicks(view, RulerOrientation::Horizontal, units), units, view.scale());
        checkRuler(rulerTicks(view, RulerOrientation::Vertical, units), units, view.scale());
    }
}

TEST_CASE("AC2: steps are 1/2/5 x 10^n and the smallest that reaches 50 px", "[viewmodel][ruler]") {
    CHECK(chooseRulerStep(1.0) == RulerStep{5, 1});     // 50 px per 50 mm
    CHECK(chooseRulerStep(10.0) == RulerStep{5, 0});    // 5 mm = 50 px
    CHECK(chooseRulerStep(9.0) == RulerStep{1, 1});     // 5 mm = 45 px is too small
    CHECK(chooseRulerStep(100.0) == RulerStep{5, -1});  // 0.5 mm
    CHECK(chooseRulerStep(0.01) == RulerStep{5, 3});    // 5000 mm = 50 px
    CHECK(chooseRulerStep(1000.0) == RulerStep{5, -2});
    CHECK(chooseRulerStep(30.0) == RulerStep{2, 0});
    // Inches use the same rule on decimal inches: 1 in = 254 px at 10 px/mm -> 0.2 in.
    CHECK(chooseRulerStep(254.0) == RulerStep{2, -1});
}

TEST_CASE("AC2: labels are exact decimals", "[viewmodel][ruler]") {
    CHECK(formatScaledDecimal(0, -1) == "0");
    CHECK(formatScaledDecimal(3, -1) == "0.3");
    CHECK(formatScaledDecimal(-3, -1) == "-0.3");
    CHECK(formatScaledDecimal(30, -2) == "0.3");
    CHECK(formatScaledDecimal(125, -1) == "12.5");
    CHECK(formatScaledDecimal(5, -3) == "0.005");
    CHECK(formatScaledDecimal(2, 3) == "2000");
    CHECK(formatScaledDecimal(-15, 0) == "-15");
    CHECK(formatScaledDecimal(10, -1) == "1");
}

TEST_CASE("AC2: 0.1-in steps never print 0.30000000000000004", "[viewmodel][ruler]") {
    ViewTransform view;
    view.setViewport(1000.0, 100.0);
    // 0.1 in = 2.54 mm; 2.54 mm * 25 px/mm = 63.5 px -> step 0.1 in.
    view.set(25.0, {0.0, 50.0});
    const RulerModel m = rulerTicks(view, RulerOrientation::Horizontal, Units::Inches);
    REQUIRE(m.step == RulerStep{1, -1});
    bool sawPointThree = false;
    for (const auto& t : m.ticks) {
        if (t.major) {
            CHECK(t.label.size() <= 4);
            sawPointThree = sawPointThree || t.label == "0.3";
        }
    }
    CHECK(sawPointThree);
}

TEST_CASE("ruler ticks sit at the right screen positions with minor subdivisions", "[viewmodel][ruler]") {
    ViewTransform view;
    view.setViewport(1000.0, 800.0);
    view.zoomToFit(Box2{{0.0, 0.0}, {254.0, 254.0}});
    const RulerModel h = rulerTicks(view, RulerOrientation::Horizontal, Units::Millimetres);
    // 720 px / 254 mm = 2.83 px/mm -> 20 mm = 56.7 px.
    REQUIRE(h.step == RulerStep{2, 1});
    const int divs = h.step.minorDivisions();
    REQUIRE(divs == 4);
    bool foundZero = false;
    for (const auto& t : h.ticks) {
        if (t.major && t.label == "0") {
            foundZero = true;
            CHECK_THAT(t.positionPx, Catch::Matchers::WithinAbs(view.worldToScreen({0.0, 0.0}).x, 1e-9));
        }
    }
    CHECK(foundZero);
    for (std::size_t i = 1; i < h.ticks.size(); ++i) {
        CHECK(h.ticks[i].positionPx > h.ticks[i - 1].positionPx);
    }

    const RulerModel v = rulerTicks(view, RulerOrientation::Vertical, Units::Millimetres);
    // Vertical ticks run top to bottom, so values decrease.
    for (std::size_t i = 1; i < v.ticks.size(); ++i) {
        CHECK(v.ticks[i].positionPx > v.ticks[i - 1].positionPx);
    }
    double prevValue = 1e300;
    for (const auto& t : v.ticks) {
        if (t.major) {
            const double val = std::strtod(t.label.c_str(), nullptr);
            CHECK(val < prevValue);
            prevValue = val;
        }
    }
}

TEST_CASE("ruler is empty for a zero viewport or absurd pan", "[viewmodel][ruler]") {
    ViewTransform view;
    CHECK(rulerTicks(view, RulerOrientation::Horizontal, Units::Millimetres).ticks.empty());
    view.setViewport(1000.0, 800.0);
    view.set(1000.0, {-1e19, 0.0});
    CHECK(rulerTicks(view, RulerOrientation::Horizontal, Units::Millimetres).ticks.empty());
}
