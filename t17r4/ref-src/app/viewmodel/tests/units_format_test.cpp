#include "carvetto/viewmodel/units_format.hpp"

#include <catch2/catch_test_macros.hpp>

using carvetto::document::Units;
using carvetto::viewmodel::formatCursorPosition;
using carvetto::viewmodel::formatFixed;
using carvetto::viewmodel::formatLength;

TEST_CASE("cursor coordinates are shown in display units", "[viewmodel][units]") {
    CHECK(formatCursorPosition({12.7, -3.176}, Units::Millimetres) == "X 12.70  Y -3.18 mm");
    CHECK(formatCursorPosition({12.7, -3.175}, Units::Inches) == "X 0.500  Y -0.125 in");
    CHECK(formatCursorPosition({254.0, 0.0}, Units::Inches) == "X 10.000  Y 0.000 in");
}

TEST_CASE("fixed formatting has no negative zero and ignores locale", "[viewmodel][units]") {
    CHECK(formatFixed(-0.0001, 2) == "0.00");
    CHECK(formatFixed(-0.0, 3) == "0.000");
    CHECK(formatFixed(-0.006, 2) == "-0.01");
    CHECK(formatFixed(1234.5, 1) == "1234.5");
    CHECK(formatLength(25.4, Units::Inches) == "1.000");
    CHECK(formatLength(25.4, Units::Millimetres) == "25.40");
}
