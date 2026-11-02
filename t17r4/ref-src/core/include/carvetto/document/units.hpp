#pragma once

#include <string_view>

namespace carvetto::document {

/// Display unit chosen by the user. Everything inside the core is stored in millimetres; these
/// helpers convert at the UI and I/O edges.
enum class Units { Millimetres, Inches };

/// Millimetres per inch (exact by definition).
inline constexpr double kMmPerInch = 25.4;

/// Converts an internal millimetre value to the display unit.
constexpr double toDisplay(double mm, Units units) {
    return units == Units::Inches ? mm / kMmPerInch : mm;
}

/// Converts a value in the display unit to internal millimetres.
constexpr double fromDisplay(double value, Units units) {
    return units == Units::Inches ? value * kMmPerInch : value;
}

/// Short suffix for the unit: "mm" or "in".
constexpr std::string_view unitSuffix(Units units) {
    return units == Units::Inches ? "in" : "mm";
}

}  // namespace carvetto::document
