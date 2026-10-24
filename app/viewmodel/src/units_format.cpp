#include "carvetto/viewmodel/units_format.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <string>

namespace carvetto::viewmodel {

int displayDecimals(document::Units units) {
    return units == document::Units::Inches ? 3 : 2;
}

std::string formatFixed(double value, int decimals) {
    if (!std::isfinite(value)) {
        return std::isnan(value) ? "nan" : (value > 0.0 ? "inf" : "-inf");
    }
    std::array<char, 64> buf{};
    const auto res =
        std::to_chars(buf.data(), buf.data() + buf.size(), value, std::chars_format::fixed, decimals);
    if (res.ec != std::errc{}) {
        return "overflow";
    }
    std::string text(buf.data(), res.ptr);
    if (!text.empty() && text.front() == '-' && text.find_first_not_of("0.", 1) == std::string::npos) {
        text.erase(0, 1);
    }
    return text;
}

std::string formatLength(double mm, document::Units units) {
    return formatFixed(document::toDisplay(mm, units), displayDecimals(units));
}

std::string formatCursorPosition(geometry::Vec2 mm, document::Units units) {
    return "X " + formatLength(mm.x, units) + "  Y " + formatLength(mm.y, units) + " " +
           std::string(document::unitSuffix(units));
}

}  // namespace carvetto::viewmodel
