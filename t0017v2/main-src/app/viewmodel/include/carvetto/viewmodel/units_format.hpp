#pragma once

#include <string>

#include "carvetto/document/units.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::viewmodel {

/// Decimal places shown for lengths in a unit (2 for mm, 3 for inches).
int displayDecimals(document::Units units);

/// Fixed-point text of `value` with `decimals` places, locale independent ('.' separator), with
/// no "-0.00" (a value that rounds to zero is written unsigned).
std::string formatFixed(double value, int decimals);

/// A millimetre length converted and formatted in `units`, without suffix ("12.70", "0.500").
std::string formatLength(double mm, document::Units units);

/// Status-bar text for a cursor at document position `mm`: "X 12.70  Y -3.18 mm".
std::string formatCursorPosition(geometry::Vec2 mm, document::Units units);

}  // namespace carvetto::viewmodel
