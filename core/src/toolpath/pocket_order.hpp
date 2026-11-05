#pragma once

// Internal to core (pocket.cpp) and its tests: the nearest-first order of a rest tool's parts
// (T-0131), callable on plain points so a test can compare it with a brute-force order (T-0142).

#include <cstddef>
#include <vector>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::toolpath::detail {

/// An order for the points `centres`: grouped by `partOf` (ascending), and within a group greedy
/// nearest neighbour, starting at the lowest (then leftmost) point. Ties go to the lower index.
/// The nearest unused point is found with a growing square search over a `BoxIndex` of the
/// group's points, so thousands of single-part groups stay linear.
std::vector<std::size_t> nearestFirstOrder(const std::vector<geometry::Vec2>& centres,
                                           const std::vector<std::size_t>& partOf);

}  // namespace carvetto::toolpath::detail
