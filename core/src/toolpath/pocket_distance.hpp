#pragma once

// Internal to core (pocket.cpp) and its tests: the segment distance behind Pocket's wall index
// (EdgeGrid, ADR-0017 §8a).

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::toolpath::detail {

/// Distance between the segments ab and cd, exact up to the rounding of the end-to-segment
/// distances (0 when they cross). Only a proper crossing (each segment's ends strictly either
/// side of the other's line, by exact orientation signs) gives 0 directly; otherwise the closest
/// pair involves an end, so the four end-to-segment distances give it: touching (an end on the
/// other segment, 0 up to rounding) and nearly collinear segments included (T-0175: rounded
/// orientations of nearly collinear segments can have any sign, and the old non-strict test
/// returned 0 for a segment on an edge's line extension 0.0099 mm beyond its end).
double segmentDistance(geometry::Vec2 a, geometry::Vec2 b, geometry::Vec2 c, geometry::Vec2 d);

}  // namespace carvetto::toolpath::detail
