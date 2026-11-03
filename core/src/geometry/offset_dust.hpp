#pragma once

// Internal to core (offset.cpp) and its tests: the nesting pass of the T-0138 dust filter
// (ADR-0006 §8), without Clipper2 types so tests can check it against a brute-force closure.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace carvetto::geometry::detail {

/// Axis-aligned bounding box of a ring on the integer grid (top <= bottom, as Clipper2's Rect64).
struct GridBox {
    std::int64_t left{0};
    std::int64_t top{0};
    std::int64_t right{0};
    std::int64_t bottom{0};
};

/// Marks as dust every ring nested, directly or through other rings, inside a ring already
/// marked: `nested(outer, inner)` decides nesting and is asked only for pairs where `inner` is not
/// dust yet and `boxes[inner]` lies within `boxes[outer]` (edges included); rings with
/// `usable[i]` false are never asked about, as outer or inner. Rings are sorted by left edge and
/// each dust ring visits only the rings whose left edge lies within its x-range, so the cost is
/// O(n log n) plus the pairs whose x-ranges overlap that way.
void markNestedInDust(const std::vector<GridBox>& boxes, const std::vector<bool>& usable,
                      std::vector<bool>& dust,
                      const std::function<bool(std::size_t outer, std::size_t inner)>& nested);

}  // namespace carvetto::geometry::detail
