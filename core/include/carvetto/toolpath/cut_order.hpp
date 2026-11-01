#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

// Shared toolpath machinery (T-0120, ADR-0016): the order in which separate cuts are machined.
// Reusable by any operation that cuts several closed or open paths.

namespace carvetto::toolpath {

/// Ordering strategies (feature-inventory §5 "Order"). Several may be enabled; then the one with
/// the shortest total travel between cuts is used (ties: the first in declaration order). None
/// enabled = greedy nearest-neighbour ("optimised").
struct OrderOptions {
    /// Keep the order of the input vectors.
    bool selectionOrder{false};
    /// Sort by bounding-box left edge, then bottom edge: column by column, bottom to top.
    bool leftToRight{false};
    /// Sort by bounding-box bottom edge, then left edge: row by row, left to right.
    bool bottomToTop{false};
    /// Rows from the bottom up (a row = cuts whose y-extent overlaps the row's first cut),
    /// alternating left-to-right and right-to-left (serpentine).
    bool grid{false};
    bool operator==(const OrderOptions&) const = default;
};

/// One cut to be ordered.
struct OrderItem {
    /// The cut path; closed paths can contain other items.
    geometry::Contour path;
    /// Position in vector-selection order.
    std::size_t selectionIndex{0};
    /// Where machining of the item starts and ends (for travel cost and nearest-neighbour).
    geometry::Vec2 start;
    geometry::Vec2 end;
};

/// Machining order of `items` (indices into it). Inner cuts always come first: an item whose
/// start point lies inside a closed item (smaller items only, so two loops never contain each
/// other) is cut before it, and the strategy orders siblings within each container.
/// `from` is the tool position before the first cut. Deterministic.
std::vector<std::size_t> orderCuts(const std::vector<OrderItem>& items, const OrderOptions& options,
                                   geometry::Vec2 from, const geometry::Tolerance& tol);

/// Rapid travel of an order: from `from` to the first start, then end → next start.
double travelLength(const std::vector<OrderItem>& items, const std::vector<std::size_t>& order,
                    geometry::Vec2 from);

}  // namespace carvetto::toolpath
