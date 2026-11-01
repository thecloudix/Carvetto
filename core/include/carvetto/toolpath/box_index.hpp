#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/geometry/box2.hpp"

// Shared toolpath machinery (T-0123, ADR-0018): a spatial index over bounding boxes, so the
// vector-pair checks of Profile (grouping, nesting, clearance, ordering) and Pocket (hole
// assignment) cost the neighbours of a vector instead of every vector.

namespace carvetto::toolpath {

/// Uniform grid over a fixed set of bounding boxes, answering "which boxes intersect this box"
/// in time proportional to the boxes near the query.
///
/// The cell size is the larger of the median box size and the size that gives about four boxes'
/// worth of cells over the whole extent (at most 2048 cells per axis). A box is listed in every
/// cell it touches; boxes touching more than 64 cells (a frame round everything) are kept in a
/// separate list that every query checks. Queries apply the exact `Box2::intersects` test and
/// return indices in ascending order, so a loop over the result visits candidates in the same
/// order as a loop over all boxes with a box test: callers stay deterministic and give the same
/// results as the quadratic loops they replace.
class BoxIndex {
   public:
    BoxIndex() = default;
    /// Indexes `boxes`; empty boxes are never returned.
    explicit BoxIndex(std::vector<geometry::Box2> boxes);

    std::size_t size() const { return boxes_.size(); }
    const geometry::Box2& box(std::size_t i) const { return boxes_[i]; }
    /// Bounds of all boxes.
    const geometry::Box2& bounds() const { return all_; }

    /// Indices of the boxes that intersect `query` (boundaries included), ascending.
    std::vector<std::size_t> query(const geometry::Box2& query) const;
    /// Indices of the boxes that contain `p` (boundaries included), ascending.
    std::vector<std::size_t> containing(geometry::Vec2 p) const { return query(geometry::Box2{p, p}); }

   private:
    std::size_t col(double x) const;
    std::size_t row(double y) const;

    std::vector<geometry::Box2> boxes_;
    geometry::Box2 all_;
    double cell_{1.0};
    std::size_t nx_{0};
    std::size_t ny_{0};
    std::vector<std::size_t> start_;  // CSR: cell c holds items_[start_[c] .. start_[c + 1])
    std::vector<std::size_t> items_;
    std::vector<std::size_t> big_;  // boxes touching too many cells: checked by every query
};

}  // namespace carvetto::toolpath
