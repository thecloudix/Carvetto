#pragma once

// Internal to core (boolean.cpp) and its tests (T-0142, ADR-0006 §10): booleans over many
// far-apart shapes in separate Clipper2 calls, and the strip crop of a big clip for open paths.
// No Clipper2 types, so tests can check the crop against brute force.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "carvetto/geometry/boolean.hpp"

namespace carvetto::geometry::detail {

/// `booleanOp` with an explicit batching threshold: `minBatchDepth` = kOffsetBatchMinDepth is
/// `booleanOp` itself; a huge value never batches (one Clipper2 call over everything); 0 or 1
/// batches whenever the input splits.
BooleanResult booleanOpBatched(BooleanOp op, const std::vector<Contour>& subject, const std::vector<Contour>& clip,
                               const Tolerance& tol, FillRule fill, std::size_t minBatchDepth);

/// A point of the Clipper2 integer grid.
struct GridPoint {
    std::int64_t x{0};
    std::int64_t y{0};
    bool operator==(const GridPoint&) const = default;
};

/// One closed ring on the grid (no repeated closing point).
using GridRing = std::vector<GridPoint>;

/// The edges of a set of rings indexed by their extent along one direction `across`, to cut the
/// rings down to narrow strips across them (`crop`).
class StripIndex {
public:
    StripIndex(std::vector<GridRing> rings, double acrossX, double acrossY);

    /// The rings cut down to what matters inside the strip lo <= p.across <= hi: every edge whose
    /// extent along `across` comes within `margin` of the strip is kept with its original end
    /// points, and each run of edges that stays beyond the margin on one side is replaced by the
    /// straight segment between its ends (which stays on that side, a half-plane being convex).
    /// Rings without a kept edge are dropped (a bounded ring cannot surround the unbounded strip),
    /// rings with every edge kept are returned unchanged, in input order. So the winding number of
    /// every point within `margin` of the strip is the same as for the whole set. O(k log n) for
    /// k kept edges of n.
    std::vector<GridRing> crop(double lo, double hi, double margin) const;

    /// Total number of edges (vertices) of the rings.
    std::size_t edgeCount() const { return lo_.size(); }
    /// The extent of a point along `across`.
    double along(GridPoint p) const {
        return static_cast<double>(p.x) * acrossX_ + static_cast<double>(p.y) * acrossY_;
    }

private:
    std::vector<GridRing> rings_;
    double acrossX_;
    double acrossY_;
    // Edges sorted by their lower extent: ring, edge (from vertex `edge` to the next), extent.
    std::vector<std::size_t> ring_;
    std::vector<std::size_t> edge_;
    std::vector<double> lo_;
    std::vector<double> hi_;
    // Implicit segment tree over the sorted edges: the largest upper extent of each node.
    std::vector<double> maxHi_;
    std::size_t leaves_{0};
};

}  // namespace carvetto::geometry::detail
