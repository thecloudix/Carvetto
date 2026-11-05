#pragma once

// Internal to core (offset.cpp) and its tests (T-0142 round 3, ADR-0006 §10): whether every ring
// wound against a group's lowest contour is nested in a ring wound like it, in near-linear time.
//
// The decision is exactly the one of the plain scan over all pairs: a hole is nested when some ring
// wound like the lowest has a bounding box holding the hole's box and holds the hole's first vertex
// strictly inside, as judged by Clipper2's PointInPolygon (IsInside). Only the work changes:
//   - the last ring wound like the lowest before the hole is tried first (input usually lists a
//     shape's holes right after it);
//   - otherwise candidate rings come from a bounding-volume hierarchy over the boxes, visited smallest box
//     first, so the cost does not depend on how many rings lie to the left of the hole (a column
//     or a grid of touching shapes), and the immediate parent is usually the first candidate;
//   - point-in-ring tests on a ring with many vertices use an index of its edges by their y
//     extent, so a test costs O(k log n) for the k edges that reach the vertex's y instead of
//     O(n). The result is Clipper2's own (same walk restricted to those edges, same arithmetic).
//
// Templates over the ring type (Clipper2's Path64 in offset.cpp, GridRing in tests), defined and
// explicitly instantiated in hole_nesting.cpp so both compile with the core's floating-point flags.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "boolean_batch.hpp"

namespace carvetto::geometry::detail {

/// Where a point lies relative to a ring, as Clipper2's PointInPolygonResult.
enum class RingSide { Outside, Inside, On };

/// Clipper2's PointInPolygon itself (the reference the locator must equal; for tests).
RingSide clipperPointInRing(GridPoint p, const GridRing& ring);

/// Point-in-ring tests against one ring, equal to Clipper2's PointInPolygon. The first test walks
/// the ring; from the second on, a ring of at least kRingIndexMinVertices vertices is indexed.
/// The ring must outlive the locator and stay unchanged.
template <class Ring>
class RingLocator {
public:
    explicit RingLocator(const Ring& ring) : ring_(&ring) {}
    RingSide locate(std::int64_t px, std::int64_t py);

private:
    void buildIndex();
    void stabbed(std::int64_t py, std::vector<std::size_t>& out) const;

    const Ring* ring_;
    std::size_t tests_{0};
    // Edge k runs from vertex k-1 (cyclic) to vertex k. Edges sorted by lower y; implicit segment
    // tree of the largest upper y over that order.
    std::vector<std::size_t> edge_;
    std::vector<std::int64_t> lo_;
    std::vector<std::int64_t> maxHi_;
    std::size_t leaves_{0};
    std::vector<std::size_t> events_;
};

/// Least vertex count for which a ring tested more than once gets an edge index.
inline constexpr std::size_t kRingIndexMinVertices = 48;

/// `kind[i]`: 1 for a ring wound like the lowest, 2 for one wound against it, 0 for neither.
/// True when every ring of kind 2 is nested (see the file comment). Stops at the first that is not.
template <class Ring>
bool everyHoleNested(const std::vector<Ring>& rings, const std::vector<int>& kind);

}  // namespace carvetto::geometry::detail
