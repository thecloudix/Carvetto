#pragma once

// Internal to core (offset.cpp) and its tests: independent batches for closed offsets
// (T-0141, ADR-0006 §9). Clipper2's union sweep walks its whole active edge list on every
// scanbeam, so rings side by side (a row of thousands of small shapes) cost O(n^2). Rings whose
// raw offsets cannot meet are offset in separate Clipper2 calls instead.

#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "offset_dust.hpp"

namespace carvetto::geometry::detail {

/// Batching starts when at least this many batches overlap one horizontal line (the sweep would
/// hold all of them in its active edge list at once). Below it, one Clipper2 call as before.
inline constexpr std::size_t kOffsetBatchMinDepth = 32;

/// Partitions boxes into batches by recursive cuts along x and y at gaps (a coordinate no box
/// covers, edges included, so touching boxes stay together): boxes in different batches never
/// overlap or touch. Each batch lists its indices in ascending order; batches are ordered by
/// their first index. Deterministic; O(n log n) per cut level.
std::vector<std::vector<std::size_t>> separableBatches(const std::vector<GridBox>& boxes);

/// Largest number of batches whose y-ranges (the union of their boxes' [top, bottom]) contain
/// one common y.
std::size_t batchDepth(const std::vector<GridBox>& boxes, const std::vector<std::vector<std::size_t>>& batches);

/// `offsetContours` with an explicit batching threshold: `minBatchDepth` = kOffsetBatchMinDepth
/// is `offsetContours` itself; a huge value never batches (one Clipper2 call over everything); 0
/// or 1 batches whenever the input splits.
std::vector<Contour> offsetContoursBatched(const std::vector<Contour>& contours, double distance, JoinType join,
                                           const Tolerance& tol, double mitreLimit, std::size_t minBatchDepth);

}  // namespace carvetto::geometry::detail
