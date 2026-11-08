#pragma once

// Internal to core (contour_ops.cpp, intersect.cpp): a per-thread count of the work done by
// closestPoint(BezierSpan, ...) (T-0180), so the intersection budget can be charged the true cost
// of the global search instead of a flat estimate (its cost grows with how badly conditioned the
// quintic is, which for huge extents is orders of magnitude above the typical).

#include <cstdint>

namespace carvetto::geometry::detail {

/// Units of work done by closestPoint on this thread so far (about 4 ns Release CPU each).
inline std::uint64_t& closestPointWork() {
    thread_local std::uint64_t work = 0;
    return work;
}

}  // namespace carvetto::geometry::detail
