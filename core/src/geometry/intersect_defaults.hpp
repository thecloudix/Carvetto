#pragma once

// Internal to core (intersect.cpp) and its tests: the limits the overloads without an
// `IntersectLimits` argument use (T-0180). They are the struct's defaults unless a test installs
// smaller ones for a scope, so that the callers of `intersect` (edit_ops, v_carve, ...) can be
// driven into the truncation path in milliseconds instead of seconds.

#include <cstdint>

#include "carvetto/geometry/intersect.hpp"

namespace carvetto::geometry::detail {

inline IntersectLimits*& defaultLimitsOverride() {
    thread_local IntersectLimits* current = nullptr;
    return current;
}

/// The limits of `intersect(a, b, tol)` and friends on this thread.
inline IntersectLimits defaultLimits() {
    const IntersectLimits* o = defaultLimitsOverride();
    return o != nullptr ? *o : IntersectLimits{};
}

/// Work units (see intersect.cpp) the most recent Bezier pair on this thread spent, for tests that
/// pin the unit counts; 0 when the pair was closed-form or refused.
inline std::uint64_t& lastPairWork() {
    thread_local std::uint64_t work = 0;
    return work;
}

/// Test switch: enumerate chord pairs by the direct double loop even for long polylines, to compare
/// it with the box-index path (they must give identical results).
inline bool& forceDirectChordPairs() {
    thread_local bool direct = false;
    return direct;
}

/// Replaces the default limits on this thread for the lifetime of the object (tests only).
class ScopedDefaultLimits {
   public:
    explicit ScopedDefaultLimits(const IntersectLimits& limits) : mine_(limits), saved_(defaultLimitsOverride()) {
        defaultLimitsOverride() = &mine_;
    }
    ~ScopedDefaultLimits() { defaultLimitsOverride() = saved_; }
    ScopedDefaultLimits(const ScopedDefaultLimits&) = delete;
    ScopedDefaultLimits& operator=(const ScopedDefaultLimits&) = delete;

   private:
    IntersectLimits mine_;
    IntersectLimits* saved_;
};

}  // namespace carvetto::geometry::detail
