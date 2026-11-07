#pragma once

// Internal to core (intersect.cpp) and its tests: the limits the overloads without an
// `IntersectLimits` argument use (T-0180). They are the struct's defaults unless a test installs
// smaller ones for a scope, so that the callers of `intersect` (edit_ops, v_carve, ...) can be
// driven into the truncation path in milliseconds instead of seconds.

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
