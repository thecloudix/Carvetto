#pragma once

// Exact-equivalent shortcuts for the offset's comparisons (T-0170). Each replaces a `std::hypot`
// based test (Vec2::length() / normalized()) with squares or raw vectors and says "not clear"
// (or evaluates the exact expression) when the answer is within a relative 1e-9 of its bound or
// when an intermediate could lose precision (squares outside [1e-250, 1e250], so no denormals).

#include <cmath>

#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry::detail {

/// Exactly `|u x v| <= 1e-12 * |u| * |v|` (two directions are parallel).
inline bool directionsParallel(Vec2 u, Vec2 v) {
    const double den = u.cross(v);
    const double u2 = u.lengthSquared();
    const double v2 = v.lengthSquared();
    if (u2 > 1e-250 && u2 < 1e250 && v2 > 1e-250 && v2 < 1e250) {
        const double bound2 = 1e-24 * u2 * v2;
        const double den2 = den * den;
        if (bound2 > 1e-200 && bound2 < 1e200 && std::isfinite(den2) &&
            (den2 < bound2 * (1.0 - 1e-9) || den2 > bound2 * (1.0 + 1e-9))) {
            return den2 < bound2;
        }
    }
    return std::fabs(den) <= 1e-12 * u.length() * v.length();
}

/// Which side of `level` (in [-1, 1]) the cosine of the turn from direction `u` to `v` lies
/// (the dot product of their unit vectors): -1 below, +1 above, 0 when that is not clear.
inline int cosineSideOf(Vec2 u, Vec2 v, double level) {
    const double u2 = u.lengthSquared();
    const double v2 = v.lengthSquared();
    if (u2 > 1e-250 && u2 < 1e250 && v2 > 1e-250 && v2 < 1e250) {
        const double n = std::sqrt(u2) * std::sqrt(v2);
        const double r = u.dot(v);
        if (r > (level + 1e-9) * n) {
            return 1;
        }
        if (r < (level - 1e-9) * n) {
            return -1;
        }
    }
    return 0;
}

}  // namespace carvetto::geometry::detail
