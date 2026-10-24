#pragma once

#include <cmath>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// Explicit tolerances for geometric algorithms. Passed in; never a global.
/// Distances are millimetres, angles radians.
struct Tolerance {
    /// Two points closer than this are considered coincident (default 0.001 mm).
    double distance{0.001};
    /// Two angles closer than this are considered equal.
    double angle{1e-9};
    /// Maximum chordal deviation when polygonising curves (default 0.01 mm).
    double chordal{0.01};

    /// Default tolerance set.
    static constexpr Tolerance defaults() { return {}; }

    /// True when |a - b| <= distance.
    constexpr bool equalDistance(double a, double b) const { return std::fabs(a - b) <= distance; }
    /// True when the two points are within `distance` of each other.
    bool equalPoints(Vec2 a, Vec2 b) const { return geometry::distance(a, b) <= distance; }
    /// True when |a - b| <= angle.
    constexpr bool equalAngle(double a, double b) const { return std::fabs(a - b) <= angle; }
    /// True when |v| <= distance.
    constexpr bool isZero(double v) const { return std::fabs(v) <= distance; }
};

}  // namespace carvetto::geometry
