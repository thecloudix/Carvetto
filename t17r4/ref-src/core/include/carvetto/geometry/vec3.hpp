#pragma once

#include <cmath>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// 3D vector / point in millimetres (machine coordinates, Z up). Plain value type; the
/// minimal set of operations needed by toolpaths. Added by T-0040; T-0001 headers untouched.
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    /// Lifts a 2D point to the given height.
    constexpr Vec3(Vec2 xy_, double z_) : x(xy_.x), y(xy_.y), z(z_) {}

    /// The XY projection.
    constexpr Vec2 xy() const { return {x, y}; }

    constexpr Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3& operator+=(Vec3 o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    constexpr Vec3& operator-=(Vec3 o) {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }
    constexpr bool operator==(const Vec3&) const = default;

    /// Dot product.
    constexpr double dot(Vec3 o) const { return x * o.x + y * o.y + z * o.z; }
    /// Cross product.
    constexpr Vec3 cross(Vec3 o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    /// Euclidean length.
    double length() const { return std::sqrt(lengthSquared()); }
    /// Squared length (no sqrt).
    constexpr double lengthSquared() const { return x * x + y * y + z * z; }
    /// Unit vector; returns {0,0,0} for a zero vector.
    Vec3 normalized() const {
        const double len = length();
        return len == 0.0 ? Vec3{} : *this / len;
    }
};

constexpr Vec3 operator*(double s, Vec3 v) {
    return v * s;
}

/// Distance between two points.
inline double distance(Vec3 a, Vec3 b) {
    return (a - b).length();
}

/// Linear interpolation: a at t=0, b at t=1.
constexpr Vec3 lerp(Vec3 a, Vec3 b, double t) {
    return a + (b - a) * t;
}

/// True when every component of a and b agrees within `tol`.
inline bool approxEqual(Vec3 a, Vec3 b, double tol = 1e-9) {
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}

}  // namespace carvetto::geometry
