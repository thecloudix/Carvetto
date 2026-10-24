#pragma once

#include <cmath>
#include <numbers>

namespace carvetto::geometry {

/// Pi, in radians.
inline constexpr double kPi = std::numbers::pi_v<double>;
/// Two pi, in radians.
inline constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

/// Wraps an angle into [0, 2*pi).
double normalizeAngle(double angle);

/// 2D vector / point in millimetres. Plain value type.
struct Vec2 {
    double x{0.0};
    double y{0.0};

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    /// Unit-length vector at `angle` radians from +X, scaled by `length`.
    static Vec2 fromAngle(double angle, double length = 1.0);

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2& operator+=(Vec2 o) {
        x += o.x;
        y += o.y;
        return *this;
    }
    constexpr Vec2& operator-=(Vec2 o) {
        x -= o.x;
        y -= o.y;
        return *this;
    }
    constexpr Vec2& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }
    constexpr bool operator==(const Vec2&) const = default;

    /// Dot product.
    constexpr double dot(Vec2 o) const { return x * o.x + y * o.y; }
    /// 2D cross product (z component of the 3D cross product).
    constexpr double cross(Vec2 o) const { return x * o.y - y * o.x; }
    /// Euclidean length.
    double length() const { return std::hypot(x, y); }
    /// Squared length (no sqrt).
    constexpr double lengthSquared() const { return x * x + y * y; }
    /// Unit vector; returns {0,0} for a zero vector.
    Vec2 normalized() const;
    /// Rotated 90 degrees counter-clockwise.
    constexpr Vec2 perp() const { return {-y, x}; }
    /// Angle from +X in radians, in (-pi, pi]. Zero vector gives 0.
    double angle() const { return (x == 0.0 && y == 0.0) ? 0.0 : std::atan2(y, x); }
};

constexpr Vec2 operator*(double s, Vec2 v) {
    return v * s;
}

/// Distance between two points.
inline double distance(Vec2 a, Vec2 b) {
    return (a - b).length();
}

/// Linear interpolation: a at t=0, b at t=1.
constexpr Vec2 lerp(Vec2 a, Vec2 b, double t) {
    return a + (b - a) * t;
}

/// True when |a-b| <= tol component-wise and in length.
bool approxEqual(Vec2 a, Vec2 b, double tol = 1e-9);

}  // namespace carvetto::geometry
