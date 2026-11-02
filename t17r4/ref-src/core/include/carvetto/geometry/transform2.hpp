#pragma once

#include <optional>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// 2D affine transform stored as a 2x3 matrix:
///
///     | a  b  tx |      x' = a*x + b*y + tx
///     | c  d  ty |      y' = c*x + d*y + ty
///
/// Composition follows matrix convention: `A * B` applies B first, then A.
struct Transform2 {
    double a{1.0};
    double b{0.0};
    double c{0.0};
    double d{1.0};
    double tx{0.0};
    double ty{0.0};

    constexpr Transform2() = default;
    constexpr Transform2(double a_, double b_, double c_, double d_, double tx_, double ty_)
        : a(a_), b(b_), c(c_), d(d_), tx(tx_), ty(ty_) {}

    /// Identity transform.
    static constexpr Transform2 identity() { return {}; }
    /// Pure translation by `offset`.
    static constexpr Transform2 translation(Vec2 offset) { return {1.0, 0.0, 0.0, 1.0, offset.x, offset.y}; }
    /// Counter-clockwise rotation by `angle` radians about the origin.
    static Transform2 rotation(double angle);
    /// Counter-clockwise rotation by `angle` radians about `centre`.
    static Transform2 rotation(double angle, Vec2 centre);
    /// Uniform scale about the origin.
    static constexpr Transform2 scaling(double s) { return {s, 0.0, 0.0, s, 0.0, 0.0}; }
    /// Non-uniform scale about the origin.
    static constexpr Transform2 scaling(double sx, double sy) { return {sx, 0.0, 0.0, sy, 0.0, 0.0}; }
    /// Scale about `centre`.
    static constexpr Transform2 scaling(double sx, double sy, Vec2 centre) {
        return {sx, 0.0, 0.0, sy, centre.x - sx * centre.x, centre.y - sy * centre.y};
    }
    /// Mirror across the vertical line x = axisX.
    static constexpr Transform2 mirrorX(double axisX = 0.0) {
        return {-1.0, 0.0, 0.0, 1.0, 2.0 * axisX, 0.0};
    }
    /// Mirror across the horizontal line y = axisY.
    static constexpr Transform2 mirrorY(double axisY = 0.0) {
        return {1.0, 0.0, 0.0, -1.0, 0.0, 2.0 * axisY};
    }
    /// Mirror across an arbitrary line through `p` with direction `dir`.
    static Transform2 mirror(Vec2 p, Vec2 dir);

    constexpr bool operator==(const Transform2&) const = default;

    /// Applies the full transform (linear part + translation) to a point.
    constexpr Vec2 apply(Vec2 p) const { return {a * p.x + b * p.y + tx, c * p.x + d * p.y + ty}; }
    /// Applies only the linear part (no translation) to a direction vector.
    constexpr Vec2 applyVector(Vec2 v) const { return {a * v.x + b * v.y, c * v.x + d * v.y}; }
    constexpr Vec2 operator()(Vec2 p) const { return apply(p); }

    /// Determinant of the linear part. Negative means the transform mirrors.
    constexpr double determinant() const { return a * d - b * c; }
    /// True when the transform flips orientation.
    constexpr bool isMirroring() const { return determinant() < 0.0; }
    /// True when the linear part is invertible (|det| > eps).
    constexpr bool isInvertible(double eps = 1e-15) const {
        const double det = determinant();
        return det > eps || det < -eps;
    }
    /// True when the transform preserves angles and ratios of lengths
    /// (uniform scale, rotation, optional mirror, translation), so circles stay circles.
    bool isConformal(double relTol = 1e-9) const;
    /// Uniform scale factor of a conformal transform (sqrt|det|). Meaningful only when `isConformal()`.
    double uniformScale() const;

    /// Returns `*this * other`: `other` applied first, then this.
    constexpr Transform2 operator*(const Transform2& o) const {
        return {a * o.a + b * o.c, a * o.b + b * o.d,        c * o.a + d * o.c,
                c * o.b + d * o.d, a * o.tx + b * o.ty + tx, c * o.tx + d * o.ty + ty};
    }
    /// Inverse, or nullopt when singular.
    std::optional<Transform2> inverse() const;
};

/// `compose(A, B)` applies B first, then A (equal to `A * B`).
constexpr Transform2 compose(const Transform2& A, const Transform2& B) {
    return A * B;
}

/// True when all six coefficients agree within `tol`.
bool approxEqual(const Transform2& a, const Transform2& b, double tol = 1e-9);

}  // namespace carvetto::geometry
