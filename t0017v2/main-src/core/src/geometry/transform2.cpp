#include "carvetto/geometry/transform2.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::geometry {

Transform2 Transform2::rotation(double angle) {
    const double cs = std::cos(angle);
    const double sn = std::sin(angle);
    return {cs, -sn, sn, cs, 0.0, 0.0};
}

Transform2 Transform2::rotation(double angle, Vec2 centre) {
    return translation(centre) * rotation(angle) * translation(-centre);
}

Transform2 Transform2::mirror(Vec2 p, Vec2 dir) {
    const Vec2 u = dir.normalized();
    if (u == Vec2{}) {
        return identity();
    }
    // Reflection matrix across a line through the origin with unit direction u: 2uu^T - I.
    const Transform2 lin{
        2.0 * u.x * u.x - 1.0, 2.0 * u.x * u.y, 2.0 * u.x * u.y, 2.0 * u.y * u.y - 1.0, 0.0, 0.0};
    return translation(p) * lin * translation(-p);
}

bool Transform2::isConformal(double relTol) const {
    const Vec2 col0{a, c};
    const Vec2 col1{b, d};
    const double l0 = col0.lengthSquared();
    const double l1 = col1.lengthSquared();
    const double scale = std::max(l0, l1);
    if (scale <= 0.0) {
        return false;
    }
    return std::fabs(l0 - l1) <= relTol * scale && std::fabs(col0.dot(col1)) <= relTol * scale;
}

double Transform2::uniformScale() const {
    return std::sqrt(std::fabs(determinant()));
}

std::optional<Transform2> Transform2::inverse() const {
    const double det = determinant();
    if (!isInvertible()) {
        return std::nullopt;
    }
    const double ia = d / det;
    const double ib = -b / det;
    const double ic = -c / det;
    const double id = a / det;
    return Transform2{ia, ib, ic, id, -(ia * tx + ib * ty), -(ic * tx + id * ty)};
}

bool approxEqual(const Transform2& x, const Transform2& y, double tol) {
    return std::fabs(x.a - y.a) <= tol && std::fabs(x.b - y.b) <= tol && std::fabs(x.c - y.c) <= tol &&
           std::fabs(x.d - y.d) <= tol && std::fabs(x.tx - y.tx) <= tol && std::fabs(x.ty - y.ty) <= tol;
}

}  // namespace carvetto::geometry
