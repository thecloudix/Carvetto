#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

double normalizeAngle(double angle) {
    double a = std::fmod(angle, kTwoPi);
    if (a < 0.0) {
        a += kTwoPi;
    }
    // fmod can return exactly kTwoPi after the add for tiny negatives.
    if (a >= kTwoPi) {
        a = 0.0;
    }
    return a;
}

Vec2 Vec2::fromAngle(double angle, double length) {
    return {std::cos(angle) * length, std::sin(angle) * length};
}

Vec2 Vec2::normalized() const {
    const double len = length();
    if (len <= 0.0) {
        return {};
    }
    return {x / len, y / len};
}

bool approxEqual(Vec2 a, Vec2 b, double tol) {
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && distance(a, b) <= tol;
}

}  // namespace carvetto::geometry
