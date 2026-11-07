#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

double normalizeAngle(double angle) {
    // T-0176: std::fmod(x, 2*pi) returns x itself for |x| < 2*pi, so skip its (slow) call there; the
    // result is bit for bit the same. Anything else, NaN and infinities included, takes the long way.
    if (angle >= 0.0 && angle < kTwoPi) {
        return angle;
    }
    if (angle < 0.0 && angle > -kTwoPi) {
        const double w = angle + kTwoPi;
        return w >= kTwoPi ? 0.0 : w;
    }
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
