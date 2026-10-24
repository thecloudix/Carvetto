#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

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
