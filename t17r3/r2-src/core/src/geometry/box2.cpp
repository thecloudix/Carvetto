#include "carvetto/geometry/box2.hpp"

namespace carvetto::geometry {

bool approxEqual(const Box2& a, const Box2& b, double tol) {
    if (a.isEmpty() || b.isEmpty()) {
        return a.isEmpty() && b.isEmpty();
    }
    return approxEqual(a.min, b.min, tol) && approxEqual(a.max, b.max, tol);
}

}  // namespace carvetto::geometry
