#include "carvetto/document/double_sided.hpp"

#include <algorithm>

namespace carvetto::document {

using geometry::Box2;
using geometry::Vec2;

Vec2 flipPoint(const Box2& material, FlipDirection direction, Vec2 p) {
    if (direction == FlipDirection::Horizontal) {
        return {material.min.x + material.max.x - p.x, p.y};
    }
    return {p.x, material.min.y + material.max.y - p.y};
}

Box2 flipBox(const Box2& material, FlipDirection direction, const Box2& box) {
    const Vec2 a = flipPoint(material, direction, box.min);
    const Vec2 b = flipPoint(material, direction, box.max);
    return Box2{{std::min(a.x, b.x), std::min(a.y, b.y)}, {std::max(a.x, b.x), std::max(a.y, b.y)}};
}

double materialTopZ(ZZero zero, bool zeroOffSameSide, double thicknessMm, Side side) {
    const bool bedZero = zero == ZZero::MachineBed;
    if (side == Side::Front || !zeroOffSameSide) {
        return bedZero ? thicknessMm : 0.0;
    }
    // Back side measured from the front side's zero face.
    return bedZero ? 0.0 : thicknessMm;
}

std::vector<toolpath::Move> shiftMovesZ(const std::vector<toolpath::Move>& moves, double dz) {
    std::vector<toolpath::Move> out = moves;
    for (toolpath::Move& m : out) {
        m.to.z += dz;
    }
    return out;
}

}  // namespace carvetto::document
