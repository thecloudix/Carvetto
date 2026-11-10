#include "carvetto/document/object.hpp"

namespace carvetto::document {

using geometry::Box2;
using geometry::Vec2;

Box2 VectorObject::bounds() const {
    Box2 b;
    for (const auto& c : contours) {
        b.expand(c.bounds());
    }
    return b;
}

void VectorObject::transform(const geometry::Transform2& m) {
    for (auto& c : contours) {
        c.transform(m);
    }
}

void BitmapObject::transform(const geometry::Transform2& m) {
    if (placementMm.isEmpty()) {
        return;
    }
    Box2 b;
    b.expand(m.apply(placementMm.min));
    b.expand(m.apply(Vec2{placementMm.max.x, placementMm.min.y}));
    b.expand(m.apply(placementMm.max));
    b.expand(m.apply(Vec2{placementMm.min.x, placementMm.max.y}));
    placementMm = b;
}

void DimensionObject::transform(const geometry::Transform2& m) {
    const std::size_t n = kind == DimensionKind::Angular ? 3 : 2;
    const double scale = m.uniformScale();
    if (kind == DimensionKind::Linear || kind == DimensionKind::Aligned) {
        // Keep the dimension line on the same side of the part: map the line's anchor point and
        // re-measure the offset along the new normal.
        const auto axis = [this](Vec2 a, Vec2 b) {
            return kind == DimensionKind::Linear ? (vertical ? Vec2{0.0, 1.0} : Vec2{1.0, 0.0})
                                                 : (b - a).normalized();
        };
        const Vec2 n0 = axis(points[0].pos, points[1].pos).perp();
        const Vec2 anchorPoint = (points[0].pos + points[1].pos) * 0.5 + n0 * offsetMm;
        for (std::size_t i = 0; i < n; ++i) {
            points[i].pos = m.apply(points[i].pos);
        }
        const Vec2 n1 = axis(points[0].pos, points[1].pos).perp();
        const Vec2 mid = (points[0].pos + points[1].pos) * 0.5;
        if (n1 != Vec2{}) {
            offsetMm = (m.apply(anchorPoint) - mid).dot(n1);
        }
        return;
    }
    for (std::size_t i = 0; i < n; ++i) {
        points[i].pos = m.apply(points[i].pos);
    }
    offsetMm *= scale;
}

Object Object::vector(geometry::Contour contour, LayerId layerId, SheetId sheetId, CreationParams params) {
    std::vector<geometry::Contour> contours;
    contours.push_back(std::move(contour));
    return vector(std::move(contours), layerId, sheetId, std::move(params));
}

Object Object::vector(std::vector<geometry::Contour> contours, LayerId layerId, SheetId sheetId,
                      CreationParams params) {
    Object o;
    o.layer = layerId;
    o.sheet = sheetId;
    o.data = VectorObject{std::move(contours), std::move(params)};
    return o;
}

}  // namespace carvetto::document
