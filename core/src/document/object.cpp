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
