#include "carvetto/document/shape_params.hpp"

#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace carvetto::document {

namespace {

using geometry::Contour;
using geometry::RectCorner;
using geometry::ShapeResult;

ShapeResult<std::vector<Contour>> wrap(ShapeResult<Contour> r) {
    if (!r.ok()) {
        return r.error();
    }
    std::vector<Contour> v;
    v.push_back(std::move(r.value()));
    return v;
}

}  // namespace

ShapeResult<std::vector<Contour>> buildContours(const CreationParams& params,
                                                const geometry::Tolerance& tol) {
    return std::visit(
        [&](const auto& p) -> ShapeResult<std::vector<Contour>> {
            using T = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<T, RectangleParams>) {
                RectCorner c = RectCorner::Square;
                if (p.corners == RectangleParams::CornerStyle::Rounded) {
                    c = RectCorner::Rounded;
                } else if (p.corners == RectangleParams::CornerStyle::Chamfered) {
                    c = RectCorner::Chamfered;
                }
                return wrap(geometry::buildRectangle(p.centreMm, p.widthMm, p.heightMm, c, p.cornerRadiusMm));
            } else if constexpr (std::is_same_v<T, CircleParams>) {
                return wrap(geometry::buildCircle(p.centreMm, p.radiusMm));
            } else if constexpr (std::is_same_v<T, EllipseParams>) {
                return wrap(geometry::buildEllipse(p.centreMm, p.widthMm, p.heightMm, p.angle, tol));
            } else if constexpr (std::is_same_v<T, PolygonParams>) {
                return wrap(geometry::buildPolygon(p.centreMm, p.sides, p.radiusMm, p.angle));
            } else if constexpr (std::is_same_v<T, StarParams>) {
                return wrap(geometry::buildStar(p.centreMm, p.points, p.outerRadiusMm, p.innerRadiusMm,
                                                p.angle));
            } else {
                return std::string("no creation parameters to rebuild from");
            }
        },
        params);
}

}  // namespace carvetto::document
