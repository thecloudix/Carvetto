#include "carvetto/document/object_property_commands.hpp"

#include <cmath>
#include <string>
#include <type_traits>
#include <variant>

#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"

namespace carvetto::document {

namespace {

bool coordOk(double v) {
    return std::isfinite(v) && std::fabs(v) <= SetShapeParamsCommand::kMaxCoordinateMm;
}

bool pointOk(geometry::Vec2 p) {
    return coordOk(p.x) && coordOk(p.y);
}

/// True for a parametric shape whose numbers are all finite and within bounds.
bool paramsFinite(const CreationParams& params) {
    return std::visit(
        [](const auto& p) -> bool {
            using T = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<T, RectangleParams>) {
                return pointOk(p.centreMm) && coordOk(p.widthMm) && coordOk(p.heightMm) && coordOk(p.cornerRadiusMm);
            } else if constexpr (std::is_same_v<T, CircleParams>) {
                return pointOk(p.centreMm) && coordOk(p.radiusMm);
            } else if constexpr (std::is_same_v<T, EllipseParams>) {
                return pointOk(p.centreMm) && coordOk(p.widthMm) && coordOk(p.heightMm) && std::isfinite(p.angle);
            } else if constexpr (std::is_same_v<T, PolygonParams>) {
                return pointOk(p.centreMm) && coordOk(p.radiusMm) && std::isfinite(p.angle);
            } else if constexpr (std::is_same_v<T, StarParams>) {
                return pointOk(p.centreMm) && coordOk(p.outerRadiusMm) && coordOk(p.innerRadiusMm) &&
                       std::isfinite(p.angle);
            } else {
                return false;  // NoParams, TextParams
            }
        },
        params);
}

}  // namespace

std::string nameProblem(const std::string& name, bool required) {
    if (name.size() > kMaxNameBytes) {
        return "The name is too long (at most " + std::to_string(kMaxNameBytes) + " bytes).";
    }
    bool blank = true;
    for (const char ch : name) {
        const auto c = static_cast<unsigned char>(ch);
        if (c < 0x20 || c == 0x7f) {
            return "The name contains a control character.";
        }
        if (c != ' ') {
            blank = false;
        }
    }
    if (required && blank) {
        return "The name must not be empty.";
    }
    return {};
}

bool RenameObjectCommand::apply(Document& doc) {
    Object* obj = doc.findObject(id_);
    if (obj == nullptr || !nameProblem(name_, false).empty() || obj->name == name_) {
        return false;
    }
    previous_ = obj->name;
    obj->name = name_;
    return true;
}

void RenameObjectCommand::revert(Document& doc) {
    if (Object* obj = doc.findObject(id_)) {
        obj->name = previous_;
    }
}

bool SetShapeParamsCommand::apply(Document& doc) {
    Object* obj = doc.findObject(id_);
    VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
    if (vec == nullptr || vec->params.index() != params_.index() || !paramsFinite(params_) ||
        !paramsFinite(vec->params) || vec->params == params_) {
        return false;
    }
    auto built = buildContours(params_);
    if (!built.ok()) {
        return false;
    }
    for (const auto& contour : built.value()) {
        const geometry::Box2 b = contour.bounds();
        if (!b.isEmpty() && (!pointOk(b.min) || !pointOk(b.max))) {
            return false;
        }
    }
    before_ = *vec;
    vec->contours = std::move(built.value());
    vec->params = params_;
    return true;
}

void SetShapeParamsCommand::revert(Document& doc) {
    Object* obj = doc.findObject(id_);
    if (obj != nullptr && before_ && obj->isVector()) {
        *obj->asVector() = *before_;
    }
}

}  // namespace carvetto::document
