#include "carvetto/document/relief_model.hpp"

#include <algorithm>
#include <cmath>

#include "carvetto/relief/effects.hpp"

namespace carvetto::document {

namespace {
bool fin(double v) {
    return std::isfinite(v);
}
bool fin(relief::Point p) {
    return fin(p.x) && fin(p.y);
}
}  // namespace

std::string reliefEffectsProblem(const relief::LevelEffects& e) {
    return relief::validateEffects(e) == relief::Status::Ok
               ? std::string{}
               : "the level's clipping, mirror or wrapping settings are invalid (a clip needs at most " +
                     std::to_string(relief::kMaxClipPolygons) + " polygons of 3 or more finite points and at most " +
                     std::to_string(relief::kMaxClipVertices) + " vertices, the mirror centre must be finite, a wrapping needs a positive finite circumference)";
}

std::string reliefComponentProblem(const relief::Component& c) {
    if (c.field.empty()) {
        return "the height field is empty";
    }
    return reliefPropertiesProblem(c);
}

std::string reliefPropertiesProblem(const relief::Component& c) {
    if (c.name.size() > kMaxReliefNameBytes) {
        return "the name is longer than " + std::to_string(kMaxReliefNameBytes) + " bytes";
    }
    if (!fin(c.shapeHeightPercent) || !fin(c.baseHeight) || !fin(c.fade.anchor) || !fin(c.fade.second) ||
        !fin(c.fade.percent) || !fin(c.tilt.pivot) || !fin(c.tilt.raise) || !fin(c.tilt.angleRad)) {
        return "a property is not a finite number";
    }
    if (relief::validateProperties(c) != relief::Status::Ok) {
        return "the fade or tilt direction is degenerate";
    }
    return reliefAppearanceProblem(c.appearance, false);
}

std::string reliefAppearanceProblem(const relief::Appearance& a, bool isLevel) {
    using relief::AppearanceKind;
    using relief::AppearanceMaterial;
    if (a.kind != AppearanceKind::SameAsParent && a.kind != AppearanceKind::SolidColour &&
        a.kind != AppearanceKind::Material && a.kind != AppearanceKind::ColourFromChildren) {
        return "the appearance kind is unknown";
    }
    if (a.kind == AppearanceKind::ColourFromChildren && !isLevel) {
        return "a component has no children: Colour from Children is for levels";
    }
    if (a.kind == AppearanceKind::SolidColour && a.rgb > 0xFFFFFFu) {
        return "the colour must be 0xRRGGBB";
    }
    if (a.kind == AppearanceKind::Material && a.material != AppearanceMaterial::Wood &&
        a.material != AppearanceMaterial::Metal && a.material != AppearanceMaterial::Stone &&
        a.material != AppearanceMaterial::Plastic) {
        return "the appearance material is unknown";
    }
    return {};
}

bool ComponentSelection::add(ComponentId id) {
    if (contains(id)) {
        return false;
    }
    ids_.push_back(id);
    return true;
}

bool ComponentSelection::remove(ComponentId id) {
    const auto it = std::find(ids_.begin(), ids_.end(), id);
    if (it == ids_.end()) {
        return false;
    }
    ids_.erase(it);
    return true;
}

bool ComponentSelection::toggle(ComponentId id) {
    if (remove(id)) {
        return false;
    }
    ids_.push_back(id);
    return true;
}

void ComponentSelection::assign(const std::vector<ComponentId>& ids) {
    ids_.clear();
    for (const ComponentId id : ids) {
        add(id);
    }
}

bool ComponentSelection::contains(ComponentId id) const {
    return std::find(ids_.begin(), ids_.end(), id) != ids_.end();
}

}  // namespace carvetto::document
