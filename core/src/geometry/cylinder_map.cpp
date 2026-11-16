#include "carvetto/geometry/cylinder_map.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace carvetto::geometry {

Vec3 CylinderMap::wrap(Vec3 flat) const {
    const double along = alongOf(flat);
    const double r = flat.z - axisZ;
    const double a = angleOf(circOf(flat));
    const double s = r * std::sin(a);
    const double c = r * std::cos(a);
    return alongX ? Vec3{along, s, c} : Vec3{s, along, c};
}

Vec3 CylinderMap::unwrap(Vec3 world, double nearCirc) const {
    const double along = alongX ? world.x : world.y;
    const double s = alongX ? world.y : world.x;
    const double c = world.z;
    const double r = std::sqrt(s * s + c * c);
    double circ = nearCirc;
    if (r > 1e-12) {
        const double a = std::atan2(s, c);
        const double base = circOf(a);
        const double turn = 2.0 * std::numbers::pi * radius;
        circ = base + turn * std::round((nearCirc - base) / turn);
    }
    const double z = r + axisZ;
    return alongX ? Vec3{along, circ, z} : Vec3{circ, along, z};
}

std::vector<Vec3> wrapPolyline(const CylinderMap& map, const std::vector<Vec3>& flat, double maxStepRad) {
    const double step = std::clamp(std::isfinite(maxStepRad) ? maxStepRad : 0.05, 1e-4, std::numbers::pi / 2.0);
    std::vector<Vec3> out;
    out.reserve(flat.size());
    for (std::size_t i = 0; i < flat.size(); ++i) {
        if (i > 0) {
            const double turn = std::fabs(map.angleOf(map.circOf(flat[i])) - map.angleOf(map.circOf(flat[i - 1])));
            const double parts = std::min(std::ceil(turn / step), 100000.0);
            for (double k = 1.0; k < parts; k += 1.0) {
                out.push_back(map.wrap(flat[i - 1] + (flat[i] - flat[i - 1]) * (k / parts)));
            }
        }
        out.push_back(map.wrap(flat[i]));
    }
    return out;
}

}  // namespace carvetto::geometry
