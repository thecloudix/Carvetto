#pragma once

#include <vector>

#include "carvetto/geometry/vec3.hpp"

// Wrapping the flat design space of a rotary job onto its cylinder (T-0320, ADR-0103). Toolpaths,
// vectors and the model are computed flat: X/Y hold the length along the axis and the distance round
// the circumference, Z the machine height. This maps that to the 3D point on the cylinder for the
// Auto-Wrapping View and the wrapped simulation display. Units are mm and radians.

namespace carvetto::geometry {

struct CylinderMap {
    /// The cylinder lies along X (the circumference is Y) when true, along Y (circumference X) when false.
    bool alongX{true};
    /// Cylinder radius at the surface, mm; the flat circumference coordinate c is the angle c / radius.
    double radius{1.0};
    /// Flip Design: the angle runs the other way round.
    bool flip{false};
    /// Machine Z of the cylinder axis, mm.
    double axisZ{0.0};

    /// Angle (radians) of circumference coordinate `c`: c / radius, negated when flipped.
    double angleOf(double c) const { return (flip ? -c : c) / radius; }
    /// Circumference coordinate of an angle (inverse of angleOf).
    double circOf(double angle) const { return (flip ? -angle : angle) * radius; }
    /// Length along the axis and circumference coordinate of a flat point.
    double alongOf(Vec3 flat) const { return alongX ? flat.x : flat.y; }
    double circOf(Vec3 flat) const { return alongX ? flat.y : flat.x; }

    /// The 3D point of a flat point. The axis is the world X (or Y) axis through the origin at Z 0; the
    /// radial distance is flat.z - axisZ; angle 0 is straight up (+Z) and a positive angle turns the top of
    /// the cylinder towards +Y (cylinder along X) or +X (cylinder along Y).
    Vec3 wrap(Vec3 flat) const;
    /// The flat point of a 3D point on the world frame of `wrap`. The circumference coordinate is chosen
    /// closest to `nearCirc` (the angle is only known modulo a full turn); a point on the axis gives `nearCirc`.
    Vec3 unwrap(Vec3 world, double nearCirc = 0.0) const;
};

/// A flat polyline wrapped onto the cylinder, each segment split so that the angle between consecutive
/// points is at most `maxStepRad` (and the radial/axial change is interpolated linearly in flat space,
/// so a straight flat segment becomes a helix). `maxStepRad` is clamped to 1e-4..pi/2.
std::vector<Vec3> wrapPolyline(const CylinderMap& map, const std::vector<Vec3>& flat, double maxStepRad);

}  // namespace carvetto::geometry
