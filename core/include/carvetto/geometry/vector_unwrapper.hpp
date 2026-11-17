#pragma once

#include <cstddef>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/geometry/vec2.hpp"

// Vector Unwrapper (T-0321, ADR-0114): turns a cross-section drawn in the end view of a rotary job into a
// contour in the unrolled plane, ready to be used as a profile for two-rail rotary modelling. A cross-section
// point is read in polar form about a centre: its angle (counter-clockwise from +X, continued across the
// seam so the contour never jumps) and its distance from the centre. The unrolled point is
// (circumference coordinate, radial distance) = (angle * radius, distance), with the angle negated for a
// flipped design (CylinderMap::angleOf). Units are mm and radians.

namespace carvetto::geometry {

/// Where the polar centre of the cross-section is.
enum class UnwrapCentre {
    Contour,    ///< The centre of the contour's bounding box.
    Workspace,  ///< `UnwrapOptions::workspaceCentre` (the middle of the design area).
    Custom,     ///< `UnwrapOptions::customCentre`.
};

struct UnwrapOptions {
    UnwrapCentre centre{UnwrapCentre::Contour};
    Vec2 workspaceCentre;
    Vec2 customCentre;
    double radius{1.0};  ///< Cylinder radius, mm: arc length per radian of the unrolled circumference. > 0.
    bool flip{false};    ///< Flip Design: the angle runs the other way.
    /// Refine unwrapped vectors: straight segments become curves when unrolled, so insert points until each
    /// piece is within `tolerance` of the true curve (sampled at a quarter, half and three quarters).
    bool refine{false};
    double tolerance{0.01};  ///< mm, > 0; only used when `refine`.
};

/// Most points one unwrapped contour may have (the input plus refinement); more is an error.
inline constexpr std::size_t kMaxUnwrapPoints = 200000;

struct UnwrapResult {
    Polyline contour;   ///< Closed only when the cross-section goes no net turn round the centre.
    Vec2 centre;        ///< The centre that was used.
    int turns{0};       ///< Net whole turns of the angle (a closed contour round the centre has +-1).
};

/// The centre `options` selects for a contour whose bounds are `bounds`.
Vec2 unwrapCentre(const UnwrapOptions& options, const Box2& bounds);

/// Unwraps a flattened cross-section. A closed cross-section that goes round the centre (net turn not zero)
/// comes out open, with its start point repeated one turn further on, so the whole sweep is kept. Errors: fewer
/// than two points, a non-finite point, a bad radius/tolerance, or more than `kMaxUnwrapPoints` points.
/// A point on the centre keeps the angle of the point before it.
ShapeResult<UnwrapResult> unwrapCrossSection(const Polyline& crossSection, const UnwrapOptions& options);

/// The inverse of `unwrapCrossSection` on the vertices: the unrolled contour put back round `centre`.
/// (Refinement points land on the original straight segments within the tolerance.)
Polyline wrapCrossSection(const Polyline& unrolled, Vec2 centre, double radius, bool flip);

}  // namespace carvetto::geometry
