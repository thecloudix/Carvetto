#pragma once

// Internal to core (profile.cpp) and its tests (T-0140 round 2, ADR-0016 §14): the pieces of the
// plane that a union of closed rings encloses, in near-linear time.
//
// One Clipper2 union over a whole scene of overlapping bands is a single sweep whose active edge
// list holds every band side by side, so it costs about the square of the bands across the scene
// (a row or grid of shapes whose kerfs overlap). Here the plane is cut into leaves instead: a k-d
// subdivision at median vertex coordinates (on the nanometre grid, moved off every vertex) until
// each leaf holds at most `leafVertices` vertices, or a cut would leave more than 4/5 of them on
// one side or a side narrower than 10 distance tolerances (at least 10 um); every ring reaching
// within a micrometre of a cut is clipped to both sides a micrometre beyond it (Sutherland-
// Hodgman, which keeps the winding number of every point on that side). A ring that surrounds a
// leaf without an edge in it only adds a constant winding number there and is taken out on the
// way down. Each leaf's material is its rectangle less the union of its rings (one small Clipper2
// call), and the parts of material that meet along a shared leaf side over at least 10 nm are
// joined (union-find over the intervals, one sort per cut line); a part on the outer frame is the
// outside. Every other class is a piece: the area of its parts less their islands; and it is
// thick when what is left of it eroded by `clearance` is not empty (each part alone first, whose
// cut edges can only make that smaller, by a point at least `clearance` from its edges or by its
// erosion; when no part alone is thick, the union of the parts).

#include <cstddef>
#include <optional>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::toolpath::detail {

/// A bounded piece of material left by the union of the rings.
struct EnclosedPiece {
    geometry::Box2 bounds;
    double area{0.0};  ///< less its islands
    /// A point of the piece at least `clearance` from the rings' union (a vertex of the eroded
    /// piece) when the piece is thicker than 2 * clearance somewhere; empty when it is not, or when
    /// its area is under `minArea` (not tested).
    std::optional<geometry::Vec2> inner;
};

/// Default most vertices per leaf.
inline constexpr std::size_t kEnclosedLeafVertices = 512;

struct EnclosedOptions {
    double minArea{0.0};    ///< pieces under this area are dropped
    double clearance{0.0};  ///< erosion radius of the thickness test
    geometry::Tolerance tol{};
    double mitreLimit{2.0};
    std::size_t leafVertices{kEnclosedLeafVertices};  ///< most vertices per leaf (tests use few)
};

/// A closed polygon (no repeated closing point).
using PointRing = std::vector<geometry::Vec2>;

/// The pieces the union of `rings` (filled NonZero) encloses with area at least `minArea`, in a
/// deterministic order (by their lowest leaf). See the file comment.
std::vector<EnclosedPiece> enclosedPieces(std::vector<PointRing> rings, const EnclosedOptions& opt);

}  // namespace carvetto::toolpath::detail
