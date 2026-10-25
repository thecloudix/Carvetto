#pragma once

#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/fixed_point.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::geometry {

/// Boolean operation between a subject set and a clip set of closed contours.
enum class BooleanOp { Union, Difference, Intersection, Xor };

/// Result of a boolean operation. Closed results are polygonal contours of line spans: outers
/// are counter-clockwise (positive area), holes clockwise. Open results come from open subject
/// contours clipped by the operation and keep their direction of travel.
struct BooleanResult {
    std::vector<Contour> closed;
    std::vector<Contour> open;

    bool isEmpty() const { return closed.empty() && open.empty(); }
};

/// Performs `op` with `subject` against `clip` (Union ignores the subject/clip distinction).
///
/// Approach: every contour is polygonised at `tol.chordal`, scaled by `kFixedPointScale` onto
/// Clipper2's integer grid, clipped with the Vatti-based Clipper2 engine, and converted back.
/// Closed input contours become closed polygons; open input contours in `subject` are clipped
/// as open paths (open contours in `clip` are ignored). Degenerate input (fewer than three
/// distinct vertices for a closed contour) contributes nothing. Arcs are not refitted.
BooleanResult booleanOp(BooleanOp op, const std::vector<Contour>& subject, const std::vector<Contour>& clip,
                        const Tolerance& tol = Tolerance::defaults(), FillRule fill = FillRule::NonZero);

/// Convenience: union of one set of closed contours with itself (resolves overlaps and holes).
std::vector<Contour> unionContours(const std::vector<Contour>& contours,
                                   const Tolerance& tol = Tolerance::defaults(),
                                   FillRule fill = FillRule::NonZero);

}  // namespace carvetto::geometry
