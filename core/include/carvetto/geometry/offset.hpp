#pragma once

#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fixed_point.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::geometry {

/// How convex corners are closed when a contour is offset outwards.
enum class JoinType {
    /// Corner becomes an arc of radius |distance| centred on the original vertex.
    Round,
    /// Corner stays sharp (mitre); corners sharper than `mitreLimit` fall back to Square.
    Sharp,
    /// Corner is cut off by a line at exactly |distance| from the original vertex.
    Square
};

/// Side of an open contour, relative to its direction of travel.
enum class OffsetSide { Left, Right };

/// Offsets closed contours by `distance`: positive moves the outermost contour outwards and any
/// contour of the opposite orientation (a hole) inwards; negative reverses. The orientation of
/// the outermost input contour defines "outwards", so a lone clockwise square still grows with a
/// positive distance, and the result keeps that convention (outers in the outermost input's
/// orientation, holes opposite). Overlapping results are merged and shapes that collapse vanish,
/// so the result may hold fewer or more contours than the input.
///
/// Approach: polygonise at `tol.chordal`, scale by `kFixedPointScale`, run Clipper2's offset
/// engine (round joins are approximated within `tol.chordal`), convert back to polygonal
/// contours of line spans. Arcs are not refitted. Open contours in the list are *outlined*:
/// both sides are offset by |distance| and the ends are capped (round caps for Round joins,
/// square caps otherwise), producing a closed contour. `mitreLimit` is the maximum mitre length
/// as a multiple of |distance| for Sharp joins (minimum 1). A non-finite `distance` (NaN or
/// infinity) yields an empty result.
std::vector<Contour> offsetContours(const std::vector<Contour>& contours, double distance, JoinType join,
                                    const Tolerance& tol = Tolerance::defaults(), double mitreLimit = 2.0);

/// Offsets one open contour to one side by |distance|, keeping it open. A negative `distance`
/// flips the side. Line and arc spans are offset exactly (an arc keeps its centre and its radius
/// grows or shrinks by |distance|); Bezier spans are polygonised first.
///
/// Corners: tangent-continuous junctions need no join. Outside corners get one join span for
/// Round (an arc of radius |distance| about the vertex) and Square (a chamfer line at
/// |distance| from the vertex, the neighbouring lines or arcs being extended along their own
/// carriers to meet it); Sharp extends both neighbours (lines along their direction, arcs along
/// their circle) to where they meet and adds no span, unless that point is farther than
/// `mitreLimit` x |distance| from the vertex, in which case the corner is squared off. Inside
/// corners trim both neighbours to their intersection, and any span (or arc whose offset
/// radius would be negative) that lies wholly within |distance| of the input is removed. So
/// the result has n spans, minus consumed ones, plus one per joined outside corner, and every
/// point of it is at least |distance| from the input (exactly |distance| for Round).
///
/// Approach: build the raw offset (offset spans + joins + inside-corner connectors), split it
/// at its self-intersections and where it crosses the rest of the |distance|-neighbourhood
/// boundary (circles about the vertices, opposite-side offsets), drop the pieces closer than
/// |distance| to the input and stitch the rest. If the surviving pieces do not form one chain
/// (only possible when distant parts of the path come within 2 x |distance| of each other) the
/// longest chain is returned.
///
/// Known limitation (tracked as T-0004): with Sharp or Square joins, an offset that curls back
/// into a closed loop can come back as the loop plus a short tail that crosses its own start,
/// i.e. a self-intersecting chain (about 0.1% of random paths in verification). Round joins
/// are not affected. T-0004 will split such results and return every chain.
///
/// An empty or degenerate contour, or a non-finite `distance`, yields an empty contour. A
/// closed contour is accepted too: the corner between its last and first spans is treated like
/// any other and the result is closed when the surviving chain still forms a loop.
Contour offsetOpenContour(const Contour& contour, double distance, OffsetSide side, JoinType join,
                          const Tolerance& tol = Tolerance::defaults(), double mitreLimit = 2.0);

}  // namespace carvetto::geometry
