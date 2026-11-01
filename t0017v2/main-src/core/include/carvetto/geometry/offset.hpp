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

/// Offsets one open contour to one side by |distance| and returns the result as chains (see
/// "Several chains" below). A negative `distance` flips the side. Line and arc spans are offset
/// exactly (an arc keeps its centre and its radius grows or shrinks by |distance|); Bezier
/// spans are polygonised first.
///
/// Corners: tangent-continuous junctions need no join. Outside corners get one join span for
/// Round (an arc of radius |distance| about the vertex) and Square (a chamfer line at
/// |distance| from the vertex, the neighbouring lines or arcs being extended along their own
/// carriers to meet it); Sharp extends both neighbours (lines along their direction, arcs along
/// their circle) to where they meet and adds no span, unless that point is farther than
/// `mitreLimit` x |distance| from the vertex, in which case the corner is squared off. Inside
/// corners trim both neighbours to their intersection, and any span (or arc whose offset
/// radius would be negative) that lies wholly within |distance| of the input is removed. So
/// when the offset stays in one piece it is one chain of n spans, minus consumed ones, plus one
/// per joined outside corner, and every point of every chain is at least |distance| from the
/// input (exactly |distance| for Round).
///
/// Approach: build the raw offset (offset spans + joins + inside-corner connectors), split it
/// at its self-intersections and where it crosses the rest of the |distance|-neighbourhood
/// boundary (circles about the vertices, offsets on both sides), drop the pieces closer than
/// |distance| to the input, and with Sharp/Square also the pieces strictly inside the region a
/// mitre or chamfer adds (so a neighbour running past a mitre loses its stub), then stitch the
/// rest. Candidate pairs come from a uniform grid, so the cost is close to linear in the span
/// count. See ADR-0012.
///
/// Several chains: when distant parts of the path come within 2 x |distance| of each other the
/// surviving offset falls apart into several chains, and all of them are returned, ordered by
/// where they start along the input path. Each chain runs in the input's direction (nothing is
/// reversed). No returned chain intersects or touches itself: where the offset crosses itself
/// (an X) or runs into itself (a T), it is cut there into the part before that point, a closed
/// loop through it (`closed` = true) and the part after it. Different chains may touch or cross.
/// Pieces no longer than tol.distance are dropped (unless that would open a gap wider than the
/// tolerance), so chains are continuous within it and a stretch of valid offset no longer than it
/// may be missing.
///
/// An empty or degenerate contour, or a non-finite `distance`, yields no chains; a zero
/// `distance` yields the (flattened) input as one chain. A closed contour is accepted too: the
/// corner between its last and first spans is treated like any other, and a result that still
/// goes all the way round is one closed chain.
std::vector<Contour> offsetOpenContour(const Contour& contour, double distance, OffsetSide side,
                                       JoinType join, const Tolerance& tol = Tolerance::defaults(),
                                       double mitreLimit = 2.0);

}  // namespace carvetto::geometry
