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
/// their circle) to where they first meet and adds no span. An arc may be extended into the gap
/// outside its (already extended) sweep, by at most a quarter turn (90 degrees) and only as far
/// as `mitreLimit` x |distance| from the vertex (for Square too). Remaining fallbacks: Sharp
/// squares the corner off when the meeting point is farther than `mitreLimit` x |distance| from
/// the vertex, when an arc would have to be extended by more than 90 degrees, or when the
/// carriers meet only by wrapping an arc past its own other end. Square falls back to Round when
/// a neighbour cannot reach the chamfer line within 1.5 x |distance| of the vertex (an arc too
/// small or curling away from it, needing more than 90 degrees or ending farther than
/// `mitreLimit` x |distance| from the vertex) or the chamfer would run backwards; a chamfer
/// shorter than the tolerance (nearly tangent corner) becomes the neighbours' meeting point
/// instead, if that is within 1.5 x |distance| and the same arc cap. Inside
/// corners trim both neighbours to their intersection, and any span (or arc whose offset
/// radius would be negative) that lies wholly within |distance| of the input is removed. So
/// when the offset stays in one piece it is one chain of n spans, minus consumed ones, plus one
/// per joined outside corner, and every point of every chain is at least |distance| from the
/// input (exactly |distance| for Round), up to the sub-tolerance junction repairs described
/// under "Junctions".
///
/// Known limitation (input with sub-tolerance spans): spans no longer than tol.distance are left
/// out of the input before offsetting, so the guarantees above hold for that flattened input,
/// not the original. Next to such a span an offset point may be short of |distance| from the
/// original input by up to about tol.distance, a Round join arc about the neighbouring vertex
/// may run up to about tol.distance beyond |distance|, and a stretch of valid offset (seen up
/// to ~1 mm) may be missing. How often depends on the input: about 1 in 1000 offsets in a
/// general random fuzz whose inputs occasionally contain such spans, but over a quarter of
/// offsets of inputs deliberately full of sub-tolerance jogs. Inputs without them are
/// unaffected. Clean the input (join or remove sub-tolerance spans) where this matters.
///
/// Approach: build the raw offset (offset spans + joins + inside-corner connectors), split it
/// at its self-intersections and where it crosses the rest of the |distance|-neighbourhood
/// boundary (the outer arcs of the circles about the vertices, offsets on both sides), drop the
/// pieces closer than |distance| to the input, and with Sharp/Square also the pieces strictly
/// inside the region a mitre or chamfer adds (so a neighbour running past a mitre loses its
/// stub), then stitch the rest and snap the junctions. Where raw elements on one line or circle,
/// or on nearly the same one (within tol.distance / 2 over their common stretch, and nearly
/// parallel over their whole length: e.g. the chamfers of neighbouring peaks of a steep zig-zag
/// whose vertices carry 1e-9..1e-4 mm of noise), overlap, their surviving pieces are merged along
/// the carrier: each stretch is kept once, on the copy farthest from the input (which is at least
/// |distance| from every copy's input), or dropped where as many copies run each way. So
/// overlaps never make a chain step back or traverse a stretch twice. Candidate pairs come from
/// uniform grids and raw elements lying wholly within |distance| of one input line are
/// discarded up front, so the cost stays close to linear in the span count even when |distance|
/// is large compared with the features. The constant factor is not uniform, though: on a dense
/// zig-zag with |distance| many times its pitch, Sharp and Square are 10-30x slower than Round,
/// because every corner's mitre or chamfer reaches out about |distance| and overlaps hundreds of
/// its neighbours', which multiplies the pieces to split and prune (pitch 0.02, amplitude 1,
/// |distance| 7, 1000 spans, Release: about 18 s against 0.7 s for Round). See ADR-0012.
///
/// Several chains: when distant parts of the path come within 2 x |distance| of each other the
/// surviving offset falls apart into several chains, and all of them are returned, ordered by
/// where they start along the input path. Each chain runs in the input's direction (nothing is
/// reversed). No returned chain intersects or touches itself: where the offset crosses itself
/// (an X) or runs into itself (a T), it is cut there into the part before that point, a closed
/// loop through it (`closed` = true) and the part after it. Different chains may touch or cross.
///
/// Junctions (for toolpaths): consecutive spans of a chain, and the last and first spans of a
/// closed chain, share their end point exactly: bit-identical wherever a line is involved;
/// between two arcs (whose end points are computed from centre and angles, ADR-0005) usually
/// bit-identical too (the second arc's centre is nudged by a few ulps), otherwise within
/// 1e-12 x max(1, |x|, |y|, radius) mm (at most 6.3e-13 mm over 540 000 fuzzed offsets with
/// coordinates up to ~100 mm), so no span starts behind the previous one's end. Pieces no
/// longer than tol.distance are dropped where their neighbours can be joined without them:
/// trimmed or extended along their own carriers to where these meet (extensions staying at least
/// |distance| from the input, and for Round not beyond it), or, for a sub-tolerance step
/// sideways, by bending the last tolerance of a neighbour outwards onto the other's end.
/// Otherwise they are kept (so a chain may hold such a short span where the offset really has a
/// sub-tolerance feature), and where there is nothing to keep a straight bridge shorter than
/// the tolerance closes the gap. Such a bridge may come closer to the input than |distance| by its sagitta
/// against the neighbourhood's round boundary, at most tol.distance^2 / (8 |distance|)
/// (1.25e-7 mm at |distance| = 1, 2.5e-6 mm at 0.05 with the default tolerance); snapped
/// extensions and bends by at most min(5e-7 x max(1, |distance|), 0.1 tol.distance^2 / |distance|).
/// Where a junction gap is closed by moving a line's end (gaps up to 1e-9 x max(1, |x|, |y|)),
/// the end that moves away from the input is moved, so the line is never tilted inwards. A span
/// no longer than tol.distance that would step back against the direction of travel (its chord
/// running against both neighbours, or a retrace), as left where three or more carriers nearly
/// meet within the tolerance, is removed and its neighbours joined as above, or by a straight
/// bridge that runs forwards with both (taking over the first or last tolerance of one if
/// needed) and passes the same distance checks; only if none of these is possible would it stay
/// (not seen over the fuzzing and noise sweeps of ADR-0012 A1). So a stretch of valid offset no
/// longer than the tolerance may be missing or straightened.
///
/// An empty or degenerate contour, or a non-finite `distance`, yields no chains; a zero
/// `distance` yields the (flattened) input as one chain. A closed contour is accepted too: the
/// corner between its last and first spans is treated like any other, and a result that still
/// goes all the way round is one closed chain.
std::vector<Contour> offsetOpenContour(const Contour& contour, double distance, OffsetSide side,
                                       JoinType join, const Tolerance& tol = Tolerance::defaults(),
                                       double mitreLimit = 2.0);

}  // namespace carvetto::geometry
