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
/// under "Junctions" (bounded there) and, for input that is not continuous or holds
/// sub-tolerance spans, the limitations described there and just below.
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
/// or on nearly the same one, overlap, their surviving pieces are merged along the carrier: each
/// stretch is kept once, on the copy outermost along the whole of it (handing over where two
/// copies cross; that copy is at least |distance| from every copy's input), or dropped where
/// copies run both ways (they enclose nothing, whatever their numbers). "Nearly the same" (e.g.
/// the chamfers of neighbouring peaks of a steep zig-zag whose vertices carry 1e-9..1e-4 mm of
/// noise) means: pieces that share a stretch of positive length, of elements at most 1e-3 rad
/// apart that drift apart by no more than tol.distance / 2 over their length, and within
/// tol.distance / 2 of each other's carrier for copies running the same way, within the snapping
/// allowance below for copies running opposite ways (wider opposite pairs bound a real thin
/// feature, such as a needle tip, and are kept). Pieces are never projected onto another
/// element: each merged stretch is part of its owner's own surviving piece. So overlaps never
/// make a chain step back or traverse a stretch twice. Where the input has a gap at a vertex
/// (the previous span ends short of the next one's start), a Round corner gets an arc about each
/// end of the gap, so it stays exactly |distance| from both. Surviving pieces are stitched end
/// to start by one global matching (closest pairs first, never closing a loop no longer than
/// 4 x tol.distance, which would be dropped), so the result does not depend on the order of the
/// pieces. Candidate pairs come from uniform grids and raw elements lying wholly
/// within |distance| of one input line are discarded up front, so the cost stays close to linear
/// in the span count even when |distance| is large compared with the features. The constant factor is not
/// uniform, though: on a dense zig-zag with |distance| many times its pitch, Sharp and Square are 10-30x
/// slower than Round, because every corner's mitre or chamfer reaches out about |distance| and overlaps
/// hundreds of its neighbours', which multiplies the pieces to split and prune (pitch 0.02, amplitude 1,
/// |distance| 7, 1000 spans, Release: about 21 s against 0.7 s for Round; tracked as T-0016).
/// See ADR-0012.
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
/// bit-identical too (the second arc's centre is moved by the residual, at most
/// 1e-10 x max(1, |x|, |y|, radius) mm, then by a few ulps of centre and radius), otherwise
/// within 1e-12 x max(1, |x|, |y|, radius) mm (only where the centre's coordinates are much
/// larger than the point's, e.g. radius 4e4 at 1e4 mm, where the nearest such residual that leaves
/// the second arc starting level with or ahead of the first's end is chosen; a closed chain of
/// arcs only is closed by turning its first arc's start angle where moving its centre would reopen
/// its first junction behind), so no span starts behind the previous one's end; arc-arc crossings
/// are computed without cancellation, so this holds for circles of any size. Two arcs whose
/// circles only nearly touch (Round arcs about vertices 2 |distance| + g apart meeting at a cusp,
/// g above that 1e-10 limit) are joined by a straight bridge of length about g instead. That
/// holds for continuous input (consecutive input spans sharing their end points): where the
/// input itself has a gap, an arc-to-arc junction can keep a gap of up to about that input gap.
/// Pieces no longer than
/// tol.distance are dropped where their neighbours can be joined without them:
/// trimmed or extended along their own carriers to where these meet (extensions staying at least
/// |distance| from the input, and for Round not beyond it), or, for a sub-tolerance step
/// sideways, by bending the last tolerance of a neighbour outwards onto the other's end.
/// Otherwise they are kept (so a chain may hold such a short span where the offset really has a
/// sub-tolerance feature), and where there is nothing to keep a straight bridge shorter than
/// the tolerance closes the gap. Such a bridge may come closer to the input than |distance| by
/// its sagitta against the neighbourhood's round boundary, at most tol.distance^2 / (8 |distance|)
/// (1.25e-7 mm at |distance| = 1, 2.5e-6 mm at 0.05 with the default tolerance), and for Round
/// never goes beyond |distance|: where it would (a needle tip whose rounded end is missing), the
/// missing arc about the nearest input point is put in if it passes the checks, and otherwise
/// the chain is cut there into two chains, each continuous, rather than bridged; snapped
/// extensions and bends by at most min(5e-7 x max(1, |distance|), 0.1 tol.distance^2 / |distance|).
/// Pieces of one raw element that the chain steps over a sub-tolerance sliver between are
/// emitted as one span again; across the sliver that span may dip inside |distance| by up to
/// about tol.distance^2 / (4 |distance|) (1.7e-7 mm seen at |distance| = 1.4, as on main). No
/// extension runs along a piece of another chain (that would traverse the stretch twice); such a
/// junction is closed by the other repairs above instead (e.g. the checked straight bridge). An open
/// chain neither starts nor ends with a span no longer than the tolerance: such an end span is
/// dropped.
/// Where a junction gap is closed by moving a line's end (gaps up to 1e-9 x max(1, |x|, |y|)),
/// the moved line is always checked like an extension (at least |distance| from the input
/// within the allowance above, for Round not beyond it), preferring the end that moves away from
/// the input; if neither end passes, the junction is handled as a wider gap. Only an extension
/// no longer than the rounding of the coordinates (16 x 2.2e-16 x max(1, |x|, |y|)) goes
/// unchecked. Every extension, bend or bridge is checked against the input at points at most
/// tol.distance / 8 apart (a moved line at up to 24 points crowded towards its moved end, as it moves by at
/// most that gap), and between those points each local extreme of the distance is located (golden-section
/// search), so the bounds hold between the samples too, not only at them. A span no longer than tol.distance
/// that would step back against the direction of travel (its chord running against both neighbours, or a
/// retrace), as left where three or more carriers nearly meet within the tolerance, is removed (in chains of
/// three or more spans) and its neighbours joined: two arcs on nearly one circle (centres and radii within
/// tol.distance / 2 in all) by one arc on the circle of either, else as above, else by a straight bridge, at
/// most 2 x tol.distance long, that runs forwards with both (taking over the first or last
/// tolerance of one if needed) and passes the same distance checks; only if none of these is
/// possible would it stay (not seen over the fuzzing and noise sweeps of ADR-0012 A1). A span
/// that runs straight back along its neighbour within 2e-9 x max(1, |distance|) mm (or 64
/// rounding units of the coordinates where that is larger, beyond ~1e6 mm; two arcs only if
/// also on one circle to that bound) is a needle tip of rounding width: it is removed and the
/// neighbour cut back to match. The bound does not depend on where the input lies, so a wider
/// needle (e.g. 3.6e-6 mm at 1e4 mm) is kept wherever it lies. Consecutive arcs on one circle to rounding
/// (1e-12 x the coordinates) running the same way are one arc. A closed chain narrower than the tolerance
/// encloses nothing a tool could follow and is dropped: two spans that never part by more than tol.distance /
/// 2, or any loop whose mean width (2 x area / perimeter) is below tol.distance / 8 and whose every point
/// lies within tol.distance / 8 of another of its spans (e.g. the lens between two nearly co-circular arcs
/// running opposite ways). So a stretch of valid offset no longer than the tolerance, or such a needle tip or
/// loop, may be missing or straightened.
///
/// An empty or degenerate contour, or a non-finite `distance`, yields no chains; a zero
/// `distance` yields the (flattened) input as one chain. A closed contour is accepted too: the
/// corner between its last and first spans is treated like any other, and a result that still
/// goes all the way round is one closed chain.
std::vector<Contour> offsetOpenContour(const Contour& contour, double distance, OffsetSide side,
                                       JoinType join, const Tolerance& tol = Tolerance::defaults(),
                                       double mitreLimit = 2.0);

}  // namespace carvetto::geometry
