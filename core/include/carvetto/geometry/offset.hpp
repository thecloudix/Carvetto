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
/// so the result may hold fewer or more contours than the input. An inward offset never comes
/// back inside out: every result point lies inside the region at least |distance| (less
/// 2 x tol.chordal of polygonisation and join error) from its boundary, so an inset by the
/// region's inradius or more yields nothing, and a shape with several lobes keeps exactly the
/// lobes deeper than |distance|; a result contour wound against the outermost input is a genuine
/// hole (T-0018, ADR-0006 §6).
///
/// Grid-level dust is dropped (T-0138, ADR-0006 §8): a result contour that Clipper2's erosion by
/// 4e-6 mm (four grid units of `kFixedPointScale`; more only beyond ~2.8e8 mm, where double
/// rounding exceeds the grid) leaves empty is removed, with any contour nested inside it. That
/// erosion is not exact on acute rings (not monotone in the radius): measured on thin
/// triangles, removed contours have an inradius of up to about 5.6e-6 mm and kept ones down to
/// about 3.6e-6 mm. Near collapse Clipper2 can return rings a few nanometres wide, some wound
/// against their nesting; so every result contour holds a disk about 7e-6 mm across, contours
/// nested at even depth are wound like the outermost input and those at odd depth (holes)
/// against it. The test is per contour, on its own interior: a region narrow only between an
/// outer and its hole (a frame) is kept. The width does not depend on `tol`: a removed contour
/// is narrower than about 1.13e-5 mm, about 8.9x below the smallest chordal tolerance Pocket
/// accepts (1e-4 mm), as Profile since T-0141; Profile offsets at min(tol.chordal, 0.5 x
/// tol.distance) with Bezier input (tol.distance >= 1e-6 mm since T-0142), so only a distance
/// tolerance below about 1.1e-5 mm brings this width up to its 2 x tol.chordal allowance.
///
/// Many closed contours side by side (T-0141, ADR-0006 §9): when at least 32 groups of closed
/// contours whose offsets cannot meet overlap one horizontal line, each group is offset on its
/// own (Clipper2's sweep is quadratic in the shapes it holds side by side), and the results
/// come back group by group in input order. They can then differ from offsetting everything at
/// once by a few grid units (Clipper2 rounds intersections within scanbeams that other shapes
/// subdivide); ring counts and orientation rules are the same. Fewer groups side by side, or
/// any open contour in the list: one Clipper2 call, as before.
///
/// Holes with nothing around them (T-0142, ADR-0006 §10): on an outset (distance > 0 relative to
/// the outermost contour), if a closed contour wound against the outermost one is not nested in
/// a contour wound like it (its box inside that contour's box and its first vertex strictly
/// inside), the closed contours are first replaced by the region they describe (Clipper2's union
/// with the group's fill rule) and that region is offset. Such a contour describes no region, and
/// offset directly it left Clipper2's corner loops behind as leftover rings (or, over-shrunk, a
/// phantom region). Contours that cross each other are not detected. Other input is offset as
/// before.
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
/// copies run both ways (they enclose nothing, whatever their numbers; such a stretch is still a
/// path, see "Narrow loops and needles" below). "Nearly the same" (e.g.
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
/// within 1e-12 x max(1, |x|, |y|, radius) mm, always with the second arc starting level with
/// or ahead of the first's end along its travel. That residual remains where the centre's
/// coordinates are much larger than the point's (e.g. radius 4e4 at 1e4 mm), and at one
/// junction of a closed chain of arcs only: there moving the closing arc's centre would reopen
/// its next junction, so where that would put the next arc behind, the closing arc's start
/// angle is turned instead (its end point stays), and where that cannot help either (the
/// previous arc arriving square to the circle), the loop is closed at another of its arcs. So no
/// span starts behind the previous one's end, with one theoretical exception never seen in
/// ~4 M fuzzed offsets: a closed loop of arcs that no arc can close that way keeps one junction a
/// few coordinate ulps behind. Arc-arc crossings are computed without cancellation, so this holds
/// for circles of any size. Two arcs whose circles only nearly touch (Round arcs about vertices
/// 2 |distance| + g apart meeting at a cusp, g above that 1e-10 limit) are joined by a straight
/// bridge of length about g instead. That holds for continuous input (consecutive input spans
/// sharing their end points). Where the input itself has a gap, an arc-to-arc junction can stay
/// open, ahead of the previous end, by up to the larger of about that input gap (2.4 x seen) and
/// 1e-10 x max(1, |x|, |y|, radius) where the two circles nearly touch (6e-11 mm seen at ~1 mm
/// for circles overlapping by 5.6e-11 mm, input gaps of 1e-16).
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
/// (1e-12 x the coordinates) running the same way are one arc. So a stretch of valid offset no longer than
/// the tolerance may be missing or straightened.
///
/// Narrow loops and needles (T-0128): a closed chain too thin to enclose anything, two spans that never
/// part by more than tol.distance / 2, or any loop whose mean width (2 x area / perimeter) is below
/// tol.distance / 8 and whose every point lies within tol.distance / 8 of another of its spans (e.g. an
/// out-and-back needle whose ends meet within the tolerance, or the lens between two nearly co-circular
/// arcs running opposite ways), is not returned as a loop. If its whole extent is within the tolerance it
/// is dropped; otherwise one of its two strands (the longer, between the two points of the loop farthest
/// apart, taken from the unsnapped offset pieces) is returned as an open chain, which passes within the
/// loop's width of every point of it, wherever it lies within 5e-7 x max(1, |distance|) beyond
/// |distance| or on a Sharp/Square corner region. Where copies of a stretch run both ways (a slot exactly
/// 2 x |distance| wide, give or take the snapping allowance), the chains are joined across it as above,
/// and one copy that lies exactly |distance| from the input there (an offset span within its own extent,
/// or a Round join arc; not a mitre extension or chamfer over a slot narrower than 2 x |distance|) is
/// returned as an open chain of its own. Either is returned only where no other chain passes within
/// tol.distance / 2 of it (sampled every tol.distance / 4) and only in stretches longer than the
/// tolerance, so a channel 2 x |distance| + w wide keeps its path for every w >= 0. Such chains are placed
/// in the order by where they start along the input and may touch the chains next to them. At a reversal
/// cusp (consecutive spans turning back by more than a right angle) whose two spans cross again within the
/// tolerance of their junction (Round arcs about vertices a hair less than 2 x |distance| apart overlap in
/// a lens), both spans are cut back to that crossing, so the chain does not cross itself there; crossings
/// whose place is rounding noise (two circles that only touch, to rounding) are left as they are.
///
/// An empty or degenerate contour, or a non-finite `distance`, yields no chains; a zero
/// `distance` yields the (flattened) input as one chain. A closed contour is accepted too: the
/// corner between its last and first spans is treated like any other, and a result that still
/// goes all the way round is one closed chain.
std::vector<Contour> offsetOpenContour(const Contour& contour, double distance, OffsetSide side,
                                       JoinType join, const Tolerance& tol = Tolerance::defaults(),
                                       double mitreLimit = 2.0);

}  // namespace carvetto::geometry
