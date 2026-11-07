# ADR-0028: Geometry edit operations (fillet, dog-bone / T-bone, trim, extend, join, close)

Date: 2026-11-07 · Status: proposed (T-0208)

## Context
T-0208 adds the pure-geometry functions behind the Fillet, Trim, Extend, Join and Close tools
(`core/geometry/edit_ops.hpp`). The feature inventory names Normal / Dog-Bone / T-Bone fillets
with a radius, "Trim Objects" (clear inside / outside, Refit Curves After Clipping),
"Interactive Vector Trim" (nearest intersections, Rejoin Trimmed Sections), "Extend Vectors"
(open end to its intersection) and node-menu Close / Join, but it does not define the geometry of
the relief fillets or the tolerances. This ADR records the choices. All functions take a
`Tolerance`, return an `EditStatus` instead of throwing, and leave the input untouched.

## Decision

### 1. Normal fillet
- A node is the joint between span `i-1` and span `i` (node 0 of a closed contour joins the last
  and first spans). Open contours have corner nodes `1 .. n-1`.
- The turn is `atan2(tA x tB, tA . tB)` of the end tangent of the incoming and the start tangent of
  the outgoing span. |turn| < 1e-6 is `NoCorner` (tangent-continuous: nothing to round); within
  1e-6 of pi is `Cusp`. The fillet centre lies on the inner side of the turn (left for a left
  turn), so the same code rounds convex and reflex corners.
- Centre = intersection of the two spans' offset curves at the signed distance r (nearest the
  corner). Offset curves are sampled (1 / up to 64 / 32 samples for line / arc / Bezier), segment
  crossings give a start, Newton on `OA(ta) - OB(tb) = 0` (central differences) polishes it to
  ~1e-13 relative. Tangent points are `A(ta)`, `B(tb)`; the spans are cut there exactly (line,
  arc and Bezier sub-spans, no polygonising) and the arc is `ArcSpan` about the centre.
- Fit: both parameters must lie in [0,1]. A span consumed exactly (tangent point at its end)
  is removed. If the radius does not fit: `RadiusTooLarge` (default) or, with
  `FilletOptions::clampToFit`, the largest fitting radius found by bisection (60 steps) is used
  and reported (`appliedRadius`, `clamped`).
- `filletAll` keeps the arcs aside (no insertion, so span indices stay valid) and applies nodes
  highest first, each on the spans as the previous fillets left them: a later fillet sees its
  already trimmed neighbour, which is what makes overlapping fillets on a short edge fail cleanly
  instead of overlapping. Per-corner results are reported. Cost O(corners x samples).
- Radii <= `tol.distance`, non-finite input, zero-length neighbours and gaps larger than the
  tolerance between the two spans are refused with a status.

### 2. Dog-bone and T-bone (the inventory does not define them)
**Both are relief cuts for internal corners only**, i.e. convex vertices of a pocket region (a left
turn of a counter-clockwise contour, a right turn of a clockwise one). Applied at a reflex vertex
they carve a bump into the region, and on the opposite winding the area change is negative.
`filletCorner` and the explicit-node `filletAll` apply them wherever told; `filletAll` with no node
list (the tool default) uses `convexNodes(contour)` for these two kinds (open contours: left turns). They are defined purely from the contour's turn direction, with
`r` = the requested radius (the tool radius). The relief is on the outer side of the turn,
so it enlarges the region on the left of the contour (the pocket); on a clockwise contour the same
node gives a bump, so the caller chooses the nodes.
- **Dog-bone:** a circle of radius r centred on the corner node. The incoming span is cut where
  it is r from the node, the outgoing span where it is r from the node (nearest the node), and an
  arc about the node joins them the long way round, through the outer side: sweep pi + |turn|
  (270 degrees for a right-angle corner), direction = turn direction. Area added for straight
  walls: (pi + |turn|) r^2 / 2. A tool of radius r centred on the node clears the vertex.
- **T-bone:** a semicircle whose diameter lies on one wall starting at the node: from the node to
  the point 2r along that wall, bulging to the outer side (area added pi r^2 / 2 for a straight
  wall). By default the outgoing span carries the relief (`tBoneAlongIncoming = false`: the
  semicircle goes from the node to the point 2r along the outgoing span, which is cut there);
  `tBoneAlongIncoming` mirrors it onto the incoming span. The arc is always the semicircle of
  diameter node-to-cut-point, so it stays a semicircle if the wall is curved.
- Both need the walls to be at least r (T-bone: 2r) long measured from the node as straight
  distance; otherwise `RadiusTooLarge` (or the clamped radius).
- Open question for the owner: whether the product's dog-bone is centred on the node (this ADR)
  or on the corner bisector, and which wall its T-bone uses. Both are one-function changes.

### 3. Trim to boundary
- Boundaries must be closed (`NotClosed` otherwise). Each span of each contour is intersected with
  the boundary spans whose box (inflated by `tol.distance`) meets its box, using
  `intersect(Span, Span)` (ADR-0009); crossings split the span at the exact parameter, overlaps
  split at both ends. Hits within `tol.distance` of each other or of a span end collapse.
- Each piece is classified at its parameter midpoint against the boundary region under a
  `FillRule` (default NonZero): winding number by a scanline stab over the polygonised boundary
  (chordal `tol.chordal`; arcs capped at 4096 chords) plus an exact distance test for
  "on boundary" (within `tol.distance`). Pieces on the boundary are kept for both modes
  (conservative: nothing coincident with the boundary is deleted). Classification is only
  approximate within `tol.chordal` of a curved boundary (the pieces there are all but
  coincident anyway).
- Cuts caused only by tangent touches do not split a span: neighbouring kept pieces of one span
  are re-merged into the original curve (so a line touching a circle stays one span). Consecutive
  kept pieces form one open contour; a closed contour with no removed piece is returned
  unchanged and closed.
- `refitCurves` merges neighbouring collinear lines and neighbouring arcs of one circle in the
  result (the exact cuts mean curves never need refitting for the trim itself). Nothing else is
  approximated.
- Cost: boundary box index (uniform grid + k-d tree fallback, `box_index.hpp`) for candidate
  pairs: O(S + B + candidate pairs) with candidates = boundary boxes meeting a span's box (a
  long diagonal span meets many, by design); classification O(log E + k) per piece with an
  implicit interval tree of polygon edges (k = edges crossing the scanline). 10k spans against
  10k boundary spans: see the performance test (near-linear, 2.5x per doubling).
- Known adversarial quadratic cases (measured by the verifier): long diagonal spans against a grid
  of small squares (6400 spans: 8.6 s), because a diagonal span's box meets many boundary boxes;
  `joinContours` with all end points coincident (64k contours: 6.7 s, every query sees every
  endpoint); and `trimSpansAt`, which uses the all-pairs library `intersect(Contour, Contour)` and
  `selfIntersections` (fine for interactive picks, not for bulk).
- Curve intersections involving a Bezier cost ~ extent / `tol.chordal` chord pairs, so the
  trim, interactive trim and extend operations refuse (`InvalidArgument`) a drawing whose extent
  exceeds 1e6 x `tol.chordal` when a Bezier is involved: scale the tolerance with the drawing. The
  extent of a Bezier is taken from its control polygon (not from `BezierSpan::bounds()`, which
  overflows for controls above ~1e154 and reports a wrong small box). The rule also refuses, falsely,
  honest drawings whose diagonal exceeds ~10 m at the default 0.01 mm chordal tolerance when they
  contain a Bezier; pass a coarser tolerance for such drawings.

### 4. Interactive trim
- Cut points on the contour = its self-intersections plus intersections with the `others`
  contours, as positions `spanIndex + t`. The pick is projected to the nearest point of the
  contour; the run between the nearest cut at-or-before and the nearest cut after it is removed.
  Open contours trim to their end where there is no cut on a side (cuts at the very ends do not
  count); with no cut at all, or a closed contour with fewer than two cuts: `NothingToTrim`.
  A pick exactly on a cut removes the run after it.
- `rejoin`: the remaining sections are joined when their ends meet within `tol.distance` (the
  loop removed from a self-crossing contour); a closed contour's remainder is closed again when
  its ends meet.

### 5. Extend
- The last span (the first, for `ContourEnd::Start`, by reversing) travels on: a line along its
  end tangent, an arc along the rest of its circle, a Bezier along its end tangent as a line.
  The probe is intersected with the target spans (boxes first); the nearest hit farther than
  `tol.distance` from the end wins, ties to the first target in input order. A line end or arc end
  is changed in place so the span count is unchanged and the end is exactly the intersection
  (arc: to the angle of the hit, i.e. within rounding of the circle); a Bezier gets an appended
  tangent line (G1, never an invented curve). No hit: `NoIntersection`; closed contour:
  `ClosedContour`.

### 6. Join and close
- `joinContours`: open contours are chained greedily from the lowest unused input, first growing
  the end, then the start, always taking the nearest end point within `tol.distance` (ties: lowest
  contour index, a start before an end). A hash grid of cell size max(tol, |coord| x 1e-12)
  makes it near-linear. Gaps up to the tolerance are kept as they are (the chain stays
  `isContinuous(tol)`; no span is moved), a chain whose ends meet becomes closed. Closed, empty and
  non-finite inputs pass through. `sources` and `unmatched` report what went where.
- `closeContour`: an end gap within `tol.distance` is snapped shut by moving the last span's end
  (line or Bezier) or else the first span's start; arcs on both sides stay as they are (the
  residual is within the tolerance); a larger gap gets a closing `LineSpan`.

### 7. Robustness
- Non-finite coordinates / radii / tolerances, or any magnitude above 1e12 (mm), negative or NaN
  tolerances: `InvalidArgument` at every public entry point (`kMaxMagnitude`). All results are
  deterministic (no hashing order or pointer values influence output).
- Span lengths inside these functions use a 32-sample estimate, not `BezierSpan::length`, which
  recurses without a useful bound on looped Beziers with coincident ends.

## Consequences
- The UI cards can call these directly; statuses carry the reason for a refusal.
- Dog-bone / T-bone follow ADR text above until the owner decides otherwise.
