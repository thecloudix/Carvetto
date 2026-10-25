# ADR-0012: Open-contour offsets return every chain

Date: 2026-10-24 · Status: accepted (director, 2026-10-24); amends ADR-0006 §5 (Sharp/Square join regions are part of the kept offset)

## Context
T-0003 gave `offsetOpenContour` an exact "raw offset + prune" algorithm (ADR-0006 §5) but
returned only the longest surviving chain, so geometry was silently dropped whenever parts of the
path came within 2d of each other, and with Sharp/Square joins a chain could cross its own start
(T-0003 round-3 residual). It also tested every pair of raw elements and every piece against every
input span, which is quadratic. T-0004 owns the signature and fixes all three.

## Decision
1. **Signature.** `offsetOpenContour` returns `std::vector<Contour>`: every chain, ordered by the
   input-path position of its first point (span index + fraction; a closed chain is rotated to
   start at its earliest piece). Each chain runs in the input direction. No chains for empty,
   degenerate or non-finite input; one chain (the input) for a zero distance.
2. **No self-intersecting chain.** Every crossing or touching point of the raw chain is already a
   split point, so a stitched component can only meet itself at piece boundaries. Each component
   is walked once; when it returns (within `tol.distance`) to a boundary point it already passed,
   the part before that point is emitted as an open chain, the part from the point back to itself
   as a **closed** chain (`closed = true`), and the walk restarts there. A loop is therefore
   reported as a closed contour even for an open input; different chains may touch or cross.
3. **Stitching follows end points, not raw order.** A kept piece continues with its raw successor
   when that starts where it ends, otherwise with whichever unused kept piece does (nearest after
   it in raw order). The boundary can turn onto an earlier or later part of the raw chain — e.g.
   round a mitre that pokes out over a neighbouring offset — and joining only consecutive pieces
   broke such offsets into spurious fragments. Components start from pieces nothing leads into;
   leftovers are cycles.
4. **Sharp/Square region semantics.** A Sharp or Square join adds a region beyond the round
   neighbourhood (vertex → offset end → extension → (chamfer) → extension → offset start →
   vertex). Raw pieces strictly inside such a region (more than 1e-9·max(1,d) from its edges,
   tested with an exact ray-crossing count over its line and arc edges) are pruned like pieces
   closer than d. Without this, a neighbour's offset running past a mitre survived as a stub
   beyond the trim point, because it is ≥ d from the input. A chamfer shorter than the tolerance
   (nearly tangent corner) is replaced by meeting the extended neighbours, so no gap is left.
5. **Overlaps.** Raw elements on one carrier (e.g. both mitre extensions over a slot narrower
   than 2d) are split at each other's end points; of two coinciding kept pieces (> 2·tol long),
   a same-direction copy is dropped and an opposite-direction pair (a zero-width sliver inside
   the shape) is dropped entirely.
6. **Pruning margin.** A piece is kept when its samples are ≥ d − ε with
   ε = min(1e-9·max(1,d), 0.1·tol²/d). ε only absorbs rounding; staying below tol²/(8d) — the depth
   of the shallowest connector or overshoot longer than the tolerance at a nearly straight inside
   corner — guarantees no such sliver survives as a cusp. (T-0003 used 1e-7·max(1,d).)
7. **Short pieces.** Pieces ≤ `tol.distance` are dropped on output, except an interior one whose
   omission would leave its neighbours more than the tolerance apart (a run of tiny pieces round
   a sub-tolerance bump); chains stay continuous within the tolerance, and a valid stretch of
   offset no longer than the tolerance may be missing. An arc whose offset radius is tiny but
   positive is kept (collapse is decided at rounding level, 1e-12·max(1, r, d)), so the raw chain
   still passes through its centre instead of a connector cutting across at more than d. Consecutive pieces of the same raw element are re-merged even across
   a dropped sub-tolerance sliver.
8. **Cost.** Candidate pairs for raw/raw intersections, raw/boundary splits, the distance test
   (input spans) and the corner-region test come from uniform grids (`BoxGrid`, about one cell per
   box, ≤ 1024 cells per axis); coincident-point lookups use a hashed point index. A 2000-span
   polyline (dense wavy spiral, many splits) offsets in ~0.05–0.08 s in Release on the dev
   container, 8000 spans in ~0.25–0.33 s.

## Consequences
- Callers iterate chains; a closed chain from an open input is a loop the offset makes on its own.
- Coverage (T-0004 AC2) holds for every point of each span's requested-side offset (and Round
  corner arcs) that is ≥ d from the whole input, except inside Sharp/Square corner regions, which
  by (4) are interior to the offset shape, and except sub-tolerance stretches (7).
- Side membership is local: a piece belongs to the side whose raw offset produced it. Where the
  turns across several consumed short spans add up to more than half a turn, a round corner arc
  produced on one side can wrap round to the other side's region; it is still reported on the
  side that produced it (it is that side's outside-corner join).
- The 2d-proximity split no longer loses geometry, but chains may now be numerous for dense,
  self-approaching input; ordering by start position keeps output deterministic.

## Amendments
- **A1 (2026-10-25, T-0006): open-offset quality for toolpaths.** Status unchanged.
  - *Cost (§8).* Profiling the 2000-step 0.5 mm random walk at d = 2 (≈1.2 s per offset in
    Release before) put ~85 % of the time in pruning (each rejected sample scanned every input span
    within d: ~200 per sample, ~440 000 pieces per offset) and ~10 % in splitting at whole vertex
    circles. Now (a) the distance test stops at the first input span closer than d and tries the
    span that rejected the previous sample first (same answer as the full scan); (b) raw elements
    whose convex hull points (line ends; arc ends and tangent apexes per ≤ 90° piece) all lie
    within d of one input *line* are discarded before splitting (distance to a segment is convex);
    (c) a vertex contributes only the arc of its circle between the neighbours' offsets on the
    outer side of the turn (the rest lies inside the neighbourhood of one of the two spans), a
    reversal or a discontinuity of the flattened input (a dropped sub-tolerance span) a full
    circle, the path ends their full circles. Release, dev container, CPU time per offset (all
    joins and sides): walk d = 1 ≤ 0.02 s, d = 2 ≤ 0.04 s; 8000-span hypotrochoid d = 2 ≤ 0.2 s;
    8000-span text-like loops d = 2 ≤ 0.06 s.
  - *Arc corners (ADR-0006 §5, §4 here).* Mitre and chamfer extension of an arc may use the gap
    outside its current (possibly already extended) sweep; previously the gap was split in half
    between "past the end" and "before the start", so the second corner of an arc whose first
    corner had already consumed part of the gap fell back (keyhole d = 0.25 Left Sharp: true
    mitres at 0.304 from the vertices, previously a chamfer). The meeting point chosen is the one
    reached first (least total extension). **Cap (director, 2026-10-25):** an arc extension may
    sweep at most 90° and the arc's new end point must stay within mitreLimit·d of the corner
    vertex (for Square too, besides its 1.5·d chamfer reach); beyond either, Sharp falls back to
    Square and Square to Round. Without it, an offset arc whose radius is ~2 % of d was dragged
    ~2 rad round its circle to reach a chamfer line. Remaining fallbacks are listed in `offset.hpp`.
  - *Junctions (§7 replaced).* Chains are exactly continuous: consecutive spans (and the closing
    junction of a closed chain) share their end point bit-identically wherever a line is involved.
    Between two arcs (ADR-0005 computes arc end points from centre and angles) the second arc's
    centre is nudged by a few ulps until the points are equal; where that cannot succeed (centre
    coordinates much larger than the point's) they agree within 1e-12·max(1, |x|, |y|, r) mm —
    at most 6.3e-13 mm over 540 000 fuzzed offsets with coordinates up to ~100 mm (accepted,
    director 2026-10-25). Each junction is first closed geometrically: both spans
    trimmed or extended along their carriers to where they meet (extensions must stay ≥ d from the
    input and, for Round, ≤ d, both to min(5e-7·max(1,d), 0.1·tol²/d)); else, for a sub-tolerance
    step sideways, the last tolerance of a neighbour is bent outwards onto the other's end; else
    the sub-tolerance pieces dropped there are put back (they are the true offset), else a straight
    bridge (< tol) closes the gap. Such a bridge may come closer than d to the input by its
    sagitta against the round neighbourhood boundary, at most tol²/(8d): 1.25e-7 mm at d = 1,
    2.5e-6 mm at d = 0.05 with tol = 0.001 (accepted, director 2026-10-25; the old code missed
    d by similar amounts at such small d through other paths). Snapped extensions and bends may
    come closer by at most min(5e-7·max(1,d), 0.1·tol²/d). A sub-tolerance piece is therefore only
    dropped where its neighbours can be joined without it, and a chain may contain a span shorter
    than the tolerance where the offset really has a feature that small. A closed chain made of
    two lines between the same two points (a zero-width sliver) is dropped like the opposite pairs
    of §5. The closing junction of a closed chain is snapped first so that no span can be trimmed
    past its own other end.
  - *Tests.* The T-0004 chain check now requires exact continuity (1e-12) and allows a span no
    longer than the tolerance wherever it is needed for that continuity (positive length, never
    at an open chain's ends, exact junctions, no backward step); it passes with
    `CARVETTO_OFFSET_TRIALS=96`. The T-0004 2000-span timing test measures CPU time (`std::clock`),
    its absolute bound Release-only, plus a CPU-time complexity ratio (1000 vs 4000 spans).
