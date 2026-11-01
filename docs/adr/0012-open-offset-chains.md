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
- **A1 (2026-10-25, T-0006; residuals 2026-10-26, T-0017): open-offset quality for toolpaths.** Status unchanged.
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
    circle, the path ends their full circles. Release, dev container (2 cores, idle), CPU time per
    offset, worst over all joins and sides, re-measured for T-0017: 2000-step walk d = 1 ≤ 0.02 s,
    d = 2 ≤ 0.04 s (four seeds); 8000-span hypotrochoid d = 2 0.23 s; 8000-span text-like loops
    d = 2 0.10 s (round 3: 0.04 / 0.21 / 0.09 s on the same machine; the AC1 bounds are 0.5 s and
    1 s). Caveat (the cost of T-0016): the cost stays close to linear in the span count, but the
    constant is not uniform. On a dense zig-zag with d many times its pitch, Sharp and Square run
    10–30× slower than Round, because every corner's mitre or chamfer reaches out ~d and overlaps
    hundreds of its neighbours' on the same line, which multiplies the pieces to split and prune.
    Release, CPU time, T-0017 build (round 3 in brackets): pitch 0.02, amplitude 1, d = 7, 1000
    spans: Sharp 20.7 s (21.8), Square 20.8 s (22.0), Round 0.73 s (0.76); pitch 0.05, amplitude
    0.2, d = 5, 4000 spans: Sharp 8.7 s (8.7), Square 8.7 s (8.5), Round 0.75 s (0.67). Hatch- or
    fill-like vectors offset at a large d with Sharp/Square (T-0120/T-0121) will feel this; it is
    T-0016's to fix. T-0017 fix round 2 (more thorough repair checks), measured in one process
    against the round-1 build on a shared, heavily loaded 2-core container, so only the ratios
    count: walk d = 1 / 2 worst 0.024 / 0.047 s (round 1: 0.020 / 0.048), hypotrochoid 0.22–0.35 s
    (0.21–0.31), text loops 0.08–0.12 s (0.08–0.13), zig-zag pitch 0.02 Round 1.15 s (1.16),
    Sharp/Square 37.2 s (35.0), zig-zag pitch 0.05 Round 1.0 s (1.05), Sharp/Square 15.3 s
    (13.3); the verifier's fuzz families 1.03–1.16× round 1.
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
    centre is moved by the residual (at most 1e-10·max(1, |x|, |y|, r), T-0017 round 2) and then
    by a few ulps of centre and radius until the points are equal; where that cannot succeed
    (centre coordinates much larger than the point's) they agree within 1e-12·max(1, |x|, |y|, r)
    mm — at most 6.3e-13 mm over 540 000 fuzzed offsets with coordinates up to ~100 mm (accepted,
    director 2026-10-25). Such a residual's direction is rounding noise (it differed between
    glibc's and MSVC's sin/cos for 0xe400137f, radius 39785 at 8e3 mm, 3.7e-12 mm behind on
    Windows), so where the unmoved arc would start behind the previous end along its direction
    of travel, the nearest searched candidate (within the ulp search, ≤ 1e-12 of the scale) that
    starts level with or ahead of it is taken; an arc that already starts level or ahead is left
    as it was. A closed chain of arcs only has no line to start its walk from, so its closing
    junction moves the first arc's centre and reopens its first junction by a few ulps; where
    that leaves the second arc starting behind, the loop is closed instead by turning the first
    arc's start angle (its end point bit-identical), exactly or, failing that, level with or
    ahead within the same bound (fam5 0xa5000ba6, a 4-arc loop at 1e5 mm, 7.2e-12 mm behind). Two arcs whose circles only nearly touch (Round arcs about vertices
    2d + g apart meeting at a cusp) were snapped to a point on neither circle and kept a gap of
    about g, 1.1e-10…8.3e-10 mm for g ≈ 8e-11…9e-10 (T-0017 round 3, B-1, as on main); such a
    snap is now refused when the residual exceeds the 1e-10 limit, and the junction is bridged by
    a checked straight line of about g instead (exact again for every g). That is for continuous input: where the input's own spans do not meet,
    an arc-to-arc junction can keep a gap of about that input gap (8.5e-10 mm seen for a 9.4e-10
    mm input gap). A Round join at such a gap was one arc about the next span's start through the
    previous offset's end, so up to the gap beyond d (2.4e-5 mm seen for a 1e-4 mm gap) and, where
    the gap runs along the corner, up to ~1e-8 mm inside d of the previous span's end, enough for
    the pruning to cut it and split the chain (T-0017 fix round 2, 0xe30005b6); it is now an arc
    of radius d about each end of the gap, the outer stretch of each kept by the usual splitting
    and pruning. Arc-arc crossings (splitting and junction snapping) are computed without
    cancellation (s = (r2 − (D − r1))(r2 + (D − r1))/2D from the first circle's far side,
    h² = s(2r1 − s)): the textbook a = (D² + r1² − r2²)/2D, h² = r1² − a² lost ~1e-4 of h for a
    U-turn arc of radius d(1 + 1e-6) entered from an arc and left 1e-7..1e-5 mm junction gaps
    there, and 7.2e-12 at a tangent two-circle cusp (fix round 2, B1; repro_hp (a), 0xe300070e). Each junction is first closed geometrically: both spans
    trimmed or extended along their carriers to where they meet (extensions must stay ≥ d from the
    input and, for Round, ≤ d, both to min(5e-7·max(1,d), 0.1·tol²/d), checked every tol/8 since
    T-0017; among equally near meeting points one reached by trimming wins); else, for a sub-tolerance
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
  - *Overlapping joins on one carrier (verification round 2).* On steep zig-zags the Square
    chamfers (also Sharp corners beyond the mitre limit) of neighbouring peaks lie on one line
    and may overlap by less than the tolerance (6–9e-4 mm seen). Both copies of the overlap
    survived, because deduplication of coincident pieces compared end points to the tolerance and
    so skipped pieces shorter than 2·tol, and because each element computed its own nearly
    tangent boundary crossings (bottom of a vertex circle on the chamfer line), cutting the two
    copies ~1e-7 mm apart; the chain then ran forward, stepped back over the overlap and ran on.
    Now the kept pieces of every group of raw elements on one line or circle (overlapping pairs
    found during splitting, joined transitively) are merged by a sweep along a common coordinate
    (distance along the line, angle round the circle): all piece ends of the group that agree to
    rounding (1e-9·max(1, d, |x|, |y|)) become one node, and each stretch between consecutive
    nodes is kept once, on the lowest-numbered element, if more copies cover it one way than the
    other, else dropped (opposite copies enclose nothing, as in §5). (T-0017: kept only if all
    copies covering it run one way, on the outermost copy; two copies one way and one the other,
    where the input overlaps itself on a circle exactly 2d from another, still enclose nothing,
    0xe20006d8.) This replaces the pairwise
    end-point matching (to the tolerance, and only for pieces longer than 2·tol), costs
    O(P log P) per group and keeps every piece end as a node, so stitching can still turn there.
    (Cutting every copy at its partners' ends instead was correct but ~18× more pieces on dense
    zig-zags at large d, where every inside-corner connector lies on one line.) Stitching also
    continues a piece with the start nearest its end (to the same rounding) and only among
    equally near starts by raw order, so a start a sub-tolerance distance behind the end is never
    taken over one exactly at it. (T-0017 fix round 2: stitching is one global matching instead,
    every end against every start within tol, links taken closest first, ties among rounding-equal
    gaps by raw order, each end and each start used once; then chains are walked from pieces no
    link reaches, and the remaining cycles closed. The greedy piece-by-piece walk took a near
    start that another chain needed and split chains spuriously: 0xe2000124, 0xe20010e4. A link
    that would close a run no longer than 4·tol into a loop is skipped, since such a loop is
    dropped on output anyway: taken first for its small gap, a loop of two sub-tolerance pieces
    took the starts the chain through them needed, 0xe40003fb.) The sideways step repair (bending
    the last tolerance of a neighbour) is not used at a needle tip (tangents within ~8° of
    opposite): there the gap runs across the tip and is bridged instead (0xe40016aa).
  - *Nearly collinear carriers (verification round 3; corrected in T-0017).* The merge above grouped
    only carriers that `sameCarrier` matches (~1e-9 of the length sideways). With 1e-9..1e-5 mm of
    vertex noise (normal for DXF or computed geometry), or exact input far from the origin, the
    copies' carriers differ by more than that but cross at ill-conditioned angles, and chains again
    retraced, stepped back or repeated stretches of other chains (one input: 5 chains for 1). So
    kept pieces of nearly the same carrier are merged too, and the result is kept on the outermost
    copy. Round 3's form of this was wrong in three ways, found by its verifier (R4, R5, R6) and
    fixed in T-0017:
    1. *No real angle limit.* "Nearly parallel" was |u×v|·max length ≤ tol/2, which for elements
       1.5–2.5e-3 mm long lets 11–20° through, and nothing stopped transitive chaining: the six
       gently turning lines of R4 all joined one group and were projected onto the first one's
       axis, so a 1.57e-3 mm stretch of one element was emitted where that element had never been
       tested for distance (a second chain, 1.86e-4 mm inside d). Now two pieces are grouped only
       if their elements are at most 1e-3 rad apart **and** drift by at most tol/2 over their
       length, the pieces share a stretch of positive length (an end or the middle of one lies
       strictly inside the other: pieces that merely continue each other end to end are not
       copies), each piece's ends lie within tol/2 of the other's carrier, and each element is also
       within 2·10⁻³ rad of the other group's first element (no drift by chaining). Each merged
       stretch is mapped back onto its owner by the exact inverse of the projection (the owner's
       point whose coordinate is c: along a line, or on the ray from the reference centre), never
       by projecting the reference point onto the owner, so it is part of the owner's own kept piece
       up to the merging of nearby ends (1e-9·scale).
    2. *Outermost at the midpoint only.* The copy outermost at a stretch's midpoint can be the inner
       one at its ends when the carriers cross inside it (R5: 2.4e-6 mm inside d). Now the owner is
       the copy outermost at the stretch's start (ties to rounding: the previous stretch's owner,
       then the one outermost at the far end), and wherever another copy overtakes it (bisection on
       the difference) the stretch is cut and handed over, so the owner is the outermost copy along
       the whole stretch.
    3. *Opposite copies.* Copies running opposite ways were merged within tol/2 too and, as many
       running each way, dropped. Their neighbours are then joined sideways across the sliver's
       width, which for a real thin feature is not a rounding repair: R6's two strands 0.8–1.9e-4
       mm apart form a genuine needle tip (main keeps it), and dropping them left a sliver loop and
       a bridge 9.5e-5 mm beyond d (Round), 4 chains for main's 3. Opposite copies are now merged
       only within the snapping allowance min(5e-7·max(1,d), 0.1·tol²/d), where the sideways join
       is within the documented bound (co-circular noise of ~1e-8 mm, as in the co-circular arcs of
       0x7300029, is still cleaned up); wider opposite pairs stay, as on main.
    Further, (d) junction snapping that closes a rounding gap by moving a line end moves the end
    that goes outwards (this was the ~1e-8 shortfall at large d). Round 1 of T-0017 took any gap
    ≤ 1e-9·max(1,|x|,|y|) as rounding and moved the end unchecked; at 1e3–1e4 mm that is 1e-6–1e-5
    mm and could tilt the line inwards (repro_hp (b), (c)). Since fix round 2 the moved line is always
    checked like an extension (≥ d − allowance, for Round ≤ d + allowance, along the whole line,
    outward end preferred), still for gaps up to 1e-9·max(1,|x|,|y|) (two nearly collinear chamfer
    copies 4e-14 mm apart would otherwise be bridged by a zero-length span, fam2 0xe2000055); if
    neither end passes, the junction is treated as a real gap. Only extensions within 16 ulps of
    the coordinates (16·2.2e-16·max(1,|x|,|y|)) count as rounding and go unchecked; (e) a safety net after snapping removes any span no
    longer than the tolerance that steps back (chord against both neighbours, or a retrace) where
    three or more carriers nearly meet, joining its neighbours by snapJunction or by a forward
    straight bridge (optionally taking over a tolerance of one neighbour, so the bridge may be up to
    2·tol long) that passes the same distance checks. Round 3 skipped chains of exactly three
    spans (its loop required more), so [arc, 6.5e-8 mm backward line, arc] survived where two
    nearly co-circular arcs met (0x7300029); T-0017 runs it on every chain of three or more spans,
    and two arcs of the same direction on nearly one circle (centres and radii within tol/2 in
    all) on either side of the removed span become one arc on the circle of either, where every
    point of it passes the distance checks (sampled every tol/8) and the moved end's other junction
    snaps again. The safety net's cuts start from the exact length needed to undo the step back on
    either side, then tol, tol/2 … tol/10 (a fixed tol … tol/8 list missed a back-step that only a
    shorter or a precisely sized cut clears: 0xe40010cb, fix round 2). (f, T-0017) A span that sets
    off straight back along its neighbour and lies wholly within 2e-9·max(1, d) mm of it (or 64
    ulps of the coordinates where larger, beyond ~1e6 mm; arcs only if also on one circle to that
    bound) is removed and the neighbour cut back to match: a needle of rounding width. Round 1
    scaled the bound with the coordinates (2e-9·max(1, d, |x|, |y|)), so the same needle was kept
    near the origin and removed at 1e4 mm, losing real path there (0xd4001274, 0xe400062f,
    0xe4001140, 0xd4001d85, 0xd4000e09); per the director's decision (fix round 2) the bound no
    longer depends on translation and a wider needle is kept wherever it lies (0x840117a: the 2.9e-3
    mm needle 3.6e-6 mm wide at ~1e4 mm is now kept, as at the origin). (g, T-0017) A closed chain narrower than the
    tolerance is dropped like the two-line sliver above: two spans that never part by more than
    tol/2 (0x82004c8: a lens of two arcs on nearly one circle running opposite ways, 0.018 mm long,
    ~2.4e-5 mm wide; main identical), or any loop whose mean width 2·area/perimeter is below tol/8
    and whose every point lies within tol/8 of another of its spans (gen3 0x9300698: a 1.45 mm
    lens ~1e-5 mm wide between opposite arcs 5e-6 mm apart, which crossed itself where 1e-3 mm
    bends closed its near-tangent ends; 0x9100821: 0.017 mm left between two tangent circles).
    (The two-span drop of lenses up to tol/2 wide is tolerance-level and accepted by the director.)
    (j, T-0017 round 3) No snapped extension may run along a kept piece of another chain (or of
    another part of the same component) on the same carrier: closing a three-span loop by
    extending a line over the start of the next chain duplicated 1.5e-3 mm of it and poked the
    corner 1.1e-3 mm past the offset (fam5 0xf500163e; also 0xf5001afe, 0xf50007aa); such a
    junction falls to the other repairs (here the checked bridge). An open chain no longer
    starts or ends with a span no longer than the tolerance (§7): a cut or a node could leave
    one there (0xf50009c9, 0xf5000dd2, 0xf5001b5b); it is dropped, like leading and trailing
    tiny pieces. Pieces of one element re-merged across a sub-tolerance sliver the chain steps
    over (one span again) may dip inside d by up to ~tol²/(4d) across it (0xf50011fd: 1.72e-7
    mm, as on main). Known: a needle narrower than the tolerance whose two strands pass within
    the tolerance of each other's nodes is cut into several chains at those nodes by the node
    walk (0xf5000fc3: 7 chains, main 6 with back-steps and overlaps instead). After a repair of
    the safety net, retrace removal or co-circular merge, the junctions of a closed chain are
    made exact walking from a line, as snapChain does: walked from span 0, the closing call moved
    an arc's centre and reopened the first junction by 3.5e-14 mm (fam5 0xf51007d9, found while
    fixing B-1).
    These are the only valid offset points the branch drops on purpose besides the corner changes
    of AC2 and exact zero-width slivers; the main-vs-branch comparison below classifies them.
    (h, T-0017) Every extension, bend and bridge is checked at points at most tol/8 apart (a line
    tilted by moving one end, at most 1e-9·max(1,|x|,|y|), at up to 24 points crowded towards the
    moved end, where the tilt is largest and a kept line comes nearest the limits: sampling it every
    tol/8 made the fuzz families 4× slower, and 4 even points let a Sharp line at 1e4 mm end 3.8e-6
    mm inside d, hp sweep), and between the samples every local extreme of the distance
    to the input is located by golden-section search (40 steps; skipped where three neighbouring
    samples agree to 1e-12 of the scale, a flat stretch such as an arc about a vertex, whose search
    made Round offsets of dense polylines 3–6× slower; the distance is queried only up to d + 2·tol
    away), because the distance along a span is the minimum of convex
    functions and peaks between samples where the nearest input element changes (round 3 checked
    an extension only at its far end and middle: a Round extension ended 1.3e-6 mm beyond d next to
    a wiggly input, gen3 0x9300156; sampling alone still let 6.7e-7 and 4.2e-7 mm through,
    0xe30009ae and 0xd4001762, fix round 2). Only a move no longer than 16 ulps of the coordinates
    (16·2.2e-16·max(1,|x|,|y|)) skips the check (round 3 skipped extensions up to the
    1e-9·max(1,|x|,|y|) "rounding" gap, 9.5e-6 mm at 1e4 mm, and one at a steep angle to another
    part of the input ended 9.4e-7 mm inside d, gen3 0x9400bf7; round 1 of T-0017 still skipped
    the line-end moves of (d) at that size),
    and where several meeting points of two carriers are equally near the junction, one reached by
    trimming both is preferred (two Round arcs about vertices almost exactly 2d apart cross twice
    ~1.6e-5 mm apart; meeting at the crossing that needs extensions left them crossing at the
    other, gen3 0x9300047). The safety-net bridge tries cuts of tol, tol/2 … tol/8 (a chord across
    a small arc dips inside d by its sagitta) and, where the input itself has a gap, may stay as
    far beyond d as the Round join next to that gap already is (gen3 0x930046a: 1.4e-5 mm gap,
    3.2e-6 mm step back otherwise left in place). The last-resort straight bridge of snapChain
    is now checked too (same sampling; inside d only by the documented sagitta, and for Round not
    beyond d): at a Round needle tip whose rounded end was not among the kept pieces it ran 1.3e-5
    mm beyond d (gen3 0x93010bf). There the missing arc about the nearest input point is put in
    where it passes the checks, and otherwise the chain is cut into two continuous chains (main
    leaves a gap in one chain there). A cut never drops a piece: the chain's spans are only
    divided between the two parts (0xe30005b6, where a cut seemed to lose 0.019 mm, was the
    Round join at an input gap described under Junctions, pruned before stitching).
    (i, T-0017) Consecutive arcs on one circle to 1e-12·max(1, |x|, |y|, r) running the same way
    become one arc (a closed chain going all the way round, one full circle): a full circle left
    as three such arcs had a closing junction the shared checker read as a step back from the
    4e-10 mm between their radii (gen3 0x94010e5; main: one arc).
    Statement in `offset.hpp`, true as written: junctions are exact for continuous input, so no
    span starts behind the previous one's end, and a sub-tolerance span that would step back is
    removed unless no join passes the distance checks (not seen in the T-0017 runs below).
  - *Known limitation: input with sub-tolerance spans* (present before T-0006, carded as
    follow-up). Flattening leaves out spans no longer than tol.distance, so the distance and
    coverage guarantees hold for the flattened input only. Next to such a span an offset point
    may be short of d by up to ~tol (9e-4 seen), a Round arc about the neighbouring vertex may be
    up to ~tol over d (8e-4 seen), and up to ~1 mm of valid offset may be missing. How often
    depends on the inputs: ~1 in 1000 offsets in the general fuzz (where such spans are
    occasional), ~28 % with the verifier's generator that injects sub-tolerance jogs after ~30 %
    of the lines. Inputs without sub-tolerance spans are unaffected (worst
    shortfall 1.5e-6 at d ≈ 0.05, within the bridge bound above). Documented in `offset.hpp`.
  - *Tests.* The T-0004 chain check now requires exact continuity (1e-12) and allows a span no
    longer than the tolerance wherever it is needed for that continuity (positive length, never
    at an open chain's ends, exact junctions, no backward step); it passes with
    `CARVETTO_OFFSET_TRIALS=96`. Backward steps are detected by one shared checker
    (`core/tests/geometry/offset_chain_checks.hpp`) on every junction including the closing one:
    a start behind the previous end, a retrace (a near-reversal passes as a needle tip only if
    the two sides diverge at every probe up to the shorter span's length, so a collinear
    sub-tolerance step back fails), and a sub-tolerance span running against both neighbours.
    Regressions: the verifier's zig-zag and closed zig-zag repros and a 3456-offset steep
    zig-zag sweep (all joins and sides, open and closed, mitre limits 1–4.1, near the origin
    and ~120 mm away); round 3 adds R1, R2, R3 and a 1296-offset noise sweep (noise 1e-12…1e-4,
    all joins, both sides, open and closed, half ~640 mm from the origin) checked for backward
    steps, overlapping stretches within 1e-6 (`countOverlaps`, across chains too), gaps and
    distance ≥ d − allowance. The timing tests (T-0004 2000-span bound and 1000/4000-span ratio,
    T-0006 AC1 bounds and walk ratio) measure process CPU time through
    `core/tests/geometry/cpu_time.hpp`: `std::clock` on POSIX, `GetProcessTimes` (user + kernel)
    on Windows, where `std::clock` is wall-clock (T-0017); absolute bounds are Release-only.
    T-0017 adds regressions R4, R5, R6, 0x7300029, 0x82004c8, 0x840117a (shrunk to 116 lines) and
    the residuals found while fixing them (0x9300156, 0x9300047, 0x9300698, 0x930046a, 0x9400bf7
    shrunk to 10 lines), inputs inlined; each but 0x9300698 fails on the round-3 code. Results,
    Release, verifier's harness with new seeds: 20 000 offsets (legacy families 0x9100000 and
    0x9200000, round-3 families 0x9300000 and 0x9400000, 5000 each) compared with main; of 219
    flagged as differing for the worse by the harness, 215 are the intended changes (main points
    in Sharp/Square corner regions, AC2; sub-tolerance sliver loops; Round excess or arc-arc gap
    no larger than a gap in the noisy input itself) and 4 are exact co-circular ties as in round 3
    (3 zero-width slivers dropped on purpose, 1 cusp where two circles touch exactly and the
    checker sees a numerical self-touch, 0x9100e5b). Exact input: 0 backward steps (shared
    checker), 0 overlaps, 0 gaps > 1e-12; all 335 near-reversal junctions are genuine needle tips;
    noisy input: 0 short back-steps; gaps only where the input itself has a larger one; 2 offsets
    (of 10 000 round-3-family ones) where the shared checker reads an out-and-back needle between
    two opposite, nearly co-circular arcs ~1e-6 mm apart (wider than the opposite-copy merge
    allowance, much narrower than the tolerance) as a retrace, where main has 3 and 2 such steps
    for the same inputs (gen3 0x9300ba3, 0x9300c6a; known residual). Noise sweep (300 zig-zags per
    level, 1e-13…1e-4, at 0 and 1e4): clean. R4/R5/R6 as required.
