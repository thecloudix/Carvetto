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
   the shape) is dropped entirely. (Since A3 such a stretch is put back as an open chain of its own where it lies
   exactly d from the input and no chain covers it.)
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
    1 s). Dense hatch-like input (T-0016): on a dense zig-zag with d many times its pitch every
    corner's mitre or chamfer reaches out ~d and lies on one line with those of hundreds of
    neighbouring peaks (k copies; likewise every inside-corner connector). Each copy is split at
    all its partners' ends (~k² pieces), and every copy of the shared chamfer line was tested
    against the input and the ~k corner regions over it, then the junctions between the copies the
    carrier merge hands a stretch over between (end points equal but for the last bits) were each
    checked with two distance profiles: Sharp and Square ran 10–30× slower than Round (T-0017
    build: pitch 0.02, amplitude 1, d = 7, 1000 spans, Sharp 20.7 s, Square 20.8 s, Round
    0.73 s; pitch 0.05, amplitude 0.2, d = 5, 4000 spans, Sharp/Square 8.7 s, Round 0.75 s).
    Now (a) the pieces of a group of overlapping raw lines on one carrier (pairs from the
    splitting step, joined transitively; inside-corner connectors, always pruned, left out) are
    tested per stretch and direction: in element order until one is kept (all of them where all
    are pruned, as before), then the copy mergeOnCarriers would keep (its own rule over the copies
    not known to be pruned) plus any copy that overtakes it before the stretch ends, and all the
    others only if that copy is pruned. A direction therefore has a kept copy over a stretch
    exactly when it had one before and merge keeps the same copy up to rounding-level ties, so the
    output is unchanged except which of several coincident copies owns a stretch (coordinates
    move by rounding; collinear spans may be split at other points). Such rounding-level shifts
    can flip a tolerance-level repair downstream: the T-0016 verifier saw outputs move by up to
    3.9e-4 mm (below tol) with identical checker metrics. (b) snapJunction moves the
    end of one of two lines on the same carrier whose ends differ by rounding only (≤ 16 ulp of
    the coordinates) without profiling the move, as it already did for rounding-level extensions.
    Release, CPU time per offset, both sides, measured in one process against main (53ae278) on a
    shared, loaded 16-thread machine: pitch 0.02, amplitude 1, d = 7, 1000 spans: Sharp
    0.75–1.03 s (main 23.5–31.4 s), Square 0.99–1.01 s (main 28.9–30.8 s), Round 0.92–1.00 s
    (main 0.79–0.84 s); 2000 / 4000 spans: Sharp 2.2 / 4.7 s (2.1× per doubling), Round 3.4 /
    8.0 s; pitch 0.05, amplitude 0.2, d = 5, 4000 spans: Sharp 1.10–1.12 s (main 11.7–12.6 s),
    Square 1.05–1.08 s (main 11.8–12.5 s), Round 1.15–1.19 s (main 1.00–1.02 s); 8000 spans:
    Sharp 2.12–2.13 s (1.9× per doubling), Round 1.96–2.02 s. Round costs the same algorithm as before
    (the new step takes ~3 ms there), but on large inputs Round, and other inputs with nothing to
    prune, run 13–15 % slower than main even with every new path disabled, which points to code
    generation in buildRaw / mergeOnCarriers / offsetOpenContour (T-0016 verifier, A/A-controlled;
    follow-up T-0170). Small inputs are not slower. Not addressed (separate cards): input whose whole length lies
    within d of itself, e.g. a walk of 2000/4000 steps of 1.5 µm offset by 1, costs super-linearly
    for every join (Round 4 s / 40 s, Sharp 12 s / 120 s, main and T-0016 alike), because each
    piece beyond d scans every input span and every vertex circle splits every raw element.
    T-0017 fix round 2 (more thorough repair checks), measured in one process
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
    ahead within the same bound (fam5 0xa5000ba6, a 4-arc loop at 1e5 mm, 7.2e-12 mm behind).
    One angle ulp moves the start by only r·ulp(angle) (about 1e-15…1e-14 mm at r = 21) against
    1.8e-12 mm coordinate ulps at 1e4 mm, so the turn is solved for (a's end may lie ρ off b's
    circle at a corner: the start moves round by ρ (n·ta)/(t·ta)) and angles a quarter of a
    coordinate ulp apart are searched around it (T-0017 round-3 re-verification: single-ulp
    steps left ~7% of bit-continuous arc loops at 1e4…2e5 mm stepping back, 2.0e-12 mm in the
    verifier's 3-arc repro). A turn cannot help where a arrives almost square to b's circle
    (along its radius, t·ta ≈ 0) and its end lies just inside or outside the circle: moving the
    start round the circle then moves it across a's travel, not along it. This happens on
    continuous input too (verifier repro, a 6-arc loop at (3e4, 7e4): 1.7e-11 mm behind). There
    the walk is repeated from each other arc and the first result without a backward junction
    is kept. If no arc gave one, the walk from the first arc would stay, with a step back of a
    few coordinate ulps at one junction; that was never seen. Verifier perturbation driver, about
    1.75 M offsets (arcs-only loops exact and noisy, mixed closed/open, large radii, at up to
    2e5 mm), and the round-3 re-verifier's ~4 M offsets including loops of 12–72 tiny arcs:
    0 backward junctions (45b2205: 26 000+ in the first). Tests reach each part on its own: the
    turn through an internal hook (`detail::turnArcOntoJunction`, core/src/geometry/
    offset_junction.hpp), the retry through the 6-arc repro.
    Two arcs whose circles only nearly touch (Round arcs about vertices 2d + g apart meeting at a
    cusp) were snapped to a point on neither circle and kept a gap of about g,
    1.1e-10…8.3e-10 mm for g ≈ 8e-11…9e-10 (T-0017 round 3, B-1, as on main); such a snap is now
    refused when the residual exceeds the 1e-10 limit, and the junction is bridged by a checked
    straight line of about g instead (exact again for every g). That is for continuous input.
    Where the input's own spans do not meet, an arc-to-arc junction can stay open, always ahead
    of the previous end, by up to the larger of about that input gap (8.5e-10 mm seen for a
    9.4e-10 mm input gap; up to 2.4× it where the gap is larger than rounding) and the 1e-10
    snapping limit of the scale, where two circles nearly touch (6.0e-11 mm at ~1 mm for circles
    overlapping by 5.6e-11 mm, input gaps 1e-16: as on 45b2205). Noisy arcs-only loops: 56 arc
    gaps above 1e-12 of the scale in 168 000 offsets (45b2205 42, 0cd9abc 75).
    A Round join at an input gap was one arc about the next span's start through the
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
    mm needle 3.6e-6 mm wide at ~1e4 mm is now kept, as at the origin). (g, T-0017; changed by A3: the loop is
    reduced to one open strand unless its whole extent is within the tolerance) A closed chain narrower than the
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
- **A3 (2026-11-05, T-0128): narrow loops and needles keep their path; no crossing at reversal cusps.**
  Status unchanged. (A2 is taken by T-0009, in progress.)
  - *Context.* T-0017 removed three things a tool centre still follows. (1) A closed chain too thin to
    enclose anything (A1 (g): two spans never more than tol/2 apart, or mean width below tol/8 with
    every point within tol/8 of another span) was dropped, including an out-and-back needle whose two
    ends the node walk (step 6) joins because they lie within the tolerance: fam5 0xc5000544, a needle
    3.6 mm long and 5e-6 mm wide (main before T-0017 kept it). (2) Opposite copies merged within the
    snapping allowance, and (3) any stretch copies cover both ways (§5), were dropped, so a slot exactly
    one tool width wide, give or take coordinate noise, lost its path. In the T-0128 channel sweep (a
    channel 3.6 mm long and 2d + w wide as an open U, the U reversed, a U with a semicircular end and a
    closed rectangle; w in [0, 1e-5]; d = 0.1317, 1, 3.175; all joins; at the origin and far from it)
    11 136 of 12 420 inside offsets lost the channel: Round kept it only above the allowance (and only
    as an open U, where the Round-only "not beyond d" check happened to refuse the closing bridge);
    Sharp, Square and the rectangle lost it for every w up to 1e-5. Separately, at a reversal cusp
    between Round arcs about vertices a hair less than 2d apart, the lens between the arcs survives the
    pruning (inside d only by rounding) and a closed chain ran down one side of it and up the other,
    crossing itself (0xc500076c: lens 2.1e-4 x 1.9e-9 mm; the round-3 verifier's cusp sweep which = 1,
    21 of 732 offsets at ~1e4 mm; main before T-0017 left arc-arc junction gaps there instead).
  - *Decision.* (a) A thin closed chain whose whole extent is within the tolerance is still dropped;
    otherwise it is cut at its two points farthest apart (two farthest-point sweeps over eight samples
    per span) and the longer of the two strands is put back as an open chain (end spans no longer than
    the tolerance dropped). It passes within the loop's width of every point of it. Both strands are not
    returned: where they coincide (w = 0) that is a retrace. The strand is taken from the raw pieces the
    loop was stitched from, not from its snapped spans: far from the origin, closing the tip of a needle
    narrower than the 1e-9 x |coordinates| rounding gap moves a line end across the needle and tilts the
    whole strand by its width (Sharp, 1.2e4 mm, w = 3e-6: 3e-6 mm beyond d). Its pieces are therefore pieces
    of raw elements (offset spans, Round arcs and inverted arcs, each exactly d from its own input
    element, or Sharp/Square corner elements on their corner regions), so it lies at d or on a corner
    region by construction. (Round 1 also sampled each strand against d + 5e-7 x max(1, d) or a corner
    region, after an intermediate version built the strand from the snapped spans and put back pieces
    1.03e-6 mm beyond d, fam5 0x2a500001, 0x2a5000a6. With the strand cut from the raw pieces that test
    rejected no sample in 480 000 fuzz offsets of families 2-5 with 5 398 loop strands, and no input
    could be found that a mutant without it gets wrong; round 2 removed it.) A strand that would cross
    itself falls back to the loop's longest span (so did one of more than 512 spans until T-0172, see the
    follow-ups below). (b) mergeOnCarriers hands back one copy of every stretch it drops because
    copies run both ways: the first forward, then backward, copy that lies exactly d from the input there.
    `RawChain::exact` records, per raw element, the parameter range at exactly d: an offset span's own
    extent before corners extended or trimmed it, or a whole Round join arc (radius d to the snapping
    allowance); chamfers, connectors and inverted arcs have none, and only those ranges are put back:
    opposite copies of mitre extensions or chamfers over a slot narrower than 2d lie farther out, inside
    the corner regions (fam5 0x285001c5: put back by a first version, 8.4e-6 mm beyond d, the exact
    checker's excess). Pieces of one element that continue each other are joined, then pieces whose ends
    coincide to rounding. (c) Strands of both kinds are split where consecutive pieces lie farther apart
    than rounding (1e-9 x the coordinates) or the strand would turn back, their junctions are made exact
    (exactJunctions), and they are split again where that fails (arcs at ~1e4 mm, 7.7e-7 mm apart: fam5
    0x2850005d). (d) Step 7 puts them back as open chains wherever no emitted chain, nor a strand already
    put back, passes within tol/2: each strand is sampled every tol/4, each uncovered run is widened by one
    sample into its covered neighbours (so it reaches at most tol/2 into them) and kept when longer than
    the tolerance. A run widened at neither end is a whole strand none of which is covered (an isolated
    piece); it is kept only when longer than 10 tol (`kStrayMinimum`; owner decision 2026-11-05: fam0
    0x71280859 gained an isolated 0.00513 mm chain, noise to the user, while the needles and slots this
    card is about are millimetres long). So a thin loop of extent between tol and 10 tol touching no other
    chain is not returned (round 1 also dropped those of extent up to 2 tol, against its own header). The
    distance g to the chains is 1-Lipschitz along a strand, so a sample vouches for the next |g - tol/2|
    (all covered or all uncovered), which is skipped; g is found exactly up to a query reach that starts at
    16.5 tol and doubles, up to 1024.5 tol, while nothing lies within it. The chains and the strands put back
    are both in grids (the strands in one laid out over all strays, filled as they are put back: a
    `BoxGrid` that can grow; since T-0172 a `BoxIndex`, a grid plus a `BoxTree`, for each, see the
    follow-ups below), and the new
    chains are merged into the output in one pass. The new chains
    go by where their component (or piece) starts along the input; with nothing put back the output is
    unchanged. They may
    touch the chains next to them ("different chains may touch or cross"). (e) At a junction turning back
    by more than a right angle, where the two spans cross again within the tolerance of it, both are cut
    back to the crossing farthest from the junction (removeCuspLoops, after the other chain repairs). The
    crossing must stand off the junction by more than 100 rounding units over the sine of the crossing
    angle (two circles touching to rounding cross twice ~1e-8 mm apart at places that are noise: fam5
    0x28500192, where cutting there made a self-crossing), and the cut spans must meet only at their new
    junction. The remaining geometry is unchanged, so no distance bound moves. A crossing below that
    bound is not a self-crossing in the sense of the header (amended at the PR #61 MSVC failure): the
    two circles touch to rounding, the spans stay within rounding of each other up to it, and where it
    falls (up to 1.4e-7 mm from the junction) moves with the last bit of a radius or of sin/cos. MSVC's
    sin/cos put one at 1.1e-8 mm from the cusp of the T-0128 synthetic cusp test at g = 0, which glibc
    happens to put on the junction itself; the tests' self-crossing count applies the same bound to
    adjacent spans. (f, same PR) exactJunction kept an arc where it started whenever its start lay ahead
    of the previous end and no centre/radius search hit the end point exactly, leaving residuals up to
    its 1e-10 x scale snapping limit open (the closing junction of the cusp test's loop at g = 3.2e-11
    stayed 3.2e-11 mm open with sin/cos 1 ulp off glibc's); it now keeps the arc only within the
    documented 1e-12 x scale and otherwise takes the nearest searched candidate that does not start
    behind. Evidence: libm calls (sin, cos, atan2, hypot, acos, asin, tan) wrapped at link time to
    return results moved by a deterministic pseudo-random -k..k ulps per argument, k = 1..4: the T-0128
    tests pass for 50 seeds at each k; the cusp sweeps (which = 0..3, d = 0.13 / 1.03 / 3.175, 150
    perturbed sweeps of 732 offsets) show no self-crossing above the bound and no junction gap, the
    largest noise crossing at 19.4 of the 100 units (a real cusp lens, 0xc500076c at about 1e4 mm, measures ~6,300 units — about 63× the bound; the same lens beyond about 6e5 mm would fall under it in both the product and the test), while a
    count with the 1e-9 slack alone finds up to 16 noise crossings per sweep (the round-1 verifier's
    checker: branch 1-5, main 16-24, and main also opens junction gaps).
  - *Consequences.* A channel 2d + w wide keeps its path for every w >= 0 (sweep: every inside offset
    covers the channel's centre line within the tolerance, except the closed rectangles at d = 3.175,
    which are shorter than the tool). Outputs differ from T-0017 only where it dropped such a path or
    at a cusp lens; chain counts can grow by the strands put back (the T-0017 chain-split regressions
    count chains on the middle circle of a ring channel exactly 2d wide apart).
  - *Results.* Exact checker (the T-0016 verifier's cmp6: xk + vchk metrics, every join and side, branch
    against origin/main in one process; harness `verifier-harness` t0016v1 with the t0009v3 generators),
    fresh seeds, final code: legacy family 0 (4000 inputs, 24 000 offsets), family 4 (2500), S-hatch 23,
    jittered zig-zag 22 and tall-teeth 25 (1500 each), families 2, 3 and 5 as far as the 50-minute job
    limit reached (1330, 2213 and 2052 inputs); earlier rounds on the intermediate versions found the
    defects quoted above. No offset is worse than main; the differences are coverage gains (the checker's
    thin misses drop in 815 offsets and its tie misses in 495, none rise) and shortfalls below d that stay
    within the allowance (at most ~5e-9 mm, d ~ 1.6, against 7.6e-8). Cusp sweeps (round-3 verifier's
    cusp.inc, 732 offsets each, Round): which = 1 at d = 1.03 / 0.13 / 3.175: self-crossings 0 (main 21 /
    0 / 15), junction gaps 0; which = 2: 1 (main 22); which = 0 and 3 as main (1 and 0). The one left in
    which = 0 and 2 each is a rounding-noise crossing of two circles that touch to rounding (g = 0 and
    6.3e-11), identical to main. The T-0017 centre-shift sweep (cs, 7776 offsets): clean. Channel sweep
    (12 420 inside offsets): every one covers the channel's centre line except the 828 closed rectangles
    at d = 3.175, shorter than the tool (main lost 11 136); the 508 flagged by vchk's overlap check are
    Round U needles above the allowance, out-and-back chains T-0017 already returned, identical to main.
    Round 2 (owner minimum, near-linear recovery; same checkers, fresh seeds 0x7228xxxx): legacy family 0
    (2000 inputs), families 2, 3, 5 (800 each, family 5 twice), 4 (1000), S-hatch 23, zig-zags 21, 22, 25
    (600 each): no offset worse than main; fam0 0x71280859 is identical to main again (its isolated 0.00513
    mm chain is gone), while 0x722804d7 keeps an isolated 0.054 mm arc (above 10 tol; it clears two of the
    checker's thin misses main has). Channel sweep at d = 0.1317, 1, 3.175 (L 3.6) and d = 3.175 (L 20): a
    path for every w >= 0 (main lost 7110 / 6900 / 4950 / 6795), worse 0. Cusp sweeps as above (which = 1:
    0 self-crossings at d = 1.03, 3.175, 0.13; main 21, 15, 0). The ADR's regression seeds above are tests
    (offset_narrow_loops_test.cpp), with cases that fail when the exact-range cut of (b), the stand-off
    test of (e), the cover test of (d) or the minimum of (d) is removed.
  - *Cost.* Release, CPU time, interleaved with main in one process on a shared 16-thread machine, min of
    3 runs. Without strays the work is main's plus a pass over the junctions (reversal cusps) and the
    exact ranges of (b): the T-0016 card case (zig-zag of 1000 spans, pitch 0.02, amplitude 1, d = 7) Round
    0.85 s (main 0.94), Sharp 0.68 s (0.68), Square 0.71 s (0.70); pitch 0.05, amplitude 0.2, d = 5, 4000
    spans: 0.75 / 0.84 / 0.81 s (main 0.81 / 0.82 / 0.79). Fuzz sets, summed per offset (1000 inputs each,
    6000 offsets): family 0 1.006x main, 2 1.011x, 3 0.995x, 4 1.007x, 5 1.012x. With strays the work grows
    linearly with their number and length: a boustrophedon hatch at pitch exactly 2d (a stray between every
    pair of lines), a comb whose slots are 2d (+ 1e-7) wide, closed rectangles 3.6 x 2d traced in one open
    path, and a serpentine of half-ring channels 2d wide, 300 / 1000 / 3000 of each, every join, cost at
    most 1.41x main (hatch, 3000 lines, Round: 0.031 s, main 0.022 s), e.g. hatch Round 0.0030 / 0.0134 /
    0.0312 s, comb Round 0.0068 / 0.0257 / 0.0717 s, rectangles Square 0.0108 / 0.0306 / 0.134 s.
    Round 1 compared each sample with every stray already put back, which made these quadratic (hatch at
    3000 lines 2.94 s, rectangles 25.3 s, serpentine 16.2 s) and sampled uncovered strands every 16 tol (or
    every tol on a loop's strand, 4.6 million samples for the Square hatch at 2d + 1e-7). The round-1
    verifier's per-family figures (1.06-1.15x main) came from the exact checker's driver, which times the
    branch first after the checker has run and main right after it; with the calls swapped, the same
    driver gives family 3 0.97x and family 5 0.83x, so that excess was cache warm-up, not the branch.
  - *Follow-ups (T-0172, 2026-11-06; findings of the T-0128 round-2 and round-3 verifiers).* Status unchanged.
    (a) *Long strays.* Step 7 returned a stray of more than 512 spans, or one that would cross itself, as one
    chain per span and dropped its spans no longer than the tolerance (Profile cut each span separately: a
    plunge per segment of a finely segmented slot wall), and a loop's strand of more than 512 spans fell back
    to the loop's longest span. The limit only kept the pairwise self-crossing test (~3 ms per 500 spans)
    affordable. That test is now one greedy pass (`selfCrossingCuts`: each span against the earlier spans of
    its part whose boxes meet its own, each box grown by `meetReach`, found through a `BoxTree`), and the
    limit is gone: a stray is one chain however many spans it has (a channel exactly 2d wide whose walls are
    514 / 2 000 unit segments: 1 chain, origin/main 513 / 1 999; the closed slot of 600 segments: 1,
    origin/main 602). One that would still cross itself (never seen) is cut greedily into consecutive parts
    that do not, each without sub-tolerance end spans (`nonCrossingRuns`). The growth makes the pass give the
    pairwise test's answer (`spansMeet`: carrierIntersections + inUnit) for every pair, near-tangent ones
    included: a meeting point lies within inUnit's slack (4 x 1e-9 of the length, for carrierParam's
    rounding) of each span's ends and within carrierIntersections' contact tolerances of each carrier, so
    an arc's box grows by a further 1e-9 mm (circles up to 1e-9 mm apart, outside or inside, touch; the arc
    points at the meeting angles are then at most that gap apart) plus 1e-7 x r, a line's by 1e-7 x its
    length (a line touches a circle up to 1.25e-10 x r beyond it, and the rounding of the discriminant's
    cancellation, at most ~10 eps |w|^2 with |w| <= len + r from the centre to the line's start, widens
    that by up to sqrt(10 eps) (len + r) < 5e-8 (len + r)), and both by 1e-12 x max(1, |x|, |y|, r) for
    the remaining rounding. (Round 1 grew the boxes by inUnit's slack only and missed near-tangent pairs
    between that and the contact tolerances: an arc 5e-10 mm from another, 1 in 300 000 random probes.)
    Its cost is O(n log n) to build the tree plus the pairs of spans whose grown boxes meet: linear on an
    offset's strays, whose spans meet only their neighbours' boxes, and output-bound where boxes overlap
    a lot (a strand going back and forth along one line, span lengths over seven decades: 17 box pairs per
    span at 1 000 spans, 36 at 16 000; 16 000 spans in 0.05 s).
    (b) *Clustered strays.* Step 7's two grids were uniform over the extent of all chains and strays, so one
    far feature (a U channel 1e5 mm from a hatch at pitch 2d) put the whole hatch into one cell and every
    sample scanned all of it. They are now two `BoxIndex`es (box_index.hpp, round 3; round 2 used one
    `BoxTree` for both, below). Round 1 used a grid that gave a crowded cell a grid of its own, recursively,
    and filed every
    box that overlapped a cell without covering it into the finer grids again: nested or overlapping boxes
    (a spiral slot's arcs, a back-and-forth strand) blew it up, super-linearly and at times exponentially in
    time and memory (round-1 verifier: a spiral slot of 400 semicircles 12.6 s and 2.5 GB, origin/main 0.47 s;
    4 000 spiral semicircle boxes 107 M entries; 120 nested collinear boxes over 8 GB). A `BoxTree` holds
    each box exactly once: in static k-d trees of the boxes as 4-D points (min x, min y, max x, max y),
    median splits along the four coordinates in turn, leaves of at most 8 boxes, each node keeping the
    bounds and the id range of its boxes; boxes added later go to new trees that are rebuilt together when
    they reach the size of the last (the logarithmic method: at most log2 n + 1 trees, O(n log^2 n) to build
    in all, O(n) memory). A query reports exactly the boxes that meet it (optionally only those with ids in a
    range), visiting O(n^(3/4)) nodes plus those of the boxes it reports whatever the boxes (any k-d tree;
    with an id range, also those of boxes outside it that meet the query):
    nested, concentric, collinear, all huge, all tiny or mixed (tested: every box held once, fewer than n / 2
    nodes, query work within 2-50 x (n log2 n + pairs reported) on 13 such families, ~1.5x what each
    measures, so that a tree split along one coordinate only fails three of them). But its weak case is
    boxes that overlap along one axis and differ only along the other: on a hatch at pitch 2d whose lines
    start and end anywhere (the round-2 verifier's `rhatch`) a small query cost ~3.4 sqrt(n) (840 node
    and box looks at 64 000 lines, against ~log n with even line ends), and round 2's step 7 took
    2.8-3.7x main's, the whole offset 1.12-1.44x main from 1 000 to 16 000 lines. Round 3 therefore puts
    each set in a `BoxIndex`: a uniform grid, laid out as main's (about as many square cells as boxes
    over the set's extent; at most max(1024, n) cells per side instead of 1024, so a long, narrow hatch
    gets cells about as tall as wide, not 16 lines tall), takes a box when it covers at most 256 cells
    and each of them holds fewer than 32 boxes; any other box goes into a `BoxTree`. A query asks both;
    a box filed in several cells is reported from the first cell it shares with the query (the cells
    of boxes that meet overlap), so each once. Evenly spread boxes (any hatch, comb, scribble) all land
    in the grid, ~8.5 looks per small query at 2 000 or 64 000 varied lines; a hatch next to a far
    feature keeps 32 boxes in the one crowded cell and the rest in the tree (~70-80 looks, O(log n));
    nested boxes (a spiral's semicircles, concentric squares) cover many cells and go to the tree. Memory
    is linear whatever the boxes: at most 32 entries per cell, 256 per box, 3 max(1024, n) + 1 cells.
    Which part holds a box changes the cost of a query, never its answer, so the distances, samples,
    cuts and outputs are unchanged. (Round 2 also fetched the tree's candidates once per window of 64
    tol along the strand; with the grid that saved nothing measurable, 0-2% on clustered hatches, and is
    gone.) Only step 7's
    grids changed: other users of `BoxGrid` act on the order in which items are reported (e.g. the
    union-find of mergeOnCarriers' groups), so another layout could change their output, and their cost on
    clustered input stays as it was before T-0128 (quadratic on this repro). Step 2's boundary split now lists
    its candidates before splitting them, in the same order: with the splitting inside the query callback GCC
    kept that scan's loop variable in memory once code was added elsewhere in the function (~12% on the
    T-0016 zig-zags at large d, which put back no strays).
    Cost (Release, CPU time, call order alternating, min of reps, shared 16-thread machine; "main" =
    origin/main b637c16, "pre" = 8cd3568 before T-0128). `perf2 lhatch` (hatch of N lines 10 mm long at pitch
    2d plus a U channel at (1e5, 1e5)): d = 0.002, N = 1 000: 0.21 s (main 2.4-2.5 s; 1.2-1.3x pre, was
    11.9x), N = 2 000: 0.73-0.87 s (main 9.6-10.0 s; 1.03-1.08x pre); d = 0.05, N = 1 000 / 4 000: 0.18-0.22 /
    3.4 s (main 0.31-0.39 / 6.5 s; 0.98-1.01x pre, was 1.7-2.0x); the same hatch with a background serpentine
    (fhatch, d = 0.002): 0.20-0.23 s (main 0.43 s; 1.04-1.09x pre). Step 7 is near-linear there; the whole offset stays quadratic like pre (its other
    grids: pre 0.32 / 1.30 s at 1 000 / 2 000 lines). Elsewhere (hatch, comb, rectangles, ring at pitch or
    width 2d, the T-0016 zig-zags, long channels), against a control build of origin/main's offset.cpp with
    the same flags, CPU cycles 0.94-1.04x, except the 3 000-line hatch at 2d, Round (1.09-1.11x; 3% more
    instructions, outside step 7, which takes ~2% of that offset) and a 10 m channel (1.04x, 0.5 ms).
    Round 2 (`BoxTree` instead of round 1's clustering grid; same machine and method, alternating with
    origin/main, min of 3-9 reps): the spiral slot of 200 / 400 semicircles 0.047 / 0.34 s (main 0.046 /
    0.30 s; round 1 1.25 / 6.4-12.6 s and 2.5 GB), the spiral of 400 semicircles with a far channel at 600 /
    50 mm 0.031 / 0.043 s (main 0.030 / 0.045 s; round 1 5.9 / 10.6 s), of 700 with one at 1e3 mm 0.11 s and
    10 MB (main 0.14 s; round 1 bad_alloc at 6 GB); `lhatch` d = 0.002, N = 1 000 / 2 000: 0.21 / 0.83 s
    (main 2.1 / 9.2 s, pre 0.19 / 0.80 s); hatch, comb, rectangles and ring at pitch or width 2d: 0.99-1.08x
    main and 0.99-1.05x round 1 (the query stack is not zeroed: zeroing it per query cost ~15% of the
    4 000-line hatch at d = 0.05). The self-crossing cuts of the back-and-forth strand: 400 spans in under
    1 ms, 40 000 in 0.1 s (round 1: 9.9 s and 2.8 GB at 400). Both spirals stay super-linear as on main
    (~7x per doubling of the semicircles, outside step 7: the nested arcs' boxes share the input grids'
    cells).
    Round 3 (`BoxIndex`; same machine and method, origin/main 4bb7542, min of 5-9 reps, call order
    alternating, two runs of the round-2 verifier's matrix on a machine at load 6-11). Varied line ends
    (`rhatch`, d = 0.05, no far feature), N = 1 000 / 2 000 / 4 000 / 8 000 / 16 000: 0.97-1.00 / 1.01-1.03
    / 0.99-1.01 / 0.93-1.07 / 0.93-1.10x main, the highest of each repeated at 1.03 / 0.93-0.99 (round 2
    1.12-1.25 / 1.20 / 1.31 / 1.40-1.44 / 1.43x); four more seeds at 4 000: 0.97-1.06x; d = 0.01 / 0.2 /
    1: 1.00-1.02 / 0.99-1.05 / 1.00-1.01x; Square, w = 1e-7: 1.01x; turned 90 degrees (vhatch, 1 000 /
    8 000): 0.99-1.03 / 0.99-1.07x. Step 7 alone (timed around putBackStrays and around main's inline
    block) 0.95-1.04x main's from 1 000 to 16 000 lines (round 2 2.8-3.7x). Uniform layouts (hatch, comb,
    rectangles, ring at pitch or width 2d, long channels, zig-zags): 0.89-1.03x. Clustered (one far
    feature): `lhatch` d = 0.002, N = 1 000 / 2 000 / 4 000: 0.22-0.25 / 0.82-0.85 / 3.4 s (main 2.3 /
    9.0-10.0 / 39-40 s), d = 0.05: 0.57-0.58x, with a far channel at 1e4 0.45-0.50x; fhatch 0.48-0.51x;
    chatch 0.70-0.84x; rhatch with a far channel 0.25x (d = 0.002) and 0.61-0.66x (4 000 lines). Spirals:
    the spiral slot of 200 / 400 semicircles 0.86-1.00 / 0.93-1.10x main (repeated 1.02x; step 7
    0.95-1.02x; 11 MB peak for both versions together), spirals of 400 / 700 semicircles with a far
    channel 0.98-1.03x (700: 10 MB), of 4 000 at d = 0.1 with one at 1e5: 5.5 s, main 5.2 s (one rep;
    15 MB). Outputs: cmp6, fresh seeds 0x9e3Fkkkk, 338 400 offsets (families 0, 2-5 x 50 400; 21-23, 25 x
    21 600): none differ, CPU 0.995-1.02x per family; channel and cusp sweeps identical to round 2; with
    the libm wrapper (K = 1..4), `[t0128]~[perf]` and `[t0172]~[perf]` 100 of 100 runs each, `[offset]~[perf]`
    12 of 12.
    (c, e) *Tests and libm.* Tests now pin a stray between tol and 10 tol that meets a chain (returned, and
    reaching that chain), the longer of a thin loop's strands, the order of strays among the chains, the
    exact-copy hand-back of mergeOnCarriers (a slot 2d wide with walls of unequal height: the copy over the
    lower wall's mitre extension is not d from the input), a long stray in one piece, a thin loop's strand of
    many spans in one piece, the self-crossing cuts against the pairwise test (near-tangent pairs included:
    every pair it accepts has grown boxes that meet), the cut fallback (`nonCrossingRuns`: consecutive
    runs, no tiny ends), `BoxTree` and `BoxIndex` queries against brute force (`BoxIndex` also with
    layouts that leave boxes outside, empty or non-finite) with their memory and work bounded (`BoxIndex`:
    at most 30 looks per small query on varied hatches of 2 000 and 64 000 lines, either way round, and
    growing less than 1.5x between them; at most 20 log2 n on the crowded hatch), timing for spiral slots,
    spirals, long or collinear strands and the varied hatch (Release, 16 000 lines: at most 1.35x the even
    hatch; measured 1.03-1.14x, round 2 1.56x), strays crowded next to a far feature (a hatch's middles
    end where they do without it; six fam4 0x2a0006b4 slots' floor strays never run along each other: the
    tree answers for the chains and for the strays put back), a long line by a tiny arc that meets it only
    through the line's 1e-7 x length growth (round-2 verifier's mutant XL), and exactJunction's fallback on
    its own
    (`detail::closeJunction`: arcs whose start is 1.5-95 x 1e-12 x the scale ahead, ~3 in 4 of them closed by the nearest candidate that does not
    start behind). The tests' crossing filter now applies only where removeCuspLoops looks (reversal
    junctions, within the tolerance along both spans). Three pre-existing checks were stricter than what
    offset.hpp documents and failed with sin/cos 1-4 ulps off glibc's (as MSVC's may be; round-3 verifier:
    14 of 24 perturbed `[offset]` runs): the T-0006 AC3 gap count used an absolute 1e-12 mm, not the
    documented 1e-12 x max(1, |x|, |y|, radius) between arcs (gaps of 2.3e-12 to 1.4e-11 mm seen; a junction
    with a line still counts as a gap unless bit-identical, as documented), and the
    T-0017 repro loops REQUIREd their literal arcs to be bit-continuous, which they are only with glibc's
    sin/cos (they are now made continuous with the platform's, a no-op on glibc). The closed loops that kept
    9.05e-12 / 9.64e-11 mm gaps under perturbation (fam4 0x7c2340188 K2 seed 3, 0x7c434015c K4 seed 3) are
    junctions next to arcs of radius 2.4e4 / 6.8e4 mm, within the documented bound (2.4e-8 / 6.8e-8 mm): the
    round-3 checker left the radius out of the scale. No product change there. With the libm wrapper
    (K = 1..4), `[offset]~[perf]` passes 24 of 24 runs and `[t0128]~[perf]` 200 of 200.
    Outputs: bit-identical to origin/main except the long strays above (cmp6, call order alternating, fresh
    seeds: 2 x 64 800 offsets of families 0, 2-5, 21-23, 25, none differ; CPU 0.99-1.01x per family).
