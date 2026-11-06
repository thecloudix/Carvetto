# Carvetto backlog

Format and rules: `docs/WORKFLOW.md`. Status values: `ready` · `in-progress` · `verifying` · `done` · `blocked`.
Phase 0 = foundations. Phase 1 = 2D/2.5D CAM MVP. Later phases are outlined in `docs/PLAN.md` and get cards when Phase 1 nears completion.

Concurrency note: T-0001 (geometry primitives) is the root dependency; everything else waits for it. Batch 1 = T-0001 alone. Batch 2 = T-0002, T-0003, T-0010, T-0020, T-0030, T-0040 in parallel.

---

## Phase 0 — Foundations

### T-0000 Repository scaffold                                                 [status: done]
Phase: 0   Module: repo   Depends on: —
Goal: CMake project, core lib + Catch2 tests, CI, docs, agent workflow. Done by the director.

### T-0001 Geometry primitives: Vec2, Box2, Transform2, Span, Contour         [status: done]
Phase: 0   Module: core/geometry   Depends on: —
Goal: The foundation every other module uses. `Vec2` (double), `Box2`, `Transform2` (affine 2×3, compose/invert/apply), `Span` as a variant of `LineSpan`, `ArcSpan` (centre, radius, start/end angle, direction), `BezierSpan` (cubic), `Contour` (ordered spans, open/closed flag, layer-agnostic), with length, bounding box, point-at-parameter, reverse, transform, and polygonisation to `Polyline` at a chordal tolerance.
Interfaces: owns `core/include/carvetto/geometry/{vec2,box2,transform2,span,contour,polyline,tolerance}.hpp`.
Acceptance criteria:
  1. `Transform2::rotation(π/2).apply({1,0})` ≈ `{0,1}` within 1e-12; compose(A,B).apply(p) == A.apply(B.apply(p)).
  2. `ArcSpan` length for a full circle r=10 is 2π·10 within 1e-9; `pointAt(0.5)` on a CCW half-circle from (10,0) to (−10,0) centred at origin is (0,10).
  3. `BezierSpan` length computed by adaptive subdivision agrees with a 10k-segment estimate within 1e-6 relative.
  4. Polygonising a circle contour r=10 at tolerance 0.01 produces a polyline whose max chordal deviation ≤ 0.01 and whose vertex count is < 200.
  5. `Contour::reverse()` of a closed contour yields the same polygon vertex set in reverse order; `bounds()` matches the polygonised bounds within tolerance.
  6. Headers compile with `-Wall -Wextra -Werror` and have `///` docs on every public type.
Out of scope: offsetting, booleans, intersection, snapping.
Notes/refs: ARCHITECTURE.md "Vectors are span lists".

### T-0002 Span intersections and contour utilities                           [status: done]
Phase: 0   Module: core/geometry   Depends on: T-0001
Goal: Robust line–line, line–arc, arc–arc intersection (Bezier via polygonisation + refinement), point-in-contour (even–odd and non-zero), contour orientation and area, closest point on span/contour, self-intersection detection.
Interfaces: `geometry/intersect.hpp`, `geometry/contour_ops.hpp`. Read-only: T-0001 headers.
Acceptance criteria:
  1. Two unit squares offset by (0.5,0.5) intersect in exactly 2 points, each within 1e-9 of the analytic answer.
  2. Tangent line–arc contact returns one intersection with a `tangent` flag.
  3. Area of a CCW 10×10 square is +100, CW is −100; `isInside` for (5,5) is true and (15,5) false for both orientations.
  4. Closest point on a half-circle arc from an external point is within 1e-9 of the analytic result.
  5. A figure-eight contour reports self-intersection; a square does not.
Out of scope: boolean operations (T-0003).
Re-scoped by director after 3 verification rounds (2026-10-24): 2-span closed contour junction defect and two tolerance-boundary findings moved to T-0005.

### T-0003 Polygon offset and booleans via Clipper2                           [status: done]
Phase: 0   Module: core/geometry   Depends on: T-0001
Goal: Wrap Clipper2 (FetchContent, pinned) behind `offsetContours(contours, distance, JoinType, tolerance)` and `booleanOp(Union|Difference|Intersection|Xor, subject, clip)` operating on `Contour`s: polygonise at tolerance, run Clipper2 in fixed-point, convert back, optionally refit arcs. Join types: round, sharp (mitre with limit), square.
Interfaces: `geometry/offset.hpp`, `geometry/boolean.hpp`. Read-only: T-0001.
Acceptance criteria:
  1. Offsetting a 10×10 square by +1 (round joins) gives one closed contour with area within 0.1% of 100+40+π.
  2. Offsetting a 10×10 square by −6 gives zero contours.
  3. Union of two overlapping squares gives one contour; difference of a square and a fully-contained square gives one outer and one hole (two contours, opposite orientation).
  4. Offset(+d) then offset(−d) of a rounded shape returns within 2·tolerance of the original (Hausdorff).
  5. Open-contour offset to the Left/Right of a polyline produces an open contour whose span count is n (the input count) minus trimmed inside corners, plus one join span per outside corner for Round/Square joins (and per mitre-limit fallback for Sharp). Every point of the result lies at distance ≥ d − tolerance from the input, and offset vertices/arcs lie at exactly d. *(Amended by director 2026-10-24: the original "same span count ±1" wording was wrong for joins at multiple outside corners.)*
Out of scope: fillets, trim (Phase 1 cards).
Re-scoped by director after 3 verification rounds (2026-10-24): self-intersecting loop+tail chains for Sharp/Square open offsets moved to T-0004 AC4.

### T-0004 Open-contour offset returns every chain; performance for dense input   [status: done]
Phase: 0   Module: core/geometry   Depends on: T-0003   ADR: 0012 (if needed)
Goal: (Also absorbs the T-0003 round-3 residual defect: with Sharp/Square joins, an offset that curls into a closed loop can return loop + tail where the tail crosses the chain start — verifier repros A/B in T-0003 notes; 8/6840 random cases.) `offsetOpenContour` currently returns only the longest chain when the offset splits (parts of the path within 2d of each other). For CAM this silently drops geometry. Return all chains in path order, and bring cost for 2000+ span inputs (polygonised Beziers) to well under a second.
Interfaces: owns the signature of `offsetOpenContour` (change return to `std::vector<Contour>`); update all call sites/tests.
Acceptance criteria:
  1. A hairpin polyline narrower than 2d offset on the inside returns ≥2 chains, each satisfying the T-0003 AC5 distance property, ordered along the input path.
  2. For fixed-seed random paths with full ±180° headings (spirals, U-turns), the union of returned chains covers every input point's offset that is ≥ d − tol from the whole input (no valid geometry dropped), and every returned point is ≥ d − tol from the input.
  3. A 2000-span polyline offset completes in < 0.5 s in Release on CI Linux.
  4. No returned chain is self-intersecting for any join type (split at T/X junctions into loop and tail). Regression repro A: lines (-6.4312190988360092,1.1428523003734297)→(-12.846939679779521,3.9348207104935331)→(-9.7434797348853266,-1.9260892800402662), then CW arc c(-17.535585621268488,-1.0001000247398364) r=7.846933811721553 a0=-0.11828212233321031 a1=-5.0521988806306926, then CW arc c(-17.021348114572877,0.45450829964057693) r=6.3041034287405493 a0=1.2309864265488941 a1=0.23263422704120162; offset d=2.5 Right with Sharp and Square.
Out of scope: closed-contour offsets (Clipper2 path), arc refitting.

### T-0005 Intersection residuals: 2-span closed contours, tolerance-boundary cases   [status: done]
Phase: 0   Module: core/geometry   Depends on: T-0002   ADR: amend 0009
Goal: Close the residual defects found in T-0002's third verification round.
Interfaces: `geometry/intersect.hpp` / `contour_ops.hpp` implementations; signatures unchanged.
Acceptance criteria:
  1. `selfIntersections` / `hasSelfIntersections` on a closed contour of exactly 2 spans ignores BOTH shared junctions (the inner one and the closing one). Repros, all must report no self-intersection: half-disc `LineSpan{(-5,0),(5,0)}` + `ArcSpan{(0,0),5,0,π,CCW}`; Bezier lens `{(0,0),(2,2),(4,2),(4,0)}`+`{(4,0),(4,-2),(2,-2),(0,0)}`; smooth 2-Bezier oval; line+Bezier D-shape. A 2-span closed contour that genuinely crosses itself (e.g. two Beziers forming a figure-eight away from the junctions) still reports the crossing. Also cover 1-span closed contours (full circle; self-looping Bezier).
  2. Arc–arc with centre distance d ≤ tol but radii differing by ≤ 2·tol, where the circles' closest approach is ≤ tol, follows the same "tangent within tol → one hit" rule as the general case (or the ADR states precisely why not). Regression: `fullCircle({644.61470300472024,365.33035654442392},181.1984687011373)` vs `fullCircle({644.61399749030329,365.33092243365013},181.1966002797156)`.
  3. Arc/Bezier and Bezier/Bezier pairs with two shallow crossings closer than ~2·tol give the same result in both argument orders; when two hits are merged into one, the merged hit carries `tangent=true`. Regression: `ArcSpan{{-3.2305423907849837,-8.7399422212084588},9.3248612306533616,-0.86772800461480459,-5.0533095891647033,CW}` vs `BezierSpan{{-14.222937128613282,-10.138318879522913},{-12.121499048115544,-10.250528577090371},{-11.910081639881342,-6.2911507142599774},{-14.011519720379081,-6.1789410166925203}}`. Symmetry fuzz on ≥9,000 AB/BB pairs: 0 asymmetries.
  4. Remove the "Known limitation (tracked as T-0005)" note from `intersect.hpp`.
Out of scope: Bezier overlap detection; Debug-build Bezier length performance.
Re-scoped by director after 3 verification rounds (2026-10-24): AC1, AC2, AC4, FillRule unification, completeness (0 misses / 48k pairs), symmetry and line/mirrored-cubic touches verified clean; residuals (circle touches dropped in bands near Bezier piece cuts, asymmetric end touches, sliver self-loops, junction-adjacent bulge crossing) moved to T-0007.

### T-0006 Open-offset quality for toolpaths: speed at large d, exact arc mitres, snapped junctions   [status: re-scoped → T-0017]
Phase: 1 (before the 2D Profile toolpath)   Module: core/geometry   Depends on: T-0004
Goal: Non-blocking findings from T-0004 verification that matter once offsets feed G-code.
Acceptance criteria:
  1. Speed: a 2000-step 0.5 mm random walk offset by d = 1 and d = 2 (all joins, both sides) completes in < 0.5 s Release; 8000-span hypotrochoid and text-like loops at d = 2 in < 1 s. Profile and fix the neighbourhood-boundary splitting / distance-pruning cost.
  2. Sharp joins at arc corners produce the true mitre whenever it lies within `mitreLimit·d` (no fallback to Square/Round caused by trimming the arc's extension range); Square likewise. Header documents every remaining fallback. Regression: keyhole, d = 0.25, Left, Sharp → mitre at 0.304 from the vertex (limit 0.5), not an arc.
  3. Consecutive spans within a returned chain meet exactly (shared endpoint bit-identical, snapped), and no span starts behind the previous end along the chain. Fuzz: 0 gaps > 1e-12 and 0 backward steps over ≥ 20k generated offsets.

Re-scoped by director after 3 verification rounds (2026-10-26, owner decision: re-scope, do not merge): AC1 speed and AC2 mitres verified clean; AC3 fixed for exactly and near-collinear steep zig-zags (noise sweep 1e-13…1e-4 clean), but the round-3 "nearly the same carrier" grouping is worse than main in ~25/10k new adversarial cases (offset inside d, Round beyond d, spurious chains) and the back-step safety net skips 3-span chains. Not merged; work continues on branch task/T-0006-offset-quality (b722cf2, pushed) under T-0017.

### T-0017 Open-offset quality II: residuals of T-0006   [status: done (2026-11-02, #24)]
Paused 2026-10-26 (owner). Branch task/T-0017-offset-quality-ii @ f8e4506 (WIP, pushed; builds, 494/494 serial). Round-1 verifier FAIL (7 items + translation-dependent needle removal); coder reports 6/7 + needle removal fixed; item 5 (self-crossing 0xe400137f) is a cancellation bug in core/src/geometry/intersect.cpp general intersect() — needs its own card. On resume: coder runs fam2/3/4 + hp/lens sweeps, -j8 repeat, CHANGELOG line, then round-2 verification.
Phase: 1 (before Profile/Pocket ship)   Module: core/geometry   Depends on: T-0006 branch (b722cf2)   ADR: 0012 (amend A1)
Goal: Finish T-0006 so it can merge: everything T-0006 verified clean stays clean (AC1 speed, AC2 mitre oracle, steep zig-zags exact and noisy, R1/R2/R3, needle-tip classification), and the round-3 verifier's residuals are fixed. Repros and harness: T-0006 round-3 verifier scratchpad `t0006v3/` (repro_r4/r5/r6.cpp, vdrv3.cpp with gen3.inc, bnd.cpp).
Acceptance criteria:
  1. Nearly-same-carrier grouping: a real angle limit (or one relative to element length) so short elements at 11–20° are never grouped, and near-carrier groups are split where their carriers cross so the outermost copy is outermost along the whole stretch. Regressions R4 (6 lines, d=0.03169564160714932, Square, Right, mitre 2.1158 → 1 chain, ≥ d − allowance), R5 (line + two near-co-circular arcs, d=0.1712, Sharp, Right → ≥ d − 5e-7), R6 (8 lines near (−5280,−5270), d=0.676, Round, Left → no span beyond d, same chain count as main).
  2. The back-step safety net runs on chains of every length (including 3 spans); nearly co-circular arcs meeting end to end are merged. Regression: 0x7300029 (verifier gen3) → no backward span.
  3. Sub-tolerance-wide closed two-arc lenses are dropped or merged (0x82004c8), and the exact overlap 0x840117a is fixed.
  4. Decisive: no result worse than main over ≥ 20k new-seed offsets covering the legacy families and the T-0006 round-3 families (short pieces, grouping boundary, co-circular noise, translations to ±1e4), judged by an exact checker; AC3 of T-0006 holds (0 gaps > 1e-12, 0 back-steps, 0 retraces, 0 duplicated chains).
  5. Header and ADR-0012 A1 are true as written: "every point ≥ d" / "Round exactly d" qualified only by the documented sub-tolerance-input limit; arc-junction gap claim qualified "for continuous input"; safety-net bridge length stated correctly; cost figures re-measured (Sharp/Square dense zig-zag figure belongs to T-0016).
  6. Timing tests that claim CPU time measure CPU time on Windows too (MSVC `std::clock` is wall-clock: use GetProcessTimes or keep the assertion a complexity ratio).

### T-0007 Intersection residuals II: circle touches near Bezier cuts, end-touch symmetry, sliver loops   [status: done]
Phase: 1 (before Trim / Vector Validator)   Module: core/geometry   Depends on: T-0005   ADR: amend 0009
Goal: Close the residuals from T-0005's third verification round.
Acceptance criteria:
  1. An external circle/arc tangent to a convex cubic with gap 0 < h ≤ tol is reported exactly once (tangent) at EVERY position, including the ±0.0035 bands around the 1/16 piece cuts and near the span start. Regressions (0 hits today, both orders): A = BezierSpan{{-3,9},{-1,-3},{1,-3},{3,9}} vs ArcSpan::fullCircle({-2.4572958060271235,5.0734751657764976},0.2) (tA≈0.123, h=1e-5) and fullCircle({-2.0592815815728511,3.4301656512398564},0.2) (tA≈0.189, h=1e-4); A = {{-1,0},{0.5,3},{1,3},{3,-1}} vs fullCircle({-0.92142472309476964,0.59261447867484451},0.2) (tA≈0.059), fullCircle({2.2044174888982013,0.91064133105013123},0.2) (tA≈0.816), fullCircle({-1.1657867390182135,0.11673084174268154},0.2) (tA≈0.003, h=5e-4). Test: fine sweep t = i/1000 on ≥ 4 convex cubics × R ∈ {0.2, 1.0} × h ∈ {1e-5, 1e-4, 0.99·tol}: 0 drops.
  2. End touches are symmetric: when A's end comes within tol of B (foot clamped at A's end) the touch is reported exactly as when B's end comes within tol of A. Regression: A = ArcSpan{{1.2115827528094094,3.0195632709834515},5.9592789586434449,5.5749720172249306,7.0509709398542197,CCW}, B = BezierSpan{{5.4971827427603293,7.1603566376986816},{7.6315225436121663,4.949460611577611},{7.7363570760098641,1.4786963115444329},{5.7389721228483266,-0.85517614189571634}} → both tA≈0/tB=1 and tA=1/tB≈0.00028 reported.
  3. Thin sliver self-loops of a single cubic (0.05–0.3 mm long, 0.3–5 µm thick) are found by selfIntersections (seed chord crossings from the curve's own self-intersection parameter pair, computed analytically or by subdivision).
  4. A Bezier that bulges ≤ ~tol outside the arc it joins and re-crosses it within ~10·tol of the junction reports the re-crossing (sign test not defeated by the exact-zero sample at the junction). Regression: Bez{{-0.18860739840346241,3.8634722338314074},{0.0522167808921572,4.3830641573127442},{1.6272051316098195,-2.0179279547519577},{1.9578386933824259,-2.3369534285717775}} vs Arc{{6.4363713372296427,1.9356191918458676},6.1896794294008179,-2.3797254616523524,3.0863225397506588,CW}.
  5. ADR-0009 guarantee bullets amended to match; the "Known limitations (tracked as T-0007)" note removed from intersect.hpp.

### T-0008 Exact line/arc intersections: end touches and shared endpoints   [status: done]
Phase: 1 (before Trim / Join / Vector Validator)   Module: core/geometry   Depends on: T-0007
Goal: The analytic line/line, line/arc and arc/arc paths (not the Bezier pipeline) miss end touches with 0 < h ≤ tol, and can miss an EXACT shared endpoint. Found while verifying T-0007; identical on main.
Acceptance criteria:
  1. An end of either span within h ≤ tol of the other span's interior or end gives exactly one hit (tangent when the spans don't cross there), in both argument orders, for L/L, L/A, A/L, A/A, full circles included. Regression (0 hits today, endpoints 1.8e-15 apart): LineSpan{{-2.092227545811626,-4.3793988799678552},{9.7310280804032239,-4.1770917092683604}} vs ArcSpan{{-2.185139788351961,0.95685533589696803},5.3370630257800924,2.4383258356681337,4.7297987310031928,CW}.
  2. Generative (fixed seed, ≥ 10k pairs per combination; oblique, normal and near-parallel approaches; h ∈ [0, 0.99·tol] and [1.05·tol, 2·tol]) vs brute force: 0 missed end touches, 0 asymmetries, 0 hits beyond tol, residuals ≤ tol.
  3. The analytic near-tangent "fold two crossings into one tangent hit" rule and the junction-erasure rule are re-examined: crossings separated by > 2·tol are both reported (as for Bezier pairs); a single tangent hit within tol of a contour junction is not erased when it is a genuine re-crossing. Document the final rules in ADR-0009.
  4. Regression vs main on ≥ 50k random pairs: every difference is an improvement by brute force; symmetry bit-exact.
  6. Ends inside near-coincident (hugging) stretches — e.g. a Bezier lying within tol of an arc that extends past it — are reported: an end of either span within tol of the other's interior gives one tangent hit even when the curves run within tol of each other along that stretch. Today ~38–45% are missed (main and T-0007 alike; T-0007 round-3 verifier measured 44.5% and 44.8% on two seeds). Generator: Beziers within tol of an arc extending ≤ 0.3 rad past them, ≥ 6,000 pairs, vs brute force → 0 misses of ends whose foot is ≥ 0.01 mm from the other span's end.
  5. Root-cause fix found in T-0007: `closestPoint(ArcSpan)` (contour_ops.cpp) returns the start point but parameter 1 when the query's angle lies a rounding error before the arc's start. Fix it at the source (parameter 0), add a fine-angle regression around both ends of CW/CCW partial arcs, and remove T-0007's wrapper in intersect.cpp. Check every other caller (contour closestPoint/distanceTo, offset pruning) is unaffected or improved.

### T-0009 Offsets of inputs with sub-tolerance spans   [status: shelved (owner 2026-11-05 after 4 rounds; main keeps the documented limitation; branches task/T-0009-subtol-spans (r1–3) and task/T-0009-subtol-r4 (r4), harnesses t0009v1–v4; revisit as a smaller re-scoped card)]
Phase: 1 (before Profile/Pocket ship)   Module: core/geometry   Depends on: T-0017
Goal: `flattenForOffset` drops spans ≤ `tol.distance`, so the distance/coverage guarantees of `offsetOpenContour` hold only for the flattened input. Next to a dropped span an offset point can be short of d by up to ~tol (9e-4 mm seen), a Round arc about the neighbouring vertex can be up to ~tol over d (8e-4 mm seen), and up to ~1 mm of valid offset can be missing (seed 0x60660ace). About 1 in 1000 such inputs; already in main before T-0006. Found by the T-0006 verifier and coder.
Acceptance criteria:
  1. Sub-tolerance spans are merged into a neighbour (path kept continuous) or their end points treated as vertices with full circles in pruning/coverage — coder's choice, recorded in ADR-0012.
  2. 0 distance, Round-over-d and coverage violations over ≥ 20k fuzzed inputs containing sub-tolerance spans (all joins, both sides, open and closed), judged by an exact checker.
  3. No result worse than main on inputs without sub-tolerance spans; T-0006 speed criteria still hold.
  4. The limitation note in offset.hpp and ADR-0012 A1 is removed or narrowed to what remains.
Owner decisions 2026-11-04: (a) corners between two sub-tolerance spans are Round whatever the requested join (corners touching a longer span keep the requested join); (b) tiny end spans are trimmed only up to tol from each open-chain end (a chain made only of tiny pieces is offset, not dropped), accepted although 5/24k AC3 offsets differ from main (4 of them main was worse). (c) a sub-tolerance near-reversal never flips the offset to the other side: the offset stays on the requested side and closed loops stay closed (as on main), at the cost of at most a tolerance-sized deviation near the cluster.

### T-0016 Sharp/Square offset cost on dense hatch-like vectors   [status: done (#58)]
Phase: 1 (alongside T-0120/T-0121)   Module: core/geometry   Depends on: T-0017
Goal: Scaling is linear, but on dense zig-zags with d ≫ pitch Sharp/Square joins are 10–20× slower than Round (1000-span zig-zag, pitch 0.02, amplitude 1, d=7: ~14 s Release vs 0.7 s Round). Hatch- and fill-like vectors will hit this in Profile/Pocket.
Also (found by the T-0009 coder, 2026-11-02): input made largely of sub-tolerance spans is now offset (main returned nothing) but slowly — a closed circle of 8,000 spans of 0.4 µm ~6 s Release at d=1; random walks of 0.5 µm steps 0.44 / 1.55 / 3.4 s at 2,000 / 4,000 / 8,000 steps (super-linear). Bring it to the same scaling bound as the hatch cases.
Acceptance criteria:
  1. The 1000-span case above runs within 3× of Round for Sharp and Square (Release, CPU time), and 4000 spans scale ≤ 2.5× per doubling.
  2. Results bit-identical to, or judged no worse than, the T-0006 output on the T-0006 fuzz set (all joins, both sides).
  3. ADR-0012 cost caveat updated with the new measurements.

### T-0010 Document model: Job, Sheet, Layer, Object, UndoStack                [status: done]
Phase: 0   Module: core/document   Depends on: T-0001
Goal: In-memory document: `Job` (type single/double/rotary, size, units display setting, Z-zero, datum), `Sheet`s (each with size/datum), `Layer`s (name, colour, visible, locked, order), `Object`s (vector object holding contours + creation params as a tagged variant; bitmap placeholder; group), stable IDs, selection set, and a command-based `UndoStack` with coalescing.
Interfaces: `document/*.hpp`. Read-only: T-0001.
Acceptance criteria:
  1. Creating a job 10×10×0.5 in with datum bottom-left stores 254×254×12.7 mm internally; a display-unit query returns inches values.
  2. Adding an object to a layer, undo, redo round-trips the object identity and layer membership.
  3. Deleting a layer with objects moves them to the default layer unless `deleteContents` is set; both paths undoable.
  4. 10 000 object inserts + undo all + redo all completes under 1 s in Debug.
  5. Object IDs are unique across undo/redo and survive copy/paste as new IDs.
Out of scope: file I/O (T-0012), rendering.

### T-0011 IdGenerator overflow guard                                           [status: done]
Phase: 0   Module: core/document   Depends on: T-0010
Goal: The id generator wraps at 2^64 and can then hand out ids that collide with existing sheets/layers/objects (found while verifying T-0012). Make allocation fail loudly instead.
Acceptance criteria:
  1. Allocating past a documented ceiling (2^62) is impossible: the generator reports exhaustion (assert in Debug + an error/no-op result from the commands that allocate in Release), never wraps, never reuses.
  2. `Document::checkIntegrity()` flags any id ≥ the ceiling and a next-id ≤ any existing id.
  3. Regression: a document whose generator is near the ceiling cannot be driven into a duplicate id by any public command sequence (fixed-seed randomized test).
Out of scope: file I/O (T-0012 already rejects out-of-range ids on load).

### T-0012 Native `.carv` file I/O                                            [status: done]
Phase: 0   Module: core/document   Depends on: T-0010   (ADR-0003 accepted)   ADR: 0013
Goal: Save/load the document to the `.carv` zip container (miniz, nlohmann/json pinned). Versioned manifest; forward-compatible unknown fields preserved.
Acceptance criteria:
  1. Save → load → save produces byte-identical `job.json` for a document with 3 layers, 50 vector objects (lines, arcs, Beziers), text params, and document variables.
  2. Loading a file with an unknown top-level JSON field keeps the field on re-save.
  3. Loading a file with a higher major version fails with a clear error, not a crash.
Out of scope: heightfields, toolpath caches (later phases).
Round-3 direction (director, 2026-10-24): untrusted JSON is parsed via SAX straight into records (no DOM), unknown fields kept as raw text under a global budget, linear group-structure pre-check in the loader; checkIntegrity performance moved to T-0014 AC4.
Re-scoped by director after 3 verification rounds (2026-10-24): JSON loader, round trips, ids, structure pre-check and 46 JSON bombs verified clean; two residuals in the zip layer (miniz in-memory inflate hang on damaged streams; overlapping entries / whole-buffered unknown members exceeding the memory target) moved to T-0015, which blocks any UI file open/save.

### T-0013 Reject preset ids already in use; integrity check for cross-kind duplicates   [status: done]
Phase: 0   Module: core/document   Depends on: T-0011
Goal: Commands that accept a caller-supplied (preset) id currently accept one already used by another kind (e.g. an object taking a layer's id) or an id below the generator's next value (re-adding a removed id). Found while verifying T-0011.
Acceptance criteria:
  1. Every command accepting a preset id refuses (no-op, returns false) an id that is in use by ANY sheet, layer or object, or that is below the generator's next id.
  2. `Document::checkIntegrity()` reports an id used by more than one item across kinds (new IntegrityIssue kind).
  3. Regression: `o.id = ObjectId{d.defaultLayer().value}; d.execute(AddObjectCommand(o))` returns false and leaves the document unchanged; fixed-seed randomized test over all commands with preset ids finds no cross-kind duplicates.

### T-0014 Document hardening: id/key mismatch check, load-robust timing test   [status: done]
Phase: 0   Module: core/document   Depends on: T-0013
Goal: Two small follow-ups found during T-0011/T-0013 verification.
Acceptance criteria:
  1. `checkIntegrity()` reports an object whose stored `id` differs from its map key (new or existing issue kind), and DuplicateId considers stored ids as well as keys. Regression: insertObject, then `findObject(o)->id = ObjectId{defaultLayer().value}` → issue reported.
  2. The T-0010 "10 000 inserts, undo all and redo all under one second" test no longer fails under parallel ctest load: measure CPU time of the test thread (or use a generous wall bound plus an operation-count/complexity assertion), and keep detecting an O(n²) regression (e.g. assert time(20k) < 3 × time(10k)).
  3. `ctest -j8` passes 10 consecutive runs on the CI Linux runner (add a CI step or document the local check).
  4. *(Added by director 2026-10-24, found verifying T-0012.)* `checkIntegrity()` is O(N log N) or better regardless of group depth/width: a 60,000-deep nested-group chain and a single group with 200,000 children each check in < 1 s Release; iterative (no recursion) cycle detection still finds a cycle at depth 50,000; all existing issue kinds and ordering unchanged.

### T-0015 .carv zip-layer hardening: bounded inflate, overlapping entries, held-bytes cap   [status: done]
Phase: 0 (must land before any UI file open/save)   Module: core/document   Depends on: T-0012   ADR: amend 0013
Goal: Close the two blocking findings from T-0012's third verification round, which are in the archive layer, not the JSON loader.
Acceptance criteria:
  1. A truncated, corrupted or size-lying deflate stream can never hang: decompression of in-memory archives detects "needs more input with no progress" (miniz 3.0.2 `mz_zip_reader_extract_iter_read` never decrements `comp_remaining` for memory archives) and returns InvalidArchive in bounded time. Regressions: central-directory comp_size=5 for job.json; deflate stream cut in half with consistent comp_size; 5 random bit flips inside job.json's compressed data. Each must fail in < 100 ms Release. Fix by tracking consumed input and bounding by the entry's comp_size (own inflate loop over tinfl, or the callback API with a no-progress guard) — do not patch the vendored miniz source.
  2. Archive entries whose local headers overlap or are duplicated (several CD entries pointing at the same data offset) are rejected; the total bytes *held* after load (members + extras) is capped at max(64 MiB, 32 × archive size); unknown `.json` members are streamed through the strict reader for validation and then either kept under the extras budget or rejected — never buffered whole beyond the cap. Regression: 60 CD entries pointing at one 1.8 MB stream declaring 16 MiB each (1.76 MB file) → rejected, peak RSS < 128 MiB; unknown_json_250M (1.8 MB) → rejected or < 128 MiB.
  3. Fuzz in CI (Release, Linux): ≥ 5,000 iterations over ≥ 2 seeds of bit flips / truncation / JSON mutation, each iteration under a per-load timeout (fork + alarm) — zero hangs, zero crashes.
  4. Timing assertions in carv_io_test.cpp use CPU time or complexity ratios (T-0014 style), not wall-clock bounds that fail under parallel ctest.
  5. ADR-0013 §9 rewritten to match measured behaviour (own numbers; unknown-member buffering stated; Debug times stated honestly).

### T-0020 Tool database and machines                                          [status: done]
Phase: 0   Module: core/tooling   Depends on: T-0001
Goal: `ToolType` (end mill, ball nose, V-bit, engraving, tapered ball nose, radiused end mill, form tool with profile, diamond drag, drill), `Tool` geometry per type, `CuttingData` keyed by (machine, material): pass depth, stepover, final/clearance stepover, spindle, feed, plunge, tool number; `Material`, `Machine` (name, accelerations, associated posts); `ToolDatabase` with groups, CRUD, JSON import/export, merge with/without overwrite.
Acceptance criteria:
  1. A V-bit 90° ⌀12.7 reports cutting width at depth 3 mm = 6 mm.
  2. Export → import → export is byte-identical JSON.
  3. Merge-without-overwrite keeps existing tool cutting data and adds new tools; merge-with-overwrite replaces.
  4. Looking up cutting data for a missing (machine, material) pair falls back to the tool's default data and says so.
Notes/refs: feature-inventory §5 "Tool Database".

### T-0030 `.pp` post-processor interpreter                                    [status: done]
Phase: 0   Module: core/post   Depends on: T-0001
Goal: Parse the `.pp` grammar (ADR-0004): global settings, `VAR` definitions with format spec, `begin/end` blocks, double-quoted output lines, `+`-prefixed comment lines. *(Amended by director 2026-10-24: `+` marks comments in existing posts; quoted strings are output.)* Interpret a `Toolpath` move list (consume `toolpath/move.hpp` and `toolpath/toolpath.hpp` from T-0040 — read-only; see ADR-0008) into text: header, per-toolpath tool change / new segment, rapid/feed/plunge/retract/arc/dwell blocks, footer; variable formatting (precision, absolute/incremental, prefix), line numbering, unit scaling, arc I/J vs R, inverse-time option.
Acceptance criteria:
  1. A hand-written Grbl post + a square profile at Z−3 with one rapid, one plunge, four feeds and a retract produces the golden G-code in `core/tests/golden/post/grbl_square.nc` exactly.
  2. Variable format `[X|A|X|1.3]` renders 12.3456 as `X12.346`; incremental mode renders deltas.
  3. Arc moves render via `CW_ARC_MOVE`/`CCW_ARC_MOVE` with I/J relative to start; when the post lacks arc blocks, arcs are linearised at the post's tolerance.
  4. Unknown block names produce a parse warning, not a failure; unknown variable tags are an error naming the line.
  5. `ROTARY_WRAP_Y` wraps Y into A for a 50 mm diameter job (Y=π·25 → A=180°).
Out of scope: saving dialogs, direct output.

### T-0040 Toolpath core types and linking                                    [status: done]
Phase: 0   Module: core/toolpath   Depends on: T-0001
Goal: `Move` (kind: rapid/feed/plunge/retract/arcCW/arcCCW/dwell; endpoint; arc centre; feed override), `Toolpath` (name, tool snapshot, moves, bounding box, length by kind), `PassPlan` (start depth, cut depth, pass depth, exact-step flag, last-pass thickness → list of Z levels), linking helper that joins contour passes with safe-Z retracts and rapids, and `estimateTime(Toolpath, rapidRate, feed, accel)` baseline.
Interfaces: `toolpath/{move,toolpath,pass_plan,linking,time}.hpp`. If T-0030 has already defined `Move`, adopt it unchanged.
Acceptance criteria:
  1. PassPlan(0 → 12 mm, pass 5 mm) yields Z levels [−5, −10, −12]; with `maintainExactStep` false it yields three equal passes of 4 mm.
  2. Linking two separate closed contours at depth −3 emits exactly: rapid to safe Z above first start, plunge, feeds…, retract, rapid, plunge, feeds…, retract.
  3. Total length by kind sums to the analytic value for a 10×10 square at one depth within 1e-9.
  4. Time estimate for 1000 mm at 1000 mm/min with infinite acceleration is 60 s; with finite acceleration it is strictly greater.

### T-0050 CI pipeline: Linux core+tests, Windows Qt app                        [status: done]
Phase: 0   Module: repo   Depends on: T-0001
Done by director during Batch 2 (Ninja + MSVC on Windows, annotations surface errors).
Goal: `.github/workflows/ci.yml` builds and tests on `ubuntu-latest` (GCC, core only) and `windows-latest` (MSVC, Qt 6 via `jurplel/install-qt-action`, full app). Caches FetchContent deps. Fails on warnings.
Acceptance criteria:
  1. A PR touching core triggers both jobs; both green on `main`.
  2. Introducing an unused variable fails the Linux job.
  3. The Windows job produces the `carvetto.exe` artifact (even if the app is a blank window).

### T-0060 Qt app shell                                                        [status: done]
Phase: 0   Module: app (+ app/viewmodel)   Depends on: T-0010   ADR: 0014
Goal: Main window with dockable panels (Design, Layers, Sheets, Toolpaths placeholders), a 2D canvas with pan/zoom/rulers rendering `Contour`s from a `Document`, a blank 3D view tab, status bar with cursor coordinates in display units, File ▸ New job dialog (job type, size, units, Z-zero, datum).
Structure (required because Qt is not available in the agents' workspace): all non-trivial view logic lives in a **Qt-free** static library `carvetto_viewmodel` (`app/viewmodel/`): view transform (world mm ↔ screen px, zoom about cursor, pan, zoom-to-fit with margin), ruler tick generation (nice-number steps in the display unit, labels), polygonisation of document contours for drawing at a screen tolerance, cursor-coordinate formatting in display units, New Job form validation → `Job`. It is unit-tested headless in `core/tests`-style Catch2 tests (`app/viewmodel/tests/`), built and run in every environment. The Qt layer in `app/src/` is thin glue.
Interfaces: owns `app/**`. Read-only: core.
Acceptance criteria:
  1. View transform: world→screen→world round-trips within 1e-9 mm; zooming about a cursor point keeps that world point under the cursor; zoom-to-fit of a 254×254 mm job in a 1000×800 px view with 5% margin fits exactly on the limiting axis.
  2. Rulers: for any zoom from 0.01 to 1000 px/mm, major tick spacing on screen is between 50 and 150 px, steps are 1/2/5×10ⁿ in the display unit (inches use the same rule on decimal inches), labels are exact (no 0.30000000004).
  3. New Job form validation rejects zero/negative sizes and produces a `Job` identical to `Job::singleSided(...)` for valid input in both units.
  4. The Qt app builds on Windows (MSVC) and Linux CI, and `carvetto --self-test` (headless, `-platform offscreen`) creates a job, adds a circle, renders one frame of the 2D canvas to an image, verifies non-background pixels exist inside the material rectangle, undoes the add, re-renders, verifies the circle pixels are gone, saves/restores dock layout via QSettings to a temp location, and exits 0. CI runs this.
  5. Core stays Qt-free: no `#include <Q` under `core/`, and `carvetto_viewmodel` links no Qt.
Out of scope: drawing tools, selection, 3D rendering (blank tab only).

---

## Phase 1 — 2D/2.5D CAM MVP

Goal: the PLAN §5 acceptance test — import a DXF, add text on a curve, pocket + profile with tabs, V-carve with a clearance tool, preview, post to Grbl, cut it on a real machine. Cards are written in batches; each batch is dispatched only after the owner approves the phase. Owner decision 2026-10-24: paused after Phase 0. **Owner decision 2026-10-25: Phase 1 started.** ADR numbers are pre-assigned per card. Concurrency 4; cards in the same module that share headers do not run together.

### Batch A — vertical slice first (prove the pipeline end to end, headless)
These five cards, together with what Phase 0 already has, make `carvetto-cli` able to turn a DXF into Grbl G-code with no UI. Everything later plugs into this spine.

### T-0100 DXF import and export (lines, arcs, polylines, splines → contours; layers)   [status: done]
Phase: 1   Module: core/io   Depends on: T-0005   ADR: 0015
Goal: `importDxf(bytes) → {layers, contours, warnings}` and `exportDxf(layers, contours)` (director 2026-10-25: geometry-level API, not document-level) for DXF R12–2018 ASCII (and binary R12+ if cheap): LINE, ARC, CIRCLE, ELLIPSE, LWPOLYLINE (bulges → arcs), POLYLINE/VERTEX, SPLINE (NURBS → cubic Beziers within chordal tolerance), INSERT/BLOCK (flattened, nested), TEXT/MTEXT (as text objects with font name, height, position — rendered later by T-0130), $INSUNITS and $MEASUREMENT for unit scaling, layer names/colours/visibility. Own parser (no third-party DXF lib; the format is public). Export writes R2000 ASCII with layers, LWPOLYLINE bulges for arcs, SPLINE for Beziers.
Interfaces: owns `core/include/carvetto/io/dxf.hpp`.
Acceptance criteria:
  1. A hand-written DXF with one of every supported entity imports to contours whose geometry matches the analytic expectation within 1e-6 mm (bulges → exact arcs; spline → Beziers within 0.01 mm chordal).
  2. Export → import round trip of a random document (lines/arcs/Beziers, 5 layers) reproduces every contour within 1e-9 mm and every layer name; exporting the same input twice is byte-identical (export(import(export(x))) need only preserve geometry within 1e-9 — director 2026-10-25).
  3. Units: $INSUNITS = inches scales to mm on import; export writes $INSUNITS = mm.
  4. Nested INSERT with rotation/scale/mirror flattens correctly (checked against hand-computed transforms).
  5. Malformed input (truncated, bad group codes, unknown entities, 100 MB of garbage) never crashes; unknown entities produce warnings, not errors; fuzz ≥ 2,000 iterations under ASan.
Out of scope: DWG, hatches, dimensions entities (imported as warnings), 3D entities.

### T-0110 Tool database defaults and the Grbl/Mach3/LinuxCNC/Marlin posts   [status: done]
Phase: 1   Module: posts, core/tooling   Depends on: —
Goal: Ship a starter tool database (JSON, ~30 common tools: end mills 1/8–1/2 in and 2–12 mm, ball noses, 60/90/120° V-bits, engraving, tapered ball nose) with conservative wood/plastic/aluminium cutting data, plus original posts for Mach3, Mach4, LinuxCNC, Marlin (mm and inch variants where idiomatic) alongside the existing Grbl post, each with a golden-file test.
Acceptance criteria:
  1. Each post parses with no warnings and reproduces its golden output for the shared square-profile + pocket-with-arcs + drill-with-dwell fixture.
  2. Each golden file hand-reviewed for machine safety (units, absolute mode, safe Z before rapids, spindle on before plunge, spindle off and program end) — the reviewer's checklist goes in `posts/README.md`.
  3. The starter tool database loads, validates, and every tool has cutting data for at least one material.
Clean-room: no text from any vendor post. Marlin/Grbl/LinuxCNC/Mach dialect facts come from their public docs.

### T-0120 2D Profile toolpath   [status: done (with T-0124, 2026-11-01, #23)]
Paused 2026-10-26 (owner). Branch task/T-0120-profile @ d75742a (pushed). Round 3 verifier: ACs 1–5 PASS, B1–B4 fixed, ~9,700 scenes 0 gouges; FAIL on one residual B5: with a separate last pass a finishing tab can be missing from the roughing cut, so roughing cuts through it at full depth with no diagnostic (repro scratchpad t0120v3/tabmin1.cpp). Proposed: re-scope B5 into a small follow-up card. Non-blocking: leads checked against r − tol not r + allowance; tab thickness > cut depth not clamped; zero-length plunge; tab length on curves.
Phase: 1   Module: core/toolpath   Depends on: T-0040 (offset API in main; T-0017 quality fixes land in parallel — owner decision 2026-10-26)   ADR: 0016
Goal: `profileToolpath(contours, Tool, ProfileParams) → Toolpath` per feature-inventory §5 "2D Profile Toolpath": Machine Vectors Outside/Inside/On (open vectors Left/Right), Climb/Conventional, Allowance Offset, Do Separate Last Pass (allowance, reverse), Use Vector Start Point, Tabs (length, thickness, constant number / constant distance, avoid corners; 3D tabs), Ramps (Smooth distance/angle, Zig-Zag, Spiral), Leads (straight / circular, lead-out, overcut), Order (vector selection / left-to-right / bottom-to-top / grid), Start At (keep / optimise / closest on bounding box), Corners (sharp external via V-bit when tool is V, sharp internal 3D). Multi-pass via PassPlan. Inner contours before outer. Deterministic.
Acceptance criteria:
  1. Outside profile of a 100×50 rectangle with a 6 mm end mill, 3 passes of 4 mm, 2 tabs 10×2 mm: move list matches a golden (hand-checked) toolpath exactly; tab regions rise to −(depth−2).
  2. Inside profile of a circle r=10 with a 6 mm tool is a circle r=7 (arc moves, not lines); r=2.9 yields no toolpath with a clear reason.
  3. Climb vs conventional reverses direction; separate last pass with 0.2 mm allowance produces two offset loops at the expected distances.
  4. Smooth ramp over 20 mm reaches full depth exactly at 20 mm along the path; spiral ramp on a closed contour descends by pass depth per revolution; leads start/end off the material and never gouge (min distance of lead to any other contour ≥ tool radius).
  5. Ordering modes produce the documented orders on a 3×3 grid of squares; inner-first holds for nested contours.
  6. Golden G-code through the Grbl post for the AC1 case; randomized property test: no feed move ever comes closer than tool radius − tol to the uncut side of any input contour.

### T-0121 Pocket toolpath (offset and raster, profile pass, ramp plunge)   [status: done (with T-0125, 2026-10-31)]
Paused 2026-10-26 (owner). Branch task/T-0121-pocket @ d432243 (pushed). Round 3 verifier: ACs 1–6 PASS, ~3,200 scenes 0 gouges, fold flags confirmed generator artefacts; FAIL on one residual: the Strict centre-region inset guard can drop a genuine lobe whose one vertex sits ~1e-6 past the slack → whole reachable area silently uncut (rare: ~1/4,000 diameters; repro scratchpad t0121v3/gap_poly.txt + gap4.cpp). Proposed: re-scope into a small follow-up card. To T-0123: raster pattern memory unbounded inside caps (bad_alloc at 1000-tooth comb), rest raster quadratic, rest tools re-trace link-cut scallops.
Phase: 1   Module: core/toolpath   Depends on: T-0040 (offset API in main; T-0017 quality fixes land in parallel — owner decision 2026-10-26)   ADR: 0017
Goal: `pocketToolpath(contours, tools, PocketParams)` per §5 "Pocketing Toolpath": Offset strategy (climb/conv, stepover, linking with minimal retracts) or Raster (angle, stepover, boundary pass), Profile Pass first/last/none, Pocket Allowance, Ramp Plunge, multi-tool clearance + rest tool (rest regions computed as the area the larger tool could not reach), islands (inner contours) respected.
Acceptance criteria:
  1. Pocketing a 50×30 rectangle with a 6 mm tool at 40% stepover: all material inside (offset by tool radius) is covered — rasterised coverage test at 0.1 mm resolution shows 100% coverage of the pocket area minus corners the tool cannot reach, and 0% outside.
  2. An island (20×10 inside the rectangle) stays uncut with ≥ tool radius clearance.
  3. Raster at 45° with a final profile pass: raster lines are parallel at the angle, clipped to the offset boundary, and the profile pass follows the boundary once.
  4. Rest machining: 12 mm clearance tool then 3 mm rest tool — the rest toolpath touches only regions within 6 mm of concave corners/narrow slots (checked against a rasterised difference).
  5. Linking: number of retracts in the offset strategy ≤ number of disconnected regions + 1 for simple shapes.
  6. Golden G-code through Grbl for AC1; gouge-free property test as in T-0120.

### T-0124 Profile: roughing tabs must match finishing tabs (residual of T-0120)   [status: done (2026-11-01, #23; conservative On guard, exact rule → T-0127)]
Phase: 1   Module: core/toolpath   Branch: task/T-0120-profile (continues d75742a)   ADR: 0016
Goal: Finish T-0120 so it can merge. With a separate last pass, a finishing tab can be missing from the roughing cut, so roughing cuts through it at full depth with no diagnostic (T-0120 round-3 verifier: 96/576 rectangle configs; 38/300 random scenes with separate last pass forced on).
Acceptance criteria:
  1. With a separate last pass, every finishing tab has a matching raised stretch on the roughing cut covering it, or both are dropped together and reported. No move of any pass goes below the tab top over finishing-tab material. Regressions: t0120v3/tabmin1.cpp (inside 100×50, 7 tabs 4×3, cutDepth 12, pass 6, allowance 0.1) and `MID=1 ./tabchk 9303 300 -1 1` → 0.
  2. TabsNotPlaced is consistent: kept + dropped = requested, per pass or per vector as documented.
  3. Everything T-0120 verified stays clean (ACs 1–6, B1–B4, ≥ 2,000-scene gouge sweep, golden unchanged).
  4. Non-blocking items from the round-3 report fixed where cheap: leads checked against r + allowance; tab thickness > cut depth clamped with a diagnostic; zero-length plunge removed; projected tab length on curves.

### T-0125 Pocket: centre-region guard must never drop a genuine region (residual of T-0121)   [status: done]
Phase: 1   Module: core/toolpath   Branch: task/T-0121-pocket (continues d432243)   ADR: 0017
Goal: Finish T-0121 so it can merge. The Strict inset guard rejects a whole centre-region lobe when one vertex sits ~1e-6 past the slack (Clipper grid + round-join sag), so a reachable part of the pocket is silently left uncut (~1 in 4,000 diameters).
Acceptance criteria:
  1. The guard tolerates Clipper's 1e-6 grid and join sag, or re-checks a dropped region by its deepest point so a region that clearly holds the tool is never discarded; phantom inside-out regions are still rejected. Regression: t0121v3/gap_poly.txt with d = 2.029 (and the 1.74–1.75 diameters) → lobe cut, coverage complete.
  2. Property test: across a diameter scan (≥ 4,000 diameters on several lobed shapes) no reachable region is dropped (independent oracle).
  3. Everything T-0121 verified stays clean (ACs 1–6, ≥ 2,000-scene sweep, caps, owner decision on Profile Pass).
  4. ADR §12 corrected: raster pattern memory is not bounded before the cap check (moved to T-0123).

### T-0021 intersect(): arc/arc cancellation with very different radii   [status: done]
Phase: 1   Module: core/geometry   Depends on: —
Goal: Found by the T-0017 coder: the general `intersect()` in core/src/geometry/intersect.cpp has a cancellation bug for arcs with very different radii (r = 39785 vs r = 0.09), reporting a false second crossing ~1e-5 mm from the real one (repro T-0017 seed 0xe400137f; offset junction exact at 50 digits). The coder had a fix + unit test, reverted as out of scope.
Acceptance criteria:
  1. Arc/arc and line/arc crossings computed without cancellation; regression for the reported pair and a generative test over radius ratios 1e-6…1e6 vs high-precision reference.
  2. No result worse than main on the T-0007/T-0008 regression families.

### T-0022 Arc angular slack and threshold crossings: distance-based tolerances   [status: done]
Phase: 1   Module: core/geometry   Depends on: T-0021
Goal: Found by the T-0021 verifier (both pre-existing on main). (a) `closestPoint(ArcSpan)` and the end-contact code accept query directions within `containsAngle`'s fixed 1e-9 rad slack, which is real distance on large arcs (1.26e-4 mm at r = 1.26e5, the full tolerance at r = 1e6): an end lying that far beyond an arc's end is treated as exact and reported untangent. Repro: arc centre (0,0), r = 125593, start 1.0, end 1.5, CCW; line from E − 0.5·(cos a, sin a) to E with a = 1.0 − 3.2e-10, E = r·(cos a, sin a) → one untangent hit, distance to arc 4.0e-5 mm. (b) When a chord 2h exceeds tol only at rounding level, `addPoint` merges the two crossings into one hit not flagged tangent, contrary to ADR-0009 (~25 of 179k threshold pairs).
Acceptance criteria:
  1. Angular slack is distance-based (≈ ε or tol/r scaled), so ends beyond an arc by more than rounding are end contacts flagged tangent; regression for the repro and a radius sweep 1e-3…1e6.
  2. Two crossings merged by the threshold rule are always flagged tangent; regression at 2h − tol ≈ ±1e-15.
  3. No result worse than main on the T-0007/T-0008/T-0021 families.

### T-0128 Open offsets: narrow loops and needles must not lose cut coverage   [status: in progress — round 2 verifying]
Owner decision 2026-11-05 (after round 1): isolated recovered stray pieces shorter than a small minimum (about 10× the tolerance, ~0.01 mm by default) are not returned; needles and slots that matter (mm long) are always kept.
Phase: 1   Module: core/geometry   Depends on: T-0017
Goal: Found by the T-0017 round-3 verifier (2026-11-01, non-blocking under "no worse than main"). T-0017 drops sub-tolerance lens loops and closed out-and-back needles as "narrow loops", and merges opposite copies within the snapping allowance. That follows the T-0017 header and AC3, but in rare cases it removes a toolpath that main keeps. (a) Seed 0xc5000544 (fam5, tieGap): an out-and-back needle 3.6 mm long and 5e-6 mm wide (x 6.43..10.08, y ≈ −5014.0939) has coincident ends, so it is closed and dropped; main keeps it. (b) Slot-width band: for a channel 2d + w wide, main keeps the needle for w ≥ ~3e-9, but T-0017 only keeps it above the snapping allowance (drops w up to 4e-7 at d = 0.1317, 1e-7 at d = 1, 3e-8 at d = 3.175). A DXF slot exactly one tool width wide plus coordinate noise can lose its path. Repro: a straight channel of width 2d + w and length 3.6, offset by d both sides (the verifier's vdrv6 `chan` driver was not preserved; seeds use the fuzz4 generators of `verifier-harness` t0017v2/gen5.inc). Also in scope: the reversal-cusp self-crossings (≤ 2.4e-8 mm; outlier 0xc500076c, a lens 2.1e-4 × 1.9e-9 mm) that contradict the header's "no returned chain intersects or touches itself".
Acceptance criteria:
  1. A degenerate loop or needle that a tool centre would still traverse (length ≫ tol) is returned as an open chain instead of being dropped; only loops whose whole extent is sub-tolerance are dropped. Regression for 0xc5000544 and a channel sweep w ∈ [0, 1e-5] at d = 0.1317, 1, 3.175: a path is returned for every w ≥ 0.
  2. No returned chain crosses itself at reversal cusps (regression for 0xc500076c and the cusp which=1 sweep), without reintroducing main's junction gaps.
  3. No result worse than T-0017 on its fuzz families (fam2–5, legacy, cusp sweeps).

### T-0126 Pocket guard hardening: edge checks after pull-in, area-weighted phantom test   [status: done (2026-11-01, #22)]
Phase: 1 (before Pocket ships to users)   Module: core/toolpath   Depends on: T-0121/T-0125 merged   ADR: 0017
Goal: Non-blocking findings of the T-0125 round-2 verifier (scratchpad t0125v2/adv.cpp raw-injection harness). (N1) After a convex vertex is pulled in or removed, only vertices are re-checked: removing a vertex on a concave join arc lets the new chord sag ~4× the arc step inside the limit (0.015 mm at chordal 0.01, 0.15 at 0.1, 0.71 at 0.5) — a gouge into exact walls. Never seen with real Clipper output, but a gap in the guard. (N2) The phantom test counts vertices, so a real 900 mm² region whose small bulge holds most vertices is rejected silently; a successful inset repair can lose 76 mm² at chordal 0.5 without a message.
Acceptance criteria:
  1. After pull-in or removal every new edge is distance-checked (exact segment-to-wall), falling back to the inset repair; regression from the raw-injection cases at chordal 0.01/0.1/0.5.
  2. Phantom classification is area- or depth-weighted; the C2/C4 cases are kept; a message whenever the repair inset exceeds ~chordal or loses area beyond a small bound.
  3. Everything T-0121/T-0125 verified stays clean (diameter scans, phantom probes, sweeps).

### T-0127 Profile On-vector tabs: exact classification and connectivity on sharp turns   [status: done (2026-11-02, #29; loose-piece reporting gaps → T-0135)]
Phase: 1   Module: core/toolpath   Depends on: T-0124   ADR: 0016 §14
Goal: T-0124 round-3 verifier (harness branch `verifier-harness`, t0124v4/): for profiles On the vector, a short tab on a sharp turn of the cut is kept although its own pass cuts it through (repro/on_corner.cpp; sweep scenes repro/sev_*.scn; F9 = seed 41006 fam 5 scene 9). T-0124 merges with a conservative guard that drops such tabs ("cut through"); this card replaces the guard with an exact rule so On tabs are kept whenever they really hold.
Acceptance criteria:
  1. On is decided by the profile side, not by `Cut::distance`; left/right material classification is correct near corners (allowance −, 0, +; with and without a separate last pass).
  2. Every kept On tab joins part and waste (independent raster oracle conn6/flood) over ≥ 1,500 new-seed scenes; tabs dropped by the T-0124 guard that would hold are kept.
  3. "kept + left out = requested" stated correctly (per loop when an offset splits) and consistent; thin necks < 0.1 mm either avoided or reported.
  4. Inside/Outside and tab-free toolpaths byte-identical to the T-0124 merge.
  5. Negative allowance (found by the T-0124 On-guard verifier, harness t0124v5/, pre-existing on c2f4709): with Inside/Outside and allowance between −r and about −r + 0.5 the tool centre runs close to the vector and tabs that are cut through are kept with no diagnostic (Outside, 30×30 square, 6 mm end mill, allowance −2.99/−2.9/−2.5, 4 tabs L=1 on the corners: whole-scene flood "part free"; scenes t0124v5/scn/neg2_s0_da*.scn, neg_s{0,1}_D6_da0_*_L1.scn). Tab survival is decided by the actual cut geometry for every side, so no kept tab is severed for any allowance ≥ −r; when r + allowance ≈ 0 the result is consistent (T-0124 changed Inside/Outside output there, driver t0124v5/tools/negallow.cpp).
  6. Short On tabs on straight edges hold (T-0124 guard drops every tab with length < 2|allowance| + max(0.25, 0.05·D), e.g. D 12, L 1, allowance ±0.3 on a straight edge; driver t0124v5/tools/straight.cpp); ADR-0016 §14 wording matches.
Note: conn6 reports false "severed" for some On tabs on straight edges and gentle curves; cross-check with the windowed flood t0124v5/wflood.py.

### T-0129 Profile: reject an allowance below −r   [status: done (2026-11-02, #31)]
Phase: 1   Module: core/toolpath   Depends on: T-0127   ADR: 0016
Goal: Owner decision 2026-11-01 (raised by the T-0127 coder): an allowance below −r (tool radius, at the cut depth for V-bits) moves the tool centre across the vector, so Outside silently behaves like Inside (and vice versa) and On/open Left/Right swap sides. Profile must reject such settings instead.
Acceptance criteria:
  1. `profileToolpath` refuses allowance < −r (and allowance + separate-last-pass allowance < −r) with a clear per-operation diagnostic naming the allowance and the limit; no moves are produced for the affected passes. Exactly −r stays valid (T-0127).
  2. Tests at −r, −r − 1e-9, −r − 1 for Inside/Outside/On/Left/Right, end mill and V-bit, with and without a separate last pass.
  3. ADR-0016 records the rule; output for allowance ≥ −r unchanged (byte-identical to the T-0127 merge).

### T-0131 Toolpath scaling and index residuals of T-0123   [status: done (2026-11-03, #47)]
Director decision 2026-11-03: the rest-tool case (e) is accepted at ~2.4–2.6× per doubling because the excess is in the Clipper2 boolean sweep (T-0142 item 3). Its test bound gets headroom until T-0142 lands.
Phase: 1   Module: core/toolpath   Depends on: T-0123   ADR: 0018
Goal: Found by the T-0123 coder and verifiers (harness t0123v1/, t0123v2/). (a) `BoxIndex` drops non-finite boxes when no box in the set is finite (constructor returns on `sizes.empty()` before `big_` is filled): `BoxIndex({Box2{{-1e308,0},{1e308,1}}}).query(Box2{{0,0},{1,1}})` returns {} (brute force {0}); same for an infinite box alone and `containing`. Driver t0123v2/bi3.cpp. (b) ADR-0017 §9 still names the removed `toolpath/pocket_ramp.hpp`; box_index.hpp comment omits the 1e-9 floor. (c) Raster zig-zag linking is quadratic (1,000-tooth comb within the caps: 6–14 s). (d) Profile with a separate last pass scales 4.2× per doubling (16 s → 69 s at 1,250 → 2,500 vectors): finishing-to-roughing tab matching compares every pair. (e) Pocket with a rest tool on a text page scales ~3× per doubling (2.3 / 7.5 / 22 s at 1,250 / 2,500 / 5,000 vectors; Clipper2 sweeps). (f) Tabs on 5,000 vectors take 17 s (linear but slow).
Acceptance criteria:
  1. (a) fixed with a one-box unit test vs brute force (finite, huge, infinite, NaN boxes).
  2. (b) docs corrected.
  3. (c)–(f): each ≤ 2.5× per doubling in Release CPU time up to 5,000 vectors (1,000 teeth for the comb), with absolute bounds recorded in the test; outputs byte-identical to the T-0123 merge (or a hand-checked, explained diff).

### T-0132 Native toolpath records in .carv (ADR-0003 job.json)   [status: done (2026-11-02, #27)]
Phase: 1   Module: core/document   Depends on: T-0122, T-0123   ADR: 0003, 0013, 0019
Goal: Replace T-0122's interim `toolpaths/NNNN.json` members (ADR-0019) with real toolpath records as ADR-0003 planned: the document holds toolpath definitions (name, operation, parameters, vector selection, tool snapshot) in `job.json`, with computed moves in a compact binary `toolpaths/NNNN.bin` cache that has its own budget, so large jobs (≥ 2M moves) save and reload. Owner decision 2026-11-01: needed before the app saves toolpaths.
Acceptance criteria:
  1. Document/Object model holds toolpath records; undo/redo and integrity checks cover them; format version bumped with a loader that still reads 1.0 files and migrates T-0122 interim members.
  2. A 2,000,000-move toolpath saves and reloads within the loader's resource budget and time limits (bounded, fuzzed like the rest of .carv I/O).
  3. carvetto-cli uses the native records; the CLI golden stays byte-identical.

### T-0133 Post safety: no G-code injection through text, finite numbers only   [status: done (2026-11-02, #28; remaining documented gap: template text after a value)]
Phase: 1   Module: core/post   Depends on: T-0030, T-0110
Goal: Found by the T-0122 verifier (harness t0122v1/, pre-existing on main). The post's text tags (toolpath name, tool name, notes …) only apply SUBSTITUTE, so a newline or other control character in a name becomes live G-code: a toolpath named "Evil\nG0 Z-50" emits the line "G0 Z-50)". Reachable today through tool-database names, and through toolpath names / crafted .carv members once T-0122 merges. Separately, the post prints non-finite or absurd numbers ("Z-inf", "Xinf", 300-digit Z from cut_depth 1e300).
Acceptance criteria:
  1. Every text value rendered by the post (all text tags, comments, headers) has control characters (U+0000–U+001F, U+007F, and the post's comment delimiters where the .pp defines them) replaced or removed, so no input can start a new block or close a comment early; regression tests for newline, CR, tab, NUL, ")" / ";" inside a name for every shipped post.
  2. The post refuses to emit a non-finite coordinate, feed, speed or dwell, and values beyond a sane machine range (document the limit, e.g. |coordinate| ≤ 1e6 mm), with a clear error instead of output; tests with ±inf, NaN, 1e300.
  3. Shipped post goldens unchanged (names in the fixtures have no control characters); posts/README safety checklist updated.
  4. (Owner decision 2026-11-02, after verifier round 1) Every text value is written as printable ASCII in every post: bytes ≥ 0x80 and Grbl realtime characters `~ ! ?` become '_' (an obvious ASCII letter allowed for Latin letters), because Grbl acts on them anywhere in the stream, even inside comments.
  5. (Owner decision 2026-11-02) LinuxCNC/Mach active comments are neutralised in the sanitiser: a text value at the start of a comment that begins with a word followed by a comma (LOGOPEN, LOGAPPEND, PROBEOPEN, MSG, DEBUG, PRINT, ABORT, py, …; case-insensitive, spaces ignored) has the comma replaced, so it stays plain text. Shipped goldens unchanged.

### T-0134 Post-safety and CLI residuals (T-0133 / T-0122 verifiers)   [status: done (2026-11-02, #32)]
Phase: 1   Module: core/post, app/cli   Depends on: T-0133, T-0122
Goal: Non-blocking findings (harness t0133v3/, t0122v1/). Post (custom .pp only, no shipped post exposed): (a) form feed / vertical tab / CR in template text between "(" and a value end the word run (commentPositionAt treats only ' ' and '\t' as spaces), so `"(\f[TOOLPATH_NAME])"` with "MSG,hi" stays active on LinuxCNC (isspace skip) — driver ff.cpp; (b) the shipped-post gap test only catches a template comma, not template keyword completions like `([TOOLPATH_NAME]OPEN)` (mut.cpp); (c) ADR-0020 sentence "no shipped post has template text between ( and a value" should read "after a value"; (d) [TOOLS_USED] at comment start now prints "(EndMill_ Drill)". CLI: (e) duplicate keys in params.json silently accepted (last wins); (f) number overflow (1e400) error lacks the "params file:" prefix; (g) `toolpath` prints "added …" before a failing final write; (h) a crafted .carv can carry zero/negative tool diameter, feeds, dwell (decoder accepts).
Acceptance criteria:
  1. (a) all whitespace/control characters in template text handled (treat isspace as space, or reject control characters in .pp lines at parse time) with tests; (b) a structural test: in every shipped post a text tag is followed only by ")", end of line or another tag; (c)/(d) docs and [TOOLS_USED] separator decided (e.g. "; " outside the word run) with goldens unchanged.
  2. (e)–(h) fixed with tests; CLI golden unchanged.
  3. (Owner decision 2026-11-02, raised by the T-0134 verifier) The post refuses spindle speed ≤ 0 (or non-finite) whenever it would write [S] or start the spindle, and refuses feeds, plunge rates, feed overrides and dwell that are > 0 but print as zero in the post's precision (F0, P0.00); clear errors, ADR-0020 updated, goldens unchanged.

### T-0135 Profile loose-piece reporting gaps and tab residuals (T-0127 round 3)   [status: done]
Phase: 1   Module: core/toolpath   Depends on: T-0127   ADR: 0016 §14
Goal: Found by the T-0127 round-3 verifier (harness t0127v3/; main reports no loose pieces at all, so these are gaps, not regressions). (a) The diagnostics de-duplication in run() merges loose-piece messages whose rounded areas are equal (symmetric parts): repro t0127v3/repro/h.scn (H shape, On, D6, a 0) → two free pieces 0.626/0.621 mm², one report. (b) addPiece treats bounding-box overlap as the same piece, so a piece inside another's concave notch is dropped: repro/abc_2_1_2.scn (B 374.7 mm² and C 64.4 mm² free, C unreported). Mutant inst/profile_mut.cpp FIX_MSG/FIX_BOX fixes both. Residuals: (c) In/Out with negative allowance and a separate last pass drops tabs as "could not be matched on the roughing cut" that main matched and that hold (repro/m1_402002_27.scn, m2_403003_37.scn); (d) conservative drops of 3D tabs with V-bits (repro/m2_403000_48.scn vs _flat.scn); (e) 0.06 mm slivers below the 0.05 mm raster (repro/m1_402012_7.scn); (f) 401007_36 keeps a redundant tab on overlapping vectors. (g) Found at the T-0127 merge: `fitTabs` (core/src/toolpath/tab_placement.cpp, shared, pre-existing) moves a tab centre to exactly f + reach and then tests |c − f| < reach, so a tab exactly `reach` from a keep-clear point is kept or dropped by rounding (scene m1_292000_22 flips with a 62-ulp change from T-0017); use a tolerance or skip the re-check for the point the tab was moved off.
Acceptance criteria:
  1. Every free piece ≥ 0.04 mm² (oracle wsf8 CNT) is reported, one message per piece (numbered or listing areas), over ≥ 2,000 new-seed scenes with leads off; held pieces never reported; regressions for h.scn and abc_2_1_2.scn.
  2. (c) tabs that hold are kept with a separate last pass (no class where main keeps holding tabs the branch drops); (d)/(e)/(f) improved or documented.
  3. No kept tab severed/partial/thin (whole-scene flood); AC4 byte identity of T-0127 preserved; perf within 10% of T-0127.

### T-0136 Native toolpath storage residuals (T-0132 / T-0134 verifiers)   [status: done (2026-11-02, #33)]
Phase: 1   Module: core/document, core/post   Depends on: T-0132   ADR: 0021, 0020
Goal: Non-blocking findings (harness t0132v3/, t0134v3/). (a) ADR-0021 §3 documents the save-time validation cost only at 2M moves; at the 8M cap the save peak grows by ~+0.5 GiB (second decoded copy during the verification load) — document it, or validate caches by streaming without holding a second copy. (b) Misleading message "cannot save more than 9999 toolpaths with computed moves" also when the problem is position (last computed record at position 10,000) and suggests 9,999 fit (the member limit allows 9,998). (c) Over-long names/strings are refused as a job.json "decompression bomb" ratio error; report "name too long" instead. (d) The interim counting pass uses a fresh probe budget, so the real pass may reserve up to min(S, 8M) moves for a member the shared budget then refuses (untouched pages on Linux; commit charge on Windows). (f) NaN/Infinity accepted in toolpath params on load (e.g. safe_z "NaN"; record != reloaded copy) — refuse non-finite params in the loader and commands. (e) A toolpath with zero moves counts as "no toolpaths" for the post, so the header still writes `M3 S0` (same as main; nothing cuts) — decide and document: skip empty toolpaths, or check spindle speed of the first non-empty toolpath for the header.
Acceptance criteria:
  1. (a)–(d) fixed or documented exactly, with tests where behaviour changes; save/load invariant preserved (fuzz from t0132v3).
  2. (e) behaviour documented in ADR-0020 and tested; goldens unchanged.

### T-0137 Profile On: refuse an allowance above +r too   [status: done (2026-11-04, #54)]
Phase: 1   Module: core/toolpath   Depends on: T-0129   ADR: 0016 §4
Goal: Owner decision 2026-11-02 (raised by the T-0129 verifier): for profiles On the vector an allowance above +r moves the tool entirely off the vector just as one below −r does; On profiles refuse |allowance| > r symmetrically (Inside/Outside keep only the −r limit: a large positive allowance there is a normal oversize cut).
Acceptance criteria:
  1. On (and open-vector Left/Right where the same geometry applies — decide and document) refuses allowance > +r (and allowance + last-pass allowance > +r) with the T-0129 diagnostic style; exactly +r valid; tests mirror T-0129's matrix.
  2. Output for |allowance| ≤ r byte-identical to the T-0129/T-0135 merge; ADR-0016 §4 updated; CLI message and exit code checked.

### T-0018 offsetContours: inward offsets past the inradius must vanish, not invert   [status: done (2026-11-02, #30; Clipper2 1.4.0 + backported fix)]
Phase: 1   Module: core/geometry   Depends on: —
Goal: Found by the T-0121 coder: a closed contour offset inward by more than its inradius comes back inside out instead of empty (a 17 mm circle shrunk by 19.2 mm returns a 2.2 mm circle; by 30 mm a 13 mm circle). Pocket guards against it locally (`insetBy`); every other caller of `offsetContours` is exposed. Also document (or fix) that Clipper drops pieces of an open path lying exactly on a clip boundary.
Acceptance criteria:
  1. Inward offset ≥ inradius of a region yields no contour (or only the genuinely surviving lobes of a multi-lobed shape), for circles, polygons, arcs/Beziers, with and without holes, all join types; generative test vs a brute-force inradius/medial check.
  2. No output contour has the opposite orientation of its source region unless it is a genuine hole; regression for the reported cases.
  3. Pocket's local guard can be removed or kept as defence; if kept, document why. Behaviour of open paths on clip boundaries documented in ADR-0006 (or fixed), with a test.

### T-0138 offsetContours: drop grid-level dust rings near collapse   [status: done (2026-11-03, #35)]
Phase: 1   Module: core/geometry   Depends on: T-0018   ADR: 0006
Goal: Found by the T-0018 verifier (harness t0018v1/dust.cpp; pre-existing, identical before the Clipper2 backport). Within ~2 µm of collapse, offsetContours can return 3-vertex rings ~0.3 µm long with |area| ~1e-11 mm², some wound against their nesting (annulus with Bezier outer at (32.939392279875733, −76.249769660635266), ro 10.338534503623912, CW hole ri 6.7009639741650435, inset by (ro−ri)/2 + 0.0015, Round, chordal 0.0002 → 8 dust rings, 2 reversed; circle r 21.723669301678942 inset by r − 0.0016 → a same-orientation dust ring nested in the genuine ring). This contradicts the offset.hpp contract sentence "a result contour wound against the outermost input is a genuine hole".
Acceptance criteria:
  1. Rings with |area| below a documented threshold (e.g. tol.distance²) or width below the grid are dropped from offsetContours results; repros clean; generative check (t0018v1/inr.cpp orientation-vs-nesting) shows 0 violations.
  2. No genuine region lost (coverage check in inr.cpp); Pocket/Profile outputs change only where dust rings were present; contract sentence true as written.

### T-0139 Storage message and test nits (T-0136 verifier)   [status: done (2026-11-03, #34)]
Phase: 1   Module: core/document   Depends on: T-0136   ADR: 0021, 0013
Goal: Non-blocking findings (harness t0136v1/). (a) Names that grow when stored (control characters escaped as \u0001, quotes, invalid UTF-8) are still blamed as a job.json ratio bomb, or reported with their in-memory size: measure the escaped/stored length in explainLongString (repro adv5.cpp: 200,000 × '\x01'). (b) The RSS test "T-0136 a: saveCarv's verification load holds no second copy" fails under ASan (41 vs 34 MiB): skip under __SANITIZE_ADDRESS__ / has_feature(address_sanitizer). (c) ADR-0021 §2 cites "ADR-0013 §1" for non-finite spellings (it is decision 3); cross-amend ADR-0013 A2 for the finite-params exception. (d) Awkward member-limit wording ("next to the 0 preserved members", "(0 toolpath move caches among them)").
Acceptance criteria:
  1. (a)–(d) fixed with tests where behaviour changes; save/load invariant preserved (fuzz3).

### T-0140 Profile: loose-piece check for Inside/Outside with allowance ≥ 0   [status: in progress]
Phase: 1   Module: core/toolpath   Depends on: T-0135   ADR: 0016 §14
Goal: Found by the T-0135 verifier (harness t0135v1/scn/m3_630018_3.scn): the loose-piece report runs only for sided cuts (On, allowance < 0), so with Inside/Outside and allowance ≥ 0 pieces that no kept tab holds come free unreported (15.1 mm² in that scene, leads off; same in T-0127). Extend the check to all tabbed profiles without changing toolpaths.
Acceptance criteria:
  1. Every free piece ≥ 0.04 mm² reported for Inside/Outside allowance ≥ 0 (whole-scene flood, ≥ 2,000 new-seed scenes, leads off); held pieces never reported.
  2. Moves byte-identical to the T-0135 merge (diagnostics may only gain loose-piece messages); perf within 10%.

### T-0141 Offset/Profile robustness residuals (T-0138 verifier)   [status: done (2026-11-03, #37)]
Phase: 1   Module: core/geometry, core/toolpath   Depends on: —   ADR: 0006, 0016
Goal: Pre-existing findings from the T-0138 verifier (harness t0138v1/, wt-T0138/build-verify3/h/). (a) A single row of 20,000 small rectangles takes ~45–50 s in offsetContours (same with no dust; likely Clipper2's sweep) — profile and fix or bound (col.cpp mode 1, rownd.cpp). (b) profileToolpath does not validate its Tolerance (Pocket requires 1e-4 ≤ chordal and 0 < distance ≤ chordal); decide and add the same validation with a clear diagnostic. (c) Clipper2 erosion of acute rings is not monotone (the T-0138 dust band is 3.6–5.6 grid units); decide whether a monotone dust test is wanted.
Acceptance criteria:
  1. (a) ≤ 2.5× per doubling up to 20k rings, or documented limit with a clear diagnostic; (b) Profile refuses invalid tolerances with tests, valid output byte-identical; (c) decision recorded in ADR-0006.

### T-0142 Boolean sweep cost and outward-offset leftovers (T-0141 coder)   [status: done (#59)]
Phase: 1   Module: core/geometry   Depends on: T-0141   ADR: 0006
Goal: Found by the T-0141 coder. (a) booleanOp/unionContours has the same O(n²) Clipper2 sweep as offsets had (a 10k row ~1.3 s); Pocket calls it on all of its input — apply the T-0141 batching (or equivalent) where results cannot interact. (b) A lone CW ring offset outward under a CCW lowest contour leaves hundreds of tiny leftover rings from Clipper2's corner joins (a 1 mm circle → ~920 at +0.3 mm; same on main) — find the cause and remove them without losing genuine regions.
Acceptance criteria:
  1. (a) ≤ 2.5× per doubling up to 20k shapes, output identical below the batching threshold and within a few grid units above it; (b) no leftover rings in the repro and a generative check; Pocket/Profile safety unchanged.
  2. (c) Remaining quadratic offset cases from the T-0141 verifier (harness t0141v1/perf.cpp): an outer containing a row of n holes (1.31/6.10 s at 10k/20k), a row whose outsets merge (1.44/5.99 s), the nested-corner spiral (~4.5× per doubling) — ≤ 2.5× per doubling or documented limit. (d) Profile: tol.distance has no lower bound and Bezier offsets use min(chordal, distance/2), so {distance 1e-8, chordal 1e-4} runs > 120 s and degrades output (bez.cpp) — add a lower bound on distance or a floor on the offset chordal (Profile and Pocket consistently), and fix the T-0141 test that accepts {1e-9, …}.
  3. (e) From the T-0131 verifier (harness t0131v1/clip90.cpp): Raster line clipping in booleanOp walks an active edge list holding every tooth. A 90° raster on a 1,000-tooth comb grows 3.5/5.2/6.6× per doubling (45°: 3.2×, 30°: 3.0×). Also covers Pocket's rest-tool whole-page boolean difference/intersection (~2.4–2.6× per doubling at 5,000 vectors; BuildIntersectList ×3.2). Target ≤ 2.5× per doubling; then tighten the T-0131 (c) and (e) test bounds back to 2.5 per doubling. Add a cheap guard for Pocket's nearestFirst (T-0131 fix): about 20k synthetic single-part groups, order equal to a brute-force nearest-first, ratio ≤ 2.5 per doubling (harness t0131v2/: reverting it is not caught by test e).

### T-0143 Point-in-contour degeneracy and drill pass cap (T-0150 verifier)   [status: done (2026-11-03, #40)]
Phase: 1   Module: core/geometry, core/toolpath   Depends on: T-0150   ADR: 0005/0009, 0022
Goal: (a) geometry::locatePoint/windingNumber return Inside (winding −1) at the centre of a ring drawn as one loop (outer arc, bridge, inner arc reversed, bridge back) when the bridge segments lie on the +X test ray and arc endpoints sit on it, so Drill drills the centre of such a ring although ADR-0022 says rings are not drilled (repro: harness t0150v1/ring.cpp — outer ArcSpan{{0,0},10,0,2π,CCW}, LineSpan (10,0)→(5,0), ArcSpan{{0,0},5,0,2π,CW}, LineSpan (5,0)→(10,0)). Fix the ray-crossing degeneracy for coincident bridge segments and endpoints on the ray. (b) Drill refuses exactly 10,000 passes although the message/ADR say "more than 10000" (check uses ceil(cut/pass)+1 ≤ 10000).
Acceptance criteria:
  1. (a) winding correct for bridges/endpoints on the test ray at every angle (generative test vs a robust oracle), all callers (Pocket, Profile, Drill) unchanged elsewhere; (b) limit consistent with message and ADR, boundary test.

### T-0154 No-spindle toolpaths for diamond-drag engraving   [status: done (2026-11-03, #49)]
Owner decision 2026-11-03 (after round 3 FAIL: "M/3", fractional M words, M259): one extra round with an allowlist: drag-section template lines may contain only listed words as plain integer words; no "/" except leading block delete, no controller real-time characters.
Owner decision 2026-11-03 (extra round FAIL: LINE_ENDING appended after the check, M0 message exemption on ;-posts, /-prefixed pause lines): ONE final round scoped to those fixes.
Owner decision 2026-11-03 (final round FAIL: Marlin M0 messages read as P/S timeouts, "M0 3" read as M03 on Grbl/LinuxCNC ;-posts, mach3 N-cap regression): drag-section M0/M1 lines are bare (optional N + M0/M1 + optional comment), shipped Marlin pause messages move to ';' comments, N allows up to 9 digits; narrow check, then merge.
Phase: 1 (Batch C)   Module: core/post, core/toolpath   Depends on: T-0151   ADR: 0020, 0024
Goal: Owner decision 2026-11-03: Quick Engrave refuses diamond-drag tools until the post supports toolpaths that run with the spindle off. A drag toolpath must skip SPINDLE_ON/[S], stop the spindle (M5) before it, and pause for the operator (tool-change style stop with a message) so the spindle is confirmed off/locked; T-0134's spindle > 0 rule exempts exactly these toolpaths.
Acceptance criteria:
  1. Diamond-drag Quick Engrave toolpaths post with spindle off, M5 + operator pause before them on every shipped post; no other toolpath can bypass the spindle rule (tests); posts/README checklist updated; goldens for existing toolpaths unchanged.

### T-0155 Engrave residuals (T-0151 verifier)   [status: done (2026-11-03, #42)]
Phase: 1 (Batch C)   Module: core/toolpath   Depends on: T-0151   ADR: 0024
Goal: (a) Fill's diagnostic rewording replaces "pocket" anywhere, including inside a quoted tool name ("my pocket cutter" → "my fill area cutter"; harness t0151v1/edge.cpp) — replace only Pocket's fixed phrases. (b) engraveLevels/PassPlan return fewer than `passes` levels when depth/passes ≤ 1e-9 mm (engraveLevels(1e-6, 1000) → 999; lv.cpp), contradicting ADR-0024 §1 — floor depth/passes or document. (c) Cosmetic: widths printed as 0.9999999999999999 mm in the stepover refusal.
Acceptance criteria:
  1. (a)–(c) fixed with tests; goldens unchanged.

### T-0144 GCC 15 Release build: -Wmaybe-uninitialized false positive in document.cpp   [status: done (2026-11-03, #41)]
Phase: 1   Module: core/document   Depends on: —   ADR: —
Goal: Every local Release build with GCC 15 (-O3 -Werror) fails on a -Wmaybe-uninitialized false positive in core/src/document/document.cpp (~283, Document::copyObjects; object.hpp ~131); every verifier works around it with -Wno-error=maybe-uninitialized. CI's compilers don't hit it, but a contributor on a current distro cannot build Release as documented.
Acceptance criteria:
  1. GCC 15 Release -Werror builds cleanly without disabling the warning globally (restructure the code or initialise explicitly; a targeted, commented pragma only if no clean fix exists); behaviour unchanged (full ctest); CI green.

### T-0145 Timing tests robust under machine load   [status: done (2026-11-03)]
Phase: 1   Module: core/tests   Depends on: —
Goal: Found by the T-0144 verifier: CPU-time ratio tests (e.g. toolpath_scale_test.cpp "AC3: thousands of small closed vectors pocket in near-linear time") measure n and 2n in separate windows (best of 3 each); on a loaded SMT machine a ~25% load swing between windows can push a true ×2.0 ratio past the ×2.5 bound (0 failures in 130 runs, but thin margins: Release 1.15–1.30 s vs < 2.0 s). Make ratio tests interleave the n and 2n runs (alternate, take per-pair minima) or otherwise robust to load swings, without weakening what they detect (quadratic stays caught).
Acceptance criteria:
  1. All CPU-time ratio tests use an interleaved/robust measurement helper in core/tests (shared with cpu_time.hpp); a deliberately quadratic mutant still fails each test; 0 flakes over ≥ 200 runs under ctest -j16 load.
  3. (Follow-ups from the T-0145 verifier) Tighten the T-0006 AC1 ratio bound from 3.5 to ~2.5 (linear measures 1.5–1.8; 3.5 misses a 3× quadratic term); add an assert/comment in cpu_scaling.hpp that small calls must be ≥ ~1 ms so the floor cannot swallow the ratio; replace the remaining std::clock absolute bounds (dxf_fuzz_test.cpp, carv_io_test.cpp) with processCpuSeconds.
  2. (Found by the T-0155 coder) PassPlan's absolute 1e-9 mm rounding guard drops one level at nanometre pass depths with maintainExactStep (cutDepth 1e-6, passDepth 1e-9 → 999 levels, not 1000; core/src/toolpath/pass_plan.cpp planStretch) — use a relative epsilon; level counts for realistic pass depths unchanged (Profile/Pocket/Drill/Engrave hashes vs main).

### T-0146 Changelog fragments (no more CHANGELOG.md conflicts)   [status: done (2026-11-03, #46)]
Phase: 1   Module: tools, docs, CI   Depends on: —
Goal: Owner decision 2026-11-03: every card currently adds a line at the top of CHANGELOG.md, so each merge makes every other open PR conflict and re-run CI (~40 min). Instead each card writes its entry to its own file `changelog.d/T-XXXX.md` (one bullet line, same style as today), never editing CHANGELOG.md; a script assembles the fragments into CHANGELOG.md's Unreleased section at release time and deletes them.
Acceptance criteria:
  1. `tools/changelog.py` (Python 3, stdlib only): `check` validates fragments (name T-XXXX.md or T-XXXX-YYYY.md matching a card id, exactly one non-empty bullet line, ASCII-safe like today's lines), `assemble` inserts them under "## Unreleased" newest-first by card number (deterministic), removes the fragments, idempotent; tests for both (pytest-free, run via ctest or a CI step).
  2. CI runs `tools/changelog.py check` on every PR and fails on a malformed fragment or on a PR that edits CHANGELOG.md's Unreleased section directly (allow the assemble commit itself, e.g. by a marker or by path rule — document).
  3. CONTRIBUTING.md, docs/WORKFLOW.md and .claude/agents/coder.md / verifier.md / director.md say: one fragment per task in changelog.d/, never edit CHANGELOG.md; the director's merge checklist ("exactly one CHANGELOG line per task") becomes "exactly one fragment per task". Existing CHANGELOG.md content unchanged.

### T-0147 signedArea cancellation for small contours far from the origin   [status: done (2026-11-03)]
Phase: 1   Module: core/geometry   Depends on: —   ADR: 0005
Goal: Found by the T-0152 verifier (harness t0152v2/drv/one.cpp, rect*.cpp): geometry signedArea (contour_ops.cpp ~52) evaluates the shoelace sum on absolute coordinates, so a small contour far from the origin cancels to 0 (1×0.5 mm at 1e8 mm: 17/20 placements give area 0; even 0.05×0.025 mm at 3e6). Callers that classify by area/orientation (nesting, region building, Pocket/Profile/Drill, medial axis) can then drop or misorient such contours.
Acceptance criteria:
  1. signedArea (and any orientation/area helper with the same pattern) computed relative to a local origin or compensated, exact to rounding for small contours up to 1e9 mm; generative test vs an exact oracle.
  2. Outputs of Pocket/Profile/Drill/offsets unchanged for normal inputs (hash comparison vs main), changes only where the old area was wrong (explained).

### T-0156 Profile loose pieces: V-bit corner moves, vanished finishing cuts, out-of-box pieces (T-0135 verifier)   [status: ready]
Phase: 1   Module: core/toolpath   Depends on: T-0135   ADR: 0016 §14
Goal: Pre-existing gaps found by the T-0135 round-3 verifier (harness t0135v3/, vfree.cpp exact oracle): (a) V-bit sharp-corner moves are not in the band model of the loose-piece search or the tab check — m2_1310149_15 leaves a 134 mm² part free with no message and a flood-PARTIAL kept tab (identical on T-0127/main); (b) when a finishing cut vanishes, its roughing loop is never searched (m0_1310036_24, 2.1 mm² slug); (c) pieces lying wholly outside every region box are not found ("#" of four bars, three circles with overlapping kerfs); (d) ADR §14 wording: band sampling allowance is up to ~11 µm for tab-layer radius < 0.0625 mm (not "at most 5 µm"), and bands are 2–7 µm wider than the cut (not "exactly"); (e) second-try tabs can have necks ≈ 0.086 mm (m1_1320038_46) — judge them with margin or document.
Acceptance criteria:
  1. (a)–(c) every free piece ≥ 0.04 mm² reported in the repro scenes and a ≥ 10k-scene vfree screen with V-bit corners on; no held piece reported; no kept tab severed/partial because of corner moves.
  2. (d)/(e) docs true as written or behaviour changed; holding tabs never fewer than the T-0135 merge.

### T-0157 Profile precision for small parts far from the origin   [status: ready]
Phase: 1   Module: core/toolpath, core/geometry   Depends on: T-0147
Goal: Found by the T-0147 verifier (harness t0147v1/far.cpp): with signedArea fixed, Profile still behaves differently from the origin for small parts at ≥ 1e7–1e8 mm (same-count 0–10/20; e.g. a circle at 3e8 gives "offset by 0.25 mm vanishes"); same on base — the cause is in offset/Profile precision (absolute coordinates in offset or clearance code), not area. Also: signedArea is now ~3–4× slower per arc span (sin/cos recomputed for junction terms; end-to-end unchanged) — cache per-span ends if cheap.
Acceptance criteria:
  1. Small Profile parts (circles, rectangles, holes; 0.5–10 mm) at 1e6–1e9 mm give the same toolpath as at the origin (up to translation, within tolerance) in ≥ 19/20 placements, or a documented limit with a diagnostic; normal-scale output unchanged (hashes).

### T-0158 Profile: platform-stable cut start at convex tips   [status: done (#63; known limits → T-0174)]
Owner decision 2026-11-03: adopt the fixed tie-break rule (start at the beginning of the corner arc centred on the tip), accepting that cut starts move in ~10% of such scenes. Owner decision 2026-11-05: mitred (sharp) convex tips start exactly at the mitre corner point. Owner 2026-11-05: a mitre squared off at the limit starts at the chamfer\'s first corner in the direction of travel; change rate ~9% stored / ~21% random pages accepted. Owner decisions 2026-11-05 (after round 3): one more round; on finely segmented convex outlines the cut starts at the tip's exact corner (mitre), not one segment off; on a merged region's polygonal round join (end mill) the cut starts at the first chord point, matching true round joins.
Owner decisions 2026-11-05 (after round 5): merge round 5 with the remaining cases documented as known limits in ADR-0016 §11 and fixed later in T-0174; self-intersecting outlines are best effort (documented, not guaranteed stable).
Phase: 1   Module: core/toolpath   Depends on: T-0135   ADR: 0016 §11
Goal: Found while merging T-0135 (MSVC CI): with "start at the vector's start point" on a convex tip, the Outside cut's round join is an arc centred on that tip, so every point of the arc is equally near the start; ArcLengthPath::project → closestPoint(ArcSpan) takes the direction of a ~1e-15 vector, which depends on rounding/libm, so GCC and MSVC choose different cut starts — tab layouts then differ (both safe; scene m1_402002_27, also m2_800016_13). Pre-existing on main/T-0127. A tie-break ("target within tolerance of an arc centre at the nearest distance → start at that arc's start", searching all arcs) was prototyped by the T-0135 coder (harness wt-T0135/build-drv pert.cpp, hsw/): stable under 1e-9 perturbations but changes start points in ~10% of scenes (owner-visible output change).
Acceptance criteria:
  1. Cut start deterministic across compilers and under ≤ 1e-9 relative perturbation for starts on convex tips (generative test with ±ulp scaling); ADR-0016 §11 documents the rule; goldens regenerated only if needed, with hand-checked diffs; tab safety unchanged (flood oracle).
  2. Restore exact tab-count assertions in the T-0135 test for m1_402002_27 once the start is stable.


### T-0159 GCC 15 / Ubuntu 26.04 Release test failures (CI runner change)   [status: done (2026-11-03, #50)]
Phase: 1   Module: core (tests)   Depends on: —   ADR: —
Goal: On 2026-11-03, GitHub's `ubuntu-latest` moved from 24.04 (GCC 13.3) to 26.04 (GCC 15.2). The Release linux-app job then failed on main (run 41704973044, same code as the green 4a5d1c8) and on PR #47 (run 41706009986, job 126161390264). The failures: "T-0147 AC1: the repro rectangle 1 x 0.5 mm at 1e8 mm keeps its area" (sanity check `naiveWrong >= 10` got 6: the naive shoelace comparison is compiler-dependent), "T-0135 round 2: exact loose pieces; add-back and fitting never cost a holding tab", "AC5 generative: span count follows the rule when nothing is consumed", and once "AC4: 10 000 inserts, undo all and redo all complete under one second" (timing). Linux jobs are pinned to ubuntu-24.04 meanwhile (task/T-0159-pin-runner).
Acceptance criteria:
  1. Each failure reproduced with GCC 15.2 Release (same flags as CI) and root-caused: a real product bug is fixed; a test that depends on compiler floating-point behaviour (contraction/FMA, excess precision, libm) is made robust without weakening what it guards.
  2. CI runs both ubuntu-24.04 and ubuntu-26.04 (or GCC 13 and 15) for the Release app job, both green; then unpin.

### T-0160 V-Carve corner sharpening   [status: done (#62)]
Owner decision 2026-11-05 (after round 1): slightly rounded corners count — any convex stretch the end mill cannot reach into (small fillets, short chamfers, few-segment polyline roundings, curved-edge junctions) is sharpened whenever its predicted leftover exceeds chordal, not only sharp junctions.
Phase: 1 (Batch C)   Module: core/toolpath   Depends on: T-0153, T-0164   ADR: 0025 §6 rewrite
Goal: Owner decisions 2026-11-05.
Code survey: corners are already sharp to tolerance everywhere except one place. With a flat depth AND clearance end mills, each convex corner of a flat area keeps a sloped floor "tent" between the wall-ring corner W and the end mill's fillet. Its peak is ≈ min(F, Rₑ(1−sin(α/2))/tan(θ/2)), about 0.88 mm for a 90° corner, a 6 mm end mill and a 90° V-bit. It is silent today (ADR-0025 §3).
Decisions:
  (1) Purpose: clean flat-floor corners only. Plain V-carve corner tips are unchanged; there is no effect without a flat depth plus clearance tools.
  (2) Thoroughness: match the floor finish. A V-bit tip groove along each corner bisector at the flat depth, plus passes either side at the carving tool's stepover, so the corner is no rougher than the rest of the floor.
  (3) Default on: a checkbox, job.json `corner_sharpen`, C++ `cornerSharpen`, UI "Corner Sharpen".
  (4) Threshold automatic: sharpen any corner whose leftover would be taller than chordal. There is no angle setting.
Acceptance criteria:
  1. Detection: convex corners of each flat area's region are found from the boundary (ArcLengthPath::kinks style), not from skeleton branch ends. A corner is sharpened exactly when its predicted leftover peak exceeds chordal.
  2. Moves:
     - The carving tool's tip runs at the flat depth along the bisector, from the wall-ring corner W into the clearance tools' swept area (overlap ≥ chordal).
     - Parallel passes run either side at the carving tool's stepover (clamped as for its rest clearing), within the tent.
     - Centres stay within the wall ring (≥ ρ+margin from the boundary), so the cone at F never crosses a wall.
     - The moves sit in that region's part of the carving toolpath, with safe links and post safety unchanged.
  3. Result:
     - The material left in each sharpened corner, measured with the exact oracle (t0153 vc3/acc.hpp style), is no higher than the floor ridge height elsewhere for the same stepover.
     - The cone never crosses a boundary.
     - Corners below the threshold are unchanged.
  4. Option: `VCarvingParams::cornerSharpen` (default true).
     - job.json `corner_sharpen` is encoded and decoded and added to kVCarvingKeys. A missing key decodes to true, so old files get the default.
     - The CLI param and its help text.
     - Format stays 1.1 unless ADR-0025 §10 requires otherwise.
  5. Output:
     - Without a flat depth, or without clearance tools, output is byte-identical to main.
     - Otherwise only the added corner moves differ.
     - Goldens are regenerated with hand-checked diffs and are identical on GCC 13, GCC 15 (FMA) and MSVC.
  6. ADR-0025 §6 is rewritten to this rule. The AC2 time bound holds. Tests ship with the code, plus a changelog fragment.

### T-0161 Drag-pause M5 lines and T-0154 leftovers   [status: done (2026-11-04, #52)]
Phase: 1   Module: core/post   Depends on: T-0154   ADR: 0020
Goal: Found by the T-0154 verifiers (harness t0154v7/postatk8.cpp G1–G3). On Grbl, a line that errors is dropped whole, so a pause block's M5 line that also carries other words can be lost: more than 79 characters (LINE_BUFFER_SIZE 80, error:overflow), 'M5 M6' (unsupported), 'M5 G0 G1' (modal-group violation). That needs a custom post; shipped posts are fine. Also: stale comments in interpreter.cpp (allowlistProblem "spaces or tabs", nonAsciiProblem "or tab", unreachable tab handling); the tab shows as a space in refusal messages; Grbl refuses N > 9,999,999 (MAX_LINE_NUMBER) while the allowlist allows 9 digits.
Acceptance criteria:
  1. In drag pause blocks the M5 line is bare like M0 (optional N + M5 + optional comment), and drag-section lines are at most 79 characters; shipped goldens unchanged.
  2. Comments corrected and the tab shown visibly in messages; N limited to ≤ 9,999,999 on Grbl-dialect posts, or documented.

### T-0162 Drag pause order and long name comments (T-0161 verifier)   [status: done (2026-11-04, #55)]
Phase: 1   Module: core/post   Depends on: T-0161   ADR: 0020
Goal: Found by the T-0161 verifier (harness t0161v1/atk9.cpp B2, README). (a) A pause block written "M0" then "M5" is accepted: the operator pause comes before the spindle stop (M5 still runs before the drag cuts). Enforce M5 before M0 in SPINDLE_OFF_PAUSE (and decide for SPINDLE_UNLOCK_PAUSE). (b) LinuxCNC counts the whole raw line, comments included, against LINELEN (254, NCE_COMMAND_TOO_LONG); shipped posts write "(Spindle stays off for diamond drag tool N: <name>)", so a ~300-character tool name aborts the program (pre-existing on main, every toolpath comment with names). Cap rendered names in comments (e.g. truncate to keep every written line ≤ 254 characters, or a lower documented limit). grblHAL may buffer comment characters (LINE_BUFFER_SIZE 257): check.
Acceptance criteria:
  1. (a) M0 before M5 in a pause block is refused, with tests; shipped goldens unchanged.
  2. (b) No written line exceeds the documented limit for any tool or toolpath name length; truncation visible and tested on all 8 posts; goldens unchanged for normal names.

### T-0163 Profile crash: lead-in planning on a closed walk with no spans   [status: done (2026-11-04, #56)]
Phase: 1   Module: core/toolpath   Depends on: —   ADR: 0016
Goal: Found by the T-0137 verifier (harness t0137v1/repro/seg_241_31.scn; tiny-scale page S=0.001, V-bit 30°, D=0.004): profileToolpath segfaults on main in planLeadIns — profile.cpp ~2507 runs `std::stable_sort(spans.begin() + 1, spans.end(), …)` when a closed walk has 0 spans (UB; ASan null read in the comparator). Fix the crash and audit the other walk/span loops for the empty case.
Acceptance criteria:
  1. The repro produces a valid result (or a clear diagnostic), never a crash; an ASan/UBSan run of the Profile generative sweeps (tiny scales, V-bits, leads on) is clean.
  2. Output otherwise byte-identical to main (hash sweep).
### T-0019 Make the T-0004 AC3 timing test CPU-time based   [status: done (by T-0017: T-0004 AC3 timing uses process CPU time, cpu_time.hpp)]
Phase: 1   Module: core/tests   Depends on: —
Goal: "T-0004 AC3" in `open_offset_chains_test.cpp` asserts a wall-clock bound (< 0.5 s) and fails intermittently under `ctest -j8` (seen by the T-0120 coder). Convert it to process CPU time (GetProcessTimes on Windows, std::clock elsewhere) or a complexity ratio, like the other timing tests. Coordinate with T-0017 if it lands first (T-0017 AC6 touches the timing helper).
Acceptance criteria:
  1. The test measures CPU time or a ratio; it passes 20× under `ctest -j8 --repeat until-fail:20`.

### T-0123 Toolpath API alignment: shared ramps, Tool → pass tool conversion, result types   [status: done (2026-11-01, #25)]
Phase: 1   Module: core/toolpath   Depends on: T-0120, T-0121
Goal: Profile (T-0120) and Pocket (T-0121) were built in parallel. Unify: one ramp module (Profile's `ramp_entry` vs Pocket's `pocket_ramp`), one result shape (`ProfileResult`/`PocketResult` with diagnostics), a `tooling::Tool` + `CuttingData` → pocket tool conversion (feeds, stepover, pass depth), and shared ordering/linking where it overlaps. Add a spatial index where vector-pair checks are O(n²) (Profile grouping/nesting/clearance; Pocket inset guard) so text-heavy jobs with thousands of vectors stay fast.
Acceptance criteria:
  1. One ramp implementation used by both; existing Profile and Pocket tests and goldens unchanged (or regenerated with a hand-checked diff explained).
  2. Both take `tooling::Tool` (+ cutting data) and return the same diagnostics type.
  3. 5,000 small closed vectors profile and pocket in < 2 s Release (CPU time), scaling ≤ 2.5× per doubling.
  4. Pocket rest tools build their pattern only over the rest areas (not the full pattern then clipped): the 200-tooth comb with a 0.1 mm rest tool (T-0121 verifier) runs ≥ 5× faster than at T-0121 merge.
  5. Raster and rest toolpaths route links through cleared area (as Offset does since T-0121 round 2), so raster pockets split by islands and far-apart rest areas retract only between disconnected regions.
Owner decisions 2026-11-01 (coder's open points): between separate rest areas/clusters the tool retracts to safe Z (no feeding across the top of the cut), even within one connected region; the Pocket tool radius is half the cutting width at the pocket bottom, the same rule as Profile (changes V-bit/ball-nose pockets). T-0123 takes ADR-0018; T-0122's interim-storage ADR is 0019.

Owner decisions 2026-10-31/11-01: run heavy verification one task at a time (machine capacity); T-0017 passes on "no worse than main" (findings identical to main become follow-up cards); T-0124 round-3 FAIL limited to On profiles → merge with a conservative On guard, exact fix in T-0127; director moved from the cloud session to Chris's local WSL machine (see docs/HANDOFF.md).

Owner decisions 2026-10-26 for T-0120/T-0121 (record in ADR-0016/0017): tab length = material left (raised path = length + tool diameter); separate last pass runs after all roughing; nested closed vectors form a region (inner = hole); climb assumes M3 (clockwise) spindle, ramps at feed rate, 3D tabs ramp at 45°, leads only on each cut's first entry and last exit, left-to-right ordering column by column — accepted as proposed; Pocket Profile Pass First/Last/None applies to the Offset strategy too (not only Raster); when a lead-in does not fit at the chosen start, Profile moves the start along the vector and warns (StartMoved).

### T-0122 Headless CLI: carvetto-cli   [status: done (2026-11-02, #26)]
Phase: 1   Module: app/cli   Depends on: T-0100, T-0110, T-0120, T-0121
Goal: A small console program `carvetto-cli` (no Qt): `import <dxf> → .carv`, `toolpath profile|pocket <params.json> <in.carv> <out.carv>`, `post <post.pp> <in.carv> <out.nc>`, `info`. Used by CI for the end-to-end test and by users for scripting.
Acceptance criteria:
  1. CI job: DXF fixture → profile with tabs + pocket → Grbl `.nc` whose checksum matches the golden; runs on Linux and Windows.
  2. Errors are clear and exit codes non-zero; `--help` documents every command.
Owner decisions 2026-11-01 (from the coder's BLOCKED report): (1) toolpaths are stored in the .carv as interim archive members `toolpaths/NNNN.json` (name, tool snapshot, moves, the params they came from) via two small public accessors in `carv_io.hpp` (read a preserved member; copy with a member added); an ADR marks this interim until a card implements ADR-0003's job.json toolpath records. (2) T-0123 merges first; the CLI is written against its API. (3) params.json: snake_case keys mirroring ProfileParams/PocketParams; lengths in mm, angles in degrees (converted at the CLI edge); tool = {"tool": {"database": <path, default data/tools/starter_tools.json>, "id": …}} plus optional material/machine for cutting data; optional "layers": [names] (default all vectors); unknown keys rejected with an error naming the key. nlohmann/json (already used) — no new dependency.
Owner decision 2026-11-01 (coder's open point): the interim JSON storage is capped by the loader's 2M-value budget for unknown content (ADR-0013 §9), ~300k–400k moves per file; accepted for now — the CLI reloads before writing and refuses with a clear error. The real fix is T-0132.

### T-0164 V-Carve follow-ups (T-0153 round-3 verifier, non-blocking)   [status: done (#57)]
Phase: 1   Module: core/toolpath   Depends on: T-0153   ADR: 0025
Goal: Non-blocking findings from harness t0153v3: (a) holes/slivers thinner than the distance tolerance are cut through, reported only as "crossing resolved"; (b) a D-hole with gap 0.0025 refuses the whole job (repro3/dhole_refused.txt); (c) cluster growth is quadratic in absorbing clusters (grow.cpp, ~3.2× per doubling); (d) the rest-tool stepover clamp to 2·limR leaves ridges up to 0.5–0.84 × the flat depth with no notice (repro3/arm_W*.txt, sq10.txt); (e) the AC1 sweep test skips the lateral cut-point check when flat depth > 0; (f) 9.5 nm cone violation at chordal 0.001 (repro2/nest_frame.txt). Safety-net hardening (T-0153 net verifier, harness t0153v6): (g) own rings are skipped on the assumption that the medial axis never silently drops an own boundary — measure own rings at branch points / ray-count seeds as a backstop (repro6/sq_hole_diag.txt); (h) membership hand-over at nodes assumes branch end == node point — add a distance check; (i) no unit test passes the `own` argument; (j) ADR-0025 §5 should say the 2c+d slack applies to every disc (full-island case carves up to 0.24 mm at chordal 1.0, within the bound).
Acceptance criteria:
  1. (a) and (d) are reported to the user (diagnostic) or documented in ADR-0025; (c) ≤ 2.5× per doubling; (e) the test checks laterals with a flat depth too; (b) and (f) fixed or documented with reasons.

### T-0165 Profile: collapsed cuts vanish silently; empty walks ordered at the origin (T-0163 coder)   [status: ready]
Phase: 1   Module: core/toolpath   Depends on: T-0163   ADR: 0016
Goal: Found by the T-0163 coder. (1) At tiny scale with chordal larger than the vector, a closed roughing offset collapses to one line, then to a point, and the cut disappears with no diagnostic (seg_241_31). (2) A cut whose walk is empty when order() runs (StartAt VectorStart / ClosestOnBoundingBox) enters ordering at (0,0) via pointAt on an empty walk, and adds a (0,0) keep-clear point to other cuts' tab placement. (3) The ASan test build fails under -Werror: carv_storage_residuals_test.cpp:232 peakGrowth unused.
Acceptance criteria:
  1. A cut that collapses reports a diagnostic (or is dropped before ordering with one); empty walks never enter ordering or tab keep-clear; regression tests.
  2. An ASan/UBSan build of the tests compiles with -Werror and passes.
  3. Output otherwise byte-identical to main (hash sweep).

### T-0166 Profile: platform-stable start for near-equal spans (symmetric shapes)   [status: needs owner decision]
Phase: 1   Module: core/toolpath   Depends on: T-0158   ADR: 0016 §11
Goal: Found by the T-0158 coder. After T-0158, GCC 13 and GCC 15 still choose different cut starts in 9 of 2,183 stored harness scenes (31 on main) and different tab-left counts in 6 (main 8). These are not arc-centre ties: with the bounding-box Centre anchor on symmetric shapes (rectangles, point-symmetric m2_1330009_35), two spans are equally near up to rounding, and "ties go to the lowest span" only works when the distances are bit-equal. A tolerance tie-break (lowest span within tol of the nearest distance) would change more outputs, so it needs the owner's approval first.
Acceptance criteria:
  1. Cut start and tab layout identical on GCC 13/15 (and MSVC CI) over the stored harness scenes; the rule documented in ADR-0016 §11; tab safety unchanged (flood oracle); output change rate reported to the owner.

### T-0167 Profile tabs: pre-existing weaknesses exposed by moved starts (T-0158 verifier)   [status: ready]
Phase: 1   Module: core/toolpath   Depends on: T-0158   ADR: 0016
Goal: Found by the T-0158 verifier (harness t0158v1, wsf.cpp flood oracle, worse.py). At starts moved by the T-0158 rule the unchanged tab code produces results main never does: m0_291002_11 (VectorStart/Outside) an unreported 0.41 mm² free piece; m0_271016_20 a severed tab (part still held); a4_770001_217 (box centre) a partial tab; random page r101_1_0_42 23 more free pieces. Overall unreported free pieces 720 → 733 across the changed runs. More from the T-0158 round-2 verifier (harness t0158v2 results/worse_attribution.txt, ulpflood_new.txt): m1_1310133_29 (VectorStart/Outside) a newly partial tab at (8.85, 8.60); m2_1330000_9 (V-bit, start at the mitre) both single tabs left out, a 1704 mm² piece comes free with no "comes free" message; m0_271013_1 and m0_271000_0 unreported free pieces. More repros from T-0158 rounds 3–4: harness t0158v3 repro3/mmrg_001_tabs_all_out.scn (V-bit merged region; all 3 tabs left out, a 1372 mm² piece comes free, reported); m1_272009_9 VectorStart Outside variant (merged round region; 7 of 10 tabs hold nothing).
Acceptance criteria:
  1. Each repro has no severed/partial tab and every free piece is reported or held; flood-oracle sweep over the harness shows no scene worse than main; regression tests.

### T-0168 Offsets: near-tolerance walks cost and inner-offset loss (T-0016 coder)   [status: ready]
Phase: 1   Module: core/geometry   Depends on: T-0016   ADR: 0012
Goal: Found by the T-0016 coder on main.
(a) Walks of 1.5 µm steps (just above tol) are super-linear for every join. Round: 4–7 s at 2000 steps, 17–43 s at 4000. Sharp: 12–13 s at 2000, 60–131 s at 4000. The whole walk lies within d of itself: pruning scans every input span per sample (closerThan) and boundary splitting is O(n²). Ideas exist on the shelved T-0009 branches (convex-hull distance hierarchy, domination trimming).
(b) An 8000 × 1.5 µm closed polygon circle (R 1.9 mm, d=1) loses its whole inner offset (0 chains): every inner piece is shorter than tol, so emit drops them all. 3 µm spans, or d=0.1, work.
Acceptance criteria:
  1. (a) ≤ 2.5× per doubling to 8000 steps, all joins; (b) the inner offset is present within tolerance; outputs elsewhere identical or judged no worse (T-0006 fuzz set, exact checker).

### T-0169 Pocket: tool-centre region dips below the tool radius at a curve/line junction (T-0164 coder)   [status: in progress]
Phase: 1   Module: core/toolpath   Depends on: —   ADR: 0017
Goal: Found by the T-0164 coder (harness t0153v3 repro2/nest_frame.txt, VC_FIX "0 6 60 0.001 0.3 0"). An edge of Pocket's tool-centre region dips below the tool radius where the curved part round the inner corner under an "A"'s crossbar joins the straight part along the crossbar. By exact distance this is 45 nm, 4.5% of a 0.001 chordal tolerance.
Acceptance criteria:
  1. Tool centres never come closer to the boundary than the tool radius + allowance − (documented numeric slack); regression test; Pocket output otherwise identical or explained.

### T-0170 Offset: Round/no-prune 13–15% slower after T-0016; super-linear growth on saw/hatch/arc inputs (T-0016 verifier)   [status: ready]
Phase: 1   Module: core/geometry   Depends on: T-0016   ADR: 0012
Goal: (a) After T-0016, Round offsets and inputs with nothing to prune are 13–15% slower than before on large inputs (zig-zag Round 1.118→1.261 s; filleted-arc zig-zag 2000 spans 8.67→9.81 s), even with every new path disabled (A/A-controlled), pointing to code generation in buildRaw / mergeOnCarriers / offsetOpenContour. Recover it (profile with symbols; e.g. keep pruneLineCopies out of line). (b) Saw / hatch / filleted-arc open offsets grow 2.6–4× per doubling for every join, Round included, on main as well (harness t0016v1 perf3.sh); the card case's first doubling is 2.7–2.8× for Round. Find the cause and bring it to ≤ 2.5× per doubling, or document it (may overlap T-0168).
Acceptance criteria:
  1. (a) Round on the t0016v1 perf inputs is within 3% of pre-T-0016 main (interleaved CPU, A/A control); (b) fixed to ≤ 2.5× per doubling or documented in ADR-0012 with measurements; outputs byte-identical; full ctest.

### T-0171 Hole nesting: guard the box hierarchy; ADR-0006 §10 wording (T-0142 round-3 verifier)   [status: done (#60)]
Phase: 1   Module: core/geometry   Depends on: T-0142   ADR: 0006
Goal: The T-0142 timing test lists each hole right after its outer, so the "last outer" shortcut always succeeds and nothing guards the box hierarchy: with it replaced by a linear scan (mutant C, harness t0142v3 results/mutC_po3.txt) every test passes while holes listed far from their outers (tcolrev, tcolshuf) grow 3.7–3.8× per doubling. Add a reversed/shuffled layout to the test. Also ADR-0006 §10 says "near-linear in the number of rings whatever the layout" just before listing the inputs that defeat it — reword.
Acceptance criteria:
  1. A timing test with holes listed far from their outers fails under mutant C and passes on the fix; ADR wording consistent; full ctest.

### T-0172 Open-offset stray recovery follow-ups (T-0128 round-2 verifier)   [status: ready]
Phase: 1   Module: core/geometry   Depends on: T-0128   ADR: 0012
Goal: Non-blocking findings from harness t0128v2. (a) A recovered stray with more than 512 spans is returned as one chain per span (the step-7 `part.size() > 512` fallback, commented "Never seen"); Profile makes each a separate Cut, so one slot gets hundreds of plunges, and spans ≤ tol are dropped there so a finely segmented wall can lose the path (repro split.cpp `./split 514 1 0 0`, bigloop.cpp `./bigloop 600 1 0 0 1 1`; main returns nothing). Chunk into runs of ≤ 512 spans instead, or remove the limit. (b) Clustered strays: one far feature stretches the stray grid — hatch at pitch 2d plus a far U channel costs 1.7–2.0× main, and 11.9× (quadratic, main quadratic too) at d = 0.002, far = 1e5 (`perf2 lhatch 1000 0.002 0 0 3 0 1e5`); `partCrossesItself` is O(n²) per stray (~3 ms per 500 spans). (c) Untested: a stray between tol and 10 tol touching a chain is returned (mutants M3, W2), the fallback size (S), the longer loop strand (P), merge order (O), exactOver in mergeOnCarriers (E2). (d) ADR-0012 A3 should mention (a) and (b).
Acceptance criteria:
  1. (a) a long stray comes back as ≤ ceil(n/512) chains with no spans dropped; (b) near-linear and ≤ 1.5× main on the clustered repros, or documented with measurements; (c) tests kill M3, W2, S, P, O (E2 if not redundant); outputs otherwise unchanged; full ctest.

### T-0173 V-Carve corner sharpening follow-ups (T-0160 round-2 verifier)   [status: ready]
Phase: 1   Module: core/toolpath   Depends on: T-0160   ADR: 0025
Goal: Non-blocking findings from harness t0160v2. (a) Super-linear: one region made of a horizontal chain of n squares joined by pinched necks — sharpening adds 0.27/0.83/2.6/11.8 s at n = 100/200/400/800 (×4.5 at the last doubling, +168% of the job); rotated 90° it is linear. The swept-area offset/difference runs over the whole part in one Clipper2 call (scanbeam); batch it per region/locality (repro perf2 chain 100 200 400 800). (b) Test gaps: mutants caught only by the golden (half-strip 0.3·s leaves 0.2683 > ridge 0.2598 in bigtri_ch0.002_apex30_V60s03; never_join; no back extension; no zig-zag) and not caught at all (ramps counted as swept; pieces < 50·distance dropped). (c) CLI help text still says "convex corners … each corner's bisector"; round 2 also cuts rounded corners, necks and spikes. (d) Pieces shorter than the distance tolerance are dropped — mention in ADR-0025 §6.
Note (owner question, pre-existing): flat areas no end mill enters keep F/2 ridges along every wall from the carving tool's rest clearing; ask the owner whether to address it.
Acceptance criteria:
  1. (a) ≤ 2.5× per doubling on the chain repro, output byte-identical; (b) the listed mutants are killed by corner tests; (c), (d) docs fixed; full ctest.

### T-0174 Profile start: degenerate start edge and region-arc band (T-0158 round-5 verifier)   [status: ready]
Phase: 1   Module: core/toolpath   Depends on: T-0158   ADR: 0016 §11
Goal: Known limits accepted at the T-0158 merge (harness t0158v5). (a) A squared-off tip whose start vertex has a duplicate or sub-µm edge next to it labels no span on that edge; the fallback's "most even feet" ties exactly between the chamfer's two corners and rounding picks one — the start jumps up to 4.3 mm under ulp/1e-9 (repro5/sq_tiny_tA2.5_e0_phi0_o0_d0_lim2.scn, sq_tiny_tA2.5_e1e-09_*; mitre limit 1: 0.45/1.86/3.28 mm at turns 0.3/1.2/2), and with a collinear 1e-7–1e-5 mm edge both directions start at the same chamfer corner (last in travel for one). Skip degenerate edges when finding the start vertex's own edges, and break any chamfer tie by first in travel. (b) Merged-region arc labelling band: arc chord sag on the chordal-slack band flips the start by one chord (repro5/mrgarc_vb_R20.771966844133722_d1.scn 1.67 mm, mrgarc_em_R18.966862802842932_d1.scn 0.36 mm). (c) Surviving mutants N1, N2, N3, N8, N11 — add tests. (d) ADR §11: describe the label-3 "from" rule. Self-intersecting outlines stay best effort (owner).
Acceptance criteria:
  1. (a) and (b) starts stable under ±ulp, 1e-12–1e-9 scaling and translations, chamfer starts at the first corner in travel in both directions; a test with a duplicate/degenerate start vertex; (c) mutants killed; no change elsewhere beyond the repros (change rate vs T-0158 reported); flood oracle no worse; full ctest.

### T-0150 Drilling toolpath   [status: done (2026-11-03, #36)]
Phase: 1 (Batch C, owner decision 2026-11-03: Batch C toolpaths first)   Module: core/toolpath   Depends on: T-0123, T-0132   ADR: new
Goal: Drill the centres of selected closed vectors (behaviour per docs/reference/feature-inventory.md "Drilling Toolpath"): drill or end-mill tool, start depth, cut depth, optional peck drilling (pass depth from the tool; retract above the cutting start by a retract gap, or above the previous pass), optional dwell at the bottom of each pass, ordering of holes, safe-Z links. Output through the existing post with plunge/retract moves (no canned cycles required); stored as a native toolpath record; CLI `toolpath drill`.
Acceptance criteria:
  1. `drillToolpath(contours, tooling::Tool + CuttingData, DrillParams)` returns moves and the shared ToolpathDiagnostic type: one hole per closed vector at its centre (define and document "centre": bounding-box centre vs area centroid — ADR), open vectors and vectors smaller than … reported; peck and dwell options; ordering (selection / nearest / left-to-right) reusing the shared cut-order helper.
  2. Safety: every rapid at safe Z; plunges at the plunge rate; peck retracts never below the start surface + retract gap; dwell > 0 only; tested against the T-0133/T-0134 post rules on every shipped post; golden G-code for a fixture.
  3. Native record round trip (params in job.json, ADR-0021 schema), CLI `toolpath drill <params.json>` with params.json keys documented in --help; CLI end-to-end test extended; MSVC CI green.

### T-0151 Quick Engrave toolpath   [status: done (2026-11-03, #38; diamond drag refused → T-0154)]
Phase: 1 (Batch C)   Module: core/toolpath   Depends on: T-0150   ADR: new
Goal: Engrave along vectors (behaviour per feature-inventory "Quick Engraving Toolpath"): depth to engrave, strategy Outline (follow the vectors at depth) or Fill (fill closed regions with a pattern and stepover), number of passes; diamond-drag tools follow the vector at the surface depth given. Nose-cone/pressure and "output direct to machine" are out of scope (no machine connection).
Acceptance criteria:
  1. Outline and Fill strategies with stepover; multi-pass; native record + CLI; goldens; safety rules as T-0150.
  2. Fill reuses the Pocket raster/offset machinery (T-0121/T-0123) — no duplicate fill code; gouge-free by the pocket oracle.

### T-0152 Medial axis of 2D regions (geometry core for V-carving)   [status: done (2026-11-03, #39)]
Phase: 1 (Batch C)   Module: core/geometry   Depends on: —   ADR: new (algorithm choice; clean-room — no proprietary code)
Goal: Compute the medial axis (inner Voronoi skeleton) of a closed region with holes made of lines, arcs and Beziers, with the inscribed-circle radius along it, within the chordal tolerance. This is the basis for V-carve depth = radius / tan(half angle).
Acceptance criteria:
  1. API returns skeleton edges (polyline/arc approximations) with radius at every vertex; regions with holes, sharp and rounded corners, thin necks, arcs and Beziers; radius error ≤ chordal tolerance against a brute-force distance oracle; skeleton points within tolerance of the true medial axis (equidistant from ≥ 2 boundary features).
  2. Robust on degenerate input (collinear, near-touching, tiny features, large coordinates), no crashes or hangs (fuzz); ≤ 2.5× per doubling in CPU time up to 50k boundary segments (text-heavy jobs).
  3. ADR records the algorithm (e.g. segment Voronoi via a library already allowed by the dependency policy, or an own implementation) and the dependency/licence decision; any new dependency goes to the owner first.

### T-0153 V-Carve toolpath   [status: done (2026-11-05, #51; core built with -ffp-contract=off on GCC/Clang)]
Owner decision 2026-11-03: corner sharpening split out to T-0160.
Owner decision 2026-11-03: identifiers `v_carve` (code, CLI command, job.json "operation"), UI label "V-Carve"; "vcarve" matches the VCarve trademark (CONTRIBUTING).
Phase: 1 (Batch C)   Module: core/toolpath   Depends on: T-0152, T-0123, T-0132   ADR: new
Goal: V-carve closed regions with a V-bit (behaviour per feature-inventory "V-Carve Toolpath"): depth from the medial axis radius and bit angle, start depth, optional flat depth (flat-bottom areas cleared with clearance tools, Offset or Raster strategy reusing Pocket), corner sharpening, ordering, safe links; native record + CLI.
Acceptance criteria:
  1. Every V-bit move stays within the region (the cone never crosses a boundary) — exact cone/wall check over a generative sweep; depth = radius / tan(θ/2) within tolerance; flat depth honoured with clearance pocketing; goldens; post safety rules.
  2. Text-heavy jobs (≥ 5,000 glyph-like regions) within a documented time bound.

### Batch B — drawing & editing (viewmodel + Qt), to be carded after Batch A is dispatched
- Drawing tools, snapping/guides, selection & node editing, transforms, align, group; boolean/shape ops UI (weld/subtract/overlap/trim/fillet/offset wrappers over T-0003/T-0004/T-0005), curve fit, join/close.
- Text (FreeType/HarfBuzz) T-0130, dimensions, measure, bitmaps/trace, layout (arrays, nesting).
- File open/save in the app (T-0015 done — unblocked), SVG/EPS/PDF I/O.

### Batch C — the rest of the toolpaths, simulation and output
- Drill, quick engrave, V-carve (medial axis) with clearance, fluting, chamfer, prism, inlay, VCarve inlay, photo V-carve, thread milling, texture, keep-out zones, templates, merge, array copy, tiling, double-sided.
- Material-removal simulation and time estimates; save-toolpaths dialog; job sheet.
- MVP acceptance run on a real machine (owner).
