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

### T-0017 Open-offset quality II: residuals of T-0006   [status: in-progress]
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

### T-0009 Offsets of inputs with sub-tolerance spans   [status: blocked]
Phase: 1 (before Profile/Pocket ship)   Module: core/geometry   Depends on: T-0017
Goal: `flattenForOffset` drops spans ≤ `tol.distance`, so the distance/coverage guarantees of `offsetOpenContour` hold only for the flattened input. Next to a dropped span an offset point can be short of d by up to ~tol (9e-4 mm seen), a Round arc about the neighbouring vertex can be up to ~tol over d (8e-4 mm seen), and up to ~1 mm of valid offset can be missing (seed 0x60660ace). About 1 in 1000 such inputs; already in main before T-0006. Found by the T-0006 verifier and coder.
Acceptance criteria:
  1. Sub-tolerance spans are merged into a neighbour (path kept continuous) or their end points treated as vertices with full circles in pruning/coverage — coder's choice, recorded in ADR-0012.
  2. 0 distance, Round-over-d and coverage violations over ≥ 20k fuzzed inputs containing sub-tolerance spans (all joins, both sides, open and closed), judged by an exact checker.
  3. No result worse than main on inputs without sub-tolerance spans; T-0006 speed criteria still hold.
  4. The limitation note in offset.hpp and ADR-0012 A1 is removed or narrowed to what remains.

### T-0016 Sharp/Square offset cost on dense hatch-like vectors   [status: blocked]
Phase: 1 (alongside T-0120/T-0121)   Module: core/geometry   Depends on: T-0017
Goal: Scaling is linear, but on dense zig-zags with d ≫ pitch Sharp/Square joins are 10–20× slower than Round (1000-span zig-zag, pitch 0.02, amplitude 1, d=7: ~14 s Release vs 0.7 s Round). Hatch- and fill-like vectors will hit this in Profile/Pocket.
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

### T-0120 2D Profile toolpath   [status: in-progress]
Phase: 1   Module: core/toolpath   Depends on: T-0040 (offset API in main; T-0017 quality fixes land in parallel — owner decision 2026-10-26)   ADR: 0016
Goal: `profileToolpath(contours, Tool, ProfileParams) → Toolpath` per feature-inventory §5 "2D Profile Toolpath": Machine Vectors Outside/Inside/On (open vectors Left/Right), Climb/Conventional, Allowance Offset, Do Separate Last Pass (allowance, reverse), Use Vector Start Point, Tabs (length, thickness, constant number / constant distance, avoid corners; 3D tabs), Ramps (Smooth distance/angle, Zig-Zag, Spiral), Leads (straight / circular, lead-out, overcut), Order (vector selection / left-to-right / bottom-to-top / grid), Start At (keep / optimise / closest on bounding box), Corners (sharp external via V-bit when tool is V, sharp internal 3D). Multi-pass via PassPlan. Inner contours before outer. Deterministic.
Acceptance criteria:
  1. Outside profile of a 100×50 rectangle with a 6 mm end mill, 3 passes of 4 mm, 2 tabs 10×2 mm: move list matches a golden (hand-checked) toolpath exactly; tab regions rise to −(depth−2).
  2. Inside profile of a circle r=10 with a 6 mm tool is a circle r=7 (arc moves, not lines); r=2.9 yields no toolpath with a clear reason.
  3. Climb vs conventional reverses direction; separate last pass with 0.2 mm allowance produces two offset loops at the expected distances.
  4. Smooth ramp over 20 mm reaches full depth exactly at 20 mm along the path; spiral ramp on a closed contour descends by pass depth per revolution; leads start/end off the material and never gouge (min distance of lead to any other contour ≥ tool radius).
  5. Ordering modes produce the documented orders on a 3×3 grid of squares; inner-first holds for nested contours.
  6. Golden G-code through the Grbl post for the AC1 case; randomized property test: no feed move ever comes closer than tool radius − tol to the uncut side of any input contour.

### T-0121 Pocket toolpath (offset and raster, profile pass, ramp plunge)   [status: in-progress]
Phase: 1   Module: core/toolpath   Depends on: T-0040 (offset API in main; T-0017 quality fixes land in parallel — owner decision 2026-10-26)   ADR: 0017
Goal: `pocketToolpath(contours, tools, PocketParams)` per §5 "Pocketing Toolpath": Offset strategy (climb/conv, stepover, linking with minimal retracts) or Raster (angle, stepover, boundary pass), Profile Pass first/last/none, Pocket Allowance, Ramp Plunge, multi-tool clearance + rest tool (rest regions computed as the area the larger tool could not reach), islands (inner contours) respected.
Acceptance criteria:
  1. Pocketing a 50×30 rectangle with a 6 mm tool at 40% stepover: all material inside (offset by tool radius) is covered — rasterised coverage test at 0.1 mm resolution shows 100% coverage of the pocket area minus corners the tool cannot reach, and 0% outside.
  2. An island (20×10 inside the rectangle) stays uncut with ≥ tool radius clearance.
  3. Raster at 45° with a final profile pass: raster lines are parallel at the angle, clipped to the offset boundary, and the profile pass follows the boundary once.
  4. Rest machining: 12 mm clearance tool then 3 mm rest tool — the rest toolpath touches only regions within 6 mm of concave corners/narrow slots (checked against a rasterised difference).
  5. Linking: number of retracts in the offset strategy ≤ number of disconnected regions + 1 for simple shapes.
  6. Golden G-code through Grbl for AC1; gouge-free property test as in T-0120.

### T-0018 offsetContours: inward offsets past the inradius must vanish, not invert   [status: ready]
Phase: 1   Module: core/geometry   Depends on: —
Goal: Found by the T-0121 coder: a closed contour offset inward by more than its inradius comes back inside out instead of empty (a 17 mm circle shrunk by 19.2 mm returns a 2.2 mm circle; by 30 mm a 13 mm circle). Pocket guards against it locally (`insetBy`); every other caller of `offsetContours` is exposed. Also document (or fix) that Clipper drops pieces of an open path lying exactly on a clip boundary.
Acceptance criteria:
  1. Inward offset ≥ inradius of a region yields no contour (or only the genuinely surviving lobes of a multi-lobed shape), for circles, polygons, arcs/Beziers, with and without holes, all join types; generative test vs a brute-force inradius/medial check.
  2. No output contour has the opposite orientation of its source region unless it is a genuine hole; regression for the reported cases.
  3. Pocket's local guard can be removed or kept as defence; if kept, document why. Behaviour of open paths on clip boundaries documented in ADR-0006 (or fixed), with a test.

### T-0019 Make the T-0004 AC3 timing test CPU-time based   [status: ready]
Phase: 1   Module: core/tests   Depends on: —
Goal: "T-0004 AC3" in `open_offset_chains_test.cpp` asserts a wall-clock bound (< 0.5 s) and fails intermittently under `ctest -j8` (seen by the T-0120 coder). Convert it to process CPU time (GetProcessTimes on Windows, std::clock elsewhere) or a complexity ratio, like the other timing tests. Coordinate with T-0017 if it lands first (T-0017 AC6 touches the timing helper).
Acceptance criteria:
  1. The test measures CPU time or a ratio; it passes 20× under `ctest -j8 --repeat until-fail:20`.

### T-0123 Toolpath API alignment: shared ramps, Tool → pass tool conversion, result types   [status: blocked]
Phase: 1   Module: core/toolpath   Depends on: T-0120, T-0121
Goal: Profile (T-0120) and Pocket (T-0121) were built in parallel. Unify: one ramp module (Profile's `ramp_entry` vs Pocket's `pocket_ramp`), one result shape (`ProfileResult`/`PocketResult` with diagnostics), a `tooling::Tool` + `CuttingData` → pocket tool conversion (feeds, stepover, pass depth), and shared ordering/linking where it overlaps. Add a spatial index where vector-pair checks are O(n²) (Profile grouping/nesting/clearance; Pocket inset guard) so text-heavy jobs with thousands of vectors stay fast.
Acceptance criteria:
  1. One ramp implementation used by both; existing Profile and Pocket tests and goldens unchanged (or regenerated with a hand-checked diff explained).
  2. Both take `tooling::Tool` (+ cutting data) and return the same diagnostics type.
  3. 5,000 small closed vectors profile and pocket in < 2 s Release (CPU time), scaling ≤ 2.5× per doubling.

Owner decisions 2026-10-26 for T-0120/T-0121 (record in ADR-0016/0017): tab length = material left (raised path = length + tool diameter); separate last pass runs after all roughing; nested closed vectors form a region (inner = hole); climb assumes M3 (clockwise) spindle, ramps at feed rate, 3D tabs ramp at 45°, leads only on each cut's first entry and last exit, left-to-right ordering column by column — accepted as proposed; Pocket Profile Pass First/Last/None applies to the Offset strategy too (not only Raster).

### T-0122 Headless CLI: carvetto-cli   [status: blocked]
Phase: 1   Module: app/cli   Depends on: T-0100, T-0110, T-0120, T-0121
Goal: A small console program `carvetto-cli` (no Qt): `import <dxf> → .carv`, `toolpath profile|pocket <params.json> <in.carv> <out.carv>`, `post <post.pp> <in.carv> <out.nc>`, `info`. Used by CI for the end-to-end test and by users for scripting.
Acceptance criteria:
  1. CI job: DXF fixture → profile with tabs + pocket → Grbl `.nc` whose checksum matches the golden; runs on Linux and Windows.
  2. Errors are clear and exit codes non-zero; `--help` documents every command.

### Batch B — drawing & editing (viewmodel + Qt), to be carded after Batch A is dispatched
- Drawing tools, snapping/guides, selection & node editing, transforms, align, group; boolean/shape ops UI (weld/subtract/overlap/trim/fillet/offset wrappers over T-0003/T-0004/T-0005), curve fit, join/close.
- Text (FreeType/HarfBuzz) T-0130, dimensions, measure, bitmaps/trace, layout (arrays, nesting).
- File open/save in the app (T-0015 done — unblocked), SVG/EPS/PDF I/O.

### Batch C — the rest of the toolpaths, simulation and output
- Drill, quick engrave, V-carve (medial axis) with clearance, fluting, chamfer, prism, inlay, VCarve inlay, photo V-carve, thread milling, texture, keep-out zones, templates, merge, array copy, tiling, double-sided.
- Material-removal simulation and time estimates; save-toolpaths dialog; job sheet.
- MVP acceptance run on a real machine (owner).
