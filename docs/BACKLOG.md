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

### T-0004 Open-contour offset returns every chain; performance for dense input   [status: in-progress]
Phase: 0   Module: core/geometry   Depends on: T-0003   ADR: 0012 (if needed)
Goal: (Also absorbs the T-0003 round-3 residual defect: with Sharp/Square joins, an offset that curls into a closed loop can return loop + tail where the tail crosses the chain start — verifier repros A/B in T-0003 notes; 8/6840 random cases.) `offsetOpenContour` currently returns only the longest chain when the offset splits (parts of the path within 2d of each other). For CAM this silently drops geometry. Return all chains in path order, and bring cost for 2000+ span inputs (polygonised Beziers) to well under a second.
Interfaces: owns the signature of `offsetOpenContour` (change return to `std::vector<Contour>`); update all call sites/tests.
Acceptance criteria:
  1. A hairpin polyline narrower than 2d offset on the inside returns ≥2 chains, each satisfying the T-0003 AC5 distance property, ordered along the input path.
  2. For fixed-seed random paths with full ±180° headings (spirals, U-turns), the union of returned chains covers every input point's offset that is ≥ d − tol from the whole input (no valid geometry dropped), and every returned point is ≥ d − tol from the input.
  3. A 2000-span polyline offset completes in < 0.5 s in Release on CI Linux.
  4. No returned chain is self-intersecting for any join type (split at T/X junctions into loop and tail). Regression repro A: lines (-6.4312190988360092,1.1428523003734297)→(-12.846939679779521,3.9348207104935331)→(-9.7434797348853266,-1.9260892800402662), then CW arc c(-17.535585621268488,-1.0001000247398364) r=7.846933811721553 a0=-0.11828212233321031 a1=-5.0521988806306926, then CW arc c(-17.021348114572877,0.45450829964057693) r=6.3041034287405493 a0=1.2309864265488941 a1=0.23263422704120162; offset d=2.5 Right with Sharp and Square.
Out of scope: closed-contour offsets (Clipper2 path), arc refitting.

### T-0005 Intersection residuals: 2-span closed contours, tolerance-boundary cases   [status: verifying]
Phase: 0   Module: core/geometry   Depends on: T-0002   ADR: amend 0009
Goal: Close the residual defects found in T-0002's third verification round.
Interfaces: `geometry/intersect.hpp` / `contour_ops.hpp` implementations; signatures unchanged.
Acceptance criteria:
  1. `selfIntersections` / `hasSelfIntersections` on a closed contour of exactly 2 spans ignores BOTH shared junctions (the inner one and the closing one). Repros, all must report no self-intersection: half-disc `LineSpan{(-5,0),(5,0)}` + `ArcSpan{(0,0),5,0,π,CCW}`; Bezier lens `{(0,0),(2,2),(4,2),(4,0)}`+`{(4,0),(4,-2),(2,-2),(0,0)}`; smooth 2-Bezier oval; line+Bezier D-shape. A 2-span closed contour that genuinely crosses itself (e.g. two Beziers forming a figure-eight away from the junctions) still reports the crossing. Also cover 1-span closed contours (full circle; self-looping Bezier).
  2. Arc–arc with centre distance d ≤ tol but radii differing by ≤ 2·tol, where the circles' closest approach is ≤ tol, follows the same "tangent within tol → one hit" rule as the general case (or the ADR states precisely why not). Regression: `fullCircle({644.61470300472024,365.33035654442392},181.1984687011373)` vs `fullCircle({644.61399749030329,365.33092243365013},181.1966002797156)`.
  3. Arc/Bezier and Bezier/Bezier pairs with two shallow crossings closer than ~2·tol give the same result in both argument orders; when two hits are merged into one, the merged hit carries `tangent=true`. Regression: `ArcSpan{{-3.2305423907849837,-8.7399422212084588},9.3248612306533616,-0.86772800461480459,-5.0533095891647033,CW}` vs `BezierSpan{{-14.222937128613282,-10.138318879522913},{-12.121499048115544,-10.250528577090371},{-11.910081639881342,-6.2911507142599774},{-14.011519720379081,-6.1789410166925203}}`. Symmetry fuzz on ≥9,000 AB/BB pairs: 0 asymmetries.
  4. Remove the "Known limitation (tracked as T-0005)" note from `intersect.hpp`.
Out of scope: Bezier overlap detection; Debug-build Bezier length performance.

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

### T-0011 IdGenerator overflow guard                                           [status: verifying]
Phase: 0   Module: core/document   Depends on: T-0010
Goal: The id generator wraps at 2^64 and can then hand out ids that collide with existing sheets/layers/objects (found while verifying T-0012). Make allocation fail loudly instead.
Acceptance criteria:
  1. Allocating past a documented ceiling (2^62) is impossible: the generator reports exhaustion (assert in Debug + an error/no-op result from the commands that allocate in Release), never wraps, never reuses.
  2. `Document::checkIntegrity()` flags any id ≥ the ceiling and a next-id ≤ any existing id.
  3. Regression: a document whose generator is near the ceiling cannot be driven into a duplicate id by any public command sequence (fixed-seed randomized test).
Out of scope: file I/O (T-0012 already rejects out-of-range ids on load).

### T-0012 Native `.carv` file I/O                                            [status: in-progress]
Phase: 0   Module: core/document   Depends on: T-0010   (ADR-0003 accepted)   ADR: 0013
Goal: Save/load the document to the `.carv` zip container (miniz, nlohmann/json pinned). Versioned manifest; forward-compatible unknown fields preserved.
Acceptance criteria:
  1. Save → load → save produces byte-identical `job.json` for a document with 3 layers, 50 vector objects (lines, arcs, Beziers), text params, and document variables.
  2. Loading a file with an unknown top-level JSON field keeps the field on re-save.
  3. Loading a file with a higher major version fails with a clear error, not a crash.
Out of scope: heightfields, toolpath caches (later phases).

### T-0013 Reject preset ids already in use; integrity check for cross-kind duplicates   [status: blocked]
Phase: 0   Module: core/document   Depends on: T-0011
Goal: Commands that accept a caller-supplied (preset) id currently accept one already used by another kind (e.g. an object taking a layer's id) or an id below the generator's next value (re-adding a removed id). Found while verifying T-0011.
Acceptance criteria:
  1. Every command accepting a preset id refuses (no-op, returns false) an id that is in use by ANY sheet, layer or object, or that is below the generator's next id.
  2. `Document::checkIntegrity()` reports an id used by more than one item across kinds (new IntegrityIssue kind).
  3. Regression: `o.id = ObjectId{d.defaultLayer().value}; d.execute(AddObjectCommand(o))` returns false and leaves the document unchanged; fixed-seed randomized test over all commands with preset ids finds no cross-kind duplicates.

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

## Phase 1 — 2D/2.5D CAM MVP (cards to be filled by the director as Phase 0 lands)

Planned task groups, in dependency order. Each becomes several T-01xx/T-02xx cards.
- Drawing tools: circle, ellipse, rectangle (corner types), polygon, star, polyline/Bezier, arc, freehand; interactive handles & quick keys.
- Snapping and guides; grid.
- Selection & node editing; move/scale/rotate/mirror/distort; align & space; group/ungroup.
- Boolean & shape ops: weld/subtract/overlap, boundary trim, interactive trim, fillets (normal/dog-bone/T-bone), extend, join/close variants, curve fit, offset, vector boundary.
- Text: FreeType/HarfBuzz outlines, text block, text-in-box, text on curve, kerning/arch handles, single-line fonts, convert to curves.
- Dimensions and measure.
- Bitmaps: import, trace (potrace + colour quantisation), edit, crop, fade; vector texture.
- Layout: array/circular/along-vector copy; nesting; plate production.
- Import/export: DXF (libdxfrw or own), SVG, EPS/AI (PostScript subset), PDF vector export.
- Toolpaths: 2D profile (tabs/ramps/leads/allowance/corners/order/start), pocket (offset/raster/multi-tool), drill (peck/dwell), quick engrave, V-carve/engrave (medial axis; flat depth; clearance), fluting, chamfer, prism, inlay (straight), VCarve inlay, photo V-carve, thread milling, texture (flat), keep-out zones, templates, merge, array copy, tiling, double-sided jobs, project-onto-3D stub.
- Simulation: heightfield stock, tool sweeps per type, quality levels, material/surface colour, animation; time estimation with acceleration.
- Output: save toolpaths (single/multi-file, ATC tool-number check, tiled), job sheet HTML, shipped posts (Grbl, Mach3, LinuxCNC, Marlin).
- MVP acceptance (PLAN §5): DXF → text on curve → pocket + profile with tabs → V-carve with clearance → preview → Grbl post → real cut.
