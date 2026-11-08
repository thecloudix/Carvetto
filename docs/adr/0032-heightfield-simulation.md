# ADR-0032: Material-removal simulation (heightfield, tool shapes, tolerance, quality, determinism)

Date: 2026-11-07 · Status: proposed (T-0222)

## Context
T-0222 adds the `simulate` module of ARCHITECTURE.md: cut a stock block along the native move
lists (ADR-0008) and give the result to the preview (T-0223). Requirements: exact along every move
(not sampled tool positions), quality levels, per-cell cut state and toolpath id, incremental
playback by move and by time, collision-type diagnostics, bounded memory and CPU, and a result that
is byte-identical on every platform. Pure core (no Qt), mm and radians.

## Decision

### 1. Model
The stock is a rectangular slab; its top face is at `Stock::topZ` (0 for "Z zero at the material
surface", the usual job setting; `thickness` for Z zero at the machine bed), its bottom (the
table/clamp) `thickness` below. The surface is a float heightfield on a regular grid; the value of
a cell is the surface height **at the cell centre** (a point sample, not an average). A tool
removes every point of the stock above its cutting surface, which is the tool profile h(r) (height
above the tip at radius r) placed at the tip. Cells only ever go down (min of heights), so passes
combine by min and the order of passes affects neither height nor cut state, only the pass id (the
first toolpath to reach the final height keeps its id; ties are decided by strict `<`). Heights are
clamped at the table: the material is gone but the table remains (the move is flagged, §6).
Overhangs cannot be represented (2.5D), which is all Phase 1 toolpaths need.

### 2. Exact sweep of a segment
For one cell centre at distance q from the line of a segment and u along it, with the tip moving
along s in [0, L] at Z = z0 + g s, the lowest surface height is
`z0 + g u + min over w of [ g w + h(sqrt(q^2 + w^2)) ]`, w = s - u. All profiles are convex and
non-decreasing in r (flat, sphere cap, cone pieces; tapered ball nose is sphere then tangent cone),
so the bracket is convex in w and has a closed-form minimum per profile piece: flat -> an end of
the interval by the sign of g; cone (slope k) -> `w = -g q / sqrt(k^2 - g^2)` when |g| < k, else an
end; sphere (radius b) -> `w = -g sqrt((b^2 - q^2) / (1 + g^2))`. The minimiser is clamped to the
feasible interval (segment ends, and the range of w where r is inside the piece). No tool position
is sampled: the value is exact (up to double rounding) for any slope, including plunges (a
vertical move is a stamp at the lower end) and ramps steeper than the cone.

### 3. Arcs and helices
Planar arcs (equal start and end Z) are exact: the lowest point of the profile over the arc is at
the point of the arc nearest the cell centre, `h(distance to arc)` - the radial distance when the
cell is inside the arc's angular wedge, else the distance to the nearer end point; the wedge test
is algebraic (cross products). Arcs are split algebraically (quarter turns, then bisection by
normalised sum, pieces pruned when their bounding box misses the stock), so the work is bounded by
the stock, not by the arc's radius. **Helices** (Z changes) have no closed form for ball and V
profiles; they are swept as the chords of an inscribed polygon with sagitta <= cellSize / 64, each
chord exactly (§2). The path error is at most cellSize / 64 horizontally. The height error is slope(h) x cellSize / 64
only away from steep tool walls; at the rim of a flat end mill or any steep wall a cell whose centre
lies within that distance of the edge can flip by up to the full depth (9 mm observed on a 0.078 mm
cell). Direction, full circles (coincident start
and end in XY, tested exactly) and zero-radius arcs follow ADR-0008; the radius is taken from the
start, the end direction from the end point.

### 4. Tool shapes (ADR-0010)
`ToolShape` has factories for a flat end mill (and diamond drag), ball nose, V-bit (also engraving
tools with a flat tip, and drill points), and tapered ball nose (ball tangent to the cone, as
`tooling::cuttingWidthAtDepth` -- a test inverts the profile and compares widths). Radiused end
mills and form tools are refused (`fromTool` returns nullopt). The flute length is a property of
the shape (the database tool has none); infinity = unlimited.

### 5. Quality, cells, memory
`Quality` is a cell size: coarse 0.5 mm, medium 0.25 mm, fine 0.125 mm (feature-inventory
"Preview Simulation Quality": each level is finer). `Stock::cellSize` is the free parameter, and
`fitCellSize` returns a coarser size that fits a cap. Memory is 5 bytes per cell (float height +
cut flag), 9 with pass ids; the cell count is limited by `Stock::maxCells` (default 50 million,
never more than 400 million). A stock over the cap is refused with `TooManyCells` and a message
naming the count; nothing is allocated. Dimensions are limited to 1e7 mm and counts are computed in
double before any integer conversion. `maxCells` limits the cell COUNT, not memory: 50 million cells
(the default) are 250 MB, 450 MB with pass ids; the hard cap of 400 million cells is 3.6 GB and
cannot fit a 32-bit process (a byte count that overflows `size_t` is refused). The allocation is
guarded: `std::bad_alloc` is caught and reported as `OutOfMemory` (constructor and `addToolpath`;
a failed queueing is rolled back), never thrown.

### 6. Diagnostics
`RapidInMaterial`: a rapid whose path (swept by the tool, not cutting) would remove a cell by more
than `collisionTolerance` (1 um), i.e. it passes through material that has not been cut; rapids at
or above the top surface and rapids along an earlier cut are fine. (The card also says "feed
moves": a feed is ordinary cutting, so only rapids can be "in material outside a previous cut";
this is the conservative reading.) `BelowTable`: a move whose tip goes below the table by more
than the tolerance (rapids included). `ToolTooDeep`: the shank (a cylinder at the flute length) is
swept over the heights **before** the move; if it would remove anything, material stands above the
flutes' top within the tool's reach (a plunge deeper than the flutes in uncut stock is flagged; in
a pocket cleared before it is not). `ToolChangeWithoutRetract`: a toolpath with another tool
(shape or tool number) starts while the previous end point is inside the stock (below the top). A tip exactly
`fluteLength` below the surface is not flagged (the flute just reaches the surface; pinned by a
test). The thresholds are `collisionTolerance` (1 um) in both directions of each test. Diagnostics
are a pure function of the job and the move index: the shank probe of a move is made on the heights
before the move's first (partial) cut and remembered, so scrubbing with `advanceToTime` and seeking
in any order gives the same list as a one-shot run (tested with random call orders). At
most `maxDiagnostics` (10,000) are kept; the rest are counted.

### 7. Determinism
The path from input to a float height uses only + - * / and IEEE `sqrt`; the core is compiled
with `-ffp-contract=off` (no fused multiply-add on GCC/Clang) and, on MSVC, with `/fp:precise` set
explicitly in core/CMakeLists.txt (never contracts on the SSE2 baseline).
`sin`, `cos`, `tan` and `atan2` differ between libm implementations in the last bit, so the cone
slope, the tapered-ball tangent point and the angle fractions of helices and of partly played arcs
use the series in `det_math.hpp` (fixed term counts, accuracy ~1e-16). No decision depends on
unordered containers; cell order is row-major; ties use strict `<` and fixed interval order.
Heights are rounded to float once. A golden FNV-1a hash of three fixed jobs is asserted at -O0,
-O2 and on MSVC.

### 8. Cost model and limits
Per move the work is the number of cells in the swept footprint clipped to the stock (a capsule of
the path widened by the tool radius, found row by row; arcs by pieces of at most a chord of
max(2 r, 4 cells)), plus O(depth) for arc splitting; a move entirely above the top costs O(1). The
total is near-linear in (moves x footprint cells). The simulator counts evaluated cells and stops
with `WorkBudgetExceeded` beyond `maxCellWork` (default 1e10, minutes of CPU) so that no input can
spin it forever. All moves are validated when queued: coordinates must be finite and within 1e7 mm; an arc whose end point's
distance from the centre differs from the start radius by more than max(0.005 mm, 1e-6 x start
radius) is refused (`InvalidMove`: controllers reject such arcs, the generators' records match to
floating-point precision; within the tolerance the arc is swept on the start radius);
arcs of zero radius that go somewhere or that end at their centre are refused (`InvalidMove`, with
the global move index; nothing of that toolpath is queued); at most `maxMoves` (8 million, about 80 bytes each) moves.
A move only touches the cells the tool can reach at its depth (`ToolShape::reachAtGap`: a V-bit or
ball 1 mm below the surface reaches a fraction of its radius), which is what makes the 3.3 million
moves of a 5,000-glyph V-carve cheap. Measured (GCC 15, -O2, shared machine): 1,000,000 short moves
(ball nose 3 mm) on 2000 x 2000 cells 0.54 s CPU; the 5,000-glyph V-carve record of T-0153 (3.33 M
moves, 5.2 M cells at medium quality) 1.9 s; doubling 20,000 -> 40,000 moves ratio ~1.5. Asserted in
Release (an order of magnitude of slack for slow CI machines): <= 10 s each, doubling the moves
<= 2.5x (also in Debug, on smaller inputs).

### 9. Playback
Moves of all queued toolpaths form one list. `advanceToMove(n)` / `advanceToTime(t)` continue from
the current state (cheap), never recompute. Time per move comes from `toolpath::moveTime` with the
toolpath's feed and plunge rates and `SimOptions::rapidRate`/`acceleration`. At a time inside a
move, a prefix of its path proportional to the elapsed fraction is cut (constant speed inside a
move; acceleration is ignored within one move); the next advance cuts the whole move again (min is
idempotent). `seekMove`/`seekTime` rewind by resetting and replaying. `advanceToMove`, `seekMove` and planar-arc time playback give, at move N, exactly the heights of a
fresh run to N, byte for byte (tested). A **partial helix prefix** under `advanceToTime` is not equal
to the exact prefix: it is subdivided into different chords, and the min of heights keeps the deeper
result; measured up to 7e-3 mm at slope 0.8 with 0.25 mm cells. This is the documented
partial-playback error; the next completed move restores the exact state. Move end times come from
`toolpath::moveLength` (libm trigonometry), so the position of a partial prefix is not bit-exact
across platforms; the golden heightfield (whole moves) is. **For T-0223:** scrub with
`advanceToMove`/`seekMove` (byte-exact); use `advanceToTime` only for animation.

### 10. Accuracy statement
Point-sampled cells: the value at each cell centre is exact for segments and planar arcs; helices
are within sagitta (§3). Between cells nothing is claimed: a cell whose centre lies within a
rounding error of the edge of the swept region can fall either way. The tests compare against
brute-force sampled sweeps (oracle bounds rigorous for tool positions every cell/8 .. cell/24) and
closed-form distances; deviations are reported in the T-0222 report.

## Consequences
- T-0223 renders `Heightfield` (heights, cut state, ids) and drives playback.
- Radiused end mills and form tools need profile pieces (a torus piece / a table); the
  convexity requirement of §2 must be kept or a different minimiser used.
- Overhangs (2.5D limit) are out of scope; Phase 2 3D relief is a different component.

## Amendment (T-0223, preview and simulation view)
- **Files** (`app/viewmodel`, Qt-free): `toolpath_preview` (scene, draw list), `time_readout`, `simulation_setup`,
  `simulation_controller`, `relief_render`, `playback`. Qt glue (`app/src`): `preview_panel` (the Preview dock),
  `relief_view` (the simulation view in the "3D View" tab), the overlay in `canvas_widget`.
- **2D preview.** Every enabled, computed toolpath becomes polylines in document coordinates, cut into chunks of at most
  256 points (arcs by chord error 0.01 mm, at most 4096 points each), with a class per run of moves: Rapid, Feed (arcs
  included), Plunge, Retract, Ramp (a feed or arc whose Z changes while it travels in XY: ramps and helices; the move
  list has no "lead" kind, so lead-ins are Feed). Vertical plunge/retract moves are dots. The geometry is keyed by the
  identity of the shared `moves` (a recalculation replaces it, nothing else rebuilds it). A frame is a draw list for the
  view: chunks outside the viewport are culled by their box, the rest decimated to points at least 1.5 px apart (a chunk
  smaller than that becomes one stroke). Cost per frame is O(chunks + drawn points), not O(moves); measured numbers are in
  the T-0223 report. The canvas paints the list from cached layer images (re-rendered only when the view, the model or the
  visible classes change; playback adds the newly played moves to the layer).
- **Visibility and staleness.** On/off, per-toolpath, "selected only", and per-class visibility are viewmodel state
  (session-only). A toolpath that needs recalculation is drawn grey and dashed; disabled or uncomputed toolpaths are not drawn.
- **Time readout.** `makeTimeReadout(JobSummary)`; it states what is missing (rapid rate not set: rapids left out; feed rate
  0: unknown; stale or uncomputed toolpaths) rather than showing a bare number. The dock lets the user enter the rapid rate
  (stored in the session's `MachineSetup`); playback pacing assumes 5000 mm/min when it is unset and says so.
- **Setup.** `makeSimulationSetup` copies out of the session: the enabled computed toolpaths (shared moves), tool shapes
  (`ToolShape::fromTool` from the database tool of the record's `ToolSource`, else a flat cutter of the snapshot diameter,
  with a note), the stock (active sheet's material, job thickness, Z zero: material surface -> top at 0, machine bed -> top
  at thickness; rotary jobs are refused). Quality is the cell size, coarsened (and reported) so that the grid never exceeds
  16 M cells (144 MB with pass ids). The id given to the simulator is the position + 1. `checkSimulationCurrent` compares
  the setup with the session (same toolpaths with the same move-list identity, same stock, none needing recalculation).
- **Controller.** One worker thread owns the `Simulator`, built from the setup's copies; it advances in chunks (adaptive,
  about 30 ms) with `advanceToMove`, so cancel and newer requests take effect within a chunk; the latest `seek` wins, a
  backward seek resets and replays. It publishes immutable frames (shaded relief raster of at most 1600 px on the long side,
  diagnostics) that the UI polls with a 30 ms timer; there are no callbacks into Qt. `cancel()` stops the run where it got to
  and a frame at that move follows. Failures (allocation, refused stock) become a `Failed` state with a message. Scrubbing and
  stepping use `advanceToMove` (byte-exact, section 9); `advanceToTime` is not used.
- **Lifetime rule.** Nothing in a setup, a frame or the worker refers to the `Document`. The panel reads the document only on
  the UI thread (`refresh()`); when the document is replaced the simulation is stopped first (`documentReplaced`), and a result
  that no longer matches (edit, recalculation, enabling, material) is shown with an "Out of date" banner and without the
  canvas tool/played marks (their move indices would refer to a different list). Tested under ASan.
- **Relief.** Top-down hill-shaded raster (light from the upper left), uncut stock in wood colour, cut cells darker with
  depth (or one colour per toolpath), table colour where the cut went through; an isometric 3D view is left for Phase 2.
