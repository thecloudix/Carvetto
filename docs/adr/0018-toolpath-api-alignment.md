# ADR-0018: Toolpath API alignment (shared operation types, one ramp, spatial index)

Date: 2026-11-01 · Status: proposed (T-0123); amended by T-0131 (2026-11-03, §12)

## Context
Profile (T-0120, ADR-0016) and Pocket (T-0121, ADR-0017) were built in parallel and grew two
APIs: Profile took a `tooling::Tool` and returned `ProfileResult` with coded diagnostics; Pocket
took hand-filled `PocketTool{ToolSnapshot, stepover, passDepth}` and returned strings. Both headers
defined `toolpath::CutDirection`, so no unit could include both (the CLI, T-0122, must). There were
two ramp implementations (`ramp_entry` and `pocket_ramp`). Every vector-pair check was a loop over
all vectors (Profile grouping, nesting, trimming, ordering; Pocket hole assignment and guard), so a
page of text (thousands of small vectors) took seconds to minutes. Pocket rest tools built the
whole pattern and clipped it piece by piece against every rest area (a 200-tooth comb with a
0.1 mm rest tool: 23.7 s), and Raster and rest links retracted wherever a straight link was blocked.
The card fixes the outcomes (one ramp, Tool + cutting data in, one diagnostics type, < 2 s for
5,000 vectors with ≤ 2.5× per doubling, ≥ 5× on the comb, routed links) but not how.

## Decision

### Shared operation API (`toolpath/operation.hpp`)
1. **One `CutDirection`** for every operation (climb keeps the wall being cut on the right of
   travel with an M3 spindle; Profile's and Pocket's definitions said the same thing in different
   words). Enumerator order kept (`Climb`, `Conventional`).
2. **One diagnostics type.** `ToolpathDiagnostic { ToolpathIssue issue; optional contour; string
   message; optional tool }` and `hasIssue`. `ToolpathIssue` holds Profile's codes unchanged and in
   the same order, then Pocket's (`NoTool`, `InvalidTool`, `OpenContourIgnored`, `NoRegion`,
   `WorkLimitExceeded`, `RegionNotKept`, `RegionTrimmed`, `NoRestMaterial`, `RestUnreachable`;
   Pocket also uses `InvalidParameters` and `ToolTooLarge`). `ProfileIssue` and
   `ProfileDiagnostic` remain as aliases, so Profile code and tests did not change. `PocketResult`
   and `pocketCentreRegion` report `ToolpathDiagnostic`s with the former message texts; `tool` is
   the index in the tool list. Both results have `ok()` and `has(issue)`.
3. **Tool → pass tool conversion.** `effectiveCuttingData(tool, optional data)` (the given data,
   else `Tool::defaultCuttingData`), `toolSnapshot(tool, data)`, and `passTool(tool, data,
   bottomDepth) → PassTool{snapshot, radius, stepover, passDepth}`. The radius is half
   `cuttingWidthAtDepth` at the bottom of the cut, as Profile already did (ADR-0016 §2): D/2 for an
   end mill (so all existing pockets are unchanged), the radius at the material surface for a
   V-bit or ball nose (the top edge of the cut keeps the distance). **Owner decision
   (2026-11-01): the pocket uses this same rule as Profile** — half the cutting width at the
   bottom of the pocket.
4. **Both operations take `tooling::Tool` + optional cutting data the same way**: Profile
   `profileToolpath(contours, Tool, ProfileParams)` with `ProfileParams::cuttingData`; Pocket a
   list of `PocketTool { Tool tool; optional<CuttingData> cuttingData; }` (one entry per tool:
   clearance then rest tools), the same "empty = the tool's default data" rule. Callers that know
   machine and material pass `Tool::cuttingDataFor(machine, material)`. Parameter field names
   (`ProfileParams`, `PocketParams`) are unchanged so params.json (T-0122) can mirror them.

5. **Refused vectors (T-0183).** `ToolpathIssue::ContourRefused` says that one input vector was
   not cut because its polygon cannot meet the chordal tolerance within the work bounds (an arc
   needing more than 2^20 chords, a Bezier past its split cap, a non-finite arc or Bezier;
   ADR-0005 A2): the booleans, offsets and `OpenPathClipper` leave such a vector out and only
   count it (`BooleanResult::refusedContours`, `OpenPathClipper::refusedContours()`; the offset
   has no count). An operation that polygonises its input therefore screens it up front with
   `geometry::refusedByClippers(contour, tol)` (`tol` = the finest tolerance the operation
   polygonises the input at, so a vector the operation would have cut is never refused), and
   reports one `ContourRefused` diagnostic per refused vector (`contour` = its input index,
   `refusedContourDiagnostic`), then goes on with the others: the refused vector is skipped like
   an empty one, the result for the rest is exactly what the operation gives without it. The
   viewmodel shows it as an Error (the toolpath of that vector is not generated). Operations:
   Profile (every vector, open or closed), Pocket and Engrave Fill (closed vectors; open ones are
   ignored as before), V-Carve (closed vectors; also a member of a crossing cluster that is only
   too coarse at the finer resolving tolerance). Not affected, because they never polygonise
   through the clippers: Drill, Thread Mill, Chamfer and Engrave Outline (they work on the spans
   and the centre / distance queries, which handle a huge arc exactly).

### One ramp module
5. `ramp_entry` owns every ramp. Its zig-zag is exposed as `zigZagLegs(u, direction, length,
   zFrom, zTo, ramp)`; `planPass` uses it, and Pocket's ramp plunge is `appendPathRamp(polyline,
   zFrom, zTo, maxLength, minLength, tol, out)` = the same two legs walked with `appendLeg` on the
   path (zero-length segments dropped). `pocket_ramp.{hpp,cpp}` are removed. Pocket output is
   unchanged up to rounding of the ramp heights (≤ 1.4·10⁻¹⁴ mm over 600 random pockets; no move
   kind changes); Profile output is byte-identical.

### Spatial index
6. **`BoxIndex`** (`toolpath/box_index.hpp`): a uniform grid over a fixed set of bounding boxes.
   Cell size = max(median box size, √(width · height / (4n)) of the overall bounds,
   extent / 2048, 10⁻⁹ mm); a box touching more than 64 cells (a frame round everything), or
   whose corners or width/height are not finite, goes to a list every query checks, and extents
   that overflow a double fall back to a single cell. A query applies the exact
   `Box2::intersects` test and returns indices **ascending**, so a loop over the result visits
   candidates in the same order as the quadratic loop it replaces and gives the same answer
   (including ties). A uniform grid rather than an R-tree: the inputs are many similar-sized
   vectors (text, nests of parts), the build is two linear passes, and the grid is a few dozen
   lines without a dependency.
7. **Indexed `ClearanceField`.** Obstacle boxes go into a `BoxIndex`; `distance(p)` searches a
   square of half-width h round p, ×4 until the best distance found is ≤ h (every obstacle outside
   the square is farther) or the square covers everything. The value is the minimum over all
   obstacles, as before. `excluding(indices)` shares the obstacles and skips some, so Profile's
   trim no longer copies every other vector per cut.
8. **Where it is used.** Profile: nesting parity, neighbour grouping, the merged group's parity,
   trimming (`allField_.excluding(sources)`), the containment tree and greedy nearest ordering in
   `cut_order` (square search with the scan's tie rule), and diagnostic de-duplication (a set).
   Pocket: hole assignment in `groupsOf`, the guard's inside test (`RegionIndex`), a bounding-box
   pre-test in the guard's edge grid, the part lookup and nearest order of rest clusters; the swept
   area is no longer computed for the last tool (nothing uses it). These keep results identical:
   2,400 random Profile scenes with every option byte-identical to main; single-tool Offset pockets
   identical apart from the ramp rounding above.
   Numbers (Release, CPU time, the T-0123 machine): 5,000 small closed vectors (squares, L shapes,
   circles, squares with holes on a 6 mm grid, 1 mm tool) profile in 0.15–0.27 s (main: 13.9 s,
   3.7× per doubling) and pocket in 0.55–0.65 s (main: 10.3 s), 1.8–2.4× per doubling; a page
   whose offsets all merge (4.6 mm grid) profiles 5,000 vectors in 3.0 s (main: 34 s at 1,250).

### Pocket rest tools and links
9. **Rest patterns over the rest areas** (amends ADR-0017 §10). The rest areas are `need ∩ C` —
   the centres the tool may visit whose disc overlaps the rest material by more than 10·distance —
   and the tool's normal pattern (Offset rings or Raster lines with the boundary pass) is built
   over each connected rest area, one cluster each. No clipping. A pattern over a region covers
   every point within a radius of it, so every rest point the tool reaches from a centre in `need`
   is cut, and every path lies inside C. The need region is offset at the region tolerance and its
   boundary thinned by Douglas-Peucker at that tolerance (the swept area's 10⁻⁴ mm round joins
   would otherwise put hundreds of vertices into every ring); it moves by at most chordal/2 and is
   then clipped to C, so safety is unchanged and the reach band left uncut grows from 10·distance
   by at most chordal/2. Clusters are ordered part of C by part, nearest first (greedy on
   bounding-box centres, ties to the lower index). The comb (200 teeth, 1 mm clearance tool,
   0.1 mm rest tool at 40 %): Raster 23.7 s → 0.21–0.27 s, Offset 0.85 s → 0.12 s.
10. **Routed links** (amends ADR-0017 §8). A Raster part gets a one-node ring tree (its own outer
    and island rings), so a link blocked by an island is routed along the rings and bridges like
    the Offset strategy's, at the floor of the previous level (cleared there when the boundary
    pass is cut; Raster + Profile Pass None still never travels at depth). Rest clusters route
    inside their own rest area through their pattern's tree. So a connected pocket retracts once
    whatever its islands. **Owner decision (2026-11-01): between separate rest areas (clusters)
    the tool retracts to safe Z**, even when they lie in one connected part of the centre region;
    it does not feed across the top of the cut there (the first T-0123 build did). A rest
    toolpath therefore retracts once per rest area; each area is cut through all its levels with
    routed links and no retract inside it, and the areas are ordered part by part, nearest first,
    to keep the rapids short.
11. **Raster pattern memory** (closes the ADR-0017 §12 correction). Lines are clipped in batches
    of 256 (open paths do not interact: the pieces are the same) and counted against a budget of
    move cap / (6 × levels) pieces (each piece costs at least 6 moves per level); past it the tool
    is refused with the move-cap message before memory fills.

### Scaling residuals (T-0131)
12. **Every remaining all-pairs loop the T-0123 verifiers found is indexed, with the same output.**
    Each change visits the candidates a full scan would accept in the same order with the same
    comparisons, or argues why the skipped ones cannot change the answer; Profile and Pocket output
    is byte-identical to main (hash sweeps over the T-0123 verifier generators, every option, and
    combs, pages and rest pockets).
    - `BoxIndex` built from boxes none of which fits the grid (only huge, infinite or NaN boxes)
      keeps them in the every-query list (they were dropped, so queries returned nothing).
    - Profile, separate last pass: a finishing cut's counterpart (median of 17 distances within
      the last-pass allowance + r) can only be a roughing cut whose box, grown by that, meets the
      17 points' box; the nearest-start fallback is a square search grown ×4 until the best
      distance is a tolerance inside the square. The other pass's start points near a cut
      (`otherStarts`) and the roughing cuts a finishing tab can be mapped onto (`mapTab` reach) come
      from `BoxIndex` queries, in index order.
    - Pocket Raster zig-zag: the next row's pieces are searched only where their span along the
      raster meets the cursor ± (link limit + slack): a farther piece is farther than the link
      limit, and then the run ends anyway. Rows are sorted by start; a running maximum of the ends
      bounds the backward scan.
    - Travel routes: the nearest ring point comes from a `BoxIndex` of the node's ring edges (a
      square search, candidates in (ring, edge) order, so ties resolve as before; cached per tree
      node like the bridges), and `walkRing` walks both ways together, the shorter so far first,
      and stops once one way is complete and the other's partial length already decides (sums of
      non-negative lengths in order never decrease), instead of building both ways round a ring of
      thousands of edges.
    - `RegionIndex` (Pocket's inside / segment tests): cells are square over the area (about one
      edge per cell, at most 2048 along the longer side) instead of a fixed count along the longer
      side, which left a long thin region with a few rows of hundreds of edges; every query
      combines its edges with OR / XOR, so the cell size changes no answer. The parity ray goes the
      way (±x, ±y) with the fewest cells when the point is more than the tolerance from every edge
      and the tolerance exceeds 10⁻⁹ × (1 + coordinate size): the crossings are then more than the
      tolerance from the point and computed within a few ulps, so every direction gives the exact
      even-odd answer; otherwise the ray to +x as before.
    - Rest areas' nearest-first order uses one `BoxIndex` per part of the centre region (the same
      squares, candidates and order as one index filtered by part) and leaves a part when its areas
      are used up, instead of growing the square over the whole page to find nothing.
    - `ClearanceField::distance` (Profile's tab check, trimming, leads): per obstacle, the span whose
      box lies nearest along x or y is measured first, and spans whose box lies farther along x or y
      than the best distance so far plus a rounding margin are skipped; the minimum of the same
      values does not depend on their order. The box-distance test skips the hypot when one axis
      decides it.
    Measured (Release, CPU, the T-0131 machine; before = the T-0123 merge on the same machine):
    1,000-tooth comb, 0.3 mm Raster: 250 / 500 / 1,000 teeth 0.07 / 0.15 / 0.36 s (0.50 / 1.46 /
    5.65 s). Profile with a separate last pass on a text page: 2,500 / 5,000
    vectors 1.1 / 2.4 s (main, the verifier's page: 16 / 69 s at 1,250 / 2,500). Pocket with a
    0.3 mm rest tool on a text page: 1,250 / 2,500 / 5,000 vectors 1.2 / 2.7 / 6.7 s (2.0 / 4.8 /
    13.6 s on the verifier's page). Tabs (one 1 mm tab per vector): 1,250 / 2,500 / 5,000 vectors
    3.3 / 6.5 / 12.7 s, linear (about 30 % less than main on the verifier's page; the per-tab
    raster with a distance query per 0.05 mm cell is the cost). What remains superlinear is
    Clipper2's own sweep: the boolean difference / intersection over a whole page for rest tools
    and the inset of a comb's outline (T-0142), and the line clipping at other raster angles,
    whose active edge list holds every tooth: a comb at 30° ~3.9× per doubling at 500 → 1,000
    teeth, at 90° 3.5 / 5.2 / 6.6× per doubling over 250 → 500 → 1,000 → 2,000 teeth (T-0131
    verifier); both left to T-0142 item 3. Pocket with a rest tool is accepted at ~2.4–2.6× per
    doubling until then (director, 2026-11-03); its test bounds 3.0 per doubling.

## Consequences
- The CLI (T-0122) can include `profile.hpp` and `pocket.hpp` together and build both operations
  from a tool-database `Tool` and the cutting data it looks up.
- Rest rings follow the rest areas' boundaries, so a rest toolpath has more (polygonal) vertices
  than the old clipped straight pieces, and fewer retracts (one per rest area instead of one per
  clipped piece: the 4-corner case 16–33 → 4).
- Still quadratic and left for follow-up cards: Raster zig-zag linking scans a whole row per step
  (a 1,000-tooth comb inside the caps: 6–14 s); Profile's separate-last-pass matching of finishing
  to roughing cuts compares every pair (only with a separate last pass); Clipper2's sweep over a
  whole region is slightly superlinear (it dominates the remaining pocket scaling). **The first
  two are resolved by T-0131 (§12); Clipper2's sweep is T-0142's.**
- The T-0121 verifier's note that rest tools re-trace link-cut scallops is not addressed here (it
  concerns what counts as swept, not the API or the pattern area).
