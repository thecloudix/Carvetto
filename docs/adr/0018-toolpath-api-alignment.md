# ADR-0018: Toolpath API alignment (shared operation types, one ramp, spatial index)

Date: 2026-11-01 · Status: proposed (T-0123)

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

## Consequences
- The CLI (T-0122) can include `profile.hpp` and `pocket.hpp` together and build both operations
  from a tool-database `Tool` and the cutting data it looks up.
- Rest rings follow the rest areas' boundaries, so a rest toolpath has more (polygonal) vertices
  than the old clipped straight pieces, and fewer retracts (one per rest area instead of one per
  clipped piece: the 4-corner case 16–33 → 4).
- Still quadratic and left for follow-up cards: Raster zig-zag linking scans a whole row per step
  (a 1,000-tooth comb inside the caps: 6–14 s); Profile's separate-last-pass matching of finishing
  to roughing cuts compares every pair (only with a separate last pass); Clipper2's sweep over a
  whole region is slightly superlinear (it dominates the remaining pocket scaling).
- The T-0121 verifier's note that rest tools re-trace link-cut scallops is not addressed here (it
  concerns what counts as swept, not the API or the pattern area).
