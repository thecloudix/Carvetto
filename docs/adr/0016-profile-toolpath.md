# ADR-0016: 2D Profile toolpath

Date: 2026-10-26 · Status: proposed (T-0120)

## Context
T-0120 asks for `profileToolpath(contours, Tool, ProfileParams)` with the options of
feature-inventory §5 "2D Profile Toolpath" (sides, direction, allowance, separate last pass, start
points, tabs, ramps, leads, order, V-bit corners), multi-pass via `PassPlan`, inner vectors first,
gouge-free and deterministic. The card fixes the behaviour to test but not how vectors combine,
how tabs/ramps/leads interact, or how a cut that cannot be made is reported. This records those
choices. Behaviour descriptions come from `docs/reference/` only.

## Decision

### API
1. `profileToolpath` returns `ProfileResult { Toolpath toolpath; std::vector<ProfileDiagnostic>
   diagnostics; }` rather than a bare `Toolpath`, because AC2 needs "no toolpath with a clear
   reason" and a profile over many vectors can partly succeed (one hole too small for the tool,
   one lead shortened). Diagnostics carry a `ProfileIssue` code, the input vector index and a
   message. An empty toolpath always has at least one diagnostic.
2. The tool is a `tooling::Tool`; feeds, speeds, tool number and the default pass depth come
   from `ProfileParams::cuttingData` or else the tool's `defaultCuttingData`. The offset radius
   is half `cuttingWidthAtDepth(tool, bottom of cut)`, so for a V-bit the top edge of the cut
   lands on the vector; for end mills it is D/2.
3. Shared machinery lives in its own headers so other operations (Pocket, T-0121) can reuse it:
   `path_walk.hpp` (arc-length paths, piecewise-linear height profiles, `appendLeg`),
   `clearance.hpp` (distance-to-vectors checks), `tab_placement.hpp`, `ramp_entry.hpp`,
   `lead_moves.hpp`, `cut_order.hpp`. `profile.hpp` holds only the operation.

### Geometry
4. **Region semantics.** Closed vectors are regions by nesting parity (a vector inside an odd
   number of others is a hole). They are oriented region-on-the-left (outer CCW, hole CW, island
   CCW). Outside keeps the region: the tool runs outside outer boundaries and inside holes;
   Inside runs inside the region. Open vectors: Outside = right of travel, Inside = left (the
   enum has `Right`/`Left` aliases). On runs on the vector (offset by the allowance only).
5. **Exact offsets first.** Each vector is offset by r + allowance with
   `geometry::offsetOpenContour` (the public API as on main; closed contours are accepted), so
   lines and arcs stay exact: an inside profile of an r = 10 circle with a 6 mm tool is one
   r = 7 arc move (AC2). Joins are Round; Sharp (mitre limit `corners.mitreLimit`) for a V-bit
   with "sharp external corner". A chain whose ends meet is treated as closed (the API returns a
   full circle as one open span).
6. **Neighbours.** If an exact offset of a closed vector comes within r + allowance − tol of
   another closed vector (checked by `findViolations`, below), the vectors are grouped
   (union-find) and offset together as one region: even-odd union of the group
   (`unionContours`, so every edge is a region boundary even where vectors cross), then
   `offsetContours` by ±(r + allowance + chordal), the sign flipped when the group lies inside
   an odd number of other vectors. The extra chordal keeps the polygonal result at least r from
   the true curves. These cuts are polygonal (`MergedWithNeighbour`). A closed vector whose own
   offset comes back as open chains is handled the same way.
7. **Safety net.** Every cut is then checked against every other vector (merged cuts also
   against their own members) and stretches closer than r + allowance − tol are removed
   (`TrimmedNearVector`); a trimmed loop becomes open pieces. Exact offsets are not re-checked
   against their own vector (ADR-0012 guarantees it; see Consequences). On profiles skip 6–7.
8. **Clearance check.** Distance along a path is 1-Lipschitz in arc length, so an interval
   whose end samples both exceed threshold + half its length is certified clear; others are
   bisected down to tol/2. Violations are bracketed by the neighbouring clear samples (removal
   errs on the safe side). Obstacles are pre-filtered by bounding box.

### Machining
9. **Direction.** Assuming an M3 (clockwise) spindle, climb keeps the vector on the right of
   travel (outside profiles run clockwise, inside counter-clockwise); conventional the reverse.
   Open vectors cut On keep their own direction for climb.
10. **Order.** Containment tree over the cuts (parent = smallest closed cut containing a probe
    point); post-order, so inner cuts always come first. Siblings are ordered by the enabled
    strategy: selection order; left-to-right (bbox left edge, then bottom: column by column);
    bottom-to-top (row by row); grid (rows bottom-up, a row = cuts whose y-extent overlaps the
    row's first, serpentine). Several enabled → the one with least rapid travel (ties: that
    order). None → greedy nearest neighbour from `startPosition`. Each cut is machined through
    all levels before the next. With a separate last pass, all roughing cuts come first, then all
    finishing cuts in the same order.
11. **Start points.** `StartAt::VectorStart` (= "Use Vector Start Point"): the cut point nearest
    the vector's start; `Optimise`: nearest the previous cut's start; `ClosestOnBoundingBox`:
    nearest one of nine anchors of the cut's bounding box. Ties follow `closestPoint` (lowest
    span). The closed cut is rotated to start there.
12. **Height profiles.** A pass is a list of legs (`PathLeg`: walk the path from u0 to u1, either
    way, with a piecewise-linear Z over travelled distance). Lines become feeds, arcs (helical
    where Z changes, exact because Z is linear in arc length between knots), profile jumps
    vertical moves (Feed up, Plunge down). Tabs are a ceiling profile combined by point-wise
    max, so ramps, tabs and passes compose without special cases. Arcs with a chord under
    0.01 mm are written as feeds (a controller would read them as full circles once rounded).
13. **Ramps** (`planPass`). Closed: Smooth descends over L (distance, or drop/tan(angle)) and
    continues until L past the start, so the ramp is re-cut at depth; the next pass starts L
    further on. Zig-zag goes back and forth over L (angle mode adds leg pairs so no leg exceeds
    the angle) and returns to the start, then cuts the loop. Spiral descends one pass depth per
    revolution, plus one flat revolution at the bottom. Open: Smooth/zig-zag ramp over the first
    L, return to the start at depth, then cut the whole pass; spiral falls back to smooth.
    "Ramp on lead-in" descends along the lead-in on the first pass. Ramps run at feed rate; the
    first plunge goes only to the top of the cut. The separate last pass never ramps (the slot
    is already cut).
14. **Tabs.** Tab length is the material left along the vector, so the raised stretch on the
    tool path is length + tool diameter (exact on straight edges). Tab top = bottom + thickness;
    only passes below it are affected. Constant number: centres at (i + ½)·L/n from the cut
    start (the plunge point is never in a tab); constant distance: n = round(L/distance) ≥ 1.
    "Avoid corners" moves each tab to the nearest place whose raised stretch lies on one line
    span; tabs that cannot be moved, overlap or do not fit are reported (`TabsNotPlaced`). 3D
    tabs ramp at 45°. With a separate last pass, tabs are placed on the finishing cut and
    projected onto the roughing cut. Interactive tab editing and "tab at machining start" are
    not implemented (see Consequences).
15. **Leads.** Straight (length, angle to the path) or circular (radius, sweep, tangent) from the
    waste side; optional mirrored lead-out; overcut (closed cuts) runs past the start before the
    lead-out. Leads are used on the first entry and final exit of each cut, not between levels.
    Each lead is checked against every vector with the clearance check at tool radius − tol;
    a gouging lead is halved up to three times (`LeadShortened`), then left out (`LeadDropped`).
16. **V-bit corners.** Sharp external = Sharp joins (5). Sharp internal: on the final level, at
    every inside-corner kink of the path the tool runs straight to the vector's nearest corner
    vertex while rising to the top of the cut and back (depth ∝ distance to the corner along the
    bisector, so the cone stays on both walls). Allowance is not applied to that move. Both
    options are ignored with `CornerNeedsVBit` for other tools.
17. **Bezier input.** Beziers are polygonised before offsetting; when present the offset chordal
    tolerance is tightened to tol.distance/2 so the cut stays within tolerance of r.

### Golden hand-check (AC1 / AC6)
`core/tests/golden/toolpath/profile_rect_tabs.moves` and `profile_rect_tabs_grbl.nc` were
checked by hand: the offset of the 100×50 rectangle by 3 is the lines y = −3, x = 103, y = 53,
x = −3 joined by quarter arcs of r = 3 about the corners (every arc centre is a corner, CW for
climb); the start (0, −3) is the end of the bottom-left arc nearest the vector start (0, 0)
(the whole arc is equidistant; lowest span wins); levels −4, −8, −12 with vertical plunges at the
start; perimeter P = 300 + 6π = 318.8496, tab centres at P/4 and 3P/4 along the cut land at
x = 20.2876 on the top edge and x = 79.7124 on the bottom edge, so raised stretches
x ∈ [12.2876, 28.2876] on y = 53 and [71.7124, 87.7124] on y = −3 (16 mm = 10 + 6), at Z −10, on
the −12 pass only (computed independently in Python). The G-code was checked against the
`posts/README.md` checklist (G21/G90, safe Z before the XY rapid, spindle on before the plunge,
I/J = centre − start, plunge at F300 and feeds at F1000 restated, retract, M5, M30).

## Consequences
- Merged neighbours lose arcs; refitting belongs to the Fit Curves card. Grouping is pairwise
  O(n²) with box pre-filters (as are nesting and containment); thousands of vectors (text) will
  need a spatial index.
- The gouge guarantee against a vector's own exact offset rests on ADR-0012 and inherits its
  documented sub-tolerance-span limitation (T-0009); T-0017 changes offset quality, not this
  code. Tests keep inputs free of sub-tolerance spans and of Sharp joins on zig-zags.
- Not implemented: interactive tab editing and per-vector stored tab positions (needs the
  document/UI side), "tab at machining start point", leads between levels, cutter compensation,
  projection onto a 3D model.
- Decisions the director should confirm with the product's behaviour: region semantics (4),
  tab length as material length (14), last pass at full depth only and roughing-before-finishing
  (10), leads only on entry/exit (15), ramps at feed rate (13), 3D tab slope (14), ordering
  definitions (10).
