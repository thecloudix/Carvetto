# ADR-0017: Pocket toolpath

Date: 2026-10-26 · Status: accepted (director, 2026-10-31; T-0121; revised in fix rounds 2 and 3, same day; amended by T-0125, 2026-10-30 and 31, and T-0126, 2026-10-31)

## Context
T-0121 adds `pocketToolpath(contours, tools, PocketParams)` (feature-inventory §5 "Pocketing
Toolpath"): Offset or Raster clearing, Profile Pass first/last/none, Pocket Allowance, Ramp Plunge
Moves, a multi-tool list of clearance and rest tools, and islands. The card fixes the behaviour
but not the region model, the linking rules, the safety margin, how rest regions are found, or
what the Profile Pass means for the Offset strategy. It runs in parallel with T-0017 (open-offset
quality) and T-0120 (Profile, ADR-0016), so it may use only the public geometry API in `main`.

## Decision
1. **API.** `toolpath/pocket.hpp`: `PocketTool` (a `ToolSnapshot` plus absolute `stepover`, with
   0 < stepover ≤ diameter, and `passDepth`), `PocketParams` (depths and `PassPlan` options, safe Z,
   strategy, `CutDirection`, raster angle, `ProfilePass`, allowance, ramp on/off and distance,
   `Tolerance`), and `PocketResult` with one `Toolpath` and one `PocketToolPlan` per tool (centre
   region, rest material, planned 2D paths with role and cluster, Z levels) plus messages. Bad
   input (open contours, a tool too large, a stepover over 100 %, zero depth, …) gives a message
   and an empty toolpath for that tool, as do parameters that would make the work explode (12).
   It never throws. With more than one tool, each toolpath is
   named `"<name> [<tool name>]"` (the inventory's tool-inclusive naming).
   **Amended by T-0123 (ADR-0018):** a `PocketTool` is now a `tooling::Tool` plus optional
   `CuttingData` (empty = the tool's default data, as `ProfileParams::cuttingData`); `passTool`
   takes from it the radius (half the cutting width at the bottom of the pocket: D/2 for end
   mills), stepover, pass depth and snapshot. Messages are `ToolpathDiagnostic`s (shared with
   Profile) with an issue code, the same texts and the tool index; `CutDirection` is shared.
2. **Region.** The pocket is the **even-odd** union of the closed input contours, so nesting
   decides islands (pocket / island / pocket again) whatever the orientation. Open contours are
   ignored with a message.
3. **Safety margin.** All region work is polygonal (Clipper2, ADR-0006). The inputs are
   polygonised and the round joins approximated at **half** the chordal tolerance. The tool centre
   is kept `radius + allowance + chordal` from the walls. Polygon sag on the unsafe side (≤ c/2)
   plus join chords (≤ c/2) can therefore never bring the cutter closer than radius + allowance to
   the true walls or islands (AC2 and AC6 hold strictly). The price is a wall skin of at most
   1.5·chordal (0.015 mm by default) left on top of the allowance.
4. **Centre region and inset guard.** `C = inset(R, radius + allowance + margin)`. Every inward
   offset goes through `insetBy`. Clipper2 1.4.0 turns a polygon inset past its inradius inside
   out: a 17 mm circle inset by 19.2 comes back as a 2.2 mm circle, by 30 as a 13 mm one. Without
   the guard, offset levels never end, and a tool too big for a small pocket would get a phantom
   region to cut. Distances use a uniform grid over the source edges.

   **Phantom test (by depth, T-0126).** A result region is genuine when some point of it keeps
   the full distance (the inset less the join tolerance) from the source and lies inside it: the
   first outer vertex that keeps the distance, or else (for a region that is not a sliver, see
   below) the centroid of the outer ring when it lies in the region. Such a point belongs to the true inset, so the region does too. An inside-out
   region has no such point: all of it is closer to the walls than the inset. Before T-0126 the
   test counted vertices (most outer vertices had to keep the distance), so a 900 mm² region whose
   small bulge towards a wall held most of its vertices was rejected silently (verifier cases C2
   and C4, now regression tests).

   For the **tool-centre region** (`Strict`), a vertex or an edge too close would be a gouge. Every
   vertex is checked, on the outer ring and on island (hole) rings alike:
   - **Kept:** every vertex keeps the distance.
   - **Pulled in:** a failing vertex that is convex for the region (a left turn; the region lies on
     the left of every ring) is moved towards the midpoint of its neighbours, by bisection, just
     far enough to keep the distance. If that midpoint fails too, the vertex is removed, which
     cuts the triangle off, and its neighbours follow in later passes. Rings keep at least three
     vertices, so a hole is never removed. Either way the new boundary lies inside the triangle
     (prev, vertex, next), so the region only shrinks, and only where it overshot. This covers a
     vertex a hair past the slack, which moves by about that hair, and the zero-width
     out-and-back spikes Clipper2 leaves in disc-union insets, whose tips overshoot by up to about
     1e-2 mm.
   - **Edges (T-0126).** Two vertices that keep the distance do not make the edge between them
     keep it: the chord left by removing a vertex of a round join around a wall corner sags
     towards the corner by about four times the arc step (0.015, 0.148 and 0.708 mm at chordal
     0.01, 0.1 and 0.5 in the verifier's raw-injection cases). After the pull-in, every edge with
     a moved end and every chord left by a removal is checked with the exact segment-to-segment
     distance and must keep the same distance as a vertex. Every other edge (Clipper2's own
     chords, which keep about the inset less one join tolerance) must stay clear of the no-gouge
     line, the inset less two join tolerances (radius + allowance for the centre region).
   - **Repaired by inset:** if a failing vertex is reflex, or an edge fails, the region is inset
     further, by twice the worst vertex or edge shortfall left after the pull-in (at least
     2e-6 mm) and then doubling, at most 12 times and never past the inset itself. Each result
     goes through the same pull-in and edge checks. Whether a region can be repaired is decided by
     whether a further inset yields valid regions, not by the size of the shortfall.

   - **Slivers:** where a lobe collapses, Clipper2 leaves dozens of zero-area sliver regions,
     some with a vertex at full depth. A sliver (area / perimeter under half the join tolerance)
     is kept if it passes as it is and otherwise dropped without a report, and its centroid
     never counts as a point at depth (a thin crescent of a collapsing knob in the verifier's S3
     scan, knobs seed 3383 at chordal 0.5, d = 5.666, had its centroid at depth, all of its
     vertices too close, and its pulled-in remains 2e-6 mm² outside Clipper's raw inset). An
     inset only makes a sliver vanish, and it holds nothing the tolerance does not already allow for (knobs scan seed
     7852, fixture `knob_poly.txt`, d = 6.696687, where the first T-0126 build reported such
     slivers as lost).

   Nothing is lost or trimmed silently. A genuine region that still cannot be kept is reported in
   the result's messages ("could not be kept within the safety margin and are not cut"), and so
   is a region repaired by an inset larger than the join tolerance, or losing more area than its
   perimeter times the join tolerance ("trimmed by more than the tolerance"). Neither has
   happened with real Clipper2 output in any scan so far: the pull-in only ever removed zero-width
   spikes, and repairs (Clipper2's 1e-6 mm grid) stayed far below the join tolerance.

   **History.**
   - Before T-0125, any failing vertex rejected the whole region. Verifier sweep seed 3150371 lost
     a lobe (4,854 reachable cells uncut, worst 6.04 mm) for one vertex 1e-6 mm past the slack.
     That is now the fixture `core/tests/fixtures/pocket/gap_poly.txt`, with d = 2.029.
   - T-0125 round 1 capped the repair at an overshoot of one join tolerance. Spike tips overshoot
     by 5.1–5.4e-3, so genuine disc-union regions of tens of mm² were reported but not cut. The
     fixture is `spike_poly.txt`, with d = 5.04 and 5.0673 (verifier seeds 301, 700 and 606).
   - T-0125 checked vertices only and classified phantoms by vertex count (see above); T-0126
     added the edge checks, the depth test and the "trimmed" report. The guard is reachable on
     hand-made insets through the internal `detail::strictInset` (`core/src/toolpath/
     pocket_guard.hpp`), which `pocket_guard_test.cpp` uses for the N1 cases at chordal 0.01, 0.1
     and 0.5, for C2 and C4, and for phantoms.

   The same computation is public as `pocketCentreRegion`. A property test scans over 5,000
   diameters on lobed shapes and disc chains at chordal 0.01, 0.1 and 0.5, against an exact
   signed-distance oracle: lobe centres kept, no vertex outside the pocket, no edge too close
   (exact segment distance, every diameter), and the toolpath plans the same region.

   Everywhere else (offset levels, rest material, openings) the guard only rejects phantoms, by
   the same depth test, and keeps genuine regions as they come (`Lenient`). A genuine region can
   come out with a vertex near a cusp of the input a little too close, which costs nothing there.
   T-0121 round 2 dropped a whole 15 × 6 mm neck of rest material for one such vertex (verifier
   dumbbell repro); round 3 kept regions where at least half the vertices kept the distance, and
   T-0126 replaced that count by the depth test.
5. **Offset strategy.** The levels are `inset(C, k·stepover)`, each taken from C directly so
   errors do not accumulate. They are grouped into regions (outer + holes) and arranged as a
   containment tree, which is cut in **post-order (inside out)**. The innermost rings come first
   and the root rings (outer wall and island walls of C) are placed by the Profile Pass (6). When
   stepover > radius, some points of a level deeper than the radius may be left uncut where the
   next level does not reach. Wherever that residual is non-empty, an extra ring
   `inset(C, k·stepover + radius)` is inserted. Rings of depth ≤ r cover their band, and the extra
   ring covers depths up to 2r ≥ stepover, so coverage is complete for every stepover ≤ diameter.
   The residual is computed with round joins at ≤ 1e-4 mm and opened by
   `sliver = min(chordal/2, 0.005)`, so only residue thinner than min(chordal, 0.01 mm) is ignored.
   That is the same order as the wall skin of (3). Round 1 opened it by 5·chordal and so lost real
   strips up to ~0.1 mm wide in narrow pockets (verifier repro: 60 × 12.08, 6 mm tool,
   stepover 6).
   Ring direction: **climb puts the material on the right** (spindle clockwise seen from above).
   Outer walls are cut counter-clockwise and island walls clockwise; conventional reverses both.
   Each ring starts at its point nearest the current position. That point is inserted when it
   falls inside an edge, and ties go to the lowest y, then x, so the choice never depends on
   Clipper2's vertex order.
6. **Profile Pass with the Offset strategy (owner decision 2026-10-26).** First/Last/None applies
   to both strategies. For Offset, the boundary pass is the tree's root rings (outer wall and
   island walls at the full inset). **First** cuts them before any clearing, then the tree inside
   out. **Last** cuts them after clearing, which is the end of the post-order anyway. **None**
   leaves them as ordinary clearing rings (role `Ring`), and the inside-out order still cuts them
   last, so Last and None give the same moves and differ only in the path role.
7. **Raster strategy.** Lines at `rasterAngle`, at most `stepover` apart and evenly spread over
   the region's extent (first and last half a pitch in), are clipped to each connected part of C
   with Clipper2's open-path intersection. The clipped ends are projected back onto the exact
   line, so every piece is exactly parallel. Pieces are linked zig-zag (each row's nearest piece
   when the link is allowed, see 8). Otherwise the next unused piece in row order starts a new
   run. The boundary pass (all rings of the part, oriented as in 5) is cut first, last or not at
   all, at every level.
8. **Linking.** Every depth level of a cluster is cut before the next cluster. A cluster is a
   connected part of C, or for a rest tool a connected part of its need region. Between two paths
   of a cluster there are three cases:
   - **Short link at depth:** a straight feed at the current depth, when it stays inside the
     cluster's centre region and is no longer than **2 × diameter**. It may cross uncut material,
     no worse than the slotting of the first ring.
   - **Travel:** any other way, including back to the first path of the next level. The tool
     **lifts** (a vertical `Feed`) to the floor of the previous level, or to the surface for the
     first level, then feeds along a way inside the cluster's centre region and descends (8a).
     Above that floor the whole cut area of the cluster is already clear, so travel engages no
     material, apart from material the strategy leaves anyway (raster without a boundary pass).
     Because the way is inside C, it keeps radius + allowance + margin from every wall.
   - **Retract:** only when there is no such way does the tool retract to safe Z, rapid and
     descend.
   The way is the straight segment when that is inside C. Otherwise, for Offset clusters, it is
   routed through the ring tree (8b). With **Raster + Profile Pass None** the walls keep scallops
   at every level, so the floor is not clear along them. Such a toolpath never travels: it
   retracts instead (8c).
   - **8a, the inside test.** `segmentInside` is our own segment-vs-polygon check. A crossing
     away from the ends rejects the segment, and so does a boundary vertex within tolerance of the
     segment's interior, because rounding can hide a crossing through a vertex. The midpoint must
     be inside. An edge the whole segment lies along (both ends within tolerance of the edge
     *segment*) is not a crossing, so links that follow the boundary are accepted even when their
     ends sit a rounding error off it. That is what made raster at 45° retract on every line in
     round 1. The infinite line of the edge is not enough: a nearly parallel edge far away would
     hide an exit, and the round-2 gouge sweep found one.
   - **8b, ring-tree routes.** From each end, climb to the parent level by a straight segment to
     the parent boundary's nearest point. The open disc around a point of a region, out to its
     nearest boundary point, lies inside the region, so each such segment is inside. At the
     lowest common ancestor the way follows its rings and **bridges**. The bridges of a node are
     a Kruskal spanning tree over the closest pairs between its rings, shortest first, and are
     built on first use. A Kruskal bridge cannot be blocked by a third ring: that ring would be
     closer to one end and would already be joined to it. Each bridge is still checked with
     `segmentInside`. The way goes ring to ring by the fewest bridges (BFS) and walks each ring
     the shorter way. It climbs further only when the bridges do not connect, which does not
     happen in exact geometry. The result is then shortened greedily ("string pulling"). Round 2
     tried only the closest pair of the two end rings, and so gave up whenever a third island
     blocked it: three or more islands retracted at every level.
   - **8c, cost.** The inside tests in raster linking, emission and string pulling use a
     `RegionIndex`, a uniform edge grid with a per-row parity ray. Each test costs the edges near
     the segment rather than all edges. A 200-tooth comb raster with 200,000 pieces per level
     dropped from 66 s to 1.7 s.
   With this, a connected pocket of any shape retracts once in total, however many islands it has
   (rows of 3–6 islands, grids up to 6 × 4, U, C, L, plus), and n disjoint regions retract
   n times (AC5). A rest cluster travels only inside its own part of the need region (∩ C): other
   rest clusters may still hold material at every depth. **Amended by T-0123 (ADR-0018 §10):**
   Raster parts and rest areas route travel along their own rings like Offset (they used straight
   travel only); between rest clusters the tool retracts to safe Z (owner decision 2026-11-01).
9. **Descent.** It is a vertical `Plunge`. With `rampPlunge`, the tool plunges through the air to
   the floor already cut (the surface for the first level) and then makes a **zig-zag ramp**
   (`toolpath/pocket_ramp.hpp`, `appendPathRamp`). The ramp runs forward along the path for
   min(rampDistance, path length) down to the midway height, then back over the same points,
   arriving at the start exactly at the level. All ramp moves are `Feed` moves on the path, so a
   ramp can never gouge. Paths shorter than 10·tolerance.distance fall back to a plunge. Since
   T-0123 this is the shared ramp module's zig-zag (`ramp_entry.hpp`, `zigZagLegs` walked with
   `appendLeg`), the same legs as Profile's zig-zag ramp (ADR-0018 §5).
10. **Rest machining.** Tools run in list order; the first usable tool is the clearance tool. The
    rest material for tool i is `opened(inset(R, allowance + margin) − swept, sliver)`. Here
    `swept` is the union of the earlier tools' cut areas: `C₀ ⊕ r₀` for the clearance tool, and
    `(N ∩ C) ⊕ r` for each earlier rest tool. These are computed with round joins at ≤ 1e-4 mm,
    so the joins' chords do not show up as fake rest material. **Amended by T-0123 (ADR-0018 §9):
    tool i now builds its normal pattern over the rest areas `N_i ∩ C_i` (N_i thinned to the region
    tolerance), one cluster per connected area, instead of the clipping described in the rest of
    this paragraph.** Before T-0123, tool i built its normal pattern
    over its own C_i and clipped every path, as an open path, to `N_i = offset(rest, r_i − 10·distance)`:
    the centres whose disc overlaps the rest material by more than 10·distance. A tool that would
    only touch it, such as the same tool again, gets nothing. N_i is deliberately *not* intersected
    with C_i: the outermost ring lies on C_i's boundary, where open-path clipping is ambiguous and
    drops pieces. Each path is clipped only against the parts of N_i whose bounding box it
    overlaps. Pieces keep their path's direction and order, a ring cut across its seam is
    rejoined, and the pieces are clustered by the connected part of N_i that holds them. With
    Raster + None the swept area is the tool disc along the actual paths rather than the whole
    reach, so the wall scallops become rest material for the next tool. Round 1
    opened the rest material by 5·chordal and so skipped real thin residue: a ~0.1 mm corner band
    at chordal 0.049 (verifier seed 31493).
11. **Output.** Moves are straight lines only, because the geometry is polygonal. Rapids are at
    safe Z, every toolpath starts with a rapid and ends retracted, and the output is
    deterministic (no unordered containers, ties broken geometrically).
12. **Work caps.** These are checked before any geometry work, in this order. Any value outside
    them gives a message and an empty toolpath, never an exception or an unbounded run:
    - chordal tolerance in [1e-4, 1] mm, and 0 < distance ≤ chordal;
    - at most 2·10⁵ vertices for the polygonised input (estimated from span types);
    - stepover ≥ 10·distance;
    - at most 10⁴ depth passes, estimated as ⌈cutDepth / passDepth⌉ + 2, before `PassPlan` runs;
    - at most 2·10⁴ rings or raster lines per part, estimated from the pocket region's bounding
      boxes shrunk by the tool, and at most 2·10⁷ estimated moves (levels × rings × vertices);
    - after the pattern is built and before any move is emitted, the actual move count,
      levels × Σ(points + 4) over the paths, must also stay within 2·10⁷. The estimate cannot see
      raster lines cut into many pieces: the round-2 comb, with ~200 pieces per line and
      900 levels, would have run for hours. Building the pattern is O(lines × pieces) and is
      refused in under 1 s. **Correction (T-0125):** this caps the moves, not the pattern built
      before the check. The raster pattern's memory is proportional to lines × pieces and is not
      bounded until then: the verifier hit `std::bad_alloc` on a 1000-tooth comb inside the
      caps. Bounding it, and the quadratic cost of rest-tool raster clipping, moved to T-0123.
      **Resolved by T-0123 (ADR-0018 §9, §11):** raster lines are clipped in batches against a
      piece budget (move cap / (6 × levels)), and rest tools no longer clip.
    The verifier's repros now return at once: pass depth 1e-9 (round 1: `std::bad_alloc` from
    2·10⁹ levels), stepover 1e-6, and chordal 1e-7 on an r = 20 circle (both ran unbounded).

## Golden file
`core/tests/golden/toolpath/pocket_rect_grbl.nc` is AC1's case through `posts/grbl_mm.pp`, in two
3 mm passes. It is regenerated only by `tools/regen_golden.py`, which now also lists
`core/tests/golden/toolpath`. Hand check: C = [3.01, 46.99] × [3.01, 26.99] (inset 3 + 0.01). Its
half-height is 11.99, so there are rings at insets 2.4k for k = 0…4 (k = 5 would need 12) and no
extra rings (2.4 ≤ r). Each ring's corner coordinates in the file were compared with 3.01 + 2.4k
and 46.99/26.99 − 2.4k. Each ring is counter-clockwise (climb). Every ring after the first starts
2.4 mm straight below the previous start (x = 12.61). Level Z−3 ends at (12.61, 3.01) and feeds back
over the cleared floor to (12.61, 12.61) before plunging to Z−6. There is one retract. The
posts/README safety checklist holds: G21/G90 first, safe Z before the first XY rapid, M3 before
cutting, M5 and M30 at the end, no arcs. `pocket_golden_test.cpp` encodes the same derivation
move by move (to Clipper2's 1e-6 mm grid), independently of the offset code.

## Consequences
- Coverage, island clearance, rest locality and gouge tests use an independent brute-force
  oracle (`core/tests/toolpath/pocket_oracle.hpp`). It does its own 0.1 mm rasterisation of the
  swept disc, point-in-polygon and segment distances, and does not touch the offset or boolean
  code.
- Arcs are not refitted, so arc moves never appear. A later "Fit Curves" pass could convert rings
  back to arcs.
- Lifts are vertical `Feed` moves, not `Retract`s. They never leave the pocket and travel only
  inside the centre region, above a cleared floor. AC5 counts retracts to safe Z.
- Raster and rest travel went straight only, so a raster pocket split by islands, or rest clusters
  far apart, retracted between runs; and rest machining built the tool's full pattern and then
  clipped it. Both resolved by T-0123 (ADR-0018 §9, §10).
- A self-overlapping input (two coincident edges of opposite direction, as in the verifier's
  "spiral" scene) is a zero-width fold with pocket on both sides under even-odd and non-zero
  alike. The pocket treats it as open, so checkers must not count such folds as walls.
- The Clipper2 inversion in (4) also affects other callers of `offsetContours` with large
  negative distances. It should be fixed or guarded in geometry (a director card, outside
  T-0017's open-offset scope).

## Verification notes (T-0125 round 2, accepted by the director 2026-10-31; updated by T-0126)
- After a convex vertex is pulled in or removed, the guard re-checked vertices, not the new edges. With hand-made insets a removed vertex on a concave join arc let the new chord sag up to about four times the arc step inside the limit (≈ chordal). Never observed with real Clipper output (all removals were zero-width spikes; 0 edges closer than r + allowance over ~330,000 diameters). **Resolved by T-0126:** changed edges are checked exactly and repaired (§4).
- The phantom test counted vertices, not area, and a successful inset repair could lose area without a message at coarse chordal tolerances; neither occurred in real scans. **Resolved by T-0126:** depth test, and a "trimmed" message for repairs larger than the join tolerance (§4).
- At stepover = diameter, travel between levels can cross the thin scallops left between offset rings (still ≥ r + margin from walls).
