# ADR-0017: Pocket toolpath

Date: 2026-10-26 · Status: proposed (T-0121)

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
   and an empty toolpath for that tool. It never throws. With more than one tool, each toolpath is
   named `"<name> [<tool name>]"` (the inventory's tool-inclusive naming).
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
   offset goes through `insetBy`, which drops any result region with an outer vertex closer than
   the inset (less the join tolerance) to its source. Clipper2 1.4.0 turns a polygon inset past its
   inradius inside out: a 17 mm circle inset by 19.2 comes back as a 2.2 mm circle, by 30 as a
   13 mm one. Without the guard, offset levels never end, and a tool too big for a small pocket
   would get a phantom region to cut.
5. **Offset strategy.** The levels are `inset(C, k·stepover)`, each taken from C directly so
   errors do not accumulate. They are grouped into regions (outer + holes) and arranged as a
   containment tree, which is cut in **post-order (inside out)**. The innermost rings come first
   and the outermost ring, the wall, comes last as the finishing pass. When stepover > radius,
   some points of a level deeper than the radius may be left uncut where the next level does not
   reach. Wherever that residual (opened by 5·chordal to drop slivers) is non-empty, an extra ring
   `inset(C, k·stepover + radius)` is inserted. Rings of depth ≤ r cover their band, and the extra
   ring covers depths up to 2r ≥ stepover, so coverage is complete for every stepover ≤ diameter.
   Ring direction: **climb puts the material on the right** (spindle clockwise seen from above).
   Outer walls are cut counter-clockwise and island walls clockwise; conventional reverses both.
   Each ring starts at its point nearest the current position. That point is inserted when it
   falls inside an edge, and ties go to the lowest y, then x, so the choice never depends on
   Clipper2's vertex order.
6. **Profile Pass with the Offset strategy.** The outermost Offset ring *is* the boundary pass
   and is always cut last. `profilePass` applies to Raster only. (This is our reading of the
   inventory, which lists Profile Pass beside both strategies. If the owner wants First/None for
   Offset, a later card can add it without changing the API.)
7. **Raster strategy.** Lines at `rasterAngle`, at most `stepover` apart and evenly spread over
   the region's extent (first and last half a pitch in), are clipped to each connected part of C
   with Clipper2's open-path intersection. The clipped ends are projected back onto the exact
   line, so every piece is exactly parallel. Pieces are linked zig-zag (each row's nearest piece
   when the link is allowed, see 8). Otherwise the next unused piece in row order starts a new
   run. The boundary pass (all rings of the part, oriented as in 5) is cut first, last or not at
   all, at every level.
8. **Linking.** A link is a straight feed when it stays inside C. The test is our own exact
   segment-vs-polygon check (no edge crossed away from the ends, midpoint inside). Within one depth
   level the link may also be no longer than **2 × diameter**: it may cross uncut material, which
   is no worse than the slotting of the first ring. Every depth level of a cluster (a connected
   part of C) is cut before the next cluster. For the clearance tool, the move from the end of
   level k back to the cluster's first path is a feed at level k (the whole level is cleared)
   when it stays inside C, followed by the descent. Anything else retracts to safe Z, rapids, and
   descends. A convex pocket therefore retracts once in total, and n disjoint simple regions
   retract n times (AC5).
9. **Descent.** It is a vertical `Plunge`. With `rampPlunge`, the tool plunges through the air to
   the floor already cut (the surface for the first level) and then makes a **zig-zag ramp**
   (`toolpath/pocket_ramp.hpp`, `appendPathRamp`). The ramp runs forward along the path for
   min(rampDistance, path length) down to the midway height, then back over the same points,
   arriving at the start exactly at the level. All ramp moves are `Feed` moves on the path, so a
   ramp can never gouge. Paths shorter than 10·tolerance.distance fall back to a plunge. This is
   pocket-specific and separate from Profile's ramp family (ADR-0016); unifying them is left to a
   later card.
10. **Rest machining.** Tools run in list order; the first usable tool is the clearance tool. The
    rest material for tool i is `opened(inset(R, allowance + margin) − swept, 5·chordal)`. Here
    `swept` is the union of the earlier tools' cut areas: `C₀ ⊕ r₀` for the clearance tool, and
    `(N ∩ C) ⊕ r` for each earlier rest tool. Tool i builds its normal pattern over its own C_i
    and clips every path, as an open path, to `N_i = offset(rest, r_i + 5·chordal)`, the centres
    whose disc reaches the rest material. N_i is deliberately *not* intersected with C_i: the
    outermost ring lies on C_i's boundary, where open-path clipping is ambiguous and drops pieces.
    Pieces keep their path's direction and order, a ring cut across its seam is rejoined, and the
    pieces are clustered by the connected part of N_i that holds them. Between levels a rest tool
    always retracts, because its region is only partly cleared.
11. **Output.** Moves are straight lines only, because the geometry is polygonal. Rapids are at
    safe Z, every toolpath starts with a rapid and ends retracted, and the output is
    deterministic (no unordered containers, ties broken geometrically).

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
- The 2 × diameter link limit and the inside-C test are simple and conservative. Raster links do
  not follow the boundary, so a raster pocket with walls nearly parallel to the raster angle
  retracts more than necessary.
- The inset guard costs O(vertices × edges) per inset, capped at about 4·10⁶ distance
  evaluations by sampling vertices. A spatial index would remove the cap if very large pockets
  need it.
- The Clipper2 inversion in (4) also affects other callers of `offsetContours` with large
  negative distances. It should be fixed or guarded in geometry (a director card, outside
  T-0017's open-offset scope).
