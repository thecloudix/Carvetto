# ADR-0096: an exact capsule-union judge for the tab layer's "no waste near" answer

Status: accepted (T-0279; closes ADR-0016 section 14 residual (l); (i) moved to T-0318; (j) argued, not measured)

## Context
`ProfileBuilder::tabHolds` (Inside/Outside, allowance >= 0) decides on a raster (0.05 mm cells) whether
a tab still joins the part to the waste beyond the kerf. Its waste is the cells further than
`c.distance + outer + 0.2 mm` from the vectors. When no cell is, it answers "holds" (a hole smaller than
the kerf: nothing to hold on to). A sliver of waste narrower than about 0.36 mm between two kerfs (the
U-part family of T-0167: a notch leaving 0.02-0.45 mm between the end mill's two passes) has no such
cell, so the answer "holds" is returned for a tab whose end tools seal it off from the sliver: the exact
flood oracle (the union of the swept tool footprints, harness o11) finds it severed. T-0167 tried widening
the discs by half a cell; on the U family it removed 148 severed tabs, dropped 32 holding ones and freed a
426 mm2 part (seed 9735 scene 49), because a rule decided by cells cannot be made safe.

Measured on main (U family, seeds 9731-9738 x 500 scenes, end mill, Outside, leads off): 209 scenes keep a
tab the oracle finds severed; in 204 of them the tab was kept by the "no waste near" answer (an
instrumented build), in the other 5 by a different path. 29 parts come free (all reported by the exact
loose-piece search).

## Decision
Replace the "no waste near" answer by an exact judgement, and only that answer (every other result of
`tabHolds`, the quick answers and the sided judge stay byte-identical).

1. **The world.** The tools at the tab layer as exact capsules: every run of consecutive open samples of one
   cut (at most 0.05 mm of arc length apart) is the band of the polyline through them
   (`detail::bandOutline`, radius tab-layer radius + 0.5 um, the oracle's own slack), a lone sample a disc,
   a V-bit's corner-move tool position a disc of its radius. As the flood oracle does, the raised stretches
   of the other live tabs count as cut (a tab holds on its own and never borrows another tab's bridge), and
   the corner moves a live tab of another group covers are not made (`cornerClass`). The window is the
   raster's box (the core inflated by `rasterReach`), closed by a frame of four bars so every piece inside
   it is bounded (`detail::enclosedPieces`).
2. **Pieces.** `before` = the pieces of the window (area >= 0.01 mm2, the oracle's rule) with the tab as
   material; `after` = the same with the tab's own capsule and corner moves added. The tab joins two pieces
   exactly when lowering it splits one, i.e. `after > before`.
3. **Verdict.** `after > before`: holds. Otherwise, with another piece in the window (`before >= 2`): the
   tab joins at most the part and nothing else, severed (dropped as cut through; it holds nothing, so
   dropping it frees nothing, and the loose-piece search is exact as before). Otherwise (`before <= 1`: no
   waste at all near the tab): holds, the old answer for a hole smaller than the kerf.
   *Soundness.* "Holds" by the split is a bridge of the world in which the other tabs are cut, which is
   the oracle's world; "severed" is claimed only in that same world. A tab that is a bridge when the other
   tabs count as material is a bridge when they are cut, not the other way round, which is why the other
   tabs are cut (a severed verdict then cannot drop a tab the oracle finds holding).
4. **Bounds.** Run only when the raster found no waste; the rings are charged to the work meter
   (60 units per vertex, as the loose-piece search), the leaf-wise `enclosedPieces` is near-linear.

## Residuals (i), (j), (l)
- (l) the notch's bottom-corner tab: fixed by the above (the test builds the U part; the oracle's verdict
  and the product's agree).
- (i): moved to T-0318, which was closed won't-fix (accepted by the director; the benign class already recorded
  in ADR-0016, "kept tabs that hold nothing"). Correction: scenes m3_9301_153 and m3_9308_412 are not absent;
  they exist in the verifier harness (/home/chris/s167/scn), only not in the archive this note was written
  from. m3_9308_412 is the benign partial-tab "comes free" case. m3_9301_153 (V-bit 60 deg, D25, SHARP,
  Outside, allowance 0.3867, 3 of 9 tabs) is the known pocket-of-waste case: a kept tab holds nothing and
  nothing is freed. Root cause: the raster "holds" answer never reaches the exact capsule judge. A 650-scene
  V-bit SHARP screen still shows three oracle-severed kept tabs (9308 scenes 13, 63, 69), identical on main
  and on this branch, so they are not caused or cured by this judge. Option B (extend the exact capsule judge
  to cover this path and rescreen about 24k scenes) is deferred.
- (j): tabs left out where another place would hold (mmrg_001_tabs_all_out, m1_272009_9). The argument that
  this is not a judge defect is: the judge answers only the "no waste near" case and only ever returns
  holds or severed in the oracle's own world, so it cannot leave out a tab the oracle finds holding; a tab
  left out where another place would hold is the placement's choice. This was not measured in this round: the
  two scenes are not in the archive, and no harness output is on hand that shows it; treat (j) as argued,
  not measured. It stays listed in ADR-0016 section 14.
- ADR-0016 section 14's other residual (f) is unchanged (9302_2199 gives the same result as before).

## Testing the judge
The judge is a free function (`detail::exactNoWasteHolds`, `exact_tab_judge.hpp`) taking the window, an
`ExactTools` (a plain data struct), the tab-layer radius and the tolerances, and the choice of which side a
sample goes to is `detail::classifyTool`. `exact_tab_judge_test.cpp` drives both with hand-built fixtures: two
bands with a gap, which a disc of the full radius seals and one of half does not. The scene tests of
`profile_tab_layer_test.cpp` pin the integration.

## Mutants
Seventeen one-line mutants of the judge and its inputs were applied by hand-driven scripts
(`scratch-t0279/mut3.sh`, not committed), each run against the T-0279 tests and, if those passed, `[tabs]`.
All are killed:
- m01 (`after >= before`), m02 (verdict always false), m03 (always true), m12 (`after < before`), m13 (own
  runs not added when lowered), m16 (own corner discs not added), m17 (cut corner discs not added), m14 (a
  lone sample's disc of half the radius), m06 (the tab's own corner moves counted before it is lowered) and
  m07 (corner moves covered by another live tab counted): the hand-built fixtures and the classification test.
- m08 (no linking of samples into runs), m15 (bands 0.03 mm too wide), m04 (`before < 3`), m05 (other live
  tabs' raised stretches counted as material), m09 (bands 0.03 mm too narrow), m10 (minimum piece 0.1 mm2),
  m11 (frame without its bottom bar): the scene test "the judge's inputs", one scene each (a scene found by
  running each mutant over 2400 U-family scenes and keeping one whose result differs and in which the flood
  oracle sides with the unmutated judge).
In an earlier round m06, m07 and m14 survived the scene tests (the corpus rarely reaches those branches: a
counter hit 0 times for m07 and 12 and 14 times for m06 and m14 in 12000 scenes) and were only argued
unreachable; they are now killed by hand-built fixtures instead of argued.

## Results
(Run by the harness of the verifier's flood oracle o11, end mill unless stated, Outside, leads off.)
- About 24.7k generated scenes (U family seeds 9731-9738 and further families) compared against the flood
  oracle: no new report of a loose piece or a severed kept tab that main did not already have.
- U family, seeds 9731-9738 x 500: scenes keeping a tab the oracle finds severed fell from 209 to 30; the
  remaining 30 keep it by other paths (not the "no waste near" answer).
- Seed 9735 scene 49 (the part the widened-cells rule freed): stays held together.
- ADR-0016 section 14: (l) fixed; (i) moved to T-0318; (j) argued not a judge defect, not measured; (f) unchanged.
- The flood-oracle numbers above (209 -> 30, about 24.7k scenes) come from the earlier harness runs and were
  not re-measured in round 2; the round 2 change only moves code into `exact_tab_judge.cpp` (the lone-sample
  disc is the same polygon) and adds tests.
