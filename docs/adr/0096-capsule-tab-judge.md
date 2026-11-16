# ADR-0096: an exact capsule-union judge for the tab layer's "no waste near" answer

Status: accepted (T-0279; closes ADR-0016 section 14 residuals (i), (j), (l) as far as they are judge defects)

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
- (j): argued not to be a defect of this judge (the verdict the judge gives agrees with the oracle's on the
  scene; the remaining difference lies elsewhere in the loose-piece reporting); not changed here.
- (i): not reproduced. The scenes the card names (9301_153, 9308_412) are absent from the archive the
  harness had, so nothing could be measured or tested; the residual stays open.
- ADR-0016 section 14's other residual (f) is unchanged (9302_2199 gives the same result as before).

## Mutants
Fifteen one-line mutants of `exactNoWasteHolds` and `exactTools` were applied by hand-driven scripts
(`scratch-t0279/mut2.sh`, not committed), each run against the T-0279 tests and, if those passed, `[tabs]`.

Killed (test failures):
- m01 (`after >= before`), m02 (verdict always false), m03 (verdict always true), m08 (no linking of samples
  into runs), m12 (`after < before`), m13 (own runs not added when lowered), m15 (bands 0.03 mm too wide):
  the three first T-0279 tests.
- m04 (`before < 3` for `< 2`), m05 (other live tabs' raised stretches counted as material), m09 (bands 0.03
  mm too narrow), m10 (minimum piece 0.1 mm2 for 0.01), m11 (frame without its bottom bar): the fourth test,
  "the judge's inputs", one scene each (found by running each mutant over 2400 U-family scenes and keeping a
  scene whose result differs and in which the flood oracle sides with the unmutated judge).

Not killed (all tests pass): m06 (the tab's own V-bit corner moves counted already in the world before the
tab is lowered), m07 (corner moves covered by another live tab counted), m14 (a lone open sample's disc
shrunk to half the radius).
- m07: unreachable in the corpus. The branch needs a V-bit corner move covered by another live tab inside the
  raster window of a tab that sees no waste; the counter hit 0 times in 12000 sharp-corner V-bit U scenes
  (seeds 9741-9780 x 300) and the mutant changed no result in them or in 2400 + 2400 others.
- m06: own corner moves occurred in 12 of 12000 such scenes (hundreds of discs each), and the mutant changed
  no result in them: there the tab either holds or is judged with no other piece regardless.
- m14: a lone open sample occurred 14 times in the same 12000 scenes and the mutant changed no result.
No sound equivalence argument was found for m06 and m14, and no small scene was built that kills them; they
are left as open and reported to the director rather than claimed equivalent.

## Results
(Run by the harness of the verifier's flood oracle o11, end mill unless stated, Outside, leads off.)
- About 24.7k generated scenes (U family seeds 9731-9738 and further families) compared against the flood
  oracle: no new report of a loose piece or a severed kept tab that main did not already have.
- U family, seeds 9731-9738 x 500: scenes keeping a tab the oracle finds severed fell from 209 to 30; the
  remaining 30 keep it by other paths (not the "no waste near" answer).
- Seed 9735 scene 49 (the part the widened-cells rule freed): stays held together.
- ADR-0016 section 14: (l) fixed; (j) not a judge defect; (i) not reproduced (scenes absent); (f) unchanged.
