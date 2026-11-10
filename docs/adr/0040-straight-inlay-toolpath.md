# ADR-0040: Straight inlay toolpath (geometry and conventions)

Date: 2026-11-10 · Status: accepted, implemented (T-0238, first slice).

## Context
The inventory lists "Inlay Toolpath" (straight inlays): male inlay Straight or Stepped (Step Depth,
Step Width), female inlay Pocket or Hole, an allowance offset / pocket allowance, and a female pocket
with automatic corner radiusing. The card requires that the male and female pieces of one vector mate
within the stated allowance. The card also covers the VCarve inlay (plug, pocket, glue gap, surface
clearance, plug destination sheet, mirrored plug, plug boundary); that is split off as its own card
(see BACKLOG) and not part of this ADR.

## Decision
1. **One toolpath per piece.** A toolpath cuts either the Male or the Female piece of the selected
   vectors (`InlayParams::piece`); the user adds one of each. Both are computed from the same shapes
   (`inlayShapes`), which is what makes them mate.
2. **Corner radiusing.** With a round flat tool of radius r a pocket leaves convex corners of radius
   r (morphological opening of the vector region) and an outside profile leaves concave corners of
   radius r (closing). The common shape is closing(opening(region)) with Round offsets: the female
   cavity. "Automatic corner radiusing" of the inventory is this; it applies to both pieces so they
   still fit. Only flat end mills are accepted (V-bit and ball-nose plugs belong to the VCarve slice).
3. **Allowance.** `allowance` >= 0 is the gap all round, taken from the male: male = cavity inset by
   the allowance (Round offset), female = cavity. A male whose allowance consumes the shape is
   refused. The inventory's "Pocket Allowance" (material left on pocket walls) is not the same thing
   and is not offered: a gap on the female would only shrink the plug the other way.
4. **Female.** Pocket: `pocketToolpath` (Offset strategy, boundary pass last) of the cavity grown
   by the chordal tolerance, because the pocket keeps a safety margin of that size from its walls, so
   the walls land on the cavity. Hole: an inside profile of the cavity, cut through `cutDepth`.
5. **Male.** Straight: an outside profile of the plug over `cutDepth`. Stepped: a shoulder, the top
   `stepDepth` of the plug being `stepWidth` wider all round. The shoulder level is an outside profile
   `stepWidth` further out; the levels below are cut as rings from `stepWidth` down to the plug
   (spacing 0.9 tool diameters, outermost first, the plug outline last), so material under the
   shoulder is cleared and it stands free. More than 1000 rings (a step very wide for a tiny tool) is
   refused. The inventory does not say which way the step faces; this reading (shoulder at the top
   of the plug) is the one implemented and is to be confirmed by the owner.
6. **Reuse.** All cutting goes through `profileToolpath` / `pocketToolpath`: passes, order, ramps,
   limits, cancel and diagnostics are theirs. Input limits (closed vectors only, 100,000 spans,
   coordinates within the profile limit) are checked before the shapes are offset.
7. **Not in this slice.** Tabs on the male cut-out, ramps and leads, separate last pass, rest
   machining, the V-carve inlay.
8. **Record and form.** `inlay` record in job.json only (like chamfer and thread mill); an Inlay form
   (step fields only for a stepped male, female style only for a female).
