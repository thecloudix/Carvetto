# ADR-0115: Rotary layout tools: Rounding toolpath, Wrapped Fluting Layout, Wrapped Spiral Layout, format 1.19

Date: 2026-11-16 · Status: proposed (T-0322) · Builds on ADR-0098 (rotary job), ADR-0103 (wrapped post, `CylinderMap`),
ADR-0041 (Fluting toolpath)

## Context
`docs/reference/feature-inventory.md` (section 6, "Wrapping submenu") names three native rotary tools: Create
Rounding Toolpath (blank square or round; method Radial / Raster / Optimized Raster; Allowance; tool; name), Wrapped
Fluting Layout (number of flutes, start and end distances, coves) and Wrapped Spiral Layout (number of strands, start
and end distances, Spiral Pitch or spacing between strands, twist direction, coves). It does not give the geometry, so
the readings below are mine and are listed for owner review. As ADR-0098/0103 fix, everything is computed flat in
the unwrapped design space (X/Y = distance along the axis and distance round the circumference, Z = machine height)
and wrapped by the post; nothing here is cylinder-aware except the angle of a square blank's faces.

## Decisions
1. **Rounding is a toolpath without vectors** (`toolpath::roundingToolpath`, `RoundingParams`). It takes the blank
   radius, the radius to turn to, the extent along the axis and the Z of the surface; the cylinder is the work piece.
   `needsVectors(params)` is false for it, so the job, the form dialog and the session need no selection; selected
   vectors are ignored and not stored. On a job that is not rotary the session refuses it.
2. **Strokes.** Pass k runs at Z = axis + radius_k with the axis at `surfaceZ - blankRadius`. The pass radii go from
   the top (the blank radius, or the already-turned radius for rest machining) to `target + allowance` in equal steps
   of at most the pass depth (`roundingPassRadii`; the pass depth and stepover are the card's own or, at 0, the
   tool's cutting data; a stepover wider than the tool is limited to its diameter). Methods:
   - *Raster*: one full turn per stroke (circumference from `circStart` for 2 pi r), strokes spaced along the axis by
     the stepover: `ceil(length / stepover)` equal steps, so a turn at both ends (steps + 1 turns); the tool lifts
     between turns and every turn runs the same way.
   - *Optimised raster*: the same turns, but the direction alternates and the tool stays down between neighbouring
     turns (a step along the axis at the seam, cutting).
   - *Radial*: strokes along the axis at `ceil(circumference / stepover)` evenly spaced angles (the stepover is the
     arc distance at the blank radius), the direction alternating and the tool staying down between neighbours (a
     step round the circumference at the end). Which of the two "optimised" and "radial" behaviours the reference
     product has is not documented; these three are distinguishable and each is useful.
   A lifted raster makes the post unwind the rotary axis by 360 degrees on every rapid back to the next turn (seen in
   `rounding_wrap_test`); the optimised raster avoids it, so it is the default.
3. **Blank.** Round: a cylinder of the job radius. Square: the square bar inscribed in it (corners on the cylinder,
   side radius x sqrt(2)), faces centred on the angles 0, 90, 180 and 270 degrees (angle = circumference coordinate /
   radius, as `CylinderMap`). At a pass radius rho a stroke is cut only where material is left (the blank radius at
   that angle exceeds rho): the whole turn at or below the inscribed circle, the four corner stretches above it
   (`roundingMaterial`), none at the corners. A square blank of another size is not offered. Rest machining on a
   square blank rested inside the inscribed circle is a round blank.
4. **Allowance** is radial material left on the finished surface. **Rest machining** (`startRadius`): the radius an
   earlier Rounding already turned to (0 = none); it only lowers the top.
5. **Limits.** Radii, lengths and Z within 100 m; at most 1000 passes and 200000 strokes (`WorkLimitExceeded`);
   a safe Z not above the surface is `InvalidParameters`; nothing to remove (target at or above the blank, or no
   material in the pass) is `NoRestMaterial`. Cancel is polled before the run and between passes.
6. **Wrapped Fluting Layout** (`geometry::wrappedFlutingLayout`): `flutes` evenly spaced round the circumference
   (spacing pi D / n), starting at `circStart + offset` (offset taken modulo the turn), each from `alongStart` to
   `alongEnd`. Shapes: centre line (open, for the Fluting toolpath), rectangle, or stadium with half-circle ("cove")
   ends (closed, for Profile/Pocket); the width must not exceed the spacing and a stadium needs length >= width. The
   depth is checked (> 0, < radius) and is the Fluting toolpath's depth. An outline that would leave the flat design
   area at the edge of the turn is reported (`crossingSeam`), not moved: the offset is the user's tool for it.
7. **Wrapped Spiral Layout** (`geometry::wrappedSpiralLayout`): `strands` evenly spaced round the circumference; the
   advance is given as pitch (axis distance per turn), spacing between strands (pitch = spacing x strands) or total
   turns (pitch = length / turns). Because the helix is a straight line in the flat design, a strand is a line of
   slope circumference / pitch, cut into pieces at the edge of the turn so the vectors stay inside the design area
   (a strand of t turns gives about t + 1 vectors, in order). Twist: right- or left-hand as the helix looks on the
   cylinder as `CylinderMap` wraps it (so the sign depends on the axis and Flip Design; the layout takes `flip` and is
   tested against `CylinderMap::wrap`). At most 1000 strands and 100000 vectors, pitch at least 0.001 mm.
8. **.carv 1.19.** `job.json` toolpath operation `rounding` with 15 params keys; written (and the minor raised to
   19) only when a Rounding record exists, so every other file is byte-identical to before; no move-cache change.
9. **Viewmodel.** `ToolpathForm` for Rounding (new forms of a rotary job start from its radius, length and surface,
   and target 80 percent of the radius); `RotaryLayoutForm` (Fluting / Spiral; preview and an apply that adds ONE
   vector object in ONE undo step and selects it; a flat job is refused; the twist is entered as total degrees).
10. **App.** Rounding is in the Toolpaths Add menu and the form dialog (no vector count needed) with a self-test.
    **Not covered (follow-up card):** menu entries and dialogs for the two layouts (the `RotaryLayoutForm` is complete
    and tested; it needs a dialog like the Vector Texture one and a self-test), the Fluting toolpath taking its depth
    from the layout, and a preview of the layouts wrapped in the Auto-Wrapping View.

## Mutation testing
New logic was mutated by hand (each mutant built and run against the `[rounding]` / `[wrapped]` tests); all killed
unless argued here. Killed: safe Z `>`->`>=`; stepover limit `>`->`>=`; the two `- eps` guards on stroke and pass counts
(`2.1 / 0.3` is `7.000000000000001`); raster reversal parity; joining every stretch; radial direction; radial
`previous`; radial angle taken relative to the start; the inscribed-circle boundary; top ignoring the blank radius;
equal-step vs fixed-step pass radii; Z from the surface instead of the axis; axis from the target radius; clipping of the
corner stretches at either end of the turn; `top > bottom + 1`; own stepover / pass depth ignored; retract position;
twist sign for the axis and for Flip Design; spacing -> pitch; first crossing of the seam; sliver drop; the final
piece's end; offset modulo the turn; seam report on the high side; width, stadium-length and depth boundaries; strand
spacing; arc direction of the swapped stadium.
Survivors, argued (equivalent):
- `top > bottom` vs `>=`: with equal radii the pass list is empty, no stroke is made and the same `NoRestMaterial`
  results either way.
- The wide loop over turns in `roundingMaterial` (`first .. first + 3`): only the turns that overlap `[c0, c0 + turn]`
  can add a stretch, and a stretch half-width is below a quarter turn, so the outer iterations never add one; they
  are kept as a margin against rounding.
- The corner presence test `> rho + 1e-9` and the half-width threshold: they differ from the exact comparison only at
  a radius equal to the blank's to within a nano-millimetre, where the cut would touch nothing.
- The `t <= 1e-9` bump in the spiral (a strand starting a hair before the seam): without it the first crossing is a
  sliver that the sliver drop removes, and the next piece starts at the end of the turn either way.
- The cancel polls between passes: with the flag never set the output is identical, as in ADR-0111; the poll at the
  start is tested.

## Readings for owner review
Method semantics (item 2), the inscribed square blank and face angles (3), the unwinding rapids of the lifted raster
(2), the twist-direction convention (7), the offset and seam policy (6), and the stepover of Radial being an arc distance.
