# ADR-0114: Wrapping level effect, Vector Unwrapper, rotary 3D import

Date: 2026-11-16 · Status: proposed (T-0321)

## Context
T-0319 made rotary jobs (ADR-0098) and T-0320 wrapped their toolpaths, view and simulation onto the cylinder
(ADR-0103, `geometry::CylinderMap`). Level effects (ADR-0079) already carry Clipping and Mirror Mode. T-0321 adds
the three remaining rotary modelling pieces named in `docs/reference/feature-inventory.md` ("Rotary, wrapped and
double-sided machining": the **Wrapping** level effect, the **Vector Unwrapper**, **Importing a 3D Model into a
Rotary job**). The inventory gives one line for each, so the behaviours below are my readings, listed for owner
review.

## Decisions: Wrapping effect
1. **Reading.** The inventory says Wrapping "wraps components that run past the edge". Read as: the relief is modelled
   flat on the design area; the circumference direction (design Y for a cylinder along X, X for along Y) is one turn
   long, so whatever lies beyond the window `[start, start + circumference)` is what the cylinder carries on
   the other side. The effect folds the composed level field: a cell inside the window takes the **highest opaque**
   value of the field at its circumference coordinate plus any whole number of circumferences; cells outside the
   window become transparent. *Alternatives for the owner*: overlapping turns could add instead of merge high
   (a sum would double the height where a relief overlaps itself); cells outside the window could keep the unwrapped
   original instead of becoming transparent.
2. **Where it lives.** `relief::Wrapping { enabled, circumferenceIsY, circumference, start }` is a member of
   `LevelEffects` (so `any()`, validation, compose, bake, undo and the file format all follow the existing effect
   plumbing). The core does not know the job: the circumference and window start are stored with the effect, and
   `viewmodel::setLevelWrapping` fills them from the rotary job (circumference of the job, start = the minimum of the
   flat design area on the circumference axis). A level wrapped for one job keeps its numbers if the job is later
   resized until the user turns the effect off and on again.
3. **Order.** Mirror, then wrapping, then clip: the mirror works on the flat model (as the user drew it), the wrap
   folds the result, the clip bounds what the level contributes, in the wrapped window. The level base height is
   still added before all effects and is not part of a bake.
4. **Sampling.** Along the circumference, linear interpolation; exactly the cell value when the copy lands on a cell
   centre (checked with a 1e-9 cell tolerance, so a whole-cell circumference is exact, no bilinear weight noise); a
   transparent neighbour with weight makes the copy transparent. Across the circumference nothing is resampled. A
   field spanning more than `kMaxWrapPeriods` (64) turns is `BadParameter` (it bounds the cost to 64 samples per cell).
5. **Command and bake.** `SetReliefLevelWrappingCommand` (Apply, Update, Remove with `enabled` false; refused for an
   unknown level, invalid numbers or no change; does not coalesce). `BakeReliefLevelEffectsCommand` already composes the
   level with all its effects, so the bake equals the composition (tested) and clears the wrapping with the rest.
   Bake enabling in the app panel (`sculpt_panel.cpp`) and a Wrapping toggle are app work, see Not done.
6. **Format 1.20.** The level `effects` record gets an optional `wrapping` object
   (`circumference_axis` `"x"|"y"`, `circumference`, `start`, all required inside it) written only when enabled; the minor
   is 20 only then, so every other file keeps its bytes. The record's existing keys stay required
   (`requireAll` became three explicit `require` calls so a 1.10 effects record still loads). Hostile cases: bad axis,
   zero/negative/non-finite/non-numeric circumference or start, duplicate and missing keys; unknown keys are skipped.
   `modelFingerprint` (toolpath recalculation) and the merged sculpt preview (full redo, like a mirror) see the effect.

## Decisions: Vector Unwrapper
7. **Reading.** "Turns a cross-section into a contour for two-rail rotary modelling": the cross-section is drawn in the
   end view round a centre; the unwrapper gives its polar form as a contour (X = angle * cylinder radius, the
   circumference coordinate of `CylinderMap::circOf`; Y = distance from the centre), which is the profile a two-rail
   sweep takes. The angle is counter-clockwise from +X in the drawing, **continued across the seam** (the
   contour never jumps by a turn); a point on the centre keeps the previous angle; Flip Design negates X.
   A closed cross-section that goes round the centre comes out **open** with its start point repeated one turn further
   (the whole sweep is kept; the turn count is reported); one that does not enclose the centre stays closed.
   `wrapCrossSection` is the exact inverse on the vertices (round trip tested).
8. **Centre options** (`UnwrapCentre`): Contour (the centre of the contour's bounding box; the centroid is the
   alternative reading), Workspace (the middle of the flat design area), Custom (a point). **Refine unwrapped vectors**
   inserts points by bisection until, at a quarter, half and three quarters of each piece, the straight chord of the
   unrolled piece is within the tolerance of the true curve; it is bounded by depth 24 and `kMaxUnwrapPoints`
   (200000, an error rather than a truncation).
9. **Form.** `viewmodel::unwrapSelectedVectors` unwraps every selected vector of the active sheet (curves flattened to
   the tolerance), adds the results as new objects on the source layer, selected, one undo step; the originals stay.
   The result is not placed on the design area (the profile is a free 2D drawing; the user moves it where the sweep
   needs it). Rotary jobs only.

## Decisions: rotary 3D import
10. **Full 3D model.** `relief::meshToRotaryField` maps a mesh whose axis lies along its X (cylinder along X) or Y:
    each cell of the flat design area (along the axis, round the circumference; angle = c / radius via
    `CylinderMap::angleOf`, so Flip Design applies) casts a ray from the axis outward at that angle and takes the
    **farthest** hit; the cell value is that distance minus `zeroRadius` (0 by default, so the value is the radius,
    which the wrapped view adds to the model base; with Z zero at the cylinder axis and no inside gap the model base is
    the axis). Cells without a hit are transparent. Surfaces seen edge-on (parallel to the ray) are skipped, rays on a
    shared edge use a 1e-9 barycentric tolerance so no cracks open. Work: a triangle is tested only against the
    columns of its along range and the rows of its angular range (a triangle that spans pi or more of angle, such as one
    round the axis, against the whole ring); `RotaryRaster::rayTests` counts the tests and
    `RotaryRasterOptions::maxRayTests` (400 million) bounds it; cancel is polled per 256 triangles.
11. **Flat model.** Needs no new core code: the mesh is rastered top-down onto the flat design area with the ordinary
    `meshToHeightField` (so what the user modelled flat is wrapped by the job, view and toolpaths), centred on the area.
12. **Form** (`viewmodel::importMeshIntoRotaryJob`): uses `prepareMesh` for units, orientation (top face, quarter turns),
    size; **Scale model to fit material** scales uniformly so the length fits the job length and the largest vertex
    distance from the axis the job radius (it also scales up); the axis passes through the middle of the model's
    cross-section, moved by **Rotation Axis Movement** (a shift in mesh mm); the model is centred along the axis or
    starts a given distance from the start of the design area. The component is added on top of the sheet's top level
    (one undo step, shared `addImportedComponent` with the flat import).

## Not done (follow-up)
App dialogs and self-test steps: a Wrapping toggle and Bake enabling in `LevelEffectsPanel`, the Unwrapper form, the
rotary import dialog with its interactive rotation rings, orientation gizmo, black axis cube, "Resize material block"
and "Reset to Original Size". Re-import (right-click, Reimport) belongs with T-0301's flat import. These sit on the
viewmodel functions above, which are covered by tests.

## Tests and mutants
Operation counters: wrapping cost bounded by the periods cap (exact boundary tested); ray tests equal an independent
brute-force count for a small triangle and `columns * rows` for a triangle round the axis; the refinement point count
is deterministic and grows as the tolerance shrinks; `maxRayTests` exact boundary. Wrapping equals the direct wrapped
surface (max over turns, computed independently) including a hole, a transposed grid, a window start and a fractional
circumference; the bake equals the composition; the raster of a prism equals the analytic polygon radius at every cell,
along X and Y (transpose), with flip, axis movement, along placement and zero radius.

Mutants run by hand (each killed unless listed below): window end `<` -> `<=`; window start `>=` -> `>`; merge high ->
merge low; wrap-before-mirror order; circumference validation `> 0` -> `>= 0`; periods cap off by one; interpolation weights
swapped; circumference axis transposed; ray raster: zero radius ignored, discard ignored, last hit instead of farthest,
flip ignored, turn offsets ignored, along placement ignored, axis point ignored (each coordinate, and swapped), work cap
off by one, no full ring for a triangle round the axis; unwrapper: no seam continuation, flip ignored, turns ignored,
refinement tolerance x4, contour centre at the corner, workspace centre replaced, centre point angle reset, closing segment
dropped, inverse without flip. Equivalent, argued:
- `kLo` `ceil` -> `floor` and `kHi` `floor` -> `ceil` (wrapping): they only widen the turn range by one, and every
  extra copy is rejected by the range test, so the result is the same.
- The `f > n - 1` guard in the interpolating branch and the `isnan(v0) || isnan(v1)` test: `f` cannot exceed `n - 1`
  by more than the snapping tolerance given the turn range, and a NaN neighbour already propagates through the
  weighted sum; both are defensive.
- `dist < -kEps` skip (rotary raster): a negative distance is discarded when the cell is read back (`d < 0` means
  no hit), so accepting it changes nothing.
- The early `points > kMaxUnwrapPoints` check (unwrapper): the same bound is applied to the output, which is never
  smaller than the input.
- The 1e-9 tolerances of the ray test (`bu`, `bv` barycentrics) and the `ceil(... - eps)` of the row range (rotary
  raster) are rounding guards: the test with rays exactly at the vertex angles of a prism and the one with a ray on a
  shared diagonal pass with and without them on this platform, because the neighbouring triangles (and the angular
  range, which is already widened by the same epsilon) cover a ray that lies on an edge to within rounding. They are
  kept so a ray on an edge cannot fall into a crack on another compiler or FPU; they differ from the strict test only
  for rays within 1e-9 of an edge.
- `n <= nMax` -> `n < nMax` (turn offsets of the angular range): `nMax` is a ceiling and the next offset's range is
  empty or outside the grid, so the loop ends one step early with the same rows.

## Round 2 notes

- The unwrapper tolerance is approximate: refinement samples each piece at 1/4, 1/2 and 3/4 and splits when any is off
  by more than the tolerance, so the deviation between samples can exceed it by a fraction of a percent (about 0.4%
  near the centre). Tests pin the three samples and allow 0.5%.
- Range guards: the unwrapper refuses points, centres and radii beyond 1e100 mm (so the bounding-box centre and the
  squares cannot overflow) and any non-finite result; `meshToRotaryField` refuses vertices and placements beyond 1e12 mm
  and clamps index arithmetic before integer casts, so nothing is silently dropped or cast out of range.
- A wrapping circumference below `kMinWrapCircumference` (1e-6 mm) is invalid in the command and refused at `.carv`
  load (it could never fit `kMaxWrapPeriods` turns, so compose would fail on a loaded file).
- `importMeshIntoRotaryJob` (Full 3D) refuses a model that hits no cell of the cylinder instead of adding an empty
  component.
