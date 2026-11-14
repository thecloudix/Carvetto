# ADR-0070: Sculpting engine

Status: accepted (T-0294)

## Decisions
- `relief/sculpt.hpp`. A stroke is a list of `Dab`s (position, weight, movement since the previous
  dab) applied in order to a working copy held by `SculptSession`. The session holds the baseline
  (Undo Brush reference and Discard target), the working field and a bounded per-stroke undo
  stack. Keep makes the working copy the baseline and clears the history; Discard restores the
  baseline bit for bit. Memory = two fields plus at most `maxUndoBytes` (default 64 MiB) of
  patches: one patch per stroke holds the pre-stroke values of the stroke's cell box, the oldest
  are dropped over the cap, and a single stroke whose box alone exceeds the cap evicts itself and
  the older history (an inexact undo is never offered). `undoStroke` is exact.
- `dabsAlong` turns a polyline into dabs by arc length (midpoint rule): n = ceil(length/spacing)
  equal pieces, one dab at each piece midpoint with weight piece/diameter and delta = the piece.
  The result depends only on the path geometry, not on its vertex density, and the weights sum
  to length/diameter whatever the spacing. A lone point is one dab of weight 1.
- Kernel: 1 inside radius*(1-smoothness), smoothstep to 0 at the radius. Smoothness 0 is a hard
  edge (0 exactly on the radius).
- Deposit / Remove add / subtract strength * kernel * weight (strength is mm per diameter of
  travel). Smooth blends toward a 3x3 binomial blur of the pre-dab values (transparent neighbours
  dropped, weights renormalised) by 1-(1-s*kernel)^weight, so repeated dabs compound exactly.
  Smudge is a backward warp of the pre-dab values by s*kernel*delta (bilinear, transparent
  contributors dropped); no movement does nothing. Undo Brush blends toward the baseline by the
  same compounding factor and sets it exactly when the factor is 1; with a transparent value on
  either side it picks the baseline when the factor is >= 0.5, else keeps the current value.
  Smooth/Smudge/Undo strength is clamped to 1.
- Modes: Raise keeps max(old, new), Lower min(old, new), Normal the brush result. Preserve
  transparency leaves transparent cells untouched; when off, Deposit/Remove treat them as 0,
  Smooth fills them from opaque neighbours and Smudge from the displaced source.
- Custom brush: any `HeightField` (a component or a bitmap relief) replaces the kernel. Its
  opaque range is normalised to [0,1] (transparent = 0, a flat stamp = 1), scaled so its longer
  side equals the diameter, rotated by `angleRad` plus, with dynamic rotation, the dab's movement
  direction. The reference's "Merge" option, overlay image, presets, constrain, tablet pressure
  and Shift-erase belong to the UI task (T-0302), which maps them onto these primitives.
- Errors reuse relief `Status` (BadParameter for non-finite or out-of-range brush data; the
  working field is untouched on error).
- Independence from the sampling rate is approximate by construction (a Riemann sum / compounded
  blend). Tested tolerances: Deposit with smoothness 1, spacing diameter/8 vs diameter/64 differs
  by < 2 % of strength per cell and < 0.5 % in volume; Smooth at spacing diameter/8 vs
  diameter/32 by < 0.05 on a 0..1 field.

## Mutants
About 40 hand-made mutants of `sculpt.cpp` were run; all but the following were killed (kernel
radius and plateau comparisons, smoothstep shape, deposit sign/weight/transparent base, preserve
flag, blur weights and transparent neighbours, smudge direction/strength clamp/source box margin,
undo-brush threshold and exact restore, Raise/Lower, NaN handling in modes, custom-profile
rotation signs, flat stamp, dynamic rotation, dab count/weight/midpoints, undo cap boundary,
keep/discard, parameter bounds).
Equivalent mutants:
- `t < inner` for `t <= inner` in the kernel: both give 1 at t == inner (u = 0).
- Removing the `dlen <= 0` early-out in Smudge: with zero movement the source is the cell's own
  centre, which resamples to the same value; the check is only a shortcut.
- Undo Brush `a >= 1.0` branch: the double lerp `old + (orig - old) * 1` is exact for floats, so
  the branch only skips arithmetic.
- The per-dab snapshot margin for non-smudge brushes (2 cells): the box already extends to the
  ceiling of the radius, so any cell with kernel > 0 has all its neighbours inside the box; the
  margin is defensive. The Smudge margin is needed and is killed by a test.
- Copying the undo patch when it exceeds the cap: the patch is evicted at once, so only a
  transient allocation differs; the cap on retained memory is what is observable and tested.
