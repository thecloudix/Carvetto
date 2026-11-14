# ADR-0068: Modify ops 1 (bake, replace below, reset, offset, smooth, emboss)

Status: accepted (T-0292)

## Decisions
- `core/relief/modify.hpp`: pure functions HeightField -> HeightField (or Component for bake); inputs are
  never modified, so "Ctrl keeps originals" is the caller not deleting the sources. Component-level use =
  `bakeComponents` / `bake` first, then the op, then `componentFromField` (neutral Add component), which is
  also what "bake smoothing" means. Errors are `relief::Status` (BadParameter, ZeroSize, TooLarge).
- Bake: the selection is combined as one level (order, modes, dynamic properties, hidden skipped) on the
  union grid at the finest cell size; result is Add, 100 %, base 0, no fade/tilt.
- Replace Below: opaque cells strictly below `threshold` become transparent or `planeHeight`; then
  `lowerBy` is subtracted from every opaque cell (planes included). Transparent cells are never filled.
  The threshold is absolute in the caller's frame (selection-relative is the dialog's arithmetic).
- Reset Inside/Outside: cell centre vs even-odd fill of closed vectors (half-open crossing rule, bottom/left
  edges inside); cleared cells become transparent. Outside is the exact complement. At most 65536 segments.
- Offset Model: offset surface of the top surface by distance d along the normal, as grey-scale morphology
  with a ball of radius |d|: d>0 `max f(q)+sqrt(d^2-|p-q|^2)`, d<0 `min f(q)-sqrt(...)`. Transparent cells stay
  transparent and are ignored as neighbours (the model does not grow into transparency). Clip to zero plane:
  heights below 0 become 0 afterwards. Radius <= 512 cells and cells*window <= 4e9, else BadParameter/TooLarge.
  Round trip d then -d is a morphological closing: exact where concave curvature radii exceed d and away
  from the grid border; tested on a convex cap (tolerance 0.05 mm, cell 0.25 mm). It cannot restore detail
  narrower than the ball, by nature.
- Smooth: normalised Gaussian convolution over opaque cells (separable, double accumulation in fixed order,
  truncated at 3 sigma, so deterministic). sigma = percent/100 * radiusMm (default 2 mm at 100 %); 0 % is
  the identity. Preserve transparency: transparent cells stay transparent and opaque cells renormalise over
  opaque neighbours (edges do not sag). Not preserved: a transparent cell becomes opaque when at least half
  of its kernel weight is opaque. The percent scale and 2 mm default are product choices to revisit.
- Emboss: `out = in - (1-scale) * smooth(in)` (transparency preserved): scale 1 identity, scale 0 keeps only
  the detail above the broad shape, scale > 1 exaggerates. Detail smoothness = the smoothing percent.

## Mutants considered
Strict vs non-strict threshold, lowerBy on planes, half-open polygon edge, even-odd hole, ball radius rim
(distance exactly r included), lift sign, clip at 0, transparency renormalisation, 0.5 fill threshold, and
kernel truncation are each pinned by a hand-computed test. Equivalent mutants: the `den` guard (an opaque
cell always has weight >= 1, so den >= 1); the `inBall` test for cells outside the ball only matters in
corners of the window and is pinned by the spike case at (7,7) and (9,5).
