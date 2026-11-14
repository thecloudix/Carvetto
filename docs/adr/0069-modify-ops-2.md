# ADR-0069: Modify operations 2 (slice, draft, split, boundary, cross section, distort)

Status: accepted (T-0293)

## Decisions
- One module, `core/relief/derive.hpp`: free functions over `HeightField`/`Component`, no mutation of
  inputs, status enum `DeriveStatus`, cooperative `cancel` on the per-row heavy ones (slice, draft, distort).
- **Contours** (`opaqueContours`): marching squares on the opaque mask with the cell centres as lattice
  and crossings at edge midpoints (so a contour lies half a cell outside the outermost opaque centres and
  corners are chamfered; a circle of radius R stays within one cell of R). Saddles keep the opaque cells
  joined. Outer contours are CCW, holes CW; collinear vertices removed; loops ordered by lowest start key.
- **Slice Model**: bands of equal thickness anchored at the model top (going down) or bottom (going up);
  count 0 = enough to cover everything. A slice component is `MergeHigh`, `baseHeight = low`, field =
  `min(h, high) - low` (so it is a thickness field in [0, t]); cells at or below `low` are transparent
  except in the floor band (bottom band from the bottom; last band from the top when count is 0). Merging
  all slices therefore rebuilds the model exactly (to float rounding); with a limited count cells below the
  lowest band (from top) are lost and cells above the highest band (from bottom) are clamped. Boundary
  vectors = contours of each slice's footprint.
- **Add Draft**: "minimum draft angle" is measured from vertical; the result is the upper Lipschitz
  envelope `h'(x) = max_y h(y) - s |x - y|`, `s = 1 / tan(angle)`, i.e. material is added at the foot of
  every wall, tops are never lowered, transparent cells stay transparent and block propagation. Computed
  by a two-pass chamfer with a 16-point stencil (axes, diagonals, knight moves; weights are the true step
  lengths), O(16 N). The gradient bound holds exactly between stencil neighbours; against the Euclidean
  distance the drop is never smaller than the exact cone and at most about 3% larger (tested on a spike).
  Angle 0 copies; angle in [0, 90 deg).
- **Split**: closed vector = even-odd inside (A) / outside (B) on cell centres. Open vector: both end
  segments are extended far beyond the field, the side is the sign of the cross product with the nearest
  segment (the pseudo-normal, sum of the two adjoining left normals, at an interior vertex); A = left of
  travel, line cells go to A. Names "<name> – A"/"– B" (en dash, UTF-8). Both keep grid and properties.
- **Vector boundary**: contours of the union of the footprints on the finest cell size; several
  components give one group.
- **Cross section**: (distance, height) polylines, one per opaque run, samples every `spacing` plus the
  end point using the bilinear sampler; optional Douglas-Peucker simplification.
- **Distort**: no 2D distort maths exists in `core` (searched geometry and relief), so this card adds the
  Bounding Box Distortion envelope: bilinear map of the cell-centre rectangle onto a convex quad (inverse
  solved analytically, convexity validated). The output stays on the original cell lattice (identity quad =
  same cells bit for bit); heights are unchanged; samples within 1e-9 of a cell centre snap to it.
  Along-a-curve and between-two-curves envelopes, rotated bounds and baking UI are not in this card
  (they need node editing and arrive with the app work); they are deviations noted on the card.

## Test argument (mutation)
Killed by exact goldens: contour vertex counts/areas/orientation (saddle rule, midpoint offsets, hole
direction); slice band edges, floor-band inclusion, h==low exclusion, clamp, base, names, count and
need rounding, reconstruction through `compose` for both anchors; draft exact wall values, per-neighbour
gradient bound on all 16 offsets (a dropped stencil entry or wrong weight breaks the bound or the spike
ratio), never-lower, transparency, angle limits; split side for straight, reversed, on-line, corner
(vertex pseudo-normal) cases, closed counts, property copy, recombination; cross-section collinearity,
slope, simplification, run splitting, trimming; distort identity bit-exactness, stretch, trapezoid points,
rotation, convexity and grid errors; every status and cancel.

Survivors argued: the `d2 < best` tie order between segments equidistant from a cell only matters on the
bisector of a corner where both segments give the same side by symmetry of the pseudo-normal rule; the
`1e-9` snap and `eps` in the quad inverse are equivalent for any representable cell coordinate away from
the quad boundary; the 3% bound is a measured property of the stencil, not a proof.
