# ADR-0060: Relief core: height fields, components, levels, composition

Status: accepted (T-0281)

## Decisions
- `core/relief`: `HeightField` (row-major float, NaN = transparent, cell (i,j) centre at
  origin + (i,j)*cellSize), `Component`, `Level`, `compose`, `bake`. Cap `kMaxCells` = 2^25; zero,
  negative or non-finite sizes and oversized grids return a `Status`, never throw or crash.
- Sampling is bilinear and exact on cell centres; outside the centre rectangle, or touching a
  transparent cell with non-zero weight, it is transparent. Composition samples every component
  onto a caller-given `Grid` (`unionGrid` proposes one at the finest cell size).
- Combine (`below` = accumulated, `above` = new): above transparent -> below, all modes. Below
  transparent -> above for Add, Merge High, Merge Low; stays transparent for Subtract and Multiply
  (nothing to subtract from or scale). Otherwise below+above, below-above, max, min, below*above.
- Component value = raw * shapeHeight% , then fade, then tilt, then + baseHeight; transparent cells
  stay transparent. Fade factor = 1 - percent/100 * t, t = projection onto anchor->second clamped
  to [0,1]. Tilt adds tan(angle) * signed distance along pivot->raise (unit vector) from the
  pivot. Applied at compose time; the stored field is never modified. `bake` = composeComponent on
  the component's own grid, so it equals the composed value exactly.
- A level combines its components in order (first against all-transparent), then adds its base
  height to opaque cells, then combines into the model with its own mode. Level base height is not
  part of any bake. Hidden levels/components are skipped.
