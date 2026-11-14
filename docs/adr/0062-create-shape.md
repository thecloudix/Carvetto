# ADR-0062: Create Shape (relief components from closed vectors)

Status: accepted (T-0283)

## Decisions
- `core/relief/shape.hpp`: `Shape` = `ShapeParams` + closed `Polyline` vectors + cell size or explicit
  `Grid`. `buildShape` returns a `Component` (field + scale %, base, fade, tilt, mode, name from the
  params); `Component` itself is unchanged. Re-editing = change the params and call `buildShape` again.
- Fill is even-odd over all polygons: holes are transparent cells, islands inside holes are shapes
  again. Orientation is irrelevant. Nesting depth = number of other polygons containing the first
  vertex. Cells are inside when the cell centre is inside; default grid = vector bounds plus a one-cell
  margin.
- Height depends on d, the exact Euclidean distance from the cell centre to the nearest vector edge
  (holes count as edges). Angled h = d tan(angle); Round: circular arc through the edge with slope
  tan(angle) reaching its top at dMax (the deepest d), i.e. a spherical cap for a circle (angle = 90
  degrees: hemisphere); Smooth: h = dMax (d/dMax)^(4^-s), s in [-1,1] (-1 concave, 0 linear, +1
  convex); Custom: piecewise-linear factor over u = d/dMax, raw height = factor * dMax. Plane: 0.
  Limit caps the raw profile (before scale). Scale = `shapeHeightPercent`.
- Product choices not fixed by the inventory (revisit if the owner disagrees): Round's arc radius is
  tied to dMax (no separate radius field); Smooth/Custom raw peak is dMax-scaled; Blend To Inner
  Vectors (Smooth/Custom only, ignored otherwise) drops hole polygons (odd depth) from the edge set,
  so the profile is computed from the outer boundaries while hole cells stay transparent; Preserve
  Internal Corners replaces the distance near a reflex vertex (nearest feature = that vertex) with the
  larger of the two adjoining edge-line distances, giving a sharp crease instead of a rounded valley
  (convex vertices and straight edges are unchanged).
- Total height = base + rawMax * scale / 100 (rawMax = stored field maximum). `setTotalHeight` solves
  either for the scale (base kept; needs rawMax > 0 and total > base) or for the base (scale kept).
- Distance method: every cell within two cells of a sampled point (every half cell) of an edge is
  seeded with the exact distance to that edge and its id; edge ids then propagate between 8-neighbours
  in alternating raster sweeps, re-testing the exact distance to the neighbour's edge, until nothing
  improves (at most 16 rounds). This is the card's distance transform with exact edge distances, so
  analytic goldens hold to float precision; the result is always the distance to a real edge (never
  lower than the true distance), and differs from it only in exotic sub-cell cases. If edges leave the
  grid, the border cells are seeded exactly by brute force (bounded by 2^28 edge tests, else
  TooMuchInput). Cells are never tested against edges of hole polygons when blend is on.
- Default grid: bounds plus margin with cell centres half a cell off the vector bounds, so symmetric
  shapes are sampled symmetrically.
- Bounded work: grid <= kMaxCells, <= 20000 polygons, <= 2^21 segments, <= 2^24 row crossings; the
  transform is O(cells); `cancel` is polled once per row and a cancelled build returns no result.
  Deterministic: no threads, no unordered containers, crossings sorted by (x, dir).

## Test argument (mutation)
Analytic goldens on every cell kill: wrong slope/limit, off-by-half-cell inside test, wrong profile
exponent/direction, Round arc offsets, hole handling (parity, islands), blend (changes a hole-adjacent
cell; Angled-ignores-flag check), mitre (reflex vertex value, never-higher, identical away from the
corner, identical for convex shapes), total-height maths, every error status, cancel and determinism.

Survivors argued: Round `min(d, dMax)` is equivalent (d never exceeds dMax, the maximum of d); flipping
only one of the two reflex flags of a vertex survives because either adjoining segment alone yields the
mitre (flipping both, and the orientation/parity rule for hole corners, are killed).
