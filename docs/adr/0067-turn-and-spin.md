# ADR-0067: Turn and Spin (revolved relief components)

Status: accepted (T-0290)

## Decisions
- `core/relief/revolve.hpp`: `Revolve` = `RevolveParams` + a profile (`vector<Point>`); `buildRevolve`
  returns a `Component` (field, mode, name, scale %, base). `Component` is unchanged; re-editing =
  change the params/profile and rebuild. Deterministic, no threads, cancel polled once per row.
- Spin: the first point is the left end and the axis is the vertical line through `center` (default the
  origin). Profile x - x0 is the radius, y the height. Height at a cell = profile value at its distance
  from the centre; cells beyond the last point are transparent. A half circle gives a hemisphere.
- Turn: axis = line from the first to the last point; profile points stay at their model positions.
  Position along the axis t and radius r = |distance to the axis| (either side counts the same). Surface
  height at lateral offset w is sqrt(r(t)^2 - w^2), transparent for |w| > r or t outside the part.
  A rectangle (0,0),(0,r),(L,r),(L,0) gives a cylinder.
- Product choices not fixed by the inventory (revisit if the owner disagrees): the position along the
  axis (Turn) or x (Spin) must not decrease; a profile that doubles back is `BadProfile` rather than
  taking an envelope. Equal positions are steps and the larger value wins (so the flat end of a cylinder
  is a full-height edge). Only the upper half of a Turn body is a relief (top surface). Turn footprint =
  the oriented rectangle's bounding box; Spin grid is centred on a cell so the axis is sampled exactly.
- Errors: fewer than 2 or non-finite points or > 100000 points, or backwards motion beyond 1e-9 of the
  length -> `BadProfile`; Turn start == end, no radius (all points on the axis), Spin zero width ->
  `Degenerate`; bad cell size / oversized grid -> `BadGrid`; non-finite base/centre/exact height ->
  `BadParameter`. Float noise below the tolerance is clamped to a step.
- Scale to Exact Height: `exactHeight` solves the component scale % (base kept) so
  base + rawMax * scale / 100 = exactHeight, with rawMax the analytic maximum (largest radius for Turn,
  largest profile y for Spin), not the sampled maximum, so it is independent of the cell size. Not
  reachable (rawMax <= 0 or height <= base) -> `BadParameter`.
- Lookup is O(log n) per cell (binary search on the monotone positions), so work is cells * log(points).

## Mutation notes
Tests pin: the hemisphere against the analytic sphere at every cell; step handling (max at a shared
position, and flat cylinder ends); transparency outside the radius and before/after the part; rotated
axis with a taper; centre offset and left-end offset; exact height with a non-zero base and the
unreachable case; every error status; backwards profiles for both kinds; determinism; cancel. Dropping
the monotone clamp is equivalent only below the tolerance (float noise), which no result can show.
