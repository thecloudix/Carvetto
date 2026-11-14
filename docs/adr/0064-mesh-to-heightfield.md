# ADR-0064: Mesh to height field

Status: accepted (T-0286)

## Context
Imported STL/OBJ models (T-0285) must become a top-down `HeightField` (ADR-0060) for the relief
pipeline, with a zero plane, undercut information and optional bottom side for two-sided jobs.

## Decisions
- API: `relief/mesh_raster.hpp`, `meshToHeightField(mesh, options)` returning
  `ShapeResult<MeshRaster>` (errors are strings, including "cancelled"), so `Status` of ADR-0060 is
  untouched.
- Sampling: one vertical ray per cell centre, in double precision. The footprint test uses edge
  functions evaluated on canonically ordered endpoints (shared edges give exactly opposite values)
  plus the top-left rule for centres exactly on an edge or vertex, so every centre is hit exactly
  once per surface crossing: no cracks, no double hits, no epsilon. Vertical/zero-area triangles
  have no footprint and are skipped. Height is the plane interpolation of the triangle.
- Highest surface wins, whatever the winding: reversed or open meshes still rasterise (this is the
  "backface caps" behaviour; faces are used for the top surface regardless of facing). Winding is
  used only for undercut detection (up-facing hit = entering material).
- Heights are `zTop - zeroPlane`. With `discardBelowZero` (default) heights below the plane are
  clamped to 0 where the mesh covers the cell (data removed, stock stays at the plane); cells with
  no mesh coverage are transparent. Without it negatives are kept. `maxHeight` (>= 0) reports the
  thickness above the plane so the caller can resize material to fit.
- Material fit: with no explicit grid, the grid covers the XY bounds, `ceil(size / cell)` cells
  with centres at `min + (i + 0.5) * cell`; an explicit `Grid` overrides it.
- Two sides: the bottom field is `zeroPlane - lowest surface` clamped to 0 (the part below the
  plane viewed from beneath, as a height field), plus `maxDepth`.
- Undercut mask: after the top surface, the first exit (down-facing hit) bounds the top slab at z2.
  With discard (the plane is the stock floor): undercut when `z2 > zero + 1e-9*scale`, i.e. the top
  slab does not reach the plane, so a cavity is lost. Without discard: undercut when an up-facing
  hit (more material) exists below z2. Open meshes (no exit hit) are never flagged.
- Work bound and cancel: triangles are swept by row (sorted by first row, an active list), work is
  O(sum of triangle row spans x cell spans + hits), memory O(triangles + one row of hits); cell
  count is capped by `kMaxCells`. `cancel` is polled once per row. Per-cell hits are ordered with a
  deterministic sort, so output is identical across runs.
- Mutation notes: `z2 > zero + tol` vs `z2 >= zero`/`> zero` is an equivalent mutant on exact data
  (the tolerance only absorbs floating-point noise from interpolated hits); all other mutants tried
  (tie rule, edge sign, winding, sort order, clamp thresholds, cancel polling, undercut branches)
  are killed by the tests.
- Not done: spatial triangle bins in 2D (the row sweep already bounds work by footprint), anti-
  aliased or multi-sample rasterising (one sample per cell by design).
