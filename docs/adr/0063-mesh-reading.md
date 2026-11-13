# ADR-0063: Triangle-mesh reading and orientation

Status: accepted (T-0285)

## Context
Phase 2 (3D relief) imports STL and OBJ models, orients them and rasterises them to a height field
(T-0286). The reader and the orientation maths are Qt-free and must be safe on hostile files.

## Decisions
- `core/io/mesh.hpp`: `TriangleMesh` (double vertices, indexed triangles, CCW as read), `meshBounds`,
  `readStl`, `readObj`, `readMesh`, `readMeshFile`. Readers take bytes, so tests and fuzzing are
  headless; only `readMeshFile` touches the file system (size checked before reading).
- STL: a file whose size is exactly 84 + 50 x count is binary (even if its header says "solid");
  otherwise a "solid" start means ASCII with a strict keyword grammar (truncation, missing
  `endsolid`, bad numbers are errors). Normals are ignored (winding comes from the vertices).
  Vertices are welded by exact float coordinates, in first-appearance order.
- OBJ: `v` and `f` only; polygons are fan-triangulated from the first vertex (correct for convex
  faces; concave faces may give overlapping triangles). Negative indices supported; a face may only
  reference vertices already defined. Materials, groups, normals and texture coordinates are ignored.
- Hostile input: caps on bytes (512 MiB), vertices (20 M), triangles (10 M) and |coordinate| (1e9),
  all in `MeshLimits`; counts are checked against the real size before allocating; NaN/inf refused.
- Orientation: the six top-face choices are exact proper rotations onto +Z (no mirroring, winding
  kept), followed by 0..3 quarter turns about Z, so the goldens are exact. Arbitrary rotation uses
  Rodrigues' formula. Lock-ratio scale drives one chosen axis; scaling is about the bounds minimum.
  Units: inches multiply by 25.4. Center Model centres the bounds box on the origin in X, Y and Z.
- The inventory says "5 rotations about Z"; only the four quarter turns (0, 90, 180, 270) are
  implemented as presets; any other angle goes through `rotateMesh`. Open question for the owner.
- Deferred: 3DS, DXF 3D mesh and SKP readers (proprietary or large formats, not needed for the first
  relief milestone), "Apply Perspective along Z". No extra card opened.
