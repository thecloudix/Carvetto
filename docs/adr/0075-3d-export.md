# ADR-0075: 3D export (STL, OBJ, grayscale heightmap)

Status: accepted (T-0287)

## Context
The composed relief (T-0281) is exported as a mesh or an image. Reading is T-0285 (ADR-0063).

## Decisions
- `core/io/export3d.hpp`. Writers return bytes (no file access), like the readers.
- Surface mesh: one vertex per opaque cell centre used by a square of four opaque cells, two
  triangles per square with a fixed diagonal, CCW from +Z. Transparent cells make holes.
- Back face: `Open` (top only) or `Flat` (back plane at an explicit `backZ`, strictly below the
  lowest cell, plus outward walls on every boundary edge including hole boundaries). Result is a
  closed manifold: every directed edge occurs once and its reverse once; tests check this by
  edge counting, Euler's formula and positive signed volume equal to the analytic column integral.
  Pinch points, where two live squares touch only at a corner, get one vertex per square (top and
  back), otherwise four walls would share one vertical edge and break the manifold.
- Not implemented from the inventory: "Close with inverted front", "Close with other side" (two-sided
  jobs), "Close sides" (rotary), tolerance/triangle-count decimation and Triangulate reporting, POV-Ray.
  A triangle cap (`maxTriangles`) is an error rather than silent decimation. Open question for the owner.
- STL: binary only, float32 coordinates (exact round trip through `readStl` at float precision),
  unit normals from the triangle, header never starts with "solid". OBJ: shortest round-trip decimal
  (`std::to_chars`, locale independent), exact round trip.
- Heightmap: 16-bit, linear, default range = lowest/highest opaque cell so those are exactly
  0/65535; explicit range clamps; transparent = black; flat model = all black; row 0 = largest y.
  PGM is binary P5 maxval 65535; PNG is grayscale 16-bit written with stored deflate blocks, our own
  CRC-32/Adler-32 (no codec dependency; files are large but valid). Edge/pixel caps from `raster.hpp`.
- `.3dclip` clipart is a proprietary-style own format and is out of scope (deferred).

## Mutation review
Boundary mutants are pinned by tests: `backZ == lowest` (error) vs just below; triangle cap at
exactly the count (ok) and one below (error), open and closed; wall edge detection on all four
sides (closed-manifold check on a holed grid with edge cells); wall winding (positive volume);
back-face winding (manifold needs opposite directions); heightmap rounding (half-up case 32767.5),
clamp on both ends, row flip, transparent-as-black, flat range, multi-block deflate (200x200),
CRC/Adler/length complement verified by an independent decoder; STL normal/layout bytes checked.
Pinch slot logic: dropping the split fails the holed-grid manifold check and the vertex count (48).
Equivalent mutants: the `do/while` first block for an empty image cannot occur (size >= 1 row).
