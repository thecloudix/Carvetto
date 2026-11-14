# ADR-0078: Textured area component

Status: accepted (T-0291)

## Decisions
- `relief/textured_area.hpp`. A textured area is a pure function of `TexturedAreaParams` (seed
  height field, boundary rings, spacing, shift, reflection, anchor, overlap mode); the parameters
  are the stored, re-editable form ("Edit Textured Area Component"). `makeTexturedComponent` wraps
  the evaluated field as a normal `Component`. The seed is a `HeightField` (bake a seed component
  first); the 2D vector texture (T-0251) is unrelated machinery and is not used.
- Boundary: closed rings, even-odd fill (holes allowed). Output grid = boundary bounding box at the
  seed's cell size, cells = ceil(extent / cell), centres at bboxMin + (i + 0.5) * cell. A cell is
  opaque iff its centre is inside (scanline, half-open: edges span (ymin, ymax], a centre exactly on
  a crossing counts as inside to the right of it, so the left/bottom edge is inside and the
  right/top edge outside) AND a tile with an opaque seed cell covers it. Clipping is therefore exact
  at cell level, never partial.
- Tiles: tile size = seed width x height (cells * cell size). Pitch = size * (1 + spacing / 100),
  spacing > -100 % (negative overlaps; pitch <= 0 is BadParameter). Shift X moves odd rows in X by
  shiftX % of the column pitch; shift Y moves odd columns in Y by shiftY % of the row pitch.
  Reflection: Horizontal mirrors odd columns left-right, Vertical mirrors odd rows top-bottom, Both
  does both (a mirror-repeat; the inventory only names the four buttons, this is the reading).
  Tile (0,0) has its lower-left at the anchor (default: bounding-box minimum).
- Sampling is nearest seed cell (no interpolation), so aligned tilings reproduce the seed exactly.
  A tile owns the half-open rectangle [left, left+w) x [bottom, bottom+h). Where tiles overlap
  the later tile (bottom row first, left to right) is the "above" operand of `combine` with
  `overlapMode` (default MergeHigh); a transparent seed cell never contributes.
- Cap: the conservative (row x column) index range must be <= kMaxTexturedTiles (100000), else
  `TooManyTiles` with a message that says what to change; total (tile, cell) visits are capped at
  2^28 (same status). Output above kMaxCells is `TooLarge`. No partial results on error.

## Mutants
Run by hand (about 50 single-token (plus shift-margin mutants killed by shifts of 300 %+) mutations of textured_area.cpp against the textured tests),
killed by the golden tests, the tie tests (tile edges and vertices on cell centres), the inclusive
tile-cap test (exactly 100000) and a brute-force per-cell reference over nine parameter sets
(non-square seed, negative/positive spacing and shift, all reflections, anchors, every overlap
mode). Remaining survivors, argued:
- The `left >= bbox.x1` style tile-skip comparisons are an optimisation: a skipped tile has no
  cell with 0 <= u < w, so skipping or not gives the same output (equivalent).
- The `-1` / `+1` margins on a tile's cell range only guard float rounding of floor(); the exact
  `u`/`v` half-open test decides membership (equivalent in exact arithmetic).
- `i1 < i0 || j1 < j0` and the `min(sw - 1, ...)` / `min(sh - 1, ...)` clamps are defensive
  guards against rounding; unreachable for non-skipped tiles / u < w.
- `std::isnan(val)` skip: `combine` already returns `below` for a transparent `above`.
- `nx * ny <= kMaxCells`: `validate(grid)` re-checks the same limit (the pre-check only avoids
  an out-of-range double to size_t cast).
- `visits > 2^28` vs `>=`: a one-visit off-by-one on a resource cap, not observable in a unit
  test (2^28 visits).
