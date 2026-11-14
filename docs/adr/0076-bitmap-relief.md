# ADR-0076: Bitmap to relief component, zero plane, Scale Model Z

Status: accepted (T-0284)

## Decisions
- `relief/bitmap_relief.hpp`. 16-bit depth is kept by a `GrayImage16` path: Netpbm P2/P3/P5/P6
  (maxval up to 65535) are read at full depth in relief itself (colour -> Rec.601 luminance,
  integer); P1/P4 and every format the app decodes (`BitmapDecoder`) arrive as 8-bit RGB and map
  to v*257. `io::RasterImage` stays 8-bit (ADR-0038 is unchanged). Sources: the `.carv` bitmap
  store (`loadBitmapObjectGray16`) or a file (`loadFileGray16`, capped at the embedding cap).
- Height = zHeight * (p - lo) / (hi - lo); (lo, hi) = (0, 65535), or the image min/max with
  auto-scale. Invert replaces p by 65535 - p before that (so auto-scale of an inverted image maps
  its darkest pixel to the top). A uniform image under auto-scale gives a flat zero field. One cell
  per pixel, square pixels (cellSize = width / px), pixel row 0 is the top (highest Y) row; the
  params give the lower-left corner of the image rectangle, so the first cell centre is half a cell
  in. The component has mode Add (selectable), 100 % height, base 0.
- Zero plane: a height-0 field with ceil(size/cell)+1 cells per axis, first centre on the
  corner, so its centre rectangle covers the job rectangle (it may overshoot by under one cell).
- Scale Model Z scales about Z = 0 (the zero plane): top side = values >= 0, bottom side = values
  < 0, only scaled with "both sides". Percent: factor = value/100. Exact height: top-only makes
  max = value; both sides make max(hi,0) - min(lo,0) = value. A model with no extent to scale
  (flat zero, no top side for top-only, all transparent, empty) is an error, never a divide by
  zero. Results are floats, so the target is hit to float resolution (error <= half an ulp,
  below 1e-6 for targets under 32 mm). The field is replaced only on success.
- Errors reuse relief `Status` (BadParameter, TooLarge) and `ShapeResult` strings for decoding.

## Mutants
Boundary cases are pinned by tests: row flip, corner offset, invert vs. auto-scale combinations,
`>=` vs `>` at zero for the top/bottom split, `hi > lo` guard (uniform image), `ceil`+1 cell
count with exact and inexact multiples, every maxval/truncation/overflow header check, and the
bothSides extent with all-negative data. The `v > 100000000` saturation in the Netpbm number
reader only avoids signed overflow; any value above the 65535 maxval or 16384 edge limit is
rejected either way, so changing the saturation constant is unobservable (equivalent mutant).
