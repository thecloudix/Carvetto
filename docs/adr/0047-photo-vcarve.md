# ADR-0047: Photo V-carve (depth mapping and conventions)

Date: 2026-11-10 · Status: accepted, core implemented (T-0250).

## Context
The inventory lists Photo V-Carve: source a selected bitmap; Max Carving Depth; Stepover Retract;
strategy Raster / Hatch / Selected Vectors with Line Spacing % and Line Angle. It does not say how
brightness maps to depth, so the mapping is decided here.

## Decision
1. **Depth.** depth = min(maxDepth * darkness, toolLimit), darkness = 1 - gray/255 (gray/255 with
   Invert). The hard clamp (not rescaling) keeps depth proportional to tone below the limit, as
   the card specifies. When maxDepth > toolLimit a `CarveDepthLimited` diagnostic is given.
2. **Tool limit.** The depth at which the cone is as wide as the tool: (D - tip) / (2 tan(a/2)), so
   the cut is never wider than the tool. V-bit (tip 0) and engraving bit (tip = flat tip diameter).
3. **Sampling.** Bilinear on pixel centres, image row 0 at the top of the placement rectangle; steps
   along a line equal the smaller pixel pitch (at least 1 micron). Positions outside the placement
   have depth 0 (Selected Vectors). Samples whose depth lies within `depthTolerance` of a straight
   ramp are merged (1D Douglas-Peucker on arc length / depth).
4. **Lines.** Line spacing = % of tool diameter, shrunk so a whole number of lines tiles the extent
   across the lines (pitch <= spacing), lines centred in their band. Zig-zag direction. Hatch =
   Raster at the angle, then a second pass at angle + 90 degrees.
5. **Stepover Retract.** Between consecutive cuts (lines, or vectors) the tool stays down and feeds
   to the next start when the depth change of the link is <= stepoverRetract and the XY link is at
   most two line spacings; otherwise it retracts to safe Z, rapids and plunges. 0 = always retract.
   A cut run is the samples deeper than 1 micron plus the zero-depth neighbour on each side.
6. **Single pass.** The cut goes to its depth in one pass (no pass-depth stepping), like engraving.
7. **Bounds.** Work is counted in samples (default limit 50 million; `ToolpathRunControl`), moves
   limited (default 20 million); both refused before generating where estimable; cancel flag
   polled per line and per vector; a refused or cancelled run returns nothing partial.
8. **Core interface.** Qt-free: `GrayImage` (+ `grayFromRaster` from T-0245's `RasterImage`) and a
   placement `Box2`. The document has no pixel store for `BitmapObject` yet, so the app wiring
   (decode via QImage, record, form, preview) is a follow-up.
