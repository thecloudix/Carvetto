# ADR-0038: Bitmap tracing and raster input

Date: 2026-11-09 · Status: accepted, core implemented (T-0245).

## Context
T-0245 traces a bitmap to Bezier/line contours (B&W threshold, colour mode of up to 16 colours,
corner fit, noise filter) and asked for an image-decoder dependency decision.

## Decision
1. **No decoder dependency in core.** The tracer consumes `io::RasterImage` (8-bit RGB, top row first).
   Core decodes only Netpbm (PBM/PGM/PPM, ASCII and binary; about 150 lines, size-checked before any
   allocation) so tests and tools need no codec. PNG, JPEG, BMP, TIFF, GIF and WebP are decoded by the
   app layer into `RasterImage` (Qt `QImage` is already a dependency there, no new licence). A later
   in-repo or vendored PNG decoder is possible without touching the tracer; that choice is deferred.
2. **Bounds.** Edge at most 16384 px, at most 16 Mpx (checked on `RasterImage::valid()` and in the
   decoder before allocating); output at most 50000 contours and 1,000,000 spans, exceeding either is
   an error, never a partial result.
3. **Algorithm (clean-room, textbook techniques).** Mask (luma threshold, or median-cut quantisation on
   a 5-bit histogram, one mask per colour) -> noise filter (small foreground components and background
   holes removed) -> exact pixel-edge boundary loops (foreground 4-connected, so a diagonal pair of
   pixels stays two shapes; outer CCW, holes CW) -> per-loop corner detection (two straight runs of at
   least `cornerRun` px meet, 8 loose .. 3 tight) -> box smoothing of the staircase between corners
   (window 4 loose .. 1 tight) -> least-squares cubic Bezier fit (chord-length parameters, Newton
   reparametrisation, split at the worst point) within 0.5 px; straight stretches become line spans.
4. **Known limits.** A 45 degree edge ending in a square corner rounds that corner by about the smoothing
   window; areas stay within 2% of the pixel count for the test shapes. "Bitmap Fading" and "Pick from
   2D View" are display/UI features, and the undoable document command and dialog are app work.
