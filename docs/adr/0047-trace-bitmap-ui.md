# ADR-0047: Trace Bitmap command, work budget and cancel

Date: 2026-11-10 · Status: accepted (T-0260; follows ADR-0038).

## Context
T-0245 left the tracer without a work cap or cancel hook: a valid 4000x4000 bitmap with one huge
boundary took 170-220 s. The app also needs the decode, dialog and undoable command.

## Decision
1. **Budget.** `TraceParams::maxBoundaryEdges` (default `kTraceDefaultMaxBoundaryEdges` = 4 M) bounds
   the pixel-edge segments of all masks after noise filtering. Edges are counted in the cheap
   edge-marking pass, so an over-budget request fails with "too complex" before any loop is followed or
   fitted. Inputs under the budget produce byte-identical output to before.
2. **Cancel.** `traceBitmap` polls `geometry::cancelRequested()` (per mask row, per loop, every 65536
   loop steps, per Bezier fit) and returns the error "trace cancelled"; callers discard it.
3. **Decode in the app only.** `decodeImageFile` (QImageReader) reads the size from the header and
   refuses (edge, pixel count, file size over 64 MB, non PNG/JPEG/BMP) before decoding; alpha is
   flattened onto white. Core keeps no codec.
4. **Command.** One undo step (macro): a new layer per traced colour (named "Trace" or "Trace #RRGGBB",
   coloured accordingly) each with ONE vector object holding all that colour's contours (even-odd region,
   outer CCW and holes CW). The size can be set as mm (or inch) per pixel, target width or target height.
5. **Preview** traces an image shrunk to at most 400 px (box average) with scaled pitch and noise
   filter, on the same worker (one trace at a time; a change cancels and restarts it).

## Not done (follow-ups)
Pick from 2D View and Bitmap Fading (display features) are not part of this slice.
