# ADR-0130: General polygon nester and Nest Selected Vectors

Date: 2026-11-19 · Status: accepted (T-0337) · Builds on ADR-0110 (shelf nester for rectangles), ADR-0007

## Context
T-0329 shipped only `io::nestRectangles` (shelf packing). The feature inventory lists Nest Parts with rotation
by step angle, mirroring, parts inside parts, a nest boundary with exclusions, nesting sheets, start corner and
direction, spacing and copies. This card brings a real polygon nester and the Edit > Nest Selected Vectors form.

## Decision
- `geometry::nestParts` (`core/geometry/nesting`): input is rigid shapes (closed outline + holes, already
  polygonised by the caller with its chordal tolerance, added to the spacing) and options; output is, per copy, a
  sheet index and a `Transform2` (rotation, optional mirror, move). No Qt, no document, no randomness, no clock.
- Algorithm: greedy bottom-left. Copies sorted by outline area (descending, then part, copy). For each, every
  existing sheet in order, then a new one. On a sheet, candidate positions are the cross product of x/y
  coordinates taken from the margin, the bounding boxes of what is placed, the hole boxes (only with
  "parts inside parts") and the boundary vertices, scanned in start-direction order; for each orientation
  (rotation step x optional mirror, de-duplicated) the first collision-free candidate is slid toward the start
  corner by bisection (two rounds, both axes) and the orientation with the smallest far edge wins. The start
  corner is handled by running the packer on parts reflected inside their own bounding box, so parts are never
  mirrored by the corner choice.
- Collision: bounding boxes, then edge-to-edge distance (segment distance below the spacing, or a proper
  crossing), then containment of one outline vertex in the other's solid (outline minus holes; holes are
  ignored unless parts inside parts is on, so a hole stays empty otherwise). A boundary polygon set is an even-odd
  region; its edges are obstacles at the border gap, every outline vertex must lie in the region and no
  exclusion may be swallowed.
- Spacing is clamped to at least 0.001 mm plus the chordal tolerance, so touching parts are never accepted;
  candidate positions sit exactly one spacing from their neighbours, so the distance test has a 1e-7 mm slack.
- Bounded: an operation counter (edge-pair, candidate and containment probes) with default 4e7; when it runs
  out the remaining copies are reported unplaced (`Budget`). `cancelRequested()` is polled per candidate.
  `maxSheets` bounds the sheet count; a shape that does not fit an empty sheet is cached as too large.
- `viewmodel::nestSelection` groups selected top-level vector objects into parts: closed outlines that overlap or
  nest form one part (union-find); a part whose outlines cross, or with islands inside holes, nests by the
  convex hull of its outlines (conservative, no holes). Open vectors travel with the part that contains their
  first point; others are left alone and reported. Copies go to new sheets (sheet-local coordinates, datum
  bottom left), the active sheet (its material rectangle) or a boundary layer; originals are removed by default.
  One undo step, all-or-nothing.

## Not done / readings for owner review
- Grain direction and offcut handling (card title) are not modelled beyond "rotation step 0 keeps the grain":
  no per-part grain flag. Nest two-sided parts, Filler Part and per-part copy counts (one global count) are left out.
- The rectangle shelf nester stays as it is (`io::nestRectangles`): its results are pinned by the parts-import
  tests; the general nester gives the same or fewer sheets on the rectangle fixture (tested).
- Placement on the active sheet or a boundary does not treat other, unselected objects as obstacles.
- Candidate-position packing is not optimal; two rounds of sliding make it close for typical parts.
