# ADR-0105: Laser Sketch Engraving

Date: 2026-11-16 · Status: proposed (T-0324)

## Context
Laser Sketch Engraving (feature-inventory "Laser Sketch Engraving") draws a bitmap or a 3D model as a hatch drawing:
Bitmap or 3D Model source, Power, Overscan, Machine Limit Boundary, Line Thickness, Stepover, Hatch Angle, Fill Together /
Fill Regionally / Fill regionally between vectors, Cross Hatch. The reference gives names, not behaviour; this ADR records
the readings. It uses ADR-0099 (beam power per move, laser marking, jet blocks) and the image machinery of ADR-0104.

## Decision
1. **What a sketch is.** Hatch lines at the stepover, across the source; along each line the beam burns a line up to
   `lineThickness` wide, **narrower where the tone is lighter and not at all where it is white**. The beam power is constant
   (`power`): tone is carried by line width, not power (that is Laser Picture's job).
2. **Line thickness by tracks.** The beam is the tool's diameter (kerf). A hatch line is drawn with
   `n = clamp(round(lineThickness / kerf), 1, 64)` parallel tracks a kerf apart, centred on the hatch line (offsets
   `(j - (n-1)/2) x kerf`). At darkness g (0..255) the number of lit tracks is `round(g / 255 x n)`, computed in integers
   `(2gn + 255) / 510`; tracks light **centre first**, the lower side first on a tie, so the drawn line stays centred and
   grows by tracks. With one track (thickness about the kerf) it is on at g >= 128. Neighbouring hatch lines
   overlap or leave gaps if the thickness and stepover do not suit each other; the form does not forbid it.
3. **Tone.** Bitmap: the darkness `255 - gray` (`gray` when inverted), read from the nearest pixel with the exact pixel
   runs of ADR-0104 (one feed per run of one lit state). Model: the sheet's composed relief model (the relief engine, as
   3D Rough/Finish use it) is rendered to a tone image by `toneFromHeightField`:
   - Depth: the lowest point black, the highest white (`255 (z - lo) / (hi - lo)`; flat = white);
   - Shaded: Lambert shading with the light from the upper left (azimuth 135 degrees, 45 degrees up), normal from
     central differences (one-sided at an edge or next to a transparent cell), normalised so a flat surface is white:
     `gray = 255 x clamp(n.L / sin 45, 0, 1)`; a slope facing away from the light is dark.
   Transparent cells are white. The image is the model's cell grid (rows from the top = the highest Y; placement = the
   cell edges), shrunk by an integer factor (averaging the opaque cells) when it exceeds the image limits.
4. **Boundary.** `Source`: the whole source (the bitmap's placement or the model's extent) is the one region. `Vectors`:
   the selected closed vectors bound the sketch (open ones are ignored with a diagnostic; the selection holds them besides
   the bitmap). The region is inset by the beam radius so the burn stays inside the vector, as Laser Cut / Fill's hatch.
5. **Fill order.** With Vectors: **Together** = the nonzero union of the vectors is one region and every hatch line
   crosses all of it before the next (a vector inside another adds nothing); **Regionally** = the same union, but each
   connected piece (an outer boundary with its holes) is sketched completely before the next, ordered by the cut order
   options; **Regionally between vectors** = the even-odd region (a vector inside another is a hole: the sketch is the area
   *between* the vectors), each piece in turn. With Source there is one region. These are product readings.
6. **Hatch lines.** On the multiples of the stepover across their direction (so a cross hatch at +90 degrees and
   neighbouring runs line up), the upper edge exclusive, angles that are multiples of a right angle exact; run as a
   serpentine, direction alternating after every track line that burns; **Cross Hatch** adds the same at +90 degrees after the
   first set, per region. Beam-off gaps of 5 mm or more split a track line into strokes (ADR-0104).
7. **Overscan.** As ADR-0104 on every stroke (rapid before, beam-off feed in, out, beam-off feed past the end). With the
   machine limit boundary the overscan runs are cut down to stay inside the limits too (T-0326, ADR-0113 §4; the T-0324
   verifier found they could leave them).
8. **Machine Limit Boundary.** When set, nothing is burned outside the machine limits the caller gives
   (`machineLimits`, a rectangle in document mm); a source wholly outside gives nothing (`NoRegion`). The viewmodel
   reading of the limits (`machineLimitsOf`): the **active sheet's material rectangle**, cut down to the tool's machine
   work area (`workAreaX/Y`, when both are stated) measured from the rectangle's lower-left corner. The document has no
   machine-bed datum yet, so this is the best-known frame; owner to confirm (alternative: the work area from the
   document origin). Without limits, or with the flag off, there is no clipping. The limits' fingerprint (the sheet
   rectangle) joins the toolpath's staleness; a machine's work area does not.
9. **Limits.** Hatch lines at most 2 million per angle, crossings 50 million (as ADR-0099); the pixel-edge work (sum
   over tracks of length x (1/pixel width + 1/pixel height) + the track lines) against `workLimit` (default 50
   million), counted before the lines are run; the moves against `moveLimit`; cancellation per track line. Nothing
   partial is returned when a limit trips.
10. **`.carv`.** Operation `laser_sketch`, job.json dialect only: `source`, `model_tone`, `boundary`, `invert`, `power`,
    `move_speed`, `overscan`, `machine_limit_boundary`, `line_thickness`, `stepover`, `hatch_angle_rad`, `cross_hatch`,
    `fill`, `safe_z`, `order`, `start_position`, `tolerance`; enums by name. Format 1.16 with ADR-0104 (written only
    when a picture or sketch record exists). The record stores the selection (the bitmap, and the vectors for the Vectors
    boundary); a model sketch reads the active sheet's relief model, like 3D Rough.
11. **Viewmodel.** `ToolpathOperation::LaserSketch`: form table (power, moveSpeed, overscan, lineThickness, stepover,
    hatchAngle, safeZ; with the Vectors boundary startX/startY, chordal, fill and the cut order; choices source,
    modelTone (model source), boundary, invert, machineLimitBoundary, crossHatch), end mill restriction, session
    calculation, staleness (bitmap, or the relief model's fingerprint, plus the vectors for the Vectors boundary and the
    sheet rectangle for the machine limit boundary). `usesBitmap` / `readsModel` replace the operation-only tests in
    the calculation and session code. The app entries are T-0326.

## Alternatives
- Line width by modulating power: that is Laser Picture; a sketch is hatching, and constant power keeps the burn mark
  of one track predictable.
- Variable-width lines as one swept stroke: the beam has one spot size; whole parallel tracks keep every move a plain
  feed with one power.
- A separate hatch module shared with Laser Cut / Fill: the card forbade refactoring T-0323; the scanline code is
  duplicated in `laser_sketch.cpp` (the sketch needs the line segments, Cut / Fill needs strokes), and can be merged later.
- Machine limits from the machine's work area alone: needs a datum the document does not have.

## Consequences
- The record has no fixed placement: a model sketch follows the relief model; a bitmap sketch follows the bitmap object.
- Product readings to confirm with the owner: tracks and lit-by-tone, the fill readings, the tone of a model, the
  machine limits frame, the inset by the beam radius.

## Mutation notes
Hand mutants of the new logic were applied one at a time and the sketch, record and viewmodel tests re-run; each was killed:
- lit-track rounding offset 255 read as 300; the beam-radius inset read as a full kerf; the even-odd / nonzero rules
  swapped between "between vectors" and the others; the serpentine flip removed; the machine limit's lower Y clamp read as
  a `min`; the track offsets without the half-track centring; the depth tone reversed; the shaded tone without the
  `/ sin 45` normalisation (a flat model must be white); hatch segments shorter than a millimetre dropped (killed by a 0.6 mm
  wide vector; survived the first tests); the tie-break `lit > rank` read as `>=` (one more track lit); the bitmap tone
  polarity swapped for invert; the cross hatch angle read as -90 degrees (killed by the direction of the first cross line,
  added after it survived); in the viewmodel: the work area clamp read as `max`, the Y clamp measured from the X corner (killed
  by a centred datum, whose corner has different X and Y; survived the first test), the Vectors boundary test inverted, the
  model source test inverted, and the machine limits not attached.
Argued equivalent, not killable: the `1e-12` margin in the track rank comparison (the offsets `(j - (n-1)/2) x kerf` are
exactly symmetric in IEEE arithmetic, so equal magnitudes compare equal with or without it; it only protects a future change
of the offsets); the snapping tolerance in `direction()` is killed by the cross hatch count (without it, the +90 degree set
gains a line at x = 0 from the 6e-17 sine).
