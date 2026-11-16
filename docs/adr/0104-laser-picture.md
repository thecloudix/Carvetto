# ADR-0104: Laser Picture toolpath

Date: 2026-11-16 · Status: proposed (T-0324)

## Context
T-0324 adds the Laser Module's picture engraving (feature-inventory "Laser Picture"): a bitmap is burned with the
beam at a power that follows the image, along Raster / Hatch / Selected-vector lines, with line spacing, line angle,
overscan, passes, power, move speed and a "Dither Image" option. T-0323 (ADR-0099) already gave moves a beam power
(`Move::jetPower`), marked laser toolpaths (`ToolSnapshot::laser`) and made the post's jet blocks act, so no new move
format is needed. The card leaves open what "power modulation" means, how dithering is done, how the lines meet the
pixels and where the picture's record and form fit. This ADR records the readings; Laser Sketch Engraving, which shares
the image machinery, is ADR-0105.

## Decision
1. **Reuse of Image Engrave.** The picture takes the same input as Image Engrave (T-0250, ADR-0047): the 8-bit
   `GrayImage` made by `grayFromRaster`, the placement rectangle of the bitmap object, the `PhotoStrategy` enum
   (Raster, Hatch, SelectedVectors), line placement (parallel lines at the line angle, the pitch shrunk so they tile the
   placement evenly, serpentine) and the viewmodel path (`PhotoInput`, `attachPhotoInput`, the bitmap selection stored
   as it is). Only the output differs: feed moves at Z 0 with a beam power instead of a depth.
2. **Tool.** As ADR-0099: an end mill whose diameter is the kerf; speed from `moveSpeed` or the cutting data's feed rate;
   no spindle (`laser`, spindle speed 0).
3. **Darkness and power.** The working image is the darkness `255 - gray` (`gray` when inverted). The beam power of
   a pixel of darkness g in 0..255 is
   - with power modulation (default): 0 for g = 0, else `minPower + (power - minPower) x g / 255` (percent);
   - without: `power` for g >= 128 (at least half dark), else 0.
   A pixel with darkness 0 is never burned, so white stays untouched whatever `minPower` is. `minPower` (default 0)
   lets a user keep faint pixels above the beam's threshold. Power is quantised by the pixel value itself (256 levels):
   no extra step.
4. **Dither Image.** Floyd-Steinberg error diffusion on the darkness image (`laserDither`), scanned row by row from
   the top, left to right: a pixel is dark (255) when its value plus the carried error is at least 128; the error goes
   7/16 right, 3/16 down-left, 5/16 down and 1/16 down-right (the last share is the remainder, so no error is lost;
   shares off the image are dropped). Integer arithmetic in 1/256 units: byte-identical on every platform and checked
   against exact rational arithmetic on reference rasters (4 images, 12 x 6 with every error path). After dithering the
   beam is at `power` or off, so `minPower` and the modulation switch have no effect (the form hides them). Dithering is
   done on the image grid, before lines are run; no other algorithm (ordered, blue noise) is offered.
5. **Pixel runs.** Each line is sampled **exactly**: the breakpoints are the crossings of the line with the pixel
   column and row edges, the stretch between two breakpoints takes the value of the pixel under its middle (nearest
   pixel, not bilinear: the picture is a raster, and a dithered image must stay two-valued), and neighbouring stretches
   of one value merge. A run of one value is one feed move with one power. The work counter is the number of edge
   crossings (limit `workLimit`, default 50 million), checked up front per pass and per vector, never a clock.
6. **Strokes and gaps.** Beam-off stretches inside a line: shorter than 5 mm are crossed with a beam-off feed
   (`jetPower` 0); a gap of 5 mm or more ends the stroke and the next one starts with a rapid. Leading and trailing white is
   dropped, so the beam never travels over an all-white end. Pieces of equal power merge only inside one straight
   segment (so a polyline's corners are kept). A line that burns nothing makes no stroke.
7. **Overscan.** As Laser Cut / Fill hatch lines: each Raster / Hatch stroke gets a rapid to the point `overscan` before
   its start, a beam-off feed to the start, the pieces, and a beam-off feed `overscan` past the end, along the line's
   direction of travel on that line (so the return lines overscan the other way). Selected vectors ignore it.
8. **Strategies.** Raster: lines at the line angle. Hatch: Raster, then the same at +90 degrees (the strokes of the
   second pass follow the first). SelectedVectors: each vector is polygonised at the chordal tolerance, closed ones return
   to their start, the pixel runs of its segments are chained into strokes (parts off the bitmap are beam-off); no overscan.
9. **Passes.** `passes` (the inventory's "No. Passes") repeats the whole path at the same Z 0, in order; the head rises
   to safe Z after the last. (Cut Depth belongs to Laser Cut / Fill; a picture has no depth.)
10. **Limits.** At most 1000 passes; line spacing 0.001..1000 mm; the move estimate (pieces + 4 per stroke) x passes
    is checked against `moveLimit` (default 20 million) as strokes are made; the line count against the move limit
    before any work. Cancellation is polled per line. Nothing partial is returned when a limit trips.
11. **`.carv`.** Operation `laser_picture`, params in the job.json dialect only (snake_case: `strategy`, `power`,
    `modulate_power`, `min_power`, `dither`, `invert`, `move_speed`, `passes`, `overscan`, `line_spacing`,
    `line_angle_rad`, `safe_z`, `tolerance`); enums by name, `passes` an integer, unknown nested keys dropped, bad
    enum / non-finite refused like the other records. The record check (`spindleOffProblem`) allows the `laser` tool
    marking on all laser operations (`isLaserOperation`). Format **1.16** (`kCarvFormatMinorLaserImages`) is written
    only when a picture or sketch record exists. 1.15 is reserved for the pending cutter compensation card, so
    this number leaves room for it; if `main` has moved when this branch is rebased, renumber (expect a textual
    conflict in `carv_io.cpp` / `carv_io.hpp`, resolve by taking both and keeping the larger minor for these records).
    No move cache change: version 2 (ADR-0099) already carries per-move power.
12. **Viewmodel.** `ToolpathOperation::LaserPicture`: form table (power, minPower, moveSpeed, passes, overscan, lineSpacing,
    lineAngle, safeZ, chordal; choices strategy, modulatePower, dither, invert; fields hidden as above), the end mill
    restriction, session calculation (bitmap decoded on the worker, cancellation), staleness (bitmap, and the vectors for
    Selected Vectors), refusals that say what is missing. The app entries, dialog and preview are T-0326.

## Alternatives
- Bilinear sampling like Image Engrave: smooth power ramps but a dithered image would no longer be two-valued, and
  the exact run boundaries (no sampling step) would be lost.
- Ordered (Bayer) or blue-noise dithering: Floyd-Steinberg is the one the reference names loosely ("dither");
  adding algorithms is a form choice later, not a format change (the record would need a key).
- A threshold parameter for the non-modulated mode: half dark is the natural reading; a user who wants another
  threshold lowers or raises the image's brightness first (T-0319 era adjustments).
- Power as a per-toolpath table: per-move power (ADR-0099) already carries it.

## Consequences
- T-0325's burn simulation reads `jetPower` and the move speed as for Cut / Fill.
- Product readings to confirm with the owner: modulation between `minPower` and `power` by darkness; non-modulated
  threshold at half dark; Floyd-Steinberg as "Dither Image"; 5 mm gap split; overscan only on lines; passes at one Z.

## Mutation notes
Hand mutants of the new logic were applied one at a time and the picture, sketch and record tests re-run; each was killed:
- dither threshold `>= 128` read as `> 128` (reference B: a uniform 50 % gray); the last error share read as a
  sixteenth instead of the remainder (survived the first references; killed by a 4 x 6 image found by search whose bottom-row
  last pixel flips, now a reference raster); non-modulated threshold `>= 128` read as `> 128`; `minPower` dropped from the
  modulation; beam-off gap split `>= 5 mm` read as `> 5 mm` (a gap of exactly 5 mm); the gap length not reset after a burned
  pixel (two 3 mm gaps must stay one stroke); the serpentine flip removed; the line count `ceil(range / spacing - 1e-9)`
  without the epsilon (2.1 / 0.3 = 7.000000000000001 must stay seven lines; survived the first test, which used a ratio
  that is exact in floating point); overscan applied to selected vectors.
Argued equivalent, not killable: in `pixelRuns` starting the edge loop at `floor(lo)` instead of `floor(lo) + 1`, or
including an edge at exactly the end of the line, only adds a breakpoint at t <= 0 or t >= 1, which the clamp turns into a
zero-length stretch that is skipped. In `clipSegment` the minimum length `1e-12` only decides between "nothing" and a
zero-length segment, and a zero-length segment has no runs.
