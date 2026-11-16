# ADR-0099: Laser Cut / Fill toolpath, beam power on moves, jet blocks in the post

Date: 2026-11-16 · Status: proposed (T-0323)

## Context
T-0323 adds the first laser toolpath (feature-inventory "Laser Module", "Laser Cut - Fill"): power, move speed,
cut depth giving a number of passes, repeats and overscan; the strategies cut outside/right, cut inside/left (both
kerf compensated), cut on and hatch fill (stepover, angle, cross hatch, fill together or regionally, outline);
an allowance. The output goes through the post's JET_TOOL_POWER / JET_TOOL_ON / JET_TOOL_OFF blocks, which ADR-0011
parsed but never emitted. The card leaves open how the beam is represented in a move list and in `.carv`, what a
"laser tool" is, and what the interpreter does with a toolpath that has no spindle. This ADR records the readings.

## Decision
1. **Laser tool.** No new tool type: the beam is an **end mill** tool whose diameter is the kerf / spot size
   (`laserToolAllowed`); its cutting data supplies the default speed (feed rate), the pass depth and, for T-0325's
   simulation, `maxBurnRate`. Power, speed, passes, repeats and overscan are parameters of the toolpath (they are
   what the form edits), not tool-database fields. A dedicated laser tool type (tool database format bump) is left
   to T-0325, which owns laser tools and machines.
2. **Beam power is a per-move value.** `Move::jetPower` (percent, 0 = beam off, otherwise (0, 100]; cutting
   kinds only; rapids and dwells run with the beam off). Per move rather than per toolpath so that T-0324's power
   modulation (dithering, pictures) needs no second change. `ToolSnapshot::laser` marks a laser toolpath: no spindle
   (the post writes no spindle start and accepts spindle speed 0; the marking is exclusive with `spindleOff`).
   Percent is the unit; posts scale it with the VAR multiplier (`VAR P = [JET_POWER|A| S|1.0|10]` gives S0..S1000).
3. **Post.** New tag `JET_POWER` (current beam power, percent). Before each move the interpreter compares the
   move's power with the beam state: JET_TOOL_POWER when the power differs from the one last set in the toolpath,
   JET_TOOL_ON when the beam starts, JET_TOOL_OFF when it stops (a rapid, a dwell or a feed with power 0), and OFF
   again at the end of a toolpath that ends lit, so the beam is never on across a toolpath, tool change or the footer.
   JET_TOOL_POWER is optional (a post may fold the power into ON); a laser toolpath needs JET_TOOL_ON and
   JET_TOOL_OFF, else the run is refused naming the block. Beam power on a move of a non-laser toolpath, outside
   (0, 100] or on a rapid is a run error. SPINDLE_ON is skipped for a laser toolpath. `LASER_SUPPORT` stays parsed only
   (T-0325 ships the laser-capable post and the machine flag); the refusal is by missing blocks, so a post that
   defines them works without the flag. The changes are confined to `jetBefore`, the toolpath loop, the spindle checks
   and one tag.
4. **Strategies.**
   - Cut Outside / Inside / On run the **profile machinery** (side by nesting parity, kerf = tool radius, allowance,
     direction, order, start points, trimming of overlapping cuts) with one level and no tabs, leads or ramps; each
     profile cut (plunge ... retract) becomes a stroke of feed and arc moves with the beam on. Closed vectors:
     Outside keeps the region (outside outer boundaries, inside holes); open vectors: Outside is the right of the
     direction of travel, Inside the left. This is how the reference describes "Cut Outside/Right, Cut Inside/Left".
   - Hatch Fill takes the closed vectors as one region (even-odd union, so a vector inside one other is a hole), insets
     it by the beam radius plus the allowance (so the burn stays inside the vector), and fills it with lines on the
     multiples of the stepover across the hatch direction (aligned to the origin, so a cross hatch at +90 degrees and
     neighbouring runs line up), run as a serpentine (direction alternates per line that has a segment; several segments
     of one line run in the line's direction). Together = lines cross every region before the next line; Regionally =
     each outer boundary with its holes is filled completely before the next (regions ordered by the cut order options).
     Open vectors are ignored with a diagnostic. Outline adds the inset region boundary as cuts after the fill (climb:
     outer rings run clockwise).
   - Overscan applies to hatch lines only: each line gets rapid to the point `overscan` before its start, a feed with the
     beam off to the start, the cut, a feed with the beam off `overscan` past its end. The cut strategies ignore it.
5. **Passes and repeats.** `cutDepth` 0 means one pass at Z 0 (marking). Otherwise the passes are
   `PassPlan{0, cutDepth, passDepth}.levels()` (pass depth 0 = the cutting data's; none = one pass at the bottom) and
   every pass runs at its own Z (focus follows the cut, as on lasers with a motorised Z; a post for a fixed-focus laser
   simply does not write [Z]). Repeats run each pass over again at the same Z, in the order pass, repeat, strokes. All
   rapids between strokes stay at the pass Z (the beam is off); after the last pass the head rises to safe Z.
   More than 1000 passes, 1000 repeats, 2 million hatch lines or 50 million line/edge crossings, or more moves than the
   run's move limit (default 20 million), are refused up front (`WorkLimitExceeded`); cancellation is polled every 1024
   crossings. All limits are counters, not clocks.
6. **`.carv`.** Operation `laser_cut_fill`, params in the job.json dialect only (snake_case, angles as `_rad`,
   enums by name, `repeats` integer; unknown nested keys dropped, bad enum/non-finite refused, like the other records).
   Tool object gains `"laser": true` (written only when set). The record check (`spindleOffProblem`, used by the commands,
   `saveCarv` and the loader) allows `laser` on laser records only and not with spindle-off or a tool type; a laser
   operation with an unmarked, not yet computed snapshot is fine. Format **1.13** is written only when a laser record exists
   (minor bump rules of ADR-0021, `kCarvFormatMinorLaser`). The move cache gains **version 2**, written only when some move
   has power: each move is followed by one flag byte (bit 0 power follows, bit 1 same as the last written) and the power
   double unless repeated; version 1 caches are byte-identical to before. The decoder validates flags, power range and
   cutting-kind-only, and the loader's size bound per move rises from 49 to 58 bytes. Existing files and builds that
   only know 1.12 are unaffected; a 1.12 build refuses a 1.13 file with the usual "please update" message.
7. **Viewmodel.** `ToolpathOperation::LaserCutFill` has a form table (power, moveSpeed, cutDepth, passDepth, repeats,
   overscan, allowance, stepover, hatchAngle, safeZ, start, chordal; choices strategy, direction, crossHatch, fillMode,
   outline, startAt, order; hatch-only fields hidden for the cut strategies), session calculation with cancellation, the
   end mill restriction and the CLI operation name. The app (Add-menu entry, dialog, preview, job-sheet rows, self-test)
   is T-0326; only the operation label in the form dialog was added so the app still builds.

## Alternatives
- A `ToolType::Laser` now: needs a tool database format bump and touches the simulator and tool shape code, which T-0325
  owns; the end-mill reading costs nothing and is replaceable.
- Beam power per toolpath: simpler but would force a move-format change again for power modulation (T-0324).
- Encoding laser on/off as move kinds: the kind field has one spare value; a power value is needed anyway.
- Hatching with Clipper2 open-path clipping: heavier than the exact scanline crossing used here, which is also
  deterministic and counts its work.

## Consequences
- Merging with the cutter compensation card (T-0327): both add a variant alternative, an enum value, key tables and
  a `.carv` minor; resolve by taking both and renumbering the minor (this card takes 1.13).
- T-0324/T-0325 build on `Move::jetPower`, `ToolSnapshot::laser`, the `JET_POWER` tag and `laserCutFillToolpath`.
- Product readings to confirm with the owner: the end-mill beam model, repeats at the same Z, overscan on hatch only,
  the inset by beam radius for hatch fill, outline after fill.


## Mutation notes
Two hand mutants survived the first verification and are now killed by tests in `laser_test.cpp`:
- Hatch inset sign (`r + allowance` read as `r - allowance`): "a hatch allowance insets the filled region" fills a
  10 mm square with allowance 1.4 and asserts seven lines (y = 2..8) spanning x 1.5..8.5.
- Overscan gate (`overscan > 0.0` read as `> 0.5`): "a small overscan still adds the overscan moves" uses 0.2 mm
  and asserts the four-moves-per-stroke layout with 0.2 mm off-beam runs.
