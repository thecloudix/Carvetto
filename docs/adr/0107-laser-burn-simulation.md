# ADR-0107: Laser burn-rate simulation, the machine laser flag, LASER_SUPPORT and the laser post

Date: 2026-11-16 · Status: proposed (T-0325)

## Context
T-0323 (ADR-0099) emits laser toolpaths: beam power per move, `ToolSnapshot::laser`, the JET_TOOL_* post blocks. The
reference (feature-inventory "Preview and simulation", "Machine configuration and post-processors", "Adapting a
post-processor for Lasers") asks for a simulation that marks the surface with each laser tool's Maximum Burn Rate, a
machine/post notion of laser support, and a laser-capable post. The card leaves the burn model, what the flags do and
the post's dialect open. This ADR records the readings.

## Decision
1. **Burn model.** The beam is a disc of the tool snapshot's diameter (kerf / spot size). Its centre follows the
   moves; at a cell centre the **dose** is the sum over beam-on moves of
   `power/100 * maxBurnRate / speed * L / D`, where `L` is the length of the move's path inside the disc of radius
   `D/2` around the cell centre (exact line/circle clipping; arcs are cut into chords with sagitta <= cell/64, the
   sweep rule of ADR-0032) and `speed` is the move's feed override or the snapshot's feed rate (plunge rate for
   Plunge/Retract). Reading of "Maximum Burn Rate": the speed in mm/min at which a 100 % beam just burns the surface
   fully (darkness 1) along its centre line; slower or stronger beams over-burn, faster or weaker ones mark lighter.
   **Darkness** is `min(1, dose)`; the raw dose is kept (up to 1e6) so a later reading of "depth" can use it. Doses of
   passes, repeats and overlapping strokes add (a hatch with a pitch below the beam diameter darkens where lines
   overlap). Because the dose integrates path length, a polyline split into two moves at any point gives the same
   doses as one move (tested to 1e-5), and a beam moving at half the speed doubles the dose.
2. **A laser removes no material.** Laser runs do not touch the heightfield, raise no collision diagnostics
   (no tool body) and are not part of the cutting work counter except through the shared cell-work budget. "Mark
   depth" of the card is read as darkness: a cut-through or burn-depth heightfield change is not modelled (a laser Cut
   at low speed simply saturates darkness). The burn layer is a `float` per cell (4 bytes), allocated when the first
   laser toolpath is queued, so a job without a laser costs nothing extra.
3. **API.** `Simulator::addLaserToolpath(path, maxBurnRate, id, from)`: `path.tool.laser` must be set, the burn rate
   finite and > 0 (an unset rate is refused as `InvalidTool` with a message naming the cutting data, instead of
   silently marking nothing), the beam diameter usable. It queues, times and validates exactly like `addToolpath`;
   laser and cutting toolpaths mix. `hasBurn`, `burnDoses`, `darkness`, `markedCells` read the layer. A move marks
   the surface when it completes: `advanceToTime` applies no partial dose (a function of `movesDone` only, so seek
   back and forward reproduces the layer exactly); `reset` clears it. The burn layer allocation has its own test
   hook bit (4).
4. **Laser tool.** Still no `ToolType::Laser`. ADR-0099's end-mill-with-kerf-diameter reading is kept: the cutting
   data already carries `maxBurnRate` (tool database format 2, `max_burn_rate_mm_min`), and a new tool type would
   bump the tool database and `.carv` tool encodings for no behaviour. `ToolSnapshot::laser` marks the toolpath.
   The viewmodel's `SimulationToolpath::maxBurnRate` is read from the tool's cutting data for the toolpath's machine
   and material (the usual fall back to default data); a laser toolpath whose tool has none makes the setup fail with
   a message to set it.
5. **Machine flag.** `tooling::Machine::laser` (tool database key `"laser"`, written only when true, so every
   database writable today is byte-identical and no format bump is needed; older readers ignore the key). Where it
   acts: `post::laserSetupProblem(machine, post)` is the single check for "may this laser job go to this machine and
   post": the machine must be flagged, the post must declare `LASER_SUPPORT = "YES"` and have JET_TOOL_ON/OFF. The
   app (T-0326) calls it before writing a job with a laser toolpath. No form field was added (app wiring is T-0326).
6. **`LASER_SUPPORT` acts.** The interpreter now refuses a laser toolpath when the post does not declare it (message
   names toolpath and setting), before the existing missing-block refusal. This tightens ADR-0099, which accepted any
   post with the blocks; it matches the reference's "laser-capable post" and is cheap to satisfy (one line). A job
   without a laser toolpath is unaffected by the flag, whatever its value.
7. **The post.** `posts/grbl_laser_mm.pp`: Grbl 1.1 laser mode (`$32=1`, `$30=1000`): `S` is power (percent x 10),
   beam on `M4`, off `M5`, power word before ON and at each change. Laser-only dialect: the header and new-segment
   blocks write no spindle start; `SPINDLE_ON` (`M3[S]`) remains for non-laser toolpaths so an end-mill toolpath
   never runs with the spindle off. It is not in the spindle fixture list (that checklist requires `M3` before a cut);
   its own golden `core/tests/golden/post/grbl_laser_mm_laser.nc` and tests check the beam rules (power before on,
   never a rapid lit, ends dark, no M3 in laser output). `docs/laser-post-guide.md` is the adaptation guide.
8. **Viewmodel.** Minimal: the setup carries the burn rate, the worker and the component preview queue laser
   toolpaths through `addLaserToolpath`, `applyBurnMarks` darkens the frame raster towards a char colour by the
   largest darkness of the cells a pixel stands for, and `SimulationFrame::burnedCells` counts marks. Full app
   wiring (entries, forms, overlays) is T-0326.

## Mutants considered
- Dropping the `/ D` or `/ speed` or the `power/100` factor: the chord-factor, power, speed and feed-override tests
  check absolute doses to 1e-5. Using the plunge rate for feeds (or vice versa): the plunge test uses a different rate.
- Clamping `t` to [0, 1] removed (marking beyond a segment's ends): the split-polyline test and the cell
  1 mm past the line's end (must be 0) fail. Using `>=` for `disc` (grazing contact, zero length): equal result,
  because `t2 > t1` is required afterwards, so it is equivalent and not distinguishable by output.
- Darkness not clamped / dose clamped at 1 (losing the excess): the slow-pass test reads the raw dose 4 x chord.
- Marking rapids, beam-off feeds or dwells: the beam-off test requires zero marked cells.
- Partial dose in time playback, or not clearing on seek back: the playback test checks `markedCells`, exact
  equality of the layer after seek, and `reset`.
- Burn on the heightfield or diagnostics: the removal test checks `cutCells`, heights and diagnostics.
- Cell range off by one at the stock edges: the edge test marks a beam half outside the stock and checks first and
  last columns and an unmarked row.
- An arc marked as its chord, or as the wrong sweep: the circle test checks the ring and the far side and the
  quarter arc test an unmarked opposite quarter.
- `laserSetupProblem` returning ok for an unflagged machine / undeclared post / missing block: one test each.
- The interpreter's `LASER_SUPPORT` refusal removed: the NO test fails; the refusal applied to non-laser
  toolpaths: the mill job in the same test fails.
- `"laser"` written when false: the single-occurrence and absent-key checks.
- Equivalent survivors: dropping the Dwell clause in `burnMove` (a dwell has zero XY length and no power, so it adds
  dose 0); dropping the `jetPower > 0` guard (zero power multiplies the dose by 0); `markedCells` using the raw dose
  rather than min(1, d) (the threshold is at most 1, so both comparisons agree).
- Open arcs over 180 degrees: the 270-degree CW and CCW test checks doses against a dense-sampling oracle.
- Laser moves stamped as cuts in time playback: the playback test compares heights with an untouched simulator.
- Machine `"laser": false` read as true: the machine JSON test.
- Survivor accepted: the `disc > 0` equivalence above; the 100 000-chord cap of one laser arc is a safety bound not
  reached by a test (an arc of that fineness needs a radius of 1e7 mm on a 1e-3 mm grid).

## Alternatives
- A `ToolType::Laser`: tool database and `.carv` bump; rejected above.
- Stamping the dose per move once per cell (a mask): double counts nothing at joints, but cannot represent
  overlapping passes or power modulation (T-0324); the exact path-length dose does.
- Refusing nothing: an unset burn rate marking nothing hides a setup error; refused instead.
- Modelling depth by lowering the heightfield proportionally to dose: needs a material removal rate per material
  that the data model does not have; left for an owner decision.

## Consequences
Pictures with modulated power (T-0324) simulate through the same per-move power. Posts that defined the jet blocks
but not `LASER_SUPPORT` stop working for laser toolpaths until the line is added.
