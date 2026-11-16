# ADR-0113: Laser app wiring: Add entries, source widgets, the save check, the machine flag, overscan inside the limits

Date: 2026-11-16 · Status: proposed (T-0326) · Builds on ADR-0099 (Laser Cut / Fill), ADR-0104 (Laser Picture),
ADR-0105 (Laser Sketch Engraving), ADR-0107 (burn simulation, `laserSetupProblem`, the machine flag), ADR-0106 and
ADR-0111 (the Drill Bank wiring this follows)

## Context
The core, the viewmodel forms and sessions, the simulation of burn marks and the laser post existed (T-0323, T-0324,
T-0325); the app had no entry for any of them. T-0324's verifier also noted that with the machine limit boundary on, the
burn is clipped to the limits but the beam-off overscan moves are not. The inventory says nothing about where the
entries sit or how the form asks for the source; the readings below are local and marked **(reading)**.

**Audit of what the app already did for a laser toolpath** (before this task): the Toolpaths list shows the operation by
`operationLabel` (the three laser names were already there) and the job sheet names it by `operationName`
(`laser_cut_fill`, `laser_picture`, `laser_sketch`), so job-sheet rows needed no change; the simulation and the component
preview already queue a laser toolpath through `Simulator::addLaserToolpath` with the tool's maximum burn rate, and
`applyBurnMarks` shades the frame; staleness already includes the bitmap, the relief model and the machine limits
(`inputsFingerprint`, `machineLimitsFingerprint`). Missing: the Add menu entries, a source box that follows the
sketch's source choice, the save-time check, the machine flag in the setup dialog, the self-test, and the overscan rule.

## Decision

### 1. Add menu and forms
The Toolpaths Add menu lists Laser Cut / Fill, Laser Picture and Laser Sketch Engraving after Project onto Model
(objects `addLaser Cut / Fill`, `addLaser Picture`, `addLaser Sketch Engraving`). The forms are the generic
`ToolpathFormDialog` over the viewmodel's field tables; nothing laser-specific is drawn by hand.

The source box ("Bitmap" or "Vectors") used to be fixed by the operation (Image Engrave: bitmap). It now follows the
parameters: `usesBitmap(params)` (Image Engrave, Laser Picture, a Laser Sketch with the bitmap source) shows the bitmap
box (no layer / open / closed / circle filters: the selected bitmap, plus the selected vectors when
`boundaryUsesVectors`: Selected Vectors strategy, the sketch's Vectors boundary); `readsModel(params)` (3D Rough,
3D Finish, Project, a sketch with the model source) shows the model text and checks `modelProblem`. Choosing the sketch's
Source re-evaluates both, so the box changes while the form is open. `boundaryUsesVectors` became a public viewmodel
function so the form and the calculation use one rule (it was already the staleness and job rule). **(reading)** A form
switched from the model to the bitmap source with "all vectors" selected starts from the current selection, as Image
Engrave does when it opens; the user can still pick what to burn by selecting before opening the form.

### 2. The save-time check
`SaveOptions::checkLaserSetup` and `SaveOptions::machine`: when set and any posted record carries a laser tool
(`ToolSnapshot::laser`, with moves), `prepareSave` parses the post and refuses the job with
`"this job has a laser toolpath, but " + post::laserSetupProblem(machine, post)`; the text reaches the Save Toolpaths
dialog's message as any post error does, and Save stays disabled. The dialog turns the check on always and passes the
machine picked in its machine list; with "(no machine)" only the post is checked (the machine flag is a property of a
machine, and a user who picks none states no machine). Off by default, so the command line and every other caller are
unchanged. A job without a laser toolpath is never checked, whatever the flags. The interpreter's own refusal for a post
without `LASER_SUPPORT` (ADR-0107 §6) stays as the second line of defence; the UI message comes first and names the
reason (machine flag, post declaration, or missing block).

### 3. The machine flag in the setup dialog
The Machines page of "Tools, Machines and Materials" has a "Laser machine" check box (`machineLaser`) next to the
limits; it edits `Machine::laser` through the same commit path as the other machine fields (validation, undo-free
database edit, written to the setup file only when set, ADR-0107 §5). Cutting data already had the "Max burn" column.

### 4. Overscan inside the machine limits **(reading)**
The verifier's note: with the Laser Sketch machine limit boundary on, the burn is clipped, but the beam-off overscan
runs before and after a stroke can leave the limits and drive an axis past its end. **Clamping is right**: the boundary
is a statement about where the machine can move, and a beam-off run is a move. `emitStrokes` takes the limits and cuts
each overscan run down to the part that stays inside (`overscanWithin`: the distance along the stroke's direction to
the limits' border, at most the requested overscan, 0 for a point already on or outside the border); a run that is
cut to nothing is left out (the rapid goes straight to the start, no approach feed; no run-out feed). The overscan is
then shorter exactly where the burn touches the border, which is where the beam still reaches full speed because the
border *is* the end of the travel the machine has. Not clamped without the boundary or without limits (the flag is the
user's statement that the limits matter), and not for Laser Picture, whose forms have no machine limit boundary (ADR-0104
lists none). The alternative, leaving the moves alone and documenting it, was rejected: it makes the flag promise less
than its name.

### 5. Preview, simulation and job sheet
No new code: the existing paths act (see the audit). The self-test proves the simulation marks cells for each kind
(`SimulationFrame::burnedCells`) and shades the picture; the job sheet row is covered by `operationName`
(tested with the other names in `toolpath_laser_image_test.cpp`).

### 6. Self-test
`laserSelfTest` (`app/src/laser_self_test.cpp`, run by `carvetto --self-test`, step 9b1): the three Add entries, the
beam tool and its burn rate, Cut / Fill with hatch (power bounds, edit in one undo step, stale after moving the vector,
recalculate, burn marks in the simulation), the Save Toolpaths dialog refusing a non-laser post and an unflagged machine
and accepting the laser post with a flagged machine, a PNG imported and Laser Picture (Selected Vectors needs vectors,
dither hides the minimum power, burn only over the dark half), Laser Sketch with the bitmap source (the Vectors
boundary needs vectors, machine limit boundary with overscan keeps every move in the limits, the source switch), stale
and "Recalculate all", a model on the sheet and a model-source sketch, a final simulation, save and reload. The setup
dialog's self-test sets and clears the laser flag.

## Alternatives
- A separate dialog class per laser operation: the generic form has every field already; a class would only repeat
  the vectors / bitmap box logic.
- A viewmodel-level `laserProblem()` the panel calls when the toolpath is added: the card says "where a laser toolpath
  is saved/posted"; the add itself needs neither a machine nor a post.
- Making the machine flag mandatory for "(no machine)": there is nothing to check; ADR-0107 §5 takes the same line.

## Consequences
- ADR-0105 §8 stays; the limit applies to the overscan as well (this ADR).
- Product readings to confirm with the owner: overscan clamped to the limits; no machine chosen means only the post
  is checked.

## Mutation notes
Hand mutants of the new logic were applied one at a time and the tests re-run:
- Overscan clamp (core, `laser_sketch_test.cpp`): the X border of the start run read as the lower limit (survived the
  first tests, which only checked "inside the limits"; killed after the run-out's end was asserted at the limit, 10.5
  and 12 mm); the lower Y branch read as the upper one (survived, killed by the vertical-line case with limits -1 .. 11);
  `before > 0` read as `>= 0` (an empty approach feed appears); the limits not given to the run-out; not given to the
  approach; the approach run taken along the stroke instead of against it. All killed.
- Save check (viewmodel, `toolpath_laser_test.cpp`): the laser test inverted; the machine dropped (only the post is
  checked); the check run without `checkLaserSetup`; the message prefix removed. All killed.
Argued equivalent: the `r.hasMoves()` term of the laser test (`makeSaveSource` already leaves out records without moves,
so every posted record has them; the term keeps the check about what is written). The old `std::max(0.0, t)` floor of
`overscanWithin` was removed instead of tested: a negative distance only arises for a point a rounding error outside the
border and is never used (`> 0` guards both uses). The form dialog's source box has no unit mutants: a wrong
`usesBitmap` / `readsModel` / `boundaryUsesVectors` routing is a `std::get` failure or a wrongly enabled OK in the
self-test, and the three predicates are tested in `toolpath_laser_test.cpp`.
