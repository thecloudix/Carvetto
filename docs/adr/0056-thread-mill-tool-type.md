# ADR-0056: Thread-mill tool type and Create Circles (T-0262)

Date: 2026-11-13 · Status: accepted, implemented (T-0262). Points marked **(owner)** are product readings
to be confirmed.

## Context
The inventory describes thread mills as single or multi point cutters with "S size, H height, D diameter,
A angle, O offset, plus threaded length for multi-point", listed in the Thread Milling form. T-0237 made the
toolpath accept an end mill as a stand-in. The inventory gives only the letters, so their meaning is read here.

## Decision
1. **New `ToolType::ThreadMill`** (`"thread_mill"`). Fields: `diameter` = D, the diameter over the teeth (the
   cutting diameter the toolpath uses, as before); `includedAngle` = A, the angle of the tooth form (60 degrees
   for ISO/unified); `shankDiameter` = S **(owner)**; `toothHeight` = H, the radial height of a tooth
   **(owner)**; `toothOffset` = O, the distance from the tip to the first tooth **(owner)**; `threadedLength`
   = the toothed length. A `threadedLength` of 0 is a single point cutter (one tooth), above 0 a multi point
   cutter. Validation: D > 0, A in (0, pi), 0 < H <= D/2, O, S, threaded length >= 0.
2. **The tool states the teeth.** With a thread-mill tool, `threadMillToolpath` takes single/multi point and the
   toothed length from the tool and ignores `ThreadMillParams::cutter`/`threadedLength` (they still apply to an
   end mill stand-in, which stays allowed). H, O, S and A are stored and shown but do not change the helix (the
   path radius depends on D only, ADR-0037); the simulator draws the cutter as a cylinder of diameter D.
3. **Tool database file version 2.** `kToolDatabaseFormatVersion` becomes 2 and the tool objects gain
   `shank_diameter_mm`, `tooth_height_mm`, `tooth_offset_mm`, `threaded_length_mm`. The importer accepts versions
   1 and 2: it reads the new keys only from version 2 (a version 1 file cannot hold them) and refuses
   `thread_mill` in a version 1 file as an unknown tool type; no migration step is needed because every new
   field defaults to 0. Export always writes version 2. An older build reads a version 2 file as "unsupported
   version" (the rule for the container formats too: a newer file is refused, never guessed at).
4. **Create circles.** With "Create circles for internal threads" on, the calculation also returns the bore
   circles (`ToolpathCalculation::boreCircles`, from `threadBoreCircles`), and `ToolpathSession` adds one vector
   object per hole (named "Thread bore") on the layer and sheet of the vector the thread was found in, inside
   the same undo macro as the toolpath: one undo removes both. A circle whose sheet and bounds already exist in
   the document is not added again, so a recalculation does not duplicate them. The toolpath's baseline vector
   fingerprint is taken after the insertion, so it does not report stale because of its own circles. (If the
   selector is "all vectors", a later recalculation sees the circles as input like any other vector.)
5. The Thread Mill form lists thread mills and end mills (`threadMillToolAllowed`); the tool setup editor shows
   the four extra fields for the type.
