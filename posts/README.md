# Carvetto post-processors

The `.pp` files here turn Carvetto toolpaths into G-code for a particular controller. They use
the plain-text `.pp` template grammar (ADR-0004), interpreted as described in ADR-0011.

Every post was written for Carvetto from scratch. The only sources were the controller's own
public documentation (listed below) and general G-code knowledge. No text was taken from any
vendor's or community post file.

| File | Controller | Units | Extension |
|---|---|---|---|
| `grbl_mm.pp` | Grbl 1.1 | mm | `.nc` |
| `grbl_inch.pp` | Grbl 1.1 | inch | `.nc` |
| `mach3_mm.pp` | Mach3 | mm | `.tap` |
| `mach3_inch.pp` | Mach3 | inch | `.tap` |
| `mach4_mm.pp` | Mach4 | mm | `.tap` |
| `linuxcnc_mm.pp` | LinuxCNC | mm | `.ngc` |
| `linuxcnc_inch.pp` | LinuxCNC | inch | `.ngc` |
| `marlin_mm.pp` | Marlin 2.x (spindle build) | mm | `.gcode` |

## How the posts are tested

Every post runs against the same fixture (`core/tests/post/post_fixture.hpp`):

- a 50 mm square profile;
- a pocket with arcs: a full counter-clockwise circle, a clockwise circle written as two half
  arcs, and a counter-clockwise quarter arc;
- a change to a second tool;
- a peck-drilled hole (three pecks with partial retracts and a 0.5 s dwell) and a single-plunge
  hole with a 0.25 s dwell.

The output must match `core/tests/golden/post/<post>_fixture.nc` byte for byte. Each post must
also parse with no warnings and run with no warnings (`core/tests/post/shipped_posts_test.cpp`).

## Machine-safety checklist

A reviewer checked every golden file by hand against this list. The same checks also run
automatically on the goldens in `shipped_posts_test.cpp` ("AC2"), so if a later edit breaks
one, the test fails.

1. **Units.** The units code (`G21` mm / `G20` inch) appears before the first move and matches
   the post's `UNITS`. In inch posts, every length and feed has been converted (4 decimals for
   lengths, in/min for feeds).
2. **Absolute mode.** `G90` appears before the first move. Arc centres are incremental: posts
   whose controller has a setting for this write `G91.1`.
3. **Safe Z before rapids.** No rapid moves X or Y unless the tool is known to be at or above
   safe Z. At the start of the program, Z counts as unknown. It also becomes unknown again after
   a tool change pause (`M0`/`M6`), because the operator may have re-zeroed, and after `G43`. So
   every post lifts to safe Z before the first XY move and again after each tool change.
   Z-only rapids inside a hole (peck retracts) are allowed.
4. **Spindle on before plunge.** `M3 S<rpm>` (with a non-zero speed) comes before any
   `G1`/`G2`/`G3`. After a tool change the spindle is started again with the new tool's speed.
5. **Tool change.** The spindle is stopped (`M5`) and the tool is at safe Z before the change
   (`M0` pause or `T<n> M6`). The controller's own tool-change word is used.
6. **Dwell units.** The `G4` word is in the unit that controller expects (seconds or
   milliseconds, see the table below). The fixture's 0.5 s and 0.25 s dwells must come out as
   the expected words.
7. **Arcs.** For every `G2`/`G3`, the start and end points are the same distance from the centre
   (within 0.002 mm / 0.0002 in after rounding). A full circle keeps its end point equal to its
   start point.
8. **Spindle off and program end.** The program ends at safe Z with the spindle off (`M5`),
   followed by the controller's program-end code. Marlin has no program-end code, so it ends
   with `M5` (see below).

Hand-review result: all eight goldens pass all eight points.

## Dialect choices per controller

| Post | Start line | Tool change | Dwell | Program end | Line numbers / endings | Comments | Based on |
|---|---|---|---|---|---|---|---|
| Grbl mm / inch | `G90 G94 G17 G21`/`G20` | `M5`, safe Z, `M0` pause, safe Z, `M3 S` (no tool changer) | `G4 P` **seconds** | `M30` | none / LF | `( )` | Grbl v1.1 wiki: https://github.com/gnea/grbl/wiki |
| Mach3 mm / inch | `G90 G94 G91.1 G40 G49 G80 G17 G21`/`G20` | `M5`, safe Z, `T<n> M6`, safe Z, `M3 S` | `G4 P` **seconds** (Mach3's default); a General Config checkbox, commonly labelled "G04 Dwell in Milliseconds", is reported to switch to ms. Verify that it is off (if it is on, dwells get shorter, never longer) | `M30` | `N10`, `N20`… / CR LF | `( )` | "Using Mach3Mill" user manual, Artsoft / Newfangled Solutions: https://www.machsupport.com |
| Mach4 mm | as Mach3 | as Mach3 | `G4 P` **seconds**, per the official Mach4 Mill manual | `M30` | none / CR LF | `( )` | Mach4 Mill G-code programming manual, Newfangled Solutions: https://www.machsupport.com |
| LinuxCNC mm / inch | `G17 G40 G49 G80 G90 G91.1 G94 G21`/`G20` | `M5`, safe Z, `T<n> M6`, `G43 H<n>`, safe Z, `M3 S` | `G4 P` **seconds** | `M2` | none / LF | `( )` | LinuxCNC G-code reference: https://linuxcnc.org/docs/html/gcode/g-code.html and M-codes: https://linuxcnc.org/docs/html/gcode/m-code.html |
| Marlin mm | `G21`, `G90` (no `G17`/`G94`, see below) | `M5`, safe Z, `M0 <message>` pause (**needs an LCD or `EMERGENCY_PARSER`, otherwise ignored**), safe Z, `M3 S`; **no T word** | `G4 P` **milliseconds** (multiplier 1000) | none, see below | none / LF | `;` | Marlin G-code reference: https://marlinfw.org/docs/gcode/ (G0-G1, G2-G3, G4, M0-M1, M3, M5) |

Every post uses `G2`/`G3` with `I`/`J` measured from the arc start, a feed (`F`) only when it
changes, and modal X/Y/Z (written only when they change).

### Controller notes

> **Marlin: the tool-change pause needs an LCD controller or `EMERGENCY_PARSER`.**
> Marlin's tool change relies on `M0`. Without an LCD controller or `EMERGENCY_PARSER`
> (resume with `M108`), Marlin **ignores `M0`**: the tool change is skipped and the next
> toolpath runs with the previous tool. Source: https://marlinfw.org/docs/gcode/M000-M001.html
>
> **Mach3 / Mach4: tool changes must not be set to "ignore".** If the controller is set to
> ignore tool changes (Mach3: the tool-change option in Config > General Config; Mach4: the
> profile's tool-change setup / `m6` macro), `M6` does not pause and the next toolpath runs
> with the previous tool. Configure `M6` to stop the spindle and wait for Cycle Start, or to
> drive an automatic tool changer.

- **Grbl** only supports incremental arc centres, so it needs no `G91.1`. It has no tool
  changer, so a tool change pauses with `M0`. `grbl_mm.pp` (from T-0030) gained the safe-Z lift
  after the pause in T-0110.
- **Mach3 / Mach4.** The arc-centre mode ("IJ mode") is a controller setting, so the start line
  forces incremental `I`/`J` with `G91.1`. The posts do not write `G43`. If your tool table holds
  length offsets, add `G43 H[T]` after `T[T] M6`. Dwells are written in seconds.
  - Mach4: the official Mach4 Mill G-code manual gives `G4 P` in seconds. It does not mention a
    milliseconds option.
  - Mach3: seconds is the default. A General Config checkbox, commonly labelled "G04 Dwell in
    Milliseconds", is reported to switch `P` to milliseconds. The official wording could not be
    confirmed, so check this on your machine. Seconds is the safe choice either way: with the
    option on, a dwell becomes very short, never minutes long.
  - Both controllers can be set to ignore tool changes. See the warning above.
- **LinuxCNC** applies the tool length offset with `G43 H<n>` after every `M6` and lifts to safe
  Z after that, so the lift uses the new tool's offset. `M2` ends the program.
- **Marlin** needs a spindle build: `SPINDLE_FEATURE` with `CUTTER_POWER_UNIT` set to RPM, so
  that `S` means rpm. For PWM255 or PERCENT, edit the `SPINDLE_SPEED` multiplier. Some builds only
  have `M3`; this post never uses `M4`. Arcs need `ARC_SUPPORT` (on by default).
  - The tool-change pause (`M0`) needs an LCD controller or `EMERGENCY_PARSER`; see the
    warning above.
  - `G21` needs `INCH_MODE_SUPPORT`. Without it, Marlin ignores or reports `G21` and stays in
    millimetres, which is harmless for this post.
  - The post writes no `G17`, because Marlin only accepts it when built with
    `CNC_WORKSPACE_PLANES`, and XY is Marlin's plane otherwise.
  - It writes no `G94`, because Marlin feeds are always per minute.
  - It writes no `T`, because on Marlin `T` selects an extruder.
  - It writes no `M30`, because on Marlin that deletes a file from the SD card.
  - Comments use `;`, because Marlin only understands `( )` with `PAREN_COMMENTS`.
  - Marlin runs `G0` at the last feed rate, so rapids are slower than on a CNC controller.
- **Spindle spin-up.** None of the posts add a dwell after `M3`. Marlin waits for its configured
  power-up delay. On Grbl, Mach and LinuxCNC, configure spindle-at-speed (or add a dwell to
  `SPINDLE_ON`/`HEADER`) if your spindle needs time to reach speed.

## Adding or changing a post

1. Write it from the controller's public documentation only, and add a row to the table above.
2. Add it to `shippedPosts()` in `core/tests/post/shipped_posts_test.cpp` with its units, tool
   change, program end and dwell words.
3. Generate its `<stem>_fixture.nc` golden. Review it by hand against the checklist, then commit
   it with the post.
