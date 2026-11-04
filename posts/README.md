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

A second fixture (T-0154) runs diamond-drag toolpaths, which run with the spindle off: a drag
toolpath first in the job, a second one with the same tool, a profile with the end mill, and a
drag toolpath again after the tool change. Its output must match
`core/tests/golden/post/<post>_drag.nc` byte for byte.

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
   A peck retract may be an upward Z-only rapid inside an already-drilled hole (the drilling
   toolpath's "retract above previous pass" chip breaking, owner decision 2026-11-03, ADR-0022),
   never sideways and never downward below safe Z.
4. **Spindle on before plunge.** `M3 S<rpm>` (with a non-zero speed) comes before any
   `G1`/`G2`/`G3`. After a tool change the spindle is started again with the new tool's speed.
   The only exception is a toolpath that runs with the spindle off (point 11).
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

9. **Names cannot become G-code.** Text tags (`[TOOLNAME]`, `[TOOLPATH_NAME]`, `[TP_FILENAME]`,
   …) sit inside a comment: `( … )`, or after `;` on Marlin. The interpreter replaces control
   characters (newline, CR, tab, NUL …) in every text value with spaces, and inside an open
   `( … )` comment turns `(`/`)` into `[`/`]`, so no name can start a new block or close the
   comment early. Every text value is printable ASCII: `~ ! ?` and non-ASCII characters (Grbl
   realtime commands, even inside comments) become `_`, accented Latin letters their plain
   letter. Values cannot form an active comment such as LinuxCNC's `(LOGOPEN,file)` or Mach's
   `(MSG,…)`, alone or together: while a comment holds only words and spaces, commas in a value
   that continue the words become `_`, and a value completing PROBEOPEN/PROBECLOSE/LOGCLOSE gets
   a leading `_`. In a custom post, do not put template text right after a text tag at the
   start of a comment (ADR-0020 lists this gap); every shipped post follows a text tag only with
   `)`, the end of the line or another text tag, and a test checks this. The parser refuses
   control characters other than tab in `.pp` lines. `[TOOLS_USED]` separates names with `; `. Keep `SUBSTITUTE = "([)]"` in paren-comment posts anyway, for
   readers of the post. `post_safety_test.cpp` checks every shipped post with hostile names and
   that text tags appear only inside comments.
10. **Numbers are finite and in range.** The interpreter refuses to write a non-finite number,
    a length, feed, spindle speed, dwell or angle beyond 1e6 (mm, mm/min, rpm, s, degrees), or
    any value that would be written beyond 1e9 after unit conversion and the VAR multiplier
    (inverse time may reach 1e9); the arc tolerance must be 1e-6 to 1 mm and a linearised arc has
    at most 100,000 chords; a wrapped job's rotary diameter must be finite and at most 1e6 mm;
    the post run fails with an error naming the toolpath, move and value (ADR-0020). A tool's
    spindle speed must be above zero wherever `[S]` is written or `SPINDLE_ON` runs (for a
    manual spindle, set any positive nominal speed), and a feed, spindle speed or dwell above
    zero that the post would print as zero (`F0`, `P0.00`) is refused, as is a written feed of
    zero or less and a dwell of zero seconds or less.

11. **Spindle-off toolpaths (diamond drag, T-0154).** A diamond-drag Quick Engrave toolpath
    runs with the spindle off. Before it the post writes its `SPINDLE_OFF_PAUSE` block: `M5`,
    safe Z, a comment saying that the spindle stays off for the drag tool and asking the operator
    to check that it is stopped or locked, a bare `M0` (on every post; Marlin's message is the `;`
    comment before it), and safe Z again. From that pause until the next tool change stop there is no
    `M3`/`M4` and no `S` word; the spindle starts again only after a tool change (`M0` or `M6`)
    with the next tool's speed. The interpreter leaves out the spindle start lines of HEADER,
    TOOLCHANGE and NEW_SEGMENT for such a toolpath and never writes SPINDLE_ON; it refuses a
    post without `SPINDLE_OFF_PAUSE` (or whose block writes no `M5` or no `M0`; lines starting
    with `/` and text after `;` do not count), a line that
    writes `[S]` together with other codes, and any spindle start or speed word in the drag
    toolpath's output, read as a controller reads it (spaces ignored: `M 3`, `S .5`, `M+3`
    count), and a spindle-off toolpath whose tool is not a diamond drag tool. No other toolpath can skip
    the spindle: only Quick Engrave marks a toolpath spindle-off, and only for diamond-drag tools
    (ADR-0020, ADR-0024). After the last drag toolpath of a drag section, before a toolpath that
    starts the spindle again, the post writes its `SPINDLE_UNLOCK_PAUSE` block: `M5`, safe Z, a
    comment asking the operator to unlock the spindle, a bare `M0`, safe Z; once per drag section, not between drag toolpaths, and also before the
    program end when the job ends with drag toolpaths. A post without it (or whose block writes no `M5` or no `M0`) refuses such jobs.
    In a drag section every written line must pass an allowlist (ADR-0020): printable ASCII
    only (non-ASCII output text is a parse error for every post, and `LINE_ENDING` must be LF,
    CR LF or CR), no `~ ! ?`, no Marlin emergency codes, plain unnested comments (a comment ends
    at the first `)`; no parentheses at all on `;`-comment posts), no M3/M4/S word in the
    template text even in comments, and live G-code made only of upper-case words `X Y Z I J F
    P` (signed numbers), `N T H` (digits), `G0 G1 G2 G3 G4 G17 G20 G21 G40 G43 G49 G80 G90 G91
    G91.1 G94` and `M0 M1 M2 M5 M6 M30`, no `/` (block delete), digit counts capped (`N` up to
    9 digits and at most 9999999, Grbl's limit; `M G T H` 5), every `M0`/`M1` line bare (`[N] M0`
    and at most one comment, no message text, on every post), the `M5` line of both pause blocks
    bare too (`[N] M5` and at most one comment: Grbl drops a line with an error as a whole, so
    `M5 M6` would lose the spindle stop), no tab, and at most 79 characters of G-code outside
    comments per line (Grbl's line buffer; comments do not count) (T-0161). Profile and Pocket refuse diamond-drag tools. In a custom post that offers
    `SPINDLE_OFF_PAUSE`, start the spindle in HEADER, TOOLCHANGE and NEW_SEGMENT on a line of its
    own (`M3[S]`).

Hand-review result: all eight goldens pass points 1–8; points 9 and 10 are enforced by the
interpreter and tested on every post. All eight drag goldens (T-0154) were reviewed against
points 1–8 and 11 (two pauses in a row where a tool change leads into a drag toolpath: the tool
change stop, then the spindle-off pause; and the unlock pause, then the tool change stop, where
the end mill follows the drag toolpaths and before the program end) and are checked by the same test with the drag rule.

## Dialect choices per controller

| Post | Start line | Tool change | Dwell | Program end | Line numbers / endings | Comments | Based on |
|---|---|---|---|---|---|---|---|
| Grbl mm / inch | `G90 G94 G17 G21`/`G20` | `M5`, safe Z, `M0` pause, safe Z, `M3 S` (no tool changer) | `G4 P` **seconds** | `M30` | none / LF | `( )` | Grbl v1.1 wiki: https://github.com/gnea/grbl/wiki |
| Mach3 mm / inch | `G90 G94 G91.1 G40 G49 G80 G17 G21`/`G20` | `M5`, safe Z, `T<n> M6`, safe Z, `M3 S` | `G4 P` **seconds** (Mach3's default); a General Config checkbox, commonly labelled "G04 Dwell in Milliseconds", is reported to switch to ms. Verify that it is off (if it is on, dwells get shorter, never longer) | `M30` | `N10`, `N20`… / CR LF | `( )` | "Using Mach3Mill" user manual, Artsoft / Newfangled Solutions: https://www.machsupport.com |
| Mach4 mm | as Mach3 | as Mach3 | `G4 P` **seconds**, per the official Mach4 Mill manual | `M30` | none / CR LF | `( )` | Mach4 Mill G-code programming manual, Newfangled Solutions: https://www.machsupport.com |
| LinuxCNC mm / inch | `G17 G40 G49 G80 G90 G91.1 G94 G21`/`G20` | `M5`, safe Z, `T<n> M6`, `G43 H<n>`, safe Z, `M3 S` | `G4 P` **seconds** | `M2` | none / LF | `( )` | LinuxCNC G-code reference: https://linuxcnc.org/docs/html/gcode/g-code.html and M-codes: https://linuxcnc.org/docs/html/gcode/m-code.html |
| Marlin mm | `G21`, `G90` (no `G17`/`G94`, see below) | `M5`, safe Z, `; <message>` comment then a bare `M0` pause (**needs an LCD or `EMERGENCY_PARSER`, otherwise ignored**), safe Z, `M3 S`; **no T word** | `G4 P` **milliseconds** (multiplier 1000) | none, see below | none / LF | `;` | Marlin G-code reference: https://marlinfw.org/docs/gcode/ (G0-G1, G2-G3, G4, M0-M1, M3, M5) |

Every post uses `G2`/`G3` with `I`/`J` measured from the arc start, a feed (`F`) only when it
changes, and modal X/Y/Z (written only when they change).

### Controller notes

> **Marlin: the tool-change pause needs an LCD controller or `EMERGENCY_PARSER`.**
> Marlin's tool change relies on `M0`. Without an LCD controller or `EMERGENCY_PARSER`
> (resume with `M108`), Marlin **ignores `M0`**: the tool change is skipped and the next
> toolpath runs with the previous tool. Source: https://marlinfw.org/docs/gcode/M000-M001.html
>
> **Marlin: the spindle-off pause (diamond drag) uses `M0` too.** Without an LCD controller or
> `EMERGENCY_PARSER`, Marlin skips that pause as well. The spindle is still stopped (`M5`) before
> the drag toolpath, but nobody is asked to check it.
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
- **Spindle-off pause.** Every post pauses with `M0` before a diamond-drag toolpath, including
  Mach3, Mach4 and LinuxCNC, whose tool change is `M6`: `M6` can be configured to do nothing,
  while `M0` always stops the program until Cycle Start. If you lock the spindle for drag
  engraving, the unlock pause after the drag toolpaths (also `M0`) is where you unlock it: the
  next spinning toolpath starts the spindle right after the tool change that follows.
- **Spindle spin-up.** None of the posts add a dwell after `M3`. Marlin waits for its configured
  power-up delay. On Grbl, Mach and LinuxCNC, configure spindle-at-speed (or add a dwell to
  `SPINDLE_ON`/`HEADER`) if your spindle needs time to reach speed.

## Adding or changing a post

1. Write it from the controller's public documentation only, and add a row to the table above.
2. Add it to `shippedPosts()` in `core/tests/post/shipped_posts_test.cpp` with its units, tool
   change, program end and dwell words, and to the stem lists of `post_safety_test.cpp`,
   `spindle_off_test.cpp` and the toolpath post tests.
3. Give it `SPINDLE_OFF_PAUSE` and `SPINDLE_UNLOCK_PAUSE` blocks (point 11).
4. Generate its `<stem>_fixture.nc` and `<stem>_drag.nc` goldens. Review them by hand against the
   checklist, then commit them with the post.
