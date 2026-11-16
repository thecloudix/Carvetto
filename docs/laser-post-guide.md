# Adapting a post-processor for a laser

How to turn a Carvetto `.pp` post (grammar: ADR-0004, behaviour: ADR-0011) into one that drives a laser, or to
change `posts/grbl_laser_mm.pp` for another controller. Written for Carvetto from scratch; the only source is the
controller documentation you use. Design record: ADR-0099 (beam power on moves, JET blocks) and ADR-0107.

## 1. Declare it

```
LASER_SUPPORT = "YES"
```

A laser toolpath (Laser Cut / Fill, ...) is refused by any post that does not declare this. Without the line a
post is for spindle machines only. The machine record (`Machine::laser`, "laser" in the tool database) says that the
machine carries a laser; `post::laserSetupProblem(machine, post)` checks the machine flag, the declaration and the
blocks, and gives one sentence naming what is missing.

## 2. The power variable

Beam power is a percentage (0 to 100) on each move. The post turns it into the controller's power word with a
variable of tag `JET_POWER`:

```
VAR JET_POWER_WORD = [JET_POWER|A|S|1.0|10]       prefix "S", format 1.0 (integer), multiplier 10
```

The multiplier maps 100 per cent onto the controller's full scale: 10 for S0..S1000, 2.55 for S0..S255, 1 for a
controller that takes per cent. The value is rounded by the format: with `1.0` the power word is an integer, so a
power below half a step writes `S0` (the beam stays dark): choose the format and multiplier so that your smallest
wanted power survives.

## 3. The laser blocks

| Block | When it is written | Typical content |
|---|---|---|
| `JET_TOOL_POWER` | before the beam comes on and whenever the power of a move differs from the one last set | `[JET_POWER]` (the power word alone) |
| `JET_TOOL_ON` | when a move with power follows a move without | `M4` (dynamic power) or `M3` |
| `JET_TOOL_OFF` | at a rapid, a dwell or a beam-off feed after a lit move; at the end of a toolpath that ends lit | `M5` |

`JET_TOOL_ON` and `JET_TOOL_OFF` are required (a post that has `LASER_SUPPORT` but not these cannot run a laser
job); `JET_TOOL_POWER` is optional (put `[JET_POWER]` in `JET_TOOL_ON` if the power word must come on the same line).
Inside any of the three, `[JET_POWER]` is the power of the move that causes the block (the power that was set,
for OFF). The beam is never on across a toolpath, a tool change or the footer: the interpreter writes the OFF block.

## 4. What is different from a spindle post

- No spindle start for a laser toolpath: `SPINDLE_ON` is skipped and spindle speed 0 is accepted. A `[S]` tag in
  `HEADER`, `TOOLCHANGE` or `NEW_SEGMENT` is still written (as `S0`), so leave spindle lines out of a laser-only post.
  Jobs that mix an end mill and a laser keep `SPINDLE_ON` (`M3[S]`) for the end mill toolpaths.
- Z: the moves carry the focus height of each pass. For a fixed-focus laser delete `[Z]` from the move blocks.
- A tool change is a laser module or lens swap: keep the pause (`M0`), with the beam off first (`M5`).
- Dynamic-power modes (`M4` on Grbl) switch the beam off during rapids by themselves; the post still writes
  `JET_TOOL_OFF`, so the same post is safe on a controller without that mode.

## 5. Check list for your post

1. `LASER_SUPPORT = "YES"`, the three JET blocks, a `JET_POWER` variable with the right multiplier.
2. Run a job with a Laser Cut, a Laser Fill (power changes, beam-off overscan feeds) and a tool change; read the
   output: the power word comes before every `M4`, every rapid has the beam off, the file ends with `M5`.
3. Cut a test pattern on scrap at a low power first.
4. Add a golden file for it like `core/tests/golden/post/grbl_laser_mm_laser.nc` and a test like
   `core/tests/post/laser_post_test.cpp`.
