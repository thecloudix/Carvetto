# ADR-0111: Drill Bank spindle sets: simulation of every spindle, the post's drill-bank blocks, format 1.18

Date: 2026-11-16 · Status: proposed (T-0335) · Builds on ADR-0106 (Drill Bank toolpath), ADR-0011 (post interpreter),
ADR-0021 (.carv records and move caches), ADR-0032 (simulation), ADR-0100 (the cutter-compensation flag on moves,
the pattern followed here)

## Context
ADR-0106 left three things for this task: the toolpath of a drill bank is the path of the head's reference spindle only,
so preview and simulation show one hole per stroke; the post-processor parses the five drill-bank blocks
(`SETUP_DRILL_BANK`, `DRILL_BANK_PATTERN`, `DRILL_BANK_EXTEND_DRILL`, `DRILL_BANK_RETRACT_DRILL`,
`FINAL_DRILL_BANK_ACTION`) and the tags `[DB] [DBX] [DBY] [DL] [CD]` but never writes them (the interpreter's tag
lookup returned 0 for them); and the app has no Add entry or per-spindle editor. The feature inventory lists the
block and tag names only; it says nothing about what they output or when. Readings the owner has not decided are marked
**(reading)**; each is a local change.

**Audit of what the post parsed versus acted on:** the parser knows the five blocks (`kKnownBlocks`) and the five tags
(`kTags`: `DB` number, `DBX` `DBY` `DL` `CD` lengths); `PostProcessor::knownBlockNames()` lists them; the interpreter
never called `emit` for any of them, and `number("DB".."CD")` returned 0. Nothing else about drill banks existed in
`core/post`. In `core/simulate` nothing knew spindles: a move was swept once with the toolpath's tool.

## Decision

### 1. What the toolpath carries
A stroke's moves (`Rapid`, `Plunge`, `Dwell`, `Retract`: unchanged from ADR-0106 §4) now each carry
`Move::bank`, a 64-bit set of spindle numbers (bit k-1 = spindle k) naming the spindles that drill in the stroke; 0 for
every move that is not part of a stroke. The head comes with the toolpath as `Toolpath::bankHead`: for every spindle
(active or not, number = index + 1) its offset from the reference spindle, its drill diameter and its hole depth, both
resolved (the tool's diameter and the cut depth where the parameters give none): `drillBankHead(params, toolDiameter)`.
**A stroke starts at the `Rapid` that carries its set**; it ends at the next move with another set, with no set, or at
the next `Rapid` with a set. This is the one rule the post, the simulator and the cache all use. The set is the spindles
of the stroke's `DrillBankPlacement::holes`, so a switched-off spindle, or an active one that has no hole in this
stroke, is not in it, and neither is the reference spindle unless it drills.

Why a per-move field and not a separate stroke list: ADR-0100 already put the compensation side on `Move` and in the
move cache; the toolpath's move list is the unit that preview slices, keep-out routing rewrites and the post walks, and a
side table of move ranges would go stale at every one of those. The keep-out router replaces a rapid by detour legs: the
leg that arrives keeps the set (`routeRapids`), the legs before it have none, which the stroke rule above accepts.

The head is **not stored in the file**: for a drill bank record it is a function of the record's parameters and tool
snapshot (`ToolpathRecord::bankHead()`, which `ToolpathRecord::toolpath()` puts on the toolpath), so there is one
source of truth and a record cannot disagree with its own parameters. A record is recalculated whenever its parameters
change, so the cached sets and the parameters change together; a hand-edited file that makes them disagree is refused
(below).

### 2. Simulation and preview: one hole per spindle
For a move with a set, `Simulator` cuts once per spindle of the set instead of once at the head's position: the move
translated by the spindle's offset, with the spindle's drill, in `cutMove`. The drill is the toolpath's own tool when
the spindle's diameter equals the tool's (so a drill's point shape is kept), else a flat drill of that diameter.
**(reading)** A spindle shallower than the stroke is stopped by the head's own stroke limit (ADR-0106 §4): its Z is
clamped to `strokeBottom + (deepest - depth)`, where `strokeBottom` is the lowest Z of the stroke's own moves and
`deepest` the deepest hole of the stroke's spindles. The head's path is still what the diagnostics (rapid through
material, shank, below table) follow; the spindles' holes are not probed, since they follow the head's own Z.
`addToolpath` refuses a toolpath whose moves name spindles the head lacks or has no head (`InvalidMove`), a used spindle
with an invalid drill diameter (`InvalidTool`) or a non-finite or out-of-range offset or depth. A stroke must be queued
whole (one call) for its bottom to be right; the simulation controller extends a slice to the end of the stroke it would
cut. Partial playback (`advanceToTime`) drills every spindle in step. The 2D preview (`buildPreviewGeometry` with the
head) draws the vertical moves of a stroke as one dot per spindle at the spindle's own place (repeated pecks at one
place merge); the head's rapids are still drawn as the head's path.

### 3. The post's drill-bank blocks **(reading)**
The feature inventory names the blocks and tags only, so the meaning below is this project's own; the sample post
`posts/samples/drill_bank_linuxcnc_mm.pp` documents it and uses invented user M codes: a template, not a statement
about any real head. Per drill bank toolpath (any move with a set):

| Block | Written | Frame |
|---|---|---|
| `SETUP_DRILL_BANK` | once per spindle that drills anywhere in the toolpath, before the first move, after the tool change and spindle start | `[DB] [DBX] [DBY] [DL] [CD]` of that spindle |
| `DRILL_BANK_PATTERN` | per stroke, right after its positioning rapid, once per spindle of the stroke (number order) | the same |
| `DRILL_BANK_EXTEND_DRILL` | per stroke, right after the pattern blocks, once per spindle | the same |
| `DRILL_BANK_RETRACT_DRILL` | per stroke, right after its last move (the retract to safe Z), once per spindle | the same |
| `FINAL_DRILL_BANK_ACTION` | once after the toolpath's last move | `[DB]` 0, other tags 0 |

`[DB]` is the spindle number, `[DBX] [DBY]` its place relative to the reference spindle (whose place is the
toolpath's `[X] [Y]`), `[DL]` its hole depth below the start depth, `[CD]` its drill diameter; the lengths convert to
inches like every length; outside these blocks the tags are 0 as before. The moves themselves are written exactly as for
Drilling (ADR-0022): the head's path.

**Needed blocks.** `PATTERN`, `EXTEND` and `RETRACT` are required: without them a controller cannot know which spindles
drill, so a post lacking one refuses a drill bank toolpath (`cannot write G-code: toolpath "x" is a drill bank
toolpath (its strokes drill with several spindles), but the post has no DRILL_BANK_PATTERN block (is it a
drill-bank-capable post?)`), as the laser blocks are required for a laser toolpath. `SETUP` and `FINAL` are optional (a
head may need nothing there): absent, they are not written, without a warning. **(reading)** This also means a drill
bank toolpath calculated by T-0328 (whose records carry no sets) still posts as before, as the head's path, and a
freshly calculated one needs a bank-capable post. The post also refuses a malformed toolpath: a set naming spindles
beyond the head (or no head), a set on a move that is not a rapid, plunge, retract or dwell, and a stroke that does not
start with its rapid (a set that changes mid-stroke).

### 4. `.carv` format 1.18 and move cache version 4
The sets live in the binary move cache (ADR-0021), as the compensation sides do: **version 4** is the version 1 layout
(no per-move flag bytes: a drill bank toolpath has no beam power or compensation, and encoding refuses a move with both)
followed by a trailer: a u64 run count (at least 1) and per run a u64 first move (0-based), a u64 move count (at least 1)
and a u64 spindle set (not 0). Runs are canonical: in order, not overlapping, maximal (a run never follows another of the
same set without a gap), covering only rapid, plunge, retract and dwell moves. Equal input gives equal bytes. The
format minor is **18** (`kCarvFormatMinorBankSpindles`), written only when some move names spindles; a drill bank
record without sets (T-0328's) stays minor 17 with cache version 1, and every other file is byte-identical to before.
An older build refuses a minor-18 file as too new.

Hostile input: the loader reads the trailer streaming and bounds the declared uncompressed size by the move count
(`kMaxMoveBytes + kMovesBankRunBytes` per move, a run being at least one move), so a cache cannot promise more data than
the move budget allows; every defect above has its own message (`drill bank run N: ...`). After the moves are read,
`drillBankMovesProblem(record)` (also run by `saveCarv`) refuses a record whose moves name spindles but which is not a
drill bank, or beyond its head (1..64 spindles from `horizontal_drills + vertical_drills`), or on a move kind other than the
four. With `CacheMoves::Discard` there are no moves to check.

### 5. The app
The Add menu offers **Drill Bank** (after Drill). The form dialog gets a table, one row per spindle of the head (it grows
and shrinks with `horizontalDrills` and `verticalDrills`): drill number, an Active check, a diameter and a depth in mm.
A blank diameter is the tool's, a blank depth the cut depth; entering any diameter fills the blank ones with the tool's
diameter (the form's lists are all or nothing, `setBankSpindles`), any depth the blank ones with 0 (= the cut depth).
A bad number shows its message and keeps the form from being accepted. The self-test (`drillBankSelfTest`) drives the
Add entry, the table (including row count following the head and a bad entry), the calculation, Edit showing the saved
list, the simulation's picture (a hole for each active spindle, none for the switched-off one and none between, the
shallower spindle's hole shaded differently), and a save and reload of the document.

## Consequences
- Every `Move` grows by 8 bytes; `Toolpath` aggregate initialisation lists the new `bankHead` member.
- A record created by T-0328 (no sets) simulates and posts as the head's path, as before, until recalculated.
- The head of a record is derived, so a future change of how parameters map to spindles changes old files' heads: the
  mapping is part of the format (ADR-0106 §1).

## Mutation testing
New logic was mutated by switching each mutant on with an environment variable in a temporary build (removed again;
none is left in the source) and running the `[spindles]` and `[drill_bank]` tests; every mutant was killed. The
survivors of the first round, killed by the tests added after it: SETUP written for the whole head instead of the
spindles that drill (a head with an unused spindle), and a stroke that does not end when the next move has no set and is
not a rapid (a following feed move).

Killed (core): the set of a stroke = every spindle of the head instead of the ones that drill; offsets without the
reference subtracted; spindle depth and diameter defaults (not resolved); the stop of a shallower spindle with the sign
of `deepest - depth` flipped; the stroke bottom as the highest instead of the lowest Z; no clamp of the start Z; the
deepest depth taken over the whole head instead of the stroke's spindles; the head's own position also cut; the tool's
shape not used when the diameters agree; the head-size check of `addToolpath`; strokes not separated at a rapid (the
bottom of a following stroke); the partial-playback end Z not interpolated; the pattern block written after every move
instead of the stroke's rapid; the retract blocks not written before a following rapid, nor before a following move
without a set; SETUP for the whole head; `EXTEND` dropped from the required blocks; the stroke-start check; `FINAL` not
written; `[DL]` written as the diameter; the head-size check of the post; cache run order (`<` against `<=`), the
same-set-adjacent check, runs merging different sets, the move-kind check of a run, the laser/compensation refusal of
the encoder; the off-by-one of the head's spindle count in `drillBankMovesProblem`, and its non-drill-bank check; the
minor ladder (1.18 above 1.17); the save-side and the load-side record checks; the arriving leg of a keep-out detour
keeping its set. Killed (viewmodel): the sign of a spindle's offset in the preview, the merge window of repeated dots,
the fallback to the head's dot, and the controller's extension of a slice to the stroke's end (the test places a stroke
across the 32768-move slice boundary).

Equivalent or argued: none left in the source. The cancel and work-limit behaviour is unchanged from ADR-0106.
