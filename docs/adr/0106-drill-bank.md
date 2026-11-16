# ADR-0106: Drill Bank toolpath (spindle pattern matching, head moves, record 1.17)

Date: 2026-11-16 · Status: proposed (T-0328) · Builds on ADR-0022 (Drilling), ADR-0021 (.carv records and move
caches), ADR-0016 §4 (cut order), ADR-0099/0100 (record patterns of the newest operations)

## Context
T-0328 adds the Drill Bank toolpath (feature-inventory §5 "Drill Bank Toolpath", Advanced Machining Module): a
multi-spindle drill head drills many holes in one stroke. The inventory lists the bank layout (horizontal drills,
vertical drills, central drill, pitch), per drill a number, diameter and reference drill, the pattern matching
(pitch tolerance, diameter tolerance) and peck drilling. The card adds which spindles are active, rows/columns
offsets, per-hole depth, reuse of Drilling's peck and dwell, and a refusal when holes cannot be matched. It does
not say what the output of such a toolpath is, how holes are matched, or what "matched" means. Readings the
project owner has not decided are marked **(reading)**; each is a local change.

## Decision

### 1. The bank (head) model
The head has a **row** of H spindles along X and a **column** of V spindles along Y, `pitch` apart, 0..32 each
(at most 64 spindles; H + V >= 1). Positions are measured from the crossing of the head's central axes: the row
lies at `Y = rowOffset` and is centred on X = 0, the column lies at `X = columnOffset` and is centred on Y = 0
(the "rows/columns offsets"). Spindles are numbered 1..H along the row from left to right, then H+1..H+V up the
column. **(reading)** The inventory's "central drill (X, Y)" is this crossing: only differences between spindle
positions matter to the program, so its coordinates do not appear as a parameter (they cancel against the
reference drill). The head does not rotate: it matches patterns as built (a column of holes needs vertical drills).

Per spindle: **active** (`inactiveSpindles` lists the switched-off numbers), **diameter** (`spindleDiameters`,
empty = the tool's diameter for every spindle) and **depth** (`spindleDepths`, empty or 0 = `cutDepth`). The three
lists are by spindle number; a non-empty diameter or depth list must have one entry per spindle. Spindles must not
sit at the same place (a row and a column crossing with zero offsets and odd counts is refused with that message:
a head has distinct spindles; give the column an offset or an even count).

`referenceDrill` is the spindle the machine's X/Y address names (default 1). It may be switched off (it is then
only the datum). It decides what the toolpath's moves are, see §4.

### 2. Holes and matching
The holes are those of Drilling (`drillHoles`, extracted from `drillToolpath` without behaviour change; ADR-0022
§1): the area centroid of every usable closed vector, with the same diagnostics (open, empty, discontinuous,
no-area, centre-outside, duplicate). The one difference: `HoleWiderThanVector` is not reported, the spindles'
drills differ.

A **stroke** puts the reference spindle at a position `R`; every active spindle `k` is then at `R + rel(k)`
(`rel` = its offset minus the reference's). It drills the not-yet-drilled hole whose centre is within
`pitchTolerance` of that position (nearest; ties: lower vector index) and, with `matchDiameters`, whose
diameter differs from the spindle's by at most `diameterTolerance`. A hole's diameter is that of the circle of
equal area (`drillBankHoleDiameter`: exact for circles, the natural measure for any outline; **(reading)** a
vector marking a hole by a small circle is then a hole of that size, `matchDiameters` off compares nothing).
`pitchTolerance` must be less than half the distance between the two closest spindles, so a hole is within the
tolerance of at most one spindle of a stroke; the check is a parameter problem, not a silent reduction.
A hole is drilled where its spindle is, up to `pitchTolerance` off its centre: one `BankHoleMoved` diagnostic
reports the count and the worst deviation (holes within the merge distance `tolerance.distance` are not counted).

`minHoles` (default 2, 1..active spindles) is the fewest holes a stroke may drill: a drill bank exists for
patterns, so a hole that fits no pattern of two or more is **unmatched**. With `minHoles` 1 any hole with an
active spindle of the right diameter is drilled (a one-spindle bank is Drilling).

**Choosing strokes (greedy set cover).** Minimising the strokes is set cover (NP-hard), so: every (hole, active
spindle) pair whose spindle accepts the hole is a candidate stroke (the spindle exactly over that "anchor" hole),
scored by the holes it would drill; the best-scoring one is taken (ties: lower anchor hole, then lower spindle),
its holes are removed, and so on while the best drills at least `minHoles`. Scores only fall as holes are taken,
so a lazy priority queue is exact: the top candidate is re-scored, and taken only if its score did not fall;
otherwise it goes back with the new score. A candidate whose anchor was already drilled is dropped (the same
stroke is still available anchored on one of its remaining holes). Isolated holes are scored once and never
again. This is a heuristic: it does not always find the fewest strokes (it does for rows, grids and every layout
the head's pattern tiles exactly), and it is deterministic.

**Work.** Cost is holes x active spindles scorings, each `active` spatial queries; the estimate (holes x
active^2) is checked against the limit first and the counter `stats.spindleQueries` against it while running
(`ToolpathRunControl::workLimit`, default 30 million; `WorkLimitExceeded`, nothing produced); both counters are in
the result so tests bound the cost without a clock.

### 3. Unmatched holes: refuse or skip
`unmatched` = **Refuse** (default): nothing is cut; the result has an empty toolpath and one `BankHoleUnmatched`
diagnostic per hole (the first 100; vector index, centre, and whether no active spindle has the diameter or no
stroke drills it with enough others) and a summary ending "nothing is cut". **Skip**: the matched holes are drilled
and the same diagnostics (summary ending "they are not drilled") say what was left. `result.unmatched` lists them
all either way. The viewmodel shows a refusal as an error and a skip as warnings.

### 4. The toolpath: the path of the head
**(reading)** The moves are the reference spindle's: per stroke `Rapid` at safe Z over `R`, a `Plunge` per pass,
optional `Dwell`, peck `Retract`s, a `Retract` to safe Z, exactly Drilling's moves (ADR-0022 §3; the loop is the same
and a one-spindle bank's moves equal `drillToolpath`'s, tested). `startDepth`, `safeZ`, `peck`, `peckRetract`,
`retractGap`, `dwellTime` keep their Drilling meaning; passes use the cutting data's pass depth. The other
spindles drill their holes because they are on the head; the toolpath does not carry them, the result's
`placements` (reference position and, per hole, spindle, depth, drilled position and deviation) does. Consequences:
the machine program is right for a real head, but Carvetto's preview and simulation show only the reference
spindle's hole per stroke, and the post-processor's bank blocks (SETUP_DRILL_BANK, DRILL_BANK_PATTERN,
DRILL_BANK_EXTEND/RETRACT_DRILL, FINAL_DRILL_BANK_ACTION, `[DB]` ...) are not written yet. Both are follow-ups
(a bank-aware simulation needs one hole per spindle; the blocks need the per-stroke spindle list on the toolpath,
which is a schema change deliberately not made here).

**Per-hole depth (reading):** the depth is per spindle. A stroke goes as deep as its deepest hole (head Z moves
only once); a shallower spindle is stopped by the head's own stroke limit, which is the head's business (the
EXTEND/RETRACT blocks above). The stroke's passes come from `PassPlan` at that depth, cached by depth.

**Order:** the strokes are ordered with the shared `orderCuts` (selection = the order they were chosen; left to
right; bottom to top; grid; none = nearest neighbour from `startPosition`, the position of the reference
spindle), each stroke an item at `R`. "Minimises moves" is therefore: fewest strokes by the greedy cover, then
the nearest-neighbour tour (the same standard as Drilling and Profile), not a proven optimum.

Tools: as Drilling (`drillToolAllowed`, `drillParamProblem`, `drillToolProblem`, both extracted from
`drill.cpp` unchanged): a drill or an end mill with valid rates. The tool snapshot is the tool at the deepest
depth.

### 5. Record, params and format minor 17
`DrillBankParams` **extends** `DrillParams` (so the Drilling fields, the order and tolerance helpers, the
viewmodel tables and `safeZ()` work unchanged; a `DrillParams` visitor does not see it, it is a separate
variant alternative, `ToolpathOperation::DrillBank`, job.json `"operation": "drill_bank"`). Params (all optional on load,
unknown keys dropped, numbers finite): the Drilling keys plus `horizontal_drills`, `vertical_drills`, `pitch`,
`row_offset`, `column_offset`, `reference_drill`, `inactive_spindles`, `spindle_diameters`, `spindle_depths`
(arrays of at most 64 numbers or integers), `match_diameters`, `pitch_tolerance`, `diameter_tolerance`,
`min_holes`, `unmatched` (`"refuse"` | `"skip"`). Counts are range-checked on load (integers 0..1000: the
refusal of a head that does not exist is the parameter check, so a file that names one still loads and its
toolpath reports the problem when recalculated). `nonFiniteParam` names `spindle_diameters[i]` etc. The format
minor is **17** (`kCarvFormatMinorDrillBank`), written only when a document has a drill bank record; the file
holds job.json only (moves in the usual cache, ADR-0021). An older build refuses with "unsupported toolpath
operation 'drill_bank'". The ladder in `saveCarv` puts 17 above 16 (laser images, T-0324), 15 (cutter compensation), 14 (rotary gaps) and 13 (laser). Dependent task: T-0329
(cabinet parts import) can create drill bank toolpaths through this record.

### 6. Viewmodel form
`ToolpathForm` for `DrillBank`: the Drilling fields plus `horizontalDrills`, `verticalDrills`, `pitch`,
`rowOffset`, `columnOffset`, `referenceDrill`, `pitchTolerance`, `diameterTolerance` (shown with
`matchDiameters`), `minHoles`, choices `matchDiameters`, `unmatched`; the per-spindle lists through
`setBankSpindles(inactive, diameters, depths)` (mm, all or nothing). `problems()` includes
`drillBankParamProblem`, so list lengths that do not fit the head are shown before calculating. Tools: drills
and end mills. The dialog gets its label; the Add menu entry and the per-spindle list editor are app work for later.

## Mutation testing
New logic was mutated by hand (each mutant built and run against the `[drill_bank]` tests); all killed unless
argued here. Killed: tolerance `<=`->`<`; `minHoles` comparison; dropping the anchor-drilled skip; the deepest
depth; reference sign; closest-spindle bound `>`->`>=`; deviation report `>`->`>=`; report cap; stroke depth max->min;
level cache; work estimate and query limit boundaries; both tie-break directions; nearest hole; diameter
comparison direction; dwell in the move estimate; row/column offset signs; drilled-hole skip; per-spindle depth
default; tool diameter default; initial candidate pruning (`gain >= 1`, caught by the evaluation counter).
Survivors, argued:
- Diameter comparison `<=`->`<` (`diameterTolerance` boundary): a hole's diameter is `2 sqrt(area/pi)` from a
  contour area, so exact equality with a user-typed diameter at tolerance 0 depends on the last bit of a
  floating-point computation (and of the platform's trigonometry): not a boundary a test can pin, and users
  cannot rely on it. The 0.15-vs-0.1/0.2 tests pin the behaviour either side of the tolerance.
- The cancel-flag polls inside the two long loops: with the flag never set the output is identical, and a
  deterministic mid-run cancel needs a second thread or a hook the card does not call for; the poll at the start is
  tested, the loops follow the same every-1024-evaluations pattern as the other operations (ADR-0016 §19).
- Loop-internal pruning `if (overLimit) break;` after an evaluation: unreachable without the limit, and the limit
  tests cover the stop (queries <= limit + one evaluation).
