# ADR-0100: Controller-side cutter compensation (G41/G42) for Profile toolpaths

Date: 2026-11-16 · Status: proposed (T-0327) · Builds on ADR-0016 (Profile), ADR-0011 (post interpreter),
ADR-0021 (.carv records and move caches)

## Context
T-0327 asks for a Profile option that makes the controller apply the tool radius: the program carries the path of the
*vector* (plus allowance) and the controller offsets it by the radius register. Behaviour reference:
feature-inventory §5 "2D Profile Toolpath" (Cutter Compensation option) and the post blocks CUTTER_COMP_LEFT,
CUTTER_COMP_RIGHT and CUTTER_COMP_OFF (parsed since ADR-0011, never emitted). The card leaves open what the
toolpath stores, how the programmed path is derived, where the blocks go, and which cuts are refused. Readings
the project owner has not decided are marked **(reading)**; each is a one-line change.

## Decision

### 1. What the toolpath stores
- The moves keep the **tool-centre path**, exactly as without the option (ADR-0008): simulation, preview, keep-out
  routing, time estimates and every other consumer are untouched, and a compensated toolpath simulates to the same
  result as the same Profile without the option (tested move for move, flags aside).
- `Move::comp` (`CutterComp::Off | Left | Right`, default Off) flags the moves that run under compensation: the
  side of the programmed path the tool is on, looking along the travel. Only cutting moves carry it.
- **Run:** one compensated cut is one run: from the lead-in's first move (the move right after the plunge, so the
  engaging move is a straight move at depth) to the last move before the lead-out. The first lead-out move
  carries `Off`: it releases the compensation. The vertical "up over a tab" move before the lead-out stays inside the run.
- **Side:** the vector lies on the left of the travel exactly when the tool is on the right of the programmed
  path: `contourOnLeft ? Right : Left`. With the Profile rules (climb keeps the vector on the right of travel,
  ADR-0016 §9) climb is `Left` (G41) and conventional `Right` (G42) for Outside, Inside and open Left/Right alike;
  the reversed finishing pass of a separate last pass flips its side.

### 2. The programmed path is derived, not stored (`cutterCompProgram`, `core/toolpath/cutter_comp.*`)
A pure function of the move list and the tool radius (the toolpath's snapshot diameter / 2). Storing only flags
keeps `Move` small (one byte in existing padding; no second coordinate set for up to 20 M moves) and cannot
disagree with the tool-centre path. For each run:
- A line moves back by the radius along its normal (to the right of travel for `Left`, and the reverse); an arc
  keeps its centre and changes its radius by the radius (inwards-curving with the tool on the inside: +r, else -r).
  An arc whose moved radius is zero (the round join of an Outside corner at allowance 0) is one point of the
  programmed path: **omitted** (`ProgramMove::omit`), the post writes nothing for it.
- Where consecutive moved moves do not meet (an inside corner of the tool-centre path: the tool-centre offset
  lines intersect, the programmed ones are the vector's corner) they are joined at the **intersection** of the two
  moved moves (line-line, line-arc, arc-arc), the one nearest the midpoint of the two ends. Joins within 1e-6 mm
  are taken as meeting.
- **Engage (reading):** the compensation vector at the end of the engaging move is perpendicular to the *next*
  move (start-up behaviour of the common controller families), so the engaging move runs from the tool position
  (where the programmed and tool-centre positions coincide) to the next move's moved start. If the lead is split
  into several moves (a tab floor under it) the first piece engages perpendicular to the collinear next piece and the
  corner with the cut is an ordinary join. **Release (reading):** the compensation vector at the start of the
  releasing move is perpendicular to the *previous* move, so the last compensated move ends at its own moved end and the
  releasing move is written at its tool-centre coordinates (no compensation any more). Controllers that offset
  differently at start-up/release need a post variant; nothing in the toolpath changes.
- Errors, never guesses (message names the 1-based move): a run not engaged or released by a straight move in the
  plane, a rapid or dwell inside a run, a side change without a release, an arc that would need a negative radius
  (smaller than the tool), moved moves that do not meet, an intersection farther than ten radii from the tool-centre
  corner (corner turning by more than ~168 degrees), and a non-finite or non-positive radius. On error `moves` is
  empty (nothing partial).

### 3. Post interpreter (minimal change)
A toolpath with any compensated move is run through `cutterCompProgram` once; the loop then writes the programmed
move instead of the tool-centre move, skips omitted moves, and before a move whose compensation differs from the
state writes the block CUTTER_COMP_LEFT / RIGHT / OFF as its own stationary block (so the block lands on its own line
before the engaging move and before the releasing move). A post missing a block it needs (LEFT only when a left
run exists, likewise RIGHT; OFF whenever any run exists) refuses the toolpath with the block name. `[CUTTER_COMP]`
(already a text tag) expands to `LEFT`, `RIGHT` or `OFF` while a block expands **(reading: the tag is the
compensation state; a post that wants a D/H register number writes it itself, e.g. `[T]`)**. Uncompensated toolpaths
are written exactly as before (goldens unchanged). The tool radius register is the controller's; the toolpath
snapshot's diameter / 2 is the radius the program assumes.

### 4. The Profile option and what is refused
`ProfileParams::cutterComp` (bool, default off). With it on, `profileToolpath` returns the same toolpath as without
it plus the flags, or **refuses** (empty toolpath, one `CutterCompRefused` diagnostic whose message ends "nothing is
cut"; the first failing vector or option is reported, never a partial toolpath):
1. Option checks (no vector needed): side On (a compensated path has a side); any tool but a plain end mill (a V-bit
   or ball nose has a depth-dependent radius); leads other than Straight with a lead-out (the engage and release
   moves are straight); straight lead length below the tool radius; zig-zag ramp (reverses direction, flipping the
   side); vectors with curves (Beziers).
2. Input arcs the tool fills: an arc concave to the tool whose radius is not larger than radius + allowance (+ last-pass
   allowance when separate) - the exact offset drops it, the controller would have no path along it ("arcs smaller than
   the tool radius").
3. Neighbours: a `MergedWithNeighbour` diagnostic (a merged group is one polygonal cut: its round joins are chords,
   not the offset of an exact programmed path, so the programmed path would be noisy at the chordal tolerance). Cuts
   merely trimmed by another vector (`TrimmedNearVector`) are exact offsets cut into pieces: each piece is its own run
   with its own lead-in and lead-out (rule 4 judges them), so they are not refused as such. This covers "tabs
   interaction" at the vector level; tabs themselves are only z changes along the cut and stay supported.
4. Per cut, from its emitted moves: no lead-in or lead-out fits (a dropped lead leaves nothing to engage or release
   on), a lead shortened below the tool radius to fit, an open vector in several passes (it reverses at its end), and any
   derivation error of section 2 (inside corners too sharp, arcs the moved radius of which is negative).
Not refused: Inside/Outside/open Left/Right, climb/conventional, a separate last pass (with or without reverse),
tabs (flat and 3D), smooth and spiral ramps, overcut, any start point rule, positive allowance. A negative allowance
is in practice refused by rule 4 (its lead-in has no room; existing behaviour) and, if a lead were found, by the
negative moved radius at convex corners.

### 5. Persistence
- `.carv`: Profile record key `cutter_comp` written only when true (older files and byte-for-byte round trips of
  uncompensated records are unchanged); **format minor 1.15** (`kCarvFormatMinorCutterComp`, the newest; 1.13 is
  the laser's, ADR-0099, and 1.14 the rotary gaps, ADR-0098) is written only when a Profile record has the option
  or some move is compensated. `saveCarv` picks the highest minor any feature needs: cutter compensation 15 >
  rotary gaps 14 > laser 13 > appearance 12 > ... An older build refuses such a file at the format version (newer
  minor), as it does for every minor it does not know.
- Move cache, coexisting with the laser's beam power (ADR-0099): **version 1** unchanged (no power, no
  compensation); **version 2** = version 1 plus a flag byte (and power) per move, exactly as ADR-0099 wrote it, for a
  toolpath with beam power and no compensation: every file writable before this task is byte-identical;
  **version 3** = the version 2 per-move layout followed by a trailer: u64 run count (at least 1), then per run u64
  first move, u64 length (>= 1), u8 side (1 left, 2 right). Runs are in order, disjoint, and maximal (a run directly
  after a same-side run is refused), so one move list has one encoding and decode-then-encode is byte-identical.
  The writer emits the lowest version that holds the moves: no power and no compensation 1, power only 2, any
  compensated move 3. A version 3 cache holds the beam power of its moves in the per-move bytes (all zero flags for
  a cutter-compensated spindle toolpath, filled in for a toolpath that is both laser and compensated, which
  the card set does not produce but the codec does not forbid), so both features coexist without a combinatorial
  set of versions. Only version 3 carries the trailer; a trailer after a version 2 cache is "data follows the last
  move". The loader's size sanity check allows 58 bytes per move (ADR-0099) plus the trailer (17 bytes per move + 8).
  Flags on Rapid/Dwell moves are not rejected by the codec (the interpreter and `cutterCompProgram` refuse them).
- The post writes the beam blocks (`jetBefore`) after the compensation switch of the same move, so a post with both
  JET_TOOL_* and CUTTER_COMP_* blocks switches each toolpath with its own; the compensation is released before the
  toolpath ends and the beam before the next toolpath.
- The calculation form gets a "Cutter compensation (G41/G42)" check box; a refusal shows as a refusal message.
  carvetto-cli params.json is not extended (no card asks for it).

## Consequences
- A compensated toolpath is only as good as the controller assumption in section 2; a post/controller that handles
  start-up differently is a post change, not a toolpath change.
- Outside corners at allowance 0 write no arc: the programmed path is the vector's own corner and the controller
  rounds it with the tool radius (what the simulation shows). With a positive allowance the corner is an arc of
  the allowance radius, written as such.
- Remaining limits: no circular lead (controllers differ on arc engagement), no V-bit/ball compensation, no On, no
  curves, open vectors in one pass only; each refused with a message rather than approximated.

## Mutant notes (T-0327)
Operator and boundary mutants of the new logic were applied by hand (script kept out of the tree) and the
`[cuttercomp]` tests run: all killed except the following, which are equivalent or tolerance-boundary only.
- `profile.cpp` dropping the `TrimmedNearVector` refusal was a deliberate decision, not a mutant: the check was redundant in
  every scene tried (the pieces fail rule 4 or succeed with an exact programmed path) and is argued unnecessary in section 4.
- `cutter_comp.cpp` `movedRadius < -kNoRadius` -> `<=`, `point = movedRadius <= kNoRadius` -> `<`, `len > kPlane` -> `>=`:
  they differ only for a value exactly on the 1e-9 boundary; no input of the tests (or any produced by Profile, whose
  moved radii are 0, an exact multiple of the allowance, or clearly negative) lands there.
- `disc < -1e-9` -> `< 1e-9` in the line-circle intersection: differs only for a double (tangent) intersection within
  1e-9; a tangent mismatch above the 1e-6 meeting tolerance is not produced by Profile (tangent joins are exact).
- Dropping the `a.point || b.point -> None` guard in `meet`: a point arc's moved start is its centre and the neighbour's
  moved end can lie off it by more than the meeting tolerance only when the neighbour is not tangent to the arc;
  then the neighbour's moved line misses the centre, so the generic code also finds no intersection (equivalent).
- The tolerance constants themselves (`kMeet` 1e-6, `kPlane` 1e-9, 10 radii): tests have joins that match to 1e-12 and
  mismatches of at least 0.1 mm, and the 10-radii limit has tests on both sides (4.9 radii accepted, 11.4 refused).
