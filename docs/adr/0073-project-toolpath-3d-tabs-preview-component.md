# ADR-0073: project toolpath onto 3D, 3D tabs, component from toolpath preview

Status: accepted (T-0298, first part; the Moulding toolpath is split off, see Deviations)

## Decisions
- **Input convention.** A 2D toolpath's cutting Z is a depth relative to the surface: 0 is the surface (the usual
  "Z zero at the material top"), negative goes into the material. The projected Z of a cutting end point is
  `max(surface(x, y) + z, stockBottom)`. Z above 0 (safe heights) and every Rapid are left untouched, so a retract
  clears the model only if the input safe height does. The floor cap is silent ("depth is capped at the material
  bottom").
- **Surface.** `DropTool`: the drop-tool surface of ADR-0071 (`dropToolSurface`, includes `zOffset`), so a cut at
  depth 0 never gouges the model at the grid. `Project`: the model height under the tool tip (+`zOffset`); a ball
  nose on a slope then gouges by design (tested with the simulator). The surface is sampled bilinearly between
  nodes; a sample is "absent" outside the node rectangle or when a node with weight is transparent/-inf, and the
  surface is then `outsideSurface` (the material top), without allowance. `allowance` (>= 0) is added where the
  model is present.
- **Pieces.** Lines and arcs are cut into straight pieces: at most `maxSegment` (default: the cell size) in XY and,
  for arcs, within the chordal sagitta `tolerance` (the helix Z is linear in the angle). Each piece is then
  bisected (at most 10 levels, not below cell/16) while the surface at its midpoint differs from the piece's linear
  Z by more than `tolerance`. Guarantee: piece end points are exactly on the projected surface; the midpoints of
  accepted pieces are within `tolerance`. The surface between nodes is not otherwise bounded (a bilinear patch
  can deviate between the samples examined); the drop-tool "never gouges" argument of ADR-0071 holds at the grid
  nodes only, not between them, so this operation promises "within tolerance", not "never gouges", away from
  the nodes.
- **Kinds.** A piece keeps the kind of its move (Feed/Plunge/Retract; arcs become Feed) and its feed override.
- **3D tabs.** Tabs are applied after projection to every *run* (consecutive Feed moves of positive XY length).
  Centres come from `placeTabs` on the run's XY polyline (tab length + tool diameter = the raised stretch, as 2D
  tabs). The tab height is the *cut bottom* plus `thickness`, where the bottom is `max(surface + lowest input Z,
  floor)` at that XY: it follows the model. The result is `max(z, ceiling)`; without `threeD` the ceiling steps up
  and down with vertical Feed moves at the stretch ends, with `threeD` it ramps at 45 degrees (height `thickness`
  over distance `thickness` on each side). Breakpoints are inserted exactly at the stretch ends and ramp feet.
  Passes already above the ceiling are unchanged. A toolpath that does not cut below the surface, or tabs that
  could not be placed (dropped, not avoided), give `TabsNotPlaced`.
- **Component from toolpath preview.** `simulate::componentFromPreview` copies the simulator heightfield
  (grid = simulator cells; node (i, j) at the centre of cell (i, j)) into a `relief::Component`, height
  `Z - zOffset` rounded to float (exact for zOffset = 0). `All` keeps uncut stock at the stock top; `CutOnly`
  makes uncut cells transparent. The Shift/Ctrl variants of the menu entry are not described in the reference
  and are left to the UI card (`cells`/`mode` are the knobs).
- **Bounds.** Work limit 2e8 surface evaluations (and the drop surface's own limit), 20 M output moves, cancel
  polled per move and inside the drop surface; refused runs return no moves.
- No new `ToolpathIssue` values (the app switches over them): `InvalidParameters`, `InvalidTool`,
  `WorkLimitExceeded`, `Cancelled`, `TabsNotPlaced` are reused.

## Mutation notes
Sixteen mutants were run by hand, one build each; every one fails at least one `project3d*` test: drop mode
reading the model instead of the drop surface, no 45-degree ramp, tab bottom without the surface, no floor cap,
no refinement (`kMaxDepth` 0), no allowance, both vertical-step sides (before/after an edge), safe heights not
kept, `zOffset` ignored in project mode, raised stretch without the tool diameter, no arc chord limit, tabs
without a cut depth, preview `CutOnly` ignoring `isCut`, preview origin at the cell corner, preview offset
sign. The step-side mutant at the stretch start first survived and led to the checks at 6.99/7.01/12.99/13.01.

## Deviations (reported to the director)
The **Moulding toolpath** (profile a moulding cross-section along drive rails with the tool shape: finishing
stepover, vary stepover, skip flat regions, sharp corners, roughing with a larger clearance tool, machine flat
regions) is not part of this change; it needs its own card, and several of its options (toolpath position,
Z from the cross-section) are product decisions.
