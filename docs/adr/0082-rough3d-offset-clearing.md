# ADR-0082: 3D rough offset clearing, profile pass, depth-first order (addendum to ADR-0071)

Status: accepted (T-0304)

## Decisions
- **Units.** A Z-level run is a sequence of *units* `(level, node set)`. The node set of level `i` is
  `S_i = {elig, T <= Lf_i, stock > L_i}` exactly as in ADR-0071 (same predicates, evaluated once per node as an
  interval `startLv..endLv` by binary search). Every node of a unit is cut at the constant tip height `Lf_i`, so
  the exact-at-grid argument of ADR-0071 applies unchanged to any walk over 4-neighbour nodes that are allowed at
  `Lf_i` (T <= Lf_i): nothing is cut below `T`, and each unit node is visited, so the surface left is `<= T + stepdown`.
  The defaults (raster clearing, Profile None, level by level, no avoid) take the old code path with one unit per
  level and the whole grid as the row range, so their output is unchanged.
- **Offset (contour-parallel) clearing.** Rings are the nodes of the unit at 8-neighbour (Chebyshev) distance `k`
  from outside the unit (outside = not in the unit or beyond the grid). Chebyshev distance is chosen because the
  ring around a rectangle or an island is then a 4-connected loop (with 4-neighbour distance the island ring
  breaks at the corners). Rings are cut outermost first; only rings `k` with `k % kStep == 0` and the innermost
  ring are cut, `kStep` being the stepover in whole cells (so with a one-cell stepover every node is visited, as in
  raster clearing; larger stepovers skip rings exactly like raster skips rows).
- **Ring walk.** The start is the lowest node index of the ring. At each node a wall vector `o` (sum of the 8-neighbour
  directions that point outside the unit or to a smaller distance) gives the heading `h = rot90ccw(o)`: the region
  interior on the left, the wall on the right (climb); Conventional negates it. The next node is the unvisited
  4-neighbour on the ring with the largest `dot(step, h)` (ties: E, N, W, S). When none is left, a breadth-first link
  through allowed nodes (at most `2*kStep + 2` steps) to the nearest pending node of the ring continues the chain;
  otherwise the chain ends (retract). Rings follow each other without retract when the link fits. Everything is
  deterministic (fixed scan and neighbour orders). Entering a ring by a link may reach a piece against the
  preferred sense when it has nothing else unvisited; that never affects safety.
- **Profile.** `First`/`Last` cut ring 0 (the boundary of the level's region: model walls, islands, the vector/boundary
  limit and the grid edge) before or after the clearing. With raster clearing it is an extra pass over ring 0 (the
  rows still visit those nodes; the duplication is accepted so the rows keep the old shape). With offset clearing
  ring 0 is already the first ring, so `None` and `First` coincide and `Last` rotates it to the end. `None` has
  no extra pass.
- **Depth-first order.** Components (4-connected) of the nodes that *start* at a level are roots; a unit is cut,
  then the components of its nodes active at the next level are cut before its next sibling (explicit stack,
  pre-order, components ordered by lowest node index). `nextLv[n]` records the next level a node still has to be cut at, so
  each node is cut at each of its levels exactly once even when the intervals are not nested (rest machining,
  nodes that start deeper become roots at their level). The row range of a unit is its bounding box.
  Output set is identical to level by level (test: identical simulator heightfields); only the order differs.
- **Avoid machined areas.** Links (the column links of zig-zag rows and the ring links) may only cross nodes of the
  unit being cut at this level, never nodes that were already machined (rest machining) or belong to other
  regions; the chain retracts instead. Without it a link may cross any node with `T <= Lf`.
  The cut node set is not affected.
- **Transparent model cells** (confirmed owner product reading, ADR-0071): they are ignored by the drop and, under
  the Material boundary, cut down to the stock floor where nothing opaque is within the tool's reach (test:
  left half opaque, right half NaN).
- Cancel is polled per unit and per ring; work (unit sizes, ring distance passes, link searches) counts toward the
  work limit; the move limit is shared with ADR-0071.

## Mutation notes (each applied, built and run against `[rough3d]`; all fail at least one test)
- Heading sign flipped (climb/conventional), wall vector inverted: the exact-vertex ring-order tests and the island test.
- Profile Last without the rotation; profile First or Last dropped on raster clearing: the chain-start tests.
- Avoid-machined ignored: the two-gap rest scenario (1 chain vs 2); link depth 1: the same scenario (needs the 4-step link).
- Depth-first: component push order reversed, `nextLv` ignored, sub-components of a unit never pushed: the
  pocket-order, nested-pocket and simulator-equality tests.
- Level interval: `T <= Lf` made strict at the end level, `start <= end` made `<`: gouge/excess matrix and exact tests.
- Ring selection: innermost ring dropped, every ring cut (stepover ignored): the 9x7 two-cell-stepover test.
- 4-neighbour instead of Chebyshev ring distance: the island test (ring breaks at the corners).
