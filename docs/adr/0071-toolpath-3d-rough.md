# ADR-0071: 3D rough toolpath (Z-level and raster over the drop-tool surface)

Status: accepted (T-0296)

## Decisions
- **Grid-exact model.** Roughing works on the nodes of the composed relief `HeightField` (ADR-0060). The
  model is the set of cell-centre heights; the guarantees below are "at the grid", which is what the
  simulator sees when its cell centres coincide with the model nodes (stock origin = model origin - cell/2).
- **Drop-tool surface** `D(p) = max over opaque cells q within the tool radius of (model(q) + zOffset - h(|p-q|))`
  with the tool profile `h` of `simulate::ToolShape`; the tip target is `T(p) = max(D(p) + allowance, stockBottom)`,
  rounded **up** to float. Evaluation uses block maxima (16x16 cells): a block is skipped when
  `blockMax - h(nearest offset in block) <= best`; this is exact (h is non-decreasing in r, so the bound
  holds) and counts cell/block evaluations against the work limit. Transparent cells are ignored; a node with
  nothing in reach has target `stockBottom` (the floor). `Project` mode sets `D = model(p)` and may gouge.
- **Why constant-Z segments are exact at the grid.** Between two 4-neighbour nodes `a`, `b`, every other
  grid node `q` has the nearest point of segment `ab` at `a` or `b` (q's coordinate along the segment is
  outside the open interval). The tool swept at constant `z >= max(T(a), T(b))` therefore leaves `q` at
  `>= model(q) + allowance` because `z + h(dist(q, seg)) >= T(a) + h(|q-a|)` (or `b`). Every cutting
  segment is such a segment, so nothing is cut below `model + allowance` at any grid cell. At each node the
  tip may dip vertically to `T(node)` (exact: a vertical move at the node is a point sweep), unless within the
  tolerance of the segment height. Hence the surface left at a node is `<= T(node) + tolerance` (raster, row
  spacing one cell). Linear Z between nodes is *not* used for cutting: it is not covered by the argument.
- **Tolerance** merges consecutive segment heights that differ by at most `tolerance` at their maximum
  (raises only; never gouges; leaves at most `tolerance` more stock).
- **Z-level:** levels from `min(stockTop, highest stock) - k * stepdown` (passDepth of the cutting data) down to
  the lowest target, which is always the last level. At a level `L` a node is visited when `T <= L` (so a
  constant-Z segment at `L` is within the argument) and the stock is still above `L` (rest tools use the
  simulated surface). Result: the surface left at node `q` is `<= T(q) + stepdown`. Clearing is pocket-style raster
  rows (parallel rows, zig-zag or one-way); runs are linked along a column to the next row when every node
  between is allowed, else retract to safe Z. Offset (contour-parallel) clearing, Profile First/Last and
  depth-first ordering are **not** part of this card (see Deviations); the strategy enum can grow.
- **Stepover** is rounded down to whole cells (>= 1) and the last row is always added; the stock left
  between rows for stepover > 1 cell is not bounded by `T` (scallops), but gouging is never possible.
- **Ramp entry** follows the profile instead of reusing `appendPathRamp`: that routine assumes a constant-Z
  level and a zig-zag back to the start at `zTo`, which gouges where the 3D profile rises along the ramp. Here
  the ramp descends at `rampAngle` from the highest stock over its first nodes, with node heights
  `max(profile, ramp)` (never below the profile, linear only between nodes whose heights are both at least the
  segment height, which the argument covers), then returns to the start at profile height and cuts the whole
  chain, so no ramped stretch is left uncut. No ramp: vertical plunge from safe Z.
- **Rest machining:** tool 0 clears; for tool k > 0 the surface left by tools 0..k-1 is obtained by running the
  existing `Simulator` on the model grid (the same cell centres), and the tool cuts only nodes where
  `stockLeft > T + restTolerance` (the inventory's minimum detail). The paths of earlier tools are the actual
  move lists, so rest follows exactly what the simulator will show.
- **Boundary:** Material (all nodes), Model (opaque nodes, offset by grid distance, bounded to 200 cells), Vectors
  (even-odd polygons, offset by edge distance). Offset > 0 grows, < 0 shrinks.
- **Cut direction:** Climb keeps uncut material on the right of the travel direction: travel +column; rows step
  to -Y for AlongX and +X for AlongY; Conventional steps the other way; `reverseSteps` reverses the row order.
- **Bounds:** `kRough3dDefaultWorkLimit` (4e9 evaluations), move limit 20 M, reach <= 2000 cells; cancel is polled
  per row and per 2048 simulated moves; cancelled runs and refused runs return empty toolpaths.

## Mutation notes (tests kill these)
- Using `min` instead of `max` for segment height, dropping the `-h(r)` term, ignoring the allowance, rounding
  the drop down, or skipping the dip all fail the simulator gouge/excess checks or the brute-force drop test.
- Pruning with the block maximum alone (dropping the profile term) is value-equivalent, so only a work bound
  can see it: the test pins the evaluation count (267835 exact bound vs 270068 mutant) below 269000. Wrong
  pruning (a profile term that is too large) fails the brute-force equality test.
- Verified by running each: segment `min`, drop without `h`, no dip, no allowance, no ramp return, a missing
  floor level and a wrong rest comparison all fail at least one test.
- Removing the ramp's return pass leaves stock above the bound (excess check); ramping below the profile gouges.
- Row order/direction/axis, boundary offset sign and rest tolerance each have a dedicated exact-position test.

## Deviations (reported to the director)
Offset-style clearing, Profile First/Last/None, depth-first ordering and "avoid machined areas" are left for a
follow-up card; Z-level uses raster clearing only.
