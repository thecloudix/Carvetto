# ADR-0072: 3D finish toolpath (raster and offset over the drop-tool surface, rest machining)

Status: accepted (T-0297)

## Decisions
- **Exact per-piece drop heights.** Like roughing (ADR-0071) finishing is exact at the nodes of the
  composed relief `HeightField`. Finishing paths are polylines in plan that need not lie on grid lines, so
  the roughing argument (4-neighbour nodes) does not apply. Instead every straight piece `ab` is cut at
  the constant height `Z(ab) = max over opaque cells q within reach of (model(q) + zOffset - h(dist(q, ab)))
  + allowance` (floor-clamped, rounded up to float): the tool swept along `ab` at constant Z leaves cell
  `q` at `Z + h(dist(q, ab))`, because `h` is non-decreasing (the sweep of a body of revolution), so no cell
  is cut below `model + allowance` on any piece, whatever the point spacing. (No vertex dips are needed: see the mutation notes.) Evaluation uses the same block-maximum pruning as the drop surface; the block
  bound uses the distance between the piece's bounding box and the block, which is a lower bound.
- **Tolerance.** `tolerance` (> 0) bounds the stock a piece may leave above the drop surface: a piece with
  `Z(ab) - min(Z(a), Z(b), Z(mid)) > tolerance` is split at its middle (down to 1/32 of the cell size, 5
  levels). The check is at the ends and the middle only, so between those points a little more can remain
  (tests use a 1.6x allowance); gouging is never possible. 
- **Raster:** lines at `rasterAngle`, `stepover` apart in plan (not rounded to cells), clipped to the node
  rectangle (so a line grazing a corner still reaches it), sampled at most one cell apart. Climb steps the
  lines to the right of the travel direction (as ADR-0071), `reverseSteps` flips it; zig-zag or one-way.
- **Offset ("constant scallop"):** lines are iso-lines of the distance from the region edge measured on the
  tool-tip (drop) surface: Dijkstra over 16 neighbours with 3D step lengths, so the spacing along the tip
  surface is constant and the scallop is the same everywhere (checked on a sphere). The grid metric overestimates
  Euclidean length by up to ~3 %, so passes are, if anything, closer than the stepover. Levels `k * stepover`
  from the region edge (level 0 runs along the edge) inwards, plus a last level just below the maximum
  distance when the innermost gap exceeds half a stepover; `reverseSteps` cuts inside out. Iso-lines are
  marching-squares loops on the field padded with -1 (all closed), oriented with the higher distance (inside)
  on the right for Climb. Loops of a level are taken nearest-first and entered at the nearest vertex.
  This is an approximation of a true geodesic-offset scheme (no fast-marching): chord and linear
  interpolation errors add to the stated tolerance; the sphere test bound has 12 % slack on the scallop.
- **Region and linking.** The region is the mask of ADR-0071 (Material / Model / Vectors with offsets; the
  Model offset uses an exact Euclidean distance transform instead of a window scan). Consecutive runs are
  linked by a straight piece when the gap is at most two stepovers and every sample on it is inside the
  region; a link is just another piece, so it is cut at its exact height and cannot gouge. Tool centres stay in
  the node rectangle, so on a slope falling outwards the cells within a tool radius of the grid edge are not
  cut to the model; the tests compare the interior only. Lines outside the region are not cut.
- **Rest machining:** the surface left by the `priors` (given toolpaths, typically the roughing) and by the
  earlier tools of the list is obtained by simulation on the model grid (as ADR-0071). A node is eligible when
  `stockLeft > T + restTolerance`, with `T` the tool's drop surface at the node (+ allowance); the cut region is
  the eligible nodes dilated by one stepover (exact distance transform), so the cells between neighbouring
  passes are reached and nothing farther is cut. With priors even the first tool is a rest tool.
- **Stepover** wider than the tool diameter is limited to it (`StepoverLimited`).
- **Work/cancel:** the same work counters and move limit as roughing; cancel is polled per line, loop and
  piece group; refused or cancelled runs return empty toolpaths with the diagnostic.
- `rough3d.cpp` is not touched: the mask and the rest simulation are repeated here in a small form, to keep this
  card independent of the concurrent T-0304 work on that file. A shared helper can be extracted later.

## Mutation notes (tests kill these)
Each mutant was applied to `finish3d.cpp`, built, and run against `[finish3d]`; all of these fail at least one test:
piece height from the endpoints instead of the piece distance; no profile term `h`; allowance ignored;
floor clamp removed (BelowTable / min-Z test); no piece refinement; refinement checking ends only; rest
tolerance ignored; no rest dilation; rest dilation of zero; priors ignored; Climb/Conventional flipped for raster
and for offset loops; no last offset level; level 0 skipped; `reverseSteps` ignored for offset; boundary offset
sign flipped; Model shrink ignoring transparency; links without the activity check; no zig-zag; stepover not
limited; grid origin dropped from raster lines; last raster line dropped; plunge at the piece height; block pruning
with a wrong bound; closed loops not rotated at inactive samples; surface distance ignoring the height change.
Argued instead of killed:
- *Vertex dips* were removed from the design: refinement bounds `Z(piece) - Z(point)` by the tolerance and the
  next piece's vertical move to its own height already reaches the vertex height, so a dip is unobservable.
- *Rounding the height up to float* (vs. down): the difference is one float ulp (about 1e-6 mm at 10 mm), below the
  1e-4 gouge tolerance of the simulator checks, which is float itself; the rule keeps the guarantee exact.
- *The link length limit (two stepovers)*: a link is an ordinary piece cut at its exact height, so it cannot gouge at
  any length; the limit only decides rapid-versus-feed. The test pins the number of cuts of a round region (5 with
  linking, 24 without), which kills "never link"; chords between line ends on a smooth boundary run within
  sampling distance of the boundary, so "always link" gives the same result there.

## Deviations / limits
- No Project tip mode (finishing always drops the tool). No spiral/radial/other strategies (not in the docs).
- The scallop bound is stated for a ball and measured against a sphere; for flat and V tools the stock left
  between passes follows their profiles and is not bounded by `ballScallopHeight`.
