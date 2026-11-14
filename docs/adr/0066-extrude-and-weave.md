# ADR-0066: Extrude and Weave

Status: accepted (T-0289)

## Decisions
- `core/relief/extrude.hpp`: `ExtrudeParams` (rails, source, options) is the stored, re-editable form;
  `extrude(params, grid, cancel, out)` is a pure function of it onto a caller-given grid (ADR-0060),
  with its own `ExtrudeStatus`. It shares the sweep's profile evaluation (monotone cubic) and
  triangle rasteriser (barycentric at cell centres, 1e-9 inclusive, overlaps keep the max) through
  the internal header `src/relief/sweep_detail.hpp` (moved out of sweep.cpp, behaviour unchanged).
- Rails: 1..64 polylines (open or closed, arcs polygonised by the caller); repeated vertices are
  dropped. A position along a rail is an arc length. The surface is built from rungs: across-rail
  lines at least every half cell, at every vertex and at every weave breakpoint; between two rungs
  triangles with linear z.
- Cross-section source: sections (u, z), u strictly increasing, centred on the rail (the middle of the
  u range is on the rail, +u is left of travel). Sections sit at a fraction t of every rail's length
  (strictly increasing); implicit anchors hold the first/last profile at 0 and 1. Sweep between spans
  blends linearly between neighbouring sections; off, each section holds until the next (cells on
  the step edge take the higher side; the last section only at its own key, as in ADR-0065).
- Component source: the field's x runs along the rail, y to the left, centred on the rail; its
  cell-centre rectangle is mapped onto the rail (bilinear samples, transparent stays transparent,
  triangles with a transparent corner are skipped). Component properties are not applied (caller
  bakes). Stretch: one copy over the whole rail. Fill: copies of the natural length, count =
  max(1, round((L - len) / (len (1 - o))) + 1), then all copies are stretched uniformly so the first
  starts and the last ends exactly on the rail (copy length L / (1 + (n-1)(1-o))); `o` in [0,1)
  (percent / 100) is the overlap; flip alternate mirrors every second copy along the rail; overlaps
  keep the max. At most 10000 copies.
- Corners: round (default) emits a fan of rungs rotating the offset direction through the turn
  (<= 0.2 rad per step), i.e. the union of the strips; sharp mitres (offset = bisector / cos(turn/2))
  and bevels (two rungs) once the miter ratio would exceed 8. Inner-side folds are harmless
  because overlaps take the max. Open ends are square.
- Weave: crossings are proper intersections of every pair of rail segments (half-open parameter
  ranges, so a crossing at a vertex counts once; segments sharing a vertex never cross), <= 2000
  segments in total (`TooLarge`). On each rail the crossings are ordered by arc length and strands
  alternate: over at index i iff (i + phase) is even. Phases propagate over the crossings so the two
  strands at a crossing differ (consistent for grids and closed curves; where a diagram cannot be
  alternating the result is still deterministic but a crossing may have two equal strands); the
  lowest rail of each connected group starts over unless `startOver` is false.
  Each crossing makes a window on the rail: half-width (source width / 2) / max(sin angle, 0.1), the
  extent of the other strand's band, with factor zOver% or zUnder% (/100). Windows of close crossings
  are clipped at the midpoint of their centres. The factor is piecewise linear through the window edges
  (so strand heights ramp between crossings), 1 beyond one window half-width past the first/last
  crossing of an open rail, periodic on a closed rail. ScaleShape multiplies z by the factor; AddBase
  adds (factor - 1) x the highest source z (the shape is kept, only lifted/lowered). Strands overlap
  by max, so the over strand hides the under one at the crossing.
- Scale to exact height as ADR-0065 (`FlatResult` if the maximum is not positive).
- Bounds: <= 256 sections, <= 4096 profile points, <= 100000 rail vertices, rungs x samples <= 2^22
  in total, grid <= kMaxCells. Cancellation is polled once per rung and returns an empty field.

## Mutation notes
Run by hand: 33 single-edit mutants of extrude.cpp against the extrude/weave tests, all killed except the
four argued below (the first run also showed which tests were missing: vertex/short-end crossings, seam clipping,
start ramp, arc step, per-span seam joints, all added). Pinned by tests: crossing half-open ranges (vertex crossing), phase propagation (ring and
line: alternation along both rails), start flag, oblique window width, window clipping, end ramps, closed wrap,
mitre/bevel/round corners (incl. the arc vs bevel cell), joints at the closed seam, copy count
rounding (both directions), overlap spacing, flip, anchors, hold step edge, centring of u, size caps
(rungs x samples), cancel.
Survivors, equivalent or defensive:
- Removing the collinear-turn branch in makeRungs (`|phi| < 1e-9`): sharp then gives the mitre of two equal
  normals, round gives one step of angle 0, i.e. the same single rung. Equivalent.
- Removing `xl = clamp(xl, 0, len)` for component samples: s is clamped to the span and the ratio
  (s - a) / (b - a) is at most 1, so the clamp only guards rounding. Defensive.
- Removing the `sin > 1e-9` test in the crossing search: parallel/collinear segments give inf/NaN
  parameters and the NaN-safe half-open test rejects them anyway. Redundant by construction.
- Removing the skip of segment pairs sharing a vertex: the half-open ranges already exclude the shared
  vertex (t = 1 or u = 0); the skip only guards rounding of t/u at the vertex. Defensive.

