# ADR-0065: Two Rail Sweep

Status: accepted (T-0288)

## Decisions
- `core/relief/sweep.hpp`: `SweepParams` (two rail polylines, closed flags, sections, options) is the
  stored, re-editable form; `sweep(params, grid, cancel, out)` is a pure function of it onto a
  caller-given grid (ADR-0060). Own `SweepStatus`; `relief::Status` is not extended.
- Rails are polylines (callers polygonise arcs). Both open or both closed. Positions along a rail
  are arc-length fractions, so rails with different node counts work (proportional positioning).
  "Add to all rail nodes" requires equal node counts (inventory) and returns `NodeCountMismatch`
  otherwise; each section lands at its own node's fraction on each rail (t1, t2).
- A section is a profile (u, z), u strictly increasing (a function of u, so no vertical walls);
  u is normalised so the first point is on rail 1 and the last on rail 2. Smooth = monotone cubic
  (Fritsch-Carlson) through the points: interpolates, never overshoots, equals linear for 2 points.
- Scale with width multiplies z by (local rail gap / section width); off, z is as drawn.
- Sections are keys (f = (t1+t2)/2); implicit anchors at 0 and 1 hold the first/last profile.
  Sweep between spans: profiles blend linearly between neighbouring keys; off, each section holds
  until the next (a step; cells on the step edge take the higher side; the last section therefore holds to the rail end).
- Rungs join rail points at matching fractions, inserted at least every half cell and at every rail
  vertex fraction; between rungs the surface is triangles with linear z, rasterised at cell centres
  (barycentric, 1e-9 inclusive tolerance), overlaps keep the max. Cells not between the rails
  are transparent. Fill centre (closed rails): cells inside the smaller-area rail and otherwise
  transparent get the mean, over rungs, of the profile end height at that rail.
- Swap rails swaps rails and section (t1, t2) but not profiles: the surface is mirrored. Reverse
  rail keeps a closed rail's start vertex, maps t to 1 - t (closed t=0 stays) and restores ascending
  section order when both rails were reversed; reversing one rail leaves the sections non-monotone
  and `sweep` then reports `BadSection` (crossing rails are not supported).
- Scale to exact height: multiplies so the maximum equals `exactHeight`; `FlatResult` if the
  maximum is not positive.
- Bounds: <= 256 sections, <= 4096 profile points, <= 100000 rail vertices, rungs x samples <= 2^22,
  grid <= kMaxCells. Cancellation is polled once per rung and returns an empty field.

## Mutation notes
Boundary-sensitive code is pinned by tests: kEps inclusion (cells on rail edges exactly equal the
analytic value, flat test), anchors (section away from ends), step vs blend (hold test, cell on the
step edge), swap/reverse remap, closed fill inner choice (both rail orders), each validation branch,
smooth monotonicity incl. the steep-neighbour clamp (steep test), size caps (TooLarge), cancel.
No known equivalent mutants.
