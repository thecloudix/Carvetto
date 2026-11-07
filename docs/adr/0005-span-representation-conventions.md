# ADR-0005: Span representation conventions

Date: 2026-10-24 · Status: accepted (director, 2026-10-24; verified consistent with ARCHITECTURE.md)

## Decision
1. `ArcSpan` stores centre, radius, start angle, end angle and an `ArcDirection` (CCW/CW). The swept angle is always taken in (0, 2π]: coincident start/end angles (mod 2π) denote a **full circle**. A zero-length arc therefore cannot be represented; degenerate pieces are `LineSpan`s.
2. `Span` is `std::variant<LineSpan, ArcSpan, BezierSpan>` with free functions (`length`, `pointAt`, `bounds`, `reversed`, `transformed`, `polygoniseInto`) rather than a class hierarchy.
3. `transformed(Span, Transform2)` returns a `std::vector<Span>`. Lines and Beziers always map to one span. An arc stays an arc when the transform is conformal (rotation, uniform scale, mirror, translation; mirroring flips its direction). Under any other affine map (non-uniform scale, shear) the arc is first converted to cubic Beziers of ≤ 90° each, which are then transformed exactly.
4. Polygonisation appends the end point of each span but not its start, so spans chain without duplicates; a closed `Contour` drops the repeated closing vertex and relies on `Polyline::closed`.
5. Span parameters `t ∈ [0,1]` are arc-length-linear for lines and arcs but the native curve parameter for Beziers. `Contour::pointAt` and `Polyline::pointAt` are arc-length-linear over the whole chain.

## Rationale
- Representing a full circle as a single span keeps circle objects, circle snapping and "Only Circles" filters simple; the (0, 2π] rule removes the ambiguity between "no sweep" and "full sweep" without an extra flag.
- A variant keeps spans as plain copyable values with no virtual dispatch, which suits Clipper2 interop, undo snapshots and serialisation.
- Stretching a circle non-uniformly produces an ellipse, which no span type represents exactly; converting to Beziers keeps the operation closed over the span set and bounded in error (standard 4/3·tan(θ/4) construction, radial error ≈ 2.7e-4·r per 90°). Returning a list makes the span-count change explicit at the call site.

## Consequences
- Downstream code must not assume one-span-in/one-span-out for transforms; `Contour::transform` already handles the expansion.
- Arc fitting after offsets/booleans (T-0003, Fit Curves) should produce `ArcSpan`s with the same angle convention.
- Zero-length spans are detected with `isDegenerate(span, tol)` and cleaned by later cards (Vector Validator).

## Appendix A2 — Work bounds of the Bezier routines (T-0178)

Every recursive or subdividing Bezier routine has an explicit bound and a stated behaviour when it is hit.

| Routine | Bound | When it is spent |
|---|---|---|
| `BezierSpan::length(relTol)` | Pass 1 (the original Gravesen subdivision, depth ≤ 40) examines ≤ 2^19 pieces; pass 2 ≤ 2^20 pieces. | Pass 1 gives up and pass 2 runs; pieces left in pass 2 take their unrefined estimate (best effort). |
| `BezierSpan::length`, flatness | A piece with `polygon − chord ≤ 8·ε·(largest coordinate of the piece)` stops (pass 2 only): below that the flatness is rounding noise. Pass 2 measures the curve relative to its first control point, so the noise is that of its size, not its position. | — |
| `BezierSpan::length`, range | Control values beyond 1e100 are scaled by an exact power of two and the result scaled back; the result is `+inf` when it exceeds the double range. | — |
| `bounds()` | Closed form (no recursion). Control values beyond 1e100 are scaled before the discriminant is formed; if evaluating the extrema still overflows, the control-polygon box (which contains the curve) is returned. The result contains the curve down to extents of about 1e-11 mm; sub-picometre curves may collapse to their end points, as before, because the derivative quadratic's coefficients are compared with an absolute 1e-14 (irrelevant at CNC sizes). | — |
| `closestPoint(BezierSpan, p)` | Quintic root isolation, depth ≤ 60. Curve and point beyond 1e100 are scaled by a power of two (`t` unchanged). | — |
| `polygoniseInto` / `polygoniseRegionInto` (Bezier) | Depth ≤ 24 and ≤ 2^20 splits per curve. | The remaining pieces become single chords: the polygon is coarser than `tol.chordal`, still ends at the curve's end. |
| `polygoniseInto` / `polygoniseRegionInto` / `ArcSpan::segmentCountFor` (arc, T-0182) | ≤ `kMaxArcSegments` = 2^20 equal chords per arc. A non-finite arc (NaN/infinite centre, radius or angles) gets 1 chord. | The count is cut to the cap: the polygon is coarser than `tol.chordal` (deviation of a chord ≈ radius·(sweep/2^20)²/8, e.g. 4.5 mm for a half circle of radius 1e12) and still ends at the arc's end point. |
| `intersect(Span, Span)` sampling / piece split | Depth ≤ 16 (≤ 65 537 samples) / depth ≤ 10 (≤ 1024 pieces) per Bezier. | Pieces at the cap are used as they are. |

Arc polygonisation (T-0182): before the cap the chord count was proportional to radius / chordal tolerance without limit (a radius of 1e12 at the default tolerance made 70 million points in 2.6 s; larger radii ran out of memory, and when `1 - tol/radius` rounds to 1 the count was an undefined float-to-integer conversion). Now `segmentCountFor` returns at most 2^20 and `segmentCountIsCapped(tol)` tells whether it cut the count (also true for a non-finite arc). `polygoniseInto`, `polygoniseRegionInto` and `Contour::polygonise` take an optional `bool* coarse` that they set (never reset) when the polygon is coarser than the tolerance: an arc past the cap, a Bezier past its depth/split cap, or a non-finite span. Measured (Release, one full circle, tolerances 1e-3 and 1e-9): radii 1e9, 1e10, 1e11, 1e12, 1e15, 1e100, 1e300, 1.7e308 all give 1 048 576 points in 0.024-0.038 s CPU (inside `ulimit -v 400000`); an infinite radius gives 1 point in 0 s. Before: 2.2 M / 7.0 M / 22 M / 70 M points in 0.09 / 0.21 / 0.72 / 2.6 s for 1e9..1e12 at 1e-3. Arcs that need no more than 2^20 chords are bit-identical to before (hash of the points of 21 720 arcs: radii 1e-6..1e9, both directions, full circles, tiny sweeps, centres up to 5e8 from the origin, tolerances 1e-9..1, open and closed contour paths, identical on main and the branch).

Callers. Those where a coarse polygon only costs fidelity use the capped result: drawing (`buildCanvasScene`), selection hit tests and the marquee, snapping, `medialAxis` winding rings, the V-carve skeleton check, edit operations and span intersection (through `segmentCountFor`). Those whose output must meet the tolerance refuse: `toClipperPath`, used by `booleanOp`/`unionContours`, `offsetContours` and `OpenPathClipper`, leaves a contour with a coarse polygon out (as degenerate input is) and counts it: `BooleanResult::refusedContours`, `OpenPathClipper::refusedContours()`. Offsetting open contours by spans (`offsetOpenContour`) keeps arcs as arcs and only polygonises Beziers, which were already bounded. `medialAxis` (the one caller that indexes by coordinates) ignores a contour with a non-finite coordinate, radius or angle, or a coordinate or radius beyond 1e12 mm in magnitude (the importer's cap; exactly ±1e12 is accepted), promptly: it is counted in `ignoredContours`, and a finite contour ignored for its magnitude also sets `accuracyCapped`. Before, a radius of about 1e100 made it hang and 1.7e308 crashed (a NaN converted to an index); a region at an offset of 1e12 mm is now ignored where it used to be computed with `accuracyCapped` set.

Accuracy of `length`: the result is within `relTol` (default 1e-10) of the true length plus the coordinate rounding noise (~ `ε · position`). Pass 1 is bit-identical to the pre-T-0178 code and is used when the largest coordinate is ≤ 1e4 × the control polygon's length and it finishes within its piece budget; every other curve (position/size above 1e4, where the old scheme could run for seconds or not terminate) takes pass 2, whose values differ from the old ones by the old scheme's own rounding noise (≤ ~1e-5 relative in the sampled corpus, typically 1e-12).

`intersect(Span, Span)` for two Beziers is polynomial in the sample and piece counts above; the near-coincident long-curve case (two loops of extent 1e6 at the default tolerance: ~1.6·10^4 raw hits) is no longer quadratic in the hits (hit merging uses a hash grid with the linear scan's first-match semantics, `hit_index.hpp`; the touch-settling trail and minima are ordered trees, not sorted vectors). Results are unchanged; CPU time for that pair fell from 5.2 s to 0.47 s (Release).

Known limits (not bounded by T-0178; carded as T-0180): `intersect(Bezier, Bezier)` is bounded only by the per-curve caps above, which is polynomial but large: pairs of near-coincident loops took 30-44 s at extent 1e7..1e8 and 125 s at extent 1e100 (T-0178 verifier, Release); only the 1e6-extent repro was brought under 1 s. The contour-level hit collection (`appendContourHits`) deduplicates hits with an `any_of` scan, O(n^2) in the number of hits. The T-0208 extent guard therefore stays in force.

The 2^20 piece cap of `BezierSpan::length` pass 2 is a defensive bound that no finite input reaches (the noise floor stops a size-1 loop after ~2^17 pieces at any scale), so it has no test.
