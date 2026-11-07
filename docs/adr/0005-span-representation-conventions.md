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
| `intersect(Span, Span)` sampling / piece split | Depth ≤ 16 (≤ 65 537 samples) / depth ≤ 10 (≤ 1024 pieces) per Bezier. | Pieces at the cap are used as they are. |
| `intersect(Span, Span)`, Bezier pairs (T-0180) | An explicit work budget, `IntersectLimits::pairWork` (default 5·10^8 units, about 1-2 s of Release CPU; see below). | The result has `status == Truncated` and holds the hits found before the budget ran out. |
| `intersect(Contour, Contour)`, `selfIntersections`, `hasSelfIntersections` (T-0180) | `IntersectLimits::totalWork` (default 4·10^9 units) over all span pairs of the call, each pair also under `pairWork`. | `status == Truncated`; `hasSelfIntersections` answers true. |
| Non-finite input (T-0180) | Any span with a NaN or infinite coordinate or arc parameter is refused before anything is computed. | `status == InvalidInput`, empty result (`hasSelfIntersections` true). |

Accuracy of `length`: the result is within `relTol` (default 1e-10) of the true length plus the coordinate rounding noise (~ `ε · position`). Pass 1 is bit-identical to the pre-T-0178 code and is used when the largest coordinate is ≤ 1e4 × the control polygon's length and it finishes within its piece budget; every other curve (position/size above 1e4, where the old scheme could run for seconds or not terminate) takes pass 2, whose values differ from the old ones by the old scheme's own rounding noise (≤ ~1e-5 relative in the sampled corpus, typically 1e-12).

`intersect(Span, Span)` for two Beziers is polynomial in the sample and piece counts above; the near-coincident long-curve case (two loops of extent 1e6 at the default tolerance: ~1.6·10^4 raw hits) is no longer quadratic in the hits (hit merging uses a hash grid with the linear scan's first-match semantics, `hit_index.hpp`; the touch-settling trail and minima are ordered trees, not sorted vectors). Results are unchanged; CPU time for that pair fell from 5.2 s to about 0.5 s (0.47-0.56 s over several runs, Release).

### T-0180: the work budget of the Bezier intersection

T-0178 left `intersect(Bezier, Bezier)` bounded only by the per-curve caps, which is polynomial but large (pairs of near-coincident loops took 30-44 s at extent 1e7..1e8 and 125 s at 1e100, NaN/infinite spans hung, and 10 of the 12 000 random pairs of the verifier's corpus took over 20 s). T-0180 adds three things.

**1. A deterministic work budget.** The Bezier path of `intersect(Span, Span)` counts work in abstract units: a chord-pair box test is 1, a curve evaluation or nearest-point step 64, a Newton refinement 256, a walk step through the minima tree 6 per node visited, a hit comparison in the merge 4, a sample of a huge arc 8 (charged before the samples are made). A unit is 1.7-4 ns of Release CPU depending on the mix (measured on a loaded 2020s machine). Counting units instead of reading a clock makes a truncated result reproducible and independent of the machine. The loops that can run long check the meter and stop; what has been found is kept. When candidate generation spends the pair's budget, the settling and merging of the candidates found so far get a reserve of one eighth and one sixteenth of it, so a truncated result is the best the budget buys and not empty; the total is therefore at most 1.19 times `pairWork`. `IntersectLimits` is passed to the overloads that take it; the others use the defaults (`pairWork` 5·10^8, `totalWork` 4·10^9), which a test can replace for a scope (`detail::ScopedDefaultLimits`, `intersect_defaults.hpp`). Line and arc pairs are closed-form and not metered.

**2. Truncation is reported, never silent.** `IntersectionResult` and `ContourIntersections` carry `status` (`Complete`, `Truncated`, `InvalidInput`) and `complete()`. A truncated result contains only genuine hits (each verified against both spans) but may miss some. Callers: `trimToBoundary`, `trimSpansAt` and `extendSpan` (edit_ops) refuse with the new `EditStatus::TooComplex` and change nothing; `hasSelfIntersections` answers true when it cannot rule a self-intersection out, and V-carve treats a pair or a self test that is not complete as a possible crossing (the vectors are resolved together with the exact union, which is also right for vectors that do not cross); snapping uses the hits it gets (best effort, a suggestion the user sees). Offset, boolean and the medial axis do not call `intersect` (they work on polygons through Clipper2 and their own routines), so they need no handling.

**3. Non-finite spans are refused up front** (`InvalidInput`, empty result) in every entry point, for all span kinds, before `isZeroLength`/`length()` or any sampling can run on them.

**What made most of the old cases fast was not the budget but two quadratic loops.** The chord-pair loops (A's chords against B's chords per piece pair, and the chords of one Bezier against each other for self-intersections) tested every pair, 65 537 x 65 537 = 4·10^9 pairs for the long curves, and the self-intersection loop paid a `hypot` per pair for pairs far apart. They now go through the box index of `box_index.hpp` when there are more than 4096 pairs (64 chords for the self loop): the pairs and their order are the same (the index only skips pairs whose boxes do not meet; the self loop grows each chord's box by half its own length so that however `segmentProximity` rounds its division, no pair it could report a crossing for is skipped). Results are bit-identical. Contour-level calls find their span pairs the same way and drop duplicate hits through `HitIndex` instead of an `any_of` scan; points outside the hash grid's range (extents beyond 1e15 cells) are scanned linearly and charged to the budget.

Measured, Release, CPU seconds on a machine at load average ~20 (about 1.5-2x inflated; `before` is main at 3586981, `after` this change with the defaults):

| Case | before | after |
|---|---|---|
| loop pair 1e6, a-a / a-shifted / a-reversed | 0.50 / 1.11 / 0.44 s | 0.06 / 0.76 / 0.06 s, complete, same hits |
| loop pair 1e7, a-a (132 717 hits) | 6.0 s | 0.57 s, complete, same hits |
| loop pair 1e7, a-shifted / a-reversed | 17.4 s / 6.5 s | 1.74 s truncated / 1.10 s complete (152 267 hits) |
| loop pair 1e8, a-a / a-shifted / a-reversed (890 130 / 324 689 / 818 779 hits) | 35.3 / 43.3 / 37.2 s | 0.98 / 1.13 / 1.10 s, truncated |
| huge curves 1e8, Bezier-Bezier / with itself | 27.4 s / 35.2 s | 0.23 s complete (2 hits) / 0.99 s truncated |
| huge curves 1e100, Bezier-Bezier / with itself | 99.5 s / over 200 s | 0.23 s complete (0 hits) / 1.13 s truncated |
| NaN / infinite control point, selfIntersections | 56-58 s | 0.00 s, `InvalidInput` |
| verifier corpus, 12 000 random pairs (all span kinds, four calls each plus contour calls) | 696 s total, 54 seeds over 2 s, 11 over 20 s (2 at the 60 s limit) | 23 s total, 1 seed over 2 s (2.15 s for ~10 calls), none truncated |

On the 12 000-pair corpus and a 10 000-pair tie-heavy corpus the result hashes (points, tangent flags, order, overlaps, contour hits, polygonisation) are identical to main on every pair that finished there, and the two corpus pairs that timed out on main now finish. Ten thousand spans of a contour pair (10 000 crossings) take 13 ms and the cost is linear to within the log factor (4.9x for 4x the spans), and 5 000 spans of a self-crossing contour 12 ms.

Remaining limits: the budget bounds CPU, not memory: one pair can hold up to a few million candidates (tens of megabytes). The touch-settling minima query (`GapMinima::lowestIn`) still scans every minimum inside a bracket, so near-coincident long loops of extent 1e7 and above truncate rather than complete; making it logarithmic is future work. A hit list that was cut short is a prefix of nothing in particular (the best candidates found first are kept, not the first along the curve). The T-0208 extent guard in `edit_ops` is not relaxed by this change: the intersections it protects are now bounded, but the other costs behind the guard (`BoundarySet` classification, the arc probe of `extendSpan`) are not covered by the budget (see T-0179).

The 2^20 piece cap of `BezierSpan::length` pass 2 is a defensive bound that no finite input reaches (the noise floor stops a size-1 loop after ~2^17 pieces at any scale), so it has no test.
