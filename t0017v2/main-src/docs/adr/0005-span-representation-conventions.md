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
