# ADR-0009: Intersection results and point-in-contour conventions

Date: 2026-10-24 · Status: accepted (director, 2026-10-24; residuals tracked in T-0005)

## Decision
1. `intersect(Span, Span)` returns an `IntersectionResult` with two lists: isolated `points`
   (`Intersection`: position, `tA`, `tB` in each span's own parameter, `tangent` flag) and
   coincident `overlaps` (`Overlap`: a parameter interval on A and the matching interval on B,
   in B's direction of travel). Points that fall inside a reported overlap are not repeated.
   Line/line and line/arc/arc pairs are solved analytically; a pair involving a Bezier is
   polygonised at `tol.chordal`, seeded from chord crossings and near misses, and refined by
   Newton on `A(t) − B(s) = 0`, falling back to golden-section minimisation of the distance when
   the tangents are parallel. Acceptance is `|A(t) − B(s)| ≤ tol.distance`.
   Line/line intersection is symmetric in its arguments: collinearity is judged against the
   infinite line of the longer segment using both end points of the shorter (equal lengths are
   tie-broken by a canonical, direction-independent segment order); non-collinear segments are
   intersected directly however small the angle (only an exactly zero cross product means
   "parallel"), and an end point within `tol.distance` of the other segment counts as a contact.
   Line/arc hits are computed from the foot of the perpendicular from the centre
   (±sqrt(r² − h²)), avoiding the cancellation of the expanded quadratic for long lines.
   Arc/arc overlaps on the same circle report B's interval in an unwrapped angle frame, so a
   full-circle B maps to the full range 0..1.
   Tangent circle pairs (external or internal, within `tol.distance`) report one point: the
   midpoint of each circle's point facing the other on the centre line. The general radical-line
   formula is not used there, because its error grows like r·gap/d when the centres are close.
   Outside the tangent band the radical-line distance and half-chord use factored forms,
   (r1−r2)(r1+r2) and (r−a)(r+a), to avoid cancellation. Concentric circles of different radii
   give nothing.
   Every reported `Intersection::point` lies within `tol.distance` of both A(tA) and B(tB). For
   line/line crossings the parameters are re-derived by projecting the reported point onto each
   segment, because at nearly parallel angles the raw line parameters are ill-conditioned along
   the lines.
2. Tangency is a property of the hit, not a separate result kind: `tangent` is set when the
   infinite-line/circle test is within `tol.distance` of touching (analytic cases) or when the
   unit tangents are parallel to 1e-6 (Bezier cases). Collinear end-to-end touches count as
   tangent points.
3. Degenerate spans (length ≤ `tol.distance`) act as points: they intersect where they lie on
   the other span, with parameter 0 on their own side.
4. Contour-level results (`ContourIntersections`) carry span indices. A hit at the junction of
   two consecutive spans is reported once (first span wins). `selfIntersections` ignores the
   shared junction of consecutive spans (including the closing junction of a closed contour) and
   reports everything else, including fold-back overlaps and a cubic Bezier looping through
   itself (`spanA == spanB`).
5. Point-in-contour uses a winding number computed by ray casting towards +X over y-monotone
   pieces (lines, arcs split at horizontal tangents, polygonised Beziers), with the half-open
   rule `y >= p.y` so vertices and horizontal edges never double count. `FillRule::EvenOdd` and
   `FillRule::NonZero` decide the interior; points within `tol.distance` of the boundary are
   `OnBoundary`, and `isInside` treats them as inside. Open contours are implicitly closed.
6. `signedArea` is exact via Green's theorem (lines: shoelace term, arcs: closed-form sector
   term, Beziers: 3-point Gauss–Legendre, exact for the degree-5 integrand); positive means CCW.
   Open contours have area 0 and `Orientation::Degenerate`, matching `Polyline::signedArea`.

7. `closestPoint(BezierSpan)` is global. Every root in [0,1] of the quintic
   g(t) = (B(t) − q)·B′(t) is isolated by de Casteljau subdivision of its Bernstein coefficients
   (no sign change ⇒ no root; exactly one ⇒ one root, bisected on a strict bracket). Split points
   and both end points are added as candidates, and the closest candidate wins. Seeded Newton
   iteration was rejected because it converges to whichever local minimum the seed lies in.

## Rationale
- Trim, Extend, Vector Validator and the interactive snapping all need "where do these touch"
  with overlap awareness; a single result type with both lists avoids a second API for the
  collinear cases that are common in CAD input (shared edges, retraced segments).
- Keeping tangency as a flag lets callers that only care about crossings ignore it, while
  fillet/offset-intersection checks can treat touches specially.
- Golden-section as the tangency fallback needs no derivatives of the distance function and is
  robust where Newton's Jacobian is singular; it is only reached when Newton fails, so the
  common crossing case keeps quadratic convergence.
- Boundary points counting as inside matches the closed-set convention users expect for
  "inside selected vectors" style queries; callers needing the strict test use `locatePoint`.

## Consequences
- Future work (outside T-0002, T-0001 code unchanged): `BezierSpan::length()` defaults to
  `relTol = 1e-10` and subdivides on the Gravesen bound (polygon − chord). That bound shrinks
  only quadratically per split, so one call needs on the order of 1e5 subdivisions: about 48 ms
  in a Debug build, against 0.5 ms at `relTol = 1e-6`. `isDegenerate(Span)` pays this cost for
  Beziers. T-0002 avoids it by bounding the length between the chord and the control polygon,
  and computes the length only in borderline cases, at 1e-6. A cheaper fix would be a
  Gauss–Legendre length per piece, or a looser default tolerance.
- Overlap detection covers line/line and arc/arc pairs only. Coincident stretches involving a
  Bezier (e.g. a Bezier against itself, or against a line it partly lies on) are reported as
  sampled tangent points along the shared part, not as `Overlap`s. Bezier overlap detection is
  future work; consumers such as Vector Validator must not rely on `overlaps` for Beziers.
- Overlaps of arcs on the same circle may be split into two intervals at A's start seam when
  the overlap wraps around; consumers that need a single interval must merge them.
- The Bezier path is tolerance-driven: a finer `tol.distance` without a finer `tol.chordal`
  still converges, but a very coarse `tol.chordal` can miss contacts between tightly curved
  spans. Callers should pass the same `Tolerance` they polygonise with.
