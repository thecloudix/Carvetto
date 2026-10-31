# ADR-0009: Intersection results and point-in-contour conventions

Date: 2026-10-24 · Status: accepted (director, 2026-10-24); amendments of 2026-10-24 (T-0005), 2026-10-25 (T-0007) 2026-10-25 (T-0008) accepted

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

## Amendments

### 2026-10-24 — T-0005 (residuals of T-0002 round 3) · Status: accepted (director, 2026-10-24; residuals tracked in T-0007)
1. **Junctions of short closed contours (decision 4).** A span pair can share two junctions: in a
   closed contour of exactly two spans, the pair (0, 1) meets at the end of span 0 and at the
   closing junction (start of span 0); hits within `tol.distance` of either are ignored. In a
   closed contour of one span the span's end meets its own start; for a Bezier that point is
   ignored as well (a full-circle arc never reports it). A cubic has at most one double point, so
   a closed one-Bezier contour never self-intersects. An open contour still reports its ends
   meeting, as before.
2. **Nearly concentric arcs (decision 1).** The concentric special case is limited to circles
   that are the same within tolerance (centre distance d and |r1 − r2| both ≤ `tol.distance`:
   overlaps). Every other pair, including d ≤ `tol.distance` with |r1 − r2| > `tol.distance`,
   follows the general rule: closest approach |d − |r1 − r2|| (internal) or |d − (r1 + r2)|
   (external) within `tol.distance` gives one tangent hit at the midpoint of the two facing points
   on the centre line. When d is tiny that direction is poorly determined, but both facing points
   lie exactly on their circles and are |r1 − r2| − d ≤ `tol.distance` apart, so the hit is within
   tolerance of both arcs wherever it lands; d = 0 with |r1 − r2| > `tol.distance` is rejected
   before the direction is needed. Coincident-arc results are computed in a canonical argument
   order (touch points are placed on A's circle).
3. **Bezier pairs: symmetry, shallow crossings, loops, merging (decision 1).** Final algorithm
   for every pair that involves a Bezier (`polygoniseAndRefine`):
   - **Canonical order.** The pair is solved in one order (by span kind, line < arc < Bezier,
     then by control points) and the other order is answered by swapping the parameters, so
     `intersect(a, b)` and `intersect(b, a)` are exact mirror images.
   - **Pieces.** Each Bezier is cut by repeated halving until the cone of its first-derivative
     control vectors spans at most 45° (the tangent turns by at most that much over the piece);
     halving stops at depth 10 or when the control polygon is shorter than `tol.distance`. Loops,
     cusps and tight bends therefore end up as several pieces, one per branch. Lines and arcs are
     one piece. Every pair of pieces whose bounds come within reach (2·`tol.chordal` +
     `tol.distance`) is searched on its own, and its candidates are mapped back to the span
     parameters (halving is an affine reparametrisation, so the mapping is exact). Without this,
     near a loop the seeds, Newton steps and nearest-point queries of one branch could settle on
     another branch, and a crossing between different branches was lost (T-0005 round 2:
     tA 0.5809 / tB 0.6657, 18·tol from the nearest other crossing).
   - **Per piece pair:** (a) chord seeds: both pieces are polygonised at `tol.chordal`, every chord
     pair within reach gives a seed, refined by Newton on A(t) − B(s) = 0. Only seeds that
     converge to an exact crossing (residual ≤ 1e-6·`tol.distance`) are kept; where the tangents
     are nearly parallel Newton stalls, and those stretches belong to the scan. (b) signed-distance
     scan along the Bezier piece: every chord of it that came within reach is cut into 8 equal
     parameter pieces and the signed distance to the other piece (nearest point found by a local
     Newton projection seeded from the previous sample, falling back to the global search of
     decision 7) is evaluated at the 9 sample points. A sign change between neighbours is a
     crossing, located by Illinois regula falsi. An interior sample that is no further from the
     other piece than both neighbours marks a dip; when it lies within 1.5·`tol.distance` plus
     twice the variation to its neighbours (the bound comes from the samples themselves, so it
     holds whatever the curvature of the other span), the extremum towards the other piece is
     searched by golden section over the two pieces around it (12 steps). An extremum beyond the
     other span brackets two crossings; one on the same side within 1.5·`tol.distance` is a touch
     candidate. A sample whose nearest point is clamped at an end of the other piece measures the
     distance to that end point, not to a foot on the curve: it never rules a dip out, and it is
     itself a touch candidate when within 1.5·`tol.distance`, as are the samples at the ends of a
     chord that are no further than their inner neighbour and the samples at the ends of the span.
   - **Settling touches.** Every candidate that is not an exact crossing but lies within
     1.5·`tol.distance` is walked to a local minimum of the distance between the curves along the
     unsplit A (`settleTouch`): probes a tiny step either side (widened while the gap is flat to
     1e-12), then a downhill walk with doubling steps capped at 1/64 of the parameter range until
     the gap rises, then golden section inside the bracket to 1e-10 in parameter. A walk that
     reaches an end of A stops there (an end touch). When the gap falls away on both sides of the
     candidate both descents are followed. The result is kept only if no closer approach exists a
     step of 1e-5 either side (the same-branch distance, via the seeded projection), which rejects
     walks that ended on a slope. Because the walk runs on the original spans, cuts between
     pieces cannot lose a touch. Gap evaluations are recorded in a sorted trail; a later candidate
     between two recorded samples less than 1/64 apart, with its gap between theirs, sits on an
     already walked slope and is skipped.
   - **Acceptance.** A settled touch is kept when its distance is ≤ `tol.distance`, flagged
     `tangent`; a settled candidate whose descent ran into a crossing is an exact hit like any
     other. An exact hit whose parameter is within 1e-6 of a span end is snapped to that end (the
     other parameter re-projected) when the snapped pair is still exact, because along a
     tangential contact Newton stops anywhere in a band where the curves agree to rounding.
   - **Merging.** Hits are merged most-exact first (smallest residual): a hit within
     `tol.distance` of an already kept one is dropped. If both are exact crossings at distinct
     points (more than 1e-3·`tol.distance` apart) the kept hit becomes `tangent`, because two
     crossings the tolerance cannot separate are a touch; a touch absorbed by a nearby crossing
     leaves the crossing as it is. Crossings further apart than `tol.distance` are all reported.
   - **Guarantees and limits.**
     - *Symmetry:* exact by construction (canonical order).
     - *Crossings:* every crossing that is more than 2–3·`tol.distance` from any other hit, at an
       angle above 1e-4 rad, is reported (brute-force references over random arc/Bezier and
       Bezier/Bezier pairs, Beziers hugging an arc within a few tolerances, and perturbed copies
       of looping or tightly bent cubics, including crossings between different branches near a
       loop).
     - *Touches (closest approach within tolerance, no crossing):* a touch whose gap h satisfies
       0 ≤ h ≤ `tol.distance` is reported exactly once, flagged `tangent`, with the reported
       point within h of both curves, for: every analytic pair (line/line, line/arc, arc/arc),
       lines against cubics, and mirrored cubics against convex cubics (verified at h ∈ {0, 1e-5,
       1e-4, 5e-4, 0.99·tol}; ~10,000 random line and mirrored-cubic touches with 0 drops).
       h > ~1.05·`tol.distance` gives nothing (the band between is decided by rounding of the
       settled gap). **Residual (resolved by T-0007, see the 2026-10-25 amendment):** an external circle or arc tangent to a
       convex cubic with 0 < h ≤ `tol.distance` is dropped in bands of about ±0.0035 in the
       cubic's parameter around the 1/16 piece cuts and near the span start (0.5–5 % of positions
       at R = 0.2 mm; e.g. `BezierSpan{{-3,9},{-1,-3},{1,-3},{3,9}}` against
       `ArcSpan::fullCircle({-2.4572958060271235, 5.0734751657764976}, 0.2)` gives no hit);
       exact tangency (h = 0) is always found. A touch is reported wherever the distance between
       the curves has a joint local minimum (a local minimum along both curves, measured to the
       same branch); for curves running within tolerance of each other over a stretch the result
       is their crossings plus those closest approaches, nothing in between (brute-force gap
       profiles over perturbed copies of random cubics).
     - *End touches (resolved by T-0007, see the 2026-10-25 amendment):* when an end of B comes within tolerance of A's interior
       the touch is reported; when an end of A comes within tolerance of B's interior (the foot
       of the walk is clamped at A's end) it can be missed, so end touches are not symmetric in
       that respect (e.g. `ArcSpan{{1.2115827528094094, 3.0195632709834515}, 5.9592789586434449,
       5.5749720172249306, 7.0509709398542197, CCW}` against
       `BezierSpan{{5.4971827427603293, 7.1603566376986816}, {7.6315225436121663,
       4.949460611577611}, {7.7363570760098641, 1.4786963115444329}, {5.7389721228483266,
       -0.85517614189571634}}` reports only one of its two end touches).
     - *Heuristics that remain:* a double crossing or a touch confined to one eighth of a chord
       whose dip is shallower than the sampled variation bound may be reported as one tangent
       touch, or missed; the 1/64 step cap and 1e-5 validation step are scale assumptions
       (parameter space, so very long spans have correspondingly coarser steps). Bezier overlaps
       are still reported as touches/crossings, not `Overlap`s.
     - *Pre-existing limits elsewhere in `intersect`, unchanged by T-0005 (sliver loops and the
       junction bulge resolved by T-0007, see the 2026-10-25 amendment):* a thin (sliver) self-loop of a single cubic, up to
       about 0.3 mm long and 0.3–5 µm thick, is not reported by `selfIntersections` (its chord
       crossings are below the chordal sampling); a Bezier that bulges ≤ ~`tol.distance` outside
       the arc it joins and re-crosses it within ~10·`tol.distance` of the junction reports only
       the junction (the exact-zero sample at the junction defeats the sign test); the analytic
       line/arc and arc/arc rules treat an infinite line or circle within `tol.distance` of
       touching as one tangent hit (resolved by T-0008, see its amendment), so two genuine
       crossings up to about 2·√(2·r·tol) apart fold into one tangent hit (≈ 0.09 mm for r = 1 mm, 0.9 mm for
       r = 100 mm); same-circle partial arcs that touch end to end report a point whose residual
       to the arc ends can reach 1.5·`tol.distance` (the midpoint rule in the angular frame).
     - *Cost:* in a Debug build a pair of nearly identical 4 mm cubics takes about 4 ms, a
       Bezier hugging an arc about 1 ms, typical crossing pairs well under 1 ms.

### 2026-10-25 — T-0007 (residuals of T-0005 round 3) · Status: accepted (director, 2026-10-25; hugging-stretch end touches tracked in T-0008)
Changes to the Bezier pipeline of the 2026-10-24 amendment (decision 1) and to `selfIntersections`
(decision 4). Everything not mentioned below is unchanged.
1. **Chord seeds are touch candidates too.** A chord-pair seed that Newton does not converge to an
   exact crossing is no longer discarded: the closest point of A's chord is projected onto A, that
   point onto B, and the pair is settled like any touch candidate (`settleTouch`), accepted when
   its distance is within 1.5·`tol.distance` + 4·`tol.chordal` (each curve lies within
   `tol.chordal` of its chord and the chords come within 2·`tol.chordal` + `tol.distance`). Every
   touch within tolerance has such a chord pair, so touches no longer depend on the scan's
   sample-based reach bounds, which narrow dips (small circles) defeated near the 1/16 piece cuts
   and the span ends (the T-0005 AC1 residual). The scan's dip search no longer stops early when
   its two probes differ little: probes either side of a narrow minimum differ by nothing while
   the minimum is far below both.
2. **Full circles have no ends.** When A is a full circle (in canonical order only an arc can be,
   never B), the settling walk wraps across the seam instead of stopping there as at an end; the
   results are reported at the wrapped parameter in [0, 1). Previously a touch whose descent
   crossed the seam was reported at the seam with the gap there (or dropped when that exceeded
   tolerance).
3. **Chord ends.** A chord end no further from O than its inner neighbour (same side) is the
   sampled bottom of a dip that straddles the chord end, a piece cut or the span end, invisible to
   the interior test. Within the interior-dip reach (1.5·`tol.distance` + twice the variation) the
   extremum over the end piece is searched by golden section (12 steps): beyond O it brackets a
   double crossing confined to that piece (e.g. S swinging across O and back just before its end:
   previously one of the two crossings was lost); otherwise the end is a touch candidate carrying
   the variation as extra reach, since the minimum may lie beyond the end.
4. **Zero samples.** A sample at zero distance (to 1e-6·`tol.distance`) carries no sign — typically
   the junction S shares with O. For a piece between such a sample and a non-zero one, the signed
   distance oriented by the non-zero sample is minimised by golden section (16 steps); a minimum
   beyond O brackets the crossing against the non-zero sample. This finds a Bezier that bulges
   past the arc it joins and re-crosses it within a few tolerances of the junction (T-0005
   residual; previously the exact zero at the junction counted as "not positive" and hid the
   sign change).
5. **Settling walks.** The trail rule that skips a candidate lying between two earlier samples
   less than a walk step apart, with its gap between theirs, now applies only to a candidate on a
   slope that descends towards the lower earlier sample, or to a chord seed (item 1, a backstop
   with hundreds of instances along near-coincident curves). A candidate at the bottom of the gap
   or on a ridge is always walked: the earlier samples say nothing about a minimum between them
   (an end touch of B 0.2 mm from a crossing was dropped once a walk towards the crossing had
   recorded samples on both sides of it).
6. **Span ends.** Each end of either span that lies within 1.5·`tol.distance` of the other span's
   bounding box is paired with its nearest point on the other span (global `closestPoint`) and,
   within 1.5·`tol.distance`, settled as a touch candidate. An end touch is a minimum of the
   distance along one span only — along the other the distance may keep falling towards a nearby
   crossing — so neither the scan (which runs along B) nor the walk (along A) reliably reached the
   end of A; seeding the ends directly makes end touches symmetric (T-0005 residual: an arc whose
   end came within tolerance of a Bezier's interior was missed).
7. **Self-intersection of one cubic (decision 4).** The double point is computed analytically:
   with B(t) = a t³ + b t² + c t + d, a double point s ≠ t satisfies a(u² − v) + b u + c = 0 for
   u = s + t, v = s t, so u = −(a × c)/(a × b), v follows by projection on a, and s, t are the
   roots of x² − u x + v = 0 (a cubic has at most one). It is refined by Newton and reported when
   both roots lie in [0, 1], the residual is within `tol.distance` and the loop reaches further
   than `tol.distance` from its double point (its largest distance from the double point, from 16
   samples over the loop refined by golden section; a loop that stays within the tolerance is a
   cusp). The distance to the parameter mid-point B((s + t)/2) is not used: on a lopsided loop it
   can be well inside the tolerance while the loop reaches 1.5–3 µm. Thin sliver loops
   (0.05–0.3 mm long, 0.3–5 µm thick) whose chords never cross at the chordal sampling are
   therefore found. The chord-crossing search remains as a fallback. A loop of a single cubic is
   reported with `tA < tB` (the double point's smaller parameter first).
8. **Arc parameters near the start (round 2).** `closestPoint(ArcSpan)` returns the start point
   but parameter 1 for a point whose angle lies a rounding error before the arc's start (the
   travelled angle wraps to just under 2π and is clamped). Inside `intersect` every closest-point
   query goes through a wrapper that replaces that parameter by 0. Scan samples and chord seeds
   near a partial arc's start were mapped to its end, so touches close to the start of a partial
   arc were dropped in bands of the arc's start angle (e.g. arcs of radius 0.2 starting
   0.12–0.18 rad before a touch near the steep end of a cubic). The bug in `closestPoint` itself
   (`contour_ops.cpp`, T-0002) is left for its owner. (Fixed at the source by T-0008; the wrapper
   is gone.)
9. **Exact tangency (round 2).** An exact contact (Newton residual ≤ 1e-6·`tol.distance`) is flagged
   `tangent` when the unit tangents are parallel to 1e-6 (as before), or when they are within
   1e-3 and A lies on the same side of B half a tolerance either way along A. At an exact tangency
   Newton stops anywhere in the band where the curves agree to rounding, where the tangents can
   differ by ~1e-5.
10. **Walk reuse (round 2).** A walk whose final bracket contains a minimum accepted by an earlier
    walk of the same pair, at or below the bracket's lowest sample, ends there instead of
    searching again. Along near-coincident curves many candidates descend into the same few
    minima; this removes most of the cost added by item 1.
11. **Span ends are always walked (round 3).** The trail rule of item 5 does not apply to the span-end
    candidates of item 6. Where the curves run within tolerance of each other the gap is flat to
    rounding, the earlier samples around an end say nothing about whether it is the bottom, and
    two end touches that `main` reported (a Bezier hugging an arc) were skipped.

**Guarantees, replacing the corresponding T-0005 bullets:**
- *Touches:* a touch with gap 0 ≤ h ≤ `tol.distance` is reported exactly once, flagged `tangent`,
  and h = 1.05·tol gives nothing, against convex cubics at every position: external circles and
  arcs of radius 0.2 and 1 mm (including arcs around the touch and circles with their seam at the
  touch point) at t = i/1000 on four convex cubics × h ∈ {1e-5, 1e-4, 0.99·tol} (0 drops; the
  T-0005 code dropped 622 of 48,048); partial arcs whose start or end lies 0.0002–0.3 rad before
  the touch, in 0.0005 rad steps, at nine positions near both ends of three cubics, radius 0.2
  and 1, both directions (778,896 calls: 0 drops; the T-0005 code dropped 122,948); exact
  tangency with a full circle whose seam is at the contact (192,048 cases: all flagged tangent);
  tangent lines, internal circles and mirrored cubics at 199 offset positions per cubic ×
  h ∈ {0, 1e-5, 1e-4, 5e-4, 0.99·tol}.
- *End touches, for pairs involving a Bezier, at ends not inside a near-coincident stretch:*
  symmetric. An end of either span within `tol.distance` of the other's interior gives one
  tangent hit, whichever span is A and whichever span's end it is (generative test with lines,
  arcs and Beziers ending at h ∈ {1e-5, 1e-4, 5e-4, 0.99·tol, 1.05·tol} from convex cubics and
  arcs, ~9,000 placements in both orders: 0 failures). An end lying inside a stretch where the
  curves run within tolerance of each other (e.g. a Bezier hugging an arc that extends past it) is
  a minimum of a gap that is flat to rounding; such end touches are still missed at roughly
  `main`'s rate (~38–39 % in the verifier's hugging family; no end touch that `main` reports is
  lost) and are tracked in T-0008. The exact line/line, line/arc and arc/arc pairs keep their own
  end-contact rules and still miss oblique end touches (pre-existing; also T-0008). (Both resolved
  by T-0008, see its amendment.)
- *Self-intersections:* every loop of a single cubic reaching further than the tolerance from its
  double point is reported once, and no loop that stays within it (random looping cubics scaled
  to 0.2–3 µm, including lopsided ones whose parameter mid-point is inside the tolerance:
  0 errors); a crossing of a Bezier with the span it joins, more than `tol.distance` from the
  junction, is reported however shallow the bulge before it (re-crossings 2–10·tol from the
  junction with bulges of 0.1–1·tol: 0 misses).
- *Regression guard (measured, round 3):* the reference set — random arc/Bezier, line/Bezier and
  Bezier/Bezier pairs, Beziers hugging arcs, 6,000 Beziers lying within tolerance of an arc that
  extends past them by up to 0.3 rad, jiggled near-coincident cubics, the touch families above,
  end-touch placements, random 1–2 span contours; 48,680 pairs and contours — was run against
  `main` for two seeds. Worse than `main`: 0. Results that differ in the number of hits: 246 and
  244; every one of the pair results was judged better by a dense brute-force gap profile
  (touches and end touches `main` drops, false touches at a full circle's seam removed, a
  crossing next to a span end), the rest are loops of a single cubic that `main` misses
  (residual ~1e-15, 4.5 µm–0.14 mm across). Results with the same hits differ only in the
  position of touches inside the band where the gap is flat to rounding (≤ 1e-4 mm in these runs;
  up to 3.3e-4 mm in the verifier's near-coincident pairs, with the same gap), exact tangencies
  (≤ 1e-6 mm) and crossings (≤ 3e-9 mm); no `tangent` flag changed. Symmetry is exact.
- *Heuristics that remain:* two local minima of the gap separated by a ridge of the order of
  1e-9 mm (curves parallel to rounding over a stretch) may be reported as the deeper one only,
  since the walk's growing steps cross such a ridge; the 1/64 walk step cap, the 1e-5 validation
  step and the trail rule (item 5) are scale assumptions in parameter space; a double crossing
  whose two roots lie within one eighth of a chord, away from a sampled dip or chord end, can
  still be reported as one tangent touch or missed. Bezier overlaps are still reported as
  touches/crossings, not `Overlap`s.
- *Cost (Release -O2, per pair, best of 3, against `main`, on the reference pairs above):*
  generic crossing pairs (random arc/Bezier, line/Bezier, Bezier/Bezier) median 1.05×, p90 1.45×,
  p99 2.3×, worst 4.5× (absolute: median 4.5 µs, worst 67 µs); touches and near misses within
  1.05·tol of convex cubics median 1.5×, p90 2.0×, p99 3.0×, worst 7.6× (median 44 µs, worst
  0.35 ms); near-coincident pairs (Beziers hugging arcs, jiggled copies) median 1.4×, p90 2.3×,
  p99 4.1×, worst 7.6× (median 61 µs, p99 2.3 ms, worst 6.5 ms for two nearly identical 4 mm
  cubics). All pairs: median 1.34×, p90 2.0×, p99 3.0×, worst 7.6×. The worst ratios are pairs
  that took 4–6 µs on `main` and now settle a few touch candidates (~30–40 µs). The verifier's
  round-1 measurement before the round-2 changes (bounding-box filter of item 6, walk reuse of
  item 10) was 1.66× median and up to 18–22× worst. The round-3 verifier, with its own generators
  (seed 31337), measured touch pairs at median 1.63×, p90 2.66×, p99 4.8×, worst 9.5×, and
  end-touch pairs at 1.56× / 2.38× / 3.6× / 9.5× (pairs that took 2–6 µs on `main` now take
  20–60 µs). `selfIntersections` of a single cubic costs about 3 µs against 0.1 µs on `main`
  (~20×), from the analytic double-point solve.
- *Contour labels at triple points:* the pre-existing point dedupe in `appendContourHits` keeps
  one hit per point, so where a span's own loop and a crossing with another span meet within
  tolerance the hit is labelled (i,i) rather than (i,k); the number of points is unchanged.

### 2026-10-25 — T-0008 (exact line/arc intersections: end touches and shared end points) · Status: accepted (director, 2026-10-25)
Changes to decision 1 (analytic pairs, and end touches of the Bezier pipeline of the earlier
amendments), decision 4 (junctions) and `closestPoint(ArcSpan)`. Everything not mentioned below is
unchanged.
1. **End contacts (every pair kind).** An end of either span within `tol.distance` of the other span
   (its interior or its end) is a contact: the end paired with its nearest point on the other span
   (global `closestPoint`), reported at their midpoint, so within half the gap of both. It is
   dropped only when another hit lies within `tol.distance` of it (the usual merge; contacts are
   taken most exact first, so a crossing or a deeper touch represents a nearby end). It is flagged
   `tangent` unless it is exact (gap ≤ 1e-6·`tol.distance`: the end lies on the other span), where
   the tangent rule of any exact contact applies (analytic pairs: parallel tangents to 1e-6). A
   full circle has no ends (its start is a seam). An end contact never counts as a crossing for the
   rule that turns two merged exact crossings into a tangent hit.
   - The rule does not ask the end to be a local minimum of the gap. An end inside a stretch where
     the curves run within tolerance of each other (e.g. a Bezier hugging an arc that extends past
     it), or on the slope down to a crossing or touch further than `tol.distance` away (an overshoot
     past a crossing at a shallow angle; an arc ending 1–2 µm from a tangency it covers), is
     reported in addition to that crossing or touch. The earlier amendments reported only local
     minima of the gap, and their settling walk leaves such ends; the T-0005/T-0007 tests that
     asked for exactly one hit next to a touch now accept further hits only when they are such end
     contacts (`isEndContact` in the tests), and four fixed regressions gained the end contact
     their pair has (counts updated there).
   - In the Bezier pipeline the ends found by the bounding-box filter of T-0007 item 6 are, within
     `tol.distance`, kept as hits as they are (`endHits`), besides being settled as touch candidates
     as before.
2. **Analytic pairs in canonical order.** Line/line, line/arc and arc/arc pairs are solved in one
   argument order (lines before arcs, then by start/end points, or by centre, radius, angles and
   direction) and the other order is answered by swapping the parameters, so `intersect(a, b)` and
   `intersect(b, a)` are exact mirror images. On `main` arc/arc crossings were computed from A's
   centre and were not bit-exact mirrors (4 % of random arc pairs in 20 × 20 mm, 35 % of arcs in
   a 0.1 mm box, 19 % of the analytic end-touch family).
3. **Crossings only on both spans.** An analytic crossing or tangent hit is reported only where it
   lies on both spans to rounding (line parameter in [0, 1], angle inside the sweep with no slack).
   `main` accepted points up to `tol.distance` beyond an end and reported them at the end as
   untangent crossings; such a contact is now the end contact of item 1 (tangent, since the spans do
   not meet there). Line/line's own end-point test is replaced by item 1. This is what made
   oblique end touches (h > 0) and an exact shared end point miss: the regression of the card, a
   line starting 1.8e-15 from an arc's end, nearly tangent to the circle, whose tangent hit at the
   foot lay 1.6 µm beyond the arc's end, gave nothing.
4. **The near-tangent fold rule (final).** Line/arc: with h the distance of the infinite line from
   the centre, h ≥ r gives one tangent hit at the foot when h − r ≤ `tol.distance` (a touch);
   h < r gives the crossings at ±s = ±√((r − h)(r + h)) along the line from the foot: both, untangent,
   when 2s > `tol.distance`, otherwise one tangent hit at the foot. Arc/arc: circles that cross
   (|r1 − r2| < d < r1 + r2) give both crossings, untangent, when the common chord 2h exceeds
   `tol.distance`; otherwise (crossing at points the tolerance cannot separate, touching, or
   passing within `tol.distance`) one tangent hit at the midpoint of the facing points, external
   when |d − (r1 + r2)| ≤ |d − |r1 − r2||, as before. So crossings further apart than the tolerance
   are always both reported, as for Bezier pairs (whose merge distance is the same); `main` folded
   any pair within tolerance of tangency, i.e. crossings up to 2·√(2·r·tol) apart. Crossing
   circles that reach the radical-line formula have d > `tol.distance` (closer centres with
   |r1 − r2| < d are the same circle within tolerance: overlaps), so the formula is well
   conditioned there; residuals of the crossings stay ~1e-11 at crossing angles down to 0.01 rad.
5. **Junction erasure (decision 4), re-examined.** The rule is unchanged: in `selfIntersections` hits
   within `tol.distance` of the junction two consecutive spans share (and of the closing junction)
   are erased. With item 4 a re-crossing more than `tol.distance` from the junction is a crossing
   of its own and survives the erasure, reported where it is; within `tol.distance` it is the
   junction to the tolerance. On `main` the junction and a re-crossing up to 2·√(2·r·tol) away
   folded into one tangent hit midway, which was erased with the junction when the re-crossing was
   up to 2·`tol.distance` away and reported midway, flagged tangent, beyond. The end contacts at a
   junction (exact) are erased with it. The Bezier pipeline already reported crossings more than
   `tol.distance` apart separately (T-0007 item 4 for re-crossings next to a junction).
6. **`closestPoint(ArcSpan)` (T-0007 item 8) fixed at the source.** A query direction that
   `containsAngle` accepts although it lies a rounding error outside the sweep (its 1e-9 rad slack)
   gets the parameter of the nearer end in angle: 0 just before the start (it was 1: the travelled
   angle wraps to just under 2π and was clamped), 1 just past the end. The point and the distance
   are unchanged. The wrapper `closestOn` in `intersect.cpp` is removed; its rule was the same, so
   `intersect` is unaffected. Other callers: `closestPoint(Contour)` returns a parameter consistent
   with its point (improved); `distanceTo`, `locatePoint` and `isInside` use the distance only
   (unaffected); `offset.cpp` (offset pruning) does not call `closestPoint` — its own
   `distanceToSpan` returns a distance only, from `containsAngle(…, 0)` — so it is unaffected.
7. **Cost guards for analytic pairs.** Before the end contacts, pairs whose coarse boxes (a line's
   box, an arc's whole circle) are further apart than `tol.distance`, circles further apart (or one
   inside the other) by more than the tolerance, and a segment wholly outside or inside a circle by
   more than the tolerance return nothing; an end is only queried with `closestPoint` when its
   distance to the other span's infinite line or circle is within tolerance.

**Guarantees, replacing the T-0007 bullet on end touches and the T-0005 limit on the fold rule:**
- *End contacts, analytic pairs:* 40,000 generated cases, 10,000 each of line/line, line/arc,
  arc/line and arc/arc (the approached span first; arcs include full circles), a span ending at
  gap h from the other's interior with oblique (up to 69° from the normal), normal (within 1.1°) and
  near-parallel (1.1–8.6° from the tangent) approaches, h ∈ [0, 0.99·tol] (a tenth exactly 0, a
  tenth 1e-9–1e-4 mm) or [1.05, 2]·tol, both orders, the gap measured by brute force: 0 failures
  (27,723 within tolerance: exactly one hit within 2·tol of the end, within the gap, tangent unless
  exact; 12,277 beyond: none; every residual within tolerance; every pair a bit-exact mirror).
  The same test fails 25,379 of the 40,000 on `main`.
- *End contacts in hugging stretches (Bezier pipeline):* 6,000 cubics approximating 0.05–0.6 rad of a
  circle (scaled by up to 0.9·tol/r, jiggled by up to 0.5·tol) against the arc extending 0–0.3 rad
  past them: of 11,610 ends within tolerance of the other span with their foot ≥ 0.01 mm from that
  span's ends, 0 have no hit within tol + gap/2 (tangent, or exact where the end lies within
  tolerance of a crossing that represents it); `main` fails 6,124 of them by the same criterion.
- *Near-tangent crossings:* 3,983 line/circle and circle/circle pairs crossing at points c apart:
  c > 1.05·tol gives both crossings (untangent, ≤ 1e-7 mm from the constructed points, residual
  ≤ 1e-9), c < 0.95·tol one tangent hit within tolerance of the chord's midpoint: 0 failures
  (`main` 1,788). Junction re-crossings (line or arc, then an arc folding back over it, 2,000
  contours, both directions): a re-crossing 1.2–20·tol from the junction is the one reported hit,
  untangent, within 2e-7 mm of the constructed point; one 0.05–0.8·tol away gives nothing:
  0 failures (`main` fails all 1,000 of the first kind).
- *`closestPoint(ArcSpan)`:* 21,600 queries 0–9e-10 rad before the start and past the end of
  CW and CCW partial arcs (radii 0.01–100, on, inside and outside the circle): parameter 0 / 1 and
  consistent with the point.
- *Regression guard (measured):* a driver links `main`'s geometry (renamed namespace) and the branch
  and runs both on 95,800 cases per seed, two seeds: random line/line (10,000), line/arc (12,000),
  arc/arc (12,000, a sixth full circles), the same in 0.01–0.1 mm boxes (11,000), the analytic
  end-touch family (6,000), near-tangent line/circle and circle/circle pairs with gaps ±2·tol and
  partial arcs (8,000), random arc/Bezier, Bezier/Bezier and line/Bezier (9,000), jiggled
  near-coincident cubics (800), the hugging family above (6,000), Bezier ends placed near lines,
  arcs and Beziers (3,000), Beziers hugging lines (2,000), touches on convex cubics (6,000),
  Beziers hugging sub-arcs (3,000), random 1–4-span contours (`selfIntersections`, 4,000) and the
  junction re-crossings (3,000). Every result that differs from `main` (and every 50th case) is
  judged by a brute-force oracle: the gap along A sampled densely (40,000 parameters for lines and
  arcs, 20,000 for Beziers, refined geometrically towards both ends), local minima of
  prominence ≥ 1e-9 mm refined by golden section, plus every end of either span within tolerance of
  the other; contours pairwise, minus contacts at shared junctions. Errors: a contact within
  0.999·tol with no hit within `tol.distance` of it, a hit that is not exact with no contact near
  it, a residual beyond tolerance, a missing `tangent` flag at a gap above 1.1e-6·tol or a
  `tangent` flag on an isolated transversal crossing, and an asymmetry. Results: worse than `main`
  0 and 0; different 17,635 and 17,512, of which better 17,588 and 17,470 and equal in errors 47
  and 42 (0 errors on both sides: the branch's hit carries the brute-force gap — the end paired
  with its nearest point, or the true minimum — where `main`'s lay at a nearby pair with a larger
  residual within tolerance, or the difference is an end contact in a band the judge leaves open:
  gap 0.999–1·tol, or within 10 % of the 1e-6·tol exactness threshold). With a looser judge
  (found within tol + gap/2, flags from 1e-5·tol) the equal counts are 464 and 452 and worse is
  still 0. The branch is a bit-exact mirror in every case; `main` is not in 3,393 and 3,444
  (arc/arc). The branch's own errors over all judged cases (none of them worse than `main` in that
  case): pre-existing Bezier-pipeline heuristics — minima separated by ridges of ~1e-9 mm
  reported as the deeper one, or touches in the band where the gap is flat to rounding (up to 9 of
  the 9,000 hugging pairs per seed), and a crossing between branches near a loop flagged tangent
  because another crossing lies within tolerance (8–14 of the 800 jiggled cubics) — plus, for the
  strict judge only, 3–6 ends per seed merged into a hit between tol and tol + gap/2 away (the
  merge rule of item 1).
- *Heuristics that remain:* those of the T-0007 amendment, unchanged. End contacts are judged by the
  end's nearest point only, so two ends within tolerance of each other (an end-to-end contact) are
  one hit, and an end whose nearest point lies on another branch of a looping cubic is paired with
  that branch.
- *Cost (Release -O2, CPU time per pair, best of 3, branch / `main`, on the cases above):*
  pairs involving a Bezier unchanged (median 1.00–1.02×, p99 ≤ 1.20×, worst 1.75×); random
  analytic pairs median 0.37× (line/line), 0.65× (line/arc), 1.02× (arc/arc), p99 2.0–3.0×
  (absolute p99 ≤ 0.9 µs, worst 1.5 µs, against 1.0 µs on `main`), faster where boxes or circles
  are apart, which `main` did not test first; analytic pairs in contact (the end-touch family,
  near-tangent pairs, arcs in 0.1 mm boxes) median 1.6–2.1×, p90 2.9–4.1×, p99 4.0–6.9×, worst
  11× (absolute median 0.37–0.58 µs, worst 1.9 µs; `main` 0.18–0.28 µs): the end contacts cost up
  to four closest-point queries. `selfIntersections` of random 1–4-span contours median 1.02×,
  p90 2.1×, p99 2.4×, worst 4.1× (median 19 µs); two-span junction contours 2.2× (0.7 → 1.4 µs).
  All 95,800 cases: median 1.01×, p90 2.3×, p99 4.0×, worst 11×.
  The independent verifier (own generators, 9,000 pairs) measured exact shared end points at
  median 3.1×, worst 10.7× (absolute median 0.5–0.6 µs, worst 2.1 µs), end touches 2.1× / 7.7×
  and near-tangent pairs 2.1× / 4.5×; all pairs median 1.34×, p99 7.4×.
- *Known limits found in verification:* in `selfIntersections` an end contact of a span's far end
  lying about 1.05–1.4·`tol.distance` from a junction can be merged into a hit within tolerance of
  that junction and erased with it by the junction rule (item 5) — 450 of 6,036 generated
  near-junction contours, the same as `main`; erasing junction hits before merging would fix it.
  A span shorter than the tolerance is handled as a point and its hit is not flagged tangent even
  at a non-zero gap (pre-existing).

### 2026-10-30 — T-0021 (arc/arc crossings without cancellation) · Status: proposed
Change to decision 1 (analytic arc/arc crossings). Everything not mentioned below is unchanged.
1. **Half chord from the smaller circle.** Crossing circles (the `2h > tol.distance` branch of
   T-0008 item 4) are now solved from the circle with the smaller radius S (A when the radii are
   equal, so pairs of equal radii, and every pair with r(A) ≤ r(B) in canonical order, give
   bit-identical results to before): `a = ((rS − rO)(rS + rO) + d²) / 2d` is S's signed distance to
   the radical line, `h = √((rS − a)(rS + a))`, and the crossings are `cS + uS·a ± u⊥·h`. The
   distance `a` carries an absolute rounding error of a few ulps of max(d, r1, r2) whichever circle
   it is measured from; the factored half chord multiplies it by 2·r of the circle it is taken
   from. Taken from the larger circle (as before, whenever A was the larger) the error of h was
   ≈ ulp(rL)·rL/h: for the pair the T-0017 coder found (r = 39785.2 against r = 0.0896, ~4·10⁴ mm
   from the origin, crossing 0.0108 mm apart) both crossings were 1.04·10⁻⁵ mm off, and the one at
   the junction of the offset chain was reported as a separate crossing 1.04·10⁻⁵ mm along the big
   arc. From the smaller circle the error of h is that of the crossing itself (the centre-line
   error over the sine of the crossing angle): 9·10⁻¹¹ mm there, at the junction to rounding.
   The near-tangent rule (when to fold into one tangent hit), the overlap and concentric rules, the
   end contacts (T-0008 item 1) and the canonical order (bit-exact mirrors) are unchanged.
2. **Line/arc needed no change.** The foot-of-perpendicular form (decision 1, T-0008 item 4) is
   already backward stable: the foot is taken from the line, and the half chord's error is that of
   the foot distance times cot of the crossing angle, i.e. no worse than the crossing's own
   conditioning. Measured with the same reference (below), its worst error is 3.1 units.

**Guarantees (measured):**
- *Accuracy against a double-double reference* (crossings recomputed in ~106-bit arithmetic from the
  same double inputs), in units of ε·scale/sin θ, where scale is the largest coordinate or radius
  and θ the crossing angle (the error of a backward-stable solve): arc/arc pairs with radius ratios
  1e-6…1e6 (smaller radius 1e-3…10, larger up to 1e6, a quarter comparable radii, half both large),
  centres translated by up to ±1e4 mm, crossings more than 2·`tol.distance` apart: worst 1.8 over
  100,000 pairs (median 0.39, p99 0.98); `main` median 0.52, p99 1.1·10⁵, worst 1.1·10⁶. Line/arc
  (radius 3e-3…1e6, segment length 1e-6…1e6 times the radius, a fifth of the lines within
  1e-4…1 rad of tangency): worst 3.1 over 76,000 pairs, identical to `main`. The unit tests assert
  ≤ 16 on ~5,900 arc/arc and ~4,600 line/arc pairs (the arc/arc test fails on `main`: 657 of
  5,897 pairs, worst 9.5·10⁵), plus the regression pair (each crossing within 1e-9 mm of the 50-digit value)
  and 3,000 generated junctions where a small arc ends on a large arc's start (one exact hit at
  the junction; `main` fails 176).
- *Regression guard (no result worse than `main`):* a driver links `main`'s geometry (`git archive
  main`, namespace renamed) next to the branch and runs both on 87,000 cases per seed, three
  seeds: random line/line, line/arc, arc/arc in a 20 mm box (34,000; a sixth of the arcs full
  circles) and in 0.01–0.1 mm boxes (11,000), the analytic end-touch family of T-0008 (6,000:
  lines and arcs ending at gaps 0…2·tol from a line or arc, oblique, normal and near-parallel),
  near-tangent line/circle and circle/circle pairs at gaps ±2·tol with partial arcs (8,000), the
  radius-ratio family above with partial arcs (10,000) and its junctions (5,000), random
  arc/Bezier and line/Bezier pairs (3,000), and `selfIntersections` of random 1–4-span line/arc
  contours (4,000), junction re-crossings (3,000) and lens slots closing where a big arc meets a
  small one (3,000). Every result that differs from `main` and every 50th case is judged by a
  brute-force oracle: the gap along the shorter span sampled at 20,000 parameters plus a geometric
  refinement towards both ends (a full circle sampled from two seams), local minima refined by
  golden section, plus every end within tolerance of the other span (exact distances in
  double-double). Errors: a contact within 0.999·tol with no hit within tol + gap/2, a hit further
  than tol from either span or from A(tA)/B(tB), an untangent hit further than 1e-6·tol from
  either span (not an exact crossing), an asymmetry. A differing case is worse when it has more
  errors, or the same and its largest untangent residual exceeds 4× `main`'s + 16·ε·scale.
  Results (seeds 1, 2, 3): worse than `main` 0, 0, 0; different 3,502, 3,499, 3,545 — all in
  arc/arc pairs and contours containing them (lines, line/arc and Bezier pairs are bit-identical);
  of the pairs among them better 1,404, 1,404, 1,484 and equal the rest (the same errors and the
  untangent residual within the 4× margin: bit-level moves of crossings with r(A) > r(B)). Errors in the radius-ratio
  families: branch 0, 0, 2 against `main` 1,213, 1,200, 1,268 (crossings 2·10⁻⁵–10⁻⁴ mm off both
  circles; largest untangent residual branch 2.1·10⁻¹⁰ mm at coordinates up to 10⁶, `main` up to
  1·10⁻⁴ mm). The branch's remaining errors are pre-existing and shared with `main` in the same
  cases: spans shorter than the tolerance handled as points, whose hit is untangent at a non-zero
  gap (T-0008 known limits). Contours: different 143, 132, 140, worse 0, no exact `main` hit lost.
  Symmetry: bit-exact mirrors in every non-degenerate case except arcs on the same circle within
  tolerance whose overlap and points disagree between the orders (0–2 per seed in the
  radius-ratio families: tiny circles r ≈ 1e-3 with d ≤ tol), identically on `main`
  (pre-existing, `finish`/`swapped` of coincident arcs; out of scope).
- *Cost (Release -O2, per pair, best of 3, branch / `main`, seed 1, the 77,000 span pairs):* median
  1.00, p90 1.02, p99 1.14 (arc/arc families median 0.97–1.00, p99 1.07–1.38); the few larger
  ratios (worst 17.8 on one pair) are timer noise on a loaded machine, the change adds a handful of
  selects and no work.
