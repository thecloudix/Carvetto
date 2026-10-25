# ADR-0009: Intersection results and point-in-contour conventions

Date: 2026-10-24 · Status: accepted (director, 2026-10-24); amendments of 2026-10-24 (T-0005) proposed, residuals tracked in T-0007

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
       line/arc and arc/arc rules treat an infinite line or
       circle within `tol.distance` of touching as one tangent hit, so two genuine crossings up
       to about 2·√(2·r·tol) apart fold into one tangent hit (≈ 0.09 mm for r = 1 mm, 0.9 mm for
       r = 100 mm); same-circle partial arcs that touch end to end report a point whose residual
       to the arc ends can reach 1.5·`tol.distance` (the midpoint rule in the angular frame).
     - *Cost:* in a Debug build a pair of nearly identical 4 mm cubics takes about 4 ms, a
       Bezier hugging an arc about 1 ms, typical crossing pairs well under 1 ms.

### 2026-10-25 — T-0007 (residuals of T-0005 round 3) · Status: proposed
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
6. **Span ends.** Each end of either span is paired with its nearest point on the other span
   (global `closestPoint`) and, within 1.5·`tol.distance`, settled as a touch candidate. An end
   touch is a minimum of the distance along one span only — along the other the distance may keep
   falling towards a nearby crossing — so neither the scan (which runs along B) nor the walk
   (along A) reliably reached the end of A; seeding the ends directly makes end touches symmetric
   (T-0005 residual: an arc whose end came within tolerance of a Bezier's interior was missed).
7. **Self-intersection of one cubic (decision 4).** The double point is computed analytically:
   with B(t) = a t³ + b t² + c t + d, a double point s ≠ t satisfies a(u² − v) + b u + c = 0 for
   u = s + t, v = s t, so u = −(a × c)/(a × b), v follows by projection on a, and s, t are the
   roots of x² − u x + v = 0 (a cubic has at most one). It is refined by Newton and reported when
   both roots lie in [0, 1], the residual is within `tol.distance` and the loop is larger than
   the tolerance (the distance from the double point to B((s + t)/2) exceeds `tol.distance`; a
   smaller loop is a cusp within tolerance). Thin sliver loops (0.05–0.3 mm long, 0.3–5 µm thick)
   whose chords never cross at the chordal sampling are therefore found. The chord-crossing
   search remains as a fallback.

**Guarantees, replacing the corresponding T-0005 bullets:**
- *Touches:* a touch with gap 0 ≤ h ≤ `tol.distance` is reported exactly once, flagged `tangent`,
  and h = 1.05·tol gives nothing, against convex cubics at every position: external circles and
  arcs of radius 0.2 and 1 mm (including arcs around the touch and circles with their seam at the
  touch point) in a fine sweep t = i/1000 on four convex cubics × h ∈ {1e-5, 1e-4, 0.99·tol}
  (0 drops; the T-0005 code dropped 622 of 48,048), and tangent lines, internal circles and
  mirrored cubics at 199 offset positions per cubic × h ∈ {0, 1e-5, 1e-4, 5e-4, 0.99·tol}.
- *End touches:* symmetric. An end of either span within `tol.distance` of the other's interior
  gives one tangent hit, whichever span is A and whichever span's end it is (generative test with
  lines, arcs and Beziers ending at h ∈ {1e-5, 1e-4, 5e-4, 0.99·tol, 1.05·tol} from convex cubics
  and arcs, ~9,000 placements in both orders: 0 failures).
- *Self-intersections:* every loop of a single cubic larger than the tolerance is reported once;
  a crossing of a Bezier with the span it joins, more than `tol.distance` from the junction, is
  reported however shallow the bulge before it (re-crossings 2–10·tol from the junction with
  bulges of 0.1–1·tol: 0 misses).
- *Regression guard:* over ~43k reference pairs and contours (random arc/Bezier, line/Bezier and
  Bezier/Bezier pairs, hugging and jiggled near-coincident pairs, the touch families above,
  end-touch placements, random 1–2 span contours) every result is unchanged except: touches
  previously dropped (now found, checked against a dense brute-force gap profile), spurious
  touches at a full circle's seam (removed), end touches previously missed (now found), one
  previously missed crossing next to a span end, three small loops (10–25 µm) previously missed,
  and positions of touches inside the band where the gap is flat to rounding (moved ≤ 5e-5 mm
  with the same gap; exact tangencies by ≤ 1e-6 mm). Crossings are unchanged to 1e-10 mm.
  Symmetry is exact as before.
- *Heuristics that remain:* two local minima of the gap separated by a ridge of the order of
  1e-9 mm (curves parallel to rounding over a stretch) may be reported as the deeper one only,
  since the walk's growing steps cross such a ridge; the 1/64 walk step cap, the 1e-5 validation step and the trail rule
  (item 5) are scale assumptions in parameter space; a double crossing whose two roots lie within
  one eighth of a chord, away from a sampled dip or chord end, can still be reported as one
  tangent touch or missed. Analytic line/arc and arc/arc end contacts keep their own rules
  (a crossing just beyond an end within the slack is not `tangent`). Bezier overlaps are still
  reported as touches/crossings, not `Overlap`s.
- *Cost:* Release, per pair: unchanged for generic crossing pairs; touch-heavy and
  near-coincident pairs about 1.5–2× slower than after T-0005 (more candidates are settled), e.g.
  ~1.8 ms for a pair of nearly identical 4 mm cubics.
