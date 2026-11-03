# ADR-0006: Clipper2 for offsets and booleans, and its fixed-point scale

Date: 2026-10-24 · Status: accepted (director, 2026-10-24; residual open-offset defect tracked in T-0004;
§6 and §7 added by T-0018, 2026-11-02; §8 added by T-0138, 2026-11-02)

## Decision
1. Polygon offsetting (`geometry/offset.hpp`) and boolean operations (`geometry/boolean.hpp`) are
   implemented on top of **Clipper2 1.4.0** (Boost Software License 1.0, GPL-compatible), fetched
   by CMake `FetchContent` pinned to the `Clipper2_1.4.0` tag. Only the core C++ library is
   built (`CLIPPER2_UTILS/EXAMPLES/TESTS=OFF`, `CLIPPER2_USINGZ=OFF`); its headers are consumed
   as SYSTEM includes so `carvetto_warnings` applies to Carvetto code only, and Clipper2's own
   `-Werror`/`/WX` is stripped from its target so third-party warnings on newer compilers cannot
   break the build (Carvetto targets keep warnings-as-errors).
2. Contours are converted to Clipper2's 64-bit integer paths with an explicit scale,
   `kFixedPointScale = 1e6` (`geometry/fixed_point.hpp`): **one integer unit is one nanometre**.
   The same constant converts offset distances and the arc tolerance, and divides results back.
3. The pipeline is polygonise at `Tolerance::chordal` → scale → Clipper2 → unscale → contours of
   `LineSpan`s. Arc refitting of the polygonal result is *not* done here; it belongs to the
   Fit Curves card and will produce `ArcSpan`s per ADR-0005.
4. Orientation convention for offsets follows Clipper2: the outermost input contour decides which
   way is "outwards"; contours wound the opposite way are holes and move the other way. Boolean
   results come back with outers counter-clockwise (positive area) and holes clockwise.
5. Single-sided offsets of open contours (`offsetOpenContour`) are computed analytically on lines
   and arcs (Beziers polygonised), not through Clipper2, because Clipper2 only outlines open paths
   (both sides plus end caps) and the result must keep the original span structure (arcs stay
   arcs). The method is "raw offset + prune": offset every span, join outside corners, connect
   inside corners, split the raw chain at its self-intersections and at the rest of the
   d-neighbourhood boundary (vertex circles, opposite-side offsets), and keep only pieces at
   distance >= d from the input. Consumed spans and collapsed arcs thereby disappear without
   case analysis, and the distance property holds by construction.
6. **Over-shrunk insets (T-0018): one upstream fix is backported into Clipper2 1.4.0.** At a
   concave join of an offset (a convex corner of the region when it shrinks) `ClipperOffset`
   emits the two offset edge ends and, between them, the original vertex, so that wherever the
   offset passes "through" the region (an inset past the local inradius) the raw path loops back
   with negative winding and the final Positive-fill union removes it. Version 1.4.0 skips that
   vertex when the join turns by less than about 8 degrees (`cos_a < 0.99`, issue #405). A
   polygonised curve turns by a few degrees per vertex, so an over-shrunk circle became its
   point reflection (orientation preserved) and survived the union: a 17 mm circle inset by
   19.2 mm came back as a 2.2 mm circle, by 30 mm as a 13 mm one (a 16-gon, turning by 22.5
   degrees per vertex, was already correct). Upstream later made the vertex unconditional
   (issues #873, #916; current Clipper2 emits it at every concave join). `core/CMakeLists.txt`
   applies exactly that one-line change to the fetched `clipper.offset.cpp` at configure time
   (idempotent; configuration fails loudly if the source no longer matches 1.4.0). An upgrade
   to a Clipper2 version with the fix drops the patch step.
   - Patching the dependency was chosen over working around it in `offsetContours`: the only
     sound workaround is to subtract the boundary's d-neighbourhood from the result, which costs
     a second boolean on every inset and leaves slivers along every true edge; filtering result
     regions by depth (as Pocket's `insetBy` does) is not sound in general, because a phantom
     can touch a genuine region and be merged with it by the union.
   - The extra vertex lies within d of the boundary, so it never reaches a genuine result
     region; it only changes which edge pairs the union intersects. On valid insets the result
     is the same region, but a vertex where two nearly collinear offset edges meet can move
     along that near-straight boundary by up to about 0.4 µm (4 of 900 random raw insets in a
     direct comparison), and toolpaths built on such regions can make different downstream
     choices (Pocket rings and rest areas, Profile's merged-region cuts). No golden file
     changed, and verifier oracles (gouge, coverage, engagement, retracts) report the same
     results before and after. Profile no longer reports "part of the cut was left out"
     (`TrimmedNearVector`) for merged-region phantoms its clearance guard used to trim away
     whole; with the guard off those phantoms were cut.
   - Contract (now tested generatively against a brute-force inradius/distance oracle,
     `offset_inradius_test.cpp`): every point of an inset by d lies inside the region at
     distance >= d (less the polygonisation and join tolerance, 2·chordal) from its boundary,
     so an inset by d >= inradius yields nothing; every lobe deep enough survives; an output
     contour wound against the outermost input is a genuine hole (nested at odd depth).
7. **Open paths lying on a clip boundary are not decided by geometry.** `booleanOp` clips open
   subject paths with Clipper2. A stretch of an open path that runs exactly along an edge of
   the clip polygon (collinear and overlapping on the integer grid) is returned by the
   Intersection, by the Difference, or by both, depending on where the edge lies in Clipper2's
   sweep and on the path's direction, not on a geometric rule (Clipper2 1.4.0 drops the bottom
   side of a CCW square from the Intersection but keeps its top side, and returns its right
   side from both). In the cases tested it was never dropped from both. This is documented
   rather than fixed: deciding it would mean re-classifying every piece against the clip
   boundary after clipping, and no Carvetto caller relies on it: Pocket's rest tools no longer
   clip paths against a region's own boundary (ADR-0017 §10, since T-0123), and its raster lines
   (at half-pitch positions) meet the case only where one runs exactly along a parallel edge of
   the centre region; that stretch lies on the region's boundary, where the tool centre may go,
   so keeping or dropping it is safe either way. **Callers that need a decided
   side grow or shrink the clip by a tolerance first** (e.g. `offsetContours(clip,
   +tol.distance)` keeps boundary stretches in the Intersection). Pinned by the AC3 test in
   `offset_inradius_test.cpp`.

8. **Grid-level dust is dropped from `offsetContours` results (T-0138).** Within a few
   nanometres of collapse (an inset by about the region's inradius) Clipper2 1.4.0 can return
   rings a few grid units wide: genuine regions shrunk to the rounding level (e.g. the lenses at
   the outward bulges of a Bezier circle, inradius 1–2.5 units over 3–7 µm in the card's repro)
   and debris of the
   rounded intersections (3 vertices, 0.1–0.3 units wide, |area| ~1e-11 mm²), some of which is
   wound against its nesting or nested inside a genuine ring of the same orientation. That broke
   the contract sentence of §6 ("an output contour wound against the outermost input is a
   genuine hole"). After `Execute`, every ring into which no disk of radius ρ fits, as judged
   by the erosion test below, is removed, with every ring nested inside a removed one, directly
   or through other rings (normally narrower still; removed explicitly so the inexactness of the
   test can never leave a hole without its outer). The erosion is Clipper2's `InflatePaths`,
   which is not exact on acute rings and not even monotone in the radius (an acute triangle
   eroded by 4.0 units can vanish while by 4.5 units it leaves a ring; e.g. the triangle
   (0.019917, 0.098387), (0.024209, 0.096417), (0.024216, 0.096426) mm, exact inradius 5.54
   units, is removed). Over 419,000 random thin (~4 µm) integer triangles removed rings had an
   inradius of up to 5.64 units and kept ones down to 3.62 units, so ρ is a nominal radius
   with that band around it:
   - ρ = 4 grid units (4e-6 mm) × max(1, 16 ε M), M the largest output coordinate in grid
     units, ε the double epsilon. Clipper2 rounds every vertex to the grid (≤ 0.71 units in the
     plane) and computes offset and intersection points in double from int64 coordinates, a few
     ulps of M; that exceeds one unit only beyond M ≈ 2.8e14 units (2.8e8 mm), where ρ grows with it.
     In the T-0018 verifier's oracle (about 37,000 insets around and past collapse over 40 seeds)
     the rings wound against their nesting were 0.1–0.3 units wide and the rounding-level lenses
     up to 2.5 units (inradius); 4 units leaves a margin above both. Rounding-level lenses are
     not bounded by that, though: near-collapse Pocket annuli keep lens triangles ~4 µm long
     with inradius 3.7–4.7 units on either side of ρ (as before T-0138; each still gives one
     plunge). A removed ring is narrower than about 11.3e-6 mm everywhere (twice the largest
     removed inradius): about 8.9× below the smallest chordal tolerance Pocket accepts (1e-4
     mm), and so inside the contract's own slack there (result points may lie 2·chordal short
     of |distance|). Profile does not validate its tolerance: it offsets at the caller's
     tol.chordal (at most 0.5·tol.distance with Bezier input), so only a chordal tolerance near
     1e-5 mm or below brings the removed width up to that slack. tol.distance has no lower
     bound, so no claim is made against it. Every point of a removed region lies within about
     5.6 nm of its boundary, i.e. of where the offset collapses, which no toolpath resolves;
     the threshold does not depend on `tol` so that a coarse
     tolerance never drops genuine thin regions (an area threshold such as tol.distance² would
     drop a 0.5 µm tool-centre disk, a pocket corner the tool just fits, and a long strip of
     any width below tol.distance²/length).
   - Test: a ring of inradius r satisfies |area| ≤ r·perimeter (every point lies within r of
     its boundary), so only rings with |area| < ρ·perimeter are candidates; for those, the ring
     alone is eroded by ρ (Clipper2 `InflatePaths`, round joins) and dropped if nothing is left.
     A mean-width test alone (2|area|/perimeter) would also drop a genuine region with a long
     hair-thin tail. Candidates are rare in practice, but not bounded: the nesting pass tests
     each removed ring against the rings whose bounding box starts within its x-range (rings
     sorted by left edge), so it is O(dust × rings) only when many removed rings span the
     same x-range as many others (adversarial input, e.g. thousands of thin rings in one
     column). The pass removes the whole closure (rings nested in a removed ring, then rings
     nested in those, in any order); for crossing rings that closure could differ from a single
     ordered pass, but Clipper2's union output never holds crossing rings, so it cannot differ
     in practice. The pass is `detail::markNestedInDust` (`offset_dust.hpp`), tested against a
     brute-force closure.
   - Per ring, not per region: a frame narrow only between an outer and its hole is two
     correctly wound genuine rings and is kept; a dust-thin hole inside a genuine region is
     dropped (filled).

## Rationale
- Clipper2 is the de-facto robust polygon clipper for CAM; it is deterministic, header-light,
  integer-exact in its sweep, and licence-compatible. Writing our own Vatti/Greiner–Hormann
  engine would cost months and still be less robust.
- **Why 1e6:** Clipper2 is exact on integers, so all approximation happens in the conversion.
  Rounding to the grid moves a vertex by at most 0.5e-6 mm (√2/2 of that in the plane), three
  orders of magnitude below the default distance tolerance (0.001 mm) and four below the default
  chordal tolerance (0.01 mm), so the polygonisation error dominates and the grid is invisible
  to callers. Clipper2's safe coordinate range is about ±2.3e18 integer units (`INT64_MAX / 4`),
  i.e. ±2.3e12 mm, so no realistic job can overflow. Intersection points inside Clipper2 are
  computed in `double` from int64 coordinates; with 1e6 scale a 1 m job has coordinates ~1e9,
  whose products (~1e18) are still exactly representable where Clipper2 uses 128-bit products
  and lose at most ~1e-7 relative where it uses doubles, i.e. far below one grid unit. A finer
  scale (1e8, Clipper2's own `PathsD` maximum) would gain nothing visible and shrink the range to
  ±2.3e10 mm while raising the chance of double-precision loss in intersections.
- Keeping the constant public and explicit lets tests pin the rounding behaviour and lets future
  code (e.g. snapping, file I/O quantisation) reason about it instead of discovering it.

## Consequences
- Results of `offsetContours`/`booleanOp` are polygonal; callers needing arcs run Fit Curves
  afterwards. Area of a polygonised circle is systematically low by ≈ 2/3 · sag · perimeter,
  which tests account for by choosing the chordal tolerance.
- Features smaller than 1e-6 mm collapse to a point; that is intended (they are noise). Offset
  result rings narrower than 8e-6 mm are dropped (§8).
- `Tolerance::chordal` controls both the input polygonisation and Clipper2's round-join arc
  approximation, so one knob governs the whole error budget: round-trip offset(+d) then
  offset(−d) stays within 2·chordal of the original (acceptance criterion 4).
- Upgrading Clipper2 means bumping the tag in `core/CMakeLists.txt` and re-running the tests; the
  interop layer (`core/src/geometry/clipper_interop.*`) is the only code that includes its headers.
