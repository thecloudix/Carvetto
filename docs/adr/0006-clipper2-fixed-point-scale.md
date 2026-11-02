# ADR-0006: Clipper2 for offsets and booleans, and its fixed-point scale

Date: 2026-10-24 · Status: accepted (director, 2026-10-24; residual open-offset defect tracked in T-0004;
§6 and §7 added by T-0018, 2026-11-02)

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
- Features smaller than 1e-6 mm collapse to a point; that is intended (they are noise).
- `Tolerance::chordal` controls both the input polygonisation and Clipper2's round-join arc
  approximation, so one knob governs the whole error budget: round-trip offset(+d) then
  offset(−d) stays within 2·chordal of the original (acceptance criterion 4).
- Upgrading Clipper2 means bumping the tag in `core/CMakeLists.txt` and re-running the tests; the
  interop layer (`core/src/geometry/clipper_interop.*`) is the only code that includes its headers.
