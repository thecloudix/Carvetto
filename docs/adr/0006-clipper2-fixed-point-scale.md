# ADR-0006: Clipper2 for offsets and booleans, and its fixed-point scale

Date: 2026-10-24 · Status: accepted (director, 2026-10-24; residual open-offset defect tracked in T-0004)

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
