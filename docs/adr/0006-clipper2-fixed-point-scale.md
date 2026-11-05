# ADR-0006: Clipper2 for offsets and booleans, and its fixed-point scale

Date: 2026-10-24 · Status: accepted (director, 2026-10-24; residual open-offset defect tracked in T-0004;
§6 and §7 added by T-0018, 2026-11-02; §8 added by T-0138, 2026-11-02; §9 and the §8 dust-test
decision added by T-0141, 2026-11-02; §10 added by T-0142, 2026-11-05)

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
   units, is removed). Measured over about 6 million random thin (~4 µm) integer triangles,
   removed rings had an inradius of up to about 5.7 units and kept ones down to about 3.6
   units (a measurement, not a bound), so ρ is a nominal radius with that band around it:
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
     of |distance|). Profile enforces the same tolerance bounds since T-0141 (ADR-0016 §18), but
     with Bezier input it offsets at min(tol.chordal, 0.5·tol.distance), and tol.distance is only
     bounded below by 1e-6 mm (T-0142), so a distance tolerance between 1e-6 and about 1.1e-5 mm
     brings the removed width up to that slack. No claim is made against tol.distance itself. Every point of a removed region lies within about
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
   - **No monotone dust test (T-0141, recommendation to the director).** The erosion is not
     monotone in the radius, so the cut between removed and kept rings is a band (inradius
     about 3.6–5.7 grid units) rather than a line. An exact, monotone test would need each
     candidate ring's true inradius (the largest inscribed disk: a segment Voronoi diagram or
     medial axis of the ring), i.e. new geometry code, for no observable gain: the whole band
     lies below 5.7e-6 mm of inradius (1.13e-5 mm of width), 8.9× below the smallest chordal
     tolerance Pocket and now Profile accept, no caller depends on where in the band a ring
     falls, and the guarantees that matter (every kept ring holds a disk about 7e-6 mm
     across and is wound for its nesting depth; nothing wider than the band is removed) hold
     either way. Simpler monotone substitutes are not sound: a mean-width test (2|area| /
     perimeter) removes genuine regions with long hair-thin tails, an area threshold removes
     long thin strips. So the band stays, documented and pinned by the acute-triangle test in
     `offset_dust_test.cpp`; revisit only if a caller needs tolerances near 1e-5 mm.
   - The nesting pass scans, per removed ring, whichever is smaller: the rings whose left edge
     lies in its x-range, or those whose top edge lies in its y-range (T-0141), so a column
     of thousands of rings (one shared x-range) no longer costs O(dust × rings). Same result:
     the closure does not depend on the order pairs are asked in.

9. **Far-apart closed contours are offset in separate Clipper2 calls when many sit side by side
   (T-0141).** Clipper2's union sweep keeps every edge crossing the current scanline in a linked
   active edge list and walks the whole list on every scanbeam (`DoTopOfScanbeam`,
   `BuildIntersectList`); inserting a local minimum scans the list from its head
   (`InsertLeftEdge`), and finding a maxima pair that is horizontal walks to its end
   (`GetMaximaPair`). With n shapes side by side the list holds ~n edges for ~n scanbeams:
   O(n²). A row of 20,000 small rectangles took 36–50 s in Release (profile: 53 %
   `GetMaximaPair`, 30 % `InsertLeftEdge`; with varying heights the per-scanbeam walks
   dominate), with or without the §8 filter. Patching those two functions would only cure
   equal heights; the per-scanbeam walks are the sweep itself.
   - `offsetContours` grows every closed path's bounding box by reach × |distance| + one grid
     unit, where reach is 1 for round joins (offset edges and arcs lie at |distance|, the
     original vertex of a concave join on the path), √2 for square joins and max(√2,
     mitreLimit) for Sharp ones (square beyond the limit); every point Clipper2 emits for the
     path lies in that box. Recursive cuts along x and y at coordinates no grown box covers
     split the paths into batches (`detail::separableBatches`, `offset_batch.hpp`), so raw
     offsets of different batches never meet and each batch's union is the global union
     restricted to it. Clipper2 offsets a whole group the way its lowest path (largest y) is
     wound (§4): a batch whose own lowest path is wound against the global lowest path joins
     that path's batch, which then decides for it exactly as one call would.
   - Batching applies only when at least `kOffsetBatchMinDepth` = 32 batches overlap one
     horizontal line (all of them would share the active edge list) and no open contour is
     being outlined; otherwise everything goes into one Clipper2 call as before, and the result
     is byte-identical to the result before T-0141. Below that threshold one call holds at most
     ~32 batches side by side, which is linear in n only while each batch is small (see the
     remaining quadratic cases below). Batches are offset in input order of their
     first contour and their results concatenated (each in Clipper2's order); the §8 filter
     then runs over the concatenation, with the same decisions (ρ depends on the largest
     coordinate overall; nesting never crosses batches).
   - **Not bit-identical to one call.** Clipper2's result for a shape depends at the grid level
     on unrelated shapes at the same height: it rounds intersection points within scanbeams and
     clamps them to the scanbeam, and every vertex height of every shape subdivides the
     scanbeams. A batch's rings can therefore differ from one call's by a few grid units,
     along nearly parallel edges. Over about 18,000 random scenes (rectangles, circles,
     triangles, slivers, holes, mixed orientation, all join types, insets near collapse,
     outsets, zero distance) the ring count was always the same, 5.7 % of the cases differed
     at all, and the largest Hausdorff distance between the two results was 11 grid units
     (1.1e-5 mm), typically 1–2. The ring order also differs (batch by batch instead of
     Clipper2's sweep order). Profile offsets region by region, so it does not reach the
     threshold in practice; Pocket offsets its whole centre region at once, so a pocket over 32
     or more shapes side by side is batched.
   - **Downstream the drift is larger than the rings' few grid units, but safety is unchanged.**
     Pocket builds rings, rest areas, rasters and links on these regions, and a grid-level
     change can tip its discrete choices. On wide pockets (rows of 32–70 rooms, T-0141
     verifier) the safety statistics (gouge, coverage, engagement) were identical to before
     T-0141 in 50 of 50 scenes, but the move hashes differed in 41 and the move counts in 32:
     ring vertices moved by up to ~5e-3 mm (different vertices picked along arcs, within the
     0.01 mm chordal tolerance), one rest-tool raster line end slid 0.021 mm, ring start
     points and link routes changed, and retract counts changed in 2 scenes (2230 → 2235,
     1650 → 1653). Toolpaths of such jobs are therefore not reproducible against releases
     before T-0141, only equally safe.
   - Measured (Release, `offsetContours` on the verifier's rows at -(0.1 − 3e-6) mm, CPU time,
     best of 7): 20,000 rectangles in a row 36–50 s → 0.06 s (2,500 → 20,000: 2.0–2.3× per
     doubling; before: 3.8–6.6×); the same with heights jittered (many scanbeams) about the
     same; a column of the same rectangles (half of them dust) 0.28–0.33 s → 0.08 s.
   - Rejected: a sentinel path per batch to force the global orientation (more machinery than
     merging, with its own output to filter); merging has one adversarial cost: thousands of
     lone holes side by side (wound against the lowest contour, which no Carvetto caller
     produces) stay in one call and stay quadratic. `booleanOp` (`unionContours`) has the same
     sweep cost (a 10,000-rectangle row: ~1.3 s); it is batched the same way since T-0142 (§10).
   - **Remaining quadratic cases (T-0142).** Batching only helps when the shapes side by side
     are in different batches. One batch holding many edges side by side is still one sweep,
     ~4.4× per doubling and unchanged from before T-0141: an outer containing a row of n holes
     (1.31 s / 6.10 s at 10,000 / 20,000) and a row whose outsets merge into one region
     (1.44 s / 5.99 s). Nested arrangements such as a spiral of nested corners stay ~4.5× per
     doubling because `separableBatches` peels off one box per cut level (still about 5×
     faster than before T-0141).

10. **Booleans are batched like offsets, raster clips are cropped to their strip, and holes with
   nothing around them are resolved before an outset (T-0142).**
   - `booleanOp` partitions all its paths (closed subjects, open subjects and clips together) with
     `separableBatches` on their bounding boxes and, when at least `kOffsetBatchMinDepth` = 32
     batches overlap one horizontal line, clips each batch in its own Clipper2 call, results
     concatenated batch by batch (a batch of clips alone only for Union and Xor). Boxes of
     different batches neither overlap nor touch, so no edges meet and every winding number comes
     from one batch: the regions are the same for every operation and fill rule (no grown boxes
     and no orientation rule are needed, unlike §9). Vertices can differ from one call by a few
     grid units, as in §9 (random rows of 32–62 shapes, all operations, both fill rules: ring and
     piece counts and areas equal, Hausdorff ≤ 2e-5 mm by test), and the order differs. Below the
     threshold the result is that of one call, byte for byte. Measured (Release, CPU): Pocket with
     a rest tool on the T-0131 text page 1.47 / 2.89 / 5.73 / 11.7 s at 1,250 / 2,500 / 5,000 /
     10,000 vectors (~2.0× per doubling; before: 1.85 / 4.07 / 10.1 / 25.8 s; the whole-page
     union, rest-material difference and intersection were single sweeps). Pocket output on such
     pages changes at the grid level (move hashes differ, retract counts by a few); the T-0142
     safety harness (wall clearance of every cutting move, unswept reachable cells on a 0.05 mm
     grid) gave identical numbers to before on every changed scene.
   - `OpenPathClipper` clips batches of open paths (Pocket's raster lines, 256 per call) against
     one clip polygonised once. When the clip has at least 1,024 edges, each call keeps only the
     clip edges whose extent across the raster comes within 2 grid units of the strip the batch
     spans (`detail::StripIndex`, a segment tree over the edges' extents); every run of other
     edges lies wholly on one side of the strip and is replaced by the straight segment between
     its ends, which stays on that side (a half-plane is convex). Winding numbers inside the strip
     are therefore unchanged and the pieces are the same. Their ends are computed by Clipper2 from
     the same edges but in different scanbeams, so an end where the subject crosses a clip edge
     at a near-parallel angle can move by tens of grid units, and there the whole-clip call is the
     less accurate one (round-1 verifier: a cropped end 0.7 units from the exact intersection,
     the whole-clip end 64 units off); a stretch lying exactly along a clip edge is decided as in
     §7. Over ~2,500 round-1 verifier calls the well-conditioned ends agreed within 2 units; the
     round-2 verifier saw ends 3 units apart where the subject crosses a clip edge at a sine of
     0.048, and there the cropped end was the accurate one (0.77 units from the exact
     intersection, the whole-clip end 3.77). Comb pocket hashes at 0/30/45/90 degrees and 5–2,000 teeth equal those before. A 90-degree raster on a comb of n teeth swept the whole comb for every batch:
     Pocket 0.78 → 0.23 s at 1,000 teeth, 5.99 → 0.73 s at 2,000.
   - Before an outset (delta ≥ 0.5 grid units), if some ring wound against the lowest path is not
     nested in a ring wound like it (a ring whose box holds its box and whose inside holds its
     first vertex), the closed paths are first replaced by their Clipper2 union with the group's
     fill rule (Positive, or Negative when the lowest path is clockwise). Cause: §6's extra vertex
     at concave joins leaves a loop of positive winding at every corner of a shrinking hole;
     inside an enclosing ring the loops merge into its region, with nothing around them they
     survived the final union (a lone clockwise 1 mm circle under a counter-clockwise lowest
     contour: 71 leftover rings besides the genuine one at +0.3 mm, chordal 0.001; 223 at chordal
     0.0001), and an
     over-shrunk one left a whole phantom region (+1.5 mm: a 1.57 mm² ring). Such a ring describes
     no region (its winding is not positive), so the union, which has only nested holes, describes
     the same region. The nesting test (`hole_nesting.hpp`) first tries the last ring wound like
     the lowest before the hole (input usually lists a shape's holes right after it), and
     otherwise takes candidate rings from a bounding-volume hierarchy over the boxes of the rings
     wound like the lowest, smallest box first. A ring of 48 or more vertices that is tested more
     than once gets an index of its edges by y extent; a test then visits only the edges reaching
     the vertex's y, in the order Clipper2's `PointInPolygon` walks them and with its arithmetic,
     so the result is Clipper2's own (`hole_nesting_test.cpp`: generative checks against
     `PointInPolygon`, with points on edges and vertices, vertices and horizontal edges on the
     test line and coordinates where the cross product rounds, and against the scan over all
     pairs). The decision is exactly that of the scan over all pairs. The cost is near-linear in
     the number of rings whatever the layout of their boxes (apart, touching or overlapping):
     O(log n) per hole for the hierarchy, plus one test per candidate tried, each costing the
     edges its test line crosses. Outsets of 40,000 → 80,000 holed squares (Release, CPU, harness
     t0142v2/po2.cpp, 3 interleaved runs) take 0.26 → 0.57 s in a column of touching squares
     (main 0.24 → 0.57), 0.64 → 1.30 s in the notches of a comb ring (main 0.68 → 1.38), 0.29 →
     0.68 s with a hole in each tooth of a comb (main 0.37 → 0.75), and the same as main within
     measurement noise (under 7 %) for rows, grids, jittered grids, nests 8 deep, a touching
     grid and a hex packing, with byte-identical output in every layout. Round 1 scanned all
     rings per hole (80,000 holed rectangles 9.9 s vs 0.8 s on main), round 2 the rings left of
     each hole within a `separableBatches` batch (the column 2.7 → 21.7 s, the comb 17.4 → 88.1
     s). Pinned by "T-0142 round 3: outsets of touching shapes ..." (column, comb notches, comb
     teeth; ≤ 2.5× per doubling). Input built to defeat it still costs more than linear: a hole
     inside many nested rings wound like the lowest whose boxes hold it but whose insides do not
     (each is tried), or a big ring whose edges a horizontal line crosses many times (holes in
     the teeth of a comb whose teeth run along y: each test crosses every tooth). Rings that cross each other are not detected by the nesting test. Pinned by
     the repro and a generative oracle (`offset_unnested_hole_test.cpp`: nested and lone rings of
     both orientations, all joins; points within d of the region are in the result, points
     farther than reach·d are not, every outer result ring lies within reach·d), which reports
     1,764 violations before T-0142 and none after. Input without such rings is offset as before.
   - **Remaining quadratic cases, documented (director, 2026-11-05).** Batching only separates
     shapes whose bounding boxes are apart; one group whose own edges sit side by side is one
     Clipper2 sweep (`InsertLeftEdge` scans the active edge list from its head, `GetMaximaPair`
     walks it to the end at every horizontal maximum). Measured before and after T-0142 alike
     (Release, `offsetContours`, harness t0141v1/perf.cpp, 5,000 / 10,000 / 20,000 shapes): an outer
     containing a row of holes 0.33 / 1.28 / 6.1 s, a row whose outsets merge into one region
     0.36 / 1.39 / 6.05 s, the nested-corner spiral 0.49 / 2.23 / 9.75 s (`separableBatches` peels
     one bar per cut level). The same holds for one ring with thousands of teeth (the comb outline
     itself: its inset ~4.4× per doubling, 0.04 s at 1,000 teeth). Curing the first and the spiral
     would need edge-level components with a base winding each and a forced orientation per call;
     the merged row and the comb produce one region whose boundary has n edges side by side, which
     any single Clipper2 sweep pays for. Shapes whose outsets touch or overlap in two directions
     (a touching grid or a hex packing of holed squares) are one region with about √n edges on
     every sweep line: ~2.9–3.0× per doubling on main and after T-0142 alike (40,000 → 80,000:
     touching grid 1.26 → 3.81 s, hex packing 1.08 → 3.32 s on main, the same after T-0142). Not done: no Carvetto caller meets these at a size where
     they matter (rows of more than ~10,000 side by side), and Pocket's work cap on the polygonised
     region (200,000 vertices) bounds the cost there.

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
