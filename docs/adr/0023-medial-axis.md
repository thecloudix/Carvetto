# ADR-0023: Medial axis of 2D regions

Date: 2026-11-02 · Status: proposed (T-0152)

## Context
T-0152 needs the inner medial axis (skeleton) of closed regions with holes, bounded by lines,
arcs and cubic Beziers, with the inscribed-circle radius along it, as the geometry core for
V-carving (T-0153: depth = radius / tan(half angle)). The card leaves the algorithm and the
dependency open and requires the owner's decision for any new dependency. Requirements: radius
error ≤ tol.chordal against a brute-force distance oracle, skeleton points equidistant (within
the tolerance) from two boundary features, robustness on degenerate input, and CPU time growing
by at most 2.5× per doubling of boundary segments up to 50,000 (text-heavy jobs).

## Options considered
- **Boost.Polygon Voronoi** (BSL-1.0, header-only, robust segment Voronoi by sweep line): a new
  dependency (owner decision), integer input only (lines; arcs and Beziers polygonised, which uses
  up the chordal budget before any medial-axis error), and pulls in part of Boost.
- **CGAL** segment Delaunay graph / straight skeleton: GPL/commercial dual licence and a very
  heavy dependency; rejected by the director up front.
- **OpenVoronoi** (incremental, points/lines/arcs): LGPL-2.1, unmaintained, not robust on
  degenerate input; rejected.
- **Exact segment Voronoi of our own** (topology-oriented incremental or sweep with arcs as
  sites): months of work to make robust, and arcs/Beziers still need special bisectors.
- **Own implementation on a refined Delaunay triangulation of boundary samples** (chosen):
  point-sample Voronoi diagrams converge to the medial axis as the sampling gets denser, and
  every quantity can be measured against the exact boundary, so the sampling can be refined
  until the result is certified within the tolerance. Needs only robust Delaunay predicates.

## Decision
1. **No new dependency.** `geometry/medial_axis.hpp` (`medialAxis(contours, tol, options)`) is an
   own implementation. Internal parts: `robust_predicates.*` (exact `orient2d` / `incircle`: a
   double-precision filter with a forward error bound, falling back to floating-point
   expansion arithmetic, after the method published by J. R. Shewchuk, 1997; written from the
   paper, no code copied; TwoProduct uses std::fma, so exactness does not depend on
   floating-point contraction; points are scaled by a power of two before exact evaluation;
   exact when the largest coordinate magnitude M is below ~1e70 and every non-zero coordinate
   and coordinate difference is at least ~1e-70·M (one tiny non-zero coordinate, ~1e-150·M or
   less, breaks it even with large differences); IEEE doubles with 53-bit rounding are assumed,
   i.e. SSE2 or x87 in double-precision mode, since x87 extended precision can double-round in
   the error-free sums), `delaunay.*` (incremental Bowyer-Watson triangulation inside a super
   triangle, visibility-walk location, exact predicates, duplicate points return the existing
   vertex), `medial_axis.cpp`.
2. **Region.** Closed contours under a fill rule (`MedialAxisOptions::fillRule`, default
   even-odd like Pocket). Each closed, finite, non-zero-area contour whose two sides differ under
   the fill rule is a boundary, with its inside side recorded; the rest are counted in
   `ignoredContours`. The sides come from winding numbers at a probe point on the contour that is
   clear of the other contours; when every probe lies on another contour (duplicated or
   coincident contours), from all contours just off both sides of it, so duplicates cancel
   under even-odd. Windings use each contour polygonised once at the chordal tolerance, in a
   row-bucketed edge index. Crossing contours are not rejected: every contour then still acts
   as a wall.
3. **Pieces and exact distance.** Contours are cut into pieces (lines; arcs of ≤ 45°; Bezier
   sub-curves by turning angle). Exact nearest-boundary queries (closest point on the true
   line/arc/Bezier) use a tree: each contour is cut into nearly straight chunks (≤ 32 pieces,
   within a quarter of their chord), each chunk is a balanced tree in boundary order whose nodes
   carry a capsule bound (chord plus the chain's largest distance from it), and a spatial median
   tree over the chunks sits on top; every node also has an 8-direction bound (x, y and the
   diagonals). Capsules keep queries logarithmic along densely polygonised smooth curves, where
   boxes alone leave every nearly equidistant piece unpruned (O(√n) per query, O(n) at the
   centre of a polygonal circle); chunking and the diagonal bounds keep zig-zag and spiky
   outlines from defeating the capsules. Bezier pieces use the same global root isolation as
   `closestPoint(BezierSpan)` (Bernstein sign changes of the stationary quintic) with each
   isolated root refined by bracketed Newton steps instead of bisection. Circumcentres closer
   than the merge distance share one query (co-circular samples). Inside/outside of a point
   comes from the side of its nearest piece (half-planes at piece ends); where coincident pieces
   disagree (two regions sharing an edge, a hole lying on the outer boundary), from the fill
   rule's winding number.
4. **Sampling and refinement.** Samples start at every piece end and at the chordal tolerance on
   curves (≤ 64 per piece); they are inserted in a biased randomised order (deterministic
   shuffle, rounds sorted along a Hilbert curve). Then two refinements alternate until stable:
   - *Gabriel*: a boundary sub-edge (consecutive samples) whose diametral circle holds another
     sample is split at its middle (first in, first out). When no sub-edge is encroached, no
     Voronoi edge of the interior crosses the boundary (a crossing point's empty circle would
     put the edge's generators inside the sub-edge's diametral circle). A sample closer than
     2·chordal to the sub-edge does not count: such a gap holds no inscribed circle above the
     tolerance, and refining it would cost samples in proportion to its length over its width
     (near-duplicate walls, slivers, near-touching contours, sharp corner tips). Sub-edges
     shorter than chordal are not split. Only sub-edges whose neighbourhood changed are
     re-checked after an insertion: those that vanished inside its cavity (an edge between
     existing vertices never comes back) and those on its new triangles, with the triangle as a
     lookup hint; edge lookups otherwise turn around both endpoints in lockstep (cost of the
     smaller degree: fan centres can have thousands of triangles).
   - *Accuracy*: for every triangle whose circumcentre lies inside the region, e = R − r compares
     the circumradius R (an empty circle through three boundary points) with the exact boundary
     distance r of the circumcentre; points along the dual Voronoi edges are checked the same way
     (ρ − r, ρ the distance to the edge's generators). Where e > chordal/2 the sub-edge holding
     the nearest boundary point is split there, or cut into equal parts at once when it is much
     longer than the spacing √(8·(chordal/2)·r) that radius needs. One request per triangle or
     Voronoi edge per round, one split per sub-edge per round, and every requester is
     re-checked in the next round, which avoids over-refining from stale requests.
   Budgets stop refinement and set `accuracyCapped`: samples (`maxSamples`; automatic: max(250,000,
   64 per piece)) and work (300 nearest-query tree steps per allowed sample). Both are
   deterministic counts, so capped results are reproducible. Text needs 30–50 samples per
   segment and ~200 steps per sample.
5. **Skeleton.** The Voronoi edges dual to Delaunay edges between two interior triangles form the
   skeleton, except "spokes": edges whose generators are consecutive samples, or whose chord runs
   within chordal/2 of the boundary (its middle is that close: a gap narrower than the tolerance,
   samples of one gently curved stretch, the very tip of a sharp corner); those generators are
   not two distinct features. Circumcentres closer than 1e-3·min(tol.distance, chordal) are merged
   (co-circular samples), degree-2 chains become branches between nodes (junctions, ends,
   isolated points such as a circle's centre, one anchor per junction-free loop); an isolated
   point is kept only when some pair of its samples is not a spoke (three consecutive samples of
   a curve give its osculating centre, not a medial point). Every edge is subdivided until the
   radius interpolates linearly within chordal/4. Each segment carries the two boundary samples
   it is equidistant to (contour, span, span parameter).
6. **Guarantees** (unless capped): every output radius is the exact distance of its point to the
   boundary (to rounding; circumcentres sharing a merge cell reuse one nearest point, error below
   the merge distance); at every output point two boundary samples that are not boundary
   neighbours lie within radius + chordal/2, i.e. the point is equidistant within the tolerance
   from two boundary features; the radius interpolates linearly within the tolerance along
   segments. Parts of the region narrower than twice the chordal tolerance (inscribed radius
   below it) and the last ~chordal of a sharp corner's branch may have no skeleton. Branches into
   obtuse convex corners are shortened much more by the spoke test, since there the two walls are
   nearly one line. The loss depends on the corner's size and grows quickly towards 180°;
   measured examples (T-0152 verifiers, corner circumradii 3–25 mm, in multiples of chordal of
   branch length): 120° 1.8–3.3, 150° 7.1–12.7, 170° 64–125, 174° 182–312 (the whole branch
   already for a 3 mm corner), 176° up to 625, and near 178° the whole bisector branch. These are
   measurements, not bounds. The inscribed circles of the remaining skeleton still reach the
   corner vertex to within ~0.5·chordal (measured 0.31–0.50·chordal at the vertex), so carved
   depth stays within tolerance, but T-0153 must not assume that every convex corner has a
   branch reaching it (corner sharpening has to find corners from the boundary, not from branch
   ends). Coordinates are supported while 1e-12 of the largest coordinate magnitude stays below
   chordal/2 (about 5e9 mm at the default tolerance), small features far from the origin
   included: the classification area is computed about each contour's own box centre, since the
   shoelace on absolute coordinates cancels (a 1 × 0.5 mm rectangle at 1e8 mm gave area 0 and
   was dropped; the shared `signedArea` is left as is, carded separately). Beyond that limit the
   result is computed but flagged `accuracyCapped`, empty results included. The limit is judged
   on all usable input contours (closed, finite, non-empty), including ones later ignored for
   zero area or equal sides; open or otherwise unusable contours do not count.
   Deterministic for a given input (own shuffle, total orders everywhere, count-based budgets).

## Consequences
- No dependency or licence change; THIRD_PARTY_NOTICES unchanged.
- Cost is driven by the sample density the point-sample Voronoi needs near walls (spacing about
  √(8·emax·r)): 30–50 samples per boundary segment on glyph-like text at the default tolerance,
  ~10–15 µs of Release CPU (up to ~60 µs on spiky adversarial outlines) and ~1.1 KB of peak
  memory per sample. Measured on the T-0152
  machine (Release, shared): glyph pages ~0.6–0.8 s per 1,000 segments (50,003 segments: 33 s,
  2.3 M samples, ~2.5 GB peak), scaling ~2.1–2.3× per doubling up to 50,000 segments; a
  50,000-vertex polygonal outline ~0.5 s; a polygonal circle of 16,000 vertices 0.08 s. Spiky
  adversarial outlines (thousands of long spikes, Bezier bulges, grid-snapped overlaps) cost up
  to ~60 µs per sample and end at the budgets (e.g. 6,500 spans: ~55 s, capped). Scaling tests:
  `medial_axis_robustness_test.cpp` (≤ 2.5× per doubling; hidden "[.medial-perf-50k]" ladder).
- A future speed-up, if T-0153 needs one, is to project output points onto the exact bisectors
  of their generator pieces, which would let sampling stop at the Gabriel level (a local
  nearest search through the triangulation was tried and was slower than the tree on text).
- T-0153 walks branches from node to node, reads depth from `MedialPoint::radius`, and can use
  `MedialBranch::sides` to tell which boundary spans a stretch carves against (e.g. to recognise
  branches that run into a corner for corner sharpening).
