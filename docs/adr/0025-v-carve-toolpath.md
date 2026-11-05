# ADR-0025: V-Carve toolpath (depth rule, flat areas, regions, safety, record schema)

Date: 2026-11-03 · Status: proposed (T-0153)

## Context
T-0153 adds the V-carve toolpath (feature-inventory "V-Carve Toolpath"): carve closed regions with
a V-bit (or a ball nose) whose depth follows the medial axis, so the tool touches both walls;
start depth; optional flat depth, the flat-bottomed areas cleared with clearance tools (Offset or
Raster, reusing Pocket); corner sharpening; ordering; safe links; a native record and
`carvetto-cli toolpath v_carve`. AC1 is a safety requirement (the tool never crosses a boundary,
checked exactly over a generative sweep; depth = radius / tan(θ/2) within tolerance; the flat depth
honoured with gouge-free clearance pocketing; goldens; post safety). AC2 asks for a documented time
bound on text-heavy jobs (≥ 5,000 glyph-like regions). The geometry comes from
`geometry::medialAxis` (ADR-0023), whose documented limits shape several choices below.

## Decision

### 1. Depth rule and accuracy: the user's chordal tolerance is the whole budget
- The tool's *footprint* at depth d is half `tooling::cuttingWidthAtDepth(tool, d)`, its
  cross-section at the start plane (the tool is narrower below it). At a skeleton point with
  inscribed radius r the tool goes to the depth whose footprint is r − margin
  (`vCarvingDepthForRadius`: V-bit r/tan(θ/2), ball nose R − sqrt(R² − r²)), limited by the depth
  limit (§3). So the tool touches both walls within the tolerance and never crosses one.
- **Accuracy (director decision, round 2: "depth = radius / tan(θ/2) within tolerance" means the
  user's chordal tolerance).** Every point of the region whose depth is above the limit is carved
  to within **1·chordal** of its depth, measured as a footprint radius (V-bit: depth within
  chordal/tan(θ/2)). The budget is split: the medial axis runs at an internal tolerance
  c = chordal/3 (`kInnerDivisor`), the margin is c/2 = chordal/6 (it covers the medial axis' linear
  radius interpolation, ≤ c/4, and the walk simplification, ≤ c/16), and the medial axis'
  resolution is about c (gaps narrower than ~2c have no skeleton, branch ends stop up to ~c short
  of corner tips, ADR-0023): about chordal/2 in total, plus chordal/8 in resolved crossing parts
  (§4). Pocket (flat areas, wall rings) runs at the same internal tolerance, since its safety
  margin (its chordal) is the gap between the wall rings and the flat area's edge. The chordal
  tolerance must be at least 0.0003 mm; the internal tolerance is floored at Pocket's minimum
  0.0001 mm (0.0003 / 3 evaluates to 9.999999999999999e-05, which Pocket refused: every flat area
  at the documented minimum failed in round 3). The first version ran the medial axis at the user's tolerance with a margin of chordal/2
  and measured up to 1.5·chordal (2.3× at a Bezier cusp, more in crossing slivers). Measured now:
  see §8. Points that need the full limit depth are flat floor (§3), not bounded by this.
- Walks are kept whenever some point has a positive allowed footprint: a ball nose's depth for a
  small footprint is tiny (r²/2R) and walks below the distance tolerance were once dropped,
  leaving thin parts uncut (10·chordal short).
- **Start depth** moves the whole carve down: the carve is computed from the plane at
  −start depth, the material above it being assumed gone over the regions (carving into a pocket
  floor); flat depth is measured from that plane. Safe Z must be above the material surface.
  Above the start plane the cone is wider than its footprint; that is the expected behaviour
  (the reference behaviour: start depth is used after a pocket to that depth, so there is no
  material there). With material left above the start plane, the cone cuts the walls above it.
- Z is linear between consecutive points. Every segment is first cut where the depth rule
  min(limit, depth(max(0, r − margin))) has a kink: where r − margin reaches 0 and where it
  reaches the footprint at the limit. Without that cut, linear Z across the limit kink makes a
  V-bit too shallow (min is concave: up to 58·chordal short in the first verifier round, e.g. a
  flat triangle's apex region). Between kinks a V-bit is exact (footprint linear in depth, depth
  linear in radius). A ball nose's depth is convex in its footprint, so its pieces are halved
  until a certified bound holds (at a chord depth the footprint exceeds the allowed one by at
  most |Δs|·(1 − K/k), K the chord slope and k the curve slope at the deeper end; ≤ chordal/4).

### 2. Tools
- Carving tool: **V-bit or ball nose** (the inventory names both; `vCarvingToolAllowed`). Engraving
  tools, tapered ball noses and the rest are refused (`InvalidTool`); the list can be widened.
  The tool must pass `tooling::validate`, have feed and plunge rates > 0.
- Clearance tools: **end mills** only (`vCarvingClearanceToolAllowed`): a flat bottom needs a
  flat-bottomed tool; a V-bit or ball nose would leave ridges. The inventory's per-tool cutting
  direction is one `clearanceDirection` for all clearance tools (a per-tool list can be added
  without a format break).
- Tools are arguments (`tool`, `cuttingData`, `clearanceTools` as `PocketTool`s), not record
  content, as in Drill and Engrave.

### 3. Depth limit and flat areas
- The limit is the flat depth when set, never deeper than the tool's full-width depth
  (`vCarvingFullWidthDepth`: a V-bit's cone height (D/2)/tan(θ/2), a ball's radius); below it the
  shank would cut vertical walls. A flat depth beyond it is clamped with `CarveDepthLimited`.
  Without a flat depth, regions wider than the tool can carve are carved to its full-width depth,
  their flat bottom cleared, and `CarveDepthLimited` says so.
- A region has a flat area when its largest inscribed radius less the margin exceeds the
  footprint at the limit. Only those regions go to Pocket.
- **With clearance tools** `pocketToolpath` clears the flat areas (tools in list order, later
  ones as rest tools; Offset or Raster with angle, direction, ramp plunge; boundary pass last;
  allowance = the carving tool's footprint at the limit, so every end mill keeps that far off the
  walls at every depth: gouge-free by the pocket oracle). The carving tool then cuts the walls of
  the flat areas at the limit along `pocketCentreRegion` (inset by the footprint plus Pocket's
  chordal margin), so the walls are cut by the V-bit, not the end mill.
- **The carving tool clears what the end mills cannot reach.** With clearance tools, the carving
  tool is added to Pocket's tool list as the last rest tool (same allowance): Pocket's rest
  machining gives it only the material the end mills left (flat areas narrower than the
  smallest end mill, inside corners). Its centres keep the allowance plus its own footprint off
  the walls, so its footprint reaches every point of the flat area to within Pocket's chordal
  margin of its edge, where the wall rings cut. Its stepover is limited to the width it cuts at
  the limit (Pocket's rule). "Nothing left" (`NoRestMaterial`), "cannot reach the rest"
  (`RestUnreachable`) and "too large" (`ToolTooLarge`: a flat area narrower than its rest pass,
  covered by its walls and skeleton) are not reported for it: the end mills' leftovers in the inside corners of a
  flat area are out of reach of any pointed tool at the flat depth (the wall ring at the corner
  slopes the floor up into it), and reporting that for every letter would be noise. This was chosen
  over a warning: the floor gets cleared, with the same machinery, and the moves come first in
  that region's part of the carving toolpath.
- **Without clearance tools** the carving tool clears the flat areas itself with `pocketToolpath`
  (its cutting data's stepover and pass depth; the boundary pass is the same walls), prepended to
  its carve. A V-bit floor has ridges of stepover/(2·tan(θ/2)); clearance tools avoid them.
- Pocket runs **region by region** (§4) in machining order, the moves of each clearance tool
  concatenated, so its caps (200,000 input vertices) and cost apply per region: one call over
  5,000 letters with a flat depth was refused at the vertex cap. Pocket's diagnostics of the
  same kind (issue and tool) are reported once, with the number of further regions.
- Pocket computes from the start plane as its surface (start depth 0, safe Z + start depth) and
  the moves are moved down by the start depth: Pocket would otherwise size a V-bit by its width at
  the material surface. Pocket's diagnostics are passed on prefixed "flat area clearance: ";
  when it refuses (invalid tool, e.g. a stepover wider than the V-bit cuts at the flat depth,
  invalid parameters, work limit) the whole V-carve is refused: no half-done flat areas.
- Machining order: the clearance tools, then the carving tool. With clearance tools every
  toolpath is named "<name> [<tool name>]" (Pocket's multi-tool naming).

### 4. Regions (crossing input resolved), walks, order, links
- **Crossing input.** Vectors that cross, touch or overlap each other or themselves
  (`geometry::intersect`, `hasSelfIntersections`, pairs pruned by `BoxIndex`) are resolved into
  their even-odd region with `geometry::unionContours`, as Pocket resolves its region: each
  cluster of crossing vectors separately (even-odd membership is the parity over all contours,
  so the region is the same), polygonised within chordal/8; the rest keep their exact lines,
  arcs and Beziers. A resolved polygon can lie up to chordal/8 away from its curves, so it can
  cut a vector closer than that (round 2: a hole 0.003 mm from a circle of a crossing pair was
  cut through). Every resolved polygon is therefore checked against the vectors outside its
  cluster (`geometry::intersect`, pairs pruned by their boxes); a vector it crosses or touches
  joins the cluster, which is resolved again, until no polygon touches an outside vector. A
  union polygon that touches itself at a vertex (two pieces meeting at a point, e.g. a letter
  tangent to a frame edge) is cut into loops there (parities add up), since the medial axis needs
  simple contours; the nesting probes skip points on the candidate's boundary: 32 points by arc
  length, then every span's middle and quarter points (round 3: a D-shaped hole resolved against
  the circle it touches lay on the outer polygon along its whole arc; the sampled probes all hit
  the arc, the hole became a part of its own and was carved). `CrossingVectorsResolved` reports it, and the margin of those parts grows by
  chordal/8 (the polygon may lie that far outside a curved wall). The medial axis of crossing
  input has nodes and branches outside the even-odd region (ADR-0023: "every contour then still
  acts as a wall"); before this the carve plunged there (verifier round 1: up to 2.5 mm deep
  inside the hole of two overlapping squares, 3.8 mm outside overlapping circles).
- **Parts.** The non-crossing boundaries are nested by containment (a point of each against the
  smaller-area candidates whose box contains it); a part is a boundary at even depth with the
  boundaries directly inside it (its holes). A letter with its counters is one part, a frame round
  a block of text another, and each letter inside the frame its own. Parts cannot interact, so
  each gets its own medial axis: cost and memory stay per part (a 5,000-glyph page is 5,000 small
  problems) and failures stay local. (The first version grouped by overlapping boxes, which put a
  frame and all the text inside it into one medial axis.)
- Skeleton branches are joined into walks: a walk starts at an odd-degree node (an end: corner
  tips) when there is one and at every node continues along the unused branch that turns least,
  so a skeleton with k ends is cut in about k/2 walks; isolated nodes (a circle's centre) are a
  plunge. Points within chordal/16 (position plus radius) of the segment joining their
  neighbours are dropped. Walks whose deepest point is above the distance tolerance are skipped.
- Each walk or wall ring: rapid at safe Z over its start, plunge at the plunge rate, cut (feed
  moves with Z following the depth: inclined cuts), retract to safe Z. Retracting between walks
  is always safe; linking at depth along the skeleton is a possible refinement.
- Parts are ordered by the shared `orderCuts` with `order` (selection, left to right, bottom to
  top, grid; none = nearest part first from `startPosition`); within a part, its own flat-area
  clearing (without clearance tools) and then its cuts nearest first from where the tool is.
- "Use Vector Start Points" does not apply to skeleton cuts (they have no vector start) and is
  not offered.

### 5. The medial axis must be accurate
When `medialAxis` reports `accuracyCapped` (sample or work budget, or coordinates beyond its limit)
for any region, nothing is produced; the same when it ignores a boundary that encloses area (every
boundary of a part separates inside from outside by construction, so an ignored one is a medial
axis failure and carving on would cut through a hole: round 2 found such cases, fixed in §7, and
this guard keeps any future one from reaching the machine). A boundary read on the wrong side, or a
hole nested as an outer, is caught too (`detail::SkeletonCheck`): at sampled skeleton points (every
point of a branch with at most 8, otherwise 8 evenly spaced ones plus its deepest and its last;
every isolated node) whose radius r is above twice the internal tolerance plus the distance
tolerance, the point must lie inside the whole even-odd region (not just its part: a hole taken for
an outer is a part of its own) and its distance to the walls must be at least r minus that margin,
i.e. the inscribed disc must agree with the true walls. Both use the boundaries polygonised at the
internal tolerance, restricted to the rings that meet the box of the skeleton's inscribed discs (no
other ring can contain a checked point or come within its radius), so the cost stays per part (AC2
unchanged within measurement noise). Round 4's version checked only that the deepest point of each
branch was inside, which a wrong-side hole whose skeleton's deepest points fall inside an island
passed (verifier fault injection: −3.3 to −5.3 mm). Otherwise nothing is carved. In every case
`MedialAxisCapped` names the region's vectors, with the advice to use a coarser chordal tolerance or simplify. A partial or degraded
carve is never returned silently, and a capped skeleton has no accuracy guarantee to make AC1
hold. Known non-capped limits (ADR-0023): gaps narrower than ~2·chordal have no skeleton (not
carved: their depth is below ~chordal/tan(θ/2)); obtuse convex corners lose branch length, but the
remaining inscribed circles reach the vertex within ~0.5·chordal, so the carve stays within the
tolerance (within the coverage bound of §1).

Known limit (to be carded): a hole thinner than the distance tolerance touches its own sides
within that tolerance, is resolved as crossing input and vanishes from the even-odd region, so it
is cut through; it is reported only as `CrossingVectorsResolved`.

### 6. Corner sharpening is out of scope (follow-up card T-0160, owner decision 2026-11-03)
The inventory names "Corner Sharpen" without describing it. Corners of the region are already
carved by the skeleton to within the tolerance (§5), so the option is not needed for accuracy;
defining what it does beyond that (e.g. lifting along corner bisectors in flat-area corners the
clearance tool cannot reach) is product behaviour to be specified first. Whatever it becomes, it
must find corners from the boundary, not from skeleton branch ends (ADR-0023: obtuse corners may
have no branch). Moved to its own card, T-0160 (owner decision).

### 7. Medial axis fixes (T-0152 code)
(a) Symmetric input could leave the skeleton empty: splitting the base of an isosceles triangle with
an obtuse apex at its middle puts both circumcentres exactly on the legs, neither inside nor
outside, so no interior triangle ever asked for refinement. `medialAxis` now also refines where
a circumcentre lies on the boundary (within the merge distance) with a circumradius above half
the chordal tolerance. Regression test in `v_carve_test.cpp`. The same degeneracy hid the corner
junctions of symmetric frames (two nested squares): their skeleton lacked the arcs round the inner
corners, whose junction with the outer corner's bisector lies at radius 10 − 5√2 ≈ 2.93 for a
5 mm wide frame; the T-0152 test asserted every radius ≤ 2.5 and now checks that junction.

(b) Classification probes: a contour's sides were read at one probe per span (the span's middle),
up to 8; a probe within chordal + distance of another contour does not count. A one-arc circle has
one probe, so a hole 0.005 mm from a wall (and small letter counters at a coarse tolerance) had no
clear probe and was ignored. Now 32 probes are spread over the contour by length (a Bezier counted
by its control polygon: its exact length subdivides exponentially at cusps) (round 3: a
D-shaped hole whose 170-segment arc hugs the outer circle had all its span-indexed probes on the
arc, none on the chord). When no probe is clear, the one farthest from the other contours is
used: farther than chordal from them (exact closest points on their spans) the rings' winding is
right; closer, but not closer than chordal/64, their winding is counted from their curves
polygonised at an eighth of that distance. Only when even that fails (coincident contours) do the old offsets at ±2·chordal
decide; round 3 showed those offsets cross a hole thinner than that and read the wrong side (the
D-hole was carved ~1 mm deep).

(c) Gabriel test hidden by an exempt sample: a sub-edge counted as encroached only when one of the
two opposite vertices of its Delaunay triangles lies in its diametral circle and is not exempt
(within 2·chordal of the sub-edge, a gap not worth refining). When the opposite vertex is in the
circle but exempt (the neighbour sample of a flat corner), other samples in the circle were never
looked at: a thin band from a frame crossed by letters had no refinement and an empty skeleton.
Now the triangles reaching into the diametral disc are searched (at most 512; beyond that the
sub-edge counts as encroached).
Regression tests for (b) and (c) in `medial_axis_test.cpp`.

### 8. Safety and accuracy evidence (AC1)
`core/tests/toolpath/v_carve_oracle.hpp` checks every non-rapid move below the start plane over
its whole length: dist(centre, boundary) − footprint(depth) ≥ 0, with exact closest points on every
line, arc and Bezier span, certified subdivision (the distance is 1-Lipschitz, the footprint
monotone in depth) and an even-odd inside test over the original vectors. The generative sweep
(stars, holed stars, symmetric polygons, rounded rectangles, glyphs, Bezier blobs, thin strips,
rings with neighbours, overlapping glyphs, overlapping circles and polygons, self-crossing
Bezier figures-of-eight and bow ties, one-arc holes nearly touching a wall, frames crossed by
letters; V-bits 30–150°, ball noses; flat depths with and without one or two clearance tools;
start depths; chordal 0.01–0.05) runs 120 cases in Debug and 800 in Release in ctest and asserts
the cone check, the end mills' allowance, the lateral accuracy at cut points and the coverage
bound of §1 (≤ chordal). The oracle counts every span as a wall, so the sweep avoids edges that
coincide exactly (they cancel under even-odd). The verifier's independent harness
(`/home/chris/harness/t0153v2`, vc2 over its 16 families) after round 2: no real cone violation;
lateral shortfall ≤ 0.5·chordal on non-crossing input and ≤ 0.55·chordal on resolved crossing
input, except family 15 (exactly coincident edges), whose coverage numbers the verifier marks as
artefacts of the cancelled edges. Every shipped post's output passes the safety rules (inclined
G1 cuts allowed).

### 9. Time bound (AC2)
Release, the T-0153 machine (shared, under load from other agents): 5,000 glyph-like regions
(45,000 segments) in ~23 s of CPU (~3.3 M moves); the same page with a 0.5 mm flat depth (the V-bit
clearing the flat areas, Pocket per region) ~29 s (~4.7 M moves); linear in the number of regions
(doubling ratio 1.93). Round 1 measured ~11 s / ~14 s: the internal tolerance of §1 (medial axis
at chordal/3) roughly doubles the samples and moves. The documented bound, checked in Release
builds (the Linux app job and Windows CI run the suite in Release), is **≤ 60 s of CPU for 5,000
regions, with or without a flat depth**, with ≤ 2.5× per doubling (625 → 1,250 regions,
interleaved runs with T-0145's `interleavedCpuSeconds`, robust to parallel ctest load); Debug
builds check the doubling on 60 → 120 regions and run the flat page with 120. The medial axis
needs no speed-up for this bound; projecting output points onto the exact bisectors (ADR-0023's
proposed speed-up) would also smooth the skeleton's zig-zag (a slight Z jitter within the
tolerance) and thin the moves, which is the next lever if the bound or the G-code size matters.

### 10. Names, API, record and params.json
- **Names (owner decision 2026-11-03).** The run-together product name is a trademark and
  CONTRIBUTING forbids it in identifiers. File names, the CLI command, ctest and golden names and
  the job.json "operation" value use `v_carve`; C++ types and functions use the generic noun
  "VCarving" (`VCarvingParams`, `VCarvingResult`, `vCarvingToolpath`, `vCarvingDepthForRadius`,
  `ToolpathOperation::VCarving`); user-facing text says "V-Carve".
- `vCarvingToolpath(contours, const Tool&, const optional<CuttingData>&, const vector<PocketTool>&
  clearanceTools, const VCarvingParams&) → VCarvingResult{clearance (one Toolpath per clearance
  tool), toolpath (the carving tool), diagnostics, regions}`; `toolpaths()` lists those with moves
  in machining order. New `ToolpathIssue` values appended: `CarveDepthLimited`,
  `MedialAxisCapped`, `CrossingVectorsResolved`.
- `ToolpathParams` gains `VCarvingParams` as its fifth alternative (`"operation": "v_carve"`);
  each toolpath of a V-carve is its own record with the same params and its own tool snapshot
  and tool source. job.json params (all optional, unknown keys dropped, numbers finite): `name`,
  `start_depth`, `flat_depth`, `safe_z`, `clearance_strategy`, `clearance_direction`,
  `clearance_raster_angle_rad`, `ramp_plunge`, `ramp_distance`, `order`, `start_position`,
  `tolerance`; params.json (`carvetto-cli toolpath v_carve`) uses `clearance_raster_angle` in
  degrees and adds `tool`, `clearance_tools` (an array of tool objects) and `layers`.
  `nonFiniteParam` names the keys the same way. The format stays 1.1 (a new optional variant
  alternative, ADR-0021).

## Alternatives
- **Depth exactly r/tan(θ/2) with no margin:** relies on the medial axis' interpolation
  guarantee being exact between points; a margin of chordal/2 keeps AC1 by construction.
- **One medial axis for the whole job:** ~0.7 s per 1,000 segments and 2.5 GB for 50,000 segments
  (ADR-0023), and one capped glyph would cap everything.
- **Resolve every vector with the boolean union:** simpler, but polygonises all curves (more
  segments, a larger margin) where only crossing input needs it.
- **One Pocket call for all flat areas:** hits Pocket's vertex cap on text-heavy jobs.
- **Skip capped regions with a warning:** a carve with holes in it; refused instead (§5).
- **Clamp the V-bit's self-clearing stepover silently:** hides a poor floor; refused with Pocket's
  message instead, which points at the stepover.
- **Clearance with any tool type:** ridged floors; end mills only for now.
