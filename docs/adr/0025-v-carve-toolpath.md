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

### 1. Depth rule: the footprint at the start plane, less half the chordal tolerance
- The tool's *footprint* at depth d is half `tooling::cuttingWidthAtDepth(tool, d)`, its
  cross-section at the start plane (the tool is narrower below it). At a skeleton point with
  inscribed radius r the tool goes to the depth whose footprint is r − chordal/2
  (`vCarvingDepthForRadius`: V-bit r/tan(θ/2), ball nose R − sqrt(R² − r²)), limited by the depth
  limit (§3). So the tool touches both walls within the chordal tolerance and never crosses one.
- The margin of chordal/2 covers the medial axis' linear radius interpolation between points
  (≤ chordal/4, ADR-0023 §5) and the walk simplification (≤ chordal/16, §4).
- **Accuracy (honest bound).** At every cut point above the limit the footprint is within
  chordal of the boundary distance (measured ≤ 0.87·chordal). Over the whole region, every point
  whose depth is above the limit is carved to within **1.5·chordal** of its depth, measured as a
  footprint radius (V-bit: depth within 1.5·chordal/tan(θ/2)); 1.625·chordal in resolved crossing
  parts (§4). The bound is the margin (0.5·chordal) plus the medial axis' resolution: gaps
  narrower than ~2·chordal have no skeleton and branch ends stop up to ~chordal short of corner
  tips (ADR-0023). Measured: up to 1.499·chordal (T-0153 verifier harness), asserted in the sweep.
  Points that need the full limit depth are flat floor (§3), not bounded by this: a V-bit's own
  clearing leaves ridges, end mills leave corners.
- **Start depth** moves the whole carve down: the carve is computed from the plane at
  −start depth, the material above it being assumed gone over the regions (carving into a pocket
  floor); flat depth is measured from that plane. Safe Z must be above the material surface.
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
  arcs and Beziers. `CrossingVectorsResolved` reports it, and the margin of those parts grows by
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
When `medialAxis` reports `accuracyCapped` (sample or work budget, or coordinates beyond its
limit) for any region, nothing is produced and `MedialAxisCapped` names each capped region's
vectors, with the advice to use a coarser chordal tolerance or simplify. A partial or degraded
carve is never returned silently, and a capped skeleton has no accuracy guarantee to make AC1
hold. Known non-capped limits (ADR-0023): gaps narrower than ~2·chordal have no skeleton (not
carved: their depth is below ~chordal/tan(θ/2)); obtuse convex corners lose branch length, but the
remaining inscribed circles reach the vertex within ~0.5·chordal, so the carve stays within the
tolerance (within the coverage bound of §1).

### 6. Corner sharpening is out of scope (follow-up)
The inventory names "Corner Sharpen" without describing it. Corners of the region are already
carved by the skeleton to within the tolerance (§5), so the option is not needed for accuracy;
defining what it does beyond that (e.g. lifting along corner bisectors in flat-area corners the
clearance tool cannot reach) is product behaviour to be specified first. Whatever it becomes, it
must find corners from the boundary, not from skeleton branch ends (ADR-0023: obtuse corners may
have no branch). Proposed as a follow-up card.

### 7. Medial axis fix
Symmetric input could leave the skeleton empty: splitting the base of an isosceles triangle with
an obtuse apex at its middle puts both circumcentres exactly on the legs, neither inside nor
outside, so no interior triangle ever asked for refinement. `medialAxis` now also refines where
a circumcentre lies on the boundary (within the merge distance) with a circumradius above half
the chordal tolerance. Regression test in `v_carve_test.cpp`. The same degeneracy hid the corner
junctions of symmetric frames (two nested squares): their skeleton lacked the arcs round the inner
corners, whose junction with the outer corner's bisector lies at radius 10 − 5√2 ≈ 2.93 for a
5 mm wide frame; the T-0152 test asserted every radius ≤ 2.5 and now checks that junction.

### 8. Safety evidence (AC1)
`core/tests/toolpath/v_carve_oracle.hpp` checks every non-rapid move below the start plane over
its whole length: dist(centre, boundary) − footprint(depth) ≥ 0, with exact closest points on every
line, arc and Bezier span, certified subdivision (the distance is 1-Lipschitz, the footprint
monotone in depth) and an even-odd inside test over the original vectors. The generative sweep
(stars, holed stars, symmetric polygons, rounded rectangles, glyphs, Bezier blobs, thin strips,
rings with neighbours, overlapping glyphs, overlapping circles and polygons, self-crossing
Bezier figures-of-eight and bow ties; V-bits 30–150°, ball noses; flat depths with and without
one or two clearance tools; start depths; chordal 0.01–0.05) runs 120 cases in Debug and 800 in
Release in ctest and asserts the cone check, the end mills' allowance, the lateral accuracy at
cut points and the coverage bound of §1. The verifier's independent harness
(`/home/chris/harness/t0153v1`, crossing and non-crossing seeds) finds no cone violation after
round 1 (worst +1.0e-3 mm; crossing input included) and lateral coverage ≤ 1.499·chordal. Every
shipped post's output passes the safety rules (inclined G1 cuts allowed).

### 9. Time bound (AC2)
Release, the T-0153 machine (shared): 5,000 glyph-like regions (45,000 segments) in ~11–14 s of
CPU (~1.9 M moves); the same page with a 0.5 mm flat depth (the V-bit clearing the flat areas,
Pocket per region) ~14 s (~2.8 M moves); linear in the number of regions. The documented bound,
checked in Release builds (the Linux app job and Windows CI run the suite in Release), is
**≤ 60 s of CPU for 5,000 regions, with or without a flat depth**, with ≤ 2.5× per doubling;
Debug builds check the doubling rule on 125/250 regions and run the flat page with 250. The
medial axis needs no speed-up for this: per part it costs ~0.24 s per 1,000 segments. Its
point-sample skeleton zig-zags within the tolerance, which keeps ~400 moves per 5 × 8 mm glyph
at the default tolerance and a slight Z jitter; projecting output points onto the exact
bisectors (ADR-0023's proposed speed-up) would also smooth and thin them.

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
