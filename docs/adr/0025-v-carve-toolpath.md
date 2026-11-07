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
  tolerance must be at least 0.0003 mm and the distance tolerance at least 0.000001 mm (T-0142, the
  same floor as Pocket, which the flat areas call; refused with V-Carve's own message); the internal tolerance is floored at Pocket's minimum
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
  slopes the floor up into it; corner sharpening, §6, clears the convex ones with passes of
its own), and reporting that for every letter would be noise. This was chosen
  over a warning: the floor gets cleared, with the same machinery, and the moves come first in
  that region's part of the carving toolpath. **Ridges (T-0164 d).** Like any V-bit floor, the
  rest passes leave ridges of stepover/(2·tan(θ/2)) (ball nose: R − sqrt(R² − (stepover/2)²)),
  at most the depth limit. A stepover wider than the width the tool cuts there is limited to that
  width, where neighbouring passes only touch at the floor and the ridges reach the full depth
  limit (verifier round 3: 0.5–0.84 × the flat depth on narrow arms and on a square the end mill
  was too large for). When that happens (the stepover beyond the width by more than the distance
  tolerance: 2·footprint can come out one ulp short of a stepover equal to it) and the carving
  tool has rest material to clear,
  `StepoverLimited` says so once: "tool '<name>': stepover <s> mm is wider than it cuts at the
  flat bottom (<w> mm); its rest clearing of the flat areas uses <w> mm and can leave ridges up
  to <limit> mm high". A stepover within the width is the user's choice and not reported.
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
  cluster found touching nothing outside is remembered with its members and not resolved or
  checked again until they change, and the clusters are worked through in the old order (always
  the lowest cluster not known to be stable) with member lists kept per cluster and rebuilt only
  for the clusters an absorption involves (T-0164: every absorption restarted the resolution of
  all clusters and rebuilt every member list, ~3.5× per doubling with many absorbing clusters;
  now ~2.1× up to 6,400 pairs, same result, since a cluster's union and checks depend only on its
  members). A
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
hole nested as an outer, is caught too (`detail::SkeletonCheck`): every skeleton point (and every
isolated node) whose radius r is above twice the internal tolerance plus the distance tolerance (2c
+ d) must lie inside the whole even-odd region (not just its part: a hole taken for an outer is a
part of its own) and have no wall closer than r − (2c + d), i.e. the inscribed disc must agree with
the true walls. Smaller discs are not checked: a wrong skeleton that thin can cut at most 2c + d
outside the region, about 0.67 mm at the largest chordal tolerance, 1 mm (0.034 mm at 0.05).
The same slack applies to every checked disc, not only to small ones: a wrong skeleton whose
discs all reach less than 2c + d past a true wall is accepted, whatever their radius (T-0164 j;
net verifier: a hole filled almost entirely by an island, read on the wrong side, carved up to
0.24 mm outside the region at chordal 1.0 and 0.007 mm at 0.05, within the bound). The
rings are the boundaries polygonised at the internal tolerance, indexed by box; each point queries
the index for its disc's box (and, within a ring, boxes of 16-segment runs). Until T-0164 only the
rings of other parts were measured, on the grounds that the medial radius is already the exact
distance to the part's own boundaries, whose rings lie within c of them. That holds only while the
medial axis uses every own boundary; one dropped without being counted as ignored was measured
nowhere (net verifier: a hole on a square's diagonal, skeleton of the square alone, passed). T-0164
(g) measures the own rings too, at every checked point, so a silently dropped boundary is caught
wherever a disc reaches more than 2c + d past it, exactly as a foreign ring would be (round 1
measured them only at nodes and ray-count seeds, and holes away from every node, e.g. in the middle
of a long strip, still carved 4.9 mm deep under fault injection). Own rings without area are never
measured: the medial axis ignores them (they bound nothing). A correct radius never fails this
(the rings lie within c of the boundaries, the slack is 2c + d).
Membership is carried along the skeleton: walking from a point whose disc passed, no ring is within
r − (2c + d) of it, so a next point nearer than that is on the same side of every ring; the exact
ray count runs only where that does not hold and once per connected piece. Membership is handed
over at a node only when the node point lies inside the disc that proved it, and taken up by the
next branch only when that branch's first point does (T-0164 h): the medial axis ends every
branch at its node's point, but the check does not rely on it. If any check fails,
nothing is carved. Round 4 checked only that each branch's deepest point was inside, and round 5's
first fix sampled 8 points per branch (plus the deepest and the last): wrong skeletons whose
checked points all fell inside islands passed (verifier fault injection, −0.47 to −5.3 mm). Cost,
Release, 5,000 glyph regions on the shared machine under load (load average ~20, two rounds
interleaved with the earlier versions): plain page 25.4 / 28.1 s CPU (round 4: 27.8 / 26.5 s, the
8-sample version: 25.4 / 28.0 s); one part with 5,000 letter holes (a frame round the page) 35.7 /
41.6 s (round 4: 40.5 / 42.2 s, the 8-sample version: 45.5 / 54.6 s), so AC2's 60 s bound keeps its
margin. In every case `MedialAxisCapped` names the region's vectors, with the advice to use a
coarser chordal tolerance or simplify.
A partial or degraded
carve is never returned silently, and a capped skeleton has no accuracy guarantee to make AC1
hold. Known non-capped limits (ADR-0023): gaps narrower than ~2·chordal have no skeleton (not
carved: their depth is below ~chordal/tan(θ/2)); obtuse convex corners lose branch length, but the
remaining inscribed circles reach the vertex within ~0.5·chordal, so the carve stays within the
tolerance (within the coverage bound of §1).

Known limit (T-0164 a, documented): a hole or sliver thinner than the distance tolerance is not
kept. Its sides touch within that tolerance, so it is resolved as crossing input and vanishes from
the even-odd region (only `CrossingVectorsResolved` is reported), or, between nested vectors
closer than the tolerance, the flat-area walls computed at the internal tolerance do not keep it
(verifier: a 0.8 µm strip between nested squares, cone 0.058 mm past it at chordal 0.005; a
0.63 µm sliver, 0.014 mm at chordal 0.1). The material lost is the sliver itself, thinner than
what the tolerance tells apart from touching, plus the cone's reach beyond it. No diagnostic of
its own: telling such a sliver from a touch would need the very distinction the tolerance gives
up, and a warning on every touching or crossing pair (letters overlapping a frame) would be noise.
To keep a gap that thin, use a smaller distance tolerance.

### 6. Corner sharpening (T-0160, owner decisions 2026-11-05)
The inventory names "Corner Sharpen" without describing it. Corners of the region are carved by
the skeleton to within the tolerance (§5) everywhere except one place: with a flat depth **and**
clearance end mills, the floor of a flat area is cut by the carving tool along the wall ring
(its centre ρ + Pocket's chordal c from the walls at the flat depth F, ρ its footprint at F) and
swept flat by the end mills (their centres inset by R + ρ + c, so a disc of radius R round each
centre). Wherever the walls are convex in a way the end mill cannot follow (a sharp corner, a
fillet or chamfer smaller than the end mill, a few-segment polyline rounding, a junction of curved
walls), a sloped "tent" of floor is left between the wall ring and the end mill's swept area. The
carving tool as Pocket's rest tool keeps 2ρ + c off the walls and never gets in (§3). Without
clearance tools Pocket's insets keep the convex corners sharp (the carving tool's own boundary
pass is the wall ring), so nothing is left.

**Owner decisions.** (1) Clean flat-floor corners only: plain V-carve corner tips are unchanged,
and there is no effect without a flat depth plus clearance tools. (2) Match the floor finish: a
tip groove along each corner's bisector at the flat depth, plus passes either side at the carving
tool's stepover, so the corner is no rougher than the rest of the floor. (3) On by default:
`VCarvingParams::cornerSharpen`, job.json and params.json `corner_sharpen` (a missing key reads
as true, so older files get the default; the format stays 1.1, §10), UI label "Corner Sharpen"
(no V-Carve form in the app yet). (4) The threshold is automatic: a corner is sharpened exactly
when its predicted leftover is taller than the chordal tolerance; there is no angle setting.
(5) 2026-11-05, after round 1: slightly rounded corners count. Any convex stretch the end mill
cannot reach into (small fillets, short chamfers, few-segment polyline roundings, curved-edge
junctions) is sharpened whenever its predicted leftover exceeds the chordal tolerance, not only
sharp junctions. So detection and passes work from the actual walls and the actual swept area,
not from boundary junctions and tangent wedges (round 1 did that: it missed curved junctions and
rounded corners, and its pass ends did not close the bands between passes on steep fillets).

**Notation.** W: the even-odd region of the wall rings as the carving tool cuts them (polygons of
their vertices; code: `RingRegion`), f(p) the distance from p to its edges. E: the end mills'
swept area at the flat depth, the union over every clearance toolpath of the discs of the end
mill's radius R round its level moves at F (consecutive level feeds chained into polylines;
plunges, ramps and rapids sweep less and are left out). h(d): the carving tool's height above its
tip at distance d from its axis (V-bit d/tan(θ/2), ball nose R_b − sqrt(R_b² − d²)), r(z) its
inverse, the footprint radius at depth z. s: the carving tool's stepover clamped as for its rest
clearing (§3: min(stepover, 2ρ), or ρ when it has none). δ = min(r(chordal), s/2). A point p of the
floor (in W) that no move sweeps keeps at most h of its distance to the nearest carving-tool cut at
F (the ring edges and the corner passes are such cuts; the skeleton and rest passes only lower it).

**Detection (code: `sharpStretches` in `v_carve_corners.cpp`).** The patches are the connected
parts of P = (W ⊖ δ) ∖ E, computed with Clipper2 (ADR-0006): the rings inset by δ less a grid
margin of 4·10⁻⁶ mm (round joins; their vertices lie on the circles, so the polygon contains
every point at least δ from the rings), minus each end mill's polylines offset by R less that
margin (round joins and caps at chordal/64, never below 2·10⁻⁶ mm; vertices on the circles, so the
polygon lies inside E). A patch is a sharpened stretch when
- its flat area is one the end mills sweep: the smallest outer wall ring round it (with the rings
  inside it) contains the start of some swept polyline. A flat area no end mill enters (narrower
  than the end mill throughout) is the carving tool's own rest clearing, whose floor keeps that
  clearing's ridges (up to the depth limit, T-0164 d), not a clean flat floor; its corners are
  left alone (decision 1; as in round 1, where the verifier listed them as non-blocking). A part of a swept flat area cut off from the
  swept area (a hole's wall ring closing a corner's tent) is sharpened;
- part of it lies farther than r(chordal) from the rings, i.e. its leftover would exceed the
  chordal tolerance (decision 4): it meets (W ⊖ (r(chordal) + margin)) ∖ E. When
  r(chordal) ≤ s/2 every patch qualifies; otherwise a patch whose points all lie within r(chordal)
  of the rings is left alone, so a stretch left alone keeps at most the chordal tolerance.
Nothing here looks at the boundary's spans or junctions, or at skeleton branch ends (ADR-0023:
obtuse corners may have no branch); a corner, a fillet, a chamfer, a curved junction, a narrow
neck between two swept areas and the spike two close reflex corners make in the wall ring are all
just patches. For a straight-edged corner of angle α and an end mill of radius R the patch reaches
R·(1 − sin(α/2)) from the ring edges at the fillet's point on the bisector, so the threshold for
it is h(R·(1 − sin(α/2))) > chordal, as round 1 predicted; the polygon of E decides it within its
chords (chordal/64).

**Moves (code: `stretchPasses`).** Each patch gets a frame: its root is its vertex farthest from E
(for a sharp corner, the wall ring's corner moved δ off both edges, on the bisector), its
direction u the unit vector from the root to the patch's centroid (for a corner symmetric about
its bisector, the bisector), x along u, y across it. Passes run along the lines y = k·s for every
integer k whose strip |y − k·s| ≤ s/2 meets the patch (k = 0 is the groove along the bisector).
The pass on line k covers [x₀ − b, x₁ + chordal], [x₀, x₁] the extent in x of the patch within
the strip, b = 2(R_max + ρ + c) back towards the wall ring, R_max the largest end mill's radius,
and the chordal tolerance on into the swept area; it is clipped to W (polygon clipping against
the rings' edges; points within 10⁻¹² of the part's coordinate scale of an edge, never more than
c/16, count as on it), and the pieces that meet [x₀, x₁] are kept, so the back extension stops
at the first ring edge and the centres stay within the wall ring, ≥ ρ + c from the walls (the
cone margin of §1 is c/2), and the cone at F never crosses a wall. Consecutive passes run in
alternate directions (zig-zag) and are joined at depth when the straight link lies inside W;
otherwise the cut ends with a retract and the next starts with a rapid at safe Z and a plunge
(§4). The corner cuts come after that part's other cuts, nearest first from where its last cut
left the tool; the next part orders its cuts from that same point, as without sharpening, so
every other move keeps its place (only the added corner moves differ); safe links and post
safety are unchanged.

**Why the leftover in a sharpened stretch is at most the floor's ridge h(s/2).** Let p be a floor
point of a sharpened stretch's flat area. (i) If p ∈ E, it is swept flat. (ii) If p ∉ E and
f(p) ≤ δ, a ring edge is within δ ≤ s/2. (iii) Otherwise p lies in a patch (the inset polygon
contains every point at least δ from the rings, and the swept polygon lies inside E); for a
sharpened patch, let k be the line nearest to p, so |y_p − k·s| ≤ s/2, and q = (x_p, k·s). p lies in
the patch and in strip k, so x₀ ≤ x_p ≤ x₁. If q ∈ W, the interval of the clipped line round q is
inside W and meets [x₀, x₁] (it contains q), so it is part of the pass: a cut at F within
|pq| ≤ s/2 of p. If q ∉ W, the segment pq leaves W and crosses a ring edge, a cut at F within
|pq| ≤ s/2. In every case the nearest cut at F is at most s/2 away, so the leftover is at most
h(s/2), the ridge between two passes of the carving tool's own rest clearing at stepover s.
Round 1 ended each pass a fixed overlap inside the swept area, which does not close the band
between passes k and k+1 where the end mill's fillet runs steeply outwards between them (acute
corners, outer passes): points there were up to 1.18 × h(s/2) from every cut. Taking each pass's
extent from the whole strip it serves, not from its own line, closes that band by construction;
the chordal tolerance beyond is the overlap the card asks for (decision 2), a margin on top.

**Short pieces and cost (T-0173).** A piece of a pass no longer than the user's distance tolerance
is dropped (it is not a cut worth a plunge): a stretch whose every piece is that short gets no
passes, and keeps at most the leftover its ring and the neighbouring cuts leave. Detection groups
the patches and the end mills' polylines by the overlap of their boxes (a polyline's inflated by
its radius) and offsets and differences each group alone: Clipper2's sweep over one call for a long
row of parts (a chain of squares joined by pinched necks, one region) costs more than the sum of
its parts (n = 100 / 200 / 400 / 800 adds 0.27 / 0.83 / 2.6 / 11.8 s; rotated 90° it was linear).
Groups share no edge, so each group's result is what the single call made of it, and the output is
byte-identical (checked on the chain at n = 30 and 100, rotated or not, and by the suite's goldens).
Point-in-ring tests for the flat area of a patch or polyline, and the wall rings' membership test,
cast their ray along the shorter side of the rings' box, so a long row costs the few edges across it.
After it the added time is 0.20 / 0.43 / 0.98 s at n = 400 / 800 / 1600 (×2.2 per doubling).

**Measured.** `v_carve_corner_test.cpp` asserts the bound with the exact oracle
(`v_carve_leftover.hpp`: the deepest tool surface over each point, in closed form for level
moves, by golden-section search for inclined ones) on a 0.04 mm grid over the floor round each
corner, and that the same floor keeps more than the ridge without sharpening: a square (90° V-bit,
stepover 0.25, 6 mm end mill, F 1 mm; ridge 0.125); a triangle (60° V-bit, 4 and 1 mm end mills);
a 30° wedge with a ball nose; a rotated square with an L-shaped hole and Raster clearance; 30° and
45° corners at chordal 0.002 (60° V-bit, stepover 0.3, 6 mm end mill: ridge 0.2598); a D (a line
meeting a convex arc); a triangle of concave arcs (13° cusps); 0.5 mm fillets; 0.2 mm chamfers.
The round 1 verifier's independent oracle (harness t0160v1 `lc`, about 150 cases, highest
leftover without → with sharpening in each corner's tent against h(s/2)): square, 90° V-bit,
stepover 0.5, 6 mm end mill: 0.500 → 0.2500 (ridge 0.2500); 60° V-bit: 0.500 → 0.3464 (0.3464);
6 mm ball nose: 0.048 → 0.0268 (0.0268); 30° corner, 60° V-bit, stepover 0.3, chordal 0.002:
0.500 → 0.2598 (0.2598; round 1: 0.3055); 120°-apex triangle's 30° corners: 0.500 → 0.2500
(round 1: 0.2746); Bezier blade (63° corners): 0.317 → 0.2500; concave-arc cusps: 0.499 → 0.2500;
arc fillets 0.0005–1.6 mm, 4- and 12-piece polyline roundings, chamfers 0.0005–0.2 mm: 0.500 →
0.2500; a 30° corner with a 0.1 mm hole in its tent: 0.502 → 0.2500. Every case with a sharpened
stretch is at or below its ridge; the exceptions are the flat areas no end mill enters (rectangles
4–7 mm wide and narrow triangles with a 6 mm end mill, a corner closed off by a diamond hole):
left alone, as above, unchanged from before T-0160. Every inserted feed, links included, keeps
ρ + c/2 from the walls except where it touches a ring edge that does not (Limits). The cone check
of §8 passes on all of them and over the AC1 sweep (sharpening is on by default there). Jobs
without a flat depth or without clearance tools are byte-identical to the output before T-0160 by
construction (sharpening does not run); with it on, the round 1 verifier's insertion check (the
output with sharpening is the output without plus complete inserted cuts at the flat depth;
clearance toolpaths and diagnostics identical) holds on 780 generated jobs with a flat depth and
clearance tools forced (126 with corner cuts), and 520 such jobs hash identically with and
without `-march=x86-64-v3` on GCC 15 (§11). Cost (Release, CPU, interleaved): 5,000 glyph
regions with a 0.5 mm flat depth and a 0.2 mm end mill, 38.4 / 41.6 s with sharpening against
32.8 / 35.6 s without (32.2 / 32.2 s before T-0160), 130,000 moves more (26 per letter): about
a fifth more, mostly Clipper2 offsetting the end mills' level moves (an end-mill swept area per
part) and the patch bookkeeping. AC2's cases (§9) have no clearance tools and do not run it
(the frame page: 37.4 s, 37.3 s before T-0160).

**Limits.** The stretch's direction comes from its root and centroid; for an asymmetric stretch
(a junction of a line and an arc, a neck) it is not a bisector in any exact sense, which the bound
above does not need. Patches in flat areas no end mill enters are left alone (decision 1 above).
A patch narrower than the grid (about 10⁻⁵ mm) is dropped by Clipper2 with its dust (ADR-0006
§8); such a patch keeps at most h(δ + 10⁻⁵). A pass that ends on a ring edge, or a link along one,
has that edge's clearance: where Pocket's polygon of the wall ring dips below the margin c/2 (by up
to about 5·10⁻⁷ mm at chords of its round joins, before T-0160 too), the corner cuts touching it
do as well, never by more than the ring cut itself (round 1 verifier's tentslit case: 1.5·10⁻⁸ mm,
the ring edge there 1.7·10⁻⁸ mm).

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
D-hole was carved ~1 mm deep). The round-3 D-hole 0.0025 mm from its circle that was refused
whole at chordal 0.01–0.1 (the hole ignored) is classified by this too and carved (T-0164 b,
regression test in `v_carve_test.cpp`).

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
bound of §1 (≤ chordal). The lateral check also runs with flat areas (T-0164 e): there the
carving tool's Pocket passes, at one Z per level with vertical lifts between them, are told apart
from the skeleton cuts by being level or vertical, so its inclined feed moves are checked. The oracle counts every span as a wall, so the sweep avoids edges that
coincide exactly (they cancel under even-odd). The verifier's independent harness
(`/home/chris/harness/t0153v2`, vc2 over its 16 families) after round 2: no real cone violation;
lateral shortfall ≤ 0.5·chordal on non-crossing input and ≤ 0.55·chordal on resolved crossing
input, except family 15 (exactly coincident edges), whose coverage numbers the verifier marks as
artefacts of the cancelled edges. Every shipped post's output passes the safety rules (inclined
G1 cuts allowed).
Known numerical excursion (T-0164 f, documented, not fixed here): verifier round 3 found the
carving tool 9.5 nm (the oracle's figure; 45 nm by the exact distance at the corner) inside a
wall at chordal 0.001 (repro2/nest_frame.txt, 60° V-bit, flat depth 0.3, no clearance tools).
The move is Pocket's, in the flat area of an "A": an edge of Pocket's tool-centre region that
joins a round join round the reflex corner under the crossbar to the straight offset of the
crossbar dips below the tool radius. Pocket keeps its centre region radius + allowance + chordal
from the walls and its own chords "about d less one join tolerance" (ADR-0017 §3–4); this edge
does not quite keep that. 45 nm is 4.5 % of the chordal tolerance there and far below machine
resolution. Fixing it belongs to Pocket's centre region (it would change Pocket's output), not to
V-Carve; reported to the director for a Pocket card. **Resolved by T-0169** (ADR-0017 §8a): the
region edge itself kept its margin; the move was a raster link from the round join to the
crossbar's offset that cut the corner between them, which Pocket's inside test accepted within
the distance tolerance. Pocket's moves now leave the centre region only by rounding (or where they
keep radius + allowance + chordal/2 from the polygonised walls); the case now keeps 1.66e-4 mm
(regression test in `pocket_junction_test.cpp`). V-Carve output changed in 15 of 1,040 cases of
the verifier's families with clearance end mills (flat depth forced) and 4 of 1,040 with the
families' own mix, diagnostics unchanged everywhere (T-0169 hash sweep).

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
  `MedialAxisCapped`, `CrossingVectorsResolved` (T-0164: `StepoverLimited`, §3).
- `ToolpathParams` gains `VCarvingParams` as its fifth alternative (`"operation": "v_carve"`);
  each toolpath of a V-carve is its own record with the same params and its own tool snapshot
  and tool source. job.json params (all optional, unknown keys dropped, numbers finite): `name`,
  `start_depth`, `flat_depth`, `safe_z`, `clearance_strategy`, `clearance_direction`,
  `clearance_raster_angle_rad`, `ramp_plunge`, `ramp_distance`, `corner_sharpen` (T-0160, §6; a
  missing key reads as true), `order`, `start_position`, `tolerance`; params.json (`carvetto-cli toolpath v_carve`) uses `clearance_raster_angle` in
  degrees and adds `tool`, `clearance_tools` (an array of tool objects) and `layers`.
  `nonFiniteParam` names the keys the same way. The format stays 1.1 (a new optional variant
  alternative, ADR-0021).

### 11. Identical floating-point arithmetic on every compiler
The V-Carve goldens failed on the ubuntu-26.04 runner (GCC 15.2, Release), while GCC 13 and MSVC
matched them. GCC 15 there targets x86-64-v3 and contracts a * b + c into one fma; with
`-march=x86-64-v3` the failure reproduces exactly, and `-ffp-contract=off` on `medial_axis.cpp`
plus `span.cpp` alone removes it (`LineSpan::pointAt`, a + (b − a)·t, places boundary samples one
ulp apart). This is not a knife edge that a tie-break can fix: the point-sample medial axis
(ADR-0023) feeds sample coordinates to exact Delaunay predicates and compares hundreds of computed
distances with refinement thresholds, so one ulp in one sample changes which samples are added and
the skeleton's points everywhere after. Byte-identical output needs identical arithmetic, so the
core and Clipper2 are compiled with `-ffp-contract=off` on GCC and Clang (core/CMakeLists.txt);
MSVC's default `/fp:precise` never contracts, and builds without FMA cannot. Explicit `std::fma`
(the exact predicates' TwoProduct) is unaffected. Outputs of GCC 13 and MSVC builds are unchanged
(no golden changes); a GCC 15 FMA build now produces the same bytes. Tests stay free to be
compiled either way (T-0159's contraction-independent tests keep guarding the code itself).

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
