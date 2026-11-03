# ADR-0016: 2D Profile toolpath

Date: 2026-10-26 · Status: accepted 2026-11-01 (T-0120; amended in place by T-0124, 2026-10-30/11-01, and T-0127)

## Context
T-0120 asks for `profileToolpath(contours, Tool, ProfileParams)` with the options of
feature-inventory §5 "2D Profile Toolpath" (sides, direction, allowance, separate last pass, start
points, tabs, ramps, leads, order, V-bit corners), multi-pass via `PassPlan`, inner vectors first,
gouge-free and deterministic. The card fixes the behaviour to test but not how vectors combine,
how tabs/ramps/leads interact, or how a cut that cannot be made is reported. This records those
choices. Behaviour descriptions come from `docs/reference/` only.

## Decision

### API
1. `profileToolpath` returns `ProfileResult { Toolpath toolpath; std::vector<ProfileDiagnostic>
   diagnostics; }` rather than a bare `Toolpath`, because AC2 needs "no toolpath with a clear
   reason" and a profile over many vectors can partly succeed (one hole too small for the tool,
   one lead shortened). Diagnostics carry a `ProfileIssue` code, the input vector index and a
   message. An empty toolpath always has at least one diagnostic. (T-0123, ADR-0018: the types
   are the shared `ToolpathDiagnostic`/`ToolpathIssue`, of which `ProfileDiagnostic`/`ProfileIssue`
   are aliases; `CutDirection` is shared with Pocket.)
2. The tool is a `tooling::Tool`; feeds, speeds, tool number and the default pass depth come
   from `ProfileParams::cuttingData` or else the tool's `defaultCuttingData`. The offset radius
   is half `cuttingWidthAtDepth(tool, bottom of cut)`, so for a V-bit the top edge of the cut
   lands on the vector; for end mills it is D/2.
3. Shared machinery lives in its own headers so other operations (Pocket, T-0121) can reuse it:
   `path_walk.hpp` (arc-length paths, piecewise-linear height profiles, `appendLeg`),
   `clearance.hpp` (distance-to-vectors checks), `tab_placement.hpp`, `ramp_entry.hpp`,
   `lead_moves.hpp`, `cut_order.hpp`. `profile.hpp` holds only the operation.

### Geometry
4. **Region semantics.** Closed vectors are regions by nesting parity (a vector inside an odd
   number of others is a hole). They are oriented region-on-the-left (outer CCW, hole CW, island
   CCW). Outside keeps the region: the tool runs outside outer boundaries and inside holes;
   Inside runs inside the region. Open vectors: Outside = right of travel, Inside = left (the
   enum has `Right`/`Left` aliases). On runs on the vector (offset by the allowance only). An
   allowance of exactly −r also puts the cut on the vector; the profile side still decides the
   direction of cut and which side is the part (T-0127; before, Inside turned into Outside there).
5. **Exact offsets first.** Each vector is offset by r + allowance with
   `geometry::offsetOpenContour` (the public API as on main; closed contours are accepted), so
   lines and arcs stay exact: an inside profile of an r = 10 circle with a 6 mm tool is one
   r = 7 arc move (AC2). Joins are Round; Sharp (mitre limit `corners.mitreLimit`) for a V-bit
   with "sharp external corner". A chain whose ends meet is treated as closed (the API returns a
   full circle as one open span).
6. **Neighbours.** If an exact offset of a closed vector comes within r + allowance − tol of
   another closed vector (checked by `findViolations`, below), the vectors are grouped
   (union-find) and offset together as one region: even-odd union of the group
   (`unionContours`, so every edge is a region boundary even where vectors cross), then
   `offsetContours` by ±(r + allowance + chordal), the sign flipped when the group lies inside
   an odd number of other vectors. The extra chordal keeps the polygonal result at least r from
   the true curves. These cuts are polygonal (`MergedWithNeighbour`). A closed vector whose own
   offset comes back as open chains is handled the same way.
7. **Safety net.** Every cut is then checked against every other vector (merged cuts also
   against their own members) and stretches closer than r + allowance − 0.05·tol are removed
   (`TrimmedNearVector`); a trimmed loop becomes open pieces. Exact offsets are not re-checked
   against their own vector (ADR-0012 guarantees it; see Consequences). On profiles skip 6–7.
8. **Clearance check.** Distance along a path is 1-Lipschitz in arc length, so an interval
   whose end samples both exceed threshold + half its length is certified clear; others are
   bisected down to a resolution (0.05·tol for the safety net and the lead checks, so what is
   kept is within 1.5 × 0.05·tol of the threshold; tol/2 for grouping). Violations are bracketed
   by the neighbouring clear samples (removal errs on the safe side). Obstacles are pre-filtered
   by bounding box.

### Machining
9. **Direction.** Assuming an M3 (clockwise) spindle, climb keeps the vector on the right of
   travel (outside profiles run clockwise, inside counter-clockwise); conventional the reverse.
   Open vectors cut On keep their own direction for climb.
10. **Order.** Containment tree over the cuts (parent = smallest closed cut containing a probe
    point); post-order, so inner cuts always come first. Siblings are ordered by the enabled
    strategy: selection order; left-to-right (bbox left edge, then bottom: column by column);
    bottom-to-top (row by row); grid (rows bottom-up, a row = cuts whose y-extent overlaps the
    row's first, serpentine). Several enabled → the one with least rapid travel (ties: that
    order). None → greedy nearest neighbour from `startPosition`. Each cut is machined through
    all levels before the next. With a separate last pass (owner decision: after all roughing), all roughing
    cuts come first; the finishing cuts then follow the roughing order: each takes the rank of its
    counterpart (the roughing cut with the smallest median distance over 17 points along it,
    when that is within the last-pass allowance + a tool radius); a finishing cut without one
    (a small loop that vanishes at the roughing distance) takes the rank of the roughing cut
    nearest its start. Finishing is never re-ordered on its own.
11. **Start points.** `StartAt::VectorStart` (= "Use Vector Start Point"): the cut point nearest
    the vector's start; `Optimise`: nearest the previous cut's start; `ClosestOnBoundingBox`:
    nearest one of nine anchors of the cut's bounding box. Ties follow `closestPoint` (lowest
    span). The closed cut is rotated to start there.
12. **Height profiles.** A pass is a list of legs (`PathLeg`: walk the path from u0 to u1, either
    way, with a piecewise-linear Z over travelled distance). Lines become feeds, arcs (helical
    where Z changes, exact because Z is linear in arc length between knots), profile jumps
    vertical moves. Every vertical descent is a Plunge and every Plunge is vertical; climbs are
    Feeds; sloped descents (ramps, 3D tab ramps, V-bit corner returns) are Feeds or arcs. A leg
    starts with a vertical move wherever the tool stands within 2·10⁻³ mm of the leg's start
    (closed cuts are also snapped to close exactly), so without a ramp a closed cut steps down
    vertically at its start and every level is a full flat loop; nothing descends along the
    first span. Tabs are a ceiling profile combined by point-wise max, so ramps, tabs, overcut
    and passes compose without special cases. Arcs with a chord under 0.01 mm are written as
    feeds (a controller would read them as full circles once rounded).
13. **Ramps** (`planPass`). Closed: Smooth descends over L (distance, or drop/tan(angle)) and
    continues until L past the start, so the ramp is re-cut at depth; the next pass starts L
    further on. Zig-zag goes back and forth over L (angle mode adds leg pairs so no leg exceeds
    the angle) and returns to the start, then cuts the loop. Spiral descends one pass depth per
    revolution, plus one flat revolution at the bottom. Open: Smooth/zig-zag ramp over the first
    L, return to the start at depth, then cut the whole pass; spiral falls back to smooth.
    "Ramp on lead-in" descends along the lead-in on the first pass. Ramps run at feed rate; the
    first plunge goes only to the top of the cut. The separate last pass never ramps (the slot
    is already cut).
14. **Tabs.** Tab length is the material left along the vector, so the raised stretch on the
    tool path is length + tool diameter (exact on straight edges). Tab top = bottom + thickness;
    only passes below it are affected. 3D tabs ramp down at 45°, i.e. over `thickness` on each
    side of the raised stretch, so a tab's **footprint** along the path is raised + 2·thickness
    (3D) or raised (2D). Constant number: centres at (i + ½)·L/n from the cut start; constant
    distance: n = round(L/distance) ≥ 1. "Avoid corners" moves each tab to the nearest place
    whose raised stretch lies on one line span (inside the allowed range below). `fitTabs` then:
    - keeps every footprint inside [0.001, L − 0.001] mm: on a closed cut a tab across the start
      is moved along the cut until it clears it (the tool plunges, and the lead arrives, at the
      start); on an open cut a tab past an end is dropped;
    - moves footprints off `keepClear` points (to the nearer side), dropping them when there is
      no room. With a separate last pass these are the other pass's cut starts projected onto
      the cut (within a tool radius + allowance), so neither pass plunges into the other's tab.
      A moved tab keeps two tolerances + 0.01 mm more from the point (room for its roughing
      cover, which is its projected ends widened by the tolerance; that is enough on straight
      stretches, but where the cover is extended round a corner, or the other pass's start lies
      across the mapped stretch, the tab can still be refused, see below), and
      a footprint within 1e-9 mm of the gap counts as clear, so a moved tab is never dropped by
      rounding; it is dropped when the move lands within the gap of another keep-clear point or
      outside the allowed range, and the tabs are then fitted in order along the path as before
      (T-0135;
      before, the tab went to exactly the gap and the re-check decided by rounding, about half the
      time either way, and a kept one could never be matched past the roughing start);
    - drops a tab whose footprint would come closer than a tool radius to the previous one, or
      to the first across the start, so neighbouring tabs never merge and every pass still
      reaches its level between them;
    - drops every tab when one footprint does not fit the path at all (a hole smaller than a
      tab is cut through without tabs).
    Tabs are explicit raised stretches (`TabSpan`, arc lengths on the cut). **Separate last pass
    (amended by T-0124):** tabs are placed once, on the finishing cut, and every finishing tab is
    carried onto each roughing cut that passes it: the tab's two ends and its middle are
    projected onto the roughing cut and the stretch between them (through the middle, either
    direction) is taken, widened by the tolerance; projection along the normals keeps the angle
    on arcs (this also fixes the 0.3–0.6 mm short roughing tabs on curves). A tab's **material**
    is described by cross-sections along its core (the finishing raised stretch less a tool radius
    at each end): every 0.1 mm, plus a fan every 5° at a corner of the cut (where the material
    runs diagonally across the corner), each running along the normal from the vector out to the
    far wall of the widest kerf (a tool diameter beyond the vector-side wall, plus the last-pass
    allowance with a separate last pass), less 0.02 mm at each end. Projection alone does not keep
    the roughing tool off that material: round a concave curve (Inside of a circle) the roughing
    cut is shorter than the finishing cut, and past a corner the path turns back towards the tab.
    So every roughing position from each mapped end out to 2·radius + allowance beyond it is
    tested against the cross-sections, and the end is moved past the furthest one that comes
    within a tool radius (0.05 mm steps, both ends by the same amount so the stretch stays
    centred, at most 4·radius + allowance; beyond that the match is refused). The match is
    refused when the projections are ambiguous (mapped length off by more than 2·(allowance +
    radius), or a point along the tab projecting outside the mapped stretch: a nearby lobe), when
    the cut passes the tab only in part, or when the mapped stretch would cover a roughing start,
    a finishing start projected onto it, or another roughing tab. With a negative allowance
    Inside/Outside the cross-sections cross the vector with the kerf (below), and round a concave
    corner of the part the roughing cut on the other leg reaches their far ends, so the cover grows
    into a start or another tab; a tab refused that way is tried once more with the cover placed by
    sections from the vector out (as for allowance >= 0), and the exact sided check below keeps it
    only if it holds (T-0135; T-0127 left such tabs out as "could not be matched on the roughing
    cut"). A refused match drops the finishing tab too, so both passes always agree; and because a
    refused tab may have kept out a neighbour when the tabs were fitted (they overlapped), the
    finishing tabs dropped by fitting are then added back where they clear every tab still kept (the
    kept ones stay where they are), and when one comes back that cut is matched again (T-0135,
    "add-back"). A tab added back can itself fail the tab check below where the plain result would
    have held, so whenever add-back changed anything, matching and the tab check are run once more
    without it, and the add-back result is kept only if it ends with more kept tabs (a tie keeps the
    plain result): add-back never leaves fewer tabs than matching without it. Finally every tab group (a tab and its
    roughing cover) is checked against all cuts of both passes (and without a separate last pass
    the only pass against itself): if any cut position outside the kept raised stretches comes
    closer than a tool radius (less 1 µm) to the tab's cross-sections — the opposite wall of a
    slot, a lobe of the roughing cut coming back across the far part of the kerf, a nearby
    vector's cut — the group is dropped, repeatedly until stable. The tab's own cut within two
    tool radii of its raised stretch is treated apart: on any convex curve, and round a corner of
    the cut, the tool at the plateau ends always nicks some cross-sections near the ends of the
    core, which only narrows the tab, so there the tab only has to stay joined to the waste beyond
    the kerf. Quick answer: a run of at least three cross-sections with no tool of its own cut
    within a radius + 0.1 mm, whose ends lie ≥ 0.2 mm apart at both walls, and whose middle one,
    carried on 0.4 mm past the far wall, reaches the waste beyond the kerf (0.2 mm past the widest
    kerf from the vectors) with no open tool within a radius + 0.1 mm of it. Otherwise a raster
    (0.05 mm cells, or a fortieth of the radius for big tools) round the core, reaching a tool
    diameter past the widest kerf, decides: cells are told apart by the distance to the vectors;
    on the vector side only the tab's material counts (cells whose nearest core point lies along
    the normal within the cross-sections' extent, so round a corner of the cut the material across
    it counts too), on the waste side every cell; every open tool position cuts the cells within
    its radius plus 1.5 cells; the tab holds when uncut cells join the material next to the vector
    to the waste beyond the kerf (or when there is no such waste near it at all: a hole smaller
    than the kerf). This distance rule serves Inside/Outside with allowance ≥ 0, where the kerf
    lies wholly on the waste side of the vector. **Sided tabs (T-0127).** On the vector, and for Inside/Outside with a negative
    allowance (the kerf then crosses the vector, so the distance to the vectors no longer tells
    the part from the waste), a tab is judged by the pieces of material the cut leaves, decided by
    the profile side and the allowance (never by `Cut::distance`), with no cross-section shortcut:
    the layer just below the tab top, where every tool position cuts a disc of the tab-layer
    radius (a V-bit's cone there: tab thickness × tan(half angle), at most the radius at the
    bottom). On a raster (0.05 mm cells) round the tab's raised stretches (both passes) out to two
    radii + 0.6 mm: the pieces are the cells no tool position cuts with every tab lowered, this one
    too; the tab's own material is the cells only its own lowered stretches would cut; a component
    of its material joins the pieces it touches. A piece counts when it covers 0.01 mm² and is
    thicker than 0.05 mm somewhere; the tab holds when it joins every counting piece within the
    radius + 0.3 mm (+ 2 cells) of its raised stretches (their exact ends included) into one, and there are at least two. (The
    cross-sections, which place the roughing cover, run across the whole kerf here: a tool radius
    to each side of the cut, plus the last-pass allowance on the side where the roughing cut runs:
    away from the vector, but towards it On with a negative allowance.) When the tab fails on that
    raster but two groups of the near pieces it leaves unjoined reach the raster's frame (they may
    be one piece joined beyond it), it is judged again on rasters reaching twice and four times as
    far, with the tools there (T-0135: a tab between two vectors was dropped as cut through although
    the flood shows it holds, and its part came free). The
    other live tabs count as cut, so a tab holds on its own and never borrows another tab's bridge.
    This is judged with the tools exactly as wide (+ 0.01 mm, which closes hairline gaps between
    cell centres) and again widened by 0.055 mm (on 0.05 mm cells, and where that fails on 0.025 mm
    cells), where the tab must not leave a near
    piece unjoined: every kept tab joins the material on both sides through a neck of about 0.1 mm
    or more, and leaves no counting piece near it loose. With fewer than two counting pieces the
    tab would hold nothing (a loop or arm narrower than the kerf, the tip of an arm the kerf
    consumes): it is dropped and reported as such ("would hold nothing"); a tab that leaves a near
    piece unjoined is dropped as cut through. Tool positions every k-th sample along the cuts (at
    most sqrt(0.04·r) apart), their discs grown by the sagitta of the gap (≤ 5 µm), stand for the
    sweep. Two exact quick answers skip the raster: when every tool position within 2r + 0.7 mm
    (plus the sample gap) of the raised stretches lies on one of the tab's own cuts close to the
    stretch along it (within 2.5 times that distance), and the cut there either bends gently (no
    corner: between samples 0.25 mm apart at most 2° or the turn of an arc of radius 2r + 1 mm), or
    turns at most 120° over any 2r + 1 mm and a run of at least three cross-sections of the tab's
    material, with their middle one carried 0.4 mm past both walls, stays clear of every tool by the
    widened radius + 0.06 mm; in the first, the core of every raised stretch of the tab (less
    r + 0.17 mm and the sample gap at each end) is clear of every tool by r + 0.12 mm and the gap
    (a roughing cover shorter than its finishing stretch leaves roughing tools there). Then no other cut
    reaches the material within r + 0.3 mm of the
    stretch, the cut leaves exactly its two sides as counting pieces there, and the tab's material
    joins them. **Loose pieces.** A kept tab holds the material next to it, but a piece elsewhere
    (the kerf consuming a neck of the part, a remnant between the finishing and roughing kerfs at a
    corner) may have no tab: after the tab check of sided tabs (On the vector, or Inside/Outside
    with a negative allowance; not for Inside/Outside with allowance >= 0, a known gap), whenever
    tabs are on (also when every tab was left out), pieces of material inside a closed primary cut, or inside a cluster of open primary cuts (a
    loop trimmed into pieces near another vector; boxes within two tab-layer radii + 0.5 mm of each
    other, searched as one region, T-0135), that join the outside through no kept tab
    are looked for and reported (known gaps below). Exact construction (T-0135; T-0127 used a
    raster of sampled tool positions and skipped it when one hole had a kept tab beside it, which
    missed real loose pieces): near the region are the cuts of both passes whose boxes reach within
    the tab-layer radius + 0.1 mm of the region's box (grown by the radius + 0.5 mm). The material
    the cuts leave at the tab layer is what the bands of their open stretches leave (each cut path
    less its raised stretches, the exact stretch ends kept, grown on both sides by the tab-layer
    radius + 2 µm + at most 5 µm); a loose piece is a hole of that union (bounded component of its
    complement) lying inside the box. Soundness: the bands are exactly what those tools cut at that
    layer (the disc model of the tab check); a hole of the union is bounded by cut all round, so its
    material joins nothing outside it, and the kept tabs' material, uncut, is part of the holes it
    lies in, so a piece a kept tab joins to the outside is no hole. Cuts that are not near cannot
    reach into the box, so a hole inside it is a hole of the whole scene's union too (other cuts
    could only cut it further, never join it to anything): every reported piece is loose, and every
    loose piece inside some region's box is found. When even with every tab lowered (full paths) the
    union has no hole inside the box, nothing is enclosed there and the open union is not built. A
    piece's area is the hole's less the islands inside it (the band of a hole in the part, with
    that hole's slug, itself a hole of the union, inside it), and it counts when it is 0.04 mm² or
    more and thicker than 0.05 mm somewhere (a hole of the union with the bands 0.025 mm wider lies
    inside it and none of its islands). A loose piece of 0.04 mm² or more, thicker
    than 0.05 mm somewhere (a hairline sliver between two kerfs does not count), is reported
    against the vector of the innermost closed primary cut that contains it, or else of the nearest
    cut ("a piece of material inside the cut (about A square mm) is not held by any kept tab and comes free"), one
    message per piece, numbered per vector, largest first ("vector 3: loose piece 2 of 4: ...", at
    most 50 per vector, then one "and N more loose pieces (about A square mm in all)"), so
    the de-duplication in run() never merges two pieces (T-0135; before, equal rounded areas were
    merged and only five were listed). A piece is seen from every loop whose box holds it; it is
    the same piece as one already found when it holds that piece's inner point (one at least
    0.025 mm clear of the tools, so every view of the same piece holds it: inside the hole and none
    of its islands), so a piece in
    another's concave notch, whose box overlaps, is still reported. Known gaps: leads are not part of this
    check (a lead crossing the part is a separate issue); pieces between neighbouring vectors that
    reach out of the region's box are not seen (also pieces enclosed by open cuts that are not one
    cluster: a5_770001_273, a5_770001_60, missed by T-0127 too); the cut paths stand for the tool's
    moves, so links, plunges and ramps that cut more than the paths are not part of it, and a piece
    they cut away or split may be reported (m1_881008_35, also in T-0127); a tab whose two "pieces" in its window
    are one piece joined beyond the window (overlapping vectors, scene 401007_36) is kept although
    it holds nothing new (redundant, never severed); and a tab judged on a wider window (above) may join the main pieces beside it yet leave a sliver
    unjoined, which is then reported loose (a2_610000_162: 0.33 mm², 0.072 mm thick). Not a gap but the rule: a tab that leaves a
    piece near it loose is dropped even when a neighbouring tab holds that piece (each tab holds on
    its own; V-bit scene m2_403000_48, flat or 3D alike: a 59 mm² piece beside two tabs, which a
    keep-all comparison counted as one tab group). Cost: within a few per cent of T-0127 and main
    on the pages measured (500 close rectangles with a separate last pass and allowance −1; On the
    vector, 500 small shapes; pages of 500–2,000 open polylines). So a short tab centred on a sharp
    turn of the cut — a convex corner or spike tip, an inside corner (Inside of a square), a gear
    root, the tip of a narrow arm — where the tool at the two ends of the raised stretch seals the
    tab's material off from the waste, is dropped and reported (On a 30 × 30 square with a 6 mm
    tool the tools at the two ends of a corner tab's raised stretch stand (L/2 + r)·√2 apart and
    overlap for L < 2.49 mm; the tab is kept once the neck between them survives the widened
    check: from L = 2.8 mm at allowance 0 and −0.3, 2.6 mm at +0.3, where the corner of the cut is
    rounded), and so is one left hanging on the part by a sliver (verify rounds 2 and 3);
    tabs that hold — short tabs on straight edges, longer tabs on corners — are kept, for every
    allowance ≥ −r. (T-0124 merged with a conservative On guard that dropped every On tab whose
    raised-stretch ends came within 2r + 2|allowance| + max(0.25, 0.05·D) of each other; T-0127
    replaced it with this rule.) The check runs as a
    worklist: when a tab is dropped, only the tabs whose material lies within reach of the cut it
    protected are checked again (a separate last pass with tabs on 500 close vectors costs about
    1.3 × the time without tabs). Together: no pass cuts below the tab top into any cross-section
    of a kept tab except its own pass's nicks, which leave the material joined, so every kept tab
    joins the part to the waste beyond the kerf.
    Counting: kept + left out = requested per primary cut (the finishing cuts with a separate last
    pass, otherwise the only cuts), so per vector when its offset is one loop, and per loop when
    the offset splits into several (each loop gets the requested tabs; the diagnostic sums them); one `TabsNotPlaced` diagnostic per
    vector says "N of M tab(s) left out" with the count for each reason (no room, could not be
    matched on the roughing cut, would be cut through, would hold nothing); another reports tabs that could not avoid
    a corner. Where a V-bit corner climb is inserted at the start of a raised stretch, the tool
    goes back up to the tab top vertically after the climb (inserts end at the leg's height there;
    path breaks closer than the tiny length merge into the last of them), so a tab is always cut
    flat at its top.
    A tab thickness beyond the cut depth is clamped to it (`TabThicknessClamped`). The overcut
    walks under the same tab ceiling, and every lead runs no lower than the tab top when it comes
    within a tool diameter of tab material (any cut's), so plunges, leads and overcut never notch
    a tab. Interactive tab editing and "tab at machining start" are not implemented.
15. **Leads.** Straight (length, angle to the path) or circular (radius, sweep, tangent) from the
    waste side; optional mirrored lead-out; overcut (closed cuts) runs past the start before the
    lead-out. Leads are used on the first entry and final exit of each cut, not between levels.
    Each lead is checked against every vector with the clearance check at tool radius + allowance
    (allowance ≥ 0) − 0.05·tol, so it never cuts into the allowance (T-0124).
    Lead-ins are planned before tabs. At the chosen start the lead is tried as asked and as a
    gentler variant (straight: perpendicular to the path; circular: half the sweep), at full
    size and then halved up to three times (`LeadShortened`). If none fits (typically an inside
    profile starting at a corner, where the approach crosses the adjacent wall) the start of a
    closed cut moves to the middle of its start span, then of the other spans from the longest
    (`StartMoved`; owner decision 2026-10-26: keep this). Only when nothing fits is the lead left
    out (`LeadDropped`). Lead-outs try the same variants and sizes at the end point and leave at
    the height the cut ended at (the tab top when the overcut ends in a tab).
16. **V-bit corners.** Sharp external = Sharp joins (5). Sharp internal, on the final level, at
    an inside-corner kink Q of an exact offset where the path turns by more than 5° (merged
    polygonal regions, On profiles and the sub-degree kinks of polygonised curves get none):
    the target V is the nearest junction of a source vector whose direction turns by more than
    5° — a genuine corner of the input — and it must lie within twice the straight-wall distance
    (r + a)/cos(turn/2) of Q. The offset radius r is the cone's radius
    at the material surface (Z = 0) with the tip at the bottom. The climb ends at the point of
    Q→V whose distance to the walls equals the allowance a (V itself when a = 0), at Z = 0, and
    starts from the knee depth max(bottom, −r / tan(α/2)) at Q (a vertical rise first when the
    cut is deeper than the cone). That straight climb is exact between two straight walls; it is
    checked against the real walls (all vectors, exact distances, every 0.005 mm): the cone's
    surface radius min(depth·tan(α/2), R) must stay within distance − a. If it would pass a wall
    (a curved wall next to the corner), the climb follows the walls instead: every 0.05 mm the
    tip is held at (distance − a − 0.05)/tan(α/2) below the surface, which keeps the cone inside
    the walls between samples too, reaching the surface within one sample of the end. The way
    back mirrors the climb (sloped feeds, vertical plunges). Tested with a cone-vs-wall sampler at
    60° and 90°, inside and outside, start depth 0 and 2, allowance 0 and 0.3, a cone shallower
    than the cut, and arcs bulging 6 and 10 mm next to the corner. Both options are ignored with
    `CornerNeedsVBit` for other tools.
17. **Bezier input.** Beziers are polygonised before offsetting; when present the offset chordal
    tolerance is tightened to tol.distance/2 so the cut stays within tolerance of r.

### Golden hand-check (AC1 / AC6)
`core/tests/golden/toolpath/profile_rect_tabs.moves` and `profile_rect_tabs_grbl.nc` were
checked by hand: the offset of the 100×50 rectangle by 3 is the lines y = −3, x = 103, y = 53,
x = −3 joined by quarter arcs of r = 3 about the corners (every arc centre is a corner, CW for
climb); the start (0, −3) is the end of the bottom-left arc nearest the vector start (0, 0)
(the whole arc is equidistant; lowest span wins); levels −4, −8, −12 with vertical plunges at the
start; perimeter P = 300 + 6π = 318.8496, tab centres at P/4 and 3P/4 along the cut land at
x = 20.2876 on the top edge and x = 79.7124 on the bottom edge, so raised stretches
x ∈ [12.2876, 28.2876] on y = 53 and [71.7124, 87.7124] on y = −3 (16 mm = 10 + 6), at Z −10, on
the −12 pass only (computed independently in Python). The G-code was checked against the
`posts/README.md` checklist (G21/G90, safe Z before the XY rapid, spindle on before the plunge,
I/J = centre − start, plunge at F300 and feeds at F1000 restated, retract, M5, M30).

## Consequences
- Merged neighbours lose arcs; refitting belongs to the Fit Curves card. Grouping, nesting,
  trimming, containment and nearest ordering were pairwise O(n²) with box pre-filters; since
  T-0123 they query a `BoxIndex` and an indexed `ClearanceField` (ADR-0018), with identical results.
- The gouge guarantee against a vector's own exact offset rests on ADR-0012 and inherits its
  documented sub-tolerance-span limitation (T-0009); T-0017 changes offset quality, not this
  code. Tests keep inputs free of sub-tolerance spans and of Sharp joins on zig-zags.
- Open vectors with a sharp turn next to an arc whose offset nearly collapses can get a stretch
  of cut that a local "right of the vector" test reads as the wrong side: the round join about
  the vertex wraps round (ADR-0012, Consequences). It stays at least the offset distance from
  the vector (no gouge); it belongs to the offset (T-0017), not to this code.
- Tab placement is greedy (tabs that do not fit are dropped, the rest are not redistributed);
  evenly redistributing the kept tabs is future work.
- Not implemented: interactive tab editing and per-vector stored tab positions (needs the
  document/UI side), "tab at machining start point", leads between levels, cutter compensation,
  projection onto a 3D model.

## Owner decisions (2026-10-26, via the director)
Confirmed as proposed: tab length is the material left (raised path = length + tool diameter,
14); the separate last pass runs after all roughing (10); nested closed vectors form a region,
the inner one a hole (4); climb assumes an M3 spindle (9); ramps run at feed rate (13); 3D tabs
ramp at 45° (14); leads only on the first entry and last exit (15); left-to-right orders column
by column (10).
