# ADR-0016: 2D Profile toolpath

Date: 2026-10-26 · Status: accepted 2026-11-01 (T-0120; amended in place by T-0124, 2026-10-30/11-01, T-0127, T-0129 and T-0141)

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
   **Allowance limit (T-0129, owner decision 2026-11-01).** An allowance below −r (r as in 2:
   half the cutting width at start + cut depth, a V-bit's cone radius there) would put the tool
   centre across the vector, so Outside would cut like Inside (and the reverse) and On, Left and
   Right would swap sides. `profileToolpath` refuses it: no moves, and one `InvalidParameters`
   diagnostic (no vector) naming the allowance and the limit, e.g. "allowance -4 mm is below
   minus the tool radius (-3 mm at the cut depth): the tool would cross the vector to its other
   side; nothing is cut". Numbers print in the shortest form that reads back exactly, so an
   allowance one ulp below the limit never prints as the limit. The rule applies to each pass's
   distance: the allowance (finishing, or the only pass) and, with a separate last pass, the
   allowance + last-pass allowance (roughing). Since a negative last-pass allowance is already
   refused and rounded addition is monotone, the roughing distance is never below the finishing
   one, so only the allowance decides in practice; the message says which passes cross. When only
   the finishing pass crosses (allowance < −r ≤ allowance + last-pass allowance) the whole
   operation is refused, roughing included: emitting the roughing alone would leave a part that
   looks machined but is a last-pass allowance oversize, with nothing in the toolpath list to
   show the finishing is missing (conservative; the user fixes the allowance). **Tolerance:**
   none. The test is `allowance < −r` in doubles; in IEEE arithmetic r + a == 0 exactly when
   a == −r and r + a < 0 exactly when a < −r, so the boundary is the same one `buildCuts` keys
   the side on (s == 0 keeps the profile side, T-0127), and a roughing sum (r + a) + lpa with
   r + a ≥ 0, lpa ≥ 0 can never round below zero. Exactly −r stays valid; one ulp below is
   refused. A tolerance band would either accept allowances whose offset lands (by a sub-ulp
   amount) on the wrong side, or need snapping values just below −r to −r, which buys nothing
   for typed values (−D/2 is exact for any D, since halving is exact) and for computed V-bit
   radii only shifts the edge. Output for every allowance ≥ −r is unchanged.
   **Upper limit On the vector (T-0137, owner decision 2026-11-02).** On the vector the allowance
   alone is the tool centre's distance from it (r is not added), so an allowance above +r runs the
   tool entirely off the vector, just as one below −r does. On refuses it the same way: no moves,
   one `InvalidParameters` diagnostic (no vector), e.g. "allowance 4 mm is above the tool radius
   (3 mm at the cut depth): the tool would run entirely off the vector; nothing is cut". Again each
   pass's distance counts. On, the roughing distance allowance + last-pass allowance is the larger
   one and can be the only one above +r (allowance ≤ r < allowance + last-pass allowance): then the
   message names the roughing, and the whole operation is refused, as above: the toolpath is
   either every pass as set or nothing, never a silently partial one (the finishing pass alone
   would cut the full depth at once where the user asked for roughing first). The −r check runs first, so a setting that breaks
   both limits (allowance < −r, allowance + last-pass allowance > +r) gets the −r message. The test
   is `allowance > r` (and `allowance + lastPassAllowance > r`) in doubles, the same doubles
   `buildCuts` offsets by, with no tolerance: exactly +r (the tool's edge on the vector) stays
   valid, one ulp above is refused. **Left/Right (decision of the T-0137 coder, AC1).** Open-vector
   Left/Right are the Inside/Outside enum values and offset by r + allowance to their side, like
   closed Inside/Outside: a positive allowance keeps the tool on the profile side and only moves it
   further from the vector, an oversize cut (e.g. leaving stock along an edge), never a side swap.
   So they keep only the −r limit, as Inside/Outside do; the +r limit is for On alone, where the
   vector is the cut's centre line. Output for every |allowance| ≤ r (and every Inside/Outside/
   Left/Right allowance ≥ −r) is unchanged.
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
    span). The closed cut is rotated to start there. Where the target has several equally near
    points, the plain nearest point follows rounding (the direction of a near-zero vector, or which
    of two equal distances comes out smaller), which differs between compilers, FMA contraction
    and libm. Two such ties are broken by rule (T-0158, owner decisions 2026-11-03 and
    2026-11-05), comparing only distances, each within the distance tolerance:
    (a) *Round corner*: the target lies at the centre of an arc of the cut that is at the nearest
    distance — a convex tip's round join on an Outside cut (VectorStart), a reflex vertex's join
    on an Inside cut, a circle from its bounding-box centre or Optimise from its centre. The cut
    starts at the beginning of the first such arc (lowest span index), so it runs the corner arc
    first; the arc start must itself be within the tolerance of the nearest distance.
    (b) *Sharp corner*: the target lies on the vector (its nearest distance is the distance the
    cut was actually offset by: the cut distance, or for a merged region — vectors offset together,
    or a vector whose exact offset did not close — the cut distance plus the chordal tolerance, or
    up to the chordal tolerance less where a region's polygonal round join cuts inside). On a V-bit
    cut with sharp external corners a convex tip has no join span: the offset lines meet at the
    mitre, and the tip's feet on both are equally near. Squared off beyond the mitre limit, the
    chamfer lies at the offset distance from the tip, so both edges and the chamfer are equally
    near; a merged region's round join is a polygon whose chords are all equally near. The cut
    starts at the corner built about the target vertex itself (owner decisions 2026-11-05), found
    from the target's own two input edges (the edges into and out of the vertex; a Bezier edge by
    its chord at the vertex, as the offset polygonises it), without comparing turn angles or edge
    lengths (T-0158 round 5):
    - The spans of the cut on each edge's offset — every span whose start, middle and end lie at
      the offset distance from that edge's line or circle, to rounding (4 × 1e-10 × the coordinates
      on an exact offset, 4 × 1e-6 mm on Clipper2's grid for a region, each × (2 + 2 × how far
      beyond the edge the point lies / the edge's length), since a line's direction comes from its
      end points; against a circle 8 × that rounding and 4e-12 × its radius, plus the chordal tolerance on a region) — bound the corner. In
      travel, from the last span on the offset of the edge met first to the next span on the other
      edge's offset, what lies between is the corner: nothing (the mitre), the chamfer, the join
      arc, the chord an exact offset puts in place of a join arc shorter than the tolerance, or a
      merged region's join chords. Of such places the one holding a span at the nearest distance
      (there must be exactly one); the cut starts at its first junction: exactly the mitre's corner point whichever way the cut
      runs, the chamfer's first corner in the direction of travel (also between curved edges), and
      the join's first point in travel (the target's foot on the first edge's offset), as a true
      round join starts at its arc's beginning. A span within that rounding of both offsets (a
      sliver of the first edge's offset split off next to the mitre, or a join chord too short to
      tell) counts as on either; then the start is the first junction among such spans that lies on
      the circle of the offset distance about the target (within 3 grid units on a region, 4e-10 ×
      the coordinates on an exact offset), else the one after them.
    - Every other junction is bounded by a span that is not on the target's own two offsets:
      neighbouring vertices — collinear (redundant), nearly collinear, short or sharp — and split
      points of the offset lines never take the start, however near they are; a finely segmented
      outline starts at the start vertex's exact mitre, and a region vertex a few µm from the target
      no longer moves its join's start.
    - Where no such corner is found (the target is no vertex; an edge's offset is consumed next to
      the corner; the two edges are collinear, so the vertex has no corner) the earlier rules
      apply, provided the spans at the nearest distance form one run of two or more consecutive
      spans (when every span of the cut is near, a loop of two or three spans, the loop is broken
      where consecutive feet lie farthest apart, if clearly the widest gap): with round joins, the
      first point of the run in travel on the circle above, backing up over the join's last,
      shorter chord where the cut runs the join that way; otherwise, of the run's junctions at which
      the target is equidistant from the carriers (lines, or circles for arcs) of the two spans
      that meet there, to within 4 times their rounding (1e-10 × the coordinates × (1 + offset /
      span length) on an exact offset, 1e-6 mm × (1 + offset / span length) on a region), the one
      whose feet of the target on its two spans lie most evenly either side of it (with none
      equidistant, of all junctions). A target at a redundant vertex thus starts at its own foot.
    A target nearest one junction starts there, as before. Limits: an input edge shorter than the
    distance tolerance is dropped by the offset (ADR-0006), so at that length the two corners at its
    ends become one corner and the start moves with the geometry; likewise a merged region whose
    vertices are about the tolerance apart can change shape (which spans Clipper2 keeps) under
    1e-9 perturbations. A sliver or chord within the rounding of both offsets moves the start by at
    most its own length (its length × sin(turn) is within that rounding: sub-µm). In the fallback
    cases above the earlier limits remain: with round joins, a vertex within about sqrt(2 × offset
    × 3e-6 mm) before the target (5 µm at a 3 mm offset) on a region, or sqrt(2 × offset × 4e-10 ×
    the coordinates) on an exact offset (0.5 µm at a 3 mm offset and 100 mm coordinates, 1.5 µm at
    1 m: below the distance tolerance only up to about 400 mm), can put its join's last point on
    that circle and move the start back by at most that vertex's distance; without, a neighbour's
    junction whose edge length × sin(turn) is within the rounding counts as equidistant and takes
    the start where its feet lie more evenly.
    Known limits accepted by the owner at merge (2026-11-05; follow-up T-0174): (a) a squared-off
    tip whose start vertex has a duplicate or sub-µm edge next to it labels no span on that edge, so
    the corner is not found and the fallback's "most even feet" ties exactly between the chamfer's
    two corners; rounding picks one, so the start can jump to the other corner (up to 4.3 mm at a 3 mm
    offset) under ulp or 1e-9 perturbations, and with a collinear edge of 1e-7–1e-5 mm both
    directions can start at the same corner (the last in travel for one of them). The DXF importer
    drops exact duplicate points; near-duplicates and API input reach this case (T-0158 round-5
    harness t0158v5 repro5/sq_tiny_*). (b) On a merged region an arc whose chord sag lies exactly on
    the labelling band (chordal slack) can flip the start by one chord (up to 1.7 mm; repro5/mrgarc_*);
    no stored, generated or random scene came within 7e-4 of the band. (c) Self-intersecting
    outlines (malformed input, cut as a region) are best effort: the start may switch between the
    corner rule and the fallback under translation (about 2 mm, repro5/selfx_*). The sliver bound
    above is sub-µm only up to about 2000 mm coordinates (1.56 µm at 3 m).
    Near spans in two or more separate places (a box-centre anchor on a symmetric shape) are not
    broken by rule and still follow rounding (T-0166). The rules apply to all three start modes.
    They are stable under ulp and 1e-9 relative perturbations and translations, and they compare
    distances only against bounds well above rounding, so builds that round differently (FMA
    contraction, another compiler or libm) choose the same start, except as listed above. They
    moved the start in about 9% of the stored harness scenes as stored (15% with their VectorStart
    / Outside, box-centre and Optimise variants) and 13% of random scenes (19% of random
    multi-vector pages), those whose start that rounding had decided.
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
      time either way, and a kept one could never be matched past the roughing start; that
      placement is still computed as one candidate, see the separate last pass below);
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
    "add-back"). None of these changes is better in every case: a tab added back can fail the tab
    check where the plain result would have held; a tab moved with the margin can fail matching
    where T-0127, which put it at exactly the gap and let rounding decide whether it clashed,
    matched it; and a tab matched only by the second try (cover from the vector out) can take room
    another tab's cover needed and still be cut through. So with a separate last pass three results
    are computed, each placed, matched and checked: exactly T-0127's matching (tabs placed as T-0127
    placed them, no slack or margin, no second try, no add-back); placed with slack and margin and
    matched with the second try, without add-back; and the same with add-back. The one ending with
    the most kept tabs is used, a tie going to T-0127's, then to the one without add-back. A scene
    thus never keeps fewer tabs than T-0127's matching (the tab check only adds to it: a tab judged
    again on a wider window), nor than the new placement without add-back; where they tie the
    output is T-0127's (the slack decides only when it gains a tab). T-0127's placement let rounding
    decide whether a tab moved to exactly the gap clashed with the point it was moved off, and that
    rounding changes with the compiler's floating-point contraction (GCC 15 at x86-64-v3 contracts
    a·b − c into one fma, T-0159), which lost a tab against the GCC 13/MSVC result. So T-0127's
    candidate is placed both ways, deterministically: such a tab refused (within 1e-9 mm beyond the
    gap) and accepted (within 1e-9 mm short of it); each is matched and checked, a tie going to the
    accepted one. The scene thus never keeps fewer tabs than T-0127 did when every moved tab rounds the same way (all refused or all accepted); a scene where rounding would mix the two is not covered, and other knife-edge rounding (start points, fitting) can still change counts between compilers (T-0159 verifier: 82 of 10,000 scenes differ between plain and x86-64-v3 builds, against 97 before). The
    extra runs happen only when they would differ: T-0127's when some cut's tabs are placed
    differently or a run used the second try (the refused variant only when it places the tabs
    differently from the accepted one), the one without add-back when add-back changed something. Finally every tab group (a tab and its
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
    lies wholly on the waste side of the vector. **Tab layer (T-0167).** The rule judges the
    tab at the layer just below the tab top, like the sided one: the cut there is the disc of the
    tab-layer radius (`tabLayerRadius`: a V-bit's cone there, tab thickness × tan(half angle), at
    most its radius at the top of the cut; any other tool's radius), far narrower than the radius
    at the top for a V-bit and a thin tab. Before, the cross-sections, the "cut through" distance,
    the quick answers and the raster all took the radius at the top, so a V-bit's tabs that hold
    were left out as cut through and the whole part came free (m2_1330000_9, Outside, V-bit 90°,
    D25, allowance 0, separate last pass: the single tab of each of two vectors, 641 and 1,704 mm²,
    at the stored start and at VectorStart; tab thickness 1.99 mm, so a cone of radius 1.99 mm at the
    tab layer where the radius at the top of the cut is 5.9 mm). Now the sections reach a tab-layer
    radius (plus the last-pass allowance) beyond the cut, the waste to reach lies a tab-layer radius
    + the allowance + 0.2 mm beyond it, the tools' discs in the raster and the quick answers' distances
    are the tab-layer radius, and the raster's core is the raised stretch less the tab-layer radius at
    each end (the tab is uncut for that much more at the layer; trimmed by the full radius, tabs that
    hold were left out). The sections that the other cuts are measured against keep the core trimmed
    by the full radius at each end, which the roughing cover is matched against: a tool of another cut
    beyond it only narrows the tab (measured against the longer core the first version left out
    holding tabs in six screen scenes). The nick test that picks the quick answers is unchanged. The
    raster's reach is a tool radius + allowance to the vector side and the tab-layer radius beyond the
    kerf, and the distance to the vectors is computed for blocks of 4 × 4 cells whose centre lies
    further than the block's radius from the thresholds (the distance is 1-Lipschitz; results are
    unchanged, cost is not: below). End mills are unchanged, moves and diagnostics byte-identical.
    **Not fixed (T-0177 (l)):** a tab round the bottom corner of a notch whose two kerfs leave a
    sliver of waste narrower than three raster cells is kept as holding "a hole smaller than the
    kerf" although the tools at its ends seal it off (m0_271016_20 at VectorStart/Outside: a 6.8 mm
    notch, end mill 3.175, 0.145 mm of waste, the end tools 2.7 mm apart; the part is held by the other
    tabs and the loose-piece search is exact, so nothing is lost). A first version of T-0167 looked
    for waste past the far wall of the kerf with the discs widened by half a cell. On the verifier's
    U-part family (end mill, Outside, notches leaving 0.06–0.5 mm of waste, 4,000 parts) it removed
    148 severed tabs but dropped 32 holding ones (slivers of up to 0.45 mm) and in one scene (seed
    9735 scene 49, a 0.079 mm sliver) left all five tabs out so that a 426 mm² part came free where
    main held it; it was reverted, because a resolution-dependent rule cannot be made safe. A rule
    from the exact capsule geometry (waste past the far wall of the kerf judged by the exact union of
    the tools, not by cells) is left to T-0177. **Sided tabs (T-0127).** On the vector, and for Inside/Outside with a negative
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
    most sqrt(0.04·r) apart), their discs grown by the sagitta of the gap (spacing² / 8r: 5 µm for a tab-layer radius r of
    0.0625 mm or more, more below it, as the spacing stays 0.05 mm: about 11 µm at r = 0.028 mm), stand for the
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
    corner, waste the kerfs close in between vectors) may have no tab: after the tab check,
    whenever tabs are on (also when every tab was left out), pieces no kept tab joins to the
    outside are looked for and reported, the same way for every kind of tab (T-0156: sided tabs
    were searched region by region, inside each closed primary cut and each cluster of open ones,
    which missed pieces inside no single region's box, such as the waste between the four bars of
    a "#" or between three circles whose kerfs overlap, and a roughing loop whose finishing cut
    had vanished; T-0140 introduced the whole-scene search below for Inside/Outside with
    allowance >= 0, and it now serves all). The whole scene is taken at once: the bands of every
    cut of both passes, less the kept raised stretches (their exact ends kept), enclose the loose
    pieces, wherever they lie (inside a vector or between vectors), leaf by leaf (enclosed_pieces.hpp).
    Each band is the cut stretch grown on both sides by the tab-layer radius + 2 µm; the polygons
    reach at most 5 µm beyond that where they approximate arcs, so the bands are 2 to 7 µm wider
    than what the tool cuts at that layer (not exactly as wide; T-0127's region search
    added the sagitta of its sampled tool positions on top, which is gone). Soundness: a hole of
    the union is bounded by cut all round, so its material joins nothing outside it, and the kept
    tabs' material, uncut, is part of the holes it lies in, so a piece a kept tab joins to the
    outside is no hole: every reported piece is loose, and every loose piece thicker than the
    bands' slack is found. A piece's area is the hole's less the islands inside it (the band of a
    hole in the part, with that hole's slug, itself a hole of the union, inside it), and it counts
    when it is 0.04 mm² or more and thicker than 0.05 mm somewhere (what is left of it eroded by
    0.025 mm is not empty, the erosion's round joins chords at most 0.005 mm inside their arcs).
    A loose piece of 0.04 mm² or more, thicker than 0.05 mm somewhere (a hairline sliver between
    two kerfs does not count), is reported against the vector of the innermost closed primary cut
    that contains it (any point of the piece gives the same answer), or else of the cut nearest the
    piece's anchor, its lowest vertex of least x, the same at every leaf size (it lies on the bands
    of the cuts round it, so cuts within 0.01 mm of the nearest count as tied and the first is
    taken; a roughing cut whose finishing cut vanished counts as a cut of its vector),
    ("a piece of material inside the cut (about A square mm) is not held by any kept tab and comes free"), one
    message per piece, numbered per vector, largest first ("vector 3: loose piece 2 of 4: ...", at
    most 50 per vector, then one "and N more loose pieces (about A square mm in all)"), so
    the de-duplication in run() never merges two pieces (T-0135; before, equal rounded areas were
    merged and only five were listed). For Inside/Outside with allowance >= 0 (T-0140; before, not
    checked at all, so a part or slug whose every tab was left out, or waste the kerfs close in
    between vectors, came free unreported) the kerf lies on the waste side of the vector, so each
    closed cut always encloses its vector-side material with every tab lowered; the search is the
    same.
    **V-bit corner moves (T-0156).** With "sharp internal corners" a V-bit makes a short excursion
    at each inside corner of a cut at full depth (the tip climbs from the cut towards the corner of
    the vector and back), which cuts the tab layer too: the cone's disc there shrinks from the
    tab-layer radius at the cut to nothing where the tip passes the tab top. Before, neither the
    tab check nor the loose-piece search saw these moves, so a tab they severed was kept and a part
    they freed went unreported (m2_1310149_15: a 134 mm² part, no message). They are planned with
    the cuts (before the tabs are judged; planExcursions, from the legs emit() walks, so a corner
    the cut passes only on a ramp has none), made only where no kept tab covers the corner (its
    raised stretch and, for 3D tabs, the ramps beside it), and swept as the convex hull of the
    cone's discs at the ends of each move. The tab check counts them as tool positions every
    0.02 mm along each move, with the disc radius there, among the cuts that a tab's lowering
    exposes (the corner under it) or that are cut anyway; the loose-piece search adds their hulls
    to the bands and builds every band when there are any (the proof that most stretches enclose
    nothing does not cover them). A raised stretch that ended exactly at such a corner used to be
    left along a slope, down to the cut's depth over the whole stretch (appendLeg merged the
    stretch's end into the corner's insert and took the height after it for the stretch), which cut the
    tab away; it now stays flat to the corner. The corner moves count in the tab check only where they really
    cut a tab through: a tab that holds without them and still joins two pieces with them is kept
    (T-0156 round 2: the first version dropped holding tabs, repro tabdrop_9001_11,
    tabdrop_570014_182), and a corner under another live tab is not counted for the tab being judged
    (it comes back, and that tab is judged again, if the other is dropped). **Known residual
    defects, identical on main (T-0156 verifier, harness t0156v1/repro, carded as T-0177):** a
    false "comes free" report on a held part (false939_7101_876: 939.84 mm² with one tab kept;
    7102_671: 7.6 mm²), six free 0.216 mm² pieces unreported (miss6_7104_351), and reported
    areas far from the flood's. New in the round-2 screen and also carded as T-0177: a false
    report in seed 9301 scene 2132 (Outside, D12, V90, allowance −0.666, leads off; moves identical
    to main, the branch reports 3 pieces of about 0.04 mm² where main reports none and the oracle
    finds none: knife-edge at the 0.04 / 0.05 mm thresholds; repro
    /home/chris/harness/t0156v2/dump/m1_9301_2132.scn). **Tabs against the T-0135 merge:** the scenes where tabs are fewer
    than at the T-0135 merge (48 in the verifier's screen) come from later commits (T-0158,
    T-0140) and are the same on main; against main, in a 123,274-scene screen (seeds 9301–9308, modes 1 and 2, V-bit corners forced,
    leads on and off) the kept tabs were never fewer in all but one scene, and in 3 scenes the branch
    keeps more. The exception is seed 9302 scene 2199 (Inside, D6, V120, allowance −1.276, leads off,
    mode 1): main keeps all 3 tabs, the branch's corner-move tab check leaves 1 out ("1 of 3 tab(s)
    left out … cut through … at a corner") although the exact oracle finds all 3 holding; no free piece
    results, so the error is conservative (repro
    /home/chris/harness/t0156v2/dump/m1_9302_2199.scn; carded as a T-0177 item).
    **Triage of the T-0158 verifier's repros (T-0167, on main after T-0156, whole-scene flood of
    the moves, leads off unless said).** Fixed: m2_1330000_9 (V-bit tabs judged at the top
    radius, Tab layer above; both starts). Not fixed, as on main: m0_271016_20 at VectorStart/Outside
    (the severed tab at the bottom corner of a notch, above; T-0177 (l)). Fixed too: m0_271000_0 at
    VectorStart/Outside (the V-bit tab that main left out is kept). Not tab defects: m0_291002_11
    (VectorStart/Outside 0.41 mm², and 4.76 mm² at the stored start), m0_271000_0 (1.19 mm²) and
    m0_271013_1 (box centre, 1.33 mm²) have an unreported free piece only with leads on, which is
    the known lead gap (no piece with leads off, moves otherwise identical). Kept and reported, not
    fixed: m1_1310133_29 (VectorStart/Outside) and a4_770001_217 (box centre) have a partial tab
    whose unjoined piece is a hairline sliver 0.05–0.11 mm thick (0.41 and 0.067 mm²) that the
    loose-piece search reports; the exact check resolves necks only from about 0.11 mm, and dropping
    the tab would free the part. mmrg_001_tabs_all_out (V-bit merged region): at the stored start
    and at VectorStart all three tabs are truly severed (kept, the flood finds them cut through),
    so the 1,373 mm² piece is reported; at the box centre three tabs hold. m1_272009_9
    (VectorStart/Outside, merged round region): 7 of 10 placed tabs have fewer than two pieces
    near them and hold nothing, none is severed, nothing is free. r101_1_0_42 (V-bit 120°, D1, On,
    616 tabs requested): with leads on 150 flood pieces ≥ 0.04 mm² against 98 reported at the box
    centre, with leads off 100 against 98 (2 unreported, the T-0177 (b) family; none at VectorStart). All
    carded in T-0177 (g)–(l). **Screen (T-0167, round 2):** the moves of main and of this change were hashed over 24,000
    V-bit-forced scenes (six configurations; harness t0167 wsf, seeds 9301–9308, 500 scenes each:
    mode 3 leads off, leads on, sharp corners forced, at VectorStart/Outside, at the box centre, and
    mode 0 with Inside/Outside and allowance ≥ 0 forced), 4,000 scenes of the natural mix and 8,000 end-mill
    scenes, and every scene whose moves differ was flooded on both sides (exact capsules at the
    layer below the tab top, per tab hold / partial / severed, free pieces against the reported
    ones). V-bit: 2,681 scenes change, 2,637 for the better (a tab kept that holds, a free piece
    held, a severed or hold-nothing tab gone), 28 with no net change and 16 flagged worse by the
    strict per-metric comparison (14 distinct scenes). End mills: moves and diagnostics are
    byte-identical in all 8,000 scenes and in the verifier's U-part family (4,000 parts, seeds
    9731–9738 of my generator). The flagged entries are benign by the flood: seed 9301 scene 153
    (V-bit 60, Outside, SHARP: main keeps no tab and two pieces of 15.5 and 2.9 mm² come free; the
    branch keeps three, two hold them, one is cut through by a closed pocket of waste that the
    quick answer keeps; no piece is free), 9308 scene 100 (eleven tabs kept, five hold nothing, none
    severed, none free), 9308 scene 412 (a partial tab whose unjoined piece is the 0.6 mm² piece that
    main also frees and reports), 9304 scene 275 (box centre: a 38 mm² piece held; a 0.075 mm²
    hairline left over is not reported, T-0177 (d)), 9307 scene 110 (two partial tabs, a 74 mm² piece
    held, nothing free), and in the Inside/Outside screen 9302/175, 9303/109, 9304/312, 9306/152,
    9308/234 and 9308/243 (main keeps no tab and frees 1 to 3 pieces; the branch keeps tabs, one of
    them cut through, and frees fewer), 9306/427 (two extra kept tabs cut through, nothing free),
    9305/161 (30.7 mm² held, one hairline of 0.045 mm² reported that the flood does not find) and
    9308/291 (a 1,622 mm² part held; six hairlines of 0.074 mm² left over, reported). **Kept tabs that hold nothing (accepted by the director).** Ten further scenes of the T-0167
    verifier (harness t0167v2, files res/v*_br.txt and _mn.txt; scene numbers 228, 314, 68, 473, 67
    (seed 5008), 193 (seed 5002, vt), 987, 444, 676 and 807) keep a tab that the flood finds
    severed, partial or holding nothing where main left it out; nothing is freed by it, and in
    several main frees pieces. It is the class above (a quick answer keeps a tab beside a closed
    pocket of waste, or a placement with more tabs wins). The "N tabs" count the user sees includes
    such tabs: a kept tab is not a promise that it holds anything there. **Cost
    (Release CPU, three interleaved reps with an A/A control, the T-0140 perf pages with a V-bit):**
    main → this change, seconds: rect 1,000 Outside V60 0.40 → 0.43 (A/A 1.07); rect 400 Outside V90
    TC6 0.217 → 0.214; rect 400 Inside V60 11.9 → 6.5 (output differs: tabs now kept); letters 200
    Outside V90 A0.2 2.47 → 1.42 (differs); gear 2,000 teeth Outside V60 3.20 → 1.07; Inside V60
    2.99 → 0.70; rect 1,000 Inside V90 TC6 18.8 → 11.7 (differs, A/A 1.28); arc 1,000 Outside V120
    4.99 → 2.19; bez 500 Outside V60 A0.2 13.6 → 14.1 (1.03); plate 500 Inside V90 A0.2 16.2 → 5.8
    (differs); end-mill rect 1,000 Outside 0.386 → 0.372. The block shortcut is `detail::settleBlock` (`block_settle.hpp`), pinned by unit tests
    against the exact distance of synthetic vectors (a block is settled only if every cell would get
    the same verdict: block radius, anchor cell and both comparisons). The first version of this change was
    1.4× to 2.7× slower on V-bit pages: it measured every sample against a second, longer set of
    sections (a guard), and its raster reached four full radii and asked the distance field for
    every cell, which dominates a long gear (96 % of the time in `obstacleDistance`). Now one set of
    sections serves, the raster reaches the tab-layer radius, and the distance is asked once per 4 × 4
    block of cells whose centre is further than the block's radius from the thresholds.
    **Band polygons.** Each stretch's band is one polygon whose winding number is positive exactly
    on the band: the sum of the boundaries of positively wound pieces covering it (a line's
    rectangle along its normals, an arc's annular sector between radii ρ − w and ρ + w or, with
    ρ ≤ w, its sector from the centre out to ρ + w, a wedge of radius w on the outer side of each
    corner, a half disc at each end), whose edges along the shared normals cancel; it runs along
    the outer offsets, back along the inner ones, and through the vertex at an inner corner, or
    through the centre of an arc tighter than w. Every point of the band lies on a normal of its
    nearest point of the path, or in a wedge or end disc, so the pieces cover the band exactly. At
    an inner corner between two lines long enough that both their pieces cover the triangle of the
    vertex and the two offset ends (each at least w·sin(turn) long, turn below 90°), the vertex is
    left out (that triangle keeps a winding of at least one), so a polygonised curve gets no spike
    per vertex. A whole closed cut (no kept tab) has no end discs: its last corner is one like the
    others and each side's chain is closed on itself, two rings whose winding numbers add up to the
    same (as one ring joined along the normal at the start it would leave a zero-width slit; since
    round 2 the material is each leaf less the bands, so the slit could no longer hide a piece, and
    the end discs it would add reach up to the polygons' 0.005 mm into the band's inside, ≤ 0.006 mm²
    over 4,800 comparisons). The polygons cover the true band and reach at most 0.005 mm (or the
    chordal tolerance, when smaller) beyond it (round 3): along its convex arcs (outer offsets,
    wedges, end discs) they run on tangents outside the arc, along its concave ones (inner offsets)
    on chords inside it. Round 2's chords inside every arc could leave a channel between two bands
    whose tools cut through (1510007_2: a thick 0.46 mm² piece missed). So a piece joined to the
    outside only through a neck narrower than 4 µm (2 µm per band), or up to about 14 µm between
    two convex band arcs, counts as loose, and a strip between two kerfs is reported from about
    0.055 mm wide (a free ring between two circular kerfs, 2 mm end mill, default tolerance:
    0.054 mm not, 0.055 mm reported; straight strips the same, the bands' straight edges being
    exact; the V-bit slivers 0.043–0.050 mm thick of 1510007_13 that round 2 reported are not).
    Clipper2's offset (still used for a stretch with a Bezier span) puts a spike back to the path
    at every vertex, which costs about the square of the vertices along each arc.
    **Nothing loose, proved first (round 3).** In most jobs nothing comes free (parts held by their
    tabs, away from other cuts), yet building and searching every band cost them 10–34 % (round 2
    verifier: 2,000 to 8,000 separated rectangles with tabs, one flower of 2,000–8,000 arcs with
    three 200 mm tabs). So before any band is built, `detail::pathsToSearch` proves for as many
    stretches as it can that their bands enclose nothing. Each band (to its reach + the polygons'
    0.005 mm) is covered by convex pieces, the convex hull of each line (long lines in pieces about
    four reaches long) and of each stretch of an arc whose sagitta is at most 0.012 mm (a quarter
    turn at most), grown by that reach, and the pieces are added in order along the path. By the
    Mayer–Vietoris sequence a union without holes stays without holes when the new (convex) piece
    meets it in one connected set, which holds when every earlier piece it meets either holds the
    path point a reach back from its start (every piece at most two reaches back along the path
    does) or meets it at a point that one of those also holds; where the path first comes back near
    an earlier part of itself no such point exists, so a loop is never closed unseen. Pieces of
    different stretches must not meet at all. Distances are bounded from the safe side: two pieces
    are apart only when their chords are farther apart than the reaches and sagittas, or the arcs'
    tangent triangles (outside them) are, refined to 8 and 64 sub-stretches near two reaches; a
    common point is taken on chords (inside the hulls). A stretch that fails, a whole closed cut
    and a stretch with a Bezier span are searched as below, and with them every stretch whose band
    reaches into the box of their cluster (a hole lies inside the box of the cluster round it). The
    pieces reach up to 0.017 mm beyond the bands, so a hole the proof could hide is at most
    0.034 mm across, where the thickness test needs more than 0.04 mm. Pairs are found on a hashed
    grid of cells two reaches wide (one cell for groups of up to 64 pieces), skipping the runs of
    pieces of one path within two reaches along it. Checked against searching every band:
    7,200 random path sets (crowded and spread, near-closing loops, gaps about two reaches, shifted
    to 1e9 mm; 20,314 thick pieces, 9,135 stretches cleared) find the same pieces, and
    144 random sweeps of whole Profile scenes (72 pairs of this branch and main, leads on and off, three scene modes, 50 scenes each) give identical moves (0 differences) and identical non-loose diagnostics (0 differences) (the loose-piece messages are the only addition).
    **Leaf by leaf (round 2).** One Clipper2 union of all the bands is a single sweep whose active
    edge list holds every band side by side, so where neighbouring kerfs overlap and the pieces are
    free it grew about 3.5× per doubling of the shapes (a grid of 4,000 shapes: 49 s in the union;
    a row of 4,000 squares: 2.9 s, 5.3× the whole run of main), and finding each hole's islands was
    cubic in nested free rings. Now (`toolpath/enclosed_pieces.hpp`) the plane is cut into leaves:
    a k-d subdivision at median vertex coordinates on the nanometre grid, at least two grid units
    off every vertex, until a leaf holds at most 512 vertices (or a cut would leave more than 4/5 of
    them on one side, e.g. long edges crossing the leaf, or a side narrower than 10 µm, a guard on
    depth and cost; then the other axis, or the leaf stays as it is), each ring reaching within
    1 µm of a cut clipped to both sides 1 µm beyond it (Sutherland–Hodgman, which keeps the winding
    number of every point on that side). A ring, or what is left of it, that surrounds a leaf
    without an edge in it adds a constant winding number there and is taken out on the way down
    (the two rings of a band round the leaf cancel; otherwise the leaf's rectangle, grown 1 µm,
    stands for them). Each leaf's material is its rectangle less the union of its rings (one small
    Clipper2 call that drops exact duplicate vertices only: round 2 dropped those within the
    distance tolerance, which shifted edges next to the cuts by up to 0.5 µm and areas by up to
    0.0025 mm²), its parts and their islands (the smallest part around each); parts that meet along
    a shared leaf side over at least 10 nm are one piece (union-find over the intervals on each cut
    line), and a piece on the outer frame (1 mm round everything) is the outside. Clipper2
    truncates (does not round) where a band crosses a leaf side, so a part's edge along the side
    could end a grid unit off it and the join was lost (round 2: one free 187.8 mm² slug reported
    as two of 93.9 mm², 1600019_46; held regions reported as pieces): output vertices within a
    unit of a side are now moved onto it, which never moves a ring's vertex (they lie at least two
    units off). A piece's area is the sum of its parts'; it is thick when one part alone is (a
    point of it, the centre of its box or its centroid, at least 0.025 mm from every edge, or else
    what is left of it eroded by 0.025 mm; its cut edges only make that smaller) or else all its
    parts together are, grown by twice the distance tolerance first and eroded that much more
    (converting them for Clipper2 drops vertices closer than that tolerance, which could open a
    crack along a leaf side: a 0.0553 mm² piece judged thin, verifier family 8). Against one leaf
    over everything (the verifier's ep.cpp, families 0–11, 4,680 scenes, 55,401 pieces, at 4 to
    512 vertices per leaf, also shifted to 1e5, 1e7 and 1e9 mm): the same pieces, areas within
    1e-4 mm², in every family but the three built to be degenerate — rectangles on whole
    millimetres touching exactly at corners and edges (2), edges nanometres apart about a cut (3)
    and necks from 1 nm (5) — where a piece joined to another only through a neck under 10 nm
    at a leaf side is split, by design, and a zero-width touch may join or not.
    **Cost (round 3, Release CPU, interleaved with main, alternating order, the T-0140 verifier's
    pages):** Normal jobs (no loose pieces, 3 to 10 interleaved pairs per page) stay within 10 % of main on every page except 'rect GAP=3 A=0.3 SEP=1', which costs 1.10 to 1.15 (about +0.3 s at 2.6 s for 2,000 shapes and +0.8 s at 5.6 s for 4,000); the director's decision is that the performance exemption covers that page too, as bounded cost for a safety check. Where every shape is loose the check does show: rect +58 to +83 % (1,000 rect: +83 %, +0.035 s), arc +32 to +48 %, row +9 to +15 %. Moves are unchanged (the check only adds messages). Sided tabs used to keep the
    region search (T-0156 replaced it by this one). Cost of the replacement (T-0156 verifier, Release, against the region search): sided pages with many close shapes got faster (2,000 rectangles Outside, allowance −0.5, GAP 3: 3.10 s to 0.60 s; 1,000 Bezier shapes On: 0.18 ×; 2,000 arcs On, GAP 3: 0.56 ×), 2,000 rectangles On 1.72 s to 1.56 s, and normal pages stay at 0.99 to 1.05 ×. Measured (T-0140 round 3, whole-scene flood of the moves as in
    T-0127, leads off, mode 4: 2,000 scenes, 304 free pieces, 303 reported, 0 false reports; mode 5: 2,000 scenes, 934 free pieces, 934 reported; the remaining mismatches were V-bit tools only; T-0156 added the corner moves, below). A V-bit's
    ramps and plunges are not in the band model (the cut paths stand for the moves): they cut less
    than its full-depth disc at the tab layer, so a reported area can be up to about 20 % smaller
    than the piece. With leads on, leads cut or split some pieces (the known
    gap below). Known gaps: leads are not part of this
    check (a lead crossing the part is a separate issue); the cut paths stand for the tool's
    moves, so links, plunges and ramps that cut more than the paths are not part of it, and a piece
    they cut away or split may be reported (m1_881008_35, 713003_20, also in T-0127); the bands are
    2 to 7 µm wider than the cut, so a piece within about that much of the 0.05 mm thickness
    threshold can be left out (713004_23: a 0.052 mm² piece); a tab whose two "pieces" in its window
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
    the height the cut ended at (the tab top when the overcut ends in a tab). A closed cut that
    collapsed to a point when its start was chosen (no spans left; it emits nothing) is dropped
    before ordering, tab keep-clear and lead planning (T-0163, T-0165); a vector left with no
    cut in a pass this way (or as a sliver no longer than the tolerance) gets one `ToolTooLarge`
    diagnostic ("its offset collapses in the roughing/finishing/cutting pass").
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
18. **Tolerance bounds (T-0141).** `ProfileParams::tolerance` must satisfy the same bounds as
    Pocket's (ADR-0017): 1e-4 mm <= chordal <= 1 mm and 0 < distance <= chordal, all finite.
    Otherwise nothing is cut and the result carries one `InvalidParameters` diagnostic, "invalid
    tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal" (since T-0142
    "... distance between 0.000001 mm and chordal", see below; the CLI
    prints it after "no profile toolpath was produced:"). Before T-0141 any tolerance was
    accepted, so a near-zero chordal tolerance multiplied the polygon vertex count without bound,
    and the offset dust filter's width budget (ADR-0006 §8) stays inside the offset slack
    only for chordal tolerances well above 1e-5 mm. Valid tolerances give byte-identical output.
    **T-0142:** distance must also be at least 1e-6 mm, one unit of the offset grid (ADR-0006 §2),
    in Profile and Pocket alike. Below it the exact offsets cannot close their junctions (a
    Bezier shape at distance 1e-8 mm fell back to a polygonal region with a trimmed cut, 8–12 s
    on the branch, minutes before, where Bezier input was polygonised at distance / 2); at 1e-6
    mm the same shape runs cleanly. The accepted range narrows from 0 < distance to 1e-6 <=
    distance; output for accepted tolerances is unchanged. Engrave (Outline and Fill) and V-Carve,
    which run Pocket for Fill and flat areas, refuse a distance below 1e-6 mm too, each with its
    own message ("... distance between 0.000001 mm and chordal", ADR-0024, ADR-0025); Drill keeps
    its own bounds.
19. **Work budget and cancellation (T-0186).** The profile had no work limit and could not be
    interrupted: the cost of a vector grows with its length (one clearance sample per
    millimetre: a 1e6 mm edge beside a neighbour took 17 s in Release), is steep on thin
    self-intersecting loops (a 730 mm tall Bezier loop 3 s, one 7.3 m tall 25 s), and hostile parameters
    allocated gigabytes (0.0001 mm pass depth, a tab every nanometre) or ran for minutes. Three
    parts, none of which changes the output of a job that stays inside them (checked against main,
    below).

    *Up-front refusals* (`ProfileBuilder::withinLimits`, closed-form, before any allocation that
    grows with the parameters). Each is `WorkLimitExceeded`, one diagnostic naming the value, an
    empty toolpath ("nothing is cut"); non-finite values are left to `validate()`'s own checks:
    - depth passes <= `kProfileMaxLevels` = 10000 (as Pocket and Drill; `PassPlan::levels()` is
      only called when the closed form is near the limit, so a 1e9 mm cut in 0.0001 mm steps
      allocates nothing);
    - |allowance|, and with a separate last pass the last-pass allowance, lead length (straight),
      lead radius (circular), overcut (any lead), ramp distance (any ramp) and tab length (tabs
      on) <= `kProfileMaxOffsetMm` = 1000 mm (an option that is off is not judged);
    - the end points of every span (a Bezier's control points) within +-`kProfileMaxCoordinateMm`
      = 1e5 mm, every arc radius <= `kProfileMaxRadiusMm` = 1e12 mm (the importers' cap; a
      nearly straight arc has a huge radius and a far centre, so the end points decide, not the
      centre: the T-0158 star tests have such arcs) and every arc angle below 1e6 rad; total vector length (lines and arcs exact, a Bezier by its control polygon, an upper bound: the exact length of a Bezier costs milliseconds and this check must stay cheap) <= `kProfileMaxLengthMm` = 1e7 mm; spans <=
      `kProfileMaxSpans` = 2,000,000;
    - tabs asked for over all vectors <= `kProfileMaxTabs` = 5e6 (constant number: count per
      vector; constant distance: length / distance, at least one per vector);
    - estimated moves <= `kProfileMaxMoves` = 20e6. The estimate is deliberately close to the
      real count for ordinary jobs and low for curve-heavy ones: levels x (2 per span + 4 per tab
      on the levels the tab stands on + the legs of a ZigZag ramp with a descent angle). It stops
      what the closed form can see (a ramp whose legs are 1e-9 apart); the emitted moves are
      counted as they are made and the run stops past the same 20e6 (`TooManyMoves`), so a
      Bezier-heavy job that polygonises into more is refused after its cut is planned, before it
      is all in memory.

    *Work meter* (`work_meter.hpp`). A `WorkMeter` counts integer work units; hot paths call
    `detail::chargeWork(n)`, which adds to the meter installed on the calling thread (a
    `thread_local` pointer; null for every other operation, so Pocket, Drill, V-Carve and Engrave
    are unaffected). One unit is meant as about 30 ns of Release CPU (a line's closest-point
    test). The charges, each calibrated against measured time (round 2, T-0186 verifier):
    - ClearanceField: span-box scan 1/4 unit a span; exact closest-point test 1 (line), 4 (arc),
      40 (Bezier, 1.2 us) or 110 (a Bezier whose control polygon crosses itself, a loop or cusp: 2
      to 3 us); a query 2 + the boxes it meets; one unit per millimetre of path sampled by
      `findViolations` (charged before the sample grid is allocated); the constructor.
    - ArcLengthPath: 15 per flattened Bezier point (evaluation and chord length), 1 per span,
      2 per span of a projection, 1 per step of `extract`.
    - Exact offset of a vector (`offsetOpenContour`), `chargeOffset`: n x (350 + n/188) units for n
      spans (11 us a span at 1,000 spans, 23 us at 64,000, 53 us at 250,000: the geometry library is
      superlinear there; a Bezier counts as one span here) plus
      14 units per polygon piece of a Bezier; plus, per Bezier span, 200 + 5e5 x sqrt(x) units with
      x = (control polygon - chord) / control polygon, which models the exact length the offset
      computes (its adaptive recursion needs about x / tolerance pieces): measured on 40 random cubics
      10 us at x = 1e-6, 80 us at 1e-4, 0.8 ms at 1e-2 and 4 to 8 ms at x = 0.2 to 0.7 (quarter
      circles, loops), so a smooth traced outline costs microseconds a span and a curvy one
      milliseconds (a flat constant, round 2, refused 4,000-span smooth outlines that main cuts in
      0.4 s); plus 8 units for each pair of span boxes that overlap once grown by the offset distance
      (a polygon of 100,000 chords of 1.3 micrometres offset by 1.5 mm took 27 s against 2 s for 63 mm
      chords; more than 7.5e7 pairs refuses the vector); plus (5 inwards, 1.5 outwards) x S^1.7 for
      S junctions turning by more than 60 degrees (a comb of thin teeth offset inwards took 6.5 s at
      20,000 teeth, 41 s at 40,000, 122 s at 60,000 on the verifier's machine: n^2.7).
    - Region union and offset of merged vectors (`chargeRegion`): 333 units per polygon vertex plus
      0.7 x P^1.35 + 50 x P for P overlapping pairs of polygon-segment boxes (found by a sweep that
      stops at 4e7 pairs or 4e8 steps, which count as over any limit). Concentric star rings
      (500 spikes x 3 to 16) took 0.06 to 5 s for P = 6e4 to 1.6e6, circles in a grid and nested
      circles 0.05 to 0.9 s; the model is within a factor of 3 of all of them.
    - Preparing the inputs: 4 units per polygon vertex (20 for a vector with Beziers); band union of
      the loose-piece check 20 + 60 per vertex; the tab-check raster one unit per eight cells; one
      unit per move emitted.
    The count depends only on the input and the parameters (not on time or load): the same job
    refuses at the same point everywhere. `ProfileResult::workUnits` reports it. When the count
    passes the limit (`ToolpathRunControl::workLimit`, default `kProfileDefaultWorkUnits` = 9e8)
    `charge` throws `WorkAborted`; `profileToolpath` catches it (and `TooManyMoves`,
    `std::bad_alloc`, `std::length_error`) and returns a fresh result: the toolpath name, no
    moves, and the single diagnostic `WorkLimitExceeded` ("... needs more work than the limit
    allows (N work units ...); nothing is cut"). Diagnostics collected so far (merges, trims) are
    dropped with the toolpath: nothing partial. `ToolpathRunControl::moveLimit` (default 20e6)
    lowers the move limit; it exists so the emit-time stop can be tested without 1.4 GB of moves.
    *One step at most 6e8 units.* The exact offset of a vector, and the union and offset of a merged
    group, are single geometry calls the meter cannot interrupt, and they are charged before they
    run. A vector or group charged more than `kMaxSingleCallUnits` = 6e8 (10 s only at about 16 ns
    per unit; at 30 to 60 ns, 18 to 36 s) is refused up front, naming it ("vector #N ... is too complex to offset in one step").
    This bounds the wait of a cancel (below) and spares the wasted work of a doomed job. The
    number is a trade-off: a lower one (1.5e8, 3 s) bounds the wait more tightly but refuses single
    contours main finishes in 5 to 10 s (2,000 curvy Beziers in one outline: 2.4e8 units, 5.3 s).
    Non-finite Bezier control points are refused up front as "a non-finite coordinate" (the Bezier
    polygonisation and the Clipper2 conversion behind it are undefined for them).

    *Calibration and the default.* Measured ns per unit, Release CPU on the shared machine at load
    average 20 to 30 (so each figure is inflated by up to about 2x against an idle machine; the
    ratios between families are what matter). Round 1 (verifier) -> round 2:

    | family | round 1 | round 2 |
    |---|---|---|
    | 500,000-span polygon circle R1000 (29.5 s on main) | 187 | refused up front (0.3 s, 1.5e9 units) |
    | 100,000-span polygon circle | 28 | 12.7 |
    | 20,000 thin spikes (0.2 mm pitch) | 197 | 52 |
    | 5,000 thin spikes (0.02 mm pitch) | 169 | 8.6 |
    | starburst 500 spikes x 50 shapes | 91 | refused by the region estimate (0.7 s) |
    | starburst 2000 x 5 / 40 x 500 | 75 / 22 | 20 / 24 |
    | 200 circles of 4 Beziers | 53 | 19.6 |
    | thin self-intersecting Bezier loop 730 mm | 52 | 31 |
    | 50 thin Bezier bow-ties | 20 | 30 |
    | 3,000 concentric circles / 500 | 31 / 49 | 29 / 40 |
    | grid of merging circles 4900 / inside 2500 / rings 1600 | 36 / 30 / 30 | 23 / 27 / 26 |
    | 160,000 tabs by distance | 51 | 49 |
    | near-straight huge arc, 99,000 mm | 29 | 29 |

    The default `kProfileDefaultWorkUnits` = 9e8 is a count, not a time, and the time it stands for
    varies 4 to 8x with the shape of the job (ns per unit: 4 to 36 on most of the families above, 57
    to 67 on a V-bit starburst and on nested circles offset inwards, whose cost is Clipper2's
    intersection search, which the charges only partly model). **Plainly: the real CPU at the
    default limit is not "12 to 27 s". The verifier measured, on loaded or idle-equivalent
    machines, runs that reach 9e8 units after 35 to 75 s: a comb of Bezier teeth offset inwards
    ("bcomb 10000 0.2 I") refused after 64 to 73 s, a 1,000-Bezier blob 63 to 68 s, 1,000 nested
    circles offset inwards accepted after 52 s (8.4e8 units), a V-bit starburst of 200 x 100 spans 34
    s; the thin Bezier loop and bow-tie families reach the limit after 20 to 28 s.** Do not read
    the default as a time bound. A bound in time needs the geometry offset to poll (T-0228). What the
    default does guarantee is that a hostile job stops and that nothing main finishes in under
    2 s is refused: over the 1200-job corpus the 1161 jobs main runs in under 2 s count at most
    1.1e8 (8x below the limit); the verifier's 13 overlap-heavy Bezier and polygon-blob jobs
    (seeds 101, 332, 333, 368, 582, 603, 705, 893, 1334, 1641, 2096, 2166, 2221; main 4 to 9 s idle)
    count 4.3e8 to 7.7e8 and all run. A 5,000-circle profile with 20 passes counts 5e6; a 3 m x
    1.5 m sheet of 10,000 rectangles in 0.1 mm passes to 2 mm counts 3e7 (0.8 s). A Bezier-built
    sign is expensive on main already (12 ms of exact length per Bezier span in the offset): 5,000
    glyphs of 30 Beziers would take main about 25 minutes and are refused here (4e9 units).

    *Tabs.* Tab planning was uncharged and quadratic in the tab count (a square of 24,000 mm with a
    tab every 3 mm: 2.2 / 8.7 / 33 / 128 s at 3,000 / 6,000 / 12,000 / 24,000 mm, at under 5e5
    units). Round 4 made the two quadratic parts linear with unchanged output: the stretches that
    cover a sample are found by position (`dropSweptTabs`), and `tabCeiling` appends a stretch that
    lies wholly after the ceiling so far instead of rebuilding every knot (checked against the
    repeated `ZProfile::max` on random sorted, overlapping and ramped stretches); the same job takes
    0.07 / 0.13 / 0.27 / 0.53 s. The stage is charged 500 units per tab (16 us) and 5 per sample of a
    cut, after the tab-count limit is checked.

    *Known over-refusals.* The charges over-count a few exotic shapes, so jobs main finishes in under
    10 s are refused: a thin sawtooth ribbon offset inwards (comb of 20,000 teeth "combzz 20000 0.2
    I": main 0.82 s, empty result "no region"; 100,000 teeth of height 5: main 5.2 s), and an open
    zigzag of 100,000 spans with V-bit leads (main about 5.6 s). The sharp-junction term and the box-pair term are
    charged even where the offset turns out empty or the contour is open. They are accepted until the
    geometry offset can report its own work (T-0228).

    *Cancellation.* `ToolpathRunControl::cancel` (an optional `const std::atomic<bool>*` in the
    new four-argument `profileToolpath`) is read with relaxed ordering once per 1024 units charged
    and between the stages (after the inputs are prepared, after the starts, after the lead
    plans, after the tabs, after each cut's moves). A set flag returns the empty toolpath and one
    `Cancelled` diagnostic, with the same nothing-partial guarantee; a flag that is already set
    returns before any work. Without a flag the polls cost one branch. Latency is the work
    between two polls (1024 units, about 30 us) plus the rest of the uncancellable geometry call in
    progress: one exact offset of one vector, one union and offset of one merged group, or the exact
    length of one Bezier span, or the tab stage of one cut (a cancel waited up to about 2.5 s at 1.3 million
    tabs). Those are charged up front and refused above 6e8 units, so the wait
    is up to about 11 s for a single Bezier outline and 26 to 35 s for a merged V-bit
    starburst group ("starburst 2000 5 V", 26.6 s and 35.0 s in two measurements), because the real cost per unit varies as above. Measured with the verifier's probes
    (Release, CPU seconds from the flag being set to the call returning, flag set at 50 ms and
    400 ms): 200 four-Bezier circles 0.02; 10,000-part sheets 0.001 to 0.08; thin Bezier loop
    and bow-ties under 0.04; 1,000 concentric squares 0.007; 60,000-span polygon 0.3 to 0.6 and
    100,000-span polygon 1.1 (one offset call); 20,000 thin teeth outside 1.1 to 1.6; 5,000 Bezier
    spikes 0.35 to 0.7; one outline of 2,000 curvy Beziers 4.9 to 5.3; 1,000 curvy Beziers (blob)
    8.2 to 8.4; one outline of 2,200 mixed Beziers 11.1. A job of many small vectors or cuts
    cancels in milliseconds. A cancel that arrives while the only vector is being offset waits for
    it. Interrupting those calls needs the geometry library (offset, Bezier length) to poll, which
    this card does not do: it is T-0228 (interruptible geometry offset), and until then the latency
    of a cancel during one huge offset is the figures above, not a bound.

    *Shared types.* `ToolpathRunControl` (cancel flag, work limit) and `ToolpathIssue::Cancelled`
    live in `operation.hpp` so Pocket, V-Carve and the rest can adopt the same hook; the meter
    (`work_meter.hpp`, `detail::chargeWork`) is installed per call by the entry point, and
    `ClearanceField`, `ArcLengthPath` and the other shared helpers charge it when one is installed.

    *Other operations.* Quick Engrave Outline does not share this code; it already refuses in core
    (`engrave.cpp`) when passes x (spans + 1) would pass the same 20e6 moves, and Pocket, Drill
    and V-Carve keep their own limits (ADR-0017, ADR-0022, ADR-0025). **For the T-0221 author:**
    the viewmodel's `profileWorkProblem` (levels, moves, parameter ranges), coordinate, length and
    span limits are now enforced by the core with the same values (its move estimate there was
    stricter: it counted every span as a polygonised arc); the session may keep its own as defence in depth, but should not report
    its messages as the core's, and should pass a cancel flag (`ToolpathRunControl`) instead of
    abandoning a running calculation. The viewmodel still needs its own checks for
    Engrave Outline's coordinates, length and span count (the core refuses only the move count
    there) and for anything it does before calling the core.

    *Limits of the budget.* Runs inside the limit are unchanged byte for byte (the meter only
    adds); a job just under it can still take half a minute, and the geometry library's
    superlinear cases (ADR-0005 amendments, T-0180) are charged by estimate, not measured: a
    vector whose offset is much slower than 300 units per span passes the limit later than the
    count says. The costs the count tracks best are clearance sampling and exact closest-point
    tests, which are what grows with edge length and with thin self-intersecting loops.

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
