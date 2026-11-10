# ADR-0036: Keep-out zones (document, rapid routing, post blocks, .carv 1.4)

Date: 2026-11-09 · Status: proposed (T-0241)

## Context
Keep-out zones (feature inventory, "Keep-out Zones") are closed outlines, copied from closed vectors
and not linked to them, that the tool's rapid moves must avoid, with a clearance value and a
show/hide toggle. The inventory lists the post blocks `INITIAL/FIRST/STRAIGHT_RAPID_MOVE` for the
rapids that go round a zone but does not say how a post picks between them.

## Decision
1. **Document.** `Document` holds `KeepOutZone{id, Contour}` in creation order, one clearance (mm,
   0..1000) and a visibility flag. Zone ids come from the shared id generator (ADR-0007). User edits
   are commands (`CreateKeepOutZonesCommand` = Create from Selection, `RemoveKeepOutZonesCommand` =
   delete / Clear Zones, `SetKeepOutClearanceCommand`); visibility is a view toggle, saved but not
   undoable. Zones are global to the job (not per sheet). Limits: 10,000 zones, 2,000,000 spans.
2. **Region.** The union (non-zero) of all zone outlines grown by the clearance with round joins
   (`offsetContours`), polygonised at the default chordal tolerance. A rapid is blocked when it passes
   through the region's interior; running along the boundary or touching it is allowed.
3. **Routing is applied when posting, not stored.** `toolpath::routeRapids` replaces a blocked rapid
   by the shortest XY detour (visibility graph over the region's vertices, A*; intermediate
   legs at the higher of the two end heights). Toolpath records and their move caches are unchanged,
   so editing zones never marks a toolpath stale. The Save Toolpaths job copies the zones and routes
   each toolpath, chaining the start position from home and the previous toolpath's end.
   A rapid whose end point lies inside the region, or that is enclosed, cannot be routed: it is left
   straight and reported as a warning (nothing is silently moved).
   The search is work-bounded (round 1 of the review: it was O(V^3)): each route counts the polygon
   edges scanned by its visibility tests, gives up past 10 million (`TooComplex`) or on a geometry
   cancel, and the rapid is then left straight with a "zones too complex" warning. A few hundred
   polygon vertices route inside the budget; a thousands-of-vertices outline does not, by design.
4. **Post blocks.** `Move::route` (`RapidRoute`, in memory only) marks detour legs. The first rapid of
   a toolpath uses `INITIAL_STRAIGHT_RAPID_MOVE`, the first leg of a detour `FIRST_STRAIGHT_RAPID_MOVE`,
   later legs `STRAIGHT_RAPID_MOVE`. A missing block falls back to `STRAIGHT_RAPID_MOVE`, then to the
   ordinary `INITIAL_RAPID_MOVE`/`RAPID_MOVE` chain, so every existing post works unchanged. (The
   inventory names the three blocks only; this mapping is our reading of it.)
5. **.carv 1.4.** Optional root record `keepout` in `job.json`: `{clearance_mm, visible, zones:[{id,
   outline}]}`. Written, and `format_version.minor` set to 4, only when a document has zones or a
   non-default clearance/visibility; documents without them keep their previous minor (1.1, 1.2 with text, 1.3 with dimension
   objects, T-0246). The written minor is the highest any used feature requires, so keep-out (4)
   and dimensions (3) are independent; a document with both is 1.4. Text-only
   documents are written as 1.2 (`kCarvFormatMinorText`). Unknown keys inside `keepout` are dropped.

## Consequences
The UI (menu entries, canvas drawing, clearance field) is a thin layer over the commands above; it
is not part of this card's headless deliverable.
