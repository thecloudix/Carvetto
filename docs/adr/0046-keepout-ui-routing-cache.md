# ADR-0046: Keep-out zones UI: routed preview and summary (T-0256)

Date: 2026-11-10 · Status: proposed (T-0256, builds on ADR-0036)

## Context
ADR-0036 routes rapids when posting and leaves toolpath records unchanged. The UI must show the
routed rapids in the 2D preview and count them in the job summary without storing them.

## Decision
1. `viewmodel::KeepOutRouter` (owned by `ToolpathSession`, `keepOutRouting()`) routes the enabled,
   computed toolpaths with the same `routeRapids` as Save Toolpaths (home = (0, 0, safe Z of the
   first toolpath), each toolpath starting at the previous one's end) and caches per toolpath by
   moves identity, start position and zone set. The preview and `summary()` share it, so a zone edit
   routes once. Save Toolpaths keeps its own per-output-file chaining (it groups by tool).
2. The preview draws the routed moves, but chunks carry the index of the original move they stand
   for, so `firstMove`/`totalMoves`, the playback and the simulation still index the record's moves.
   Detour legs are drawn as rapids; a first-move detour starts at the routing start position.
3. `JobSummary` time and length count the detours; `moves` stays the record's count. It also reports
   routed / unroutable / too-complex rapid counts, which `makeTimeReadout` turns into warnings.
4. Zones are drawn from `CanvasScene` (`keepOutZones`, `keepOutHalo`); the halo (union grown by the
   clearance, round joins) is cached per widget. Show/hide is the document's visibility flag.
5. Routing runs on the GUI thread in `sync()`/`summary()`: cost is bounded by ADR-0036's per-rapid
   work budget; a worker thread is a possible later refinement.

## Consequences
Cheap when a document has no zones (no routing, no copies). Large zone sets with many blocked
rapids can make a preview refresh take a moment until routing moves off the GUI thread.
