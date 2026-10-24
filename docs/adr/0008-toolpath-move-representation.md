# ADR-0008: Toolpath move representation

Date: 2026-10-24 · Status: proposed (T-0040; consumed by T-0030 post-processor)

## Decision
1. A `toolpath::Move` is a plain value: `MoveKind kind` (Rapid, Feed, Plunge, Retract, ArcCW, ArcCCW, Dwell), `Vec3 to` (absolute machine coordinates, mm, Z up, tool tip), `Vec2 centre` (XY arc centre, arcs only), `std::optional<double> feedOverride` (mm/min; ignored for Rapid and Dwell) and `double dwellSeconds` (Dwell only).
2. **The start point of a move is implicit:** it is the end point of the previous move in the list. A `Toolpath` does not store the position before its first move; queries that depend on it (`lengthByKind`, `bounds`, `estimateTime`) accept an optional `from` and otherwise treat the first move as zero length. By convention the first move of a toolpath is a `Rapid` at safe Z.
3. Arcs are stored as end point + centre, direction in the kind. The radius is taken from the start point. Coincident start and end (in XY) denote a **full circle**, the same convention as `geometry::ArcSpan` (ADR-0005). A Z change across an arc is a helix with linear Z interpolation.
4. `Vec3` lives in a new header `geometry/vec3.hpp` (`carvetto::geometry::Vec3`), header-only and minimal (arithmetic, dot/cross, length, `xy()`, `distance`, `lerp`, `approxEqual`). No T-0001 header was modified.
5. `Toolpath` carries a `ToolSnapshot` (name, tool number, diameter, feed rate, plunge rate, spindle speed) copied at calculation time rather than a reference into the tool database, so later database edits do not silently change computed output or the post's tool-change block.
6. Time estimation (`toolpath/time.hpp`) models each move independently with a symmetric trapezoidal velocity profile from rest to rest: `t = d/v + v/a` when `d ≥ v²/a`, otherwise `t = 2·sqrt(d/a)`; infinite acceleration gives `d/v`. Rates are mm/min as users enter them, acceleration mm/s², output seconds split by kind. Retract moves run at the plunge rate.

## Rationale
- Implicit start points halve the storage and, more importantly, make the list impossible to leave discontinuous: every post-processor block (`RAPID_MOVE`, `FEED_MOVE`, `CW_ARC_MOVE` with I/J relative to start, …) already assumes a current position, so the post interpreter walks the list carrying the previous end point exactly as the controller will.
- End point + centre (rather than radius or sweep) is the lossless form that both `.pp` arc blocks (I/J) and R-form output derive from, and avoids the two-solutions ambiguity of an R-form arc.
- Separate `Plunge`/`Retract` kinds exist because posts have dedicated blocks for them (`PLUNGE_MOVE`, `RETRACT_MOVE`) and the time estimate and simulation treat them differently from lateral feeds.
- Per-move `feedOverride` covers ramping, lead-in/out and finishing passes at a different rate without a second parallel array.
- A per-move rest-to-rest profile is deterministic, cheap, and conservative with respect to controllers that blend consecutive moves; it is the baseline the "Machine Acceleration" option in the feature inventory asks for and can be refined later without changing the data model.

## Consequences
- `post` (T-0030) consumes `toolpath::Move` unchanged and must carry the current position while interpreting; a post without arc blocks linearises arcs itself from end point + centre.
- `simulate` can replay the same list; rotary wrapping (`ROTARY_WRAP_*`) stays in `post` as ARCHITECTURE.md states.
- Serialisation of `toolpaths/*.bin` (ADR-0003) serialises this struct; adding a field is a format change.
- `PassPlan` equalised mode allows the step to grow by up to 15 % (`kStepSlack`) before adding a pass, which is one reading of "default spacing may vary the step by up to 15 %" in the feature inventory; the director should confirm against the product's observed behaviour.
