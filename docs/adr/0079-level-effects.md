# ADR-0079: Level effects: clipping and mirror mode

Status: accepted (T-0295)

## Decisions
- **Where.** `relief::LevelEffects { Clip clip; MirrorMode mirror; Point mirrorCenter; }` is a member
  of `relief::Level` and of the document's `ReliefLevel`; `Document::reliefModel` copies it. The
  algorithms live in `relief/effects.hpp` (`applyClip`, `applyMirror`, `applyEffects`,
  `validateEffects`, `bakeLevel`). `compose` runs them per level after the components are combined
  and the level base height added, before the level combines into the model. Stored component
  fields are never written: the effects work on the composed per-level field only.
- **Order.** Mirror first, then clip, so the clip always bounds what the level contributes
  (a mirrored copy cannot escape the clipping vectors). Because a constant offset commutes with
  both a copy and a mask, the level base height is added before the effects and still never part
  of a bake.
- **Clipping.** The level keeps a copy of the selected closed vectors as flattened polygons in
  design space (not references to objects): Apply, Update and Remove are all one command
  (`SetReliefLevelClipCommand`, empty polygons removes), so the clip survives deleting or editing
  the vectors until the user updates it. Cells whose centre is inside stay; fill rule is even-odd
  (nested polygon = hole, winding irrelevant). Evaluated per row with a half-open crossing rule
  (`a.y <= y < b.y`, crossing `x <= cell x` counts), so a centre exactly on a left or bottom edge is
  inside, on a right or top edge outside, and vertices are never double counted. Cost is
  O(rows * edges + cells). Flattening curves into polygons is the caller's job (app, T-0302).
- **Mirror.** Axes pass through `mirrorCenter` (the caller picks it, normally the sheet or model
  centre; stored so the result does not depend on the compose grid). Directions are design space,
  Y up: "Left to Right" copies the left half onto the right, "Top to Bottom" copies the top
  (larger Y) onto the bottom. "Four quadrants" is read as four modes, one per source quadrant
  (top-right, top-left, bottom-left, bottom-right) copied onto the other three; the reference
  inventory only names "the four quadrant modes". Target cells take the bilinear sample of their
  mirror image in the pre-mirror level field (exact when the axes are on cell-centre symmetry);
  a transparent or out-of-field source makes the target transparent; cells exactly on an axis keep
  their value. Product reading to confirm with the owner: which quadrant/side semantic and
  default axis the UI offers (T-0302).
- **Bake.** `relief::bakeLevel` composes the level's visible components on their union grid with
  the effects (base height excluded) into one component named "<level> (baked)"; mirrored content
  beyond the union grid is cut off. `BakeReliefLevelEffectsCommand` replaces the level's components
  by it and clears the effects, keeping level mode/base/visibility; hidden components are
  discarded; undo restores them, the effects and the component selection; ids are kept for redo.
  Refused with no effects, nothing visible, or if the component/cell caps would be exceeded.
- **Caps.** At most 256 clip polygons of at least 3 finite points, 65536 vertices per level, finite
  mirror centre (`validateEffects`; commands, loader, saver and `checkIntegrity`
  (`BadReliefEffects`) share it).
- **Format 1.10.** A relief level record gets an optional `effects` object
  (`mirror`: `none|left_to_right|right_to_left|top_to_bottom|bottom_to_top|quad_top_right|quad_top_left|quad_bottom_left|quad_bottom_right`,
  `mirror_center` `[x,y]`, `clip` list of `[[x,y],...]`), written only when a level has an effect;
  the minor is 10 only then, so every other file keeps its bytes and 1.9 files load unchanged.
  Older readers refuse a 1.10 file by the version check rather than silently dropping the effects.
  The loader bounds polygons and vertices before storing them and charges the load budget.
  Mirror-centre edits coalesce in undo; clip and bake do not.

## Tests and mutants
Mutants run by hand against `[effects]`/`[relief]` (each killed): crossing `<=` -> `<`; half-open
test `<=` -> `<`; mirror source side inverted; mirror axis `>` -> `>=`; effects order swapped;
effects skipped in compose; polygon minimum 3 -> 2; vertex cap off by one; polygon cap off by one;
bake keeps level base height; bake leaves effects set; bake undo drops selection; minor not bumped;
mirror centre not written; clip polygons not read; quadrant source mapping.
