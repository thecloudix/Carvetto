# ADR-0029: Transforms, interactive handles and alignment

Date: 2026-11-07 · Status: accepted (director 2026-11-07)

## Context
T-0205 adds move, scale, rotate, mirror, align and space on the selection, both interactively (the
T-0201 selection box handles) and through numeric forms. `TransformObjectsCommand` already exists
and coalesces, but it left creation parameters stale (a moved rectangle kept its old centre), and
nothing decided which objects a transform may touch.

## Decision
1. **Targets** (`transformTargets`): vector and bitmap leaves of the selection on the active sheet
   whose layer and every ancestor group's layer is visible and unlocked. A selected group is one
   operand made of all its leaves and transforms whole; a group with any unusable leaf (locked,
   hidden, other sheet) is skipped entirely rather than torn apart. Bounds are the exact bounds of
   the contours.
2. **Parametric rule lives in the core command, and is verified.** `TransformObjectsCommand`/`Paste`
   run `transformData`. Creation parameters are kept only when (a) the map is a translation or a
   rotation by a multiple of 90 degrees, to 1e-12 on every entry of the linear part (so small
   rotations, near-multiples and near-unit scales such as scale(1, 1+5e-10) are NOT quarter turns),
   AND (b) rebuilding the shape from the candidate parameters (`buildContours`) reproduces the
   transformed contours span by span (same kinds, start, end and mid points within 1e-9 mm + 1e-14 of
   the coordinate size), start point and direction included. Otherwise the object becomes a plain
   vector (`NoParams`). In practice translations keep the parameters; quarter turns keep them only
   where the rebuilt start is the turned start (polygons, stars, rotated ellipses); a turned rectangle
   or circle starts at another corner/point and is dropped. Scaling out and back also ends as a
   plain vector. The pre-transform snapshot includes the parameters, so undo is exact. T-0209's array
   copies apply the same rule independently on their own copies.
3. **Operations are functions over a `Document`** (`transform_ops.hpp`): each validates everything
   first (finite, range 1e9 mm, scale factors in [1e-6, 1e6]) and then pushes one macro, so a refusal
   changes nothing, not even the undo history; a no-op (already aligned) pushes nothing. A mirrored
   copy is a `PasteObjectsCommand` plus a transform of the pasted roots in one macro, and selects
   the copies.
4. **Interactive gestures** (`TransformGesture`, owned by `ToolController`, Select tool): the
   gesture keeps one absolute transform from the press and pushes only the delta since the last
   step; the pushes coalesce on the undo stack (`breakCoalescing()` before the first and after the
   last), so any number of pointer steps is one undo step. Pointer positions are snapped with an index built once at the first drag step with the dragged
   objects left out of it (`SnapIndex` `skip`), valid while the undo generation is the one the
   gesture itself produced; the generation-keyed index of ADR-0027 serves everything else. Escape reverts the gesture's step and drops it from the stack (`UndoStack::undoAndDiscard`, nothing becomes redoable); an unrelated
   edit during the drag ends it. Handles: Scale mode (corner proportional, Alt free, Shift about
   the centre; edge handles one axis, Shift proportional), Rotate mode (corner handles rotate, Alt
   15 degree steps, the middle handle drags the pivot), body drag and the middle handle in Scale
   mode move (Alt one axis). A click without drag on a selected object or the middle handle
   toggles Scale and Rotate (the inventory's "click the selection again"). Shift/Ctrl on a handle
   are transform modifiers; on the object body they still extend the selection.
5. **Numeric forms** (`TransformForm`) follow the T-0204 field contract (`fields`, `setField`,
   display units, reject without side effect); the Qt dialogs only host line edits.

## Interpretations (inventory is terse; director may overrule)
- Mirror "Horizontal" flips left-right (X reverses, vertical axis); "Vertical" flips top-bottom.
  Use Rotated Bounds mirrors about the centre of the minimum-area rotated box along its sides.
  Flip About Line uses the last selected object when it is a single straight line.
- Align: Centre (both axes), Vertical Centre (centres Y), Horizontal Centre (centres X), Left,
  Right, Outside Top (placed above), Outside Bottom (placed below) of the last selected object.
- Space Selection: equal gaps between bounding boxes; ends stay put; ordered by position, or by
  selection order (Ctrl); "inside last vector" spreads the others in its bounds with equal edge gaps.
- Absolute rotation: the current orientation is the angle parameter of a single selected ellipse,
  polygon or star, otherwise 0.
- Shortcuts per the inventory: M, T, R, H, V, Ctrl+H/Ctrl+V (copy), Shift+H/Shift+V (about the
  material centre), F9, F10. Ctrl+V may later need to share with Paste.

## Consequences
Moves keep shapes re-editable. Transforming an object with a rotated rectangle is an ordinary
`NoParams` vector. Dragging the dragged objects out of the snap set keeps snapping stable, at the


## Deferred (director decision 2026-11-07)
Ctrl+drag paste-copy and Ctrl+Alt+drag axis-aligned copy (inventory, common handles) are outside
this card.

## Known limits (verifier notes, round 2)
- Ctrl+H and F9 also fire while a numeric field has focus (they are not text keys); the plain letters
  are typed as text there.
- Window-driven undo/redo refreshes the action enabling; a direct `doc.undo()` that bypasses the
  window leaves it stale until the next change notification.
- The node-editing keys H and V (T-0206) collide with Mirror H/V. T-0206 handles this with a
  ShortcutOverride / enable switch while node editing is active; confirm once both are merged.
