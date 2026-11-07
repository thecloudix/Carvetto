# ADR-0029: Transforms, interactive handles and alignment

Date: 2026-11-07 · Status: proposed (T-0205)

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
2. **Parametric rule lives in the core command.** `TransformObjectsCommand`/`Paste` now run
   `transformData`, which keeps `CreationParams` (moving the centre, swapping a rectangle's
   width/height and adding the angle of ellipses/polygons/stars) for translations and rotations by
   a multiple of 90 degrees, and sets `NoParams` for every other map (arbitrary rotation, any
   scale, mirror). The pre-transform snapshot includes the parameters, so undo is exact. T-0209's
   array copies apply the same rule independently on their own copies.
3. **Operations are functions over a `Document`** (`transform_ops.hpp`): each validates everything
   first (finite, range 1e9 mm, scale factors in [1e-6, 1e6]) and then pushes one macro, so a refusal
   changes nothing, not even the undo history; a no-op (already aligned) pushes nothing. A mirrored
   copy is a `PasteObjectsCommand` plus a transform of the pasted roots in one macro, and selects
   the copies.
4. **Interactive gestures** (`TransformGesture`, owned by `ToolController`, Select tool): the
   gesture keeps one absolute transform from the press and pushes only the delta since the last
   step; the pushes coalesce on the undo stack (`breakCoalescing()` before the first and after the
   last), so any number of pointer steps is one undo step. Pointer positions are snapped (the dragged
   objects are excluded from the snap query; the generation-keyed index of ADR-0027 is rebuilt per
   step because every step changes the generation). Escape reverts the gesture's step; an unrelated
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
price of an index rebuild per pointer step (acceptable for ordinary drawings; dense drawings pay
O(objects) per step).
