# ADR-0027: Drawing tool controller (one state machine, parameter-first creation)

Date: 2026-11-07 · Status: proposed (T-0204)

## Context
T-0204 adds interactive drawing (rectangle, circle, ellipse, polygon, star, arc, polyline) and moves
click-to-select behind the tool layer. ADR-0014 requires the logic to be Qt-free; the canvas must
stay a thin event forwarder. The shape builders (T-0200) and snapping (T-0203) already exist.

## Decision
1. **One `ToolController` class** (`carvetto/viewmodel/tools.hpp`) with a `ToolKind` switch, not a
   class per tool. The tools share almost all state (placed points, press/drag detection, typed
   overrides, snapping, commit); a tool is a few draft functions that map (placed points, pointer,
   modifiers, typed values) to creation parameters. Adding a tool means adding a draft function and
   a field list.
2. **Parameters first.** Parametric tools build a `CreationParams` and take the contours from
   `document::buildContours(params)`, so a created object is by construction identical to a rebuild
   from its stored parameters (and to a `.carv` round trip). Arc and polyline results are
   `NoParams` contours (the variant has no arc/polyline parameters yet).
3. **Events in screen pixels, results in millimetres.** The canvas forwards move/press/release/
   double-click with modifier flags and a few keys; the controller snaps every pointer position
   (`SnapIndex`, rebuilt when the undo stack, object count or active sheet changes) and exposes the
   snap, the effective point, and a `ToolPreview` for the overlay.
4. **Completion = exactly one `AddObjectCommand`.** Escape and tool switches only reset controller
   state, so cancelling never touches the document or the undo stack.
5. **Numeric entry** is a controller contract: `fields()` (id, label, kind, sign/zero rules, live
   value text), `setField()` (parse with the display unit, validate, store as an override, reject
   bad input without side effects), `commitNumeric()`.
6. **Modifiers** (the card named Shift/Ctrl; the feature inventory wins where it speaks):
   Shift disables snapping for the event (inventory, snapping section) and, in the polyline tool,
   additionally constrains to 45 degrees; Alt draws rectangles and ellipses from the centre and a
   circle by its diameter; Ctrl makes rectangles square and ellipses circles (inventory, shape
   tools). The grid snap kind starts off in the controller.
7. **Dense-drawing bound** (T-0203 follow-up): the pairwise intersection search takes only the
   `SnapIndex::kMaxPairSpans` (20) spans nearest to the cursor and skips pairs whose common box
   misses the cursor window; a genuine crossing between spans that are not among the 20 nearest
   is not reported until the user zooms in (documented in `snapping.hpp`).

8. **Director-accepted conservative decisions** (T-0204): Circle defaults to centre plus radius, Alt
   makes the drag the diameter; the marquee is Window when dragged left to right and Crossing right
   to left; Select acts on release so a click can be told from a marquee; the grid snap kind starts
   off; a created object is not auto-selected; Arc start/end/radius takes the side from the pointer
   and the radius from typing or the circle through the pointer, with a "larger arc" option;
   polyline Bezier click-drag, S/T quick keys and extending an existing vector are not implemented.

## Consequences
The Qt layer needs no tool logic: toolbar actions call `setTool`, the canvas forwards events and
paints `preview()`/`snap()`, a small panel renders `fields()`. Pointer-driven arc and polyline
results cannot be re-opened as parametric forms until the variant grows (out of scope).
