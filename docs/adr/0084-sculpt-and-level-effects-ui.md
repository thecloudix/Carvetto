# ADR-0084: Sculpt mode and level effect actions (app)

Status: accepted (T-0302, first subset)

## Decisions
- Sculpt logic is Qt-free in `viewmodel/relief_sculpt.hpp`: `BrushSettings` (palette kind,
  diameter 0.5-200 mm, strength, smoothness, Normal/Raise/Lower, preserve transparency),
  `BrushPresets` (built-in presets that cannot be overwritten, user presets kept for the session)
  and `SculptController`, which wraps one `relief::SculptSession` on one component. The widget is
  `app/src/sculpt_panel.*`; the level effect section is `LevelEffectsPanel` in the same files and
  `viewmodel/relief_effects.hpp`.
- Sculpting never touches the document until Keep. Keep is ONE `ReplaceReliefComponentCommand`;
  painting continues from the kept state (the session baseline moves). Discard returns to the last
  kept state, Undo Stroke reverts one stroke, and the palette's Undo Brush paints back to the
  last kept state. Stopping the mode drops strokes not kept. When the component is deleted the
  session ends; when an outside edit (Undo of a Keep) changes it and no strokes are pending the
  session restarts from the document.
- The Model view shows the working copy of the sculpted component ALONE (the same mechanism as the
  modelling previews), not merged with the other components. Painting is left-drag on the Model
  view; `pickModelPoint` (viewmodel/model_view) casts the pixel ray against the displayed surface,
  so it works from any camera angle, and the picked design-space point is the brush position.
- A stroke is re-applied from its whole path on every mouse move (undo the open stroke, apply
  `dabsAlong` of the path), so the result does not depend on how often the mouse reported. Cost grows
  with stroke length (a path is capped at 50000 points).
- Clipping uses the selected closed vectors flattened at the default chordal tolerance into polygons
  (design space); Apply and Update are the same action; refused with a message when nothing closed
  is selected or the caps (256 outlines, 65536 points) would be broken. Mirror Mode stores the
  centre of the sheet's model (union grid of the visible components) at the time it is set. The
  level acted on is the one picked in the Component Tree, else the level of the selected
  component, else the top level. Bake Effects is offered since the command exists.

## Not in this subset (T-0302 follow-up)
Overlay (reference) image over the model, custom brush stamps from an image, a brush ring cursor
in the Model view, preset persistence across sessions, sculpting merged with the rest of the model.
