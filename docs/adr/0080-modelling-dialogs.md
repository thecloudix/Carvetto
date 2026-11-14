# ADR-0080: Modelling forms (Create Shape, Bitmap, Zero Plane, Scale Z)

Status: accepted (T-0300, first subset)

## Decisions
- One docked "Modelling" panel with a tool selector and one form per tool, plus the common
  Preview / Apply / Reset / Start New Component buttons. Logic is Qt-free in
  `viewmodel/relief_modelling.hpp`; the widget (`app/src/modelling_panel.*`) only reads and writes
  form text. Numbers are typed text, parsed in the viewmodel (mm, degrees).
- Preview builds the component without touching the document and shows the field alone in the
  Model view (`ModelView3D::setPreview`); it is cleared by Apply, Reset, a tool change or an error.
  Showing the preview merged with the rest of the model is not done (it would need a composed copy
  of the document per keystroke).
- Apply is ONE undo step. The first Apply of a creating form adds the component to the top level of
  the active sheet (a "Level 1" is created in the same step when the sheet has none) and selects
  it. Apply again replaces that same component in place (new `ReplaceReliefComponentCommand`: whole
  component data, same id, level and position), so tuning a form does not pile up copies. Start New
  Component forgets the component, so the next Apply adds another. A component the user deleted
  meanwhile is not re-applied to.
- Scale Model Z applies to the selected components, or to every component of the active sheet when
  none is selected, as one macro step; a component that cannot be scaled (flat, empty) refuses the
  whole edit with its name in the message.
- Create Shape uses the selected closed vector contours (polygonised at the default chordal
  tolerance, 0.01 mm). The Custom kind has no profile editor yet and uses a straight ramp.
- Bitmap source: a file path, or the selected bitmap object (its width and corner pre-fill the form).

## Not in this subset
Replace below, reset inside/outside, offset, smooth, emboss, slice, draft, split, vector boundary,
cross section and distort forms; they can reuse `ModellingSession` and `ReplaceReliefComponentCommand`.
