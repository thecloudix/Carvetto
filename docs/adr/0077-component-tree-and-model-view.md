# ADR-0077: Component Tree panel, Component Properties form and the Model 3D view

Status: accepted (T-0299)

## Decisions
- **Viewmodel.** `viewmodel/relief_tree.hpp` holds all panel logic Qt-free: `buildReliefTree` (levels and
  components of the active sheet, top of the stack first, with the storage index the reorder commands
  take) and one function per user action. Each action is exactly one undo step through the commands of
  ADR-0061 (multi-command actions such as "Show only this", "Hide all" and the properties form run in an
  undo macro), validates first and returns an `EditResult`, so a refusal never touches the history.
  Numbers are entered in millimetres and degrees; shape height is limited to 0..1000 %, tilt to below 89
  degrees, fade/tilt need two different points.
- **Appearance is a view setting.** The document format (ADR-0061) stores no per-component colour or
  material, and this card does not change the format. The Model view therefore has one material choice
  (Solid Color, Wood, Metal, Stone, Plastic) for the whole picture. Per-component and per-level
  "Same as Parent / Color from Children" appearance needs a format change and is left to a later card.
- **Model 3D view.** `viewmodel/model_view.hpp` is a software renderer: the visible model of the active
  sheet is composed onto a grid of at most 512 cells along the longer side (`composeModelSurface`; cell
  size multiplied by an integer stride, coarse cells are exact composition samples), drawn as a
  Gouraud-shaded height surface with a z-buffer from an orthographic orbit camera. The cost is bounded
  by the surface cells plus the covered pixels, so a 2000 x 2000 field is no slower than a 512 x 512
  one. A software renderer keeps the picture deterministic and testable headlessly (pixel samples in
  the viewmodel tests and the self-test); no OpenGL dependency is added.
- **Controls** follow the inventory: right-drag orbits, wheel zooms, middle-drag or Ctrl+right-drag
  pans, standard Top/Front/Right/Fit buttons. Perspective, orientation cube, zoom to box and rotary
  wrapped view are not part of this card.
- **App.** A "Model" tab next to the simulation "3D View" tab (which keeps showing the cut stock). The
  Components dock holds the tree and the properties form; the main window refreshes both and the view
  after every document change. Dragging a component onto a component or level calls the same drop action
  the self-test uses; the tree widget never lets Qt move items itself, it is rebuilt from the document.
