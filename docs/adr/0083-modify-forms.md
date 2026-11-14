# ADR-0083: Modify forms and the custom shape profile

Status: accepted (T-0306)

## Decisions
- The remaining modelling tools (Replace Below, Reset Inside/Outside Vectors, Offset Model, Smooth,
  Emboss, Slice Model, Add Draft, Split Component, Create Vector Boundary, Create Cross Section
  Vector, Distort Component) are pages of the existing Modelling panel. They are data-driven:
  `viewmodel::modifyFields(tool)` lists label, kind (number / check / choice), default and tip, and a
  `ModifyForm` is a label -> text map, so the panel builds and reads every page generically and the
  headless tests cover the same forms.
- Targets: the selected components, else every component of the active sheet. Per-component tools
  (replace below, reset, offset, smooth, emboss, split, distort) work on each target's own height
  field and keep its dynamic properties. Whole-model tools (slice, draft, cross section) use the
  selected components baked together, else the composed visible sheet model.
- Apply is ONE undo step (a macro): in-place replacements (`ReplaceReliefComponentCommand`), new
  components inserted directly above the first target in its level (slice parts, draft, split part
  B), and one vector object for produced vectors (boundary, slice footprints, cross section). A
  split replaces the target with part A and adds part B. Added components become the selection.
- Unlike the creating forms, modify forms have no re-apply-in-place session: each Apply works on the
  current data, so applying twice applies the tool twice (undo steps back one at a time). Start New
  Component is disabled for them.
- Preview shows the first changed or added height field alone, like T-0300; vector-only results
  report a count in the status line and show nothing.
- Cross Section is placed at its start point: vertex (distance, height) becomes (start.x + distance,
  start.y + height). Product reading, listed for owner review. Cross Section and Distort start from
  the bounds of the first target (cross section along its mid-line; distort with the unchanged
  corners), so editing a corner is the way to distort.
- Reset and Split take their vectors from the vector selection (closed only for Reset; one open or
  closed vector for Split; the first selected is used).
- Custom shape profile: a text field "depth,height; depth,height; ..." (depth 0..1 strictly rising,
  at least two points), parsed by `parseShapeProfile`. A graphical editor is not done.

## Not done
Preview merged with the rest of the model (as in ADR-0080); a graphical profile editor.
