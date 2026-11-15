# ADR-0087: Surface-building forms (sweep, extrude, turn, textured area)

Status: accepted (T-0308)

## Decisions
- Two Rail Sweep, Extrude and Weave, Turn and Spin and Textured Area are pages of the Modelling
  panel, data-driven like the modify forms (ADR-0083): `viewmodel::buildFields(tool)` lists the
  fields, a `BuildForm` is a label -> text map, `previewBuild` builds the component without
  touching the document (`app/viewmodel/relief_build.hpp`).
- They are creating forms, so they use `ModellingSession` (ADR-0080): the first Apply adds a
  component through `AddReliefComponentCommand` (one undo step), Apply again replaces it in place,
  Start New Component adds another.
- Inputs come from the selection. Sweep: the first two selected vectors are rail 1 and rail 2.
  Extrude: every selected vector is a rail; the Component source copies the first selected
  component (baked). Turn/Spin: the Profile text, else the first selected open vector. Textured
  Area: the first selected component is the seed tile, the selected closed vectors the boundary.
  After Apply the previous component selection is restored for the build forms, so the seed does
  not silently become the component just made.
- Profiles are typed as "x,y; x,y; ..." (a graphical section editor is not done). Sweep places the
  same profile at t = 0 and t = 1 of both rails, or at every node ("Section at every node");
  Extrude uses one section centred on the rail.
- The grid of Sweep and Extrude is the rail bounding box (grown by the profile or component
  half-width plus one cell) at the typed cell size; Turn/Spin and Textured Area use their own
  footprint. Preview shows the result alone, as in ADR-0080.

## Not done
Per-section positions and profiles for sweeps (only start/end or every node), multiple different
sections for extrusions, a profile picked from a vector, both-sides mesh import and a draggable
zero plane (the optional parts of the card), preview merged with the rest of the model.
