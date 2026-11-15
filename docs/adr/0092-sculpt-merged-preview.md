# ADR-0092: Sculpt preview merged with the model (app)

Status: accepted (T-0316; resolves the "Not done" item of ADR-0090)

## Decisions
- **Split once, patch per move.** `viewmodel::SculptMergedPreview::begin` composes the visible model of
  the sheet on the display grid (the same grid and stride as `composeModelSurface`) with the sculpted
  component taken out: the levels below its level as one field, the components below it in its level
  as one field, each component above it in its level, and each level above it (with effects). A move
  then recomputes only the surface cells whose bilinear sample can see a changed component cell and
  combines the working component with those cached layers using `relief::combine` in the same order
  as `relief::compose`, so the surface equals the composed model of a document holding the working
  copy (tested cell for cell, including coarse strides, clipping and mirror).
- **Dirty region from the controller.** `SculptController::takeDirty` reports a design-space rectangle
  (bounding box of the open stroke tail plus one brush diameter) accumulated since the last call, or
  `all` after a session start, Undo Stroke or Discard. The preview copies only those component cells
  into its own working component and does no work outside the rectangle.
- **Clipping and mirror.** A clip is pointwise, so it is applied to a crop of the rectangle. A mirror
  reads cells anywhere, so a level with a mirror redoes its effects and the model combine over the whole
  display grid on every move (still no re-sampling of other components). This is the one case whose move
  cost depends on the model; it is documented rather than hidden.
- **Hidden target.** The component and its level are shown while sculpted even when hidden in the document.
- **Rebuild on document change.** `SculptPanel::refresh` composes the rest again (once per document
  change, not per move) so edits to other components show up. If the model cannot be composed the panel
  falls back to the component alone (`onPreview`).
- **Cost accounting.** `cellEvaluations()` counts surface cells per layer consulted; tests compare a
  move on a small and a large model (equal) and against the start cost, instead of timing.
- **Display copy.** `ModelView3D::setPreviewSurface` copies the display surface (at most 512 x 512
  floats) per move, as the renderer already walks all of it each frame.

## Not done
- A mirror level is not incremental (see above).
