# ADR-0121: Drawing in the 3D view

Date: 2026-11-17 · Status: proposed (T-0345) · Builds on ADR-0014 (Qt-free viewmodel), ADR-0077 (3D model view)

## Context
T-0345 lets the user draw the basic shapes by clicking in the 3D Model view. The view is an orthographic orbit
camera over the composed relief; the shape tools already exist on the 2D canvas (`ToolController`).

## Decision
1. **Ray-to-plane pick.** `pickZeroPlane` finds the point of the model's zero plane (design plane, model height 0)
   under a pixel. The camera is orthographic, so the plane maps to the screen affinely; the pick projects the plane
   origin and the two unit axes with `projectModelPoint` (the same framing the renderer uses) and inverts the 2x2
   part. No second copy of the camera maths.
2. **Refusals.** A pick is refused (with a message, nothing happens) for an empty surface, the wrapped rotary view
   (the flat plane is not what is on screen), non-finite input, a view within about 6 degrees of edge-on
   (|sin| < 0.1), and a point more than 4 model diagonals from the model's centre.
3. **Reuse, not a second tool.** `Draw3DController` wraps a `ToolController`; it feeds it the picked point as
   pseudo-pixels (mm x on-screen scale), so snapping, numeric entry, the drag threshold, creation parameters and the
   single `AddObjectCommand` per shape are exactly the 2D behaviour. A Line is a Polyline that finishes at its second
   point.
4. **Plane without a model.** While a draw shape is set and the sheet has no usable model, the view shows a flat
   zero-height surface over the job material so there is something to draw on; the drawn vectors of the active sheet
   are laid on the plane over the surface. With drawing off the view is unchanged.
5. Selecting/moving/node-editing from the 3D view is T-0346.
