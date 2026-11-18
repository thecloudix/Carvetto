# ADR-0125: Editing vectors from the 3D view

Date: 2026-11-18 · Status: proposed (T-0346) · Builds on ADR-0121 (drawing in the 3D view), ADR-0029 (transform gestures), ADR-0030 (node editing)

## Context
T-0345 let the user draw on the zero plane of the 3D Model view. T-0346 adds selecting, moving, scaling, rotating and
node-editing those vectors there, with no second copy of the geometry code.

## Decision
1. **Same controller, two more modes.** `Draw3DShape` gains `Select` and `NodeEdit`. `Draw3DController` already feeds a
   `ToolController` with the picked plane point as pseudo-pixels (mm x on-screen scale), so the Select tool's marquee,
   click-select and move/scale/rotate handles, and the NodeEdit tool's handles, drags and Escape-revert are exactly the
   2D code paths, including their one-undo-step-per-drag coalescing and the snapping settings.
2. **Same refusals.** Edge-on and wrapped views refuse the pick (nothing happens), as for drawing.
3. **Overlay.** The app drapes the selection box with its handles, or the node/control handles, on the plane
   (`projectModelPoint`); during an edit drag the plane vectors are refreshed per step and the whole window on release.
4. Hit tolerances are in screen pixels because the pseudo-pixel scale is the on-screen plane scale.
