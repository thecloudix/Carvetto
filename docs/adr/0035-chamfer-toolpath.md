# ADR-0035: Chamfer toolpath (geometry and linked fields)

Date: 2026-11-09 · Status: accepted, implemented (T-0236).

## Context
The inventory lists Angle (A), Width (W), Cut Depth (C) (W and C linked), Max Cut Depth, Overcut,
Chamfer Type (inside/outside, slope up/down) and Cut Direction. The card fixes one result: a 90 degree
V-bit, width 2, gives cut depth 1.

## Decision
1. **Width is the full width the V cuts at the surface**, so the face spans W/2 horizontally and
   C = W / (2 tan A), A the half angle from the vertical (W = 2, A = 45 degrees: C = 1). The record keeps
   `width`; cut depth is derived (`chamferCutDepth`, `chamferWidth` for the inverse).
2. **Face and tool position.** The face is a plane at A. Type Inside/Outside is the side of the vector
   the face lies on, in Profile's terms. Slope Down: the face falls to the vector (tip on the vector).
   Slope Up: the face rises to the vector (tip W/2 into the face side). Overcut e carries the face along
   its plane past its bottom edge: the tool moves e further from the face (Down) or into it (Up) and
   e / tan A deeper.
3. **Implementation.** One constant-depth `profileToolpath` run at the tip depth, with the signed offset
   mapped to Profile's side and allowance (`On` for 0), so vector handling, order, direction, trimming
   and bounds are Profile's.
4. **Depth limit.** The cut is refused (InvalidParameters, nothing produced) when the tip would go deeper
   than the tool's maximum cut depth: V-bit cone height (D/2) / tan A (`vCarvingFullWidthDepth`); ball
   nose its radius.
5. **Round nose.** The ball is tangent to the face plane: the virtual cone apex is at the cut depth and
   the tip sits R (1 / sin A - 1) above it; refused when the contact point would lie above the surface.
   A ball cuts a scalloped, not a flat, face; this is the stated approximation.
6. **Form.** A V-bit's angle comes from the tool (copied into the params, field hidden); W and C edit
   each other. The record is `chamfer` in job.json only (like v_carve); the CLI has no chamfer command.
