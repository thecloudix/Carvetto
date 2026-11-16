# ADR-0103: Rotary wrapping: wrapped post output, Auto-Wrapping View, wrapped simulation

Date: 2026-11-16 · Status: proposed (T-0320)

## Context
Toolpaths of a rotary job are computed flat (ADR-0098: X/Y hold the length along the axis and the distance
round the circumference, Z the machine height). T-0320 makes the post-processor wrap them correctly, adds the
Auto-Wrapping View of the flat design on the cylinder, and lets the material-removal simulation run on rotary
jobs. The behaviour in `docs/reference/feature-inventory.md` (Rotary, wrapped and double-sided machining; the
post settings list) is the only source; it does not specify the details below, so the readings are mine and are
listed for owner review.

## Audit: what existed
- `core/post`: `ROTARY_WRAP_X/Y` parsed into `PostSettings::rotaryWrapX/Y` (axis letter, `-` negates); the
  interpreter turned the wrapped linear axis into `360 v / (pi D)` degrees in `[A]` and rendered the wrapped
  linear tag empty; arcs were linearised when wrapping; no diameter was a run error (ADR-0011).
- Only parsed or ignored: `MAXIMUM_ROTATION`; the job's along-axis (`Job::rotaryAxis`) and Flip Design never
  reached the post (`PostContext` had only `rotaryDiameter`); feed was written unchanged (mm/min) on a move that
  turns the rotary axis; no refusal for a post that wraps the wrong axis, or a move past the cylinder axis.
- `app/viewmodel` `makeSimulationSetup` refused rotary jobs ("cannot be simulated yet"); no wrapped view.

## Decisions (post)
1. **The wrapped axis is the circumference.** A cylinder along X turns about X, so the post must wrap **Y**
   (`ROTARY_WRAP_Y`); a cylinder along Y needs `ROTARY_WRAP_X`. `PostContext::rotaryAlong` ('X'/'Y', 0 = not
   stated, which keeps the old behaviour and tests) makes a post that wraps the axis the cylinder lies along a
   run error naming both axes. A rotary job with a post that wraps nothing runs but warns that the
   circumference is written as flat distance.
2. **Angle** = arc length / radius, in degrees: `A = c / (pi D) * 360` for the flat circumference coordinate `c`
   (document/machine coordinates, not relative to the datum; the datum is a document offset and machine
   coordinates equal document coordinates, as for X and Y). Moves are linearised, so a straight flat segment is
   a helix, which a controller reproduces by interpolating the linear axis and A together.
3. **Sign.** The post's `-A` negates the angle (a machine whose rotary axis runs the other way). **Flip Design**
   (the job flag, `PostContext::rotaryFlip`) also reverses the angle: the design is mirrored round the cylinder.
   The two combine as XOR (`-A` plus Flip Design is the plain angle). Flip Design does not mirror along the axis.
4. **Feed.** A flat feed is a speed over the surface (mm/min). A controller reads a move that includes the rotary
   axis as a straight line in mixed units (mm and degrees), so the feed written for a wrapped feed move is
   `F * sqrt(rest^2 + turn^2) / L`, where `L` is the surface length of the flat move, `rest` its length without
   the wrapped-axis component and `turn` the rotary change in degrees. A move that only travels along the axis keeps
   its feed; a move that only turns gives `F * 180 / (pi r)` degrees/min. The surface radius is used (the tool
   tip is below it, a flat toolpath makes the same approximation). Inverse-time output is unaffected (it is time
   based). `FEED_RATE_CHANGE` blocks write the nominal flat feed.
5. **MAXIMUM_ROTATION** (degrees, >= 0, 0 = no limit; parse error otherwise) is read as the largest rotary change
   one written move may make: a wrapped move (rapid or cutting) turning further is split into equal parts, the
   linear axes interpolated, so no output line turns more than the limit. A limit that would need more than 100000
   parts for one move is a run error. This is my reading of the setting name, by analogy with `MAXIMUM_LENGTH`
   (which is still only stored).
6. **Refusals / diagnostics** (only when the post wraps): error when a move goes to a Z at or below the cylinder
   axis (`PostContext::rotaryAxisZ`, set from the job as surface Z minus the radius) naming toolpath and move.
   Warnings: a tool wider than the cylinder radius (a flat toolpath on so small a cylinder is cut inaccurately;
   `ToolSnapshot` has no length, so "tool longer than radius" in the card is read as diameter), a toolpath whose
   wrapped extent is more than 360 degrees (it goes more than once round).
7. Callers (`save_toolpaths`, CLI) fill `rotaryAlong`, `rotaryFlip`, `rotaryAxisZ` from the job for rotary jobs only.

## Decisions (view and simulation)
8. **`geometry::CylinderMap`** (core, Qt-free): flat `(x, y, z)` to the 3D point on the cylinder. The axis is the
   world X (or Y) axis at Z 0; radial distance is `z - axisZ`; angle 0 is straight up; positive angle turns the
   top towards +Y (cylinder along X) or +X (along Y); Flip Design negates it. `unwrap` inverts it near a reference
   circumference coordinate. `wrapPolyline` splits segments by angle so a straight flat segment draws as a helix.
9. **Auto-Wrapping View** (`viewmodel/wrapped_view`): a software renderer (z-buffered triangles, orbit camera,
   deterministic, like `renderModel`) of a `WrappedSurface` (height grid in machine Z) plus depth-tested paths.
   Sources: the composed relief model (model height `h` wraps at radius `gap inside + h`, i.e. machine Z
   `rotaryModelBottomZ() + h`), the simulated stock, vectors (flat contours at the surface Z) and toolpath moves
   (arcs chorded). `RotaryViewState` is the flat/wrapped toggle; for a non-rotary job the view is always flat.
   `renderRotaryModel` returns the existing flat render or the wrapped one.
10. **Wrapped simulation.** The simulation stays flat (the same approximation as the flat toolpaths: the tool
    footprint is not re-projected onto the curvature): a rotary job's stock is the unwrapped cylinder, a
    `length x circumference` sheet whose height is machine Z, topped at the surface Z (radius for an axis zero, 0
    for a surface zero) and floored at the cylinder axis (`thickness = radius`). `makeSimulationSetup` no longer
    refuses rotary jobs. `wrappedSurfaceFromSimulation` draws the cut stock on the cylinder (cut cells tinted).
    No `.carv` change.

## Not covered (follow-up card)
App wiring of the above: the Auto-Wrapping View menu entry and toggle in the 2D/3D and simulation widgets,
drawing the wrapped raster in Qt, wrapped playback in the simulation panel, a rotary refusal dialog, and the
`--self-test` for them. Vectors in the viewmodel are drawn from the same contour polygonisation as the flat
canvas. A true re-projection of tool shape onto the curvature is out of scope.
