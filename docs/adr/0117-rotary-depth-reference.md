# ADR-0117: Rotary job depth reference

Date: 2026-11-17 · Status: proposed (T-0343) · Builds on ADR-0098 (rotary job), ADR-0103 (wrapped simulation)

## Context
The default rotary job zeroes Z on the cylinder axis, so the cylinder surface is at Z = radius. A Profile with cut
depth 2 computed Z around -2 with a safe Z of 5: below the surface, so the simulation cut nothing. The feature
inventory describes depths as "from the material surface"; it does not say what a rotary job with an axis zero does.

## Decision (my reading, for owner review)
1. **Depths are always measured from the cylinder surface**, whatever the Z zero. For a rotary job whose Z zero is the
   cylinder axis, the Z of every move of a surface-referenced toolpath is raised by the radius
   (`surfaceDepthShift`, applied when the record is made). With the surface zero the shift is 0. Cut depth 2 therefore
   cuts 2 mm into the cylinder under both choices; the toolpath, post and simulation then see true machine Z.
2. **Absolute-Z operations are not shifted**: Rough 3D, Finish 3D, Project Toolpath onto Model, Moulding and
   Rounding take stock/surface Z as fields. Their defaults now follow the job (`modelDefaults`): on a rotary job the
   stock top is the surface Z (radius or 0) and the bottom is the axis. Rounding already did this (ADR-0115).
3. No file format change: the shift is applied to the calculated moves, which are already stored.

## Alternative (noted for owner review)
Default the Z zero of a new rotary job to the material surface instead of the axis, and leave depths unshifted.
Rejected as the sole fix because the axis zero is a legitimate choice (the owner's reference lists it) and would still
cut nothing; it can still be adopted in addition by changing the `Job::rotary` default.

## Limits
Changing the Z zero after toolpaths exist does not recompute them (as for any job change); recalculate. Rest
machining priors of a shifted toolpath are compared against unshifted params, as before for non-rotary Z zero changes.
