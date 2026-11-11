# ADR-0041: Fluting toolpath (depth profile and conventions)

Date: 2026-11-10 · Status: accepted, implemented (T-0239).

## Context
The inventory lists Flute Depth; Flute Type (ramp over complete length / ramp at start / ramp at
start and end); Ramp Length or Ramp %; Ramp Type (linear / smooth); the ramp is placed at the
vector's start point. The card adds flat or ball-nose tools and a depth profile verified along a
straight and a curved vector.

## Decision
1. **Path.** The tool centre follows each vector as drawn (open or closed), from its start point
   along its direction; no offset. Curves are polygonised to `tolerance.chordal`; the profile is laid
   over the arc length of that polyline (`flutingDepthAt`).
2. **Profile.** Depth D below the surface. Ramp over complete length: D*f(s/L). Ramp at start: D*f(s/R)
   for s < R, then D. Ramp at start and end: also D*f((L-s)/R) for s > L-R. f is the identity
   (linear) or smoothstep 3t^2 - 2t^3 (smooth: zero slope at both ends). The ramp length R is the given
   length or the given percentage of the vector's length; a ramp longer than the vector is
   shortened to the vector (two ramps: half of it each) and reported (`LeadShortened`).
3. **Moves.** Rapid to the start at safe Z, plunge to the profile's depth there (the surface),
   straight feed moves with Z changing, retract. Samples are the polyline vertices, the ramp ends and,
   for smooth ramps, 32 equal steps per ramp (deviation from the exact curve below D/1000).
4. **Tools.** End mill (flat) and ball nose; the toolpath is the tool tip's, so the bottom of the
   flute is the profile for both; the shape of the flute section is the tool's.
5. **Passes.** A flute deeper than the cutting data's pass depth is cut in passes at levels
   k * passDepth (last = D). Pass k cuts only where the profile is deeper than level k-1, clipped to
   level k; runs are split at the crossings, each its own plunge/retract.
6. **Record.** `fluting` record in job.json only (like chamfer and thread mill); a Fluting form.
7. **Not in this slice.** Wrapped Fluting Layout (rotary gadget) and a start-depth offset.
