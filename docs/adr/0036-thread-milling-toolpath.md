# ADR-0036: Thread milling toolpath (geometry and conventions)

Date: 2026-11-09 · Status: accepted, implemented (T-0237).

## Context
The inventory lists Start Depth, Max Depth (M), Thread Length (L); single / multi point thread mills;
pitch preset (ISO Metric / Universal), pitch, diameter, fit tolerance, "create circles for internal
threads"; internal / external, right / left hand, cut down / up. The card fixes one result: an M6x1
internal thread advances Z by the pitch per revolution.

## Decision
1. **Input.** One thread per closed vector, on its area centroid (`drillCentre`: the hole of an
   internal thread, the pin of an external one), ordered and deduplicated like Drill.
2. **Tool.** Any end mill, its diameter the cutting diameter of the teeth. The tool database has no
   thread mill type; adding one is a database/file-format change outside this card. The cutter kind
   (single / multi point, with its threaded length) is a parameter of the toolpath, not of the tool.
3. **Path radius** (tool axis to thread axis), t the signed diametral fit tolerance, d the cutter
   diameter, P the pitch: internal `(D + t)/2 - d/2` (the teeth reach the nominal diameter); external
   `(D - 2h - t)/2 + d/2`, h = 17/24 H = 0.6134 P the ISO external thread depth (the teeth reach the
   root). Positive t = looser fit. Internal needs the cutter narrower than the bore D - 1.0825 P.
4. **Hand and direction.** A right-hand helix turns counter-clockwise while rising, so
   right + up and left + down are CCW, right + down and left + up CW. Climb/conventional follows from
   these plus internal/external, so it is not a separate parameter.
5. **Helix.** Teeth span [tip, tip + T]; single point T = P (one tooth), multi point T = the threaded
   length (P <= T <= L, else refused). The tip travels (L - T) + P: one pitch more than the length
   not covered by the teeth, so every groove is completed. Down starts at tip = top - T + P (the extra
   pitch above the thread) and ends at the thread bottom; Up starts at the bottom and ends at
   top - T + P. The tip never goes below start depth + L. The helix is one arc per revolution (start
   and end coincide, Z changing by exactly P, the last one landing exactly on the end Z) plus an arc of
   the remaining fraction.
6. **Entry/exit.** Internal: plunge on the axis, a half circle (diameter = path radius) to the helix
   start, and a half circle back to the axis, both in the direction of travel and tangent to the helix,
   at constant Z. External: rapid to a point one cutter diameter back along the tangent at the helix
   start, plunge, a tangent line in, the helix, a tangent line out of the same length, retract.
7. **Presets.** ISO 261 coarse pitches for M1 .. M64 and the unified coarse (UNC) series for 1/4 .. 1
   in, matched by nominal diameter (0.005 mm). A preset is stored by name and resolved when the
   toolpath is computed, so the record keeps the pitch field for Custom.
8. **Max depth.** M > 0 refuses a thread deeper than M; 0 = no limit.
9. **Create circles.** The flag is stored and `threadBoreCircles` makes the circles (bore diameter
   D - 1.0825 P). Inserting them into the document is not wired in this slice.
10. **Record and form.** `thread_mill` record in job.json only (like chamfer); a Thread Mill form
    (pitch hidden while a preset is chosen; threaded length only for multi point).
