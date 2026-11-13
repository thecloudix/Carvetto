# ADR-0054: V-carve inlay toolpath (pocket and plug)

Date: 2026-11-13 · Status: accepted, implemented (T-0265).

## Context
The inventory lists a "V-Carve Inlay Toolpath" (depths: Pocket Depth, Glue Gap, Surface Clearance; a
V-bit and clearance tools; a Plug Destination Sheet where the plug is mirrored and, on another sheet,
centred; a Plug Outer Boundary of Sheet Limits / Vector Offset / Boundary Offset). Its Pocket and Plug
sub-pages are not in the reference, so what each field does is decided here. Every point marked
**(owner)** is a product reading to be confirmed.

## Decision
1. **One toolpath per piece**, as the straight inlay (ADR-0040): `VCarveInlayParams::piece` is Pocket
   or Plug; the user adds one of each. New operation `VCarveInlay`, record `vcarve_inlay` (job.json
   only). Both pieces are built on `vCarvingToolpath` (ADR-0025): the V-bit carves along the medial
   axis and the flat areas are cleared by clearance tools (end mills) or by the V-bit itself. The
   carving tool must be a V-bit (the flank angle is what makes the pieces mate); clearance tools end
   mills. The surface is Z = 0 (no start depth).
2. **Pocket.** The V-carve of the vectors' even-odd region with flat depth = Pocket Depth (limited to
   the depth at which the V-bit reaches its full width, `CarveDepthLimited`). With flank slope
   t = tan(half the V-bit angle) the pocket depth at distance s from the vector is min(P, s / t).
3. **Plug.** Stock is carved away round the plug, so it stands on the backing with flanks widening
   downward; after cutting it off and turning it over it seats in the pocket. Let P = pocket depth, g =
   glue gap. The plug's top face (the face that goes into the pocket first) is the mirrored vectors
   inset by (P - g) x t. The stage V-carves the plug's outer boundary minus that top face, flat depth
   F = P + surface clearance (limited to full width). The flank then runs from the top face outwards
   and down to the vectors' outline grown by g x t at depth F. **Glue gap (owner):** the vertical gap
   between the plug's tip and the pocket floor; the flanks touch, the plug stands g proud of the
   surface (sanded flush after gluing). Test: pocket and plug surfaces, from the toolpath tips and
   the V-bit's cone, sum to P - g + gap = P: the gap is g on the floor and 0 on the flanks.
   **Surface clearance (owner):** how much deeper than the pocket the plug stock's flat background
   is carved, so the skirt round a seated plug clears the base. It has no effect on the pocket.
4. **Mirroring and sheet.** The plug is mirrored left-right about the vertical line through the middle
   of the vectors' bounds. On the same sheet it stays where it is (the user selects the plug's own
   copy of the vectors; this is the same "in place" reading as the inventory's mirror, **(owner)**). On
   another sheet (`plugSheet` = a sheet id, 0 = the vectors' sheet) it is also translated so the
   middle of its bounds is the middle of that sheet's material rectangle. In a double-sided job the
   plug's records are linked to that sheet. The core takes the sheet's bounds as an argument; the
   view model supplies them (`attachPlugSheet`). A sheet id that no longer exists refuses the job. The
   form offers the destination as a plain sheet-id number (the form has no document access).
5. **Plug outer boundary.** SheetLimits: the plug sheet's material rectangle. VectorOffset: the
   mirrored vectors' outline grown by `boundaryOffset` (outer outlines only, round corners).
   BoundaryOffset: the bounding box of the mirrored vectors grown by `boundaryOffset`. The flank needs
   room: the boundary must leave 2 x F x t beyond the top face, else the plug is refused
   (`ToolTooLarge`) rather than silently cut with a shallow flank.
6. **Limits.** A glue gap not below the pocket depth is refused. Parts of the shape narrower than
   2 x (P - g) x t have no top face and so no plug: reported as `RegionTrimmed` (and refused when none
   is left). Sharp (convex) corners of the pocket are pyramids while the plug's are rounded cones (the
   V-bit grows the top face outwards), so a plug is looser at corners, a property of V-bit inlays.
7. **Not in this slice.** Plug cut-out profile and backing removal, tabs, the plug on a rotary job,
   automatic placement of the plug away from the pocket on the same sheet.
