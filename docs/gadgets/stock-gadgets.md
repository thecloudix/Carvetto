# Stock gadgets

Gadgets that ship with Carvetto (`gadgets/stock/`). They are installed into your gadgets folder the first time
the application starts (and updated when a new release changes their version) and appear in the Gadgets menu.
Everything a gadget does is ONE undo step; if it fails or you press Abort, your drawing is not changed.
Design notes: `docs/adr/0113-stock-gadgets.md`. The API they use: `docs/adr/0108-scripting-api-and-gadgets.md`.

## Celtic Weave Creator
Makes the vectors for a woven grid, ready for Extrude and Weave.
* **Vertical / Horizontal strands** (1 to 30), **Strand spacing**, **Strand width** (smaller than the spacing).
* **Intersections**: *Alternating* (over, under, over, ...), *All cut vertical*, *All cut horizontal*, *Cut neither*,
  or *Custom*: one letter per crossing, rows from the top, rows separated by `/`, for example `VH/HV`.
  `V` = cut vertical (the horizontal strand runs over), `H` = cut horizontal (the vertical strand runs over),
  `N` = cut neither.
* **Cross section**: Round (a circular arc of the given height), Flat (rectangle) or Pointed (triangle), with a
  height; its width is the strand width.
* **Output**: open vectors for every strand piece on the layer "Weave rails" (a strand is cut at its `V`/`H`
  crossings, leaving a gap of one strand width), selected; and the closed cross-section profile on the layer
  "Weave cross section", to the right of the grid, standing on its base line.

## Drag Knife Toolpath
Select the vectors to cut first. The blade trails the tool centre by the **blade offset**.
* **Tolerance angle**: a corner sharper than this is swivelled; gentler corners are followed directly.
* A swivel overshoots to the corner plus the offset along the incoming direction, lifts to the **swivel depth**,
  turns on an arc of radius = offset around the corner, and goes back to the cut depth.
* **Cut depth** is reached in the given number of **passes** (each pass repeats the contour). Each contour starts
  with a lead-in of one blade offset along its first segment so the blade is aligned when the cut begins.
* **Use vector start points** (default on) cuts each contour as drawn, in selection order; off, contours are
  ordered nearest-first, open ones reversed when their end is nearer, closed ones started at the nearest vertex.
* Feed, plunge rate, safe Z, tool and toolpath name. The swivel depth cannot be deeper than the cut depth.
* The toolpath keeps the computed moves; it is not recalculated when the drawing changes.

## Keyhole Toolpath
Select the vectors that mark the entry holes (the bounding-box centre of each is used).
* **Slot direction** (degrees, 0 = right, 90 = up), **Depth**, **Slot length**, feed and plunge rates, safe Z.
* For every keyhole the tool plunges at the centre, slides along the slot for the slot length, and lifts out.
* **Create preview vectors** adds, on the chosen layer, a circle of the **Entry hole diameter** and a rounded
  slot outline of the **Slot diameter** at every keyhole.

## Job Setup Sheet Editor
Dresses the job sheet (Toolpaths > Create Job Sheet) of this job.
* **Header / footer colour** (`#RRGGBB`, empty = none), shown in the HTML sheet.
* **Include** Job Layout, Material Setup, Toolpaths Summary (with the tool list) and Individual Toolpath Summaries
  (the toolpath list). Job Notes is stored for a notes section the sheet does not have yet.
* **Restore default** puts everything back (the sheet prints exactly as before).
* Stored in the job file as the document variables `jobsheet_*`, so the settings travel with the job.
* Not available yet: a logo image.

## DXF Batch Processor
Choose a folder; every `.dxf` file in it (not sub-folders) is imported onto a layer named after the file and the
parts are placed left to right with the **gap** you give, each part's lower-left corner on the same base line.
Optionally a profile toolpath (outside, the **cut depth** and **tool** you choose) is made for every part. Files
that cannot be read are listed at the end and do not stop the others. At most 200 files of 16 MiB each; links are
not followed. The gadget can only read the folder you chose.
