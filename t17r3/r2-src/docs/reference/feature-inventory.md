# Vectric Aspire 12.5 — Feature Inventory (from the official Reference Manual)

**Source:** https://docs.vectric.com/docs/V12.5/Aspire/ENU/Help/ (all sections below come from **V12.5** pages unless tagged **[V12.0]**).
The V12.0 "What's New" page was used only for §7. Extra sources are tagged where used (vectric.com/upgrade).
Page URL pattern: `.../Help/form/<Page Name>/index.html`. Some pages exist but are not in the sidebar TOC (for example `form/Component Properties`).

**Legend:** `[NOT FOUND]` means the docs have no page or section for this item. `[PARTIAL]` means the page exists but does not list all its fields.

---

## 0. Application shell (menus, panels, views)

### Main menus
- **File:** New · New from template… · Open… · Close · Save · Save As… · Save As Template… · **Increment and Save** (saves as `_N+1`, never overwrites) · Print · Print Preview · Print Setup · Import ► (Import Vectors / Bitmaps…, Import Component / 3D Model…, Import PhotoVCarve or Cut3D Toolpaths…) · Export ► (EPS, DXF, AI, SVG, PDF) · Open Application Data Folder… · Exit.
- **Edit:** Undo · Redo · Cut · Copy · Paste · Delete · Selection ► (Select All Vectors, Select All Open Vectors, Select All Duplicate Vectors, Select All Vectors On Current Layer, Unselect All, Vector Selector…) · Align Selected Objects ► · Join Vectors · Curve Fit Vectors · Nest Selected Vectors · Job Size and Position · **Swap Sides** (two-sided jobs) · **Notes** (free-text job notes) · Document Variables · Snap Options · Options.
- **Model:** Create Component ► (Two Rail Sweep, Extrude and Weave, Turn and Spin, Create Textured Area Component, Add Zero Plane) · Create Component From… ► (Bake Components, Create Component From Visible Model, Add Draft To Model, Create an Offset Model, Slice Model, **Create Component from Toolpath Preview**; Shift or Ctrl changes the result) · Export as STL · Export as Grayscale Bitmap (highest areas white) · Export as 3D Clipart (.3dclip) · Sculpt Visible Model · Smooth Selected Components · Reset Inside Selected Vectors · Reset Outside Selected Vectors · Split Selected Component · Create Vector Boundary Around Selected Components · Emboss Component · Scale Model Z Height · Distort Selected Component · Replace Below · Selected Component Properties.
- **Machine:** Machine Configuration… · Add Machine… · Install Offline Package… · Search for Machine Online… · Post-Processor Management… · Install Post-Processor… · Update Post-Processor Database.
- **Toolpaths:** Show Toolpaths Tab · Recalculate All Toolpaths · Toolpath Drawing ► (Draw 2D Previews, Make 2D Previews Solid, Auto Open 3D View, Draw rapid moves (red), Draw plunge moves (cyan), Draw retract moves (green), Draw join moves (purple)) · Preview Simulation Quality ► (Standard; High 2×; Very High 4×; Extremely High 8×; Maximum 25× slower) · Templates ► (Save Selected Toolpath as Template, Save All Visible Toolpaths as Template, Load Template) · Merge Visible Toolpaths · Create Job Sheet · Materials… · Tool Database….
- **View:** Zoom to Drawing · Refresh 2D View (F5) · Guide Lines ► (Guides Visible, Delete All Guides, Lock All Existing Guides, Unlock All Existing Guides) · Draw Origin · Draw Modeling Plane · Draw Material Block · Multi Sided View · Auto-Wrapping View (rotary) · Color Shaded View · Use Shaded Background · Light follows viewer · Use Shadow Shading (Ctrl+NumPad± sets intensity; not for 2-sided or rotary jobs) · Save Shaded Image (BMP/JPG/PNG/GIF) · Tile Windows Horizontal / Vertical · Cascade Windows.
- **Gadgets:** Install New Gadget… · Gadget Shortcuts · plus the installed gadgets (see §6).
- **Help:** Help Contents · Keyboard Shortcuts · Video Tutorial Browser… · What's New · Release Notes · Third Party Licences · Enter License Code · Vectric Online FAQ / User Forum / Support / User Portal links · Post Processor Editing Guide · Migrate From Older Version · Check for Updates · Run Kickstarter · About.

### Interface elements (Interface Overview, View Toolbar)
- **Panels:** Main Menu Bar; Design Panel (left; Drawing and Modeling tabs, now one combined panel [V12.0]); Toolpath Tab (right; operations at the top, Toolpath Tree below); 2D Design Window and 3D View (F2/F3, or tabbed); interface layout buttons; quick drop-downs for Layer, Sheet and Component Level; push-pin auto-hide panels; Switch Layout buttons for the design and machining layouts (F11/F12); a Help (?) icon on every form; 3D View help prompts ("Show Usage Information").
- **View Toolbar:**
  - Window layout: stack 2D/3D vertically or horizontally.
  - Visibility toggles: component previews, bitmaps.
  - 2D toolpath drawing (lines or solid).
  - Sheet, Layer and Level drop-downs.
  - Snapping toggles: geometry, smart, grid.
  - View: Pan/Twiddle; zoom to box; zoom to all (Shift = active sheet only); zoom to selection.
  - Two-sided: flip-direction indicator, active side, single/two-sided view toggle.
  - Rotary: wrapped/flat 3D toggle.
  - [V12.0] also lists toggles for vectors, toolpaths and keep-out zones.
- **2D View Controls:** Pan; Zoom (wheel, or Shift+right-drag); Zoom Box (Shift+click zooms out); Zoom Extents; Zoom Selected; sheet outlines with the active sheet marked; out-of-bounds indication; double-click a sheet to activate it.
- **3D View Controls:**
  - Mouse: 3D Twiddle (right-drag); Zoom (wheel); Pan (Ctrl+right-drag, middle-drag, or left+right drag).
  - Zoom to box / fit / selection.
  - Orthographic/Perspective toggle.
  - Rotate around model (rotary jobs, wrapped view).
  - **View Control** (orientation cube: click a face to view it, double-click for the opposite face).
- **Common on-canvas handles (every drawing/editing tool, in 2D and 3D):**
  - Middle handle moves (Alt = one axis); a second click switches to the rotation centre and the Rotational Anchor can be dragged.
  - White corners scale proportionally (Alt = non-proportional; Shift = about the centre).
  - Edge handles scale on one axis (white in 2D, black in 3D; Shift = proportional).
  - Rotate: black corner (2D) or top rotation arrow (3D); Alt = 15° steps.
  - 3D only: white circles edit form values in place.
  - Exact entry: release the mouse, type a value, press Enter. In 3D, Tab moves from the X field to the Y field.
- **Calculation Edit Boxes:**
  - Expressions evaluate when you type `=`. Operators: `+ - * / ^` and parentheses.
  - `'` means feet.
  - Letter variables: W/X = width, H/Y = height, T/Z = thickness, P = π, I = mm→inch, M = inch→mm.
  - `{DocumentVariable}` inserts a document variable (see §1).
- **Keyboard shortcuts** (Keyboard Shortcuts page):
  - General: Ctrl+A; Esc; N (node edit); T (Scale form); M (Move form); R (Rotate form); J (Join form); 9/0 (rotate 45°); Z (zoom box); Ctrl+F; F/F6; F1–F5; F8 (Drawing/Modeling tab); F9 (centre in material); F10 (Alignment Tools); F11/F12; Ctrl+L (Layers tab); Ctrl+D (Drawing tab); Ctrl+M (Measure); Ctrl+Shift+Alt+V (composite model volume).
  - Two-sided: 1 = Top side, 2 = Bottom side, `=` = Multi-Sided View.
  - Groups: G, U, Ctrl+U, Shift+U, Ctrl+Shift+U.
  - Nudge: arrows (Ctrl = smaller step, Shift = larger, Ctrl+Shift = fixed distance).
  - Drag modifiers: Ctrl+drag pastes a copy; Alt+drag moves axis-aligned; Ctrl+Alt+drag copies axis-aligned.
  - Quick keys typed during a drag are listed under each tool in §2.

---

## 1. Job setup, sheets, layers, levels, variables, snapping, guides

### Job Setup (Job Setup Form; separate pages for Single, Double and Rotary)
- **Job Type:** Single Sided / Double Sided / Rotary.
- **Job Size:** Width (X), Height (Y), Thickness (Z). Rotary jobs use Length and Diameter instead.
- **Units:** inches or mm.
- **Z-Zero Position:** material surface or machine bed. Double Sided adds **Zero off same side**. Rotary offers material surface or **Cylinder Axis** (recommended).
- **XY Datum Position:** any corner or the centre, shown as a red square in 2D. **Use Offset** sets a datum other than (0,0).
- **Flip Direction Between Sides** (Double Sided): horizontal or vertical flip.
- **Orientation** (Rotary): **Along X Axis** / **Along Y Axis**, plus **Flip Design**.
- **Design Scaling:** when an existing job is resized, scales the existing drawing proportionally.
- **Modeling Resolution:** Standard (fastest) / High (~3× slower) / Highest (~7× slower).
- **Appearance ► Material Settings:** colour or material effect for the base 3D model.
- **Job Templates:** `.crv3dt` / `.crvt` files that store dimensions, machine setup and geometry. Created with File ► Save As Template; used with New from template.
- **Native file types:** `.crv3d` (Aspire 3D), `.crv`, `.crvt`, `.crv3dt`. Toolpaths are stored inside the file.

### Material Setup (Toolpaths tab; one form for Flat jobs, one for Rotary)
- **Active Sheet**; **Thickness** (Flat) or **Diameter** (Rotary); **Show detailed summary on toolpath tab**.
- **Side Z-Zero:** Material Surface / Machine Bed. Rotary: cylinder surface / centre. In a two-sided job this applies to each side separately.
- **Model Position in Material:** Gap Above / Gap Below Model (Rotary: Gap Outside / Gap Inside Model). Double-click a slider line to snap the model to top, centre or bottom. Also: Model Thickness (read-only, with a **Set…** button that opens Set Model Height) and Modeling Plane Z (read-only).
- **Rapid Z Gaps above Material:** Clearance (Z1) and Plunge (Z2).
- **Home / Start Position.**

### Sheets (Sheets Tab, Sheet Management, Edit Sheet forms for Single, Double and Rotary)
- Multiple sheets per job, each with its own size, Z-zero and datum. The active sheet is shown in bold. Only one sheet shows in the 3D view.
- **Actions:** Add New (duplicates the selected sheet, or uses the active sheet's size); Activate (double-click the list entry or the outline in 2D, or use the Sheet drop-down); Edit (multi-select with Ctrl edits all selected); Rename; Delete (at least one sheet must remain); **Duplicate**, which copies 2D objects, the component tree and toolpaths (duplicated toolpaths are flagged with a red "!" and must be recalculated); Visibility toggle.
- **Toolpath link:** toolpath groups are tied to their sheet, and toolpaths can be copied or moved to another sheet or side (12.5).

### Layers (Layer Management)
- **Layer list:**
  - Visibility icon; lock (padlock); colour swatch, which colours every vector on the layer (More Colors…); empty-layer indicator; rename by double-click.
  - Reorder with arrows or drag-and-drop. Order sets stacking for bitmaps and component previews.
- **Right-click menu:** Activate, Show (four variants), Hide, Lock/Unlock, Insert New Layer, Delete, Rename, Merge Visible, Select Layer Vectors, **Duplicate** (12.5).
- **New Layer settings:** Name, Drawing Color, New Layer is Visible, New Layer is Active.
- **Moving objects between layers:** right-click an object ► Copy to / Move to ► Layer / Level / Sheet / New Layer… (and Other Side in two-sided jobs).
- **Layer Filter:** All Layers, or Current sheet and active layer (hides empty layers; 12.5).

### Levels (3D) — see §4 Component Tree.

### Document Variables
- Named numeric values used in any Calculation Edit Box as `{Name}`. A name must start with a letter, followed by letters, digits or `_`.
- Create them from Edit ► Document Variables, or right-click an edit box ► Insert New Document Variable.
- Import and export as a text file; on import, same-named variables are overwritten.
- A variable cannot be deleted while a toolpath uses it or while a toolpath form is open.

### Snapping (Snap Options, F4)
- **General options:** Display Text at Cursor; Snap to Guides; Snap Guides to Geometry; Snap to Grid (with Grid Spacing); Snap Distances (zoom-dependent); Snap to Job Center and Corners; Fixed Nudge Distances; Snapping Radius (pixels).
- **Geometry Snapping:** object centres, span end and mid points, arc centres, intersections, angle and distance guides, guide intersections.
- **Smart Snapping:** Object Bounds; Vertical/Horizontal lines through "woken" nodes; Tangents; Perpendicular to Tangents; Connecting Lines (with midpoint); Span Geometry; Angular Constraints; Job Edges & Centre; End Point Alignment (while rotating).
- **Modifiers:** Shift disables snapping temporarily. Geometry, smart and grid snapping each have a toolbar toggle. Settings persist between sessions.
- **12.5:** the snap grid and guide lines also show in the 3D view, with adjustable colour and thickness.

### Guides
- Drag a guide from the rulers (left ruler = vertical guide, top ruler = horizontal). Shift overrides snapping to ruler units. Ctrl+drag from an existing guide creates a relative guide.
- **Guide Properties** (right-click a guide): New Position, **New Angle** (angled guides; only relative parallels allowed), Lock Guide, Create New Guide (absolute or relative).
- Visibility toggle (corner of the 2D view, or the View menu); Delete All; Lock/Unlock All. In 3D, click a guide to edit its position.

### Options dialog (Edit ► Options)
- **Window Layout:** Save Tab Layout; Save Dialog Layout; Save View Layout; Display Splash Screen; Top / Bottom Side Ruler Color; Tab location.
- **3D View:**
  - Background: Shaded Background Style (Solid/Gradient/Image); Background Color; Gradient Background Color; Image File Path; Print 3D View Shaded Background.
  - Startup display: Draw Origin; Draw Material Block; Use Color Shaded View; Shadow Shading.
  - Rendering: Use Software Renderer; Anti-Aliasing (None, 2X, 4X, 6X); Smooth Shading; Animate Camera Moves.
  - View Controls Mode (Modern/Legacy); Show Usage Information.
- **Toolpath:**
  - Display: Show Toolpath Operations with Preview; Auto Open 3D view; 2D Solid Preview Color; Create 2D Previews.
  - Calculation: Select Sheet When Edit Toolpaths; Toolpath Geometry Fixing Timeout; **Drop Tool** (drop the tool onto the surface instead of projecting it); 2D / 3D / VCarve Toolpath Tolerance; **Use new raster**.
  - List management: Maximum Toolpath Undo Stack Size (MB); Append duplicated toolpath; Append new toolpaths to selected group.
  - Checks and filtering: Check depth of all toolpaths; Default toolpath filtering mode.
- **General:** Use Graphics tablet (for sculpting); Process User Files; Recent File / Recent Font list sizes; Show additional file operations; Show the clipart Subfolder Contents; Always open local documentation; Smooth Join Vectors; Default to last used text anchor position; File Dialog Default (Global/Operation/Job).
- **Confirmation Choices:** Sheet Toolpath Deletion Check; Toolpath Vector Intersections Check; Radiused / Sharp Offset Intersections Check; Warn about unsuitable components for embossing; Ask which version of the tool database to download.

---

## 2. Drawing / vector creation and file I/O

### Shape tools
Each shape tool has an Exact Size form, interactive drag creation, Quick Keys typed during the drag, edit-in-place (Shift+click selects the next object to edit) and the common handles from §0.
- **Draw Circle:** Radius or Diameter, centre X/Y. Alt+drag draws from the centre. Quick keys: `value R`, `value D`.
- **Draw Ellipse:** Width, Height, origin. Alt draws from the centre; Ctrl draws a circle. Quick keys: `w,h`, `value X`, `value Y`, `W/H`.
- **Draw Rectangle:**
  - Fields: Width, Height, XY Origin. **Corner Type:** Square / Radiused Externally / Radiused Internally, with Radius.
  - Four green handles adjust the corner radii individually.
  - Alt draws from the centre; Ctrl draws a square.
  - Quick keys: `w,h`, `X`, `Y`, `R…X`, `R…Y`, `W…H`, `R…W…H`.
- **Draw Polygon:** number of sides, centre, Radius or Diameter. Quick keys: `D`, `S` (sides), `R`.
- **Draw Star:** Number of Points, Center Point, Outer Radius, **Inner Radius Percentage**. Quick keys: `D`, `R`, `I` (inner %), `P` (points).
- **Draw Line (Line / Polyline):**
  - Click adds points; click-and-drag adds a Bezier span. **S** toggles curve/straight mode; **T** draws a tangent line (point↔arc, arc↔arc). **Tab** closes the vector; Space finishes and starts a new line; Backspace undoes the last point. Ctrl+click on an open vector's end extends it.
  - Quick keys: `dist`, `x,y`, `angle A length L`.
  - **Next Point** form: X/Y, Angle/Length, dx/dy, Add, Undo, Finish.
  - 12.5: arrow keys cycle the input modes in the 3D view.
- **Draw Arc:** Through 3 Points; Center, Start and End; Exact Size (Start X/Y, End X/Y, Radius or Height).
- **Freehand Drawing:** Smoothness slider; **Stabilise** slider; End Vector at Cursor; Fade selected Bitmap.
- Fluting, wave and spiral drawing tools: `[NOT FOUND]` as drawing tools. The nearest equivalents are **Create Vector Texture** (below) and the **Wrapped Fluting Layout** / **Wrapped Spiral Layout** gadgets (§6).

### Text
- **Create Text:** modes **Text** and **Text in a Box** (the auto-layout mode).
  - Fonts: TrueType/system list; vertical fonts (names starting with @); **Single Line** (engraving) fonts.
  - Height; Text Alignment; Anchor.
  - **Text in a Box:** Bounding Box Dimensions; Margin Size (None / Normal 80% / Wide 60%); Vertical Stretch (none, line spacing, characters); Horizontal Stretch (none, word spaces, kerning, character size).
  - **Spell Checker** (not available for Japanese).
- **Text Selection (interactive):**
  - Letter **kerning:** click between letters; Shift spreads them; Ctrl doubles the step; Alt applies to every pair on the line. 2D view only.
  - **Line spacing:** the same click and modifier scheme.
  - **Arching:** green handles bend the text; white handles move the text or the arc centre; red handles rotate about the centre (Ctrl = 15° steps); blue handles change the radius.
- **Text On A Curve:** Maintain text Size / Scale text to fill curve; Spacing slider; Above / On / Below Curve; Text on other side; Offset Distance; Left / Middle / Right alignment (closed curves align to the start node); Align to Curve / Keep Vertical; drag the anchor handle to slide the text. Right-click ► **Remove From Curve**.
- **Right-click on text:** Remove From Curve; **Break Block into Lines**; **Convert to Vectors**. **Convert Text to Curves** is also a tool page.
- **12.5:** text can be created, arranged and rotated directly in the 3D view.

### Dimensions
- **Types:** Length (any angle), Vertical, Horizontal, Angle (arm, centre, arm, then place), Arc (radius, or diameter with **Show Diameter**; not available for Bezier or polygonised arcs).
- **Options:** Decimal Places (angles have a separate setting, up to 6); Font and Text height; Offset; **Use Custom Text** (saved per dimension); **Place Dimensions on Layer** (default "Dimensions" layer).
- Dimensions are associative: they update when scaled. Shift-select an existing dimension to edit it.
- **12.5:** dimensions are editable through control points (3D handles: white = size, text position, arc radius; black = leg length, text height).

### Measure
- **Measure – Inspect** (Ctrl+M) has two modes:
  - **Measure Between 2 Points:** Distance, Angle, X/Y Distance, First Point, Cursor Position.
  - **Span / Contour Properties:** span Type, Length, Start/End points; for the whole vector, Area, Perimeter, Num. Spans.

### Bitmaps
- **Trace Bitmap:**
  - Trace Selected Area; Black and White mode (Threshold); Color mode (reduced to 16 colours, colour-count slider).
  - Trace Color, with similar-colour linking checkboxes and Reset; Pick from 2D View.
  - **Corner Fit** (Loose to Tight); **Noise Filter** (minimum pixel size); Bitmap Fading; Preview / Apply.
- **Edit Picture:** Contrast, Brightness, Gamma, Invert, Grayscale, Add Border (Rectangular / Oval, Fade width).
- **Crop Bitmap:** select the bitmap, then Shift-select closed vectors; the crop keeps what is inside.
- **Bitmap Appearance** (right-click): fade slider for unselected bitmaps (default 50%). 12.5 keeps the original colours when fading.
- **Create Vector Texture:** generates wavy line-fill vectors. Angle (−90 to 90°); Max. Spacing; Line Spacing variation; Wavelength; Amplitude; Noise; Place Vectors on Layer / Name; Preview.

### Plate Production (data-merge badge layout)
- **Layout:** Material Size; Plate Size (taken from the selected vectors' bounding box); Sheet Margins (Top/Bottom/Left/Right or Equal Margins); Spacing; Number of Plates (calculated).
- **Toolpath Options:** calculates the template's toolpaths for every plate.
- **Mode:** **Layers** (overflow goes to new layers) or **Sheets** (each plate array on its own sheet; 12.5).
- **Data merge:** Import from file…; Separator (Comma/Tab/Semicolon/Space); First row is column names; **Variable Assignment** (template text `!!VAR!!` mapped to a column); **Number Formatting** (counter variable).
- Calculate / Close.

### File import and export
- **Open as a job:** CRV3D, CRV, CRVT, CRV3DT, DXF, DWG, EPS, AI, PDF (each page goes to its own layer), SKP.
- **Import Vectors / Bitmaps** (one unified import button [V12.0]):
  - Vectors: DXF, EPS, AI, PDF, plus SKP (see below).
  - Bitmaps: BMP, JPG, TIF, GIF, PNG, and **WebP** (12.5).
  - **SVG import is not listed** in the 12.5 import docs.
- **SketchUp (.skp) import:**
  - Layout: **Exploded Flat Layout**, with Part Orientation (Auto Orientate / Orientate by material) and Gap between parts; or **Three Views – Front, Top, Side**.
  - Create Circles / Arcs (refit polygonal geometry).
  - **import visible data on selected layers.**
  - Group imported parts; Keep components starting with `__` together; Replace outer boundary (flat jobs only).
- **Import PhotoVCarve or Cut3D Toolpaths** (`.pvc`, `.v3d`; the user guide also lists Design and Make Machinist `.v3m`): imports pre-calculated 3D toolpaths as a movable grayscale thumbnail. They can be moved, mirrored and rotated but not scaled; Start Depth, feeds and speeds are editable.
- **Export (vectors):** EPS, DXF (keeps layers), AI, SVG, PDF. With nothing selected, all visible vectors are exported.
  - **PDF Export dialog:** Export All / Export Selected Only; per-sheet checkboxes (each sheet becomes a page); All visible layers / Selected layers only; **Export Job Bounds**.
- **Printing:** Print, Print Preview and Print Setup in the File menu, plus the Options setting "Print 3D View Shaded Background". `[PARTIAL]` (no dedicated print page).
- 3D import and export are covered in §4.

---

## 3. Vector editing and transforms

### Selection
- **Modes:** **Vector Selection** (default), **Node Editing** (N), **Interactive Selection** (click the selection again to show the handles: move, rotate, scale, rotation centre). No skew or shear is documented.
- **Box selection:** dragging left→right selects objects fully inside; right→left selects objects inside or touching.
- **Edit ► Selection:** All / Open / Duplicate / On Current Layer / Unselect All.
- **Right-click ► Select:** Select by type, **Invert selection**, **Select by Part** (smart part selection: every vector inside a boundary; 12.5).
- **Vector Selector:**
  - Filters: Open vectors / Closed vectors / Only Circles (diameter and tolerance); layer filter (specific visible layers or all).
  - In toolpath forms: **Associate with toolpath** (the selection is re-applied on recalculation and saved in templates) and **Set toolpath Cut Depth from imported vectors**.
  - Objects count readout.

### Node Editing
- **Mouse:** drag nodes, control points and spans; Ctrl+drag moves a whole arc or Bezier span. Shift or a selection box selects several nodes.
- **Span right-click menu:** Convert to Line / Bezier / Arc; Insert Point; Insert Midpoint; Cut Vector; Delete Span; **Keep Bezier Tangency**; Reverse; Close; Join; Exit.
- **Node right-click menu:** Delete Point; Smooth; Insert Point at virtual midpoint; Cut Vector; Start Point / Extend; **Mirror Mode** (horizontal/vertical, nodes move symmetrically); Close; Join; **Properties** (exact X/Y).
- **Keys:** I, D, S, C, B, A, L, P, X/Y (match coordinates to the first node), H/V (mirror mode), K, J. Typed values move nodes relative or absolute.
- **12.5:** the full node-editing set works in the 3D view, with per-element numeric fields (line length, Bezier handles, arc radius and centre). **Quick Node Editing with Shapes** switches between shape tools and node editing without leaving the tool.

### Transforms
- **Move Selection** (M): Anchor; Absolute / Relative; X/Y Position.
- **Set Size / Scale** (T):
  - Mode: Scale selection / Scale items individually.
  - Anchor; **Link XY**; **Auto Scale Z** (components).
  - Typed `value S` scales by a factor.
- **Rotate** (R): pivot snap points (corners and centre) or **Use Coordinates**; **Relative / Absolute** rotation. Alt = 15° steps; 9/0 = 45°.
- **Mirror:**
  - **Create a mirrored copy**; **Use Rotated Bounds**; **Flip About Line** (Shift-select a line as the mirror axis).
  - H/V keys with Ctrl (copy) and Shift (about the material centre).
- **Distort Object** (vectors or components, not mixed):
  - Envelope types: **Bounding Box Distortion**; **Along a Single Curve** (the last selected object is an open vector); **Between Two Curves**.
  - The envelope is edited with the node-editing tools. **Use Rotated Bounds**; **Bake Distortion**.
- **Alignment Tools** (F10):
  - Center in Material (F9) / X only / Y only.
  - Align to the last-selected object: Center, Vertical Center, Horizontal Center, Left/Right, Outside Top/Bottom.
  - **Space Selection:** horizontal or vertical even spacing; Ctrl = use selection order; **Inside last vector**.
- **Move to Front/Back** (2D stacking order within a layer).

### Boolean and shape operations
- **Weld Vectors:** union of overlapping closed vectors (keeps the inner regions of text characters).
- **Subtract Vectors:** removes the last-selected shape from the others.
- **Overlap Vectors:** keeps the overlap with the last-selected shape. This is Vectric's intersect; no separate "Intersect" tool exists.
- The Weld, Subtract and Overlap pages do not list form fields. `[PARTIAL]`
- **Trim Objects** (boundary trim): Clear inside boundary / Clear outside boundary / **Refit Curves After Clipping**.
- **Interactive Vector Trim:** click or drag across spans to trim to the nearest intersections; **Rejoin Trimmed Sections**. 12.5 adds trim in the 3D view (spans to be deleted show in red).
- **Create Fillets:** **Normal Fillet**, **Dog-Bone Fillet**, **T-Bone Fillet**, with a radius. 12.5 adds fillets in the 3D view.
- **Chamfer (vector tool):** `[NOT FOUND]`. Only the Chamfer *toolpath* exists (§5).
- **Extend Vectors:** interactively extends an open-vector end to its intersection with another span.
- **Join / close:**
  - **Join Open Vectors** (J; automatic join and close within a user tolerance).
  - **Join / Close Vector with a Straight Line.**
  - **Join / Close Vector with a Smooth Curve** (two methods).
  - **Join/Close by Moving End Points** (to the midpoint).
- **Fit Curves To Vectors** (Curve Fit): Circular Arcs / Bezier Curves / Straight Lines; Tolerance; **Keep Sharp Corners** with Max Angle; Replace selected vectors.
- **Smart smooth:** `[NOT FOUND]` as a named tool. Smoothing is available through node Smooth (S) and Fit Curves.
- **Offset Vectors:** Outwards (Outwards/Right for open vectors) / Inwards (Inwards/Left); Distance; **Create sharp offset corners**. Works in 2D and 3D. The docs do not list "outline open vectors" or "delete originals". `[PARTIAL]`
- **Vector Boundary:** Offset Boundary (distance); **Rubber Band Boundary**.
- **Vector Unwrapper** (rotary jobs only): turns a cross-section into a contour for two-rail rotary modelling. Center options: Use center of contour / Use center of workspace / Use custom point. **Refine unwrapped vectors.**

### Copies and layout
- **Array Copy:** Selected Objects Size (readout); Rows and Columns; Spacing (**Gap** or **Offset** X/Y); **Symmetry** (block mirror/rotate buttons); **Row/Column displacement** (offsets alternate rows or columns); Group Copies.
- **Circular Copy:** Rotation Center (X/Y or drag); Rotate Copies; number of copies; **Total Angle** or **Step Angle** (a negative step goes counter-clockwise).
- **Copy Along Vectors:** Copy Object or **Copy Circles** (diameter); Distance between copies; **Force even spacing**; or Number of copies; **Align Objects to curve**; Create Copies on new layer; Reverse Direction.
- **Nest Parts:**
  - Parts: closed vectors, text, components and contained open vectors; overlapping or contained items form one part.
  - Spacing: Tool Dia. (D); Clearance (C); Border Gap.
  - Fit options: Rotate Parts to find best fit (step angle); Mirror parts to find best fit; **Allow parts inside other parts**; **Nest two-sided parts**; Remove original parts; **Filler Part**.
  - Start and direction: Nest From (corner); Nest Direction (X/Y).
  - Per-part properties: **Number of Copies**.
  - Area: **Nest Boundary** (vectors on a layer, including holes/exclusions); Active Sheet; **Nesting Sheets (Customize)** via the Custom Sheet Selection form. 12.5 can nest onto a chosen sheet instead of always creating new sheets.

### Grouping
- **Group** (G); **Ungroup** variants: U (to original layers), Ctrl+U (to the group's layer), Shift+U / Ctrl+Shift+U ("deep" ungroup).

### Validation
- **Vector Validator:**
  - Checks: Overlapping contours, Intersections, Zero-length spans.
  - Controls: Search / Stop; **Fix zero length Spans**; **V-Carving Mode** (ignores intersections inside text).
  - Scope: the selection, or all visible vectors on the current side and sheet.
- **Select All Duplicate Vectors** (Edit menu).

---

## 4. 3D modeling

### Component Tree and Levels
- **Component Tree:** levels contain components. Components can be grouped and ungrouped.
- **Level right-click menu:**
  - Combine mode; Show/Hide; Select components.
  - **Effects:** **Clipping** (Apply / Update / Remove: dynamically clips the level to selected closed vectors); **Mirror Mode** (Left to Right, Right to Left, Top to Bottom, Bottom to Top, plus the four quadrant modes; non-destructive and dynamic); **Wrapping** (rotary jobs: wraps components that run past the edge).
  - Insert / Delete / Rename; **Export as .3dClip**.
- **Component right-click menu:** Combine mode; Front/Back; Copy/Duplicate; Export as .3dClip; Group/Ungroup; Delete/Rename; Show (This / Only This / All But This / All); Hide (This / All); Properties; **Move to Level**; Move Up/Down within the level (3D); **Create sculpting brush**; **Re-import model** (imported meshes); **Bake** from the context menu (12.5).
- **Combine Modes:** **Add**, **Subtract**, **Merge** (Merge High: keep the higher value), **Low** (Merge Low: keep the lower value), **Multiply**. They apply to components and to levels.
- **Components drop-down** on the document toolbar [V12.0]: switch the active level, toggle visibility, set the combine mode, add a level.

### Component Properties
- **Name**; **Combine Mode**; **Shape Height** (% scale); **Base Height** (Z offset; for a level it is applied after combining and is excluded from baking).
- **3D view height handles:** Model Height / Shape Height / Base Height; double-click resets the value.
- **Fade:** linear Z fade from an anchor toward a second point, by a percentage.
- **Tilt:** pivot point, a point to raise, and an angle.
- **Bake** (makes the dynamic properties permanent).
- **Appearance:** Same as Parent / Solid Color / Use Material (wood, metal, stone, plastic) / Color from Children.
- **Highlight Selected Component**; Close.
- Levels expose only Name, Combine Mode, Base Height and Appearance.

### Shape creation (Create Shape)
Replaced the earlier Shape Creator in [V12.0]. Shapes remember their creation settings and can be re-edited.
- **Common buttons for all modeling forms:** Combine with other components (name and mode); Reset; Apply; Start New Component; Close.
- **Create Shape – Plane (Flat):** Base Height, Fade, Tilt. Shift+click the button creates a material-size zero plane.
- **Create Shape – Round:** Profile (edge angle); Limit (height cap); **Preserve Internal Corners**; Scale (to Height); Base Height; Fade; Tilt; **Total Height** (editable; updates Scale and Base).
- **Create Shape – Angled:** the same fields as Round.
- **Create Shape – Smooth:** Smoothing (concave↔convex), Limit, Preserve Internal Corners, **Blend To Inner Vectors**, Scale, Base Height, Fade, Tilt, Total Height.
- **Create Shape – Custom:** Limit, Preserve Internal Corners, Blend To Inner Vectors, Scale, Base Height, Fade, Tilt, Total Height (a user-defined profile).
- **Create Zero Plane / Add Zero Plane:** a job-size component with zero height (no options).

### Sweeps and lathe-style tools
- **Two Rail Sweep:**
  - Rails: Use Selection; Reverse Rail; **Swap Rail Order**; Make / Insert Start Point; Clear Rails.
  - Cross sections:
    - Attach to the rails (proportional positioning by default). **Add To All Rail Nodes** requires equal node counts on both rails.
    - Edit: drag to reposition; replace; Delete / Remove All.
    - Per-section **Smooth** toggle.
  - Shape: **Scale cross sections with width**; **Sweep between spans**; **Fill center of inner closed vector rails**; Scale to Exact Height.
- **Extrude and Weave:**
  - Rails: Use Selection, Reverse Rail, Make/Insert Start Point, Clear Rails.
  - Source: **Use Vector Cross Sections** or **Use a Component**.
  - Cross sections: start and end profiles are created automatically; Add To All Rail Nodes; Delete; Smooth.
  - Sweep options: **Create sharp corners**, **Sweep between spans**.
  - Component options: Stretch to drive rail length / **Fill with multiple copies** (Overlap %, **Flip alternate copies**).
  - **Weave:** Z Under %, Z Over %, **Scale Shape** or **Add Base**.
  - Scale to Exact Height; Reselect Rails.
- **Turn and Spin:** **Turn** (revolves the profile about its start–end line); **Spin** (revolves about its left end into a circular component); Scale to Exact Height.
- **Moulding** as a separate *modeling* tool: `[NOT FOUND]` in 12.5. Moulding exists only as a toolpath (§5); modelled mouldings use Extrude & Weave or Two Rail Sweep.

### Texture and sculpting
- **Create Texture Area** (Create Textured Area Component): tiles a seed component inside a vector boundary.
  - Modes: Transform Object / Edit Textured Area Component.
  - Spacing X/Y % (negative values overlap); X/Y Shift % (alternate rows or columns); **Reflection** (four tile buttons: none/H/V/both).
- **Sculpting** (Sculpt Visible Model):
  - **Tools:** Smooth (1), Smudge (2), Deposit (3), Remove (4), Undo Brush (5, Shift = erase); **Custom Brush** (a model or bitmap; Load Brush, Angle, **Dynamic Rotation**, Merge).
  - Constrain with V/H.
  - **Sliders:** Diameter, Strength, Smoothness.
  - **Preserve Transparency** (0 key); Show Visible Model (F).
  - **Mode:** Normal / Raise / Lower.
  - **Overlay image:** opacity; show/hide with O.
  - **Presets:** Add/Remove.
  - **Keep / Discard / OK / Cancel.**
  - Graphics-tablet support is an Options setting.
- **Smooth Components:** Smoothing % slider; Preserve Transparency; **Bake Current Smoothing**.

### Modify and derive
- **Emboss Component:** Scale Height; Detail Smoothness (flattens a relief while keeping its detail).
- **Distort Selected Component:** see Distort Object in §3.
- **Fade** and **Tilt:** dynamic properties found in Component Properties and in every Create Shape form.
- **Slice Model:**
  - Slicing: Slice Thickness; From Top; Number of Slices; Create Boundary Vectors.
  - Slice height editing: a height bar and list (Apply, Delete, double-click to add); in 3D, double-click or Shift+wheel.
  - Output: one component per slice.
- **Baking Components:** combines the selection into one component. Ctrl+click keeps the originals.
- **Create Component From Visible Model:** copies the composite model into one component on the active level.
- **Create Component from Bitmap:** from a selected bitmap or from a file (keeps 16-bit depth). The result is auto-scaled and set to Add.
- **Create Component from Toolpath Preview** (Model menu).
- **Scale Model Height:** Scale Height %; Set Exact Height…; **Scale Both Sides** (two-sided jobs).
- **Offset Model:** Offset distance (±); Preview (repeatable); **Clip Z values to the Zero Plane**.
- **Add Draft to Model:** minimum draft angle (slider or value). Bakes the visible components into a new drafted component.
- **Replace Below:**
  - Replace Value Below.
  - Replace Value With: Transparent / Plane.
  - **Lower Component by Value.**
  - Height is measured relative to the selection, which can be components or the active level; a draggable pin plane in 3D sets it.
- **Clear Component Inside / Outside Selected Vectors** (Model ► Reset Inside/Outside).
- **Split Component:** split with an open or closed vector; results are named "– A" and "– B".
- **Create Vector Boundary From Selected Components:** silhouette vectors (grouped when there are several).
- **Create Cross Section Vector** (3D Cross-Section Tool [V12.0]): click two points on the model to get a profile vector.
- **3D Segmenting** (from the 3D import dialog):
  - **Edit** mode: segmenting plane (handles or fields), Segment button, segment tree (delete children, drag to merge).
  - **Layout** mode: orientation arrows; O = free rotation handle; **Visualize undercuts**.
  - Import options: **Create Backface Caps**, **Resize Material To Fit Slices**, **Create boundary vectors**.
  - Re-import returns to the last segmenting state.

### 3D import
- **Formats:**
  - Placed directly: CRV3D (merged to one component), **3DCLIP** (keeps its structure), **V3M**.
  - Through the Orientate 3D Model dialog: **STL, DXF (3D mesh), 3DS, OBJ, SKP**.
  - The Sketch Carving page lists further formats accepted by that tool (3DM, RLF, ASC, PRJ, X, LWO, WRL).
- **Import 3D Model into Single or Two Sided Job:**
  - Initial Orientation: 6 top-face choices × 5 rotations about Z.
  - Interactive rotation: X/Y/Z rings and a free-rotation sphere.
  - Model Size: **Lock XYZ ratio**, Apply; Units; Scale mm/inches (×/÷25.4); **Center Model**; **Apply Perspective along Z**.
  - Buttons: Position & Import, Segment, Cancel.
- **Positioning an Imported Model:** **Zero Plane Position In Model** (slider; Middle/Bottom buttons); **Create both sides** (two-sided jobs); **Discard data below zero plane**; **Highlight Undercuts**; Back / Import.
- **Importing a 3D Model into Rotary job:**
  - Imported model type: **Full 3D model** / **Flat model**.
  - Orientation and interactive rotation, with a black cube that moves the cylinder axis.
  - **Rotation Axis Movement** (On/Off).
  - Scaling: Model units, Lock ratio, Scale model to fit material, Reset to Original Size, Resize material block.
  - 12.5 adds an orientation gizmo.
- **Re-import:** right-click the component ► Reimport.

### 3D export
- **Export as STL:**
  - Limit Triangles By: Approximate tolerance / Maximum number of triangles.
  - Back face: Leave Open; Close with inverted front; Close with a flat plane; Close with other side (two-sided jobs); **Close sides** (rotary end caps).
  - **Triangulate** reports Num Tri Created and Maximum Error, with **Draw shaded triangulation**.
  - **Save Triangulation…** as STL, **Wavefront (.obj)** or **POV-Ray (.pov)**.
- **Export as Grayscale Bitmap**: the composite model as a heightmap.
- **Export as 3D Clipart** (.3dclip) for a model, level or component.

### Draft
- See **Add Draft to Model** under Modify and derive.

---

## 5. Toolpaths, tooling, machines, output

### Fields shared by most toolpath forms
- **Start Depth (D)** and **Cut Depth (C)**.
- **Tool:** Select (from the Tool Database) and Edit (per-toolpath override; the database is unchanged). A tooltip shows the tool's source in the database.
- **Pass Depth Control:**
  - Default spacing may vary the step by up to 15%; **Maintain Exact Step Depth** keeps it fixed.
  - **Edit Passes…** opens a pass list and diagram: add, edit, delete, Clear All.
  - **Set Last Pass Thickness**; **Set Associated Passes**; Number of passes.
- **Safe Z** and **Home Position** (both set in Material Setup).
- **Project toolpath onto 3D Model** (only when a model exists; depth is capped at the material bottom).
- **Vector Selection** (Vector Selector; used by templates).
- **Name.**
- **Use Vector Selection Order** (otherwise the order is optimised).
- Multi-tool lists: **Select / Remove / Edit / Up/Down** buttons. Tools run in list order and act as rest/clearance tools.
- **12.5 cross-toolpath features:**
  - Tool-inclusive naming for multi-tool operations.
  - **Visual Toolpath Status** (marks toolpaths that need recalculation).
  - Sheet-specific toolpath groups.
  - Transfer toolpaths between sheets and sides (right-click).

### 2D toolpaths
- **2D Profile Toolpath:**
  - **Machine Vectors:** Outside / Inside / On (open vectors add Left/Right). Direction: Climb / Conventional.
  - **Allowance Offset** (±). **Do Separate Last Pass:** Last pass allowance, Reverse direction.
  - **Use Vector Start Point.**
  - **Tabs:** Add tabs (Length, Thickness); **Create 3D Tabs**; Edit Tabs. Tab placement is covered under Toolpath Tabs below.
  - **Ramp:** Smooth (distance or angle; **Ramp on Lead In**) / Zig Zag (distance, or angle+distance) / Spiral.
  - **Leads:** Straight Line Lead (angle, length) / Circular Lead (radius, angle 0.1–90°); lead-out; **Overcut distance**.
  - **Order:** Vector Selection Order / Left to Right / Bottom to Top / **Grid**. When several are ticked, the quickest is used; inner vectors are always cut first.
  - **Start At:** Keep Current Start Points / **Optimize Start Points** / **Closest on Bounding Box**.
  - **Corners:** **Sharp External Corner**; **Sharp Internal Corner (3D)** (a V-bit plunges into inside corners).
  - Cutter Compensation option (Advanced Machining Module).
- **Toolpath Tabs:**
  - Automatic: **Constant number** / **Constant distance between tabs**; **Tab at machining start point**; **Avoid corners and curved regions**.
  - Interactive: click to add or delete, drag to move; Delete All Tabs.
- **Pocketing Toolpath:**
  - Multi-tool list (clearance and rest tools).
  - **Strategy:** **Offset** (Climb/Conventional) or **Raster** (**Raster Angle** 0–90°).
  - **Profile Pass:** First / Last / No Profile Pass.
  - **Pocket Allowance**; **Ramp Plunge Moves**.
  - 12.5: optimized linking reduces retracts.
- **Drilling Toolpath:**
  - Drills the centres of closed vectors.
  - **Use Peck Drilling** (Pass Depth from the tool): **Retract above cutting start** (Retract Gap R) or **Retract above the height of the previous pass**.
  - **Dwell at the bottom of each drill pass** (Dwell Time; the post-processor must support it).
  - 12.5 visualises peck drilling in the preview.
- **Quick Engraving Toolpath:**
  - **Depth to engrave / mark** or **Pressure** (diamond drag).
  - Strategy: **Outline** / **Fill** (three fill patterns) with Stepover.
  - **Use Nose Cone**; Number of Passes.
  - **Immediate Output:** Post Processor, Add side to toolpath name, **Output direct to machine**.
- **Chamfer Toolpath:**
  - Angle (A) (half the V-bit angle, or editable for round-nosed tools); Width (W); Cut Depth (C) (W and C are linked); Max Cut Depth (read-only).
  - **Overcut**; **Chamfer Type** (inside/outside, slope up/down); Cut Direction.
- **Thread Milling Toolpath:**
  - Depths: Start Depth; Max Depth (M); Thread Length (L).
  - Tools: **Single Point** or **Multi Point** thread mill (S size, H height, D diameter, A angle, O offset, plus threaded length for multi-point).
  - Thread: **Pitch Preset** (ISO Metric / Universal); Pitch; Diameter; **Fit Tolerance**; **Create Circles for Internal Threads**.
  - Thread Type: Internal / External. Thread Direction: right / left. Cut Direction: down / up.
- **Inlay Toolpath** (straight inlays):
  - Male inlay: **Straight**, or **Stepped** (Step Depth, Step Width).
  - Female inlay: **Pocket** or **Hole**.
  - Allowance Offset / Pocket Allowance.
  - Sub-form **Female Inlay – Pocket**: the Pocket form with automatic corner radiusing.
- **Drill Bank Toolpath** (Advanced Machining Module):
  - Bank layout: Horizontal Drills; Vertical Drills; **Central Drill** (X,Y); **Pitch**.
  - Per drill: Drill Number, Diameter, Reference Drill.
  - Pattern matching: **Pitch Tolerance**, **Diameter Tolerance**.
  - Peck drilling.

### V-carving family
- **V-Carve Toolpath** (V-Carve / Engraving):
  - Start Depth; **Flat Depth (F)**; V-bit or ball-nose tool.
  - **Use Clearance Tools** (multi-tool list): **Clearance Strategy** (Offset or Raster with Raster Angle), Cutting Direction per tool, **Ramp Plunge Moves**.
  - **Corner Sharpen**; **Use Vector Start Points**; Use Vector Selection Order.
- **Advanced V-Carve** (fluted-end or flat-bottom variant): `[NOT FOUND]` as a separate page in 12.5. The flat-bottom behaviour is the **Flat Depth** option above.
- **VCarve Inlay Toolpath** [V12.0 new]:
  - Depths: **Pocket Depth**; **Glue Gap**; **Surface Clearance**.
  - V-bit; Use Clearance Tools.
  - **Plug Destination Sheet** (the plug is mirrored, and centred when on another sheet).
  - **Plug Outer Boundary:** Sheet Limits / Vector Offset / **Boundary Offset**.
  - Sub-pages "VCarve Inlay Toolpath – Pocket" and "– Plug" exist (fields not fetched).
- **Prism Carving Toolpath:** Prism Depth; **Set Depth for Full Prism**; Cut Direction.
- **Photo V-Carve Toolpath:**
  - Source: a selected bitmap.
  - Depths: Max Carving Depth; Stepover Retract.
  - Strategy: **Raster** / **Hatch** / **Selected Vectors**, with Line Spacing % and Line Angle.
- **Sketch Carving** [V12.0 new]:
  - Source **Selection:** Bitmap or 3D Model (many formats).
  - Start Depth; Flat Depth. Tools: ball nose, tapered ball nose, V-bit or engraving tools only.
  - **Machine Limit Boundary:** Bitmap/Model Boundary / Selected Vector / Selected Level, with Boundary Offset.
  - **Line Thickness** (0–100).
  - 12.5 previews sketch carving in 3D.
- **Fluting Toolpath:**
  - Flute Depth.
  - **Flute Type:** Ramp over complete length / Ramp at Start / Ramp at Start and End.
  - **Ramp Length** or **Ramp %**; **Ramp Type:** Linear / Smooth.
  - The ramp is placed at the vector's start point.

### 3D toolpaths
- **3D Rough Toolpath:**
  - **Machining Limit Boundary:** Model Boundary / Material Boundary / Selected Vector(s) / Selected Level. **Boundary Offset**.
  - **Machining Allowance.**
  - **Strategy:**
    - **Z Level:** Profile First / Last / None; Order **Level by level** / **Depth-first**.
    - **3D Raster.**
  - Raster Direction (Along X / Y); **Reverse Step Direction** (Z Level only); **Avoid Machined Areas**; Ramp Plunge Moves.
- **3D Finish Toolpath:**
  - Multi-tool list (rest machining).
  - Machining Limit Boundary and Boundary Offset (as in roughing).
  - **Area Machining Strategy:** **Offset** (with **Stepover Retract**) or **Raster** (Raster Angle).
  - **Rest Machining:** **Minimum Detail**, rest Boundary Offset.
  - Spiral, radial and other finishing strategies: `[NOT FOUND]` in 12.5 docs.
- **Moulding Toolpath:**
  - Toolpath Position (Z taken from the cross-section height).
  - Drive Rail(s) (Reverse, Clear); Profile (open vector, Ctrl+click; Reverse Profile).
  - **Finishing:** Stepover; **Vary Stepover**; **Skip Flat Regions**; Create Sharp Corners.
  - **Roughing:** Use Larger Area Clearance Tool (creates a "[Clear]" toolpath); **Machine Flat Regions**; Ramp Plunge Moves; Machining Allowance.
  - **Boundary Offset** / **Use automatic boundary offset**.
- **Texture Toolpath:**
  - **Use selected vectors as pattern** (with **Ramp at start and end**).
  - Scallops: Max. Cut Depth; Min. Depth %; Max. Cut Length; Min Length %; **Max. Overlap %**; **Variation** %.
  - Stepover; Angle; Boundary Vector Offset.
- **Profile with 3D** (as a named toolpath): `[NOT FOUND]` in 12.5. The equivalent is the **Project toolpath onto 3D Model** option on 2D toolpath forms. Vector boundaries for 3D toolpaths come from Create Vector Boundary From Selected Components.
- **Imported 3D toolpaths:** PhotoVCarve, Cut3D or Machinist toolpaths can be positioned, duplicated or mirrored, and their Start Depth, feeds and speeds edited.

### Toolpath management
- **Toolpath Tree:**
  - Display: visibility checkbox; tool-type icon.
  - Editing: double-click to edit; right-click menu.
  - Sheets: **Sheets Filter** (All Sheets shows inactive sheets faded).
  - Ordering and groups: Up/Down order (sets preview and output order); groups (Create Empty Group, Group Visible, Ungroup, drag in or out; groups cannot span sheets).
  - Copy To / Move To ► Other Side / Sheet.
  - Recalculate ► This / Visible / All; Delete ► This / All Invisible / All Visible / All; Show/Hide submenus (incl. **Show All With This Tool**); Activate Sheet; Edit / Rename / Duplicate.
  - F12 toggles the Toolpath tab.
- Other management pages: **Edit Toolpath**, **Duplicate Toolpath**, **Delete Toolpath**, **Recalculate All Toolpaths**.
- **Toolpath Templates:**
  - **Save Selected Toolpath as Template** / **Save All Visible Toolpaths as Template**.
  - **Load Toolpath Template:** Missing Layers dialog (create missing layers automatically / Delete all toolpaths associated with missing layers / load as is). Ctrl+Open applies to all sheets, including empty layers. Templates can be applied to every sheet.
  - Templates use Vector Selector associations to pick their vectors.
- **Merged Toolpath** (Merge Visible Toolpaths): merges visible same-tool toolpaths; ordering checkboxes (quickest is chosen); **Merge By Part**; feeds and speeds adjusted per segment.
- **Array Copy Toolpath:** Rows (Y) / Columns (X); **Offset** or **Gap** (X/Y); **Minimize tool changes**. Source toolpaths stay nested and editable under the array item.
- **Toolpath Tiling Manager:**
  - Strategy: **Individual Tiles** (width, height, overlap) / **Feed-through in X** / **Feed-through in Y**.
  - **Machine smallest tile first**; Update Tiles; Draw toolpaths in original position.
  - Tile Toolpaths checkbox, which links to **Output Tiled Toolpaths** in Save Toolpaths.
- **Keep-out Zones** [V12.0 new]: Clearance; Clear Zones; **Create from Selection** (closed vectors; zones are not linked to the source vectors). Rapid moves are routed around the zones (post blocks `*_STRAIGHT_RAPID_MOVE`).

### Preview and simulation (Preview Toolpaths)
- **Material Appearance:** Use Solid Color / Use Material (wood, metal, stone, plastic); custom textures (images in the Textures folder, about 1000×1000 recommended).
- **Machined Area Color:** Material Color / **Surface Color** (12.5: separate surface colour, e.g. painted or laminated stock) / Global Fill Color / **Toolpath Color** (per toolpath, or No Fill).
- **Lithophane** simulation (back-lit) with a brightness slider.
- **Animation:** Animate preview; Draw tool.
- **Preview controls:** Preview Toolpath; **Preview Control Simulation** (Run / Single Step / Run to Retract); Preview All Sides; Preview All Toolpaths; Preview Visible Toolpaths; Reset Preview; Save Preview Image.
- **Quality levels:** Standard → Maximum (Toolpaths menu).
- Laser simulation marks (chars) the surface using each tool's **Maximum Burn Rate**.
- **Create Component from Toolpath Preview** (Model menu).

### Time estimation (Estimating Machining Times / Toolpath Summary)
- Per-toolpath and total times (H:M:S).
- Inputs: **Rapid Rate**; **Scale Factor**; **Machine Acceleration**, using the selected machine's XYZ and angular acceleration (12.5: acceleration-aware estimation).
- **Notes** field.

### Create Job Sheet
- An HTML sheet per sheet and per side (`_Top` / `_Bottom`). Ctrl+click opens it automatically.
- Contents: Job Layout thumbnail, Material Setup, Toolpaths Summary, Toolpaths List (feed, plunge, spindle).
- Customised with the **Job Setup Sheet Editor** gadget (§6).

### Tool Database
- **Tool types:** V-Bit, Engraving, Tapered Ball Nose, Ball Nose, End Mill, Radiused End Mill, Form Tool (built-in Round-over and Ogee, plus **custom form cutters** drawn as the right-hand half-profile), Diamond Drag, Drill. Laser tools appear in the Laser Module forms. Thread mills (Single/Multi Point) appear in the Thread Milling form.
- **Tool definition:** Name, Tool Type, Notes, Diameter, Number of Flutes (plus type-specific geometry such as angle).
- **Cutting data** (stored per machine × material): Pass Depth, Stepover, **Final Pass Stepover**, **Clearance Pass Stepover** (V-bit), Spindle Speed, Feed Rate, Plunge Rate, **Chip Load** (calculated), **Maximum Burn Rate** (laser), **Tool Number** (per machine only).
- **Tree actions:** New Tool, Copy Tool, Delete Tool, New Tool Group, Export Tools, Import Tools (Import, or Merge with or without overwrite).
- **Material Management:** Add / Duplicate (Copy Cutting Data) / Delete / Name.
- **Machine Management:** see Machine configuration below.
- **Online Tool Database** (V&Co account): Login, Download, Upload backup. **Cloud Version Selector** chooses a backup by product and version.
- **Remote Tool Databases:** subscribe by URL (read-only); copy tools into the local database; switch databases from a drop-down.
- **Naming:** **Tool Database – Name Format** and **Custom Naming Variables** pages exist (fields not fetched).

### Machine configuration and post-processors
- **Machine Configuration Management:**
  - Machine List; Add Machine; **Install Offline Package**; **Search Machine Online**; Duplicate Machine (Copy Cutting Data); Delete Machine (optionally deleting its cutting data); Name.
  - **Acceleration (XYZ)** (per axis, several unit choices) and **Acceleration (Angle)** (deg/s² or rad/s²). Acceleration is used only for time estimates.
  - **Associated Post-Processors:** add or remove; **Post-Processor Version** ("Latest (Vn)" or a pinned Vn); right-click View / **Make Default**.
  - Other machine parameters (bed size, ATC, laser flags): `[PARTIAL]`. The page says optional parameters exist but does not list them.
- **Search for Machine Online:** Manufacturer → Series → Model → Configuration (rotary axis on X or Y). Downloads the post-processor, machine layout and starter feeds and speeds.
- **Kickstarter Wizard** (first run): Signing in → Registering with V&Co → Looking for your machine online → Manual configuration.
- **Post-Processor Management:**
  - Per post: **View** (Post-Processor Content), **Changelog**, **Customize** (creates an editable copy in `My_PostP`).
  - Custom posts: Open File Location, Delete.
  - **Install Post-Processor**; **Update Post-Processor List** (needs a V&Co login).
- **Post-processor file format** (`.pp` text; "15. Advanced – Post-Processor Editing"):
  - **Global settings:** POST_NAME, FILE_EXTENSION, UNITS, PRINT_DIRECT, DIRECT_OUTPUT, LINE_ENDING, RAPID_PLUNGE_TO_STARTZ, RAPID_SPEED, MAXIMUM_LENGTH, MAXIMUM_ROTATION, MIN/MAX_ARC_RADIUS, **ROTARY_WRAP_X / ROTARY_WRAP_Y**, INVERSE_TIME_MODE, SPINDLE_SPEED_RANGE, **LASER_SUPPORT**, SUBSTITUTE, TAPE_SPLITTING, FIRST_TOOLPATH_NEW_SEGMENT, LINE_NUMBER_START/INCREMENT/MAXIMUM.
  - POST_BASE inheritance is no longer supported; a migration page exists.
  - **Blocks** (`begin NAME`):
    - Program flow: HEADER, FOOTER, TOOLCHANGE, NEW_SEGMENT, SPINDLE_ON.
    - Rapids: INITIAL_RAPID_MOVE, RAPID_MOVE.
    - Feed moves: FIRST_FEED_MOVE, FEED_MOVE, FEED_RATE_CHANGE.
    - Arcs: [FIRST_]CW/CCW_ARC_MOVE, and the helical forms (FIRST_*_HELICAL_ARC_PLUNGE_MOVE, FIRST_*_HELICAL_ARC_MOVE, *_HELICAL_ARC_MOVE).
    - Plunge, retract, dwell: FIRST_PLUNGE_MOVE, PLUNGE_MOVE, RETRACT_MOVE, DWELL_MOVE.
    - Keep-out-zone rapids: INITIAL/FIRST/STRAIGHT_RAPID_MOVE.
    - Laser and jet tools: JET_TOOL_POWER/ON/OFF.
    - Drill banks: SETUP_DRILL_BANK, DRILL_BANK_PATTERN, DRILL_BANK_EXTEND_DRILL, DRILL_BANK_RETRACT_DRILL, FINAL_DRILL_BANK_ACTION.
    - Cutter compensation: CUTTER_COMP_LEFT/RIGHT/OFF.
  - **Variables:** `VAR NAME = [label|A/C|prefix|format|multiplier]`.
    - Position and arcs: [X] [Y] [Z] [A] [I] [J] [IA] [JA] [ArcStartX/Y] [ArcMidX/Y] [Radius] [Angle].
    - Feeds and speeds: [F] [FC] [FP] [S].
    - Tools: [T] [TP] [TOOLNAME] [TOOL_NOTES] [TDIA].
    - Line, time and file: [N] [TIME] [DATE] [TP_FILENAME] [TP_DIR] [TP_EXT] [PATHNAME].
    - Home and material: [XH] [YH] [ZH] [SAFEZ] [XMIN] [XMAX] [XLENGTH].
    - Other: [DWELL] [FI] [PRODUCT] [FILE_NOTES] [TOOLS_USED].
    - Drill banks: [DB] [DBX] [DBY] [DL] [CD]. Cutter compensation: [CUTTER_COMP].
  - The fetched part of the page (about 100k of 104k characters) does not mention Lua in post-processors.
  - **Adapting a post-processor for Lasers:** power variable, new laser blocks, marking a post laser-capable.

### Saving toolpaths (Save Toolpaths)
- **Machine** drop-down (filters the post list); **Postprocessor** drop-down (with Add Post-Processors).
- **Scope:** Selected Toolpath / **Visible Toolpaths To One File** (same tool or an ATC; the ATC check verifies distinct tool numbers) / **Visible Toolpaths To Multiple Files** (with **Group where possible**).
- **Output direct to machine** (when the post supports it, e.g. VTransfer); **Output Tiled Toolpaths**; Add side to toolpath name.
- **VTransfer** (helper app that streams toolpaths to Grbl controllers):
  - Settings: Controller; Connect On.
  - Jog tab: Homing, Unlock, Z Touch Plate, jog buttons, X Y / Z Origin Here, Reset Default Origin.
  - Cut tab: **Cut Now!**, Estimated Time Remaining, Feedrate Override, Abort.

### Rotary, wrapped and double-sided machining
- **Rotary (wrapped):**
  - Job Setup Rotary: Length, Diameter, Along X/Y, Cylinder Axis zero.
  - Material Setup Rotary; **Auto-Wrapping View** (wrapped preview and model); Wrapping level effect; Vector Unwrapper; rotary 3D import.
  - The rotary gadgets: Rounding Toolpath, Wrapped Fluting Layout, Wrapped Spiral Layout.
  - Toolpaths are computed flat; a wrapping post-processor (ROTARY_WRAP_X/Y) converts X or Y into A.
  - The docs do not cover indexed vs. continuous wrapping, or dedicated rotary roughing/finishing strategies. `[NOT FOUND]`
- **Double-sided:**
  - Setup: Job Setup Double (Flip Direction, Zero off same side); Swap Sides; keys 1/2/= ; Multi Sided View.
  - Copying to the other side: Copy to ► Other Side for vectors; Copy To / Move To ► Other Side for toolpaths.
  - Preview All Sides; Nest two-sided parts.
  - 3D: Create both sides on 3D import; Scale Both Sides; STL "Close with other side".
  - Job sheets per side.

### Modules (paid add-ons, enabled with Help ► Enter License Code)
- **Advanced Machining Module:**
  - **Cabinet Import** (CabinetParts Pro, CabinetSense, KCD, Mozaik, or a Custom Mapping File; auto nest and toolpath).
  - **Cutter Compensation** (Profile toolpaths; G41/G42-style output through the CUTTER_COMP blocks).
  - **Drill Bank Toolpath.**
- **Laser Module:**
  - **Laser Cut – Fill:**
    - Laser settings: Power; Move Speed; Cut Depth → No. Passes; **No. Repeats**; **Overscan**.
    - Strategy: Cut Outside/Right, Cut Inside/Left (both kerf-compensated), Cut On, **Hatch Fill** (Stepover, Hatch Angle, **Cross Hatch**, Fill Together / Fill Regionally, **Outline**).
    - Allowance; Immediate Output.
  - **Laser Picture:** Power, Move Speed, No. Passes, Overscan; Raster / Hatch / Selected; **Dither Image**; Line Spacing; Line Angle.
  - **Laser Sketch Engraving** (12.5): Bitmap or 3D Model source; Power; Overscan; Machine Limit Boundary; Line Thickness; Stepover; Hatch Angle; Fill Together / Fill Regionally / **Fill regionally between vectors**; Cross Hatch.
  - Laser simulation through Maximum Burn Rate.

---

## 6. Gadgets, clipart, cloud, printing, preferences

- **Gadgets:**
  - Add-ons scripted in **Lua**. The SDK and tutorials are at gadgets.vectric.com/developerinfo.html; the API itself is not documented in the help.
  - Installed to `Public Documents/Vectric/Aspire/Gadgets` with Install New Gadget…; the menu is built at startup.
  - **Gadget Shortcuts** (Ctrl+F-key).
  - Lua API reference: `[NOT FOUND]` in the Reference Manual (external SDK only).
- **Preinstalled gadgets:**
  - **Celtic Weave Creator:** grid size; click intersections to cycle Cut Vertical / Cut Horizontal / Cut Neither; cross section for Extrude; Apply (creates vectors for Extrude & Weave).
  - **Drag Knife Toolpath:** Blade Offset; Tolerance angle; Cut Depth; Swivel Depth; Passes; Use Vector Start Points; Feed/Plunge.
  - **Keyhole Toolpath:** slot direction, depth and length; optional preview vectors (entry-hole diameter, slot diameter, layer); tool; name.
  - **Job Setup Sheet Editor:** logo, title, header and footer colours, include/exclude sections (Job Layout, Job Notes, Material Setup, Toolpaths Summary, Individual Toolpath Summaries), restore default.
  - **Wrapping submenu:**
    - **Create Rounding Toolpath:** blank square or round; method Radial / Raster / Optimized Raster; Allowance; tool; name.
    - **Wrapped Fluting Layout:** number of flutes; start and end distances; coves.
    - **Wrapped Spiral Layout:** number of strands; start and end distances; Spiral Pitch or spacing between strands; twist direction; coves.
- **Clipart tab:**
  - Library Browser: Add Folder / Remove Folder. Local Files browser shows V3M and CRV thumbnails, in three thumbnail sizes.
  - Placing clipart: double-click to place at the job centre; **Import to Level**; drag into the 2D or 3D view; drag V3M files in from Explorer.
  - **Cloud icon** shows online clipart not yet downloaded; **Refresh** logs in to the Customer Portal.
  - Right-click: Import to Level, Open Containing Folder, Download.
  - Search, Design & Make store and V&Co marketplace integration: `[NOT FOUND]` in 12.5 docs.
- **Vectric account (V&Co) and cloud:**
  - **Signing In** (top-right button, License dialog, Tool Database, Clipart tab) enables tool-database backup and restore, licence updates and clipart access.
  - **License Dialog:** Online (V&Co login) or Manual (Load from File `.vlicence`, Enter License Data, Enter User Name & License Code); **Add Module**.
  - V&Co also supplies machine packages and the post-processor list (§5).
  - Other pages: Migration Dialog (copies settings from an older version), Crash Handling. There is no general "Vectric Cloud" file-storage feature. `[NOT FOUND]`
- **Printing:** File ► Print / Print Preview / Print Setup; Options ► Print 3D View Shaded Background. `[PARTIAL]`
- **Options and preferences:** see §1.
- **Application Data Folder** (File menu): textures, gadgets, post-processors (`My_PostP`), the tool database, default toolpath settings, vector textures.

---

## 7. New in V12.0 and V12.5

### V12.0 [V12.0 What's New]
- **Interface:**
  - Improved UI and new theme (DPI scaling; vector and component tools merged into one panel).
  - Components drop-down; Sheet drop-down.
  - **Orthographic View Mode**; **View Control** cube; **View Toolbar**.
  - Unified import.
- **Toolpaths:** **Sketch Carving**; **VCarve Inlay Toolpath** (plug and pocket, mirroring, sheet placement); **Keep-out Zones**.
- **3D design:** **Create Angled Shape**, **Create Rounded Shape**, **Create Smooth Shape**, **Create Flat Plane**. They replace the old shape creator and keep their parameters for later editing.
- **Design:** **3D Cross-Section Tool**; **Drawing in the 3D View**; **Vectors in the 3D View** toggle.

### V12.5 (V12.5 What's New)
- **Toolpaths:**
  - **3D Sketch Carving Preview**; **Preview Surface Colour**.
  - **Optimized Pocketing for Fewer Retracts**; **Acceleration-Aware Time Estimation**.
  - **Transfer toolpaths between sheets** (right-click); **Sheet Specific Toolpaths**.
  - **Laser Sketch Engraving**; **Visual Toolpath Status**; **Tool inclusive naming**; **Peck Drilling Visualization**.
- **Design:**
  - **Vector editing in the 3D View**; **Text creation in the 3D View**; **Fillet tool in 3D view**; **Trim tool in 3D view**.
  - **Edit Dimension objects** (control points, associative); **Smart part selection** (Select by Part); **Quick Node Editing with Shapes**; **Polyline Enhancements** (arrow keys cycle input modes in 3D).
  - **Duplicate Sheet with all contents**; **Flexible Nesting Options** (nest onto a chosen sheet); **Enhanced Plate Production** (one plate per sheet).
  - **WebP Support**; **Color preserving Bitmap Fading**.
- **Interface:** **Snap Grid and Guide lines in 3D view** (colour and thickness); **Re-sizeable Interface Elements** (drop-downs); **Rotary Import Orientation** gizmo.
- **Layers:** **Duplicate Layers**; **Drag and Drop Layer Management**; **Layer Filtering** (hides empty layers).
- **3D Design Tools:** **Quick Component Baking** (from the context menu).
- **vectric.com/upgrade** (marketing page) adds **Surface Color Simulation** (painted or laminated stock) and "realistic material visualization".

---

## Gaps and items without documentation (summary)

**Not found in the 12.5 docs:**
- Advanced V-Carve (as a separate toolpath).
- Profile with 3D (as a named toolpath; use "Project toolpath onto 3D Model").
- Moulding as a modeling tool (only the Moulding toolpath exists).
- Fluting, wave and spiral drawing tools (only gadgets and Vector Texture).
- A vector Chamfer tool; a "Smart Smooth" tool; a separate Intersect tool (it is called Overlap).
- SVG *import* (only SVG export is documented).
- Spiral and radial 3D finishing strategies; indexed rotary or dedicated rotary toolpaths.
- A Lua API reference.
- Clipart search and Design & Make / V&Co store integration; a Vectric Cloud file feature.

**Partial (page exists but fields are not listed):**
- Weld, Subtract and Overlap form fields.
- Offset options beyond those listed.
- Machine Configuration hardware parameters.
- The Quick Engrave fill-pattern names (shown only as images).
- VCarve Inlay Pocket and Plug sub-pages (not fetched).
- Tool DB Name Format and Custom Naming Variables pages (not fetched).
- Print dialog details.
- Create Zero Plane and Cross Section tool (no options documented).
- About 4% at the end of the post-processor editing page was not read.
