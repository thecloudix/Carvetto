# Open Source CNC — Project Plan (Aspire-equivalent CAD/CAM)

Status: draft v0.1, 2026-10-24. Based on a walkthrough of Aspire Trial Edition 12.511 on Chris's PC plus the Vectric 12.5 Reference Manual. Companion docs in this project: `aspire-feature-inventory.md` (the full tool-by-tool list) and `aspire-ui-observations.md` (what the installed app shows).

## 1. Decisions so far

| Decision | Choice | Notes |
|---|---|---|
| Target OS | **Windows first**, Linux later | Pick a stack that is portable so Linux is a build target, not a rewrite. |
| First release scope | **2D/2.5D CAM first** (VCarve-level), then 3D relief | Default chosen because no preference was given; see §4. Can be flipped. |
| License | **GPL-3.0** (default, open) | No preference given. Permissive (MIT/Apache) is the alternative; decide before the first public commit. |
| Approach | Clean-room re-implementation | Replicate *functionality*, not Vectric's code, icons, UI artwork, clipart, fonts, or file format internals (see §7). |

## 2. What Aspire is, in one paragraph

A single-window desktop app with a 2D drawing canvas and a 3D view. The user sets up a **job** (material block: single-sided, double-sided or rotary), draws or imports **vectors** on **layers** across one or more **sheets**, optionally builds a 3D **relief** from **components** stacked in **levels** with combine modes, then creates **toolpaths** (2D profile/pocket/drill/V-carve…, 3D rough/finish) using a **tool database**, **previews** material removal, and **saves G-code** through text-template **post-processors** for a configured **machine**. Add-ons are Lua **gadgets**; modules add laser and advanced machining.

## 3. Architecture

### 3.1 Stack (proposed)

- **Language/core:** C++20 for geometry and toolpath kernels (performance matters: V-carving, 3D raster, offsetting on 1M-point reliefs). Alternative: Rust (safer, slower ecosystem for CAD UI). Decide early; it's the one choice that's hard to reverse.
- **UI:** Qt 6 (Widgets or QML) — native on Windows, trivially portable to Linux, mature docking panels, OpenGL/Vulkan views via RHI.
- **Geometry libraries (all permissive/GPL-compatible):**
  - 2D offsetting/booleans: **Clipper2** (polygon offset, union/difference/intersection).
  - Curves: own Bezier/arc span library (Aspire's vectors are span lists of lines, arcs, Beziers) + biarc/arc fitting.
  - Fonts/text: **FreeType** + **HarfBuzz** for outlines; single-line (engraving) fonts need a custom stroke-font format.
  - Image tracing: **potrace** (B/W) + colour quantization for colour mode.
  - Meshes: **libigl**/own code for STL/OBJ/3DS load; **assimp** optional.
  - Relief (heightfield) engine: own code — a 2D float array per component with combine modes is the core of Aspire's 3D.
  - Nesting: **SVGnest-style** NFP nesting or **libnest2d**.
  - Scripting: **Lua 5.4** (gadget compatibility goal) and/or Python.
- **File format:** open, documented, versioned — e.g. a zip container with JSON (job, layers, vectors, toolpath params) + binary blobs (component heightmaps, bitmaps). Do **not** attempt to read/write `.crv3d`.

### 3.2 Module layout

```
core/
  geometry/     spans (line/arc/bezier), contours, offset, boolean, fillet, trim, curve-fit, snapping
  text/         font outlines, text-on-curve, kerning, text-in-box, single-line fonts
  bitmap/       trace, edit picture, crop, fade
  relief/       heightfield component, levels, combine modes, shape creation, sweeps, turn/spin,
                texture area, sculpt brushes, smooth, emboss, slice, offset, draft, replace-below,
                mesh import → heightfield, STL export
  document/     job (single/double/rotary), sheets, layers, levels, document variables, undo stack
  tooling/      tool database (types, cutting data per machine×material), materials, machines
  toolpath/     one module per operation (profile, pocket, drill, vcarve, …, 3d rough/finish)
                shared: pass depths, ramps, leads, tabs, ordering, start points, projection onto 3D,
                keep-out zone routing, merge, array copy, tiling, templates
  simulate/     voxel/heightfield material-removal preview, time estimation (acceleration-aware)
  post/         .pp-compatible post-processor interpreter (blocks + VAR format), save-toolpaths
  scripting/    Lua gadget host + API
app/
  ui-qt/        main window, 2D canvas, 3D view, Design/Sheets/Layers/Components/Clipart/Toolpaths panels,
                forms for every tool, options, job sheet (HTML)
  io/           DXF, EPS, AI, PDF, SVG import/export; bitmaps incl. WebP; STL/OBJ/3DS; SKP (if a lib exists)
tests/          golden toolpath tests, geometry fuzzing, post-processor output comparisons
```

### 3.3 Key design points learned from Aspire

- Everything is **parametric and re-editable**: shapes, text, dimensions, components (Create Shape remembers its settings; toolpaths store their vector association and recalc). Store creation parameters on every object.
- **Vector Selector associations** let toolpaths re-select vectors by layer/type on recalc and make templates work. Build this in from day one.
- Levels/components use **non-destructive effects** (Clipping, Mirror Mode, Fade, Tilt, Base Height) that are composed at render time and made permanent only by **Bake**.
- Toolpaths are computed **flat** even for rotary; wrapping happens in the post (`ROTARY_WRAP_X/Y`). Cheap way to get rotary support.
- Post-processors are plain-text templates (`begin HEADER … end`, `VAR X = [X|A|X|1.4]`). Being **.pp-compatible** means the hundreds of existing community posts just work — a big adoption win.
- Multi-tool toolpaths (pocket clearance, V-carve clearance, 3D finish rest machining) share one "tool list → rest machining" mechanism.

## 4. Roadmap

### Phase 0 — Foundations (weeks 1–6)
- Repo, license, CI (Windows MSVC; Linux GCC kept green), coding standards, Qt skeleton with 2D canvas + 3D view, undo/redo, document model, open file format, DXF/SVG import & export.
- Tool database (all 9 tool types, cutting data per machine×material, import/export), machine config, `.pp` interpreter with golden-file tests against a few public posts (Grbl, Mach3, LinuxCNC).

### Phase 1 — 2D/2.5D CAM MVP (≈ VCarve Desktop parity)
- **Drawing:** circle, ellipse, rectangle (corner types), polygon, star, polyline/Bezier, arc, freehand, text (TrueType + single-line, text-in-box, text on curve, kerning/arch handles), dimensions, measure, trace bitmap, edit/crop bitmap, vector texture.
- **Editing:** node editing, move/scale/rotate/mirror/distort, align & space, weld/subtract/overlap, trim (boundary + interactive), fillets (normal, dog-bone, T-bone), extend, join/close variants, curve fit, offset, vector boundary, array/circular/along-vector copy, nesting, group, vector validator, select-by-part, layers/sheets, snapping & guides, document variables.
- **Toolpaths:** 2D profile (tabs, ramps, leads, allowance, corners, ordering, start points), pocket (offset/raster, multi-tool, profile pass), drilling (peck, dwell), quick engrave, V-carve/engrave (flat depth, clearance tools), fluting, chamfer, prism, inlay (straight), VCarve inlay, photo V-carve, thread milling, texture (2D on flat), keep-out zones, templates, merge, array copy, tiling, double-sided jobs, job sheet HTML.
- **Simulation:** material removal preview (quality levels, material/surface colours, animation), time estimates.
- **Output:** save toolpaths (single/multi-file, ATC check, tiled), direct output hook.

### Phase 2 — 3D relief modeling (Aspire differentiator)
- Heightfield component engine, levels, combine modes, component properties (fade/tilt/base height/appearance), Create Shape (flat/round/angled/smooth/custom), zero plane, two-rail sweep, extrude & weave, turn & spin, textured area, sculpting (5 brushes + custom), smooth, emboss, slice, bake, offset model, draft, replace-below, split, clear inside/outside, vector boundary from components, cross-section vector, create from bitmap, scale Z, distort.
- 3D import (STL/OBJ/3DS/DXF mesh with orientation dialog, zero-plane slider, undercut highlight, segmenting) and export (STL with back-face options, OBJ, grayscale heightmap, clipart format).
- Toolpaths: 3D rough (Z-level, 3D raster), 3D finish (offset/raster, rest machining), moulding toolpath, project-onto-3D for 2D toolpaths, create component from toolpath preview, 3D tabs.

### Phase 3 — Rotary, modules, scripting
- Rotary job type, auto-wrapping view, vector unwrapper, rotary import, rounding/fluting/spiral layout tools, wrap posts.
- Laser: cut/fill, picture, sketch engraving, burn-rate simulation.
- Advanced: cutter compensation (G41/G42 blocks), drill bank, cabinet import mapping, sketch carving.
- Lua gadget host with a documented API; port the stock gadgets (Celtic weave, drag knife, keyhole, job-sheet editor, DXF batch).
- Clipart library browser (local folders; open community clipart later).

### Phase 4 — Polish & Linux
- 3D-view drawing/editing, plate production/data merge, options dialog parity, printing, localization, installer, Linux packages.

## 5. Suggested MVP acceptance test
Open a DXF, add text on a curve, pocket + profile with tabs, V-carve the text with a clearance tool, preview it, post to Grbl and cut it on a real machine. When that works end-to-end the project is usable.

## 6. Open decisions (need Chris's call)
1. **C++ vs Rust** for the core.
2. **Project name** and GitHub org; GPL-3.0 vs permissive confirmed.
3. **Phase 1 vs Phase 2 first** — the default above is 2D first; if the goal is "Aspire specifically", the relief engine could be pulled into Phase 1.
4. **Lua gadget compatibility** as a hard goal (requires matching Vectric's SDK API shapes) or just "a scripting API".
5. Whether to depend on **Qt (LGPL)** or a lighter UI toolkit.

## 7. Legal/IP guardrails
- Reproduce functionality and workflow, never Vectric's source, icons, textures, clipart, help text, or sample files. Write our own UI copy and artwork.
- Avoid the trademarked names ("Aspire", "VCarve", "Vectric") in the project name; "Aspire-compatible" style claims only where factual (e.g. `.pp` post format).
- Don't reverse-engineer `.crv3d`/`.v3m`; import via open interchange formats (DXF, SVG, STL, OBJ).
- Post-processor files from Vectric's distribution are Vectric's; test against community-published posts or write our own.
- Keep a `CONTRIBUTING.md` clean-room statement: contributors must not paste code or assets from Vectric products.

## 8. Known gaps in this inventory
The 12.5 online docs don't cover: the Lua API itself (external SDK), full Machine Configuration hardware fields, Weld/Subtract/Overlap form details, Quick Engrave fill-pattern names, and clipart store integration. These need to be filled in by exercising the trial on the PC when we reach those phases.
