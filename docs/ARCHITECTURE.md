# Carvetto architecture

Decisions here are binding for coder agents. Changes go through an ADR in `docs/adr/`.

## Layers

```
app/        Qt 6 desktop application (Windows first, Linux later). Thin: views, forms, commands.
app/viewmodel/  carvetto_viewmodel — Qt-free view logic (view transform, rulers, canvas scene,
            unit formatting, form validation). Headless-tested; the Qt layer is glue (ADR-0014).
core/       carvetto_core — static library, C++20, NO Qt. Everything testable headless.
posts/      Post-processor templates (.pp text format) written for Carvetto.
tools/      Developer scripts (golden-file regeneration, format checks).
```

Dependency direction: `app → viewmodel → core`. Never the reverse. Third-party libs are fetched by CMake `FetchContent` (pinned tags) or found via `find_package` for Qt.

## Core modules (namespace `carvetto::<module>`)

| Module | Owns | Key types |
|---|---|---|
| `geometry` | 2D primitives, spans, contours, bounding boxes, transforms, offsetting, booleans, fillets, trim, curve fitting, snapping maths | `Vec2`, `Span` (Line/Arc/Bezier), `Contour`, `Polyline`, `Transform2`, `Box2` |
| `text` | Font loading (FreeType), glyph outlines → contours, layout, text on curve, single-line fonts | `Font`, `TextBlock`, `TextLayout` |
| `bitmap` | Raster images, trace (potrace), adjustments, crop | `Image`, `TraceOptions` |
| `document` | Job (single/double/rotary), sheets, layers, objects, levels, selection, undo, document variables, native file I/O | `Document`, `Job`, `Sheet`, `Layer`, `Object`, `UndoStack` |
| `relief` | Heightfield components and composition (Phase 2) | `HeightField`, `Component`, `Level`, `CombineMode` |
| `tooling` | Tool database, materials, machines, cutting data | `Tool`, `ToolType`, `CuttingData`, `Machine`, `ToolDatabase` |
| `toolpath` | One sub-namespace per operation + shared machinery (passes, ramps, leads, tabs, ordering, linking) | `Toolpath`, `Move`, `ToolpathOp`, `PassPlan` |
| `simulate` | Material removal preview (heightfield), time estimation | `Stock`, `Simulator`, `TimeEstimate` |
| `post` | `.pp` template parser/interpreter, output writer | `PostProcessor`, `PostContext` |
| `io` | Interchange formats: DXF, SVG, EPS/AI (PostScript subset), PDF (vector), STL/OBJ/3DS, bitmaps | `importDxf()`, `exportSvg()` … |
| `scripting` | Lua host and API (Phase 3) | `ScriptHost` |

## Core conventions

- **Units:** millimetres and radians inside core. The document stores the user's display unit; the UI converts.
- **Precision:** `double` for geometry, `float` for heightfields. Geometric tolerance is explicit (`Tolerance` struct passed in), default 0.001 mm.
- **Vectors are span lists.** A `Contour` is an ordered list of `Span`s (line, circular arc, cubic Bezier) that is open or closed. Polygonisation to `Polyline` happens at a stated tolerance for algorithms that need it (Clipper2 offsets/booleans), and arcs are refitted afterwards where the feature requires it (`Fit Curves`).
- **Toolpaths are move lists** in machine coordinates (mm, absolute, Z up), tool-tip positions, with move kinds: rapid, feed, plunge, retract, arc (CW/CCW with centre; Z change = helix), dwell (ADR-0008). Wrapping for rotary happens in `post`.
- **Parametric objects.** Every created object stores its creation parameters so it can be re-edited (shapes, text, dimensions, toolpaths). Toolpaths store a `VectorSelector` association, not just raw geometry, so recalculation re-resolves vectors.
- **No global state.** No singletons. The `Document` is passed explicitly; services (fonts, tool DB) are injected.
- **Errors:** `std::expected<T, Error>` (or a small equivalent) for recoverable failures; exceptions only for programmer errors.
- **Determinism:** same input → byte-identical toolpath. No wall-clock, no unordered iteration affecting output.

## Native file format (ADR-0003)

`.carv` = zip container: `job.json` (document, layers, objects, parameters, toolpath definitions), `toolpaths/*.bin` (computed moves, optional cache), `components/*.hf` (heightfields, zstd), `bitmaps/*`, `manifest.json` (format version). Human-readable where cheap, binary where big.

## Post-processors (ADR-0004)

Carvetto's `post` module implements the widely used plain-text `.pp` template grammar: global settings (`POST_NAME`, `FILE_EXTENSION`, `UNITS`, …), `VAR NAME = [tag|A/C|prefix|format|multiplier]` definitions, and `begin BLOCK … end` sections (HEADER, FOOTER, TOOLCHANGE, RAPID_MOVE, FEED_MOVE, arcs, plunge/retract/dwell, laser, cutter-comp, drill-bank blocks). Variables documented in `docs/reference/feature-inventory.md` §5. Carvetto ships its own posts under `posts/`; it does not redistribute third-party posts.

## Testing

- Catch2 v3 via FetchContent. `ctest` runs everything headless.
- **Golden files** for toolpaths and post output under `core/tests/golden/`; regenerate only via `tools/regen_golden.py` with a reviewed diff.
- Geometry gets property-style tests (random polygons: offset(+d) then offset(−d) ≈ original, etc.).

## Build matrix (CI)

- Linux (Ubuntu latest, GCC): core + tests. Mandatory.
- Windows (MSVC 2022, Qt 6 via `aqtinstall`): core + tests + app. Mandatory once `app/` has code.
