# ADR-0026: SVG import and export (own XML subset parser, limits, security rules)

Date: 2026-11-07 · Status: proposed (T-0212)

## Context
T-0212 adds `importSvg` / `exportSvg` in `core/io`, mirroring the DXF API of ADR-0015: plain
geometry (layers, contours tagged with a layer index, warnings) in millimetres and radians, no
dependence on the document model. SVG files come from the internet and from other tools, so the
reader is an attack surface (entity expansion, deep nesting, huge path data, `<use>` explosions).
No new third-party dependency is allowed without owner approval, and the clean-room policy
excludes copying from other SVG libraries; the SVG 1.1 specification is public.

## Decision

### 1. Own bounded XML subset parser (`core/src/io/svg_xml.*`)
- Not a general XML parser. Supported: elements, attributes (single or double quoted), comments,
  CDATA, processing instructions (skipped), UTF-8 with optional BOM, the five predefined entities
  and numeric character references **in attribute values**. Text content is never stored, so
  entity references in text are neither decoded nor validated (nothing can be expanded).
- **Refused with a clear error:** any `<!` declaration other than a comment or CDATA, i.e.
  DOCTYPE, ENTITY, ELEMENT, ATTLIST (`SvgError::Code::Unsupported`); entity references other than
  the predefined and numeric ones; numeric references to invalid XML characters; duplicate
  attributes; mismatched or unterminated tags; text outside the root; more than one root; a root
  that is not `<svg>`. Hence no entity can be defined, so no entity expansion ("billion laughs",
  external entities) is possible. There is no network or file access anywhere in the importer:
  `<use>` and `<image>` references to anything but an in-document `#id` are skipped with a
  warning.
- **Namespaces** are not resolved. Unprefixed and `svg:`-prefixed element names are SVG; other
  prefixed elements (`inkscape:`, `sodipodi:`, ...) are ignored silently. Attributes are matched
  by their qualified spelling: `inkscape:label`, `inkscape:groupmode`, `xlink:href`, `href`.
- The tree is stored in a flat vector (first-child / next-sibling indices) so destruction and
  traversal never recurse; the parser and the element walk use explicit stacks, so even a
  deep document cannot exhaust the call stack.

### 2. Limits (all in `SvgImportOptions`, violation = `ResourceLimit` with a message)
Input size 32 MiB; element depth 128; elements 1,000,000; attributes per element 256; attribute
value 64 KiB, except `d` and `points` 16 MiB; name length 256 (fixed); spans in the result
1,000,000 (a 15 MB path used about 0.8 GB at 5,000,000); element visits during rendering 2,000,000 (each `<use>` expansion counts, which bounds
exponential reference trees); `<use>` nesting 16; layers 100,000. Path-data syntax errors are not
fatal: as the SVG error rules say, the path is kept up to the error and a warning names it.
Numbers: overflow (`1e999`) is an error at that token, underflow reads as 0, `NaN` / `Infinity` are
not numbers. Coordinates beyond 1e12 mm (after transforms) drop that contour with a warning;
arc radii beyond 1e12 user units become lines with a warning (the endpoint conversion is
scale-invariant, so huge radii cannot overflow). Non-conformal transforms turn arcs into cubic
Beziers (deviation bounded by `tolerance.chordal`, span count bounded by the span budget).

### 3. Import semantics
- Elements: `path` (all commands, absolute and relative; arc endpoint-to-centre conversion per
  the SVG 1.1 implementation notes), `rect` (with `rx`/`ry`), `circle`, `ellipse`, `line`,
  `polyline`, `polygon`, `g`, `a`, nested `svg` (as a group with x/y; its viewport is ignored with a
  warning), `defs`, `symbol` and `use` (target of `#id`, with x/y and `transform`).
  Circular arcs (|u| = |v|, perpendicular, which includes any similarity transform) become exact
  `ArcSpan`s; other elliptical arcs and ellipses become cubic Beziers via the shared
  `detail::appendEllipticalArc` of the DXF importer. Quadratics are raised exactly to cubics.
  A closing `Z` adds a line only when the current point is farther than `tolerance.distance`
  (mm, after transforms) from the subpath start; otherwise the contour is closed as it is, with no
  sliver line, and the ends keep their own coordinates (the first point wins, nothing is moved).
  `<circle>` is imported as the SVG drawing direction, clockwise on screen, i.e. CW in Y-up.
- Y is flipped so the page's bottom-left corner is the origin. Units: unitless and `px` follow
  `SvgImportOptions::dpi` (default 96, the CSS value); `mm`, `cm`, `in`, `pt`, `pc` are exact;
  `%` refers to the viewBox; `em`/`ex` are unsupported (the element is skipped with a warning).
  `viewBox` + `preserveAspectRatio` are honoured for the root.
- Layers: an element with `inkscape:groupmode="layer"`, or a direct child `<g>` of the root, that
  has an `inkscape:label` (preferred) or `id` becomes a layer, named by it; same names merge.
  Everything else lands on layer "0", created on first use. Nested plain groups inherit.
- Hidden: `display:none` (attribute or inline `style`) skips the subtree silently;
  `visibility:hidden` hides drawing until a descendant sets `visible`. Fill and stroke are
  ignored. CSS stylesheets are not interpreted (`<style>` is skipped with a warning).
- Skipped with one warning per element type (counted): text, images, gradients, patterns, clip
  paths, masks, filters, markers, `switch`, `style`, `script`, `foreignObject`, unknown elements;
  `clip-path` / `mask` / `filter` attributes on drawn elements produce an "ignored" warning.

### 4. Export
One `<g inkscape:groupmode="layer" inkscape:label="...">` per layer (duplicate names get `_2`,
...), one `<path>` per contour in millimetres (`M L C A Z`, absolute). Arcs above 270 degrees, including full circles (closed or
open, any start angle and direction), are written as two `A` commands starting at the contour's own
start point and going in its own direction, so direction, start point and the closed flag survive a
round trip (a full circle comes back as two half arcs). Layer names are escaped and anything not
allowed in XML 1.0 or invalid UTF-8 becomes U+FFFD; an empty name is written as "Layer". Dropped
input (contours without spans, zero-length spans, changed names) is reported through the optional
`warnings` out-parameter as counted notes. The page is the geometry's bounding box
plus `margin`, at least 1 mm square, `width`/`height` in mm and `viewBox="0 0 W H"`; Y is
flipped. The root carries `data-origin-mm="x y"` (model position of the page's bottom-left
corner) which `importSvg` honours, so a round trip restores positions even when the geometry
lies at negative coordinates. Output is deterministic (shortest round-trip numbers, fixed order).
Contours are grouped by layer, so a round trip keeps order within a layer but not across layers.

### 5. Smaller diagnostics
UTF-16 input (BOM or `<` with NUL bytes) fails with "UTF-16 input is not supported (convert to
UTF-8)". `<ellipse>` with only one of rx/ry and elements under a non-invertible transform (zero
scale) are skipped with counted warnings. `<use>` width/height on a symbol are ignored.

## Consequences
- Files using a DTD with entities are refused rather than partially read; those are rare for
  geometry interchange and are the main attack vector.
- The importer needs no resolution of XML namespaces; files that bind SVG to a prefix other than
  `svg:` are read as empty drawings with warnings.
- Hostile inputs are bounded by construction (explicit stacks, caps above) and are exercised by
  the fuzz tests, which are also run under ASan/UBSan in verification.
