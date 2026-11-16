# ADR-0110: Parts-list import through a mapping file (generic CSV, rectangular panels, nesting)

Date: 2026-11-16 · Status: proposed (T-0329, for owner review) · Builds on ADR-0007 (commands, one undo step),
ADR-0031 (toolpath session), ADR-0022 (Drilling)

## Context
T-0329 (feature-inventory "Advanced Machining Module / Cabinet Import" at behaviour level) imports a parts list
describing rectangular panels with holes and dados, nests the panels and makes Profile, Drilling and Pocket
toolpaths. The card allows only generic open formats: a plain delimited-text (CSV) parts list and a user-editable
**mapping file** that says which column holds what. No third-party format, product name or schema is used or
implied; the mapping schema below is Carvetto's own. Readings the project owner has not decided are marked
**(reading)**; each is a local change.

Audit of what exists: the feature inventory lists "Nest Parts" (rotate, mirror, parts inside parts, boundary,
copies) and the backlog has a nesting card, but **core has no nesting implementation yet** (no `Nest` code in
`core/` or the viewmodel). T-0329 therefore brings a deliberately small nester for rectangles only
(`io::nestRectangles`, section 4). When a general Nest Parts lands it can replace the call in `buildPanels`; the
panel import does not depend on its internals.

## Decision

### 1. Layers of the feature
- `core/io/parts_import` (pure text in, plain structs out): CSV reader, mapping parser/validator, parts reader,
  rectangle nester. No document, no Qt.
- `core/io/parts_build`: `buildPanels` turns a `PartsList` into document content as one undo step.
- `app/viewmodel/parts_import_form` (Qt-free): files and tools in, `ToolpathSession` calls for the toolpaths,
  everything in ONE undo step (an outer macro around the builder's and the session's own). **The app dialog
  (file pickers, diagnostics list, tool combos, menu entry) is a follow-up card**; the form is what it will drive.

### 2. The mapping file (the schema for owner review)
Plain UTF-8 text, `key = value` per line, `#` starts a comment line, blank lines ignored, keys case-insensitive, a
leading byte-order mark and CRLF tolerated. Unknown keys, repeated keys, malformed lines and bad values are errors
that name the line; missing required keys are errors naming the key. Nothing is guessed.

| key | value | default |
|---|---|---|
| `units` | `mm` or `inch` for the numbers of the parts list | none: units must be explicit (this key or `column.units`) |
| `delimiter` | `auto`, `comma`, `semicolon`, `tab` | `auto` |
| `header` | `yes` / `no` (first record names the columns) | `yes` |
| `column.length`, `column.width` | header name (case-insensitive) or `#N` (1-based) | **required** |
| `column.thickness` | same | required unless `default.thickness_mm` |
| `column.material`, `column.label`, `column.quantity` | same | optional |
| `column.units` | per-row units cell (`mm`, `in`, `inch`, `inches`) overriding `units` for that row | optional |
| `column.holes` | holes cell, below | optional |
| `column.dados` | dados cell, below | optional |
| `default.thickness_mm`, `default.material` | used when the column is absent or the cell empty | none |
| `layer.panels`, `layer.holes`, `layer.dados` | layer names (reused when they exist) | `Panels`, `Holes`, `Dados` |
| `sheet.length_mm`, `sheet.width_mm` | stock sheet size (always millimetres) | 2440 x 1220 |
| `nest.gap_mm`, `nest.margin_mm` | spacing between panels / to the sheet edge (millimetres) | 6 / 10 |
| `nest.rotate` | allow quarter turns | `no` |

Keys that carry no number of the parts list (`default.thickness_mm`, `sheet.*`, `nest.*`) are in millimetres by
their name, so `units` cannot change their meaning. Whole-file rules: `header = no` needs `#N` columns; one column
cannot serve two roles; the three layers need different names; the header names the mapped columns exactly once
(zero or several matches is an error listing the headers). Cell conventions:

- **Part frame**: x along the length, y along the width, origin at the lower-left corner. `length`/`width` are the
  panel's two sides as listed; they are not reordered (grain direction stays with the length).
- **Holes cell**: items separated by `|`, each `x:y:diameter[:depth]` (centre from the origin corner); depth empty
  or 0 or equal to the thickness = through, else a blind hole of that depth (at most the thickness). Holes must lie
  inside the panel.
- **Dados cell**: items separated by `|`, each `L:position:width:depth` (groove runs along the length, occupying
  y = position..position+width) or `W:...` (runs along the width, x = position..position+width); position is the
  groove's NEAR EDGE from the origin edge **(reading)**; always edge to edge; depth strictly between 0 and the
  thickness (a through cut is a cut-out, not a dado: refused).
- **Quantity**: whole number 1..1000, empty = 1. **Label/material**: free text, see section 5.

### 3. Units
Everything in `core` is millimetres. The mapping's `units` and/or a per-row units cell convert at the parts
reader; an unset unit is an error, not a default. Inches use 25.4 exactly. A row's cell wins over the mapping.

### 4. Nesting, sheets and toolpaths
- Panels are grouped by (material, thickness rounded to a micrometre) in order of first appearance; each group is
  nested on its own sheets (a sheet carries one thickness), sheet size from the mapping, thickness of the group.
  The document's active sheet is reused for the first sheet when it has no objects, otherwise sheets are added;
  names read `<material> <thickness>mm #n`. The first sheet becomes active.
- `nestRectangles`: shelf packing, first fit decreasing by height: items oriented (length along X; with
  `nest.rotate` the orientation giving the lower shelf, quarter turns only), sorted by height then width then input
  order, put on the first shelf with room (sheets in order), else a new shelf on the first sheet with room, else a
  new sheet (limit 64 per group). Deterministic; margins and gaps are guaranteed (checked by property tests). It
  is not optimal; it is simple, explainable and bounded **(reading)**.
- **One panel that fits no sheet refuses the whole import** and changes nothing (a cabinet with a missing panel is
  worse than none); the diagnostic names the panel and its row.
- Objects: panel = rectangle on the panel layer named `<label> (n/qty)`; hole = circle on the holes layer; dado =
  rectangle on the dados layer. A quarter-turned panel maps part (x, y) to (width - y, x) before translating.
- Toolpaths (only with a session and a tool per kind; no tool = a note, not an error): per sheet, one **Pocket**
  per distinct dado depth, one **Drilling** per distinct hole depth (through = the sheet thickness), then the
  **Profile** (outside) of all panels at full thickness, selecting exactly the objects built. Through cuts have no
  tabs (the Profile form can be edited afterwards) **(reading)**. A toolpath the session refuses is reported and
  the import stands.

### 5. Hostile input and limits
| limit | value | limit | value |
|---|---|---|---|
| parts file | 8 MiB (size checked before reading) | holes per part | 200 |
| mapping file / lines / line | 64 KiB / 512 / 1024 B | dados per part | 32 |
| CSV records | 20000 | quantity per row | 1000 |
| cells per record | 64 | panels (sum of quantities) | 5000 |
| bytes per cell | 4096 | part rows | 10000 |
| sheets per group | 64 | label / material | 128 B |
| diagnostics kept | 200 (the rest counted and summarised) | dimensions | 1..10000 mm |

CSV: RFC 4180-style quoting (doubled quotes, delimiters and line breaks inside quotes), BOM dropped, LF/CRLF/CR
records, blank records skipped, text after a closing quote kept with a warning, NUL bytes, an unterminated quote
and every limit above are errors (the parse stops; nothing is partially returned). Delimiter `auto` picks the
first record's reading with most cells (ties: comma, semicolon, tab) and warns when more than one reading splits it.
**Cells are inert data**: numbers are parsed by a strict decimal grammar (optional `-`, digits, one `.`; no `+`,
exponent, hex, `inf`, `nan`, decimal comma or anything a spreadsheet would evaluate), text cells are never
interpreted, control characters become spaces, over-long text is cut on a UTF-8 boundary, and a label starting
with `=`, `+`, `-`, `@` is kept verbatim with a warning (it is only ever an object name here). Row errors skip the
row and are reported with the record number; the form refuses to import while errors exist unless the user
allows importing the valid rows (`allowPartial`) **(reading)**. Files are never executed or `eval`ed; paths come
only from the caller.

### 6. XML
Not delivered: the card marks it optional and the CSV path is the one the mapping describes. A later XML reader
would produce the same `CsvTable` (rows of cells), so mapping and everything after it is unchanged.

### 7. Failure and undo
`buildPanels` is all-or-nothing for everything that can be known beforehand (mapping, rows, nesting, layers hidden
or locked). Only exhaustion of the document's identifier space can stop it midway; the partial result is then one
undoable step and is reported as an error. Undo removes sheets, layers, objects and toolpaths together; redo
restores them.

## Consequences
- The mapping schema is the contract to review: key names, the holes/dados cell syntax and the readings marked above
  (dado position = near edge; through cuts without tabs; no import with errors unless allowed; shelf nesting).
- Mixed-thickness panels never share a sheet; offcuts and grain-aware nesting are not modelled.
- A general Nest Parts, when built, replaces `nestRectangles` behind `buildPanels` without changing the files.

## Mutation review (new logic)
Hand-mutated and run against the `[parts]` tests; killed: nest fit comparisons (x and y, with and without the
epsilon), shelf height order, gap accumulation (both axes), rotation preference and its tie, rotated hole/dado
mapping, hole/dado extent, position, depth and diameter bounds, hole and dado count limits, dimension lower bound,
header match counting, delimiter tie-break and mixed warning, column index bound, file-size bound, header-only
detection, reuse of an empty active sheet, refusal on unplaced panels. Survivors, argued equivalent: comparisons
`>=` vs `>` against `thickness - 1e-9` for a dado depth (differ only inside the 1e-9 tolerance); `x <= limit + eps`
vs `x < limit + eps` (same, tolerance); the 10000-row cap is only reachable below the 5000-panel cap with rows of
quantity 0, which are rejected, so the cap is a cheap pre-check whose exact boundary cannot be observed (it is
tested one past); the compound mutation of the delimiter `best` bookkeeping preserves first-maximum-wins.
