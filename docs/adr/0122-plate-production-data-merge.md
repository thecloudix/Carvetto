# ADR-0122: Plate production (data merge)

Date: 2026-11-17 · Status: proposed (T-0347) · Builds on ADR-0014 (Qt-free viewmodel), ADR-0007 (commands)

## Context
The feature inventory lists Plate Production: a template replicated per data row, `!!VAR!!` text variables mapped to
columns, a counter variable, layout on sheets, optionally one toolpath per plate. The card asks for a bounded,
deterministic, generic-CSV engine. The inventory is silent on several details; these are my readings, for owner review.

## Decision
1. **Engine split.** `core/io/delimited` parses RFC 4180-style text (separator, quotes, CRLF, BOM) with caps (4 MiB, 20000
   records, 256 columns, 4096 bytes per field) and line-numbered diagnostics for malformed records. The viewmodel
   `data_merge` builds the plates; the app only hosts the dialog.
2. **Template and variables.** The template is the current selection on the active sheet; the plate is its bounding
   box. A placeholder is `!!NAME!!` (letters, digits, `_`, starting with a letter) and is recognised only in text
   objects. Text objects with placeholders are laid out again per plate with their embedded font; everything else is
   copied by translation. Unbound placeholders refuse the merge. Document variables (`{Name}`, numeric) are not
   involved: they are one value per document, a plate needs one value per row.
3. **Counter.** `prefix + zero-padded(start + k * step) + suffix`, numbered by plates actually made.
4. **Layout.** Grid (fill the sheet row by row from the top-left inside the margins, overflow to new copies of the
   active sheet) or one plate per sheet. "Layers" mode of the inventory (overflow onto layers of one sheet) is not
   implemented: plates would overlap on one sheet.
5. **All or nothing, one undo step.** Malformed rows or texts that cannot be laid out refuse the whole merge with
   line numbers; `skipBadRows` opts into merging the good rows. Cap: 1000 plates and plates x (objects + spans) <= 500000.
6. **"Toolpath-ready".** Plates are ordinary vector objects fully inside the material, so any toolpath form applies to
   them. Calculating toolpaths per plate and writing one file per plate (inventory "Toolpath Options") is not part
   of this card; it can reuse the existing toolpath and save-toolpaths operations on the plate selection.
