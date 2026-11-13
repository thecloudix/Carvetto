# ADR-0057: Job sheet output: formats, safe writing and bounded storage

Status: accepted (T-0259, T-0263)

## Context
T-0244 made the Qt-free job sheet (`makeJobSheet`, HTML and text renderers). The app needs a File
menu action that writes it, a PDF option, and the sheet must stay bounded for huge jobs.

## Decisions
- File > Create Job Sheet... asks for a file with a filter for HTML, text and PDF; a typed extension
  wins over the filter. HTML is opened in the system browser afterwards (not in headless/quiet runs).
- HTML and text go through `writeJobSheet`, which reuses `writeOutput` (random staged file, exclusive
  create, atomic move, no links followed) after `checkJobSheetPath`: refuses empty paths, any `..`
  component, an extension of another format, file names that are not their own sanitised stem
  (control characters, reserved device names, trailing dots) and existing folders.
- PDF is made in the app layer with Qt's `QTextDocument` and `QPdfWriter` (Qt Gui, no extra module),
  written through `QSaveFile` so a failed run leaves no partial file. The PDF carries the HTML sheet
  without the inline SVG layout, because Qt rich text has no SVG. The PDF path refuses a symbolic-link target, as the HTML/text writer does, so all three
  formats treat links the same way. A direct print dialog is not
  added; the PDF and the browser cover printing.
- Per-sheet/per-side split is left to T-0261 (double-sided jobs).
- Storage: `makeJobSheet` stores only the capped rows (500 toolpaths, 100 tools, 20 toolpath names
  per tool) and keeps the real counts (`toolpathsTotal`, `toolsTotal`, per-tool `toolpathsTotal`,
  `thumbnailOmitted`) so the "N more" notices are right; strings are clipped when stored. The tool
  table is keyed on (number, clipped name, diameter) in an ordered map, so lookup is logarithmic
  (20,000 distinct tools well under a second). Tools whose names differ only after the clip limit
  are shown as one.
- The thumbnail states how many toolpaths it leaves out.
