# ADR-0126: Printing

Status: accepted (T-0349)

- Page layout and tiling are Qt-free in `viewmodel/print_layout`: `layoutPrint` takes the world box, the
  paintable page and the options (fit to page, or true size at a scale with tiling) and returns the tiles
  (world window and registration marks per page). Tiles overlap by a configurable band (default 10 mm) and
  are numbered row by row from the top-left; a page cap (400) refuses absurd 1:1 prints.
- Registration marks are crosses at the corners of each page's printable area and at the inner corners of
  the overlap band. A mark of one sheet lies at the same world position as a mark of its neighbour, so
  sheets are aligned by laying the crosses over each other.
- The app paints every page onto a `QPagedPaintDevice` (`PrintController::render`), so the same code serves
  QPrinter, the print preview dialog and `QPdfWriter`. Pages are vector: the canvas's own scene painter runs
  with a view in device dots (`CanvasWidget::paintForPrint`, white paper, optional toolpath preview strokes).
  The job sheet reuses the existing HTML rendering printed through `QTextDocument`.
- `QPrinter`, `QPrintDialog` and `QPrintPreviewDialog` need Qt PrintSupport, which is an optional
  dependency (`CARVETTO_HAVE_PRINTSUPPORT`): without it only Print to PDF works and Print / Print Preview
  are disabled. Print to PDF needs only Qt Gui and is what `--self-test` drives (page count read back from
  the PDF).
