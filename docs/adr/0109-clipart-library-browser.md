# ADR-0109: Clipart library browser over local folders

Status: accepted (T-0333)

## Context

Users keep collections of SVG, DXF and PNG clipart on disk and want to browse them with thumbnails,
find a piece by name, mark favourites and put it into the drawing. The importers already exist (SVG in
core, DXF and bitmap in the view model); what is missing is a bounded way to list, preview and insert
files that nobody vetted. There is no online store, no account and no clipart shipped with Carvetto.

## Decision

Everything except the widgets is Qt-free in `carvetto_viewmodel` (`clipart_files`, `clipart_library`,
`clipart_thumbnail`, `clipart_insert`); the dock (`app/src/clipart_panel.*`) replaces the placeholder
in the Library dock and is glue only.

* **Recognition.** A file counts as clipart by extension (.svg .dxf .png, any case) *and* content: the
  first 4 KiB must be a PNG signature, an XML document with an `<svg` element, or an ASCII/binary DXF.
  A mismatch ("named .png but the content is svg") is refused with a rename hint. Compressed containers
  (gzip incl. .svgz, zip, 7z, xz, bzip2, zstd) are refused by name: Carvetto never decompresses
  clipart files, so a decompression or zip bomb cannot reach any parser. (A PNG's own zlib stream is the
  one exception, and it is bounded; see "PNG decompression bombs" below.)
* **Budgets** (`ClipartLimits`, far below the importers' own caps): SVG/DXF files 8 MiB, PNG files
  32 MiB, PNG pixel data 80 MiB inflated (below), checked from the file size before reading; PNG 8192 px an edge and 16 Mpx, read from the IHDR
  before the decoder is called (so a tiny file claiming a huge canvas never allocates); SVG nesting 48,
  200k elements/spans, 400k visits and `<use>` depth 8 (entity expansion is already refused as DOCTYPE);
  DXF 200k spans, 50k block instances, depth 16; at most 20k contours per file (each becomes an object).
  A refusal message always starts with the file name and says which limit was hit.
* **Scan** (`scanClipartFolder`): depth-first, sorted, depth 4 below a library folder, 5000 files and
  60 000 directory entries examined per scan (shared across folders), symbolic links and dot-names
  skipped (no link loops), unreadable folders reported. Oversize files are listed with a refusal text
  and no thumbnail is attempted. Work is counted (`visited`) so tests assert bounds without timing.
* **Library model** (`ClipartLibrary`): up to 32 folders and 10 000 favourites, persisted through an
  injected `ClipartStore` (the app implements it with QSettings; tests use `MemoryClipartStore`);
  stored items are sanitised on load. Search is ASCII case-insensitive, whitespace-separated words that
  must all occur in the file name or its folder path. A favourite is a file path; favourites outside the
  current folders are kept but not shown. Duplicate files reached through two folders list once.
* **Thumbnails.** A software renderer (no Qt) draws SVG/DXF outlines Y-up and box-filters decoded PNGs
  (never enlarged); DXF text is not drawn. One worker thread (`ClipartThumbnailService`) renders and
  puts results in an LRU cache bounded by bytes (32 MiB, never below 1 MiB) and by entries (4096);
  failures are cached as small entries with their message so a bad file is not retried every paint.
  The list of finished paths between `takeCompleted` calls is bounded by the cache's entry bound (the
  oldest are dropped; their thumbnails are evicted by then). The UI thread only requests the visible rows (queue bounded at 512; a full queue is retried on the
  next timer tick) and converts finished results to icons. `clear()` and `cancelPending()` bump a
  generation so a result finishing afterwards is discarded. The cache is keyed by path; Refresh (or any
  folder change) clears it, so edited files are re-read after a refresh, not on every paint. The PNG
  decoder is the app's existing QImage decoder, injected.
* **Insertion** (`insertClipartFile`): one macro "Insert clipart" containing the importer's own steps
  (SVG shapes as vector objects on the active layer; DXF layers matched or created by name by
  `importDxfIntoDocument`; a PNG embedded by `importBitmapIntoDocument`) and one transform. Vector
  files are parsed first with the tight limits, so a hostile one never touches the document. If anything
  fails after the macro began (an importer error, an unmeasurable result, or any `std::exception` thrown
  while the macro is open), the macro is closed and then `undoAndDiscard` removes the whole attempt
  (no redo entry, no residue). Whether the macro recorded anything is read from
  `UndoStack::generation()`, which changes with every command pushed. The rollback is exact for the
  document (the `.carv` bytes are identical afterwards), the history (step count, index, no redo tail)
  and the selection; only the generation itself differs, as it does after any undo.
  `ClipartInsertOptions::afterImport` is a test seam that lets a test fail the insert at that point.
  Selection becomes the new objects.
* **Dropping** works on the canvas for any SVG/DXF/PNG URL, whether it comes from the browser's list
  (which drags a local file URL) or from the file manager.

## PNG decompression bombs

The IHDR limits (8192 px an edge, 16 Mpx) say nothing about the IDAT stream: a 1 x 1 header over a zlib
stream of zeros inflates to gigabytes at about 1000 : 1 (a 2 GiB stream is 2.1 MB, inside the 32 MiB
file cap), and the Qt decoder inflates all of it, measured at about 3.5 s on the UI thread at insert
time (the verifier's figure, with a 2 GiB stream; thumbnails reach the same decoder on the worker).
`io::checkPngPixelData` (`core/src/io/png_guard.cpp`, Qt-free, miniz's `tinfl` that `core` already
carries for `.carv`) therefore runs before any decoder call, in `checkClipartBytes` and again in the
thumbnail renderer: it computes the raw size the header declares (every scanline with its filter byte,
every Adam7 pass; invalid colour type / depth / interlace combinations are refused), refuses a header
above `ClipartLimits::maxPngRawBytes` (80 MiB: the 64 MiB of a 16 Mpx RGBA 8-bit image plus filter
bytes; 16-bit RGBA at the full pixel limit is refused), and inflates the concatenated IDAT payloads with
an output budget of that size plus 4096 bytes of slack for padding encoders. The first call that pushes
the total over the budget ends the check ("far more pixel data than its header declares"); `tinfl`
writes at most one 32 KiB dictionary window per call, so no more than 32 768 bytes are inflated past
the budget. Truncated or corrupt streams and a missing IDAT are refused as damaged; CRCs and the
Adler-32 trailer stay the decoder's job.

Measured here (Debug build of `core`, `miniz` at -O2, loaded machine; the figure varies by run):

| file | declares | inflates | guard |
|---|---|---|---|
| 1 x 1 RGB, 2 GiB of zeros in a 2.09 MB file | 4 bytes | stops at 32 768 bytes | 1.3 ms |
| 4096 x 4096 RGBA 8-bit, exact (a 64 KiB file) | 67 112 960 bytes | 67 112 960 bytes | 86 ms |

So the worst case a hostile file can cost before the decoder sees it is the declared size, never above
80 MiB + slack (about 110 ms), instead of the stream's own length; a bomb is refused in about a
millisecond. A legitimate image of the largest allowed size pays the 86 ms once more before its normal
decode. Tests sit on the exact budget (+0 and +1 byte), the cap (+0 and +1), both zlib shapes (stored
blocks, fixed-Huffman runs), split IDAT chunks, Adam7 sizes and damaged files; `inflatedBytes` in the
result is asserted, so the bound is checked by counting work, not timing.

## Product readings (not dictated by the card)

* **Scale to fit** means: uniformly scaled so the bounding box just fits a box of 50% of the active
  sheet's material width and height (100 mm x 100 mm when the material is empty), enlarging small
  clipart as well as shrinking large; a zero-width or zero-height drawing is scaled by its other side.
  The factor is `ClipartInsertOptions::fitFraction`.
* **Where**: a drop lands centred on the pointer; double-click and the Insert button centre on the
  material (the feature inventory's "double-click places at the job centre"). There is no stored
  cursor position because the pointer is over the panel when the button is pressed.
* The three thumbnail sizes of the inventory are reduced to one (96 px); "Open containing folder" and
  "Import to Level" have no counterpart here (Level import belongs to relief clipart, not to 2D files).
* Layers: SVG has no layer notion the user relies on, so its shapes go on the active layer; DXF keeps
  its layers.
* Bitmaps keep the existing import behaviour (embedded verbatim, ADR-0051) and are then scaled by the
  same transform.

## Mutant review

No mutant is left in source. Boundary mutants (`>`/`>=`, +-1) in the size limits, PNG edge/pixel
limits, contour limit, scan depth/file/visit limits, cache byte/entry bounds, favourites and folder caps
are killed by tests that sit exactly on and one past each limit. Behavioural mutants killed: sniff
rules (each archive, 5-digit group code, missing SECTION, 4 KiB window), name/extension agreement,
Y-flip and padding of vector thumbnails (pixel probes), box-filter averaging (checker -> 128), the
generation discard and the "re-requested after clear stays pending" branch of the service, single undo
step and undo text of every insert, scale choice by the tighter axis on a non-square material, the fit
fraction, drop-point centring, refusal leaving `UndoStack::generation()` unchanged. Argued equivalent:
`std::max(y0 + 1, ...)` in the box filter (the image is never enlarged, so the source range is never
empty), the `lru_.empty()` guard in `evict()` (bytes/entries cannot exceed bounds with an empty list),
`kPad` and the exact rounding of thumbnail sizes (cosmetic; the size tests pin the current values
anyway). Dot-name skipping is killed by a `.hidden.svg` file and a `.hiddendir`.

Added after verification (each killed by applying the mutant by hand and seeing the named test fail):
the 4 KiB sniff window (an `<svg` and a `SECTION` ending at byte 4096 are found, one byte later not;
`kSniffBytes` 4095 fails), the cache's `cost > maxBytes_` (an entry costing exactly the bound is kept;
`>=` fails), the 100 mm fit box of an empty material (50 fails), the height-only and width-only scale
branches on a non-square material (swapping fitW / fitH fails both), the budget comparison of the PNG
guard (`>=` fails), the completed-list bound (dropping the newest instead of the oldest fails), and the
rollback after the macro recorded steps: `undoAndDiscard` -> `undo` leaves a redo entry and the history
one step longer, `if (generation changed)` -> never rolls nothing back and leaves the objects, layers
and a cleared selection in place; the test compares the `.carv` bytes with those of a reference
document that inserted and discarded the same file (the id counter is never rolled back, so the
reference advances it identically). Before this fix an exception thrown while the macro was open went
to the outer handler, which closed the macro but left its steps on the undo stack; the earlier claim
that every failure after the macro began was rolled back was true only for importer errors, not for
exceptions.

## Consequences

* A folder of thousands of files costs one bounded scan and thumbnails only for the rows in view.
* The importers' looser caps are untouched; the browser simply refuses earlier.
* `ClipartStore` keeps QSettings out of the view model and the self-test away from the user's settings
  (the panel uses a scratch store until `setPersist(true)`).
