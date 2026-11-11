# ADR-0052: Photo V-carve app wiring (T-0275)

Date: 2026-11-11 · Status: accepted (T-0275, follows ADR-0047 and ADR-0051)

## Context
T-0250 delivered the Qt-free photo V-carve and T-0273 the embedded bitmap store. What was missing is
everything between a picture on disk and a toolpath in the Toolpaths dock: import, a record, a form,
calculation, staleness, canvas display. The cards leave open how a toolpath record refers to its bitmap
and how the existing vector-based session machinery copes with an operation that reads a bitmap.

## Decision
1. **Record.** `ToolpathParams` gains `PhotoVCarveParams` (variant index 10, `ToolpathOperation::PhotoVCarve`,
   operation name `photo_vcarve`, params in `job.json` only, like V-carve). The record's `selection.objects`
   names the bitmap object (the first bitmap object among them) and, for Selected Vectors, the vectors to
   follow. No new record field and no format minor bump: older readers refuse an unknown operation name
   exactly as they do for chamfer, inlay and the other operations added since 1.0.
2. **Selection.** The form's vector selector is an explicit object list (it starts from the current
   selection). Hidden/locked layers, other sheets and a bitmap without embedded pixels are refused with
   a message saying so. "All vectors" and "Layers" name no bitmap.
3. **Staleness.** The "vectors" fingerprint of a photo toolpath hashes the bitmap object (image key,
   pixel size, placement) and, for Selected Vectors, the resolved vectors (`inputsFingerprint`). Moving,
   scaling or replacing the bitmap, or editing the followed vectors, makes the toolpath stale; undo
   makes it current again, as for vector toolpaths.
4. **Calculation.** The job copies the encoded image bytes and placement (`PhotoInput`) so it stays
   independent of the document. Decoding (core reads Netpbm, the app's QImage decoder everything else) and
   gray conversion happen in `ToolpathJob::run`, on the worker thread, where the cancel flag is passed
   to `photoVCarve` (polled per line) and the 50 million sample work limit refuses oversized runs
   before generating. A cancelled run stores nothing. One calculation, one undo step, as for the other
   operations.
5. **Decoder.** `ToolpathSession::setBitmapDecoder` takes the app's decoder (`qtBitmapDecoder`: QImage
   to 8-bit RGB, transparency composited over white, size limits checked before conversion; thread safe).
6. **Import.** File > Import Bitmap embeds the file verbatim (`makeImportBitmapCommand`, one undo step,
   identical images shared), places it on the active layer scaled to fit the sheet's material (aspect
   kept, centred; 100 mm wide when the material is empty) and selects it. The user resizes it with the
   transform tools; the placement rectangle is what the toolpath uses. Hidden or locked active layers,
   unreadable images and files over the embedding caps are refused without changing the document.
7. **Canvas.** The canvas paints embedded bitmaps (decoded once, shrunk to at most 1024 pixels an edge)
   under the vectors, with a frame in the layer colour; picking already covered bitmaps (T-0201). The
   toolpath is previewed by the existing toolpath preview.
8. **Form.** Max Carving Depth, Stepover Retract, Safe Z, Line Spacing (% of tool diameter), Line Angle
   (not for Selected Vectors), chordal tolerance (Selected Vectors only), choices Strategy
   (Raster/Hatch/Selected vectors) and Invert. V-bits and engraving bits only (`photoVCarveToolAllowed`).
   The depth tolerance of ADR-0047 keeps its default and is stored in the record but is not a form field.

## Consequences
A photo toolpath depends on its bitmap object staying in the document; deleting the bitmap leaves a stale
toolpath that cannot be recalculated ("select a bitmap"), like deleting the vectors of any other toolpath.
