# ADR-0053: Trace Bitmap, Pick from 2D View and Bitmap Fading

Date: 2026-11-13 · Status: accepted (T-0274; completes the deferred part of ADR-0049).

## Decision
1. **Canvas draws bitmaps.** The 2D view draws each bitmap object (visible layer, active sheet) from its
   embedded image (ADR-0051), cached per asset key and dropped when the asset goes (undo).
2. **Bitmap Fading is a canvas display setting** (`CanvasWidget::setBitmapFade`, share of brightness 0..1).
   It is not part of the document, is not saved and is not an undo step. The Trace dialog applies it
   (default 60 % faded, adjustable) only when its image is a bitmap picked from the drawing, and restores
   the previous value when the dialog closes or asks to pick again.
3. **Pick from 2D View.** The dialog closes with result `kPickRequested`; the menu command then lets the
   canvas capture the next left click (Esc cancels) and `viewmodel::bitmapAtPoint` finds the topmost visible
   bitmap under it (vectors drawn over it do not matter). A new dialog opens on that bitmap with the same
   settings. A missed click or Esc returns to the previous dialog.
4. **Placement.** A picked bitmap traces from its embedded image (decoded in the app with QImageReader,
   same caps as files); the result width is the placement width and the origin its lower-left corner, so the
   traced shapes lie on the picture. The trace result is the usual single undo step.
