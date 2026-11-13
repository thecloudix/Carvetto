# ADR-0058: Text caret, selection, in-canvas frame, and keeping a draft across tool switches (T-0230)

Date: 2026-11-13 · Status: accepted, implemented (T-0230). Point 2 is a product reading **(owner)** to confirm.

## Context
The T-0211 text draft only appended and removed at the end, drew only the glyph outlines, and was discarded
when another tool was chosen. T-0230 listed a caret and selection, an in-canvas text frame, and the open
question "tool switch mid-draft: commit or keep".

## Decision
1. **Caret and selection live in the Qt-free `TextEditor` draft** as UTF-8 byte offsets (`caret`, `anchor`)
   that always sit on code point boundaries. Typing replaces the selection, Backspace/Delete remove the
   selection or one code point, `moveCaret` (Left/Right/line start/end/text start/end, optionally extending)
   and `selectAll` move it. Line moves follow the "\n" of the text, not box-wrapped lines. The caret and
   selection never touch the document (the draft is still document-free until commit).
2. **A draft is kept when another tool is chosen (owner).** Committing on a switch would create undo steps
   the user did not ask for and the opposite (discarding) loses typed work. The draft is not painted and takes
   no keys while another tool is active; choosing Text again resumes it with its caret; Escape in the Text
   tool, a click elsewhere with Text (commits as before), or a document replacement ends it. A box being
   dragged is a pointer gesture and is not kept. `ToolController::cancel()` only clears a draft while the
   Text tool is the active one.
3. **Overlay geometry is computed in the view model** (`ToolPreview::textFrameMm/textCaretMm/
   textSelectionMm`) so the Qt layer only paints. The frame is the box in Box mode, else the glyph bounds
   grown by a quarter em. Caret and selection quads are produced for Point mode only (one layout of the draft
   per edit); Box and Curve drafts have a frame but no drawn caret yet.

## Consequences
Caret positioning with the pointer, caret and selection for wrapped box text and text on a curve, and
bidirectional or vertical text remain open (see the T-0230 card).
