# ADR-0030: Node editing (node and handle model, undo granularity, parametric objects)

Date: 2026-11-07 · Status: proposed (T-0206)

## Context
T-0206 adds the NodeEdit tool (key N). The feature inventory ("Node Editing") names the mouse
gestures, the span and node context menus and the letter keys, but not their geometry, the
numbering of nodes, the undo granularity or what happens to parametric objects. This ADR records
the choices. The logic is Qt-free (ADR-0014) and lives behind the tool controller (ADR-0027).

## Decision

### 1. Layers
- `viewmodel/node_ops.*`: pure functions on one `Contour` (move nodes, set control point, reshape,
  split, insert, delete, cut, convert, smooth, rotate start). No documents, no view, no undo.
- `viewmodel/node_edit.*` (`NodeEditor`): model, hit testing, selection, pointer gestures and the
  operations on the document. `ToolController` owns one, routes the pointer events of
  `ToolKind::NodeEdit` to it and supplies the snapped point.
- Core: `document::SetObjectContoursCommand` (new file `node_edit_command.hpp`) replaces the
  contours of vector objects and resets their parameters to `NoParams`; `UndoStack::cancelLast`.

### 2. Nodes, handles, hit order
- A closed contour of n spans has n nodes (node i starts span i); an open one n + 1. A full-circle
  arc has no nodes of its own: the editor always works on `nodeops::normalised(contour)` where it is
  two half arcs, so a circle shows two nodes and is stored as two half arcs after its first edit.
- Handles: nodes, Bezier control points (p1 at the span start, p2 at its end), and a virtual
  midpoint per span at parameter 0.5 (Bezier t = 0.5, line/arc half length). Hit test in pixels
  (default 6): nodes beat control points beat midpoints beat the span itself; nearest wins in a class.
- Node and span selections are kept in the editor, not in the document, and are validated on every
  query (removed object, locked/hidden layer, other sheet, index out of range are dropped), so stale
  references are harmless. A span counts as selected for span operations when both its end nodes
  are selected.
- Edited objects: the vector leaves of the document selection (groups edit their leaves) on the
  active sheet with visible, unlocked layers (the leaf's and every ancestor group's).

### 3. Moving things
- Node move: lines and Beziers end exactly on the target (Bezier handles keep their offset from
  the node); arcs keep their *bulge* `tan(sweep / 4)` and are re-fitted to the new end points (a
  pure translation of both ends translates the arc exactly). Moves that would make a span shorter
  than 1e-6 mm, leave +-1e9 mm or be non-finite are refused as a whole; during a drag the step is
  skipped and the last valid state stays.
- Control point: placed at the pointer; with *Keep Bezier Tangency* the Bezier on the other side of
  the node is turned collinear (its handle length kept).
- Span drag: a Bezier moves its control points so the curve point at the grabbed parameter (clamped
  to 0.05..0.95) moves by the pointer delta (minimum-norm split of the delta onto p1 and p2); an arc is
  re-fitted through its end points and the moved point; a line moves with both end nodes.
  Ctrl+drag moves any span with both end nodes (neighbouring spans follow).
- Mirror mode (H/V): when the grabbed node moves by d, selected nodes on the other side of the axis
  through the centre of the selected nodes' bounding box move by d mirrored across that axis
  (horizontal mode: left/right symmetry, x mirrored), nodes on the axis keep their coordinate on
  the mirrored axis, nodes on the grabbed node's side move by d. A symmetric node set stays
  symmetric to rounding. Typed relative moves use the same rule with the first selected node as the
  grabbed one.

### 4. Structure operations
- Insert point: exact split (line, arc by angle, Bezier by de Casteljau): same curve. Refused within
  1e-6 mm of an end. Insert Midpoint uses parameter 0.5.
- Delete point: the spans around the node become one; two lines give a line, anything with a Bezier
  gives a Bezier keeping the outer handles (a line or arc side gets a handle a third along the new
  chord), arc-only joins become a line. Open contours drop the end span when an end node is deleted.
  A result with fewer than two nodes (closed lines: three) is refused.
- Delete span: a closed contour opens (the part after the removed span becomes the start); an open
  one splits. Cut vector: open pieces that together are exactly the original spans; a closed
  contour cut at k nodes gives k open pieces; a span is cut at the stated position (else its
  midpoint) after an exact insert.
- Convert: line: end points; Bezier: handles a third along the chord, or through a stated point at
  t = 0.5 (quadratic raised to cubic); arc to Bezier: `ArcSpan::toBeziers` (up to four spans);
  arc: through three points or by bulge; from a Bezier the arc passes through its t = 0.5 point;
  from a line without input the bulge is 0.25 (a shallow arc, about 56 degrees; the inventory gives
  no default). Collinear points are refused.
- Smooth: G1 at the node. Two Beziers share the bisector of the incoming and outgoing tangents
  (handle lengths kept); a Bezier next to a line or arc follows that side; two lines become
  Beziers; a line next to an arc becomes a Bezier aligned with the arc; arc to arc converts the
  second arc to Beziers. Already-smooth nodes are skipped; a cusp (tangents opposed) and the ends of
  open contours are refused.
- Close / Join / Extend use `closeContour`, `joinContours`, `extendSpan` (ADR-0028) with the
  default tolerance. Join works on all open contours of the edited objects: a chain stays in its first
  source's object, objects left without contours are removed (selection pruned), all in the same
  undo step. Extend's targets are all editable contours of the sheet except the contour itself.
- Start point: closed contour rotated so the node becomes node 0. Match X / Y: set the coordinate
  of every selected node to that of the first selected one. Properties: absolute assigns the typed
  coordinate(s) to every selected node (exact doubles), relative adds.

### 5. Keys (the inventory lists the letters without meaning)
I insert point, D delete point, S smooth, C cut vector, B convert to Bezier, A convert to arc, L
convert to line, X / Y match coordinates, H / V mirror mode (toggle), K keep Bezier tangency (toggle),
J join, P properties dialog. Close, Reverse, Delete Span, Start Point, Extend and Insert Midpoint
have no key. Letters act only while the NodeEdit tool is active; the canvas accepts the
`ShortcutOverride` for plain letters then, so window-level shortcuts of other cards (M, T, R, H, V,
J) do not fire in node editing.

### 6. Undo granularity
- Every operation is one `SetObjectContoursCommand` (a macro with `RemoveObjectsCommand` for Join
  that empties objects). A drag pushes the same command per pointer step; consecutive commands on
  the same objects coalesce (`mergeWith`: first snapshot, latest result), `breakCoalescing()` before
  the first and after the last step, so any number of steps is one undo step and the final
  geometry is what the single numeric edit gives (each step is computed absolutely from the
  geometry at drag start, not incrementally).
- A step equal to the drag-start state is not pushed before something was pushed (a click does not
  touch the object or its parameters); after the first push every step is pushed, so dragging back
  to the start still replaces the earlier result.
- Escape (or a tool switch) during a drag calls `UndoStack::cancelLast`: the latest step is reverted
  and dropped, so no redo entry remains. If anything else edited the document during the drag
  (undo, another command, a replaced document) the editor notices that the undo stack's
  `generation()` changed, ends the drag without reverting anything and ignores the snapshots.
- Typed operations call `breakCoalescing()` before and after, so two typed moves are two steps.

### 7. Parametric objects
A node edit resets `CreationParams` to `NoParams` in the same command; undo restores the whole
payload verbatim, so a rectangle's parameters return exactly. A drag that never changes anything
keeps the parameters.

### 8. Snapping during a drag
Every pointer position goes through the snapper (Shift disables it) with the objects being edited
in `SnapQuery::exclude`. A drag changes the document on every step, which would invalidate the
generation-keyed `SnapIndex` (ADR-0027) 60 times a second; but the only objects that change are the
excluded ones, and the index never dereferences an excluded entry. So while a node-edit drag is
active the controller keeps the index built at the drag's start (key: the generation of that
moment); `NodeEditor::validateGesture` ends the drag if the generation moves for any other reason,
and the index is rebuilt by the normal key check at the next event after the drag.

## Consequences
- A pointer step costs the edited objects only (10k objects, one edited: 0.06 ms in Release).
- Circles become two-node closed vectors of half arcs after node editing; the parametric circle
  is restorable only through undo.
- Interpretations listed here (keys, default bulge, mirror rule, handle choice on delete) are one
  function each if the owner decides otherwise.
