# ADR-0007: Document model storage, ids and command-based undo

Date: 2026-10-24 · Status: proposed (coder, T-0010; for director review)

## Context
T-0010 asked for an in-memory document (job, sheets, layers, objects, selection, variables) with
stable ids and an undo stack with coalescing, but left the mechanism open. The choices below are
the ones a later card (file I/O, toolpaths, UI) would otherwise have to reverse-engineer.

## Decision
1. **Command pattern, not whole-document mementos.** `Command` has `apply(Document&) -> bool`,
   `revert(Document&)`, `text()` and `mergeWith(const Command&)`. Each command records only the
   delta it needs (allocated ids, removed objects, previous layer per object). Commands whose
   state is small (`Job`, `Layer`, `Sheet`, a variable) are *local* mementos: they store the
   before/after value of that one thing. `TransformObjectsCommand` snapshots the affected
   payloads because the inverse transform is not exact once arcs degrade to Beziers.
2. **`apply` returning false means "no-op, do not record".** The stack discards such commands
   (e.g. deleting the last layer, moving objects to the layer they are already on). Callers that
   need to know ask the return value of `Document::execute`; nothing throws for product-level
   refusals. Programmer errors (inserting an object on a non-existent layer, duplicate ids) are
   `assert`s.
3. **Ids are 64-bit, monotonic and never reused**, from one `IdGenerator` shared by objects,
   layers and sheets. A command that creates something allocates its id on the first `apply` and
   reuses it on redo, so undo/redo preserves identity; paste always allocates fresh ids. File
   loading will call `IdGenerator::reserveThrough` with the largest id read.
4. **Objects live in a `std::map<ObjectId, Object>`.** Because ids are monotonic, iteration order
   is creation order, which is the deterministic stacking order within a layer. Layers and sheets
   are small `std::vector`s whose position *is* the order (no `order` field to renumber). The
   default layer is whichever layer sits at index 0.
5. **Coalescing is opt-in per command type and bounded by a "run".** `UndoStack` asks the top
   command to `mergeWith` the new one only while a run is open; `undo`, `redo`, `setClean`,
   `endMacro` and `breakCoalescing()` close it. The UI is expected to call `breakCoalescing()` on
   mouse release / form close. Transform commands merge when their id set is identical; property
   edits (layer, sheet, job, variable) merge per target.
6. **Macros nest.** `beginMacro`/`endMacro` collect pushes into one `MacroCommand`; only the
   outermost `endMacro` records it, and an empty macro records nothing.
7. **Groups are objects.** `GroupObject` holds child ids; children stay in the object table with
   `parent` set, so selection, layer membership and bounds queries work uniformly. Removing a
   group removes its descendants; removing a child from a surviving group edits the parent's
   child list (both undoable).
8. **Selection is not undoable** and is pruned automatically when an object is removed.

## Rationale
- A whole-document memento per step would make the 10 000-insert criterion impossible and
  would copy heightfields later; per-command deltas are O(change).
- Returning `bool` from `apply` keeps "nothing happened" out of the undo history without a
  separate validation API duplicating each command's preconditions.
- Monotonic ids remove the need for a "free list" and make `.carv` files append-friendly.
- Keeping order implicit (map order, vector position) avoids an `order` field that every
  insert/delete would have to renumber, and is trivially serialisable.

## Consequences
- `.carv` I/O (T-0012) must persist the id generator's high-water mark and write objects in id
  order to keep `job.json` byte-stable.
- Future commands (toolpath create/edit) follow the same contract; a command touching many
  objects should snapshot only the payloads it changes.
- Deleting a sheet deletes the objects on it (undoable). Whether the product should instead move
  them is a product question left to the owner; the command is isolated so either is a local
  change.
