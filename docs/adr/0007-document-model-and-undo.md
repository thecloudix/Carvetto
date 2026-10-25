# ADR-0007: Document model storage, ids and command-based undo

Date: 2026-10-24 · Status: accepted (director, 2026-10-24; amendments A1 T-0011, A2 T-0013, A3 T-0014)

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
   payloads because the inverse transform is not exact once arcs degrade to Beziers. Commands
   that change object structure (add, remove, group, ungroup, layer/sheet moves, delete layer or
   sheet) record each object they touch in an `ObjectChangeSet` (its state, or absence, before
   and after) and undo/redo by restoring those snapshots, so redo replays exactly what the
   first `apply` produced rather than recomputing it.
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
7. **Groups are objects, and group links are an invariant.** `GroupObject` holds child ids;
   children stay in the object table with `parent` set, so selection, layer membership and bounds
   queries work uniformly. Every command keeps these rules, and `Document::checkIntegrity()`
   reports any breach (dangling/duplicate child ids, child/parent disagreement, non-group or
   missing parent, cycles, missing layer/sheet, a child on another sheet than its group, stale
   active layer/sheet or selection):
   - **Removal detaches.** Removing an object removes it from its parent's child list. A group
     left with **no** children is removed too, recursively up the chain. A group left with **one**
     child is kept (the user can ungroup it); it is never collapsed implicitly.
   - **Removing a group** with `RemoveObjectsCommand` removes its descendants.
   - **Ungroup splices.** Dissolving a group puts its children in the group's place in its parent's
     child list (same position, same order), or makes them top-level when the group was top-level.
   - **Delete layer with contents** dissolves every group on that layer the same way (so a child
     on another layer survives — top-level if the group was top-level, parent cleared), then
     removes every remaining object on the layer, detaching it from groups on other layers and
     pruning groups left empty.
   - **Moves follow the tree.** Moving a group to another layer or sheet moves all its
     descendants; a child can still be moved to another *layer* on its own and stays grouped.
     Groups never span sheets, so an object moved to another sheet without its group is detached
     from it first.
   - **Delete sheet** removes the objects on it (pending owner decision; confined to
     `DeleteSheetCommand::apply`) using the same dissolve-then-remove helper.
   - `AddObjectCommand` with a `parent` that is an existing group appends the object to that
     group (on the group's sheet); any other parent value is cleared.
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
- Tests run `checkIntegrity()` after every step, undo and redo of a randomized command mix
  (120 seeds × 120 steps), and compare the document after undo-all / redo-all with the
  recorded start / end states.
- `.carv` I/O (T-0012) must persist the id generator's high-water mark and write objects in id
  order to keep `job.json` byte-stable.
- Future commands (toolpath create/edit) follow the same contract; a command touching many
  objects should snapshot only the payloads it changes.
- Deleting a sheet deletes the objects on it (undoable). Whether the product should instead move
  them is a product question left to the owner; the command is isolated so either is a local
  change.

## Amendments

### A1 (T-0011, 2026-10-24): id ceiling and exhaustion
Decision 3 is amended. The shared generator no longer wraps at 2^64:
- **Ceiling.** Every id lies in `[1, IdGenerator::kMaxId)` with `kMaxId = 2^62`
  (`carvetto/document/ids.hpp`). File I/O (T-0012) is to use the same constant to reject out-of-range ids on
  load. 2^62 leaves headroom so `value + 1` and small sums never overflow 64 bits.
- **Exhaustion.** Once `peek() == kMaxId` the generator is exhausted and stays so: it never wraps,
  never reuses a value and never goes backwards. `reserveThrough(v)` with `v >= kMaxId - 1` clamps
  to exhausted instead of wrapping. `tryAllocate()` returns nullopt; `allocate()` asserts in Debug
  and returns 0 (the null id) without advancing in Release. Raw `Document::insertSheet/Layer/Object`
  with a null id on an exhausted generator therefore assert (Debug) or insert nothing and return a
  null id (Release).
- **Commands follow decision 2.** Every command that creates something (`AddObjectCommand`,
  `PasteObjectsCommand`, `GroupObjectsCommand`, `AddLayerCommand`, `AddSheetCommand`) checks
  `IdGenerator::canAllocate(n)` before allocating anything and returns false from its first `apply`
  when too few ids remain, or when a caller-supplied id is at or above `kMaxId`. It is then a no-op:
  nothing changes, nothing is recorded, `Document::execute` returns false; same in Debug and Release.
  Paste is all-or-nothing (needs one id per clipboard object). Redo never allocates, so undo/redo
  is unaffected by exhaustion.
- **Integrity.** `Document::checkIntegrity()` reports `IdOutOfRange` for a null id or an id
  `>= kMaxId` on any sheet, layer or object (including an object whose stored id differs from its
  key), and `IdNotBelowNext` when the generator's next id is `<=` any existing id.

### A2 (T-0013, 2026-10-24): preset ids must be fresh
Decision 3 is amended. One generator serves sheets, layers and objects, so an id value names at most
one item of *any* kind, and a value once handed out is never taken again — not even through a
caller-supplied (preset) id.
- **Commands.** `AddObjectCommand`, `AddLayerCommand` and `AddSheetCommand` (the only commands that
  accept a preset id; paste and group always allocate) accept a non-null id only when it is in
  `[IdGenerator::peek(), kMaxId)` and not used by any sheet, layer or object. Otherwise the first
  `apply` returns false: no-op, nothing recorded, `Document::execute` returns false (decision 2).
  This also covers an id freed by undoing (or deleting) the item that held it: it is below the next
  id, so it cannot be re-taken by a new preset add. Only redo of the original command brings it
  back. Redo never re-runs the check (the command's own id is below the next id by then).
- **Raw mutators** (`insertSheet/Layer/Object`) still only assert same-kind duplicates; they are
  the building blocks undo/redo and file loading use and are not a user-level entry point.
- **Integrity.** `Document::checkIntegrity()` reports `DuplicateId` once per id value used by more
  than one sheet, layer or object (across kinds, or two sheets/layers with the same id). The issue's
  `object` is the object owning the value, if any. File I/O (T-0012) gets this check on load for free.

### A3 (T-0014, 2026-10-24): an object's stored id must equal its key
Decision 4 is amended. Objects are keyed by id in the map *and* carry their id in `Object::id`;
the two must agree. Nothing in the command layer can make them disagree, but raw mutation
(`findObject(k)->id = …`) or a faulty loader can, and a mismatch would make the stored id an
untracked claim on an id value.
- **Integrity.** `Document::checkIntegrity()` reports `IdKeyMismatch` for every object whose stored
  `id` differs from its key (`object` = the key, `related` = the stored id). The stored id is also
  range-checked (`IdOutOfRange` / `IdNotBelowNext`, as in A1).
- **DuplicateId counts stored ids.** A mismatching stored id is an extra owner of its value
  (named "object (stored id)" in the message), so it is reported as `DuplicateId` when that value
  is also a sheet, layer or object key. An object whose stored id equals its key is counted once.
- File I/O (T-0012) gets this check on load for free; a loader that writes objects from their
  stored id never produces a mismatch.
- **Complexity.** Because the loader runs `checkIntegrity()` on untrusted input, it must be
  O(N log N) in the number of objects whatever the group depth or width: (group, child) links are
  collected into a set once (no linear search of a child list), and parent-chain loops are found by
  one iterative walk over the parent links with on-path/done marks (no recursion, no per-object
  re-walk). Issue kinds, their per-object order and the `Cycle` issue's `related` object (the first
  object on the loop that the chain reaches) are unchanged.

### A4 — Sheet deletion (owner decision, 2026-10-24)
Deleting a sheet deletes the objects on it (undoable), as implemented in `DeleteSheetCommand`. The project owner confirmed this behaviour; the alternative (moving the objects to another sheet) was rejected. The "pending owner decision" notes in T-0010/T-0012 are closed.

