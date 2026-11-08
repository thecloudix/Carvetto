# ADR-0031: The toolpath session (definitions, selectors, calculation, recalculation)

Date: 2026-11-07 · Status: proposed (T-0220)

## Context
The core computes and posts every Phase-1 toolpath and `carvetto-cli` drives them, but the app has
no way to create one. T-0220 adds the Qt-free layer the Toolpaths dock (T-0221), the preview
(T-0223) and the save dialog (T-0224) sit on: the toolpath list, one form contract per operation,
the link between a toolpath and its vectors, the calculation, and recalculation. The native
records (ADR-0021) store `ToolpathParams` (the core's params structs), a `VectorSelection` (all /
layers / objects), the tool snapshot and `ToolSource`, and the computed moves; ADR-0018 gives every
operation the same result and diagnostic types; ADR-0007 makes ids monotonic.

## Decision
1. **Files** (`app/viewmodel`, no Qt): `vector_selector` (the selector), `toolpath_calc` (definition,
   job, calculation), `toolpath_form` (field contracts), `toolpath_session` (the list over a
   `Document`).
2. **A definition is the core's own data.** `ToolpathDefinition` = `ToolpathParams` (mm, radians; the
   variant index is the operation) + the tool as a `ToolSource` (tool database ids, with the
   machine and material that pick the cutting data) + V-carve clearance tools + a `VectorSelector`.
   Nothing is translated: the form writes into the core params structs, which are what the record
   stores, so the session cannot drift from the CLI. The cutting data are never overridden per
   toolpath (the record cannot hold them); feeds, speeds, pass depth and stepover come from the tool
   database, and a form field left at 0 ("use the tool's") shows the tool's value.
3. **Forms** (`ToolpathForm`) have the T-0204 contract: numeric `fields()` (id, label, group, kind,
   live text in the display units, typed/from-tool flags), option `choices()` (drop-downs and
   check boxes), `setField`/`setChoice` that validate one entry and change nothing when it is bad,
   an empty entry resetting to the default, `setTool` checking the tool against the database and
   the operation (`toolAllowedFor`), `problems()` listing what blocks a calculation. Lengths take
   the document's units (or an explicit `mm`/`in` suffix), angles are degrees converted with the
   CLI's expression (`deg * (pi / 180)`) so the same typed value gives the same double. Field
   tables are data (one per params struct), so a definition built in code is judged by the same
   tables (`validateParams`: finite, within 1e9 mm / 360000 degrees, counts in range, options valid).
4. **Vector association** (`VectorSelector`): a base `VectorSelection` plus filters (open, closed,
   only circles with an optional diameter and tolerance). It is re-resolved against the document
   whenever the toolpath is calculated or checked, never cached as a list. Objects on other sheets
   than the active one, on hidden or locked layers (or under such a group) never contribute; stale
   ids name nothing. The record stores the base when there are no filters and the resolved objects
   otherwise (ADR-0021 cannot hold the filters). The filters, the V-carve clearance list and the
   enabled flag are session state; a reloaded file gets the selector back as the stored selection
   and V-carve groups are inferred from adjacent records with equal parameters and selection whose
   tools are end mills followed by the carving tool.
5. **Calculation is a pure function** (ADR-0018 types): `makeToolpathJob` copies the contours and
   resolves the tools (as `carvetto-cli` does: tool must exist, machine and material both or
   neither, a pair without data falls back to the tool's default); `ToolpathJob::run()` calls the
   core operation exactly as the CLI and returns records (name, params with the Profile cutting data
   cleared, selection, tool snapshot, tool source, shared moves) plus messages. The records are
   byte-identical to the CLI's, which the tests check on every operation. Messages carry a severity
   (Info: notes such as duplicate holes or vectors left out; Warning: something skipped but a toolpath
   produced; Error: refusals, and every diagnostic when nothing was produced), the core issue, and the
   drawing object / contour index the diagnostic named. `run()` takes an optional cancel flag, checked
   before and after the (synchronous) core call, so cancelling takes effect at once before the run
   and only after the core call returns during it (latency = the rest of that call, bounded by the
   operations' own work limits; seconds for large jobs); out-of-memory and other exceptions become errors.
   Inputs over 2 M spans, coordinates beyond 1e5 mm, more than 1e7 mm of vector length in all, non-finite numbers and names with control
   characters (a name reaches the G-code as a comment) are refused before the operation runs.
6. **Applying** (`ToolpathSession::apply`) is the only step that touches the document. It checks
   that the calculation is still valid (the resolved vectors and the tools hash to what the job was
   made from, the target still exists, the records pass the guards of the core's toolpath commands,
   ids are available, the target's records are still those the job was prepared from), then executes the core commands in ONE macro (never coalescing with a
   neighbour), and rolls back if a command fails anyway. Several calculations may be applied in one
   macro (`applyAll`, recalculate all). A refusal changes nothing, not even the undo history.
   Add/edit/duplicate/delete/reorder/rename/recalculate are each one undo step.
7. **V-carve groups.** A V-carve with clearance tools makes several records (clearance first, the
   carving tool last). A calculation maps its results to existing records by role (clearance tool i,
   or the carving tool): those present are replaced in place (`SetToolpathCommand`, same id and
   position), new ones inserted before the next member, and members without a result removed. Rename
   and duplicate act on the group; recalculating any member recalculates the group.
8. **Staleness without storing it.** The session remembers, per computed `moves` vector (by identity,
   `weak_ptr` with `owner_less`), the definition, the hash of the resolved vectors and the hash of the
   tools it was calculated from (`Baseline`). A record is "needs recalculation" when the vectors it
   machines now, the parameters in the record (ignoring the name) or the tool database values differ
   from its baseline. Because the key is the moves the record holds, undo and redo find the right
   baseline and undoing the change that made a toolpath stale makes it current again; nothing is
   written to the document. Records the session did not make (a loaded file, another command) are
   adopted with the current inputs as baseline. A calculation that changes nothing refreshes the
   baseline in place. Staleness is cached by the undo stack's generation (plus the tool database
   revision, the active sheet and `invalidate()` for changes made without a command). Recalculation
   happens only on request (`recalculate`, `recalculateAll(onlyStale)`); a toolpath whose tool left
   the database, or a clearance toolpath without its carving toolpath, is listed with
   `canRecalculate == false` and a note.
9. **Job summary** (`summary()`): per toolpath and total moves, XY bounds, Z range, length and time
   (`estimateTime` with the toolpath's feed rates, the machine's rapid rate and acceleration set by
   `setMachineSetup` / `setMachine`), tool changes between consecutive enabled toolpaths and the
   number of distinct tools. The core `Machine` has no rapid rate, so the session takes it from the
   caller; unset (0), rapid moves are left out of the time and `rapidRateMissing` says so rather than
   inventing a rate.
10. **Enabled toolpaths** are session state (not in the file, not undoable): the summary counts only
    enabled, computed toolpaths and `enabledToolpaths()` lists what the save dialog should post.

## Consequences
- The Qt dock binds to `list()`, `form()/newForm()`, `add/update/recalculate*` and `summary()` and
  needs no logic of its own; a background thread can run `ToolpathJob::run()`.
- Filters of the selector do not survive a save and reload (the record keeps the resolved objects);
  a future format revision could add them without changing the session.
- A cutting-data override per toolpath is not possible until ADR-0021 stores the data.
- The core operations cannot be interrupted; cancellation is checked around them, and the cost is
  bounded by the operations' own work limits.

## Amendment (verifier round 1)
- **Unsavable toolpaths are refused at apply.** The file writer allows at most one move per byte of the
  compressed file, known only after compression, so after the macro runs (for documents of 2000 or
  more moves) the session asks `saveCarv` itself; a `ResourceLimit` refusal rolls the step back
  (`undoAndDiscard`) and is reported ("cannot be saved"). The rule is not repeated in the session.
- **Stale targets.** A job remembers the target's records at `prepare`; `apply` refuses it when any
  of them has been edited, renamed or recalculated since.
- **Form ranges vs the CLI.** Field bounds follow the core's documented ranges, including the lead
  sweep of 0.1 to 90 degrees; they apply to definitions built in code too (`validateParams`), so a
  params.json with a larger sweep would be refused by the session where the CLI passes it on to the
  core. Offsets/leads/overcuts are additionally bounded to +-1000 mm (decision 4 of the report).
- The "rapid rate not set" case is part of the summary cache key.
