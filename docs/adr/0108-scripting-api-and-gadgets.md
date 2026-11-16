# ADR-0108: the scripting API, the gadget runner and the Gadgets menu (T-0331)

Status: accepted (implementation). It builds the API proposed in ADR-0102 on the host of ADR-0101. The
product questions of ADR-0102 are still open for the owner (listed at the end, unchanged and extended).

## Context
T-0330 delivered a sandboxed Lua host. T-0331 binds the designed `carvetto` API to the document, makes a
gadget run one undo step, and adds the app side: gadget discovery, the Gadgets menu, Install New Gadget,
Gadget Shortcuts and an error dialog. Clean room: the API is our own (ADR-0102); nothing here follows a
vendor SDK.

## Decisions

1. **Layers.** `core/scripting`: `gadget_api` (the binding and `runGadget`, no Qt) and `gadget_manifest`
   (manifest parsing, discovery, install; filesystem only). `app/viewmodel`: `GadgetManager` (menu model,
   shortcuts, running with the toolpath session, failure wording; no Qt). `app/src`: `GadgetsMenu`, the
   form, shortcuts and error dialogs (Qt glue only).
2. **Services, not dependencies.** Toolpath calculation lives in the viewmodel (`ToolpathSession`), which
   core must not depend on. `runGadget` therefore takes `GadgetServices` (forms, messages, tool list,
   `createToolpath`, `toolSnapshot`); the viewmodel supplies the real ones (`makeGadgetServices`), tests
   supply scripted ones. A missing service is a script error ("no user interface available").
3. **One run = one undo step, all-or-nothing.** `runGadget` opens an `UndoStack` macro, every edit is a
   normal command (`AddObjectCommand`, `TransformObjectsCommand`, `SetDocumentVariableCommand`,
   `AddLayerCommand`, `AddToolpathCommand`, ...), and on success `endMacro` records ONE step named after
   the gadget. On ANY failure (script error, instruction/memory/output limit, Timeout, cancel, a refused
   edit) the macro is abandoned with the new `UndoStack::abortMacro(doc)`, which reverts the collected
   commands newest first and records nothing, so history, redo tail and clean point are exactly as before
   (it bumps the edit generation so derived caches refresh). The selection (not undoable) is restored too.
   `ToolpathSession::add` is used unchanged for toolpaths: its own macro nests inside the gadget's.
   A gadget cannot start while another macro is open (error, nothing runs).
4. **`ScriptValue::Map`.** ADR-0101 item 9 reserved a string-keyed kind. A table whose keys are all strings
   (no sequence part) crosses as a `Map` (entries sorted by key, so conversion is deterministic); mixed or
   non-string keys are still refused; `{}` stays an empty array. Cost accounting and depth/element/byte
   caps apply as for arrays (a key costs like a string, an entry like an array entry). `registerFunction`
   accepts a dotted module path ("carvetto.doc") and nests tables.
5. **Host-callback failures for oversized values** (T-0330 verifier): while pushing a host function's
   results, an out-of-memory error from the allocator caps (string above `maxStringBytes`, block above
   `maxAllocationBytes`, heap at its limit) escaped as an opaque C++ exception and was reported as "host
   function threw an unknown exception". The push phase now reports "value too large (the result does not
   fit the script's memory limits)"; a catch-all outside the push phase keeps the old text.
6. **The deadline does not run while the user or the CPU-heavy host is busy.** The injectable clock of
   `ScriptLimits` is wrapped so time inside a form, a message, a confirmation or a toolpath calculation is
   subtracted. Otherwise a user reading a dialog for six seconds, or a big pocket, would time the gadget
   out and roll everything back. The instruction, string and value budgets still bound what the script
   itself does between host calls, and `GadgetLimits` caps what a script can ask the host for per run
   (objects 100 000 including layers, toolpaths 64, custom moves 500 000, log lines and bytes).
7. **Failure wording.** `describeFailure` gives each status a title and a sentence a user can act on, always
   "Your drawing was not changed". A Timeout is "took too long on this machine" and says it is not
   necessarily a mistake in the gadget (ADR-0102 question 4, as proposed there). The script's own message
   (chunk, line) and traceback go under "Show Details".
8. **Gadget packaging.** A gadget is a folder with `manifest.json` and `gadget.lua` (fixed names). The
   manifest: `name` and `api_version` required; `version`, `menu` ("Group/Sub/Item", 1-4 parts, default
   the name), `shortcut`, `description` optional; unknown keys ignored. Discovery scans the immediate
   subfolders of the user gadgets folder (`<AppData>/gadgets`), sorted by folder name, skipping hidden
   folders and symbolic links; an unusable manifest is reported in `skipped`, a gadget needing a newer API
   or lacking its script is listed disabled with the reason. The menu is built at window creation and
   again after an install or shortcut change; submenus first, entries by label.
9. **Shortcuts** are `Ctrl+F1..F12`, optionally with Shift (24 slots; plain F-keys, letters and Alt are
   refused so gadgets cannot take the application's own keys). The first gadget in folder order keeps a
   contested manifest shortcut, the others get a note. The Gadget Shortcuts dialog lets the user assign
   any free key per gadget (a taken key is refused inline); the choices are stored by folder name in the
   user settings (real runs only; the self-test never touches settings) and win over the manifest.
10. **Install New Gadget** takes a folder (folder picker), shows the manifest, the menu path, the shortcut
    and a trust warning, then copies ONLY `manifest.json` and `gadget.lua` into a new folder named after
    the gadget (sanitised to `[a-z0-9-]`, so a name cannot escape the gadgets folder) via a temporary
    folder and a rename; an installed gadget is replaced only after a second confirmation.
11. **Toolpaths.** `toolpath.create(kind, objects, params)`: kinds `profile`, `pocket`, `drill`, `engrave`.
    `params` uses the field and choice ids of the operation's own form (`cutDepth`, `passDepth`,
    `startDepth`, `safeZ`, `allowance`, `side`, `direction`, `strategy`, ... see `toolpath_form.hpp`) plus
    `tool` (a tool id) and `name`; millimetres and degrees, whatever the job's display units. Choices take
    the option text case-insensitively, either whole or as one part ("Outside / right" is `"outside"`,
    `"right"` or `"outsideright"`), booleans take `true`/`false`. The call builds a `ToolpathForm`, so
    every validation of the UI applies, an unknown parameter is an error listing the known ones, and
    `ToolpathSession::add` calculates and applies it.

## Deviations from ADR-0102
* **`carvetto.object.create_shape` kinds and parameter names** are fixed here: `rect` (`cx cy width height
  corner corner_size`), `circle` (`cx cy radius`), `ellipse` (`cx cy width height angle`), `polygon`
  (`cx cy sides radius angle`), `star` (`cx cy points outer inner angle`); angles are radians in the API.
  The ADR's "..." (other shapes) is not built; text, bitmaps and groups are readable (`kind`, `bounds`,
  `name`, `layer`) but cannot be created.
* **Added** beyond the ADR text, because gadgets need them: `layer.create`, `layer.set`, `layer.locked`,
  `doc.active_layer`, `doc.variables`, `object.set_layer`, `object.name`, `object.layer`, `toolpath`
  creation from the selection when `objects` is nil, `tool.list` entries as tables.
* **`object.contours`** returns, per contour, `{points = {{x, y}, ...}, bulges = {...}, closed = bool}`
  (the ADR's `{points=..., closed=...}` plus the per-vertex bulge it mentioned); arcs become bulges
  (`tan(sweep / 4)`, positive counter-clockwise; an arc over half a turn is cut into equal pieces so no
  bulge is infinite) and Beziers are flattened at the default chordal tolerance. `create_contour` takes
  the same bulges in a fourth argument.
* **Edits on a locked layer are refused** (`create_*`, `set_layer`); hidden layers are fine.
* **`toolpath.create_custom` stores an engrave-operation record** with an empty vector selection and the
  computed moves: the record format (`ToolpathParams`) has no "custom" operation. Recalculation of such
  a toolpath would run the engrave operation on an empty selection; the toolpath panel lists it as a
  normal record. A real `Custom` operation needs a record-format change and is a product decision (open
  question 5).
* **`ui.open_file` / `ui.save_file` are not built**: the ADR marks them as "may"; they belong with open
  question 2 (file access for the DXF batch gadget, T-0332).
* **Install takes a folder only**, not a zip: unpacking an archive from the internet needs its own review
  (path traversal, size and count caps); left for a follow-up card.
* **Gadget limits cannot be raised per gadget** (the ADR's "confirmation dialog"): the app runs every
  gadget with the host's default limits. A manifest hint and a confirmation dialog wait for the measured
  needs of the stock gadgets (question 3).
* **One gadgets folder** (the user's). A folder of shipped gadgets beside the application is T-0332's
  business (the manager takes one folder; two would be a small change).

## Mutants considered
New logic was attacked with the mutants below; each is killed by a named test unless argued. Fifteen of
them were also applied to the source by hand and the tests re-run (none committed): paused time ignored,
pause not accumulated, object cap `>=`, toolpath cap `>=` (in `create` and in `create_custom`), bulge side
flipped, abort replaced by commit, selection not restored, rotation centre at the origin, `sy` default 1,
form maximum exclusive, locked layers allowed, log byte cap removed, feed `< 0` instead of `<= 0`, dwell
seconds without the lower bound. All were killed; one (the toolpath cap in `create`) first SURVIVED a
test that only reached the cap through `create_custom`, and led to the stronger "caps are exact" test above. A third reading, the paused-clock term for a pause
in progress, was removed rather than tested: the script thread is the only reader of the clock and it
sits inside the host callback during a pause, so no read can observe it.
* `now()` ignoring the paused time, or `Pause` not accumulating: killed by "time spent in a dialog does
  not count" (an hour in a dialog must not time out) and by "but time between dialogs does".
* `abortMacro` not reverting / reverting in forward order / recording the macro: killed by the undo-stack
  test (variables gone, count unchanged, redo tail kept) and by every rollback case of the gadget tests
  (`State` equality over objects, layers, toolpaths, steps, redo, selection, variables).
* A failure path that forgets to restore the selection, or to reset `createdObjects`: killed by the
  `State` comparison (selection) and by `r.createdObjects.empty()`.
* Limit comparisons `>` vs `>=` (objects, toolpaths, form number limits, log bytes): killed by the
  "per-run caps are exact" and "limits of a number field are inclusive" tests, which test the value ON
  the limit and one past it.
* Bulge geometry (sign of the perpendicular, `h = (c/2) / tan(theta/2)`, direction by sign, piece count
  and `tan(sweep/(4n))`): killed by the bulge test (apex 2.5 below for +0.5 and above for -0.5, the
  253.7 degree arc read back as two pieces with bulge -0.618, apex 10 above for bulge -2) and the D shape.
* Default rotation/scale centre and the `sy = sx` default: killed by the modify test (a square rotated
  about its own centre stays put; one scale factor scales both axes).
* Unknown-parameter rejection (`Params::finish`), unknown field/move kinds, wrong types: killed by the
  "bad arguments" and "bad specifications" lists, one case each.
* Manifest: each rejection rule has a case; shortcut normalisation has cases at every boundary (F0, F13,
  F01, Alt, plain F, duplicated modifiers); conflict resolution, unrunnable gadgets owning no shortcut,
  folder-name sanitising and replace/duplicate install each have a test.
* Menu model: grouping, submenus-before-items, case-insensitive order, the disabled entry and its
  tooltip are asserted in the viewmodel test and again in the offscreen self-test.
* Argued, not killed: the exact wording of messages other than the ones tests match on; the
  `std::min(size, 1 << 20)` pre-size of a map table (a performance hint, no behaviour); the order of
  `lua_remove`/`lua_getfield` in the dotted-module walk is covered by a two-level test but a
  mutant that only changes stack slack is unobservable.
* Not testable here: the QSettings persistence of shortcut choices (the self-test never persists, by
  design); the viewmodel round trip `overrides()` -> `setOverrides()` -> `rescan()` is tested instead.

## Open questions for the owner (ADR-0102, unchanged, plus new ones)
1. API compatibility with existing community scripts: yes (adapter later) or no (this design only)?
2. Should gadgets be able to read and write files without a user prompt (a DXF batch converter wants to)?
   Proposed: only inside a folder the user chose in the form (`ui.open_file` / `save_file` /
   `ui.choose_folder`, not built yet).
3. Default budget (50 M instructions, 64 MiB heap): enough for a weave creator on a large grid? Proposed to
   measure with the stock gadgets (T-0332) and set per-gadget manifest hints plus a confirmation.
4. Wall-clock deadline (5 s default): a gadget stopped by it is reported as "took too long on this
   machine". Proposed: accept; a manifest may later ask for a longer deadline.
5. (new) A custom toolpath (drag knife, keyhole) needs a record that is not "recalculable from vectors":
   add a `Custom` operation to the record format (a `.carv` format change) or keep the engrave stand-in?
6. (new) Install from a zip (and from a URL) is not built: wanted, and with which size and file-count caps?
7. (new) Should gadgets be able to run on a selection of toolpaths and edit existing ones
   (`toolpath.set`/`delete`)? Not built; read-only for now.
8. (new) Where do shipped gadgets live (next to the application) and may a user override one by installing
   a gadget of the same name?
