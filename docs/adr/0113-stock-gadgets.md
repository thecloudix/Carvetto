# ADR-0113: the stock gadgets (T-0332)

Status: accepted (implementation). Builds on ADR-0108 (scripting API and runner). Behaviour comes only from
`docs/reference/feature-inventory.md` section 6 and the Gadgets menu list in `ui-observations.md`; the
scripts are our own (clean room).

## Context
T-0331 delivered the runner, the manifest format and the Gadgets menu. T-0332 ships the gadgets the
inventory lists as preinstalled: Celtic Weave Creator, Drag Knife Toolpath, Keyhole Toolpath, Job Setup
Sheet Editor and the DXF Batch Processor, as Lua scripts, with an install path from the application.
The user documentation is `docs/gadgets/stock-gadgets.md`.

## Decisions
1. **Shipped as gadgets, not as code.** Each gadget is a folder `gadgets/stock/<name>/` with `manifest.json`
   and `gadget.lua`, exactly what a user could install. Nothing in `core` knows about a particular gadget.
2. **Install path.** `core/scripting::installShippedGadgets(shippedRoot, userRoot)` installs a gadget that is
   missing, replaces one whose manifest `version` differs, and leaves one at the same version alone (so a
   user's edit survives until the next release; to keep an edit across releases, install it under another
   name). A user gadget that sits in the folder a shipped gadget would use (same derived folder name,
   different manifest name) is never overwritten, it is reported. The app does this when settings
   persistence is switched on (real runs, not the self-test), so the gadgets are in the menu from the first
   start. The shipped folder is `<exe>/gadgets`, `<exe>/../share/carvetto/gadgets` or, in a development
   build, the source tree (the same candidate order as the post-processors). A user who deletes a stock
   gadget gets it back at the next start; a "do not reinstall" marker is not built.
3. **Forms are the only interaction.** ADR-0108's form has no custom widgets, so the weave creator's
   "click an intersection to cycle Cut Vertical / Cut Horizontal / Cut Neither" is a pattern chooser
   (Alternating, all one state, or a Custom text grid of V / H / N letters, one row per line separated by
   `/`, top row first). The pattern semantics (what V, H, N mean) follow the inventory.
4. **File access, the owner question of ADR-0102 (question 2), answered conservatively as proposed there.**
   Two additions to the API (API version stays 1, they only add):
   * `carvetto.ui.choose_folder(title)` asks the user for a folder (service `GadgetUi::chooseFolder`) and
     returns its path, or nil if cancelled. It counts as a dialog (cap, abort button, wait budget).
   * `carvetto.file.read_dxf_folder(path)` reads the `.dxf` files of a folder **that the user chose with
     `choose_folder` in this run** (any other path is an error: no `..`, no absolute paths of the
     script's own choosing). It does not recurse, skips symbolic links and non-regular files, reads at
     most `GadgetLimits::maxDxfFiles` (200) files of at most `maxDxfFileBytes` (16 MiB) each, hands over at
     most `maxDxfContours` (50 000) contours in total, and caps the importer's own budgets (200 000 spans,
     1 000 texts, 20 000 block instances). A file that cannot be read or parsed comes back as an entry with
     an `error` text (it does not abort the others). The result per file is `{name, file, contours,
     warnings, error?}` with contours in the form of `object.contours`, in millimetres. Writing files is not
     offered. This is a product decision the owner can reverse: if broader access is wanted, it is a change
     in this one function.
5. **Custom toolpaths for the drag knife and keyhole.** Both use `toolpath.create_custom` (ADR-0108): the
   record stores the computed moves, so they survive a .carv round trip, but a recalculation would run an
   engrave operation on an empty selection (open question 5 of ADR-0108, a real "custom" operation, is
   still open). The drag knife flattens arcs and Beziers to 5-degree steps and follows the vector's own
   start points unless told otherwise.
6. **Job sheet editor stores its choices as document variables** (`jobsheet_header_rgb`,
   `jobsheet_footer_rgb`, `jobsheet_show_layout|notes|material|summary|toolpaths`). They are numbers
   because a document variable is a number: a colour is 0xRRGGBB (0 = none; black is stored as 1), a
   section flag is 1 or 0. The job sheet (`viewmodel::makeJobSheet`, `JobSheetStyle`) reads them; absent
   variables, or "Restore default", print the sheet byte for byte as before. Colours dress the HTML header
   and add a footer band (digits only: nothing a gadget supplies reaches the markup); sections apply to
   HTML and plain text. Variables travel with the .carv file, so the sheet looks the same on another
   machine. Not built, and reported to the owner: the **logo** (an image file the sheet would embed:
   needs a read-file service with image validation, like the clipart browser's) and a **notes** section
   (the sheet has none; the flag is stored for it); the **title** is the job's name, which the sheet
   already uses.
7. **DXF Batch Processor semantics.** The inventory names the gadget only; no behaviour is documented. We
   define: every DXF in the chosen folder is imported onto its own layer named after the file, parts are
   laid out left to right with a gap (each part's lower-left corner on the base line), and optionally each
   part gets an outside profile toolpath. Unreadable files are listed at the end. Choosing a different
   behaviour (convert to another format, nest) is a product decision for the owner.

## Consequences
* The measured needs of the stock gadgets (ADR-0102 question 3): the largest weave the form allows
  (30 x 30 crossings, 961 strand pieces) runs within the default script limits (test "the largest grid
  runs within the default script limits"); the other gadgets are small. No manifest limit hints are needed
  yet.
* One more core dependency of `scripting` on `io` (the DXF importer), and a filesystem read in the API,
  guarded as above.

## Mutants considered
Applied by hand to the Lua scripts, test run, seen failing, reverted: weave gap half-width (`w/2` to `w`);
alternating pattern flipped (this one SURVIVED the first tests, because a square grid has the same piece
counts either way; the 1 x 1 alternating test now checks which strand is cut); swivel arc direction
flipped; swivel arc centre moved off the corner; lead-in removed; passes ignored; `use_start` ignored;
tolerance angle scaled; keyhole direction read as radians; job-sheet section flag read inverted; DXF
batch gap removed; DXF batch part origin not shifted. NOT applied (C++ changes need a rebuild of the
whole test set; covered by the named boundary tests but not mutated): the chosen-folder check, the
symlink skip, the file and contour cap comparisons, `installShippedGadgets` replacing at the same
version or overwriting a foreign gadget, the job-sheet colour mask.
