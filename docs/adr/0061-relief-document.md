# ADR-0061: Relief in the document and in `.carv`

Status: accepted (T-0282)

## Decisions
- **Model.** `Document` owns a flat list of `ReliefLevel` (`document/relief_model.hpp`); each level
  names its `SheetId`, so relief is per sheet and the levels of one sheet keep their relative order
  (bottom first). A level holds `ReliefComponent` = `ComponentId` + the core `relief::Component`
  (ADR-0060). `Document::reliefModel(sheet)` copies a sheet's levels into the `relief::Level` list
  that `relief::compose` takes. `LevelId` and `ComponentId` are new `Id<Tag>` kinds drawn from the
  document's single id generator, so ids are never reused, never collide with objects, layers,
  sheets, toolpaths or zones, and `checkIntegrity()` covers them (range, below next id, duplicates,
  level on a missing sheet, component with an empty field or unstorable properties).
  A toolpath selection naming a relief id is reported by the existing toolpath-selection check.
- **Selection.** Components are selected in `Document::componentSelection()` (`ComponentSelection`,
  ordered, no duplicates, not undoable), separate from the object selection because the id kinds
  differ. Raw removal deselects (as for objects); moving or reordering keeps the selection. A stale
  id is an integrity issue.
- **Commands** (`relief_commands.hpp`): add level, add component, delete components, delete level,
  rename component/level, set component combine mode, set component properties (shape height,
  base height, fade, tilt, visible), set level properties (mode, base height, visible), reorder
  component (within its level), reorder level (among its sheet's levels), move component to
  another level, duplicate component. Ids are allocated on the first apply and kept for redo;
  every command is a no-op for unknown ids, unchanged values and violated caps; property edits of
  one target coalesce. A component's field moves in and out of the command on apply/revert rather
  than being copied, so undo history holds each field once. Duplicate names the copy
  "<name> copy" (truncated to the name cap) and places it directly above the original. Moving a
  component to a level on a different sheet is refused (field coordinates belong to a sheet's
  design space); a sheet that still has relief levels cannot be removed (as for objects).
- **Caps** (commands and loader): 256 levels, 1024 components, names of at most 256 bytes (non-empty
  through commands), and 2^26 cells over all fields of a document (256 MiB) on top of the 2^25 per
  field of ADR-0060. Properties must be finite even while a fade or tilt is disabled (so a JSON
  file never needs a NaN) and an enabled fade/tilt needs a real direction (`relief::validateProperties`,
  newly public).
- **Format 1.9.** `job.json` gets an optional `relief` array of levels (`id`, `sheet`, `name`,
  `mode`, `base_height_mm`, `visible`, `components`); a component record holds `id`, `name`, `mode`,
  `shape_height_percent`, `base_height_mm`, `visible`, `fade` and `tilt` objects. The minor is 9
  only when a relief level exists, so every other document keeps its bytes and older 1.x readers
  are unaffected; old files load unchanged. Unknown keys inside relief records are skipped, not
  preserved (same as keep-out zones); unknown *members* keep the existing extras mechanism.
- **`components/<id>.hf`.** One member per component, named by its decimal id (canonical, 1 to
  2^62-1; other `components/...` names stay ordinary preserved members). Layout: 40-byte header
  (`CVHF`, version 1, width, height, cell size, origin) then float32 cells in byte-planar order
  (all low bytes, then the next ...). Planar order groups the near-constant exponent bytes so the
  archive's deflate packs real terrain well and flat fields extremely well; the card allowed zstd
  or the existing helper, and the existing helper (the zip's deflate) is used, so no new
  dependency. Every NaN is written as 0x7FC00000, infinities are refused, so equal documents give
  equal bytes (save -> load -> save is byte-identical).
- **Hostile input.** `.hf` members are exempt from the zip ratio checks (a flat field is ~1000:1)
  and bounded instead by the cell caps, exactly like toolpath move caches by the move budget.
  Before any member is inflated the loader sums the declared sizes of the fields job.json names and
  refuses more than 2^26 cells' worth; the decoder reads the 40-byte header, rejects zero/oversized
  dimensions (checked in 64-bit so 2^32-1 x 2^32-1 cannot wrap), a non-finite or non-positive
  cell size or origin, and a declared size different from `40 + 4*w*h`, all before allocating
  `w*h` floats; so allocation never exceeds the declared member size, itself within the caps. The
  cells stream through a 64 KiB buffer (no second copy of the member). A member no component names
  is dropped unread. A missing member, bad signature/version, truncation or an infinite cell is
  `InvalidContent`/`MissingMember` with the member name in the message; a CRC mismatch is
  `InvalidArchive`.
- **Save.** Checks every level, name, property and the total cell cap before encoding any field,
  so an over-cap document is refused cheaply; the existing verification load then proves the file
  loads again.

## Tests and mutants
Mutants run by hand against `[relief]` (each killed): zip ratio exemption removed; header size
check removed; cell cap loosened; sheet-with-relief removal guard; reorder rotate direction; move to
another sheet allowed; stale member decoded; NaN not canonicalised; delete-level restore index;
total-cell cap `<` instead of `<=`; component and level count caps off by one; selection not
preserved on level reorder; infinite cell accepted on load.
Not killable cheaply and argued instead: the loader's two total-cell checks (pre-inflate sum and
post-decode sum) need more than 256 MiB of member data to trip, which would make the suite heavy on
shared machines; they use the same constant that the command tests hit exactly (fill to 2^26,
refuse +1) and that `saveCarv` enforces before encoding (tested with +1 cell), and the per-field
boundary (2^25 cells) is round-tripped through save and load in a `[large]` test. `IdNotBelowNext`
for relief ids cannot arise through the raw inserts, which reserve the generator.
