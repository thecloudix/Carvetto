# ADR-0021: Native toolpath records in `.carv` (job.json definitions, binary move caches)

Date: 2026-11-01 · Status: proposed (T-0132) · Supersedes the interim storage of ADR-0019 §1

## Context
ADR-0003 (owner decision) puts toolpath definitions in `job.json` and computed moves in optional
binary `toolpaths/*.bin` members. Until T-0132 nothing implemented it: carvetto-cli (T-0122) kept
each toolpath as an interim JSON member `toolpaths/NNNN.json` (ADR-0019), which the loader counted
against its 2 M-value budget for unknown content, so a file held only about 350,000 moves. T-0132
asks for toolpath records in the document model (undo/redo, integrity), a format version bump
that still reads 1.0 files and migrates the interim members, a bounded and fuzzed binary cache
that holds at least 2,000,000 moves, and the CLI on the native records. The card leaves the
schema, the cache format and its budget open; they are decided here.

## Decision

### 1. Document model
- `ToolpathRecord` (`document/toolpath_record.hpp`): `ToolpathId id`, `name` (display name, also the
  name the post sees), `params` = `std::variant<ProfileParams, PocketParams>` (the operation is the
  variant index), `VectorSelection selection`, `ToolSnapshot tool`, `ToolSource toolSource` (tool,
  machine and material ids in the tool database, for recalculation; may be empty) and
  `SharedMoves moves` = `shared_ptr<const vector<Move>>` (null = not computed). Moves are shared and
  immutable, so the undo history and copies of a 2 M-move record cost a pointer, not 144 MB.
- `ProfileParams::cuttingData` is tool-database input and is not part of a record (it is not saved
  and loads empty); the feeds a toolpath was computed with are in `tool`. `ProfileParams`,
  `PocketParams` and `geometry::Tolerance` gained a defaulted `operator==`.
- The document holds `std::vector<ToolpathRecord>`; position = job order (the order of cutting and
  posting), like layers. Raw mutators `insertToolpath` / `takeToolpath` / `moveToolpath`; toolpath
  ids come from the document's single `IdGenerator` (ADR-0007), so a value names one sheet, layer,
  object or toolpath, and preset ids follow amendment A2.
- Commands: `AddToolpathCommand`, `RemoveToolpathsCommand`, `SetToolpathCommand` (replace a record;
  consecutive edits of one toolpath coalesce) and `MoveToolpathCommand`.
- **Vector selection.** `VectorSelection{ bool all; vector<LayerId> layers; vector<ObjectId> objects }`.
  `all` = every vector object. Otherwise the listed objects (a group stands for its descendant
  vectors) and the vectors on the listed layers; an empty selection selects nothing (so a layer
  name that no longer matches, after migration, does not turn into "everything").
  `Document::selectedVectors` resolves it: listed objects in list order (groups expanded in id
  order), then layer vectors in id order, each once.
- **Deleted references are allowed.** Deleting a vector or layer does not edit the selections that
  name it: ids are never reused, so the reference names nothing until an undo brings the item
  back, and then names it again. No object or layer command had to learn about toolpaths.
- **Integrity** (`checkIntegrity`, so also every load): toolpath ids are range-checked and counted
  in `DuplicateId`; new kind `BadToolpathSelection` for a selection id that was never allocated
  (≥ next id, 0, ≥ 2^62), that belongs to an item of another kind (a layer id listed as an object,
  a toolpath id, …), or `all` combined with explicit layers/objects.

### 2. `job.json` schema (format 1.1)
- Optional root key `toolpaths`: an array of records in job order, each one line per leaf record
  in the canonical writer:
  `{"id", "name", "operation": "profile" | "pocket", "params": {…}, "selection": {"all", "layers",
  "objects"}, "tool": {"id", "machine", "material", "name", "tool_number", "diameter", "feed_rate",
  "plunge_rate", "spindle_speed"}, "moves": {"count", "member"}}`. `moves` is absent when the
  record has no computed moves.
- **Parameters use carvetto-cli's params.json schema** (ADR-0019 §2, owner decision for the CLI):
  the same snake_case keys and nesting mirroring `ProfileParams` / `PocketParams`, lengths in mm
  without a suffix. One difference: params.json gives angles in degrees under `<key>`; job.json
  stores the radians under `<key>_rad` (`ramp.angle_rad`, `leads.angle_rad`, `leads.sweep_rad`,
  `raster_angle_rad`, `tolerance.angle_rad`), because a degree round trip is not exact and save →
  load → save must be byte-identical (ADR-0013 §1). This deviates from ADR-0013 §5's `_mm` suffix
  rule for lengths, so that one set of keys serves both files. params.json's `tool` object becomes
  the record's `tool` (plus the snapshot), and its `layers` names become `selection.layers` ids.
- Every params key is optional on load (the core default stays), like params.json; unknown keys
  nested inside `params`, `selection`, `tool` and `moves` are validated and dropped, unknown keys of
  the record itself are preserved per toolpath id (ADR-0013 §7). An unknown `operation` is an
  error ("written by a newer Carvetto?"), like an unknown object type.
- The tool-database path of a params.json is not kept: it is relative to the params file and
  meaningless later.

### 3. Move cache `toolpaths/NNNN.bin`
- One member per record with computed moves, NNNN = its 1-based position in the job order (so at
  most 9999 such records, `kCarvMaxCachedToolpaths`). The member is always written when the record
  has moves: the loader cannot recompute toolpaths (that needs the tool database and the
  operation code), so dropping the cache would lose them. Templates (`.carvt`) may omit them later.
- Layout (little-endian; doubles as their IEEE bits, so every value round-trips exactly):
  header `"CVTP"`, u32 version 1, u64 move count; then per move a tag byte (bits 0–2 kind; bit 3 a
  feed override follows; bits 4–6 x / y / z equal to the previous end point and not written;
  bit 7 the feed override equals the last one written) followed by the values it calls for:
  1 to 49 bytes per move. A 2 M-move raster-like toolpath is 9.8 bytes per move (19.6 MB, 12.3 MB
  deflated). Fields a kind ignores (centre of a line, seconds of a non-dwell, feed override of a
  rapid or dwell; ADR-0008) are not stored and load as zero/empty. Non-finite values are refused
  by both `saveCarv` (InvalidContent, naming the toolpath) and the loader.
- Deflated at level 6 with the other members, written in name order with them (ADR-0013 §1).
- A cache member that no record names is dropped on load, unread. `CarvExtras::withMember` refuses
  cache names, since `saveCarv` owns them.

### 4. Loader budget for moves
- New absolute limit `LoadLimits::maxMoves` = `kCarvMaxToolpathMoves` = **8,000,000 moves per file**
  (all records, caches and migrated interim members together), about 550 MiB decoded at 72 bytes
  per `Move`. A record's `moves.count` is charged while `job.json` is parsed, so an oversized
  count is refused before any cache is inflated. `saveCarv` refuses to write a document over the
  limit (ResourceLimit with a hint), so no file the loader refuses is ever written; the CLI's
  reload check (ADR-0019 §1) is gone.
- Before inflating a cache its declared size must lie in [16 + count, 16 + 49 · count], so the
  inflate work is bounded by the count, and the count by the budget. The cache is decoded straight
  from the bounded inflate stream (ADR-0013 §9) into moves, never held as bytes; the vector grows
  as moves arrive. The CRC is checked at the end: a CRC failure is reported as `InvalidArchive`,
  otherwise defects are `InvalidContent` naming the member and move.
- Caches are exempt from the 20:1 binary ratio checks (per member and overall), which exist to
  bound bytes held in memory: caches are not held, and a regular toolpath can compress far better
  (300,000 identical moves are 1 byte each). The total-size cap (1 GiB) and member cap (256 MiB)
  still apply. Absolute bounds rather than file-relative ones follow ADR-0013 §9's approach; the
  price is that a small crafted file can make a load allocate up to the move budget (550 MiB).
- At most 10,000 toolpath records per file (`ResourceLimit`), each charged as a record (256 bytes)
  plus its strings and selection ids to the document budget; params text is charged as document
  bytes once decoded.
- Measured (GCC 15, this machine, process CPU time, AC2 test): a 2,000,000-move toolpath (raster
  feeds, ramps, arcs, plunges, retracts) saves in 1.57 s Release / 2.2 s Debug (mostly deflating
  the 19.6 MB cache) and loads in 0.26 s Release / 0.47 s Debug, with 176 MiB peak RSS growth for
  the load (forked child, the ADR-0013 method; the decoded moves alone are 144 MB). The test
  asserts save < 6 s and load < 3 s CPU under NDEBUG and < 400 MiB peak growth on Linux.

### 5. Version and migration
- The format becomes **1.1** (`kCarvFormatMinor` = 1): a minor bump, since `toolpaths` and the
  caches are optional additions. 1.0 files load unchanged. A 1.0 build opening a 1.1 file keeps
  `toolpaths` as an unknown root key and the caches as unknown members (forward compatibility,
  ADR-0013 §7) as long as they fit its unknown-content limits.
- **Interim members** `toolpaths/NNNN.json` (ADR-0019) are converted on load in any version: each is
  streamed through the strict reader (charged to the global value budget and the move budget, not
  to the unknown-content budget) and becomes a record after the file's own, in member-name
  (creation) order, with fresh ids. Its params.json is read with the same key tables in the
  degrees dialect (same arithmetic as the CLI, so the radians are bit-identical to what the CLI
  computed), its `tool.id/machine/material` become `toolSource`, its `layers` names select every
  layer of that name (no `layers`: every vector). The member is not kept: the next save writes the
  record natively. A damaged interim member fails the load with a message naming it.

### 6. carvetto-cli
- `toolpath` adds a record through `AddToolpathCommand` (selection = the named layers' ids, or all
  vectors) and saves normally; `post` and `info` read `Document::toolpaths()`; `post` refuses a
  record without moves. Output text says "as toolpath N" instead of the member name. The CLI
  golden is byte-identical, and a migration test rebuilds an interim file from the native one and
  posts the same program.

## Alternatives
- **Typed params vs. keeping params.json text in the record:** text would keep the CLI's exact
  input but give the app nothing to edit and two sources of truth; typed params are what the
  forms and recalculation need.
- **Degrees in job.json** (params.json verbatim): not exact across save → load → save.
- **ADR-0013 `_mm` suffixes on every length:** consistent with creation params, but two schemas for
  the same parameters; the director asked for one where sensible.
- **Pruning selections when vectors are deleted:** every object/layer command and its undo would
  have to edit toolpaths; dangling-but-never-reused ids give the same behaviour for free.
- **Moves in `job.json` or a JSON member:** the problem T-0132 exists to fix (budget, size, speed).
- **float32 or delta-encoded coordinates:** smaller, but not exact; the same-as-previous flags keep
  exactness and still halve the size of typical toolpaths.
- **A file-relative move budget** (moves per archive byte): tighter against crafted files, but a
  regular toolpath (drilling, repeated passes) can compress so well that a legitimate file would
  be refused.

## Consequences
- The app (Batch B/C) can create, edit, reorder and undo toolpaths and save them; recalculation
  re-resolves `selectedVectors(selection)` and looks the tool up through `toolSource`.
- Adding an operation means a new variant alternative, its params codec (both dialects) and an
  `operation` name; files written then are still 1.x if the old operations keep their schema.
- `ToolpathRecord::operator==` compares moves by content (the AC tests use it).
- Open: the 8 M move budget and the per-file absolute bound are a trade-off the owner may tune.
