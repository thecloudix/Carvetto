# ADR-0021: Native toolpath records in `.carv` (job.json definitions, binary move caches)

Date: 2026-11-01 · Status: proposed (T-0132) · Supersedes the interim storage of ADR-0019 §1 ·
Move budget tied to file size by the project owner (2026-11-02, after verifier round 1) ·
saveCarv verifies its output with the loader (verifier round 2) · Storage residuals (T-0136):
verification without a second decoded copy, clearer limit messages, finite parameters

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
  object or toolpath, and preset ids follow amendment A2. **Toolpaths are a fourth id kind**
  next to sheets, layers and objects (recorded as ADR-0007 amendment A5).
- Commands: `AddToolpathCommand`, `RemoveToolpathsCommand`, `SetToolpathCommand` (replace a record;
  consecutive edits of one toolpath coalesce) and `MoveToolpathCommand`. Add and Set drop repeated
  selection ids (first kept), refuse (no-op) a selection `checkIntegrity` would reject (an
  unallocated id, an item of another kind, `all` with lists), and clear `cuttingData`. `shareMoves` zeroes the fields a move's kind ignores (see §3),
  so a record built through these equals its saved and reloaded copy. Records built by hand and
  inserted raw are not normalised.
- **Vector selection.** `VectorSelection{ bool all; vector<LayerId> layers; vector<ObjectId> objects }`.
  `all` = every vector object. Otherwise the listed objects (a group stands for its descendant
  vectors) and the vectors on the listed layers; an empty selection selects nothing (so a layer
  name that no longer matches, after migration, does not turn into "everything").
  `Document::selectedVectors` resolves it: listed objects in list order (groups expanded in id
  order), then layer vectors in id order, each once. Layer ids made from names (carvetto-cli, and
  migration below) are in one canonical order: names as listed, each name's layers in stacking
  order, each layer once.
- **Deleted references are allowed.** Deleting a vector or layer does not edit the selections that
  name it: ids are never reused, so the reference names nothing until an undo brings the item
  back, and then names it again. No object or layer command had to learn about toolpaths.
- **Integrity** (`checkIntegrity`, so also every load): toolpath ids are range-checked and counted
  in `DuplicateId`; new kind `BadToolpathSelection` for a selection id that was never allocated
  (≥ next id, 0, ≥ 2^62), that belongs to an item of another kind (a layer id listed as an object,
  a toolpath id, …), an id listed twice, or `all` combined with explicit layers/objects. At most
  one such issue is reported per toolpath, so a hostile file cannot make the report huge.

### 2. `job.json` schema (format 1.1)
- Optional root key `toolpaths`: an array of records in job order, each one line per leaf record
  in the canonical writer:
  `{"id", "name", "operation": "profile" | "pocket", "params": {…}, "selection": {"all", "layers",
  "objects"}, "tool": {"id", "machine", "material", "name", "tool_number", "diameter", "feed_rate",
  "plunge_rate", "spindle_speed"}, "moves": {"count", "member"}}`. `moves` is absent when the
  record has no computed moves. `tool` may also hold `"spindle_off": true` and
  `"tool_type": "diamond_drag"` (T-0154, ADR-0020, ADR-0024 §6), both written only for a
  spindle-off snapshot (additive; the format stays 1.1). `spindle_off` is allowed only on engrave
  records whose `tool_type` is `diamond_drag`, and `tool_type` only together with
  `spindle_off`; the loader, `saveCarv` and the toolpath commands refuse anything else
  (`spindleOffProblem`), so a crafted V-bit engrave record marked spindle-off does not load.
- **Parameters use carvetto-cli's params.json schema** (ADR-0019 §2, owner decision for the CLI):
  the same snake_case keys and nesting mirroring `ProfileParams` / `PocketParams`, lengths in mm
  without a suffix. One difference: params.json gives angles in degrees under `<key>`; job.json
  stores the radians under `<key>_rad` (`ramp.angle_rad`, `leads.angle_rad`, `leads.sweep_rad`,
  `raster_angle_rad`, `tolerance.angle_rad`), because a degree round trip is not exact and save →
  load → save must be byte-identical (ADR-0013 §1). This deviates from ADR-0013 §5's `_mm` suffix
  rule for lengths, so that one set of keys serves both files. params.json's `tool` object becomes
  the record's `tool` (plus the snapshot), and its `layers` names become `selection.layers` ids.
- **Parameters are finite numbers (T-0136).** Every number in `params` (lengths, angles,
  tolerances, `start_position`) must be finite. Unlike other `job.json` doubles (ADR-0013 decision 3
  "Numbers", e.g. an empty box is ±infinity) the spellings `"NaN"`, `"Infinity"` and `"-Infinity"` are
  refused there, in both dialects (`job.json.toolpaths[0].params.safe_z: must be finite`,
  `toolpaths/0001.json.params.raster_angle: must be finite`); a NaN parameter also broke
  `record == reloaded copy`. `nonFiniteParam` (`toolpath_record.hpp`) names the first offending
  key in the same job.json spelling; `AddToolpathCommand` / `SetToolpathCommand` refuse such a
  record (no-op) and `saveCarv` refuses it (InvalidContent, `toolpath 'X' (#7): parameter
  leads.angle_rad is not a finite number`). Everything else keeps its T-0132/T-0134 behaviour:
  move values must be finite (the cache cannot hold anything else), a computed record's tool
  diameter and feeds must be finite and positive, and an uncomputed record's snapshot (spindle
  speed included) may still hold any double.
- Every params key is optional on load (the core default stays), like params.json; unknown keys
  nested inside `params`, `selection`, `tool` and `moves` are validated and dropped, unknown keys of
  the record itself are preserved per toolpath id (ADR-0013 §7). An unknown `operation` is an
  error ("written by a newer Carvetto?"), like an unknown object type.
- The tool-database path of a params.json is not kept: it is relative to the params file and
  meaningless later.

### 3. Move cache `toolpaths/NNNN.bin`
- One member per record with computed moves, NNNN = its 1-based position in the job order (so at
  most 9999 such records, `kCarvMaxCachedToolpaths`). Every member counts against the loader's
  limit of 10,000 archive members (ADR-0013 §9): manifest + job + caches + preserved members, so
  without preserved members at most **9,998** computed toolpaths fit, one fewer per preserved
  member. `saveCarv` counts them all and refuses (ResourceLimit) before writing; it also checks
  the member-size (256 MiB) and total-size (1 GiB) caps on what it is about to write.
- **saveCarv verifies its output (verifier round 2).** Hand-written pre-checks drifted from the
  loader twice (member count, then the 10,000-record cap, repeated selection ids, string caps,
  JSON ratio, interim extras). So the finished archive now goes through the real load path
  (`loadImpl` with the default `LoadLimits` / `ZipLimits`) before `saveCarv` returns it; any
  failure is returned with the loader's own code and message, prefixed "the file would not load
  again:", and nothing is written. The promise "saveCarv never writes a file loadCarv refuses"
  is therefore structural. Cost: one in-memory load (0.17–0.18 s Release for the AC2 job's 2 M
  moves); measured save times went from 1.36–1.43 s to 1.39–1.47 s Release (deflate dominates and
  varies), 2.3 s Debug. The pre-checks stay for clearer messages.
- **No second decoded copy (T-0136).** Until T-0136 the verification load decoded every move again
  (72 bytes each: +144 MB at 2 M moves, about +0.5 GiB at the 8 M cap on top of the encoded caches
  and the archive). The verification now runs the loader with its caches in *validate-only* mode
  (`detail::validateMoves`): each cache is inflated and checked move by move with exactly the
  decoder's checks and messages (same code path, a test compares the two on 3,000 mutated caches),
  but no move is kept; the record gets an empty move list, which is enough for the checks that
  look only at whether a record is computed. The uncompressed caches are also freed once the
  archive is written, before the check. The invariant is unchanged: everything the loader checks,
  including every move, is checked on the finished bytes.
  Measured (GCC 15 Release, this machine, peak RSS growth of `saveCarv` in a forked child, harness
  t0132v3/big3.cpp with a valid tool; before → after): raster 2 M moves 184 → 60 MiB, noisy 2 M
  199 → 90 MiB, raster 8 M (46 MB file) 724 → 218 MiB, noisy 8 M (115 MB file) 790 → 350 MiB.
  What remains is the encoded caches (1–49 bytes per move, ~25 for noisy coordinates) and the
  compressed archive, both needed to write the file; save CPU is unchanged within noise (2.1 s
  Release for raster 2 M). The test asserts < 36 bytes per move for 1 M moves on Linux (was 98 MiB
  for 1 M moves in Debug before).
- **Limit messages (T-0136).** Too many archive members names how many computed toolpaths fit:
  `the file would have 10001 archive members (9999 toolpath move caches among them); a .carv file
  holds at most 10000 members including manifest.json and job.json, so at most 9998 toolpaths with
  computed moves fit (clear the moves of some toolpaths, or split the job)` (with preserved
  members: `… fit next to the 1 preserved members`). It is checked before the position rule, and a
  computed toolpath past position 9,999 is named with its position (`toolpath 'Late' (#12) is at
  position 10000 of the job and has computed moves; only toolpaths at positions 1 to 9999 can keep
  computed moves (move it up the job order, or clear its moves)`); the loader says the same for a
  crafted file (`the toolpath at position 10000 (#…) has computed moves; …`).
- **Over-long names (T-0136).** A string over the loader's 1 MiB cap is refused by the loader
  without a field name, and a long repetitive string makes `job.json` compress beyond the 200:1
  ratio check, which then fires first ("a decompression bomb?"). When the verification fails with
  either of these job.json errors, `saveCarv` looks for the longest string (value or key) in the
  `job.json` it wrote and reports it with its path: over 1 MiB, `the file would not load again:
  name too long: job.json.toolpaths[0].name holds 1100000 bytes; a name or text holds at most
  1048576`; under it but at least a quarter of `job.json` with a ratio failure, `name too long:
  job.json.toolpaths[0].name holds 700000 bytes of repetitive text, so job.json would expand more
  than 200:1 and be refused as a possible decompression bomb`. Otherwise the loader's message is
  kept. Known gap: an unknown field preserved from a newer file may hold a string up to 16 MiB;
  if such a string is the longest when a *known* string exceeds 1 MiB, the message names the
  unknown one (the refusal itself is right).
- `CarvExtras::withMember` also refuses interim names `toolpaths/NNNN.json`: records replace them,
  and a member added that way would turn into an extra record on the next load. The member is always written when the record
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
- **Positive tool sizes, feeds and dwells (T-0134).** A dwell's seconds and a feed override must
  also be greater than zero (0, -0.0 and negative values are refused). A record **with computed
  moves** (job.json `moves` present, i.e. a cache, even one of zero moves) must also have a tool
  `diameter`, `feed_rate` and `plunge_rate` that are finite and > 0, because those moves go to
  the post. A record **without computed moves** keeps T-0132's behaviour: any snapshot, the
  default all-zero one included, saves and loads (it is filled in when the toolpath is
  calculated). Interim members always carry moves, so their tool and moves are checked on
  migration. `saveCarv` refuses an offending computed record before writing (InvalidContent,
  naming the toolpath; the loader check behind it would refuse it anyway), and the loader
  refuses it with the member and move, e.g.
  `toolpaths/0001.bin: move 1: dwell seconds must be greater than zero` or
  `job.json.toolpaths[0].tool.diameter: must be a finite number greater than zero for a toolpath
  with computed moves`. Moves in cache messages are numbered from 1, as in the post's errors
  (JSON paths of interim members keep their 0-based array indices).
- Deflated at level 6 with the other members, written in name order with them (ADR-0013 §1).
- A cache member that no record names is dropped on load, unread. `CarvExtras::withMember` refuses
  cache names, since `saveCarv` owns them.

### 4. Loader budget for moves (owner decision 2026-11-02: tied to file size)
- A file of S bytes may hold at most **min(S × 1, 8,000,000) toolpath moves**
  (`kCarvMovesPerFileByte` = 1, `kCarvMaxToolpathMoves` = 8,000,000; `LoadLimits::maxMoves` and
  `movesPerArchiveByte`), counting all records, caches and migrated interim members together.
  Real toolpaths take 6–9 bytes of file per move (the AC2 job: 6.2), so only crafted or
  degenerate, highly repetitive toolpaths reach the rule. A record's `moves.count` is charged while
  `job.json` is parsed, so an oversized count is refused before any cache is inflated (the
  records before it in `job.json`, objects included, are parsed first, within the document
  budget): a 9 KB file can claim about 9,000 moves.
- `saveCarv` applies the same rule to the compressed archive it is about to return (it knows the
  size only then) and refuses (ResourceLimit, with a hint) instead of writing, so no file the
  loader refuses is written; the CLI's reload check (ADR-0019 §1) is gone. A highly repetitive
  toolpath (e.g. 1.2 M identical moves) is refused on its own and saves once the file holds enough
  other content. A file that loads can still be refused on re-save if re-saving makes it smaller
  (stale caches dropped, padding members removed, canonical re-encoding) so that it no longer
  holds one byte per move; the refusal is clean (no file written).
- The decoder reserves exactly the declared count (no power-of-two growth), so a load's memory for
  moves is at most 72 bytes per byte of the file: measured (Release, forked child) 1.6 MiB peak
  growth for a 10.5 KB file at the limit, 71 MiB for 1.0 MB and 558 MiB for 7.9 MB (padded files
  of one-byte moves, the worst case per byte); 138 MiB for a realistic 2 M-move 18 MB file, and
  550 MiB for 8 M moves in 73 MB. Migrated interim members have no declared count, so each is
  read twice: a counting pass under a throwaway copy of the load budget (T-0136: a copy, not a
  fresh budget, so it stops where the shared budget would and the real pass never reserves moves
  the shared budget then refuses; before, a member could reserve up to min(S, 8 M) moves first,
  untouched pages on Linux but commit charge on Windows), then the real pass
  with exactly that many moves reserved (a realistic 400,000-move interim member in a 7.7 MB file:
  0.45 s Release for both passes, 28 MiB peak, i.e. 72 bytes per move). Interim files from the
  T-0122 CLI held at most ~350,000 moves, so the second pass is cheap.
- Before inflating a cache its declared size must lie in [16 + count, 16 + 49 · count + 48], so
  the inflate work is bounded by the count, and the count by the budget. The cache is decoded
  straight from the bounded inflate stream (ADR-0013 §9) into moves, never held as bytes. The CRC is checked at the end: a CRC failure is reported as `InvalidArchive`,
  otherwise defects are `InvalidContent` naming the member and move.
- Caches are exempt from the 20:1 binary ratio checks (per member and overall), which exist to
  bound bytes held in memory: caches are not held, and a regular toolpath can compress far better
  (300,000 identical moves are 1 byte each). The total-size cap (1 GiB) and member cap (256 MiB)
  still apply; the move-per-byte rule bounds what a cache can decode to.
- At most 10,000 toolpath records per file (`ResourceLimit`), each charged as a record (256 bytes)
  plus its strings and selection ids to the document budget; params text is charged as document
  bytes once decoded.
- Measured (GCC 15, this machine, process CPU time, AC2 test): a 2,000,000-move toolpath (raster
  feeds, ramps, arcs, plunges, retracts; 12.3 MB file) saves in about 1.4 s Release / 2 s Debug
  (mostly deflating the 19.6 MB cache) and loads in about 0.23 s Release / 0.35 s Debug, with
  141 MiB peak RSS growth for the load (forked child, the ADR-0013 method; the decoded moves alone
  are 144 MB). The test asserts save < 6 s and load < 3 s CPU under NDEBUG and < 400 MiB peak
  growth on Linux.

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
  layer of that name in the canonical order of §1 (no `layers`: every vector). The member is not kept: the next save writes the
  record natively. A damaged interim member fails the load with a message naming it.

- **1.2 (T-0211, ADR-0033 amendment A1)** adds text parameters and embedded fonts; a document
  without text objects is still written as 1.1.

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
- **An absolute move budget only** (round 1 of T-0132): a 9 KB crafted file could make a load
  allocate 581 MiB; the owner chose the per-byte rule, accepting that a degenerate repetitive
  toolpath alone in a file is refused on save.

## Consequences
- The app (Batch B/C) can create, edit, reorder and undo toolpaths and save them; recalculation
  re-resolves `selectedVectors(selection)` and looks the tool up through `toolSource`.
- Adding an operation means a new variant alternative, its params codec (both dialects) and an
  `operation` name; files written then are still 1.x if the old operations keep their schema.
- `ToolpathRecord::operator==` compares moves by content (the AC tests use it).
- **Known limitation for the app:** a record holds one `ToolSnapshot`, so a multi-tool pocket
  (clearance + rest tools) is one record per tool, each with its own moves; it cannot be one
  recalculable record. A later card can widen `ToolSource`/the snapshot to a tool list.
- Round trips drop what the format does not store: unknown keys nested in params/selection/tool,
  `cuttingData`, and move fields a kind ignores. Records made through the commands and
  `shareMoves` are normalised so `==` still holds.
