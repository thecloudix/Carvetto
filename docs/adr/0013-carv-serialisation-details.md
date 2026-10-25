# ADR-0013: `.carv` serialisation details

Date: 2026-10-24 · Status: accepted (director, 2026-10-24) · Amended 2026-10-24 (A1, T-0015)

## Context
ADR-0003 fixes the container (zip of `manifest.json`, `job.json` and optional binary members).
T-0012 needs save → load → save to be byte-identical for the **whole archive**, exact doubles,
forward compatibility within a major version, and clean failures on bad input. This ADR records
the choices the card left open.

## Decision
1. **Members and order.** `manifest.json` first, `job.json` second, then preserved unknown members
   sorted by name. Each is deflated by miniz 3.0.2 at a fixed level (6). miniz is built with
   `MINIZ_NO_TIME`, so every member has DOS time and date 0 (otherwise it would use local time).
   There are no extra fields or comments. Byte identity holds for a fixed miniz version: upgrading
   miniz or changing the level may change the compressed bytes but never the JSON.
2. **Canonical JSON.** Keys are sorted (nlohmann's `std::map` objects). Output uses a two-space
   indent and ends with a final newline. Arrays of scalars go on one line, and so do *leaf
   records*: objects whose values are all scalars or arrays of scalars (spans, params, layers,
   sheets, the job, the version). That gives one entity per line for diffs and roughly halves the
   size; a 100k-object document's `job.json` is about 60–75 MB. Preserved unknown values are
   written in their canonical compact form (decision 7). Strings are UTF-8; invalid UTF-8 in
   in-memory strings is replaced by U+FFFD on save.
3. **Numbers.** Doubles are written in the shortest form that reads back to the same bits
   (`std::to_chars`). A `.0` is appended when the text has no `.` or exponent, so a double always
   reads back as a double and `-0.0` keeps its sign. Non-finite values are written as the strings
   `"NaN"`, `"Infinity"` and `"-Infinity"`; for example, an empty `Box2` is ±infinity. Ids are
   unsigned 64-bit integers. Readers accept any JSON number where a double is expected.
4. **Ordering inside `job.json`.** `sheets` and `layers` arrays are in document order (the array
   position *is* the stacking order, ADR-0007). `objects` are in id (creation) order. A group's
   `children` keep their list order. `variables` is an object keyed by name. `next_id` stores the
   id generator's high-water mark. On load it is restored without touching the document API, by
   inserting and immediately removing a layer with id `next_id - 1`.
5. **Schema conventions.** Keys are snake_case. Lengths are millimetres and angles are radians,
   with `_mm` / `_rad` suffixes on parameter fields. Points are `[x, y]` and sizes `[x, y, z]`.
   Enums are lower-case strings, colours are `"#rrggbb"`, and an object has `type` =
   `vector` | `bitmap` | `group`, with `parent` omitted for top-level objects. Selection and undo
   history are not saved; a loaded document has a clean, empty history.
6. **Versioning.** The manifest holds `format: "carvetto"`, `format_version: {major, minor}` and
   `app_version`. The manifest is checked before `job.json` is parsed. A major version other than
   1 fails with `UnsupportedVersion`, and the message says when the file is newer. A higher minor
   version of the same major loads. Minor bumps may only add optional fields, members or
   parameter kinds. Saving always writes this build's version.
7. **Unknown-content preservation (forward compatibility).** Unknown keys are kept in an opaque
   `CarvExtras` that `loadCarv` returns and `saveCarv` accepts.
   - *What is kept.* Unknown keys of the manifest, the `job.json` root and the `job` record, and
     of each sheet, layer and object record (keyed by id). Unknown archive members are kept as raw
     bytes. Per-entity extras follow their entity: they are dropped when the entity is deleted,
     and since ids are never reused they cannot attach to something else.
   - *How values are stored.* Each value is stored as *canonical compact JSON text* (never a DOM):
     - keys are sorted;
     - separators are `", "` and `": "`;
     - numbers are canonical: a token without `.`/`e` that fits 64 bits is written as that
       integer (so `-0` becomes `0`, and `1.0` stays `1.0`); everything else in the shortest
       double form. Numbers in unknown fields are thus normalised, not preserved byte for byte;
     - strings are re-escaped.

     On save this text is written verbatim, and re-reading it gives the same text again, so save
     → load → save stays byte-identical.
   - *Rules.* On save, known keys win over preserved ones. Creation parameters of an unknown
     `type` load as `NoParams` and are written back verbatim while the object still has
     `NoParams`. Unknown keys *nested inside* contours, spans, params, the bitmap placement or the
     version object are validated but not preserved. An unknown object type, span type or enum
     value is an error, since dropping it would silently change geometry. A known key that belongs
     to another object type (e.g. `children` on a vector) is also an error.
8. **Parsing and validation.** Loading never throws and never asserts. Untrusted JSON never
   becomes a DOM.
   - *Streaming.* `manifest.json` and `job.json` are decompressed on the fly (miniz extract
     iterator, 64 KiB at a time) into a strict streaming pull parser (`detail::JsonReader`): RFC
     8259 with UTF-8 validated, no trailing commas, and no duplicate keys in records or preserved
     values. It decodes straight into `Job`, `Sheet`, `Layer` and `Object` records. Neither
     member's text is ever held whole.
   - *nlohmann.* nlohmann/json is used only by the writer, on trusted in-memory data. Its SAX
     interface was not used because it gives no byte positions, which exact error offsets and
     raw/canonical capture of unknown values need.
   - *Order of checks.* Each step only runs if the previous ones passed:
     1. zip structure and the archive limits (§9);
     2. the manifest, and then the version check;
     3. `job.json` against the global budget (§9), with per-field types;
     4. after the members' CRCs, id uniqueness across sheets, layers and objects, and `next_id`
        greater than every id;
     5. object layer/sheet and the active layer/sheet references;
     6. the linear group-structure pre-check (§9);
     7. only then is the `Document` built.
   - *Damage vs. bad JSON.* A JSON error in a member of at most 16 MiB triggers a CRC check, so
     damage is reported as `InvalidArchive` rather than `InvalidJson`.
   - *Messages.* Each failure is a `CarvError` with a category and a message naming the member
     and the JSON path, e.g. `job.json.objects[3] (id 17).layer`, or a byte offset for syntax
     errors.
9. **Resource limits (`CarvError::Code::ResourceLimit`).** The memory and time a load can use are
   bounded by absolute limits, independent of the file's size or JSON shape. Allocation failure
   (`std::bad_alloc`) is also reported as `ResourceLimit`.
   - *Archive limits.* These are checked against the sizes declared in the central directory,
     before anything is decompressed. The local header of every entry is read and its data range
     computed; entries whose [local header, data end) ranges overlap or coincide are refused
     (`InvalidArchive`), so one compressed stream can never be inflated more than once.
     Extraction never produces more than the declared size, and binary members are read into a
     buffer that grows as data arrives, never pre-sized to the declared size.
     - At most 10 000 members, and at most 1 GiB declared across all of them.
     - A `.json` member may be at most 256 MiB. Every `.json` member is streamed through the
       strict reader: `manifest.json` and `job.json` are decoded, and any *other* `.json` member
       is validated (and its values charged to the unknown-content budget) before it is kept
       byte for byte. So for all of them the cap bounds time; memory is bounded by the budget and
       the held-bytes cap below. A 100k-object `job.json` is about 75 MB.
     - Any other member may be at most 256 MiB.
     - *Held-bytes cap.* The members kept verbatim after a load (everything but manifest.json and
       job.json) may hold at most max(64 MiB, 32 × archive size) in total, checked cumulatively on
       declared sizes as members are read (members before the offending one are already in
       memory, so peak memory is bounded by the cap, not below it). Unknown-field extras have
       their own 16 MiB budget on top. With the ratio checks in force nothing can exceed
       20 × the file size, so in practice this cap is a backstop; the test exercises it with the
       ratios relaxed.
     - Ratio checks apply to a member over 1 MiB: a `.json` member may expand at most 200:1 and
       a binary member at most 20:1. The binary ratio also applies to all binary members
       together, so many small members cannot add up to a bomb. Measured: a real 100k-object
       `job.json` compresses about 19:1, and 124:1 when all objects are identical; deflate bombs
       run at about 1000:1; zstd/PNG data is about 1:1.
   - *Bounded inflate.* Members are inflated by Carvetto's own loop over `tinfl_decompress`
     (`detail::BoundedStream`), not by `mz_zip_reader_extract_iter_read`, whose in-memory path
     never decrements its remaining-input counter and spins forever on a truncated, corrupted or
     size-lying stream. The loop feeds exactly the entry's declared compressed bytes, stops with
     `InvalidArchive` when the inflater asks for more input after all of it is consumed or
     consumes nothing, refuses output past the declared size, and checks the CRC and size at the
     end. Stored (method 0) members are served directly with the same size and CRC checks.
   - *One global budget across all members* (`detail::LoadLimits`):

     | Limit | Value |
     |---|---|
     | JSON values (scalars, containers and keys) | 24 M |
     | JSON values inside unknown fields | 2 M |
     | Decoded-record bytes | 192 MiB, estimated (see below) |
     | Canonical text of unknown fields | 16 MiB |
     | Number token length | 64 characters |
     | String length (known fields and keys) | 1 MiB |
     | String length (inside unknown fields) | 16 MiB |
     | Elements in one array/object of an unknown field | 1 M |
     | Nesting depth | 64 |
     | Sheets / layers | 1000 each |

     Decoded-record bytes are estimated per item: object 448, contour 64, span 144, group child
     16, sheet/layer/variable 256, plus 32 + the length of each stored string.

     A legitimate document of 3-span objects reaches the document limit at about 200k objects.
   - *Group structure.* A linear pre-check runs before the `Document` is built:
     - every parent exists, is a group and lists the child exactly once;
     - every listed child exists, points back and shares the group's sheet;
     - there is no cycle (memoised iterative walks);
     - groups nest at most 1024 deep, otherwise `ResourceLimit`.

     `Document::checkIntegrity()` (linear since T-0014) then runs on every load and must come
     back clean, so the group rules are checked twice: once before a `Document` exists, so a
     pathological shape never reaches it, and once on the built document.
   - *Ids.* Ids must be in 1 .. 2^62−1 (`IdGenerator::kMaxId`), and `next_id` may be at most 2^62
     (an exhausted generator).
   - *Measured* (GCC 13, Linux, this machine; peak RSS growth measured in a forked child after
     `malloc_trim`, with `/proc/self/clear_refs` reset and `VmHWM` read, the same method the
     regression test uses).
     - *Unknown-field bombs* (combo_str, manifest_str, arr_str, arr_arr, arr_zero, arr_true,
       arr_obj, many_keys, dup_keys, long_string, long_number, deep_wide, medium_objs; 0.3–4.7 MB
       files): each refused in Debug in 0.0005–0.45 s with 3–50 MiB peak RSS growth. They trip
       the small unknown-content limits (1 M elements, 2 M values, 16 MiB text, 64-char numbers,
       16 MiB strings) early.
     - *Known-field floods* are slower and heavier: a job.json of 600k minimal objects (1.66 MB
       archive) runs up the 192 MiB document budget and is refused after 5.0 s in Debug (0.76 s
       Release) with 225 MiB peak RSS growth, because records are staged in `JobFile` before the
       budget trips and the estimate under-counts that staging by about 15%. The bound holds but
       is not tight; a Debug build spends 4–6.5 s on such a file.
     - *Legitimate documents* (3 spans and params per object, Release): 100k objects load in
       2.0 s with 82 MiB peak RSS growth; 200k in 3.8 s with 164 MiB; 400k minimal objects in
       1.7 s with 242 MiB; a 200k-child group loads in about 1 s; a 60k-deep chain is refused in
       0.3 s.
   - *Zip-layer attacks* (Release, GCC 13, the regression tests' own measurements; peak RSS growth
     as above):
     - a central directory declaring `comp_size = 5` for job.json, the deflate stream cut in half
       with consistent headers, and 5 random bit flips inside the compressed data each fail with
       `InvalidArchive` in 0.08–0.35 ms (Debug: under 15 ms). Before amendment A1 the first two
       hung forever;
     - 60 directory entries sharing one 2.7 MB stream of 16 MiB each (2.7 MB file) are refused
       as overlapping in 54 µs with 1 MiB peak RSS growth (before: about 980 MiB);
     - a 250 MiB unknown `.json` bomb in a 255 KB file is refused by the 200:1 ratio check in
       37 µs with 1 MiB peak growth (before: about 325 MiB);
     - the fuzz test forks each of 5 000 loads (2 seeds; byte flips, truncation, compressed-data
       bit flips, directory lies, JSON mutation) under a 5 s alarm: 0 hangs, 0 crashes, in 6 s
       Release and 10 s Debug on this machine.

## Rationale
Shortest round-trip formatting is the only option that is both exact and stable across
re-saves. A fixed precision would either lose bits or add noise digits. Zeroed timestamps and
fixed member order make the archive reproducible, so tests and diff tools can compare whole
files. Keeping unknown content outside `Document` leaves the model (ADR-0007) untouched and
confines forward-compat logic to the I/O layer.

## Consequences
- The UI must hold on to `CarvExtras` between open and save to keep newer-version content.
- A newer-version file whose *unknown* content exceeds the unknown-field limits (2 M values,
  16 MiB) cannot be opened by this version; the limits trade that for bounded loads.
- `loadCarv` is intended to be safe on untrusted files: every known bomb and damage shape fails
  in bounded time and memory, and the forked fuzz test in CI guards that.
- Adding a field to the model means adding it to the encoder, the decoder and the known-key list
  (otherwise it would be treated as unknown and duplicated).
- Later members (`components/*.hf`, `toolpaths/*.bin`, `bitmaps/*`) already round-trip as
  preserved members until their phases give them a schema.

## Amendments
- **A1 (2026-10-24, T-0015): zip-layer hardening.** Decision 9 now states the bounded inflate
  loop over `tinfl`, the overlapping-entry rejection, the held-bytes cap and the streaming of
  unknown `.json` members, replacing the "Known limitations (tracked as T-0015)" list. The
  figures under *Zip-layer attacks* were measured with the T-0015 regression tests. Decision 8's
  claim that `.json` members are streamed now holds for every `.json` member.
