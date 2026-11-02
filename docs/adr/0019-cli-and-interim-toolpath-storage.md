# ADR-0019: carvetto-cli and interim toolpath storage in `.carv`

Date: 2026-11-01 · Status: proposed (T-0122) · Storage, ordering and params.json schema decided by
the project owner (2026-11-01, recorded under the T-0122 card)

## Context
T-0122 adds a headless console program, `carvetto-cli`, with the commands
`import <dxf> <out.carv>`, `toolpath profile|pocket <params.json> <in.carv> <out.carv>`,
`post <post.pp> <in.carv> <out.nc>` and `info`. CI uses it for an end-to-end test on Linux and
Windows. Computed toolpaths have to survive from `toolpath` to `post` inside a `.carv`. But the
document model and `saveCarv`/`loadCarv` have no toolpath records yet. ADR-0003 plans toolpath
definitions in `job.json` with an optional `toolpaths/*.bin` cache, and no card has built them.

## Decision

### 1. Interim storage: one archive member per toolpath (owner decision)
- Each computed toolpath is stored as the archive member `toolpaths/NNNN.json`, numbered from
  0001 in creation order. A new toolpath takes one above the highest existing number, up to 9999.
  The core loader keeps these as unknown members and writes them back on every save (ADR-0013 §7),
  so the app does not lose them.
- Content: one compact JSON object written with nlohmann/json (keys sorted, round-trip numbers,
  so the output is deterministic):
  - `format` = `"carvetto-toolpath"`, `format_version` = 1.
  - `operation` (`"profile"` or `"pocket"`), `name` and `safe_z`.
  - `params`: the params.json the toolpath was made from, canonicalised.
  - `tool`: the `ToolSnapshot` (name, tool number, diameter, feed, plunge rate, spindle speed).
  - `moves`: an array of moves. Each move is `["kind", x, y, z]`; arcs add `cx, cy` and a dwell
    adds its seconds; an optional trailing `{"feed": f}` is a feed override.

  Doubles round-trip exactly.
- Core change: two small public accessors in `carv_io.hpp`. `CarvExtras::member(name)` reads a
  preserved member. `CarvExtras::withMember(name, bytes)` returns a copy with a member added or
  replaced. It throws `std::invalid_argument` for names `saveCarv` cannot write (empty, reserved,
  absolute, `.`/`..` or empty segments, backslash or colon, over 255 bytes).
- **Known limit.** The loader counts the values in these JSON members against its budget for
  unknown content: 2,000,000 JSON values per file (ADR-0013 §9), about 5–7 values per move, so
  roughly 300,000–400,000 moves in all toolpaths of a file. Without a check, `toolpath` would
  write a file that the next command cannot open (seen with a 450,000-move raster pocket).
  `toolpath` therefore loads the encoded archive again before writing. If it would not load, the
  command fails with a message, names this storage as the cause and writes nothing. The
  replacement storage should keep moves out of the JSON budget, for example in ADR-0003's binary
  `toolpaths/*.bin` cache.
- **This storage is interim.** It is replaced once a card implements ADR-0003's `job.json`
  toolpath records (definitions with their parameters and vector selection, plus a cache of moves).
  That card should read `toolpaths/NNNN.json` members and convert them.

### 2. params.json (owner decision)
- Keys are snake_case and mirror `ProfileParams` / `PocketParams`, including nested objects
  (`passes`, `tabs`, `ramp`, `leads`, `order`, `corners`, `tolerance`).
- Lengths are in mm. Angles are in degrees and are converted to radians in the CLI.
- The tool is `{"tool": {"id", "database", "machine", "material"}}`:
  - `machine` and `material` go together. They pick the cutting data with
    `Tool::cuttingDataFor`, and a fallback to the default data is reported as a warning.
  - Without them the tool's default cutting data is used.
- `"layers": [names]` restricts the vectors. The default is every vector object, in creation order.
- Unknown keys are errors that name the key and list the keys known at that level.
- Duplicate keys are errors that name the key path, and objects/arrays nested more than 32 levels deep are refused (T-0134); JSON parsers would otherwise keep the last duplicate silently, and the schema needs three levels.

Implementation choices made here:
- **Relative paths.** A relative `tool.database` is resolved against the params file's directory.
  The command's working directory is not used, so a job folder can carry its own database.
- **Default database.** Without `tool.database`, the CLI uses the starter database
  `data/tools/starter_tools.json`, compiled into the executable at build time. CMake turns the file
  into a byte array, because MSVC limits string literals to 64 KiB. The CLI therefore works from
  any directory and after installation. A test checks that the embedded copy matches the file.
- **One tool per pocket.** A pocket takes one tool. Rest-machining tool lists are not in the
  agreed schema.

### 3. Command behaviour
- **Exit codes.** 0 means success. 1 means the command ran but failed (unreadable or invalid input,
  no toolpath produced, post failure). 2 means the command line is wrong. Errors and warnings go to
  stderr as `carvetto-cli: error: …` and `carvetto-cli: warning: …`. A failing command writes nothing.
- **import.**
  - Each DXF layer becomes one document layer (name, colour, visibility). The default "Layer 1"
    is removed when the drawing has layers.
  - Each contour becomes one vector object.
  - Text is skipped with a warning until text rendering exists (T-0130).
  - The job is the default job.
- **toolpath.**
  - Toolpath diagnostics are printed as warnings.
  - An operation that produces no moves fails, and its diagnostics form the error message.
  - Pocket tools that produce an empty toolpath are not stored.
  - The archive is loaded again before it is written (see the known limit in §1).
- **post.** The post-processor context:
  - `[TP_FILENAME]` is the output file's stem.
  - `[DATE]` and `[TIME]` are empty, so the output is reproducible.
  - Safe Z, and home Z with home at X0 Y0, are the highest `safe_z` of the stored toolpaths.
  - Material extents come from the job.
- **Code layout.** All logic is in `carvetto_cli_lib` (`app/cli/src`), and `main.cpp` only
  forwards `argv`. The tests (`app/cli/tests`, as for `app/viewmodel`) drive commands in-process.

### 4. Build and CI
- `CARVETTO_BUILD_CLI` (default ON) builds the CLI in every configuration, including the headless
  one, because the CLI does not use Qt.
- The golden G-code is `core/tests/golden/cli/part_grbl.nc`. It is made from
  `app/cli/tests/data/part.dxf`, then `pocket.json`, then `profile.json` (with tabs), using a
  fixture tool database so edits to the starter database do not move it, and `posts/grbl_mm.pp`.
- Three checks compare against the golden:
  - the in-process test;
  - the ctest `cli.end_to_end_grbl`, which runs the real executable and compares SHA-256;
  - an explicit CI step in the linux-core and windows-app (MSVC) jobs.
- `tools/regen_golden.py` regenerates the golden through the `[.regen]` case of
  `carvetto_cli_tests`.

## Alternatives
- Implement ADR-0003's `job.json` toolpath records first: the right long-term design, but much
  larger, and it would churn with the Profile/Pocket parameter types (owner chose interim storage).
- Keep toolpaths out of `.carv` (separate toolpath files, or one `run` command): changes the
  commands the card specifies.
- Read the default tool database relative to the working directory: breaks as soon as the CLI
  runs outside the repository.
