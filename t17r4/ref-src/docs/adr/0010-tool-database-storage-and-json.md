# ADR-0010: Tool database identity, storage and JSON interchange

Date: 2026-10-24 · Status: accepted (director, 2026-10-24) 

## Decision
1. **Ids are caller-supplied strings.** Tools, groups, materials and machines carry a stable, non-empty `id` string chosen by whoever creates them (the UI will use UUID strings). `ToolDatabase` never generates ids and never renumbers; cutting data is keyed by `(machineId, materialId)`. Merge matches items by id.
2. **Containers are `std::map` keyed by id.** Every iteration (export, `toolsInGroup`, merge) is therefore in id order and deterministic. Display ordering (by name, by user drag order) is a UI concern.
3. **Cutting-data lookup falls back explicitly.** `Tool::cuttingDataFor(machine, material)` returns the exact entry or the tool's `defaultCuttingData` together with an `isFallback` flag, so callers can tell the user the values are not machine/material specific.
4. **JSON via nlohmann/json 3.11.3** (MIT), fetched by `FetchContent` from the pinned release tarball with a SHA-256 hash, included as a SYSTEM header and linked PRIVATE to `carvetto_core`; no public Carvetto header exposes it.
5. **Deterministic file format** (`"format": "carvetto-tool-database"`, `"version": 1`): object keys sorted (std::map-backed `nlohmann::json`), arrays in id order, two-space indent, trailing newline. Lengths are written in mm rounded to 1e-6; angles and angular accelerations are written in **degrees** rounded to 1e-6 (the core keeps radians). Rounding is `round(v·1e6)/1e6`, whose result is exactly the double the parser returns for the printed decimal, so export → import → export is a fixed point.
6. **Import is strict about shape, lenient about extras.** Unknown keys are ignored (forward compatibility); missing optional keys take defaults; wrong types, unknown tool types, duplicate ids, unknown groups/parents and invalid tool geometry (`validate(Tool)`) are errors naming the item. Groups may be listed in any order.

## Rationale
- String ids keep the core free of random/clock sources (ARCHITECTURE "Determinism", "No global state") and make merge semantics (keep vs overwrite) unambiguous.
- Fixed 1e-6 mm precision is far below any machining tolerance, hides binary-decimal noise (0.1+0.2 → 0.3) and makes diffs of exported databases readable.
- Degrees in files match how users think about V-bit and taper angles; conversion stays at the I/O edge per CONTRIBUTING.
- nlohmann/json is the de-facto standard header-only JSON library, MIT licensed, and will also serve `job.json` in the `.carv` container (ADR-0003).

## Consequences
- Other modules that need JSON should reuse the same dependency rather than add another.
- Values with more than six decimals of meaningful precision are not preserved through export; none are expected in tooling.
- A later format version must keep reading version 1 files; `kToolDatabaseFormatVersion` gates this.
