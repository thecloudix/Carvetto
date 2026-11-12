# Contributing to Carvetto

## Clean-room policy (read first)

Carvetto reproduces the *functionality and workflow* of commercial CAM software. It must never contain their code, assets, or protected content.

Contributors (human or agent) must not:

- Paste, transcribe, or port source code, scripts, or decompiled output from any proprietary CAM package.
- Copy icons, textures, clipart, sample files, fonts, help text, or UI artwork from those products.
- Reverse-engineer proprietary file formats (`.crv`, `.crv3d`, `.v3m`, `.3dclip`). Interchange happens through open formats (DXF, SVG, STL, OBJ, PDF, bitmaps).
- Redistribute post-processor files shipped with proprietary products. Test against community-published posts or posts written for Carvetto.
- Use trademarked product names in Carvetto's own UI or identifiers. Reference docs under `docs/reference/` describe behaviour only and name no third-party product.

Describing what a feature does, what its options are, and how a user expects it to behave is fine, and is what `docs/reference/feature-inventory.md` is for.

## Workflow

Work is organised as tasks in `docs/BACKLOG.md`, executed by coder agents and checked by verifier agents under a director. See `docs/WORKFLOW.md`. Every change arrives as a branch with:

1. The code and its unit tests (Catch2) in the same change.
2. A passing serial `ctest` run on Linux (core), and `ctest -j8` passing repeatedly (CI repeats it 5×; run `ctest -j8 --repeat until-fail:10` locally for timing-sensitive changes) — Windows app build is checked by CI.
3. A changelog fragment: one file `changelog.d/T-XXXX.md` per task (`T-XXXX-YYYY.md` for one entry covering several cards) holding exactly one bullet line in the style of `CHANGELOG.md` (`- T-XXXX: ...`). **Never edit `CHANGELOG.md`** — the fragments are assembled into its *Unreleased* section at release time (`tools/changelog.py assemble`). Rules in `changelog.d/README.md`; `python3 tools/changelog.py check` validates them (ctest and CI run it). CI fails a PR that changes `CHANGELOG.md` in any way other than the two documented shapes, the director's `assemble` PR and a release PR (see `changelog.d/README.md`).
4. A `docs/adr/NNNN-*.md` file when the change makes an architectural decision.
5. A row in `THIRD_PARTY_NOTICES.md` when the change adds a third-party dependency.

## CI, caching and reruns

Every dependency is fetched by CMake `FetchContent` at configure time, pinned by tag or by `URL_HASH SHA256` (never relax a pin to get past an outage). GitHub release assets sometimes answer 503/504; that is not a code failure.

- **Cache (T-0232).** Each CI job restores `.fc-src/` with `actions/cache`, keyed on the OS (and runner image) plus a hash of every `CMakeLists.txt` that declares a dependency (`core/`, `core/tests/`, `app/**`). `tools/ci/configure.sh` (Linux) and `tools/ci/configure.ps1` (Windows) pass `-DFETCHCONTENT_SOURCE_DIR_<NAME>=.fc-src/<name>-src` for each cached source tree, so a cache hit needs no network access. After a successful configure the scripts copy newly downloaded `_deps/*-src` trees into `.fc-src/`, and the cache is saved when the job ends green. Only source trees are cached, never build directories. Changing a version or hash in a declaration changes the key and forces a fresh, hash-verified download. When you add a dependency, put its `FetchContent_Declare` in a `CMakeLists.txt` under `core/` or `app/` so the key covers it. ccache is deliberately not used: it does not help the outage and adds floating-point/compiler-flag cache-poisoning risk to the Release matrix.
- **Retry.** If configure fails and its log shows a download or network error (HTTP 5xx, DNS, TLS, timeout, git clone/fetch failure), the scripts re-run it up to 3 attempts in total with 15 s and 45 s backoff and print a `::warning`. Any other failure (a SHA256 mismatch, a CMake error) fails immediately. Run `tools/ci/configure.sh -S . -B build ...` locally to see the same behaviour.
- **Rerun policy.** A red job whose annotation or `configure.log` tail shows a download error after three attempts is an infrastructure failure: use "Re-run failed jobs" once or twice, don't change code, and don't merge around a red check. A job failing for any other reason is a real failure, whatever it looks like.

## Code conventions

- C++20, warnings as errors (`-Wall -Wextra -Werror` / `/W4 /WX`).
- `carvetto_core` has **no Qt dependency**. UI code lives in `app/`.
- Units inside the core are **millimetres** and **radians**; conversion happens at the UI and I/O edges.
- Geometry is `double` precision; heightfields are `float`.
- Format with `clang-format` (config in repo). Headers under `core/include/carvetto/<module>/`, sources under `core/src/<module>/`.
- Public types are documented with a one-line `///` comment; algorithms get a short paragraph describing the approach and its references.
- No global mutable state. Every operation takes its inputs explicitly and is unit-testable headless.

## Windows-safe names

CI compiles with MSVC, which most of us cannot run locally. `<windows.h>` (pulled in by the test helpers
`core/tests/geometry/cpu_time.hpp` and `cpu_scaling.hpp`, or included directly) defines the macros `far`,
`near` (and `small`, `interface`, `IN`, `OUT`, `ERROR`, `min`, `max` unless `NOMINMAX`/`WIN32_LEAN_AND_MEAN`
are set) and, without `NOGDI`, GDI functions named `Polyline`, `Polygon`, `Rectangle`, `Ellipse`, `Arc`,
`Pie`, `Chord`, ... A test that uses such a name as an identifier compiles on Linux and fails on MSVC.
Rules: include `cpu_time.hpp`/`cpu_scaling.hpp`/`<windows.h>` **last**; never name a variable, parameter,
member or lambda `far`, `near`, `small`, `interface`, `IN`, `OUT`, `ERROR`; write `std::min`/`std::max`
(or `(std::max)(...)`); qualify GDI-like type names (`geometry::Polyline`) in files that can see
`<windows.h>`; do not name locals like C library functions (`round`, `abs`, `y0`, `index`, ...).

`python3 -I tools/check_windows_names.py` checks the first three groups on Linux (ctest `lint.windows_names`; its
own tests are `lint.windows_names_unit`). It strips comments and string literals and only looks at files that can see
`<windows.h>`: those that include it (directly or via the helpers) and shared headers under `tests/`. Escape hatch for a
provably safe use: `// lint-allow-windows-name` on the line, or an entry (`path` or `path:line`) in
`tools/windows_names_allowlist.txt`.
