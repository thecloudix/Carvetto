# Contributing to Carvetto

## Clean-room policy (read first)

Carvetto reproduces the *functionality and workflow* of commercial CAM software. It must never contain their code, assets, or protected content.

Contributors (human or agent) must not:

- Paste, transcribe, or port source code, scripts, or decompiled output from any Vectric product or other proprietary CAM package.
- Copy icons, textures, clipart, sample files, fonts, help text, or UI artwork from those products.
- Reverse-engineer proprietary file formats (`.crv`, `.crv3d`, `.v3m`, `.3dclip`). Interchange happens through open formats (DXF, SVG, STL, OBJ, PDF, bitmaps).
- Redistribute post-processor files shipped with proprietary products. Test against community-published posts or posts written for Carvetto.
- Use trademarked product names in Carvetto's own UI or identifiers. Reference docs under `docs/reference/` may name them for comparison purposes only.

Describing what a feature does, what its options are, and how a user expects it to behave is fine, and is what `docs/reference/feature-inventory.md` is for.

## Workflow

Work is organised as tasks in `docs/BACKLOG.md`, executed by coder agents and checked by verifier agents under a director. See `docs/WORKFLOW.md`. Every change arrives as a branch with:

1. The code and its unit tests (Catch2) in the same change.
2. A passing serial `ctest` run on Linux (core), and `ctest -j8` passing repeatedly (CI repeats it 5×; run `ctest -j8 --repeat until-fail:10` locally for timing-sensitive changes) — Windows app build is checked by CI.
3. A changelog fragment: one file `changelog.d/T-XXXX.md` per task (`T-XXXX-YYYY.md` for one entry covering several cards) holding exactly one bullet line in the style of `CHANGELOG.md` (`- T-XXXX: ...`). **Never edit `CHANGELOG.md`** — the fragments are assembled into its *Unreleased* section at release time (`tools/changelog.py assemble`). Rules in `changelog.d/README.md`; `python3 tools/changelog.py check` validates them (ctest and CI run it). CI fails a PR that changes `CHANGELOG.md` in any way other than the two documented shapes, the director's `assemble` PR and a release PR (see `changelog.d/README.md`).
4. A `docs/adr/NNNN-*.md` file when the change makes an architectural decision.
5. A row in `THIRD_PARTY_NOTICES.md` when the change adds a third-party dependency.

## Code conventions

- C++20, warnings as errors (`-Wall -Wextra -Werror` / `/W4 /WX`).
- `carvetto_core` has **no Qt dependency**. UI code lives in `app/`.
- Units inside the core are **millimetres** and **radians**; conversion happens at the UI and I/O edges.
- Geometry is `double` precision; heightfields are `float`.
- Format with `clang-format` (config in repo). Headers under `core/include/carvetto/<module>/`, sources under `core/src/<module>/`.
- Public types are documented with a one-line `///` comment; algorithms get a short paragraph describing the approach and its references.
- No global mutable state. Every operation takes its inputs explicitly and is unit-testable headless.
