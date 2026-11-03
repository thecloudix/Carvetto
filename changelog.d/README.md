# Changelog fragments

Every task card adds its changelog entry here, as its own file, and **never edits `CHANGELOG.md`**.
One file per card (or per group of cards delivered together), so open pull requests no longer
conflict on the top of `CHANGELOG.md`.

- **Name:** `T-XXXX.md` for one card, e.g. `T-0146.md`. For one entry covering several cards
  (squashed together, like "T-0120/T-0124"), join the 4-digit ids with `-` in ascending order:
  `T-0120-0124.md`. Every id needs a card in `docs/BACKLOG.md`; a card appears in only one fragment.
- **Content:** exactly one line, in the same style as the lines in `CHANGELOG.md`, starting with
  the prefix that matches the name: `- T-0146: ...` or `- T-0120/T-0124: ...`. No blank lines,
  no heading. UTF-8 (no BOM); the usual typographic characters (—, ≤, °, …) are fine, control
  characters (tabs included) and invisible characters (zero-width, bidi overrides) are not.
- **Check locally:** `python3 tools/changelog.py check` (also run by `ctest` as
  `tools.changelog_check` and by CI on every PR).

CI (`python3 tools/changelog.py check --base origin/main` on every PR) compares the whole of
`CHANGELOG.md` with the PR's merge base, as multisets of non-blank lines, and accepts only two
kinds of change; anything else (a line added anywhere by hand, an existing line edited, removed or
duplicated, a second `## Unreleased`, existing Unreleased lines reordered) fails the PR:

1. **Assemble PR** (the director, when collecting entries): run `python3 tools/changelog.py assemble`
   on a branch and commit the `CHANGELOG.md` change together with the fragment deletions, nothing
   else in `CHANGELOG.md`. It passes because every line it adds sits under `## Unreleased` and is
   the line of a fragment the same PR deletes (each once).
2. **Release PR** (cutting a version): insert a `## <version>` heading directly below
   `## Unreleased`, so the existing entries now sit under the version and `## Unreleased` is empty
   again. Only the heading is new; every other line keeps its count (it merely moved), so it passes.
   Do not reword entries in the same PR. Assemble first (PR 1), then release (PR 2).

This README is ignored by the tools.
