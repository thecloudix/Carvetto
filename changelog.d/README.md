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

At release time the director runs `python3 tools/changelog.py assemble` on a branch: it inserts all
fragments under `## Unreleased` in `CHANGELOG.md`, newest card first, and deletes them. That commit
(the `CHANGELOG.md` change plus the fragment deletions, nothing else in `CHANGELOG.md`) goes through a
normal PR. CI accepts it because every line it adds to the Unreleased section comes from a fragment
the same PR deletes; any other edit to the Unreleased section fails `check --base`.

This README is ignored by the tools.
