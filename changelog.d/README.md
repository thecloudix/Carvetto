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
  A line that is already in `CHANGELOG.md` is refused (it would be a duplicate entry).
- **Files:** regular files only (no symlinks, mode 100644).
- **Check locally:** `python3 tools/changelog.py check` (also run by `ctest` as
  `tools.changelog_check` and by CI on every PR).

CI (`python3 tools/changelog.py check --base origin/main` on every PR) compares `CHANGELOG.md`
byte for byte with its version at the PR's merge base. It must be identical except for exactly
one of these three changes; anything else fails the PR (a line added, edited, removed, moved or
reordered anywhere, a look-alike or second `## Unreleased`, changed line endings or whitespace,
even added blank lines):

1. **No change** (every ordinary PR). The PR may add fragments. It may delete fragments only if it
   also adds at least one: renaming a fragment, or merging `T-0120.md` and `T-0124.md` into
   `T-0120-0124.md` when cards are squashed. Deleting a fragment outright fails (the entry would
   be lost), unless its line is already in `CHANGELOG.md` (then nothing is lost; `assemble` also
   just deletes such a fragment).
2. **Assemble PR** (the director, when collecting entries): on a branch, run
   `python3 tools/changelog.py assemble` and commit its result as is: the `CHANGELOG.md` change plus
   the deleted fragments, no new fragment. Passes only if the lines of exactly the deleted
   fragments, each once, were inserted at the top of `## Unreleased` in `assemble`'s order and
   layout (newest card first, one blank line below the heading), with no other byte changed.
3. **Release PR** (cutting a version; deletes no fragment): insert one heading
   `## X.Y.Z` or `## X.Y.Z - YYYY-MM-DD` (ASCII digits without leading zeros, a real calendar date,
   a version greater than every version already in the file) at the top of a non-empty
   `## Unreleased`, followed by a blank line:

   ```
   ## Unreleased

   ## 0.2.0 - 2026-12-01

   - T-0151: ...   (all the former Unreleased lines, unchanged and in order)
   ```

   Every former Unreleased line now belongs to the release and `## Unreleased` is empty again.
   Assemble first (PR 2), then release in a separate PR. Never reword entries.

CI judges what is committed: `check --base` reads `CHANGELOG.md` and the fragments from the git
blobs at HEAD and at the merge base, never from the working tree (uncommitted changes only get a
note), and requires both to be regular files.

Line endings: `.gitattributes` keeps `CHANGELOG.md` and `changelog.d/*.md` as LF in every checkout;
a PR converting `CHANGELOG.md`'s line endings is refused like any other edit.

This README is ignored by the tools.
