# ADR-0088: the .carv moves-per-byte guard follows the deflate ceiling

Status: accepted (T-0312; amends ADR-0021 section 4, which recorded the owner's 2026-11-02 value)

## Context
ADR-0021 limited a file of S bytes to min(S x 1, 8,000,000) toolpath moves. Real files take 6-9
bytes per move, but a very regular toolpath (a plain 3D finish ramp) has constant deltas, its cache
deflates to a few bits per move, and `saveCarv` refused it as "more than one move per byte".

## Decision
- `kCarvMovesPerFileByte` is 1024. Every move takes at least one byte in its cache (the loader
  already rejects a record whose cache cannot hold `count` moves) and deflate cannot compress more
  than about 1032:1, so a ratio of 1024 admits every honest file and the guard is effectively
  "moves <= what the compressed archive could possibly encode". The same rule is applied by the
  loader (while `job.json` is parsed, before any cache is inflated) and the writer.
- The hostile-input bound is therefore the absolute cap `kCarvMaxToolpathMoves` (8,000,000 moves,
  about 576 MB decoded), the archive size caps and the 20:1 ratio for non-cache binary members, not
  the per-byte ratio. A crafted file of about 8 KB can still claim the whole cap, but must then
  carry a valid, fully inflating 8 MB cache of well formed moves; the loader validates caches as
  they are inflated and the memory use is bounded by the cap.
- A claim above S x 1024 or the cap is still refused from `job.json` before any cache is read.
- `LoadLimits::movesPerArchiveByte` stays configurable, and the tests exercise the exact boundary
  at 1 per byte through it.

## Consequences
The memory-amplification protection of the old rule (a 9 KB file loading 8M moves) is traded for
regular jobs saving; the worst case is unchanged from what a legitimate 8M-move file costs. If a
tighter bound is wanted later, lower `kCarvMaxToolpathMoves` or charge moves by decoded memory.
