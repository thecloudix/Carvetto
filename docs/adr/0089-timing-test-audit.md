# ADR-0089: Audit of CPU-time complexity tests (T-0313)

Status: accepted (T-0313; follows T-0010/T-0310)

## Context
About 30 tests assert complexity by the ratio of CPU times of a small and a large input
(`interleavedCpuSecondsRetry`, `core/tests/geometry/cpu_scaling.hpp`). T-0310 showed that even with
CPU time, interleaving and retries such a test can fail under load. The preferred cure is to count
operations of the algorithm under test, which no scheduler can disturb.

## Decisions
- **Converted to operation counts** (the algorithm is ours, has a clear unit of work, and the counter
  costs one increment):
  - `Document::checkIntegrity(std::size_t* work = nullptr)` reports its elementary steps (one per
    object, per listed child, per parent-chain object walked, marked or searched on a loop). The three
    `integrity_scale_test.cpp` cases assert `steps <= 8 N` (linear is about 5 N; the quadratic
    chain walk it replaced takes about N^2/2), at two sizes each. They no longer time anything.
  - `selection removal scales near-linearly` now uses `REQUIRE`, so a quadratic mutant stops at the
    first size instead of running every order and size. Slot moves are the same loop as the slots
    scanned (`compaction` moves a subset of them), so counting them separately adds no sensitivity.
- **Kept as CPU-time ratios** (with this justification): the remaining tests time third-party or
  numerically driven code (Clipper2 booleans, offsetting, medial axis, Bezier intersection, V-carve,
  toolpath generation, simulation, undo/redo of documents) where there is no single loop whose
  iterations are the cost; a counter would have to be threaded through code that is not ours or
  would count something only loosely tied to time (a mutant could stay quadratic in untracked work).
  Each keeps the T-0145/T-0270 safeguards, which together make noise irrelevant to a pass:
  - process CPU time, not wall time, so waiting for a core costs nothing;
  - small and large runs interleaved, ratio = min(ratio of least times, least pair ratio);
  - the small run repeated until one batch lasts at least 0.1 s, and its time floored at 50 ms,
    so a coarse clock tick or a tiny absolute time cannot inflate the ratio;
  - up to three attempts, keeping the least ratio: noise is independent per attempt, whereas a
    genuinely super-linear cost exceeds the bound in every attempt.
  The bounds (2.5 for doubling, 7-9 for quadrupling, 64 for the deliberate wide ones) sit well above the
  linear/n log n ratio and well below the quadratic one, so a quadratic mutant fails all attempts.
  The undo/redo test in `commands_test.cpp` (T-0014, T-0159) stays timed because its cost is spread
  over the command, document and map code; a counter on the undo stack alone would not see a
  quadratic `Document` mutator.

## Consequences
- `checkIntegrity` gains one defaulted out-parameter; behaviour is unchanged.
- Future complexity tests should count operations when the code under test is ours and has a clear
  unit of work; the timed helper is for the rest.
