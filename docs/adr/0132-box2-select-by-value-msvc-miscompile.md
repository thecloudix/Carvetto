# ADR-0132: Box2 selects by value (an MSVC miscompile that looked like a hardware-dependent failure)

Date: 2026-11-20 · Status: accepted (T-0364) · Follows T-0359 (orderCuts flake), ADR-0025 section 11 and
ADR-0032 section 7 (bit-exact arithmetic)

## Context
Windows CI showed failures that depended on the runner: orderCuts "same order as the exhaustive scans"
(T-0359, about 25 % of probe jobs), and on PR 231 the V-carve goldens and every V-carve corner/stretch test
(`cs.size() == 4`, got 0). The CPU model did not predict them (EPYC 7763 and 9V74 both passed and failed), so the
suspicion was the UCRT's run-time FMA3 selection (`__isa_available`) or other CPU-dependent library paths.

## What was measured (Release, MSVC 14.52.37025, windows-latest, 2026-11-20)
- One test binary built once and run on 50 runners (EPYC 7763 x20, 9V74 x18, 9V45 x3, Xeon 8573C x8, Xeon 8370C x1),
  tests `[corner]`, `AC1: the V-carve fixture*`, orderCuts: 50 of 50 failed the same 11 test cases, with and
  without `_set_FMA3_enable(0)`. The hardware hypothesis is false: the run machine is irrelevant.
- The same sources built on 6 runners produce a byte-identical `v_carve_corners.cpp.obj` disassembly (dumpbin
  /disasm), so the compiler is deterministic on every CPU. /fp:strict builds fail like /fp:precise. About 30
  builds of the test target in the failing arrangement all failed; with the change below 6 of 6 passed.
- Traces in the failing run: `RingRegion`/`IndexedRing` stored a `BoxIndex` whose boxes were `[inf inf]-[-inf
  18.98]`, `[1.0167 inf]-[-inf -inf]`, ...: of the four fields of each segment box only one had been written.
- Disassembly of `IndexedRing::IndexedRing` (`Box2 box; box.expand(a); box.expand(b);` inlined): after the first
  expand the new min/max sit in registers; the second expand's `std::min(min.x, p.x)` is compiled as a
  conditional move between the *addresses* of the operands and loads the result from memory, where the
  box field still holds its initial +-infinity (the register value was never stored back). With `a.x == b.x`
  or `a.x <= b.x` the stale slot wins. The same shape (`Box2 all; all.expand(p)` in a loop) is T-0359's
  "extent came out as the last start only".

## Decision
- `Box2` computes its corners with by-value helpers (`box_detail::minOf/maxOf`, same results as
  std::min/std::max including NaN) and writes both corners at the end of `expand`. No std::min/std::max on the
  fields of a box in `box2.hpp`.
- The T-0359 scalar accumulation in cut_order.cpp stays (harmless).
- Regression tests in box2_test.cpp (every ordering of coordinates through the `segmentBox` / ring-extent shapes);
  they are plain behaviour checks on Linux and catch the miscompile on MSVC if the pattern returns.
- No `_set_FMA3_enable` helper: the FMA3 hypothesis was measured and rejected, so adding it would be noise.

## Consequences
- Which build "fails" depends on inlining decisions (source layout), not on the machine, so a green run says little:
  a nearby change can flip it. Other `std::min/std::max` on fields of local aggregates may hit the same bug; if a
  Windows-only wrong result shows up again, check for this pattern first (dumpbin /disasm of the object).
