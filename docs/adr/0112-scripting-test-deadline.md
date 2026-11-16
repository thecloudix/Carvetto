# ADR-0112: Scripting tests never depend on the wall-clock deadline

Date: 2026-11-16 · Status: accepted (T-0338) · Builds on T-0330 (ScriptLimits::deadlineMillis backstop)

## Context
`ScriptLimits` defaults include a real 5 s wall-clock deadline. Tests asserting a deterministic stop
(InstructionLimit, MemoryLimit, pattern budget) ran with default limits, so on a machine at load ~59 the
deadline fired first (Timeout) and the tests failed.

## Decision
- `core/tests/scripting/script_host_test.cpp` has `testLimits()` (defaults with `deadlineMillis = 0`); every
  `ScriptLimits`/`ScriptHost` in tests starts from it. Deadline behaviour is tested only with the injected
  fake clock, plus one real-clock smoke test (150 ms deadline, asserts 100 ms <= elapsed < 20 s).
- Guard: the test "scripting: tests never use default ScriptLimits" greps the scripting test sources for
  `ScriptLimits{}`, `ScriptLimits x;`, `ScriptHost x;`, `ScriptHost x({})` and fails with the offending
  lines (`// lint-ok` exempts a line). It is a text heuristic, not a parser, but cheap and it catches the
  usual slips. Any new scripting test file in that directory is covered automatically.
- Audit: no gadget tests exist yet in `core/tests` or `app/viewmodel/tests`; gadget test files added later
  must use an equivalent helper (the lint covers `core/tests/scripting` only).
