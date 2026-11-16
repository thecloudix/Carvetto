# ADR-0101: the Lua scripting host (T-0330)

Status: accepted (host); the API surface it serves is ADR-0102 (proposed, for owner review)

## Context
Phase 3 adds user scripts ("gadgets", feature-inventory section 6) that read and create vectors and
toolpaths. Scripts are written by third parties and shared as files, so the host must treat them as
hostile: no file or process access, bounded CPU and memory, cancellable, and a failing script must give
an error a user can act on (chunk, line, message). The card requires Lua, no Qt in `core/`, and builds
on GCC and MSVC.

## Decisions

1. **Lua 5.4.8, vendored by FetchContent** like the other dependencies: the release tarball from lua.org
   pinned by SHA256 (`4f18ddae...29ae`, computed from the official download), built as a small private
   static library `carvetto_lua` from the plain sources (not `lua.c`, `luac.c`, the Makefile). Licence:
   MIT, compatible with the project's GPL-3.0; the notice is in the source tree fetched at build time and
   must be shipped with binaries (T-0331 adds it to the third-party notices). Size: a few hundred KB of
   object code (to be measured on the release build), no runtime dependencies; no `dlopen` support is compiled (`package.loadlib` unsupported),
   so even a granted `package` cannot load native code. Lua 5.4 rather than LuaJIT (stuck at 5.1 semantics, weaker
   portability) or a newer series (not yet proven in our CI); which Lua version existing community gadgets
   target is unknown to us (ADR-0102, question 1).
2. **Compiled as C++.** The `.c` files are compiled with the C++ compiler (`LANGUAGE CXX`), which Lua
   supports: errors become C++ exceptions instead of `longjmp`. Hooks, host callbacks and `print` can then
   use `std::string`/`std::vector` and RAII without setjmp hazards, on MSVC as on GCC. Third-party
   warnings are silenced for this target only; Carvetto's own code keeps warnings-as-errors.
3. **One source patch**, applied at configure time and idempotent like the Clipper2 ones: `match()` in
   `lstrlib.c` calls `carvettoLuaMatchStep(L)`. The VM instruction hook never fires inside C library
   code, and a pattern such as `('.-'):rep(8) .. 'b'` on a 2000 character subject runs for minutes there.
   The step counter is bounded by `ScriptLimits::maxPatternSteps` and polls cancellation. The patch fails
   configuration loudly if the source is not 5.4.8.
4. **One Lua state per `run()`.** A run creates a state with a counting allocator, runs the chunk,
   destroys the state. Scripts cannot leak globals into each other, budgets start full, and there is no
   re-entrancy to reason about. Registrations (host functions, constants) live in `ScriptHost` and are
   installed into each new state. Cost: building a state is small next to a gadget's own work.
5. **Libraries.** Always: base, coroutine, table, string, math, utf8, reduced as below. `io`, `os`,
   `debug`, `package` exist only if `ScriptGrants` names them (default none; the app grants none in T-0331).
   Removed from base: `dofile`, `loadfile`. `load` accepts a string chunk in text mode only (binary chunks
   are a known escape; function readers are refused); `string.dump` is removed. `collectgarbage` offers
   `"count"`, and `"collect"`/`"step"` as no-ops (a forced full cycle is a time-burning primitive).
   `setmetatable` refuses a metatable with `__gc`, because finalizers run inside `lua_close` where Lua
   disables hooks and an infinite loop there would hang the host. `math.randomseed(0)` is applied at
   start: random numbers are repeatable until the script seeds them.
6. **Limits (`ScriptLimits`, defaults in the header).**
   * *Instructions:* a count hook every 1000 VM instructions adds to a budget (default 50 M, roughly half
     a second); the run stops within one block. The budget is shared by coroutines. After a trip the hook
     is re-armed to fire on every instruction, so a script that wraps the loop in `pcall` (or loops over
     `pcall`) is stopped by the next instruction outside the `pcall`.
   * *C-side work:* the string and table library functions charge units proportional to their operands
     (strings 1 per 64 bytes, tables 1 per element, sort 16 per element) against the same budget, so
     `table.insert(t, 1, x)` or `s:upper()` on a large operand in a loop cannot hide in C.
     `table.move` is bounded by `maxTableMoveElements`.
   * *Pattern steps:* decision 3.
   * *Heap:* the allocator refuses growth past `maxMemoryBytes` (default 64 MiB); Lua raises "not enough
     memory", which a script may catch; its loop is then stopped by the instruction budget.
   * *Strings:* `string.rep` is refused above `maxStringBytes`; every other way to grow a string
     (`..`, `format`, `concat`) is bounded by the heap limit.
   * *Output:* `print` is captured, not written anywhere, and bounded by `maxOutputBytes`.
   * *Recursion:* Lua's own limits stay (about 200 nested C calls or metamethods, 1 M stack slots), and the
     heap limit bounds them in practice; the result is a normal error (`stack overflow`).
   * *Source size:* `maxSourceBytes`, refused before parsing.
7. **Cancellation.** `run(source, name, const std::atomic<bool>*)`: polled in the count hook and every 256
   pattern steps; checked once before starting. A cancelled script cannot catch the error (see 6).
   The flag is the same kind of flag as `geometry::CancelScope`; the two will be raised together by T-0331.
8. **Results.** `ScriptResult` carries a status (Ok, SyntaxError, RuntimeError, InstructionLimit,
   MemoryLimit, OutputLimit, Cancelled, LimitsInvalid), the message `name:LINE: text` as Lua reports it,
   the parsed line, a traceback, the captured output, the returned values and the budget used. Chunk names
   are passed as `=name` so messages read `gadget.lua:12:`.
9. **Values across the boundary** (`ScriptValue`): nil, bool, int64, double, string, array of values
   (a Lua sequence 1..n, depth at most `maxValueDepth`). Nothing else crosses: tables with other keys,
   functions, userdata and threads are refused with a script error; they are returned from a chunk as
   nil. Host functions may throw `ScriptError` (or any `std::exception`); the script sees a catchable Lua
   error carrying the script's own line. Host functions never see a `lua_State`, so the Lua headers stay
   inside `core/src/scripting/script_host.cpp`. Richer data (vectors, layers, toolpaths) crosses as
   integer handles plus arrays (ADR-0102); a string-keyed table kind can be added to `ScriptValue` when
   T-0331 needs it.

## Known limits (honest)
* Instruction counts bound CPU only approximately: a single C call that does a lot of work in one go
  (for example sorting 4 M elements once) runs to completion; its cost is charged afterwards.
  The heap limit caps how big such an operand can be.
* The budget is deterministic (instruction counts), not wall-clock; tests never time anything.
* The sandbox is a policy over a language runtime, not an OS boundary: it protects a user from a
  gadget's file/process access and runaway resource use, not against memory-safety bugs in Lua itself.
  If that is wanted later, run scripts in a helper process.

## Consequences
`carvetto_core` links `carvetto_lua` privately; no public header includes Lua. Tests (`core/tests/
scripting/script_host_test.cpp`) cover the sandbox, every limit with hostile scripts, cancellation and
the value boundary.
