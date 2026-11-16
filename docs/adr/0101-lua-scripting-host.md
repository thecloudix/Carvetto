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
   object code (to be measured on the release build), no runtime dependencies; native library loading is compiled out on every platform (a configure-time edit keeps only the
   stub in `loadlib.c`: neither `dlopen` nor Windows `LoadLibrary`), so even a granted `package` cannot
   load native code. Lua 5.4 rather than LuaJIT (stuck at 5.1 semantics, weaker
   portability) or a newer series (not yet proven in our CI); which Lua version existing community gadgets
   target is unknown to us (ADR-0102, question 1).
2. **Compiled as C++.** The `.c` files are compiled with the C++ compiler (`LANGUAGE CXX`), which Lua
   supports: errors become C++ exceptions instead of `longjmp`. Hooks, host callbacks and `print` can then
   use `std::string`/`std::vector` and RAII without setjmp hazards, on MSVC as on GCC. Third-party
   warnings are silenced for this target only; Carvetto's own code keeps warnings-as-errors.
3. **Two source patches (the second one is `loadlib.c`, decision 1)**; the first, applied at configure time and idempotent like the Clipper2 ones: `match()` in
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
   * *Instructions:* a count hook every 100 VM instructions adds to a budget (default 50 M; measured on a loaded Debug build, `while true do end` stops in about 0.4 s;
     Release not measured); the run stops within one block. The budget is shared by coroutines. After a trip the hook
     is re-armed to fire on every instruction, so a script that wraps the loop in `pcall` (or loops over
     `pcall`) is stopped by the next instruction outside the `pcall`.
   * *C-side work:* the string and table library functions charge units proportional to their operands
     (strings 1 per 8 bytes, tables 1 per element, sort 16 per element; 1000 calls of `s:upper()` on a
     10 MB string stop after about 40 calls, about 1 s in Debug) against the same budget, so
     `table.insert(t, 1, x)` or `s:upper()` on a large operand in a loop cannot hide in C.
     `table.move` is bounded by `maxTableMoveElements`.
   * *Pattern steps:* decision 3. Default 2 M steps (lowered from 20 M); measured 20-60 ns per step in
     Debug, so a full budget takes 0.05-0.12 s there, for the patterns tried (backtracking on 10 MB
     subjects, nested captures). A verifier measured about 10 s for the old 20 M on a loaded machine
     with a pattern we could not reproduce; the lower default leaves 10x margin.
   * *Heap:* the allocator refuses growth past `maxMemoryBytes` (default 64 MiB); Lua raises "not enough
     memory", which a script may catch; its loop is then stopped by the instruction budget.
   * *Strings and blocks (per-operation caps):* the allocator refuses any single new string above
     `maxStringBytes` (default 1 MiB; this covers `..`, `rep`, `format`, `tostring`, `concat`, error
     messages, `gsub` results) and any single block above `maxAllocationBytes` (default 8 MiB: one table
     array, stack, `luaL_Buffer`). A refused request is an ordinary "not enough memory" (MemoryLimit).
     `string.rep` also refuses up front with "resulting string too large". `load` refuses text chunks
     above `maxLoadBytes` (256 KiB, returns nil plus a message). The point is not accounting but
     latency: one C-side operation touches at most a few MiB, so the deadline poll below fires within
     milliseconds of any operation. Consequence: arrays above about 500 k elements cannot be built in a
     script.
   * *Values crossing the boundary:* one conversion (a returned value or the arguments of a host
     call) visits at most `maxValueElements` (1 M) values and copies at most `maxValueBytes` (16 MiB) of
     string data, besides the depth limit. Without this, a table built as a shared DAG (60 references
     to the previous level, 15 levels) expands to 60^15 copies in the C++ heap. Over budget, a host
     call fails with a catchable script error and a returned value ends the run with RuntimeError.
     Cancellation is polled every 1024 values. Host exceptions of any type become script errors.
     *Conversion is paid for:* the budgets above bound one conversion, but a loop of host calls could
     repeat a maximal one millions of times for ~4 VM instructions each. So every value visited costs 8
     budget units (a table entry walked during the shape check 4, strings 1 per 8 bytes), in both
     directions (arguments, host results, the returned value). A conversion may spend at most the run's
     remaining budget and stops there (trip -> InstructionLimit, also for rejected values), and the units
     are added to the instruction counter afterwards; the post-run conversion is accounted the same way
     (a script that spent its whole budget cannot also return a large table). Rule for the class: any
     host-boundary work done outside the instruction hook is charged to the budget, bounded, or both. The
     work a host function itself does is the registering code's responsibility.
   * *Wall-clock deadline (backstop):* three verifier rounds showed that charging cannot bound every
     C-side path of the Lua libraries (each round plugged one and found more). So the guarantee is
     layered: the deterministic budgets above give reproducible stops for ordinary runaway scripts, and
     `ScriptLimits::deadlineMillis` (default 5000, 0 = off) ends whatever they miss with status
     `Timeout`. The deadline is polled every 100 VM instructions (also inside coroutines, and again
     after a trip, so `pcall` cannot swallow it), every 256 pattern steps, every 1024 values converted
     in either direction, and at each host-call and library-call boundary. The clock is injectable
     (`ScriptLimits::clockNanos`, default `std::chrono::steady_clock`) so unit tests use a fake clock and
     never sleep. **A Timeout is explicitly non-deterministic**: whether and where a script is stopped
     depends on machine speed and load, unlike every other status; callers must not treat it as
     reproducible and must not cache or compare it. The host stays usable afterwards (each run has its
     own Lua state, nothing is shared). Work done by a host function itself is not interrupted; the
     deadline fires when it returns.
   * *Measured (Debug, loaded machine, defaults, verifier attack scripts a1..a17):* a10 coroutine churn
     and a12 `tonumber` loop stop on the deadline at 5.0 s; a13 sort 1.95 s, a17 insert/remove 1.1 s,
     a6 deep-stack errors 1.6 s, a9 big keys 1.5 s, a7 unpack 0.84 s, a11 `load` loop 1.45 s (all
     instruction budget); a16 `gsub` 0.07 s and a4 `table.concat` 0.006 s (memory cap); a3, a3s, a5, a8,
     a14, a15 stop in under 1 ms (string cap / invalid format); a2 0.014 s and a1 0.5 s (the host
     function that builds a 1 M-element array dominates; the push is refused by the block cap). No
     attack ran past the deadline plus one capped operation.
   * *Output:* `print` is captured, not written anywhere, and bounded by `maxOutputBytes`.
   * *Recursion:* Lua's own limits stay (about 200 nested C calls or metamethods, 1 M stack slots), and the
     heap limit bounds them in practice; the result is a normal error (`stack overflow`).
   * *Source size:* `maxSourceBytes`, refused before parsing.
7. **Cancellation.** `run(source, name, const std::atomic<bool>*)`: polled in the count hook and every 256
   pattern steps; checked once before starting. A cancelled script cannot catch the error (see 6).
   The flag is the same kind of flag as `geometry::CancelScope`; the two will be raised together by T-0331.
8. **Results.** `ScriptResult` carries a status (Ok, SyntaxError, RuntimeError, InstructionLimit,
   MemoryLimit, OutputLimit, Cancelled, Timeout, LimitsInvalid), the message `name:LINE: text` as Lua reports it,
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
  (for example sorting 500 k elements once) runs to completion; its cost is charged afterwards. The
  per-operation caps limit the operand size and the deadline ends the run at the next poll.
* The instruction, pattern, heap and size budgets are deterministic; the wall-clock deadline is not
  (see 6). Tests use a fake clock; the one real-clock test is a smoke test with generous margins.
* The sandbox is a policy over a language runtime, not an OS boundary: it protects a user from a
  gadget's file/process access and runaway resource use, not against memory-safety bugs in Lua itself.
  If that is wanted later, run scripts in a helper process.

## Consequences
`carvetto_core` links `carvetto_lua` privately; no public header includes Lua. Tests (`core/tests/
scripting/script_host_test.cpp`) cover the sandbox, every limit with hostile scripts, cancellation and
the value boundary.
