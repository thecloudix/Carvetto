# ADR-0097: preview and simulation leftovers (T-0317, from T-0231)

Status: accepted

## Context
T-0231 left three costs for pathological jobs (500k full-width lines, 1M moves): simulation start froze the UI,
first paint and zoom were slow, and the first job summary took ~160 ms. Measured first (Release, GCC, a shared
machine under load, so numbers are indicative; tests use operation counters, never time), on a 1M-move job of
500k alternating rapid and full-width feed passes:

| Step | Before | After |
|---|---|---|
| `makeSimulationSetup` | < 1 ms | unchanged |
| `SimulationController::start()` on the UI thread | < 1 ms (the worker prepares) | unchanged |
| worker preparation (heightfield + queueing 1M moves) | 93 ms, not interruptible between toolpaths | 83 ms, interruptible every 32768 moves |
| `stop()` / restart join while preparing | up to the whole queueing call of the longest toolpath | at most one slice (~3 ms here) |
| first job summary (`Toolpath` copy + 5 passes) | 67-86 ms | 14-23 ms (one pass, no copy) |
| `PreviewModel::sync` (builds the polylines, once per move list) | 151 ms | unchanged |
| `PreviewModel::draw` per zoom step | 20-35 ms, emitting up to 1.5 M points | 6-33 ms, emitting ~400 k points |

The multi-second first paint and zoom steps and the 54 s start reported in T-0231 are not the model or the worker:
the draw list is cheap and the preparation is linear. The cost was the amount of geometry handed to Qt (the level
of detail bounded the *budgeted* points only; every chunk past the budget still added a two-point stroke, 1.5 M
points at the lowest zoom) and a start/replace that had to wait for a preparation that checked `quit` only between
toolpaths. The Qt stroking time itself was not re-measured here (no display; the app self-test covers
correctness), only the point and stroke counts handed to it.

## Decisions
- **Cancellable start.** The worker queues each toolpath in slices of 32768 moves (one `addToolpath` call per
  slice; the simulator continues from the end of the previous slice, so the queue, the timeline and the frames are
  identical to a single call, tested against a direct run) and looks at `quit` between slices. `stop()` and
  `start()` (which stops the previous run) therefore wait for at most one slice. It also stops copying the whole move
  list into a transient `Toolpath` (one slice at a time instead).
- **`cancel()` while preparing** keeps its meaning (it stops the run requested before; preparation itself is not
  abandoned, an abandoned preparation would leave no simulation to show). New: the cancel is looked at *before the
  first cutting step* of a run, not only after the first chunk, so a cancel that arrived while preparing cuts
  nothing (frame at move 0, `cancelled`).
- **Summary cost.** `toolpath::summarizeMoves` computes bounds, length by kind, min/max Z and the time estimate in
  one pass over a `span` of the shared moves; `ToolpathSession::statsOf` uses it (no copy, 1 pass instead of
  copy + 5). The results are value-for-value those of the separate `Toolpath` queries (tested). The result is still
  cached by move-list identity.
- **Preview level of detail.** Past the point budget, a coarse stroke whose style and end-point pixel cells (cell =
  the point spacing, at least 1 px) match one already emitted is not emitted again (`chunksMerged`; still counted in
  `chunksDrawn`, they are represented). Output past the budget is then bounded by the distinct screen strokes, not
  by the number of chunks. A 64-bit hash key is used: a collision would hide one stroke of a picture that is already
  past its budget (accepted). No effect without a budget.
- Counters for tests: `SimulationController::counters()` (moves queued, queue slices, cut chunks),
  `summarizeMovesVisited()`, `PreviewDrawStats::chunksMerged`. Test hook `setPrepareDelayForTest` pauses the worker
  after each slice so "stopped before the end of preparation" is deterministic.

## Not done
- Building the polylines (`sync`, 150 ms at 1M moves) stays on the UI thread; it runs once per move list.
- Preparation of a single *very large heightfield* allocation is not interruptible (one allocation).
