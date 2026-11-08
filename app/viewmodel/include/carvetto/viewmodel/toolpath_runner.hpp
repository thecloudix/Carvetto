#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "carvetto/viewmodel/toolpath_calc.hpp"

// Runs toolpath calculations off the UI thread (T-0221, ADR-0031, ADR-0014). Qt-free: a
// `ToolpathJob` is a self-contained copy (no reference into the document), so a worker can run it
// while the document is edited, replaced or destroyed; the result is applied afterwards through
// `ToolpathSession::apply`, which refuses a result that no longer matches the document.
//
// One batch at a time. Jobs of a batch run in order on one worker thread; `cancel()` skips the
// jobs not yet started and makes the running one report `cancelled` once the (uninterruptible)
// core call returns. The destructor cancels and joins, so it waits at most for the call in
// progress. `start`, `cancel`, `finished`, `take` and the destructor are for the owner's thread;
// the completion callback runs on the worker thread (post to the UI thread from there).

namespace carvetto::viewmodel {

class ToolpathRunner {
   public:
    ToolpathRunner() = default;
    ~ToolpathRunner();
    ToolpathRunner(const ToolpathRunner&) = delete;
    ToolpathRunner& operator=(const ToolpathRunner&) = delete;

    /// Starts `jobs` on a worker thread. False (nothing started) when a batch is already running
    /// or its result has not been taken. `onFinished` (optional) is called on the worker thread
    /// after the last job; it must not touch the runner.
    bool start(std::vector<ToolpathJob> jobs, std::function<void()> onFinished = {});
    /// Asks the batch to stop (no effect when idle).
    void cancel();
    /// True from `start` until `take`.
    bool busy() const { return state_ != nullptr; }
    /// True when the batch has completed and `take` will not block.
    bool finished() const;
    /// The results of the finished batch, one per job (cancelled ones flagged), and idle again.
    /// Blocks until the batch is done; nullopt when idle.
    std::optional<std::vector<ToolpathCalculation>> take();

   private:
    struct State {
        std::vector<ToolpathJob> jobs;
        std::vector<ToolpathCalculation> results;
        std::atomic<bool> cancel{false};
        std::atomic<bool> done{false};
        std::function<void()> onFinished;
    };
    std::shared_ptr<State> state_;
    std::thread worker_;
};

}  // namespace carvetto::viewmodel
