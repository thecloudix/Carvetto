#pragma once

// Fail-fast guard for tests of "must not hang" properties (T-0179). `failFast` runs a callable on
// a worker thread and ends the process with a HANG report once the process has used more than
// the given CPU seconds, so a regression of a hang guard fails in seconds of CPU instead of
// stalling the suite until the runner timeout. The limit is CPU time (not wall-clock), so a
// heavily loaded CI machine slows the test down without flaking it.
//
// The callable must not use Catch2 assertions (those belong to the thread running the test case):
// return whatever the test needs to check and assert on it from the main thread.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <utility>

#include "cpu_time.hpp"

namespace carvetto::test {

/// Wall-clock backstop for a worker that blocks without using CPU (never reached by a spinning
/// hang, which trips the CPU limit first; below ctest's default 1500 s timeout, far above what a loaded CI machine needs).
inline constexpr double kFailFastWallBackstopSeconds = 300.0;

template <class F>
auto failFast(double cpuSeconds, const char* what, F&& fn) {
    const double cpuStart = processCpuSeconds();
    const auto wallStart = std::chrono::steady_clock::now();
    auto fut = std::async(std::launch::async, std::forward<F>(fn));
    while (fut.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
        const double used = processCpuSeconds() - cpuStart;
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
        if (used > cpuSeconds || wall > kFailFastWallBackstopSeconds) {
            // The stuck worker cannot be cancelled, so the process ends right after reporting.
            std::fprintf(stderr, "HANG: %s used %.1f s CPU (%.1f s wall) without finishing (limit %g s CPU)\n", what,
                         used, wall, cpuSeconds);
            std::fflush(stderr);
            std::_Exit(2);
        }
    }
    return fut.get();
}

}  // namespace carvetto::test
