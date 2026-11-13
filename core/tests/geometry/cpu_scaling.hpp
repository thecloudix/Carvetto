#pragma once

// Load-robust CPU-time scaling measurements for the complexity-ratio tests (T-0145).
//
// A ratio test times a small input (n) and a large one (2n, 4n, ...) and checks that the large
// run costs at most a bound times the small one, which catches quadratic regressions on any
// machine. Measuring all small runs first and all large runs afterwards (best of k each) is
// fragile under parallel ctest on an SMT machine: CPU time is immune to waiting for a core but not
// to sharing one with a busy sibling thread, so a load swing between the two windows (~25 %)
// moves the ratio. Here the runs alternate (small, large, then large, small, ...) so every window
// of load affects both sizes alike, and the ratio is the smaller of
//   - the overall minima, least(large) / least(small): each minimum is the quietest sample of its
//     size, and the quiet moments are shared because the runs are interleaved; and
//   - the least per-pair ratio: neighbouring runs see nearly the same load, so a pair taken in a
//     slow phase still gives a fair ratio.
// A quadratic cost is over the bound in every pair and in the minima, so it is still caught.
//
// Timer resolution: the Windows process clock (GetProcessTimes) ticks every ~15.6 ms, and an
// optimised build does some of the small inputs in 10-30 ms. So the small run is first repeated
// in batches until one batch lasts at least `minWindowSeconds` (default 0.1 s; at most 64
// repetitions), and every measured run then calls its function that many times; the ratio of
// equal batches is the ratio of single calls. This first batch also warms caches and the
// allocator. The small batch time is further floored at `kCpuTimeFloor` in the ratio, so a batch
// that is only a few ticks long cannot fail it. Debug builds mostly need no repetition.
// `smallSeconds`/`largeSeconds` are per call (least batch time / repetitions), for the absolute
// Release bounds.

#include <algorithm>
#include <cmath>
#include <limits>

// Identifiers avoid `small`, `near` and `far`, which <windows.h> may define as macros.

#include "cpu_time.hpp"

namespace carvetto::test {

/// Least small-batch CPU seconds a ratio divides by: ~3 ticks of the Windows process clock.
inline constexpr double kCpuTimeFloor = 0.05;
/// Default least duration of one small batch (see the file comment).
inline constexpr double kCpuMinWindow = 0.1;
/// Most calls of a function per measured batch.
inline constexpr int kCpuMaxReps = 64;

/// CPU seconds of interleaved small and large runs (see the file comment).
struct ScalingTimes {
    double smallSeconds{std::numeric_limits<double>::infinity()};  ///< least small-run CPU seconds per call
    double largeSeconds{std::numeric_limits<double>::infinity()};  ///< least large-run CPU seconds per call
    double pairRatio{std::numeric_limits<double>::infinity()};     ///< least large / floored small of a pair
    double timeFloor{kCpuTimeFloor};  ///< lower bound applied to small-batch times
    int reps{1};                      ///< calls per measured batch

    /// Large over small cost, robust to load swings: min(minima ratio, least pair ratio).
    double ratio() const {
        const double smallFloored = std::max(smallSeconds, timeFloor / static_cast<double>(reps));
        return std::min(largeSeconds / smallFloored, pairRatio);
    }
};

/// Runs `runSmall()` and `runLarge()` in `pairs` alternating pairs of equal-size batches (the
/// first of a pair alternates between the two) and returns their CPU times. Each call of a
/// callable does one run of the measured work and keeps whatever the test needs for its checks.
template <class Small, class Large>
ScalingTimes interleavedCpuSeconds(int pairs, Small&& runSmall, Large&& runLarge,
                                   double timeFloor = kCpuTimeFloor,
                                   double minWindowSeconds = kCpuMinWindow) {
    ScalingTimes t;
    t.timeFloor = timeFloor;
    const auto timed = [&t](auto& f) {
        const double t0 = processCpuSeconds();
        for (int k = 0; k < t.reps; ++k) {
            f();
        }
        return processCpuSeconds() - t0;
    };
    // Calibrate the batch size on the small run (also a warm-up; not part of the result).
    for (;;) {
        const double ts = timed(runSmall);
        if (ts >= minWindowSeconds || t.reps >= kCpuMaxReps) {
            break;
        }
        const double grow = ts > 0.0 ? std::ceil(1.5 * minWindowSeconds / ts) : 16.0;
        const double next = static_cast<double>(t.reps) * std::clamp(grow, 2.0, 16.0);
        t.reps = static_cast<int>(std::min(next, static_cast<double>(kCpuMaxReps)));
    }
    double leastSmall = std::numeric_limits<double>::infinity();
    double leastLarge = std::numeric_limits<double>::infinity();
    for (int i = 0; i < pairs; ++i) {
        double ts = 0.0;
        double tl = 0.0;
        if (i % 2 == 0) {
            ts = timed(runSmall);
            tl = timed(runLarge);
        } else {
            tl = timed(runLarge);
            ts = timed(runSmall);
        }
        leastSmall = std::min(leastSmall, ts);
        leastLarge = std::min(leastLarge, tl);
        t.pairRatio = std::min(t.pairRatio, tl / std::max(ts, timeFloor));
    }
    t.smallSeconds = leastSmall / static_cast<double>(t.reps);
    t.largeSeconds = leastLarge / static_cast<double>(t.reps);
    return t;
}

/// `interleavedCpuSeconds` repeated up to `attempts` times until `ratio() <= bound`; returns the
/// attempt with the least ratio (T-0270). Load noise is transient and independent between
/// attempts, whereas a genuinely super-linear cost exceeds `bound` in every attempt, so a
/// regression is still caught while a load spike on one attempt is not. The callables are run
/// again each attempt, so whatever they record for the test's other checks is from the last
/// attempt (the work is deterministic, so these are the same in each).
template <class Small, class Large>
ScalingTimes interleavedCpuSecondsRetry(double bound, int pairs, Small&& runSmall, Large&& runLarge,
                                        double timeFloor = kCpuTimeFloor,
                                        double minWindowSeconds = kCpuMinWindow, int attempts = 3) {
    ScalingTimes best;
    for (int a = 0; a < attempts; ++a) {
        const ScalingTimes t = interleavedCpuSeconds(pairs, runSmall, runLarge, timeFloor, minWindowSeconds);
        if (a == 0 || t.ratio() < best.ratio()) {
            best = t;
        }
        if (best.ratio() <= bound) {
            break;
        }
    }
    return best;
}

}  // namespace carvetto::test
