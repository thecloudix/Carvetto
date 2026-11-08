#pragma once

#include <atomic>
#include <cstdint>

// Deterministic work meter for the toolpath operations (T-0186, ADR-0016 section on the budget).
//
// A meter counts integer work units. Code on the hot paths calls `chargeWork(n)`, which adds n
// units to the meter installed on the calling thread (a no-op when there is none: Pocket, Drill
// and every other caller see no change). When the total passes the limit, or the cancel flag is
// set, `charge` throws `WorkAborted`; the operation's entry point catches it and returns a
// diagnostic and an empty toolpath, so nothing partial escapes.
//
// The count depends only on the input and the parameters (never on time, threads or load), so a
// refusal is reproducible. The cancel flag is polled once per `kPollUnits` units charged, so the
// latency of a cancel is the time to run that many units plus the longest single uncounted call
// (a Clipper2 offset or union of at most a few hundred thousand points).

namespace carvetto::toolpath::detail {

/// Thrown by `WorkMeter::charge`; never leaves the operation that installed the meter.
struct WorkAborted {
    bool cancelled{false};
};

class WorkMeter {
   public:
    /// Units between two polls of the cancel flag.
    static constexpr std::uint64_t kPollUnits = 1024;

    WorkMeter(std::uint64_t limit, const std::atomic<bool>* cancel) : limit_(limit), cancel_(cancel) {}

    /// Adds `units`; throws `WorkAborted` once the limit is passed or a cancel is seen.
    void charge(std::uint64_t units) {
        if (units > limit_ - used_) {  // used_ <= limit_ here, so this cannot wrap
            used_ = limit_;
            throw WorkAborted{false};
        }
        used_ += units;
        if (cancel_ != nullptr && used_ >= nextPoll_) {
            nextPoll_ = used_ + kPollUnits;
            if (cancel_->load(std::memory_order_relaxed)) {
                throw WorkAborted{true};
            }
        }
    }
    /// Polls the cancel flag now (coarse points where no units are charged).
    void poll() const {
        if (cancel_ != nullptr && cancel_->load(std::memory_order_relaxed)) {
            throw WorkAborted{true};
        }
    }
    std::uint64_t used() const { return used_; }
    std::uint64_t limit() const { return limit_; }

   private:
    std::uint64_t limit_;
    const std::atomic<bool>* cancel_;
    std::uint64_t used_{0};
    std::uint64_t nextPoll_{0};
};

/// The meter installed on this thread (null when none).
inline WorkMeter*& currentMeter() {
    thread_local WorkMeter* meter = nullptr;
    return meter;
}

/// Installs `meter` on this thread for the guard's lifetime (restoring the previous one).
class MeterScope {
   public:
    explicit MeterScope(WorkMeter& meter) : previous_(currentMeter()) { currentMeter() = &meter; }
    ~MeterScope() { currentMeter() = previous_; }
    MeterScope(const MeterScope&) = delete;
    MeterScope& operator=(const MeterScope&) = delete;

   private:
    WorkMeter* previous_;
};

/// A work estimate as units: huge or infinite counts to 1e18 (always over any limit); a NaN or
/// negative estimate (a degenerate input the operation deals with itself) counts as one unit.
inline std::uint64_t unitsOf(double estimate) {
    if (estimate >= 1e18) {
        return static_cast<std::uint64_t>(1e18);
    }
    return estimate >= 1.0 ? static_cast<std::uint64_t>(estimate) : std::uint64_t{1};
}

/// Charges `units` to the thread's meter, if any.
inline void chargeWork(std::uint64_t units) {
    if (WorkMeter* m = currentMeter()) {
        m->charge(units);
    }
}

/// Polls the thread's cancel flag, if there is a meter.
inline void pollCancel() {
    if (const WorkMeter* m = currentMeter()) {
        m->poll();
    }
}

}  // namespace carvetto::toolpath::detail
