#include "carvetto/viewmodel/time_readout.hpp"

#include <cmath>
#include <cstdint>

namespace carvetto::viewmodel {

std::string formatDuration(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0) {
        return "unknown";
    }
    // Beyond ~100 years the number is meaningless; avoid integer overflow.
    if (seconds > 3.0e9) {
        return "unknown";
    }
    const std::uint64_t total = static_cast<std::uint64_t>(std::llround(seconds));
    const std::uint64_t h = total / 3600;
    const std::uint64_t m = (total % 3600) / 60;
    const std::uint64_t s = total % 60;
    auto two = [](std::uint64_t v) { return (v < 10 ? "0" : "") + std::to_string(v); };
    if (h > 0) {
        return std::to_string(h) + " h " + two(m) + " min " + two(s) + " s";
    }
    if (m > 0) {
        return std::to_string(m) + " min " + two(s) + " s";
    }
    return std::to_string(s) + " s";
}

namespace {

std::string plural(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

}  // namespace

TimeReadout makeTimeReadout(const JobSummary& s) {
    TimeReadout r;
    if (s.toolpaths == 0) {
        r.headline = "Estimated machining time: no toolpaths";
        return r;
    }
    if (s.included == 0) {
        r.headline = "Estimated machining time: no computed, enabled toolpaths";
        r.incomplete = true;
        if (s.toolpaths > s.computed) {
            r.warnings.push_back(plural(s.toolpaths - s.computed, "toolpath is", "toolpaths are") + " not computed.");
        }
        return r;
    }
    const double total = s.time.total();
    if (!std::isfinite(total)) {
        r.headline = "Estimated machining time: unknown";
        r.incomplete = true;
        r.warnings.push_back("A feed or plunge rate is 0: the time of those moves cannot be estimated.");
    } else {
        r.headline = std::string(s.rapidRateMissing ? "Estimated machining time (without rapids): "
                                                    : "Estimated machining time: ") +
                     formatDuration(total);
        r.incomplete = s.rapidRateMissing;
        r.breakdown = "Cutting " + formatDuration(s.time.feed) + ", plunging " + formatDuration(s.time.plunge) +
                      ", retracting " + formatDuration(s.time.retract) + ", rapids " +
                      (s.rapidRateMissing ? std::string("not included") : formatDuration(s.time.rapid));
        if (s.time.dwell > 0.0) {
            r.breakdown += ", dwell " + formatDuration(s.time.dwell);
        }
    }
    if (s.rapidRateMissing) {
        r.warnings.push_back("The machine's rapid rate is not set: rapid moves are not in the estimate.");
    }
    if (s.needRecalculation > 0) {
        r.warnings.push_back(plural(s.needRecalculation, "toolpath needs", "toolpaths need") +
                             " recalculation: the estimate uses their old moves.");
    }
    if (s.toolpaths > s.computed) {
        r.warnings.push_back(plural(s.toolpaths - s.computed, "toolpath is", "toolpaths are") +
                             " not computed and not in the estimate.");
        r.incomplete = true;
    }
    if (s.toolChanges > 0) {
        r.warnings.push_back("Tool changes (" + std::to_string(s.toolChanges) + ") are not timed (operator time).");
    }
    for (const ToolpathSummaryEntry& e : s.entries) {
        if (!e.included) {
            continue;
        }
        r.perToolpath.push_back(e.name + ": " + formatDuration(e.time.total()) + " (" +
                                plural(e.moves, "move", "moves") + ")");
    }
    return r;
}

}  // namespace carvetto::viewmodel
