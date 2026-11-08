#pragma once

#include <string>
#include <vector>

#include "carvetto/viewmodel/toolpath_session.hpp"

// The machining-time readout (T-0223, feature-inventory "Time estimation"): text built from the
// session's job summary. Qt-free and pure, so it is tested without a window.

namespace carvetto::viewmodel {

/// "1 h 02 min 03 s", "4 min 07 s", "12 s", "0 s"; "unknown" for a non-finite or negative value.
/// Rounded to whole seconds.
std::string formatDuration(double seconds);

struct TimeReadout {
    /// One line: "Estimated machining time: 4 min 07 s" (or why there is none).
    std::string headline;
    /// Breakdown by move kind ("Cutting 3 min 10 s, plunging 12 s, rapids 45 s"); empty when none.
    std::string breakdown;
    /// One line per included toolpath: "Profile 1: 2 min 03 s (1,204 moves)".
    std::vector<std::string> perToolpath;
    /// Reasons the number is incomplete or out of date: rapid rate not set, a feed rate of 0
    /// (time unknown), toolpaths that need recalculation, toolpaths not computed.
    std::vector<std::string> warnings;
    /// True when the total is an under-estimate or unknown (any warning that affects it).
    bool incomplete{false};
};

/// Readout of `summary`. The total covers the included (enabled, computed) toolpaths, as the summary.
TimeReadout makeTimeReadout(const JobSummary& summary);

}  // namespace carvetto::viewmodel
