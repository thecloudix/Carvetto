#pragma once

#include <limits>
#include <optional>

#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::toolpath {

/// Machine rates used for time estimation (feature-inventory §5 "Time estimation").
/// Rates are mm/min as users enter them; acceleration is mm/s². The default acceleration is
/// infinite, i.e. every move runs at its programmed rate from start to end.
struct MachineRates {
    /// Rate of Rapid moves, mm/min.
    double rapidRate{0.0};
    /// Rate of Feed and Arc moves without a feed override, mm/min.
    double feedRate{0.0};
    /// Rate of Plunge and Retract moves without a feed override, mm/min.
    double plungeRate{0.0};
    /// Linear acceleration applied to every move, mm/s². Infinity = instantaneous.
    double acceleration{std::numeric_limits<double>::infinity()};

    /// Rates taken from the toolpath's tool snapshot plus the machine's rapid rate.
    static MachineRates fromTool(const ToolSnapshot& tool, double rapidRate,
                                 double acceleration = std::numeric_limits<double>::infinity()) {
        return {rapidRate, tool.feedRate, tool.plungeRate, acceleration};
    }

    bool operator==(const MachineRates&) const = default;
};

/// Estimated machining time in seconds, split by move kind. Arcs count under `feed`.
struct TimeEstimate {
    double rapid{0.0};
    double feed{0.0};
    double plunge{0.0};
    double retract{0.0};
    double dwell{0.0};

    double total() const { return rapid + feed + plunge + retract + dwell; }
    bool operator==(const TimeEstimate&) const = default;
};

/// Time in seconds to travel `distance` mm at a programmed `rate` (mm/min) with symmetric
/// trapezoidal acceleration `acceleration` (mm/s²), starting and ending at rest.
///
/// With v = rate/60: when the move is long enough to reach v (distance ≥ v²/a) the profile is a
/// trapezoid and t = distance/v + v/a; otherwise it is a triangle and t = 2·sqrt(distance/a).
/// Infinite acceleration gives the plain distance/v. Zero distance gives 0; a non-positive rate
/// gives +infinity for a non-zero distance.
double moveTime(double distance, double rate, double acceleration);

/// Deterministic time estimate for a toolpath. Every move is modelled independently (it starts
/// and ends at rest), so the estimate is conservative relative to a controller that blends
/// moves. `feedOverride` on a cutting move replaces the feed or plunge rate for that move. The
/// first move is measured from `from` when supplied and otherwise has zero length.
TimeEstimate estimateTime(const Toolpath& toolpath, const MachineRates& rates, std::optional<Vec3> from = {});

/// Baseline form from the task card: `feed` is used for every cutting move (plunge and retract
/// included); `acceleration` in mm/s², infinity for instantaneous.
TimeEstimate estimateTime(const Toolpath& toolpath, double rapidRate, double feed,
                          double acceleration = std::numeric_limits<double>::infinity());

}  // namespace carvetto::toolpath
