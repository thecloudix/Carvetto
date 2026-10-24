#include "carvetto/toolpath/time.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::toolpath {

double moveTime(double distance, double rate, double acceleration) {
    if (distance <= 0.0) {
        return 0.0;
    }
    if (rate <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    const double v = rate / 60.0;
    if (!(acceleration > 0.0) || std::isinf(acceleration)) {
        return distance / v;
    }
    // Distance needed to accelerate to v and brake again: v²/a.
    const double rampDistance = v * v / acceleration;
    if (distance >= rampDistance) {
        return distance / v + v / acceleration;
    }
    return 2.0 * std::sqrt(distance / acceleration);
}

TimeEstimate estimateTime(const Toolpath& toolpath, const MachineRates& rates, std::optional<Vec3> from) {
    TimeEstimate out;
    for (std::size_t i = 0; i < toolpath.moves.size(); ++i) {
        const Move& m = toolpath.moves[i];
        const double len = moveLength(m, toolpath.startOf(i, from));
        switch (m.kind) {
            case MoveKind::Rapid:
                out.rapid += moveTime(len, rates.rapidRate, rates.acceleration);
                break;
            case MoveKind::Feed:
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                out.feed += moveTime(len, m.feedOverride.value_or(rates.feedRate), rates.acceleration);
                break;
            case MoveKind::Plunge:
                out.plunge += moveTime(len, m.feedOverride.value_or(rates.plungeRate), rates.acceleration);
                break;
            case MoveKind::Retract:
                out.retract += moveTime(len, m.feedOverride.value_or(rates.plungeRate), rates.acceleration);
                break;
            case MoveKind::Dwell:
                out.dwell += std::max(m.dwellSeconds, 0.0);
                break;
        }
    }
    return out;
}

TimeEstimate estimateTime(const Toolpath& toolpath, double rapidRate, double feed, double acceleration) {
    return estimateTime(toolpath, MachineRates{rapidRate, feed, feed, acceleration});
}

}  // namespace carvetto::toolpath
