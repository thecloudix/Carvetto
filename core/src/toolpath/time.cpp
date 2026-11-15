#include "carvetto/toolpath/time.hpp"

#include <algorithm>
#include <atomic>
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

namespace {
std::atomic<std::size_t> g_summarized{0};
}

std::size_t summarizeMovesVisited() {
    return g_summarized.load(std::memory_order_relaxed);
}

MoveSummary summarizeMoves(std::span<const Move> moves, const MachineRates& rates, std::optional<Vec3> from) {
    MoveSummary out;
    out.moves = moves.size();
    g_summarized.fetch_add(moves.size(), std::memory_order_relaxed);
    Vec3 prev = from.value_or(moves.empty() ? Vec3{} : moves.front().to);
    for (std::size_t i = 0; i < moves.size(); ++i) {
        const Move& m = moves[i];
        const Vec3 start = i == 0 ? prev : moves[i - 1].to;
        const double len = moveLength(m, start);
        out.bounds.expand(moveBounds(m, start));
        if (!out.minZ || m.to.z < *out.minZ) {
            out.minZ = m.to.z;
        }
        if (!out.maxZ || *out.maxZ < m.to.z) {
            out.maxZ = m.to.z;
        }
        switch (m.kind) {
            case MoveKind::Rapid:
                out.length.rapid += len;
                out.time.rapid += moveTime(len, rates.rapidRate, rates.acceleration);
                break;
            case MoveKind::Feed:
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                out.length.feed += len;
                out.time.feed += moveTime(len, m.feedOverride.value_or(rates.feedRate), rates.acceleration);
                break;
            case MoveKind::Plunge:
                out.length.plunge += len;
                out.time.plunge += moveTime(len, m.feedOverride.value_or(rates.plungeRate), rates.acceleration);
                break;
            case MoveKind::Retract:
                out.length.retract += len;
                out.time.retract += moveTime(len, m.feedOverride.value_or(rates.plungeRate), rates.acceleration);
                break;
            case MoveKind::Dwell:
                out.time.dwell += std::max(m.dwellSeconds, 0.0);
                break;
        }
    }
    return out;
}

}  // namespace carvetto::toolpath
