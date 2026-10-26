#include "carvetto/toolpath/tab_placement.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"

namespace carvetto::toolpath {

namespace {

// Nearest centre to `c` whose raised stretch [c - w/2, c + w/2] lies within one line span.
// Returns false when no line span is long enough.
bool nearestStraight(const ArcLengthPath& path, double c, double w, double& out) {
    const double len = path.length();
    double bestDist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.spanCount(); ++i) {
        if (!std::holds_alternative<geometry::LineSpan>(path.contour().spans[i])) {
            continue;
        }
        const double lo = path.spanStart(i) + 0.5 * w;
        const double hi = path.spanStart(i) + path.spanLength(i) - 0.5 * w;
        if (hi < lo) {
            continue;
        }
        const int copies = path.closed() ? 1 : 0;
        for (int k = -copies; k <= copies; ++k) {
            const double shift = k * len;
            const double cand = std::clamp(c, lo + shift, hi + shift);
            const double d = std::fabs(cand - c);
            if (d < bestDist) {
                bestDist = d;
                out = path.closed() ? path.wrap(cand) : cand;
            }
        }
    }
    return bestDist < std::numeric_limits<double>::infinity();
}

}  // namespace

TabPlacement placeTabs(const ArcLengthPath& path, double raisedLength, const TabOptions& options) {
    TabPlacement result;
    const double len = path.length();
    if (!options.enabled || path.isEmpty() || len <= 0.0 || raisedLength <= 0.0) {
        return result;
    }
    long n = 0;
    if (options.spacing == TabSpacing::ConstantNumber) {
        n = std::max(0, options.count);
    } else if (options.distance > 0.0) {
        n = std::max(1L, std::lround(len / options.distance));
    }
    if (n == 0) {
        return result;
    }
    std::vector<double> centres;
    for (long i = 0; i < n; ++i) {
        double c = (static_cast<double>(i) + 0.5) * len / static_cast<double>(n);
        if (options.avoidCorners) {
            double moved = c;
            if (nearestStraight(path, c, raisedLength, moved)) {
                c = moved;
            } else {
                ++result.notAvoided;
            }
        }
        centres.push_back(c);
    }
    std::sort(centres.begin(), centres.end());
    // Drop tabs that overlap an earlier one or do not fit an open path.
    for (double c : centres) {
        if (!path.closed() && (c - 0.5 * raisedLength < 0.0 || c + 0.5 * raisedLength > len)) {
            ++result.dropped;
            continue;
        }
        if (!result.centres.empty() && c - result.centres.back() < raisedLength) {
            ++result.dropped;
            continue;
        }
        if (path.closed() && !result.centres.empty() && result.centres.front() + len - c < raisedLength) {
            ++result.dropped;
            continue;
        }
        result.centres.push_back(c);
    }
    return result;
}

ZProfile tabCeiling(const ArcLengthPath& path, double u0, double u1, const std::vector<double>& centres,
                    double raisedLength, double zTab, double zBottom, bool threeD) {
    ZProfile ceiling = ZProfile::flat(kNoCeiling);
    const double len = path.length();
    if (centres.empty() || len <= 0.0 || (path.closed() && len < 1e-6)) {
        return ceiling;
    }
    const double dir = u1 >= u0 ? 1.0 : -1.0;
    const double travel = std::fabs(u1 - u0);
    const double ramp = threeD ? std::max(0.0, zTab - zBottom) : 0.0;
    const double reach = 0.5 * raisedLength + ramp;
    const double lo = std::min(u0, u1) - reach;
    const double hi = std::max(u0, u1) + reach;
    std::vector<std::pair<double, double>> stretches;  // in travelled distance
    for (double c : centres) {
        long kmin = 0;
        long kmax = 0;
        if (path.closed()) {
            kmin = static_cast<long>(std::floor((lo - c) / len));
            kmax = static_cast<long>(std::ceil((hi - c) / len));
        }
        for (long k = kmin; k <= kmax; ++k) {
            const double cc = c + static_cast<double>(k) * len;
            const double a = (cc - 0.5 * raisedLength - u0) * dir;
            const double b = (cc + 0.5 * raisedLength - u0) * dir;
            const double s0 = std::min(a, b);
            const double s1 = std::max(a, b);
            if (s1 + ramp < 0.0 || s0 - ramp > travel) {
                continue;
            }
            stretches.push_back({s0, s1});
        }
    }
    std::sort(stretches.begin(), stretches.end());
    for (const auto& [s0, s1] : stretches) {
        ZProfile tab;
        if (ramp > 0.0) {
            tab.knots = {ZKnot{s0 - ramp, kNoCeiling, zBottom}, ZKnot{s0, zTab, zTab}, ZKnot{s1, zTab, zTab},
                         ZKnot{s1 + ramp, zBottom, kNoCeiling}};
        } else {
            tab.knots = {ZKnot{s0, kNoCeiling, zTab}, ZKnot{s1, zTab, kNoCeiling}};
        }
        ceiling = ZProfile::max(ceiling, tab);
    }
    return ceiling;
}

}  // namespace carvetto::toolpath
