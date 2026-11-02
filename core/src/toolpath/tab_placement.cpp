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

// Nearest centre to `c` in [cmin, cmax] (the allowed centre range) whose raised stretch
// [c - w/2, c + w/2] lies within one line span. Returns false when there is no such place.
bool nearestStraight(const ArcLengthPath& path, double c, double w, double cmin, double cmax, double& out) {
    double bestDist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.spanCount(); ++i) {
        if (!std::holds_alternative<geometry::LineSpan>(path.contour().spans[i])) {
            continue;
        }
        const double lo = std::max(cmin, path.spanStart(i) + 0.5 * w);
        const double hi = std::min(cmax, path.spanStart(i) + path.spanLength(i) - 0.5 * w);
        if (hi < lo) {
            continue;
        }
        const double cand = std::clamp(c, lo, hi);
        const double d = std::fabs(cand - c);
        if (d < bestDist) {
            bestDist = d;
            out = cand;
        }
    }
    return bestDist < std::numeric_limits<double>::infinity();
}

}  // namespace

TabPlacement placeTabs(const ArcLengthPath& path, double raisedLength, const TabOptions& options,
                       const std::vector<double>& keepClear, double minGap, double moveMargin) {
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
    // Allowed centres: the whole footprint inside the path, clear of its start (where the tool
    // plunges and the lead arrives) and of an open path's ends.
    const double half = 0.5 * tabFootprint(raisedLength, options);
    const double cmin = half + kMinTabGap;
    const double cmax = len - half - kMinTabGap;
    std::vector<double> centres;
    for (long i = 0; i < n; ++i) {
        double c = (static_cast<double>(i) + 0.5) * len / static_cast<double>(n);
        if (options.avoidCorners) {
            double moved = c;
            if (nearestStraight(path, c, raisedLength, cmin, cmax, moved)) {
                c = moved;
            } else {
                ++result.notAvoided;
            }
        }
        centres.push_back(c);
    }
    TabPlacement fitted =
        fitTabs(path, std::move(centres), tabFootprint(raisedLength, options), keepClear, minGap, moveMargin);
    result.centres = std::move(fitted.centres);
    result.dropped = fitted.dropped;
    result.requested = std::move(fitted.requested);
    result.origin = std::move(fitted.origin);
    return result;
}

double tabFootprint(double raisedLength, const TabOptions& options) {
    return raisedLength + (options.threeD ? 2.0 * std::max(0.0, options.thickness) : 0.0);
}

TabPlacement fitTabs(const ArcLengthPath& path, std::vector<double> centres, double footprint,
                     const std::vector<double>& keepClear, double minGap, double moveMargin) {
    minGap = std::max(minGap, kMinTabGap);
    TabPlacement result;
    result.requested = centres;
    const double len = path.length();
    const double half = 0.5 * footprint;
    const double reach = half + kMinTabGap;  // centre-to-point distance that keeps a point clear
    // A footprint must lie inside the path, clear of its start by kMinTabGap: on a closed path the
    // tool plunges and the lead arrives there; an open path must not lose its ends.
    if (footprint + 2.0 * kMinTabGap > len) {
        result.dropped = centres.size();
        return result;
    }
    const double lo = reach;
    const double hi = len - reach;
    // A centre within `reach` of a keep-clear point is moved to `reach` (+ moveMargin) from it, which
    // may come out a few ulps short; the test allows kKeepClearSlack (1e-9 mm, far below kMinTabGap)
    // so the moved tab is always accepted there instead of kept or dropped by rounding.
    constexpr double kKeepClearSlack = 1e-9;
    auto within = [&](double c, double f) { return std::fabs(c - f) < reach - kKeepClearSlack; };
    auto clashes = [&](double c) {
        return std::any_of(keepClear.begin(), keepClear.end(), [&](double f) { return within(c, f); });
    };
    struct Placed {
        double c;
        std::size_t from;  // index in `centres`
        bool moved;        // moved off a keep-clear point
        bool operator<(const Placed& o) const { return c < o.c || (c == o.c && from < o.from); }
    };
    std::vector<Placed> placed;
    for (std::size_t i = 0; i < centres.size(); ++i) {
        double c = centres[i];
        bool moved = false;
        if (path.closed()) {
            c = std::clamp(c, lo, hi);  // move it off the start
        } else if (c < lo || c > hi) {
            ++result.dropped;  // would run past an open end
            continue;
        }
        // Move it off any other point that must stay clear, to the nearer side.
        for (double f : keepClear) {
            if (within(c, f)) {
                const double away = reach + std::max(0.0, moveMargin);
                c = c < f ? f - away : f + away;
                moved = true;
            }
        }
        if (c < lo || c > hi || clashes(c)) {
            ++result.dropped;  // no room next to the points that must stay clear
            continue;
        }
        placed.push_back({c, i, moved});
    }
    std::sort(placed.begin(), placed.end());
    // Tabs that stayed where they were asked for first (each dropped when it overlaps the previous
    // one kept, or the first across the start); then the tabs moved off keep-clear points, each
    // only where it overlaps none kept, so a moved tab never pushes out one that was not moved.
    std::vector<Placed> kept;
    for (const Placed& t : placed) {
        if (t.moved) continue;
        if (!kept.empty() && t.c - kept.back().c < footprint + minGap) {
            ++result.dropped;  // overlaps (or touches) the previous tab
            continue;
        }
        if (path.closed() && !kept.empty() && kept.front().c + len - t.c < footprint + minGap) {
            ++result.dropped;  // overlaps the first tab across the start
            continue;
        }
        kept.push_back(t);
    }
    for (const Placed& t : placed) {
        if (!t.moved) continue;
        const bool overlaps = std::any_of(kept.begin(), kept.end(), [&](const Placed& o) {
            const double d = std::fabs(t.c - o.c);
            return d < footprint + minGap || (path.closed() && len - d < footprint + minGap);
        });
        if (overlaps) {
            ++result.dropped;  // would overlap a tab already kept
            continue;
        }
        kept.insert(std::upper_bound(kept.begin(), kept.end(), t), t);
    }
    for (const Placed& t : kept) {
        result.centres.push_back(t.c);
        result.origin.push_back(t.from);
    }
    return result;
}

ZProfile tabCeiling(const ArcLengthPath& path, double u0, double u1, const std::vector<double>& centres,
                    double raisedLength, double zTab, double zBottom, bool threeD) {
    std::vector<TabSpan> spans;
    spans.reserve(centres.size());
    for (double c : centres) {
        spans.push_back({c - 0.5 * raisedLength, c + 0.5 * raisedLength});
    }
    return tabCeiling(path, u0, u1, spans, zTab, zBottom, threeD);
}

ZProfile tabCeiling(const ArcLengthPath& path, double u0, double u1, const std::vector<TabSpan>& spans, double zTab,
                    double zBottom, bool threeD) {
    ZProfile ceiling = ZProfile::flat(kNoCeiling);
    const double len = path.length();
    if (spans.empty() || len <= 0.0 || (path.closed() && len < 1e-6)) {
        return ceiling;
    }
    const double dir = u1 >= u0 ? 1.0 : -1.0;
    const double travel = std::fabs(u1 - u0);
    const double ramp = threeD ? std::max(0.0, zTab - zBottom) : 0.0;
    const double lo = std::min(u0, u1);
    const double hi = std::max(u0, u1);
    std::vector<std::pair<double, double>> stretches;  // in travelled distance
    for (const TabSpan& t : spans) {
        long kmin = 0;
        long kmax = 0;
        if (path.closed()) {
            kmin = static_cast<long>(std::floor((lo - t.b - ramp) / len));
            kmax = static_cast<long>(std::ceil((hi - t.a + ramp) / len));
        }
        for (long k = kmin; k <= kmax; ++k) {
            const double shift = static_cast<double>(k) * len;
            const double a = (t.a + shift - u0) * dir;
            const double b = (t.b + shift - u0) * dir;
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
