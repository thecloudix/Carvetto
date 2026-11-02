#pragma once

// Backward-step detection for offset chains (T-0004 / T-0006 tests), written independently of
// the implementation. A toolpath chain must never move backwards: no span starts behind the
// previous one's end, no span sets off straight back along its predecessor (a retrace, whatever
// its length), and no short span runs against both of its neighbours (a step back hidden between
// two spans that go forwards).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry::testing {

// Exact distance from `p` to a line or arc span.
inline double distanceToSpanExact(Vec2 p, const Span& s) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 ab = line->end - line->start;
        const double len2 = ab.lengthSquared();
        const double t = len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0;
        return distance(p, line->start + ab * t);
    }
    const auto& arc = std::get<ArcSpan>(s);
    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
        return std::fabs(distance(p, arc.centre) - arc.radius);
    }
    return std::min(distance(p, arc.startPoint()), distance(p, arc.endPoint()));
}

// Junction a -> b steps backwards: b starts behind a's end along a's direction of travel, or b
// sets off back along a. A near-reversal (tangents within ~0.8 degrees of opposite) is only
// accepted as the tip of a needle whose two sides really diverge: probed at several distances
// from the junction, up to the shorter span's length, each side must stand off the other by
// more than 1e-6 of that distance. A collinear retrace, however short, fails every probe.
inline bool junctionStepsBack(const Span& a, const Span& b, std::string* why = nullptr) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    const Vec2 ta = tangentAt(a, 1.0);
    if ((s - e).dot(ta) < -1e-12) {
        if (why) {
            *why = "next span starts behind the previous end";
        }
        return true;
    }
    if (ta.dot(tangentAt(b, 0.0)) >= -0.9999) {
        return false;
    }
    const double la = length(a);
    const double lb = length(b);
    const double shortest = std::min(la, lb);
    for (const double f : {0.01, 0.1, 0.5, 1.0}) {
        const double delta = f * shortest;
        if (delta <= 0.0) {
            continue;
        }
        const Vec2 pa = pointAt(a, std::max(0.0, 1.0 - delta / la));
        const Vec2 pb = pointAt(b, std::min(1.0, delta / lb));
        const double apart = std::min(distanceToSpanExact(pb, a), distanceToSpanExact(pa, b));
        if (apart <= 1e-6 * delta + 1e-13) {
            if (why) {
                *why = "next span retraces the previous one";
            }
            return true;
        }
    }
    return false;
}

// A span no longer than the tolerance that runs against both neighbours while they go the same
// general way: a backward step between two forward spans (the shape the overlapping chamfers of
// a steep zig-zag produced, T-0006).
inline bool shortSpanRunsBack(const Span& prev, const Span& s, const Span& next, const Tolerance& tol) {
    if (!isDegenerate(s, tol)) {
        return false;
    }
    const Vec2 chord = endPoint(s) - startPoint(s);
    const Vec2 tin = tangentAt(prev, 1.0);
    const Vec2 tout = tangentAt(next, 0.0);
    return tin.dot(tout) > 0.0 && chord.dot(tin) < 0.0 && chord.dot(tout) < 0.0;
}

// Number of backward steps in one chain (every junction, including a closed chain's closing
// junction, and every short span against its two neighbours). `why` receives the first one.
inline std::size_t countBackwardSteps(const Contour& chain, const Tolerance& tol, std::string* why = nullptr) {
    const std::size_t n = chain.spans.size();
    std::size_t count = 0;
    auto note = [&](const std::string& text, std::size_t i) {
        if (why && why->empty()) {
            *why = text + " at span " + std::to_string(i);
        }
    };
    for (std::size_t i = 0; i < n; ++i) {
        const bool hasNext = chain.closed ? n > 1 : i + 1 < n;
        if (hasNext) {
            std::string reason;
            if (junctionStepsBack(chain.spans[i], chain.spans[(i + 1) % n], &reason)) {
                ++count;
                note(reason, i);
            }
        }
        const bool hasPrev = chain.closed ? n > 2 : i > 0;
        if (hasPrev && hasNext &&
            shortSpanRunsBack(chain.spans[(i + n - 1) % n], chain.spans[i], chain.spans[(i + 1) % n], tol)) {
            ++count;
            note("short span runs against both neighbours", i);
        }
    }
    return count;
}

// Stretches that the result traverses twice: pairs of spans (in one chain or in two) lying on
// nearly the same line or circle, within `lateral` of each other, that run alongside each other
// for more than `lateral` (a toolpath would cut them twice). Consecutive spans of one chain
// meeting at their junction are not an overlap; a retrace is (junctionStepsBack catches it too).
// O(n^2): for test-sized results.
inline std::size_t countOverlaps(const std::vector<Contour>& chains, double lateral,
                                 std::string* why = nullptr) {
    struct Item {
        const Span* span;
        std::size_t chain;
        std::size_t index;
    };
    std::vector<Item> items;
    for (std::size_t c = 0; c < chains.size(); ++c) {
        for (std::size_t i = 0; i < chains[c].spans.size(); ++i) {
            items.push_back({&chains[c].spans[i], c, i});
        }
    }
    // Length of the stretch of `a` that lies within `lateral` of `b`, sampled at 64 points.
    auto alongside = [&](const Span& a, const Span& b) {
        int inside = 0;
        for (int k = 0; k < 64; ++k) {
            if (distanceToSpanExact(pointAt(a, (k + 0.5) / 64.0), b) <= lateral) {
                ++inside;
            }
        }
        return length(a) * inside / 64.0;
    };
    std::size_t count = 0;
    for (std::size_t x = 0; x < items.size(); ++x) {
        for (std::size_t y = x + 1; y < items.size(); ++y) {
            const Span& a = *items[x].span;
            const Span& b = *items[y].span;
            if (a.index() != b.index()) {
                continue;
            }
            double shared = 0.0;
            if (const auto* ca = std::get_if<ArcSpan>(&a)) {
                const auto& cb = std::get<ArcSpan>(b);
                if (distance(ca->centre, cb.centre) > lateral ||
                    std::fabs(ca->radius - cb.radius) > lateral) {
                    continue;
                }
                shared = std::max(alongside(a, b), alongside(b, a));
            } else {
                // Lines: nearly the same carrier (b within `lateral` of a's line at both ends),
                // then the overlap of b's projection onto a with a itself.
                const auto& la = std::get<LineSpan>(a);
                const auto& lb = std::get<LineSpan>(b);
                const Vec2 u = (la.end - la.start).normalized();
                if (std::fabs(u.cross(lb.start - la.start)) > lateral ||
                    std::fabs(u.cross(lb.end - la.start)) > lateral) {
                    continue;
                }
                const double s0 = (lb.start - la.start).dot(u);
                const double s1 = (lb.end - la.start).dot(u);
                shared = std::min(length(a), std::max(s0, s1)) - std::max(0.0, std::min(s0, s1));
            }
            const bool arcs = std::holds_alternative<ArcSpan>(a);
            if (shared > 2.0 * lateral + (arcs ? 2.0 / 64.0 * std::max(length(a), length(b)) : 0.0)) {
                ++count;
                if (why && why->empty()) {
                    *why = "chain " + std::to_string(items[x].chain) + " span " +
                           std::to_string(items[x].index) + " overlaps chain " +
                           std::to_string(items[y].chain) + " span " + std::to_string(items[y].index);
                }
            }
        }
    }
    return count;
}

}  // namespace carvetto::geometry::testing
