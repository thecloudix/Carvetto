#pragma once

// Independent checks for the V-carve tests (T-0153). Nothing here uses the medial axis or the
// V-carve code under test: distances are exact closest points on every boundary span (lines,
// arcs, Beziers), the tool shape comes from `tooling::cuttingWidthAtDepth`, and every move is
// checked over its whole length by certified subdivision (the boundary distance is 1-Lipschitz
// along the move and the footprint is monotone in depth, so each interval has a lower bound).
//
// - `coneCheck`: for every non-rapid move below the start plane, the smallest value over the
//   move of dist(centre, boundary) − footprint(depth), the footprint being half the tool's
//   cutting width at that depth below the start plane (the tool's cross-section there; the tool
//   is narrower below). >= −eps means the tool never crosses a wall, and the centre stays inside.
// - `carvedDepth`: the depth (below the start plane) the moves leave at a point: the deepest
//   tool surface over that point, sampled densely along every move.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::test::vcarve {

using geometry::Contour;
using geometry::Vec2;
using geometry::Vec3;

/// Exact distance from p to the nearest boundary span.
inline double boundaryDistance(const std::vector<Contour>& contours, Vec2 p) {
    double best = std::numeric_limits<double>::infinity();
    for (const Contour& c : contours) {
        if (!c.closed) {
            continue;
        }
        for (const geometry::Span& s : c.spans) {
            best = std::min(best, geometry::closestPoint(s, p).distance);
        }
    }
    return best;
}

/// Even-odd inside test over the closed contours (points on a boundary count as inside).
inline bool insideRegion(const std::vector<Contour>& contours, Vec2 p) {
    const geometry::Tolerance tol;
    bool in = false;
    for (const Contour& c : contours) {
        if (!c.closed || c.isEmpty()) {
            continue;
        }
        const auto loc = geometry::locatePoint(c, p, geometry::FillRule::EvenOdd, tol);
        if (loc == geometry::PointLocation::OnBoundary) {
            return true;
        }
        if (loc == geometry::PointLocation::Inside) {
            in = !in;
        }
    }
    return in;
}

inline double footprintAt(const tooling::Tool& tool, double depthBelowStart) {
    return 0.5 * tooling::cuttingWidthAtDepth(tool, std::max(0.0, depthBelowStart));
}

struct ConeReport {
    /// Smallest dist − footprint over every checked move (+inf when nothing was checked).
    double minClearance{std::numeric_limits<double>::infinity()};
    Vec3 worstAt{};
    std::size_t worstMove{0};
    std::size_t checkedMoves{0};
    bool allInside{true};
    bool rapidsAtSafeZ{true};
};

/// Exact cone/wall check (see the file comment). Moves are straight (V-carve and Pocket emit no
/// arcs); `eps` is the resolution of the certified bound.
inline ConeReport coneCheck(const toolpath::Toolpath& tp, const std::vector<Contour>& region,
                            const tooling::Tool& tool, double startDepth, double safeZ, double eps) {
    ConeReport rep;
    const double plane = -startDepth;
    struct Sample {
        double t;
        Vec3 p;
        double dist;
        double rho;
    };
    auto sample = [&](Vec3 a, Vec3 b, double t) {
        const Vec3 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
        const Vec2 xy{p.x, p.y};
        return Sample{t, p, boundaryDistance(region, xy), footprintAt(tool, plane - p.z)};
    };
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const toolpath::Move& m = tp.moves[i];
        const Vec3 a = tp.startOf(i);
        const Vec3 b = m.to;
        if (m.kind == toolpath::MoveKind::Rapid) {
            if (std::fabs(b.z - safeZ) > 1e-9) {
                rep.rapidsAtSafeZ = false;
            }
            continue;
        }
        if (std::min(a.z, b.z) >= plane) {
            continue;  // above the start plane
        }
        ++rep.checkedMoves;
        const double len = std::hypot(b.x - a.x, b.y - a.y);
        std::vector<std::pair<Sample, Sample>> stack;
        stack.push_back({sample(a, b, 0.0), sample(a, b, 1.0)});
        for (const Sample& s : {stack.back().first, stack.back().second}) {
            if (s.rho > 0.0 && !insideRegion(region, {s.p.x, s.p.y})) {
                rep.allInside = false;
            }
        }
        int guard = 0;
        while (!stack.empty() && guard < 200000) {
            ++guard;
            const auto [s0, s1] = stack.back();
            stack.pop_back();
            for (const Sample& s : {s0, s1}) {
                const double c = s.dist - s.rho;
                if (c < rep.minClearance) {
                    rep.minClearance = c;
                    rep.worstAt = s.p;
                    rep.worstMove = i;
                }
            }
            const double dt = s1.t - s0.t;
            const double lower = 0.5 * (s0.dist + s1.dist - len * dt) - std::max(s0.rho, s1.rho);
            if (lower >= -eps) {
                continue;
            }
            if (len * dt <= eps * 0.25 && std::fabs(s1.rho - s0.rho) <= eps * 0.25) {
                continue;  // resolved: the endpoints carry the value within eps / 2
            }
            const Sample mid = sample(a, b, 0.5 * (s0.t + s1.t));
            stack.push_back({s0, mid});
            stack.push_back({mid, s1});
        }
    }
    return rep;
}

/// Height of the tool surface above its tip at radius s from its axis (infinite outside it).
inline double toolHeightAt(const tooling::Tool& tool, double s) {
    const double radius = 0.5 * tool.diameter;
    if (s > radius) {
        return std::numeric_limits<double>::infinity();
    }
    if (tool.type == tooling::ToolType::VBit) {
        return s / std::tan(0.5 * tool.includedAngle);
    }
    if (tool.type == tooling::ToolType::BallNose) {
        return radius - std::sqrt(std::max(0.0, radius * radius - s * s));
    }
    return 0.0;  // flat bottom (end mill)
}

/// Depth below the start plane the cutting moves of `tp` leave at p (0 when untouched), each
/// move sampled every `step` mm.
inline double carvedDepth(const toolpath::Toolpath& tp, const tooling::Tool& tool, double startDepth, Vec2 p,
                          double step) {
    double best = 0.0;
    const double radius = 0.5 * tool.diameter;
    const double plane = -startDepth;
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const toolpath::Move& m = tp.moves[i];
        if (!toolpath::isCutting(m.kind)) {
            continue;
        }
        const Vec3 a = tp.startOf(i);
        const Vec3 b = m.to;
        if (std::min(a.z, b.z) >= plane) {
            continue;
        }
        // Skip moves that pass farther than the tool radius from p.
        const Vec2 ab{b.x - a.x, b.y - a.y};
        const double l2 = ab.lengthSquared();
        double tc = l2 > 0.0 ? ((p - Vec2{a.x, a.y}).dot(ab)) / l2 : 0.0;
        tc = std::clamp(tc, 0.0, 1.0);
        if (geometry::distance(p, Vec2{a.x, a.y} + ab * tc) > radius) {
            continue;
        }
        const double len = std::sqrt(l2);
        const auto n = static_cast<std::size_t>(std::ceil(len / step)) + 1;
        for (std::size_t k = 0; k <= n; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(n);
            const Vec2 c{a.x + ab.x * t, a.y + ab.y * t};
            const double z = a.z + (b.z - a.z) * t;
            const double h = toolHeightAt(tool, geometry::distance(p, c));
            if (std::isfinite(h)) {
                best = std::max(best, (plane - z) - h);
            }
        }
    }
    return best;
}

}  // namespace carvetto::test::vcarve
