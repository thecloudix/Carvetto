#pragma once

// Exact leftover oracle for the V-carve corner sharpening tests (T-0160). Independent of the
// V-carve code: the depth a toolpath leaves at a point is the deepest tool surface over it,
// found per move in closed form or by a search over a concave function (no sampling along the
// move):
//  - flat-bottomed tools (end mills): the deepest Z of the move over the part of it whose centre
//    lies within the tool radius of the point (an interval of the move, from a quadratic);
//  - V-bits and ball noses: depth(t) = (plane − z(t)) − h(|p − c(t)|), with z linear in t and the
//    tool's height h above its tip convex and increasing in the distance, which is convex in t,
//    so depth(t) is concave over the interval where the tool reaches p; golden-section search
//    to 1e-13 of the move, plus both ends.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::test::v_carve {

/// Height of a tool's cutting surface above its tip at distance `s` from its axis (+inf beyond
/// its radius): V-bit s/tan(θ/2), ball nose R − sqrt(R² − s²), flat bottom 0.
inline double surfaceHeight(const tooling::Tool& tool, double s) {
    const double radius = 0.5 * tool.diameter;
    if (s > radius) {
        return std::numeric_limits<double>::infinity();
    }
    if (tool.type == tooling::ToolType::VBit) {
        return s / std::tan(0.5 * tool.includedAngle);
    }
    if (tool.type == tooling::ToolType::BallNose) {
        return radius - std::sqrt(std::max(0.0, (radius - s) * (radius + s)));
    }
    return 0.0;
}

/// Parameter interval [t0, t1] ⊂ [0, 1] of the move a → b (XY) whose centre lies within `radius`
/// of p; false when there is none.
inline bool reachInterval(geometry::Vec2 a, geometry::Vec2 b, geometry::Vec2 p, double radius, double& t0, double& t1) {
    const geometry::Vec2 d = b - a;
    const geometry::Vec2 f = a - p;
    const double A = d.dot(d);
    const double B = 2.0 * f.dot(d);
    const double C = f.dot(f) - radius * radius;
    if (!(A > 0.0)) {
        if (C > 0.0) {
            return false;
        }
        t0 = 0.0;
        t1 = 1.0;
        return true;
    }
    const double disc = B * B - 4.0 * A * C;
    if (disc < 0.0) {
        return false;
    }
    const double sq = std::sqrt(disc);
    t0 = std::max(0.0, (-B - sq) / (2.0 * A));
    t1 = std::min(1.0, (-B + sq) / (2.0 * A));
    return t0 <= t1;
}

/// Depth below the start plane (at Z = −startDepth) that one straight cutting move a → b leaves
/// at p (−inf when the tool does not reach p).
inline double exactMoveDepth(const tooling::Tool& tool, double startDepth, geometry::Vec3 A, geometry::Vec3 B,
                             geometry::Vec2 p) {
    const double radius = 0.5 * tool.diameter;
    const double plane = -startDepth;
    const geometry::Vec2 a{A.x, A.y};
    const geometry::Vec2 b{B.x, B.y};
    double t0 = 0.0;
    double t1 = 0.0;
    if (!reachInterval(a, b, p, radius, t0, t1)) {
        return -std::numeric_limits<double>::infinity();
    }
    auto depthAt = [&](double t) {
        const geometry::Vec2 c = a + (b - a) * t;
        const double z = A.z + (B.z - A.z) * t;
        // Within [t0, t1] the tool reaches p (the ends lie on its rim, up to rounding).
        return (plane - z) - surfaceHeight(tool, std::min(radius, geometry::distance(p, c)));
    };
    double local = std::max(depthAt(t0), depthAt(t1));
    if (A.z == B.z) {
        // Level: the tool surface rises with the distance, so the deepest point over p is at the
        // closest point of the move (clamped into the reach interval).
        const geometry::Vec2 ab = b - a;
        const double l2 = ab.lengthSquared();
        const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, t0, t1) : t0;
        return std::max(local, depthAt(t));
    }
    if (tool.type == tooling::ToolType::VBit || tool.type == tooling::ToolType::BallNose) {
        // Concave on [t0, t1]: golden-section search for its maximum.
        constexpr double kG = 0.6180339887498949;
        double lo = t0;
        double hi = t1;
        double x1 = hi - kG * (hi - lo);
        double x2 = lo + kG * (hi - lo);
        double f1 = depthAt(x1);
        double f2 = depthAt(x2);
        for (int it = 0; it < 200 && hi - lo > 1e-13; ++it) {
            if (f1 < f2) {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + kG * (hi - lo);
                f2 = depthAt(x2);
            } else {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - kG * (hi - lo);
                f1 = depthAt(x1);
            }
        }
        local = std::max({local, f1, f2});
    }
    return local;
}

/// The cutting moves of a toolpath below its start plane, with their tool, for repeated
/// `depthAt` queries: moves are bucketed in a uniform grid by their box grown by the tool radius,
/// so a query visits only the moves that can reach p.
class CarvedSurface {
   public:
    CarvedSurface(const toolpath::Toolpath& tp, const tooling::Tool& tool, double startDepth)
        : tool_(tool), startDepth_(startDepth) {
        const double radius = 0.5 * tool.diameter;
        double x0 = std::numeric_limits<double>::infinity();
        double y0 = x0;
        double x1 = -x0;
        double y1 = -x0;
        for (std::size_t i = 0; i < tp.moves.size(); ++i) {
            const toolpath::Move& m = tp.moves[i];
            if (!toolpath::isCutting(m.kind)) {
                continue;
            }
            const geometry::Vec3 a = tp.startOf(i);
            const geometry::Vec3 b = m.to;
            if (std::min(a.z, b.z) >= -startDepth) {
                continue;
            }
            M mv{a, b, std::min(a.x, b.x) - radius, std::min(a.y, b.y) - radius, std::max(a.x, b.x) + radius,
                 std::max(a.y, b.y) + radius};
            x0 = std::min(x0, mv.x0);
            y0 = std::min(y0, mv.y0);
            x1 = std::max(x1, mv.x1);
            y1 = std::max(y1, mv.y1);
            moves_.push_back(mv);
        }
        if (moves_.empty()) {
            return;
        }
        ox_ = x0;
        oy_ = y0;
        cell_ = std::max({radius, (x1 - x0) / 512.0, (y1 - y0) / 512.0, 1e-6});
        nx_ = static_cast<std::size_t>((x1 - x0) / cell_) + 1;
        ny_ = static_cast<std::size_t>((y1 - y0) / cell_) + 1;
        cells_.assign(nx_ * ny_, {});
        for (std::size_t k = 0; k < moves_.size(); ++k) {
            const M& m = moves_[k];
            for (std::size_t i = col(m.x0); i <= col(m.x1); ++i) {
                for (std::size_t j = row(m.y0); j <= row(m.y1); ++j) {
                    cells_[j * nx_ + i].push_back(k);
                }
            }
        }
    }
    /// Depth below the start plane left at p (0 when untouched).
    double depthAt(geometry::Vec2 p) const {
        double best = 0.0;
        if (moves_.empty() || p.x < ox_ || p.y < oy_) {
            return best;
        }
        const auto i = static_cast<std::size_t>((p.x - ox_) / cell_);
        const auto j = static_cast<std::size_t>((p.y - oy_) / cell_);
        if (i >= nx_ || j >= ny_) {
            return best;
        }
        for (const std::size_t k : cells_[j * nx_ + i]) {
            const M& m = moves_[k];
            if (p.x < m.x0 || p.x > m.x1 || p.y < m.y0 || p.y > m.y1) {
                continue;
            }
            best = std::max(best, exactMoveDepth(tool_, startDepth_, m.a, m.b, p));
        }
        return best;
    }

   private:
    struct M {
        geometry::Vec3 a;
        geometry::Vec3 b;
        double x0, y0, x1, y1;
    };
    std::size_t col(double x) const { return std::min(nx_ - 1, static_cast<std::size_t>((x - ox_) / cell_)); }
    std::size_t row(double y) const { return std::min(ny_ - 1, static_cast<std::size_t>((y - oy_) / cell_)); }
    tooling::Tool tool_;
    double startDepth_{0.0};
    std::vector<M> moves_;
    double ox_{0.0};
    double oy_{0.0};
    double cell_{1.0};
    std::size_t nx_{0};
    std::size_t ny_{0};
    std::vector<std::vector<std::size_t>> cells_;
};

/// Depth below the start plane (at Z = −startDepth) that the cutting moves of `tp` leave at p
/// (0 when untouched).
inline double exactCarvedDepth(const toolpath::Toolpath& tp, const tooling::Tool& tool, double startDepth,
                               geometry::Vec2 p) {
    return CarvedSurface(tp, tool, startDepth).depthAt(p);
}

}  // namespace carvetto::test::v_carve
