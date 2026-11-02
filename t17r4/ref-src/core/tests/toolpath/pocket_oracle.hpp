#pragma once

// Independent checking code for the pocket tests (T-0121). Nothing here uses the offset, boolean
// or contour_ops code under test: polygons are plain vertex lists, point-in-polygon is a direct
// even-odd ray count, distances are point/segment formulas, and coverage is a brute-force
// rasterisation of the tool disc swept along every cutting move.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::test::pocket {

using geometry::Vec2;
using geometry::Vec3;
using Poly = std::vector<Vec2>;

inline double segDist(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    double t = l2 > 0.0 ? ((p - a).dot(ab)) / l2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    const Vec2 q{a.x + ab.x * t, a.y + ab.y * t};
    return std::hypot(p.x - q.x, p.y - q.y);
}

/// True when segments ab and cd properly cross or touch.
inline bool segsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    auto orient = [](Vec2 p, Vec2 q, Vec2 r) {
        const double v = (q - p).cross(r - p);
        return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0);
    };
    const int o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    if (o1 != o2 && o3 != o4) {
        return true;
    }
    return (o1 == 0 && segDist(c, a, b) == 0.0) || (o2 == 0 && segDist(d, a, b) == 0.0) ||
           (o3 == 0 && segDist(a, c, d) == 0.0) || (o4 == 0 && segDist(b, c, d) == 0.0);
}

/// Exact minimum distance between two segments.
inline double segSegDist(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    if (segsIntersect(a, b, c, d)) {
        return 0.0;
    }
    return std::min({segDist(a, c, d), segDist(b, c, d), segDist(c, a, b), segDist(d, a, b)});
}

/// Even-odd point in polygon set.
inline bool insideEvenOdd(const std::vector<Poly>& polys, Vec2 p) {
    bool in = false;
    for (const Poly& r : polys) {
        for (std::size_t i = 0, n = r.size(), j = n - 1; i < n; j = i++) {
            if ((r[i].y > p.y) != (r[j].y > p.y)) {
                const double x = r[j].x + (p.y - r[j].y) * (r[i].x - r[j].x) / (r[i].y - r[j].y);
                if (x > p.x) {
                    in = !in;
                }
            }
        }
    }
    return in;
}

/// Distance from p to the nearest edge of any polygon.
inline double boundaryDist(const std::vector<Poly>& polys, Vec2 p) {
    double d = std::numeric_limits<double>::infinity();
    for (const Poly& r : polys) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            d = std::min(d, segDist(p, r[i], r[(i + 1) % n]));
        }
    }
    return d;
}

/// Minimum distance from segment ab to any polygon edge.
inline double segBoundaryDist(const std::vector<Poly>& polys, Vec2 a, Vec2 b) {
    double d = std::numeric_limits<double>::infinity();
    for (const Poly& r : polys) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            d = std::min(d, segSegDist(a, b, r[i], r[(i + 1) % n]));
        }
    }
    return d;
}

inline Poly rectPoly(double x0, double y0, double x1, double y1) {
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

/// A cutting move as a straight 3D segment (pocket toolpaths contain no arcs).
struct Seg3 {
    Vec3 a;
    Vec3 b;
    toolpath::MoveKind kind;
};

/// Every move with its start point; the first move starts at its own end point.
inline std::vector<Seg3> segments(const toolpath::Toolpath& tp) {
    std::vector<Seg3> out;
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        out.push_back({tp.startOf(i), tp.moves[i].to, tp.moves[i].kind});
    }
    return out;
}

/// Regular grid of cell centres x0 + (i + 0.5)·h, y0 + (j + 0.5)·h.
struct Grid {
    double x0{0.0};
    double y0{0.0};
    double h{0.1};
    int nx{0};
    int ny{0};
    std::vector<std::uint8_t> cells;

    Grid(double minX, double minY, double maxX, double maxY, double cell)
        : x0(minX),
          y0(minY),
          h(cell),
          nx(static_cast<int>(std::ceil((maxX - minX) / cell))),
          ny(static_cast<int>(std::ceil((maxY - minY) / cell))),
          cells(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0) {}

    Vec2 centre(int i, int j) const { return {x0 + (i + 0.5) * h, y0 + (j + 0.5) * h}; }
    std::size_t index(int i, int j) const {
        return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i);
    }
    std::uint8_t& at(int i, int j) { return cells[index(i, j)]; }
    std::uint8_t at(int i, int j) const { return cells[index(i, j)]; }

    /// Marks every cell whose centre lies within r of segment ab.
    void sweepDisc(Vec2 a, Vec2 b, double r) {
        const int i0 = std::max(0, static_cast<int>(std::floor((std::min(a.x, b.x) - r - x0) / h)));
        const int i1 = std::min(nx - 1, static_cast<int>(std::ceil((std::max(a.x, b.x) + r - x0) / h)));
        const int j0 = std::max(0, static_cast<int>(std::floor((std::min(a.y, b.y) - r - y0) / h)));
        const int j1 = std::min(ny - 1, static_cast<int>(std::ceil((std::max(a.y, b.y) + r - y0) / h)));
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                if (segDist(centre(i, j), a, b) <= r) {
                    at(i, j) = 1;
                }
            }
        }
    }
};

/// Marks the floor swept at Z = `z` (within 1e-9): every cutting move whose two ends are at z.
inline void sweepFloor(Grid& g, const toolpath::Toolpath& tp, double r, double z) {
    for (const Seg3& s : segments(tp)) {
        if (!toolpath::isCutting(s.kind)) {
            continue;
        }
        if (std::fabs(s.a.z - z) <= 1e-9 && std::fabs(s.b.z - z) <= 1e-9) {
            g.sweepDisc(s.a.xy(), s.b.xy(), r);
        }
    }
}

/// Smallest distance from any cutting move below `surfaceZ` to the boundary edges, and whether
/// every such move starts inside the region (so distance > 0 implies it never leaves it).
struct GougeReport {
    double minDistance{std::numeric_limits<double>::infinity()};
    bool allInside{true};
    bool rapidsAtSafeZ{true};
    std::size_t checkedMoves{0};
};

inline GougeReport gougeCheck(const toolpath::Toolpath& tp, const std::vector<Poly>& boundary,
                              double surfaceZ, double safeZ) {
    GougeReport rep;
    for (const Seg3& s : segments(tp)) {
        if (s.kind == toolpath::MoveKind::Rapid) {
            if (std::fabs(s.b.z - safeZ) > 1e-12) {
                rep.rapidsAtSafeZ = false;
            }
            continue;
        }
        if (std::min(s.a.z, s.b.z) >= surfaceZ) {
            continue;  // in the air
        }
        ++rep.checkedMoves;
        if (!insideEvenOdd(boundary, s.a.xy()) || !insideEvenOdd(boundary, s.b.xy())) {
            rep.allInside = false;
        }
        rep.minDistance = std::min(rep.minDistance, segBoundaryDist(boundary, s.a.xy(), s.b.xy()));
    }
    return rep;
}

}  // namespace carvetto::test::pocket
