#pragma once

// Independent oracles for the simulator tests (T-0222). Nothing here shares code with the
// simulator: tool profiles are written again from their geometric definitions (std::sin/cos/tan,
// no piece tables), and the swept surface is found by brute force - the tool is placed at many
// closely spaced positions along the path and the lowest cutting height at a point is taken.
// Sampling can only miss the true minimum between two samples, so each oracle gives a
// rigorous pair of bounds (see `bounds`): `hi` (minimum over the samples) is never below the true
// swept height, and `lo` (every sample moved by at most half the spacing, as if the tool were
// shrunk and lowered accordingly) is never above it.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <functional>
#include <limits>
#include <vector>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"
#include "carvetto/simulate/simulator.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::test::sim {

using geometry::Vec2;
using geometry::Vec3;
using toolpath::Move;
using toolpath::MoveKind;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kInf = std::numeric_limits<double>::infinity();

/// A cutter as a height function over radius (independent of simulate::ToolShape).
struct Profile {
    double radius{1.0};
    std::function<double(double)> height;  ///< h(r) for 0 <= r <= radius
    double at(double r) const { return r <= radius ? height(r) : kInf; }
};

inline Profile flatProfile(double d) {
    return {d / 2, [](double) { return 0.0; }};
}
inline Profile ballProfile(double d) {
    const double r0 = d / 2;
    return {r0, [r0](double r) { return r0 - std::sqrt(std::max(0.0, r0 * r0 - r * r)); }};
}
inline Profile vProfile(double d, double angle, double tip = 0.0) {
    const double k = 1.0 / std::tan(angle / 2);
    const double t = tip / 2;
    return {d / 2, [k, t](double r) { return r <= t ? 0.0 : (r - t) * k; }};
}
inline Profile taperedProfile(double d, double angle, double ballD) {
    const double a = angle / 2;
    const double rb = ballD / 2;
    const double rt = rb * std::cos(a);
    const double ht = rb - rb * std::sin(a);
    return {d / 2, [=](double r) {
                if (r <= rt) {
                    return rb - std::sqrt(std::max(0.0, rb * rb - r * r));
                }
                return ht + (r - rt) / std::tan(a);
            }};
}

/// Tool positions along a sequence of moves, sampled no further apart than `step` (XY or Z).
struct Samples {
    std::vector<Vec3> p;
    double spacingXY{0.0};  ///< largest XY distance between neighbouring samples (arc length for arcs)

    // Optional bucket index (see `buildIndex`): bucket b holds the sample indices inside it.
    bool indexed{false};
    double ix0{0.0}, iy0{0.0}, ibucket{1.0};
    long inx{0}, iny{0};
    std::vector<std::vector<unsigned>> buckets;

    /// Index the samples in square buckets of size `bucket` (>= tool radius + spacing / 2 makes the
    /// 3 x 3 neighbourhood of a point contain every sample that can matter).
    void buildIndex(double bucket) {
        ibucket = bucket;
        double xmaxs = -1e300, ymaxs = -1e300;
        ix0 = 1e300;
        iy0 = 1e300;
        for (const Vec3& q : p) {
            ix0 = std::min(ix0, q.x);
            iy0 = std::min(iy0, q.y);
            xmaxs = std::max(xmaxs, q.x);
            ymaxs = std::max(ymaxs, q.y);
        }
        if (p.empty()) {
            return;
        }
        inx = static_cast<long>((xmaxs - ix0) / bucket) + 1;
        iny = static_cast<long>((ymaxs - iy0) / bucket) + 1;
        buckets.assign(static_cast<std::size_t>(inx * iny), {});
        for (std::size_t k = 0; k < p.size(); ++k) {
            const long bi = static_cast<long>((p[k].x - ix0) / bucket);
            const long bj = static_cast<long>((p[k].y - iy0) / bucket);
            buckets[static_cast<std::size_t>(bj * inx + bi)].push_back(static_cast<unsigned>(k));
        }
        indexed = true;
    }
};

inline void addSegment(Samples& s, Vec3 a, Vec3 b, double step) {
    const double len = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        s.p.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t});
    }
    s.spacingXY = std::max(s.spacingXY, std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)) / n);
}

inline void addArc(Samples& s, Vec3 a, const Move& m, double step) {
    const double cx = m.centre.x;
    const double cy = m.centre.y;
    const double rho = std::hypot(a.x - cx, a.y - cy);
    const double a0 = std::atan2(a.y - cy, a.x - cx);
    const double a1 = std::atan2(m.to.y - cy, m.to.x - cx);
    double sweep;  // signed
    if (m.kind == MoveKind::ArcCCW) {
        sweep = a1 - a0;
        while (sweep <= 1e-12) sweep += 2 * kPi;
    } else {
        sweep = a1 - a0;
        while (sweep >= -1e-12) sweep -= 2 * kPi;
    }
    const double len = std::fabs(sweep) * rho;
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const double ang = a0 + sweep * t;
        s.p.push_back({cx + rho * std::cos(ang), cy + rho * std::sin(ang), a.z + (m.to.z - a.z) * t});
    }
    s.spacingXY = std::max(s.spacingXY, len / n);
}

/// Samples the cutting moves of a list starting at `start` (rapids and dwells only move the position).
inline Samples sampleMoves(const std::vector<Move>& moves, Vec3 start, double step) {
    Samples s;
    Vec3 cur = start;
    for (const Move& m : moves) {
        switch (m.kind) {
            case MoveKind::Feed:
            case MoveKind::Plunge:
            case MoveKind::Retract:
                addSegment(s, cur, m.to, step);
                break;
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                addArc(s, cur, m, step);
                break;
            default:
                break;
        }
        cur = m.to;
    }
    return s;
}

struct Bounds {
    double lo;
    double hi;
};

/// Bounds of the swept surface height at (x, y) over the sampled path, clamped to [bottom, top].
inline Bounds bounds(const Samples& s, const Profile& tool, double x, double y, double top, double bottom) {
    const double e = s.spacingXY / 2 * 1.0000001;
    double hi = top;
    double lo = top;
    const double reach = tool.radius + e;
    auto visit = [&](std::size_t k) {
        const Vec3& p = s.p[k];
        const double dx = x - p.x;
        const double dy = y - p.y;
        if (std::fabs(dx) > reach || std::fabs(dy) > reach) {
            return;
        }
        const double r = std::sqrt(dx * dx + dy * dy);
        if (r > reach) {
            return;
        }
        hi = std::min(hi, p.z + tool.at(r));
        // Between this sample and the next the tool is within e of one of them, at a height not
        // below the lower of the two.
        const double zl = k + 1 < s.p.size() ? std::min(p.z, s.p[k + 1].z) : p.z;
        const double zl2 = k > 0 ? std::min(zl, s.p[k - 1].z) : zl;
        lo = std::min(lo, zl2 + tool.at(std::max(0.0, r - e)));
    };
    if (s.indexed) {
        const long bi = static_cast<long>(std::floor((x - s.ix0) / s.ibucket));
        const long bj = static_cast<long>(std::floor((y - s.iy0) / s.ibucket));
        for (long j = std::max(0L, bj - 1); j <= std::min(s.iny - 1, bj + 1); ++j) {
            for (long i = std::max(0L, bi - 1); i <= std::min(s.inx - 1, bi + 1); ++i) {
                for (unsigned k : s.buckets[static_cast<std::size_t>(j * s.inx + i)]) {
                    visit(k);
                }
            }
        }
    } else {
        for (std::size_t k = 0; k < s.p.size(); ++k) {
            visit(k);
        }
    }
    hi = std::max(bottom, std::min(top, hi));
    lo = std::max(bottom, std::min(top, lo));
    return {lo, hi};
}

/// Result of comparing a heightfield with the oracle.
struct Deviation {
    double belowLo{0.0};   ///< largest amount by which a cell is lower than the lower bound
    double aboveHi{0.0};   ///< largest amount by which a cell is higher than the upper bound
    double gap{0.0};       ///< largest hi - lo over all cells (the oracle's own width)
    double maxHiMinusSim{0.0};
    std::size_t cells{0};
    std::size_t violations{0};
};

/// Compares every cell (or every `stride`-th in both directions) of `field` with the oracle.
inline Deviation compare(const simulate::Heightfield& f, const Samples& s, const Profile& tool, double tol,
                         std::size_t stride = 1) {
    Deviation d;
    for (std::size_t j = 0; j < f.ny(); j += stride) {
        for (std::size_t i = 0; i < f.nx(); i += stride) {
            const geometry::Vec2 c = f.centreOf(i, j);
            const Bounds b = bounds(s, tool, c.x, c.y, f.topZ(), f.bottomZ());
            const double v = f.height(i, j);
            ++d.cells;
            d.belowLo = std::max(d.belowLo, b.lo - v);
            d.aboveHi = std::max(d.aboveHi, v - b.hi);
            d.gap = std::max(d.gap, b.hi - b.lo);
            d.maxHiMinusSim = std::max(d.maxHiMinusSim, b.hi - v);
            if (v < b.lo - tol || v > b.hi + tol) {
                ++d.violations;
            }
        }
    }
    return d;
}

/// FNV-1a over the bytes of the heights.
inline unsigned long long hashHeights(const simulate::Heightfield& f) {
    unsigned long long h = 1469598103934665603ULL;
    for (float v : f.heights()) {
        unsigned char b[sizeof(float)];
        std::memcpy(b, &v, sizeof(float));
        for (unsigned char c : b) {
            h ^= c;
            h *= 1099511628211ULL;
        }
    }
    return h;
}

}  // namespace carvetto::test::sim
