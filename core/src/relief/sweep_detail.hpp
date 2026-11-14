#pragma once

// Internal helpers shared by the sweep and the extrude/weave generators (T-0288, T-0289).
// Not installed, not part of the public API.

#include <algorithm>
#include <cmath>
#include <vector>

#include "carvetto/relief/sweep.hpp"

namespace carvetto::relief::detail {

// A profile prepared for evaluation over normalised u in [0,1].
struct Prof {
    std::vector<double> u;  // normalised, strictly increasing, u[0]=0, u.back()=1
    std::vector<double> z;
    std::vector<double> d;  // slopes (dz/du normalised) for smooth
    double width = 1.0;
    bool smooth = false;

    void build(const SweepSection& s) {
        const std::size_t n = s.profile.size();
        const double u0 = s.profile.front().x;
        width = s.profile.back().x - u0;
        u.resize(n);
        z.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            u[i] = i + 1 == n ? 1.0 : (s.profile[i].x - u0) / width;
            z[i] = s.profile[i].y;
        }
        u[0] = 0.0;
        smooth = s.smooth && n > 2;
        if (!smooth) return;
        // Fritsch-Carlson monotone cubic Hermite tangents.
        std::vector<double> sec(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i) sec[i] = (z[i + 1] - z[i]) / (u[i + 1] - u[i]);
        d.assign(n, 0.0);
        d[0] = sec[0];
        d[n - 1] = sec[n - 2];
        for (std::size_t i = 1; i + 1 < n; ++i) d[i] = sec[i - 1] * sec[i] > 0 ? 0.5 * (sec[i - 1] + sec[i]) : 0.0;
        for (std::size_t i = 0; i + 1 < n; ++i) {
            if (sec[i] == 0.0) {
                d[i] = 0.0;
                d[i + 1] = 0.0;
                continue;
            }
            const double a = d[i] / sec[i];
            const double b = d[i + 1] / sec[i];
            const double h = a * a + b * b;
            if (h > 9.0) {
                const double tau = 3.0 / std::sqrt(h);
                d[i] = tau * a * sec[i];
                d[i + 1] = tau * b * sec[i];
            }
        }
    }

    double eval(double x) const {
        const std::size_t n = u.size();
        std::size_t i = static_cast<std::size_t>(std::upper_bound(u.begin(), u.end(), x) - u.begin());
        i = i == 0 ? 0 : i - 1;
        if (i + 1 >= n) i = n - 2;
        const double h = u[i + 1] - u[i];
        const double t = std::clamp((x - u[i]) / h, 0.0, 1.0);
        if (!smooth) return z[i] + (z[i + 1] - z[i]) * t;
        const double t2 = t * t;
        const double t3 = t2 * t;
        return (2 * t3 - 3 * t2 + 1) * z[i] + (t3 - 2 * t2 + t) * h * d[i] + (-2 * t3 + 3 * t2) * z[i + 1] +
               (t3 - t2) * h * d[i + 1];
    }
};

struct V3 {
    double x, y, z;
};

inline constexpr double kEps = 1e-9;

inline void raster(HeightField& f, const V3& a, const V3& b, const V3& c) {
    const Grid& g = f.grid();
    const double det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    const double scale = std::max({std::fabs(b.x - a.x), std::fabs(c.x - a.x), std::fabs(b.y - a.y), std::fabs(c.y - a.y)});
    if (!(std::fabs(det) > 1e-12 * scale * scale) || !(scale > 0.0)) return;
    const double minx = std::min({a.x, b.x, c.x});
    const double maxx = std::max({a.x, b.x, c.x});
    const double miny = std::min({a.y, b.y, c.y});
    const double maxy = std::max({a.y, b.y, c.y});
    const double pad = kEps * g.cellSize;
    auto lo = [&](double v, double o) { return std::max(0.0, std::ceil((v - pad - o) / g.cellSize)); };
    auto hi = [&](double v, double o, std::size_t n) {
        return std::min(static_cast<double>(n) - 1.0, std::floor((v + pad - o) / g.cellSize));
    };
    const double i0 = lo(minx, g.origin.x);
    const double i1 = hi(maxx, g.origin.x, g.width);
    const double j0 = lo(miny, g.origin.y);
    const double j1 = hi(maxy, g.origin.y, g.height);
    for (double jj = j0; jj <= j1; jj += 1.0) {
        const double py = g.origin.y + jj * g.cellSize;
        for (double ii = i0; ii <= i1; ii += 1.0) {
            const double px = g.origin.x + ii * g.cellSize;
            const double l1 = ((b.y - c.y) * (px - c.x) + (c.x - b.x) * (py - c.y)) / det;
            const double l2 = ((c.y - a.y) * (px - c.x) + (a.x - c.x) * (py - c.y)) / det;
            const double l3 = 1.0 - l1 - l2;
            if (l1 < -kEps || l2 < -kEps || l3 < -kEps) continue;
            const float v = static_cast<float>(l1 * a.z + l2 * b.z + l3 * c.z);
            const std::size_t i = static_cast<std::size_t>(ii);
            const std::size_t j = static_cast<std::size_t>(jj);
            const float old = f.at(i, j);
            if (std::isnan(old) || v > old) f.set(i, j, v);
        }
    }
}


}  // namespace carvetto::relief::detail
