#include "carvetto/relief/rotary_raster.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace carvetto::relief {
namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kEps = 1e-9;

struct V3 {
    double a, s, u;  ///< Flat along coordinate, then the two cross-section coordinates.
};

V3 sub(V3 p, V3 q) { return {p.a - q.a, p.s - q.s, p.u - q.u}; }
V3 cross(V3 p, V3 q) { return {p.s * q.u - p.u * q.s, p.u * q.a - p.a * q.u, p.a * q.s - p.s * q.a}; }
double dot(V3 p, V3 q) { return p.a * q.a + p.s * q.s + p.u * q.u; }

double nearAngle(double a, double ref) { return a - kTwoPi * std::round((a - ref) / kTwoPi); }

}  // namespace

geometry::ShapeResult<RotaryRaster> meshToRotaryField(const io::TriangleMesh& mesh, const RotaryRasterOptions& opt) {
    using R = geometry::ShapeResult<RotaryRaster>;
    if (mesh.vertices.empty() || mesh.triangles.empty()) return R(std::string("mesh is empty"));
    if (!(std::isfinite(opt.map.radius) && opt.map.radius > 0.0) || !std::isfinite(opt.axisPoint.x) ||
        !std::isfinite(opt.axisPoint.y) || !std::isfinite(opt.meshAlongOrigin) || !std::isfinite(opt.alongStart) ||
        !std::isfinite(opt.zeroRadius)) {
        return R(std::string("bad cylinder or placement"));
    }
    if (validate(opt.grid) != Status::Ok) return R(std::string("bad grid"));
    for (const auto& v : mesh.vertices) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
            return R(std::string("mesh has a non-finite vertex"));
        }
    }
    for (const auto& t : mesh.triangles) {
        for (const std::uint32_t i : t) {
            if (i >= mesh.vertices.size()) return R(std::string("mesh triangle index out of range"));
        }
    }
    const bool alongX = opt.map.alongX;
    const Grid& g = opt.grid;
    const std::size_t nAlong = alongX ? g.width : g.height;
    const std::size_t nCirc = alongX ? g.height : g.width;
    const double alongOrigin = alongX ? g.origin.x : g.origin.y;
    const double circOrigin = alongX ? g.origin.y : g.origin.x;
    const double cs = g.cellSize;
    const double turn = kTwoPi * opt.map.radius;

    RotaryRaster out;
    if (HeightField::create(g, kTransparent, out.field) != Status::Ok) return R(std::string("grid too large"));
    std::vector<V3> pts;
    pts.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices) {
        const double along = (alongX ? v.x : v.y) - opt.meshAlongOrigin + opt.alongStart;
        const double s = (alongX ? v.y : v.x) - opt.axisPoint.x;
        pts.push_back({along, s, v.z - opt.axisPoint.y});
    }
    std::vector<double> best(nAlong * nCirc, -1.0);  // index circ * nAlong + along; -1 = no hit

    // Rows (circumference indices) for one interval of circumference coordinate.
    const auto rowRange = [&](double cA, double cB, long long& lo, long long& hi) {
        lo = std::max<long long>(0, static_cast<long long>(std::ceil((cA - circOrigin) / cs - kEps)));
        hi = std::min<long long>(static_cast<long long>(nCirc) - 1,
                                 static_cast<long long>(std::floor((cB - circOrigin) / cs + kEps)));
    };
    const double circMin = circOrigin;
    const double circMax = circOrigin + static_cast<double>(nCirc - 1) * cs;

    std::size_t tri = 0;
    for (const auto& t : mesh.triangles) {
        if ((tri++ & 255u) == 0 && opt.cancel && opt.cancel()) return R(std::string("cancelled"));
        const V3 v[3] = {pts[t[0]], pts[t[1]], pts[t[2]]};
        const double aMin = std::min({v[0].a, v[1].a, v[2].a});
        const double aMax = std::max({v[0].a, v[1].a, v[2].a});
        const long long i0 = std::max<long long>(0, static_cast<long long>(std::ceil((aMin - alongOrigin) / cs - kEps)));
        const long long i1 = std::min<long long>(static_cast<long long>(nAlong) - 1,
                                                 static_cast<long long>(std::floor((aMax - alongOrigin) / cs + kEps)));
        if (i0 > i1) continue;
        // Angular range of the triangle seen from the axis.
        bool full = false;  // a triangle round the axis spans pi or more of angle, which sets it below
        double th[3] = {0.0, 0.0, 0.0};
        for (int k = 0; k < 3 && !full; ++k) {
            if (std::hypot(v[k].s, v[k].u) <= 1e-12) {
                full = true;
            } else {
                th[k] = std::atan2(v[k].s, v[k].u);
            }
        }
        double lo = 0.0, hi = 0.0;
        if (!full) {
            const double d1 = nearAngle(th[1], th[0]) - th[0], d2 = nearAngle(th[2], th[0]) - th[0];
            lo = th[0] + std::min({0.0, d1, d2}) - kEps;
            hi = th[0] + std::max({0.0, d1, d2}) + kEps;
            if (hi - lo >= std::numbers::pi - 1e-6) full = true;
        }
        struct Span {
            long long j0, j1;
        };
        Span spans[64];
        std::size_t nSpans = 0;
        if (full) {
            spans[nSpans++] = {0, static_cast<long long>(nCirc) - 1};
        } else {
            double cA = opt.map.circOf(lo), cB = opt.map.circOf(hi);
            if (cA > cB) std::swap(cA, cB);
            const long long nMin = static_cast<long long>(std::floor((circMin - cB) / turn));
            const long long nMax = static_cast<long long>(std::ceil((circMax - cA) / turn));
            for (long long n = nMin; n <= nMax && nSpans < 64; ++n) {
                long long j0 = 0, j1 = 0;
                rowRange(cA + static_cast<double>(n) * turn, cB + static_cast<double>(n) * turn, j0, j1);
                if (j0 <= j1) spans[nSpans++] = {j0, j1};
            }
        }
        std::uint64_t cells = 0;
        for (std::size_t k = 0; k < nSpans; ++k) cells += static_cast<std::uint64_t>(spans[k].j1 - spans[k].j0 + 1);
        cells *= static_cast<std::uint64_t>(i1 - i0 + 1);
        if (out.rayTests + cells > opt.maxRayTests) return R(std::string("the model needs too many ray tests"));
        out.rayTests += cells;
        const V3 e1 = sub(v[1], v[0]), e2 = sub(v[2], v[0]);
        for (std::size_t k = 0; k < nSpans; ++k) {
            for (long long j = spans[k].j0; j <= spans[k].j1; ++j) {
                const double theta = opt.map.angleOf(circOrigin + static_cast<double>(j) * cs);
                const V3 dir{0.0, std::sin(theta), std::cos(theta)};
                const V3 p = cross(dir, e2);
                const double det = dot(e1, p);
                if (std::fabs(det) < 1e-18) continue;
                const double inv = 1.0 / det;
                for (long long i = i0; i <= i1; ++i) {
                    const V3 origin{alongOrigin + static_cast<double>(i) * cs, 0.0, 0.0};
                    const V3 tv = sub(origin, v[0]);
                    const double bu = dot(tv, p) * inv;
                    if (bu < -kEps || bu > 1.0 + kEps) continue;
                    const V3 q = cross(tv, e1);
                    const double bv = dot(dir, q) * inv;
                    if (bv < -kEps || bu + bv > 1.0 + kEps) continue;
                    const double dist = dot(e2, q) * inv;
                    if (dist < -kEps) continue;
                    double& cell = best[static_cast<std::size_t>(j) * nAlong + static_cast<std::size_t>(i)];
                    cell = std::max(cell, dist);
                }
            }
        }
    }
    for (std::size_t j = 0; j < nCirc; ++j) {
        for (std::size_t i = 0; i < nAlong; ++i) {
            const double d = best[j * nAlong + i];
            if (d < 0.0) continue;
            double h = d - opt.zeroRadius;
            if (opt.discardBelowZero && h < 0.0) h = 0.0;
            out.maxRadius = std::max(out.maxRadius, d);
            ++out.hitCells;
            if (alongX) {
                out.field.set(i, j, static_cast<float>(h));
            } else {
                out.field.set(j, i, static_cast<float>(h));
            }
        }
    }
    return out;
}

}  // namespace carvetto::relief
