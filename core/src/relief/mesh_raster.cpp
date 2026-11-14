#include "carvetto/relief/mesh_raster.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace carvetto::relief {
namespace {

struct P {
    double x, y;
};

double rawEdge(P a, P b, P p) { return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x); }

// Evaluated on the canonically ordered endpoints so the two triangles sharing an edge get exactly
// opposite values (no cracks, no double hits).
double edgeValue(P a, P b, P p) {
    if (b.y < a.y || (b.y == a.y && b.x < a.x)) return -rawEdge(b, a, p);
    return rawEdge(a, b, p);
}

// Top-left rule for a point exactly on the directed edge a->b of a CCW triangle.
bool ownsEdge(P a, P b) {
    const double dy = b.y - a.y;
    return dy < 0.0 || (dy == 0.0 && b.x > a.x);
}

struct Tri {
    P v[3];
    double z[3];
    bool up;
    std::size_t rowLo, rowHi;
};

struct Hit {
    std::uint32_t cell;  // column within the row
    double z;
    bool up;
};

}  // namespace

geometry::ShapeResult<MeshRaster> meshToHeightField(const io::TriangleMesh& mesh,
                                                    const MeshRasterOptions& opt) {
    using R = geometry::ShapeResult<MeshRaster>;
    if (mesh.vertices.empty() || mesh.triangles.empty()) return R(std::string("mesh is empty"));
    if (!(opt.cellSize > 0.0) || !std::isfinite(opt.cellSize) || !std::isfinite(opt.zeroPlane))
        return R(std::string("bad cell size or zero plane"));
    for (const auto& v : mesh.vertices)
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
            return R(std::string("mesh has a non-finite vertex"));
    for (const auto& t : mesh.triangles)
        for (std::uint32_t i : t)
            if (i >= mesh.vertices.size()) return R(std::string("mesh triangle index out of range"));

    Grid grid;
    if (opt.grid) {
        grid = *opt.grid;
    } else {
        const io::MeshBounds b = io::meshBounds(mesh);
        const double w = std::ceil((b.max.x - b.min.x) / opt.cellSize - 1e-9);
        const double h = std::ceil((b.max.y - b.min.y) / opt.cellSize - 1e-9);
        const double lim = static_cast<double>(kMaxCells) + 1.0;
        if (!(w < lim) || !(h < lim)) return R(std::string("grid too large"));
        grid.width = static_cast<std::size_t>(std::max(1.0, w));
        grid.height = static_cast<std::size_t>(std::max(1.0, h));
        grid.cellSize = opt.cellSize;
        grid.origin = {b.min.x + opt.cellSize / 2.0, b.min.y + opt.cellSize / 2.0};
    }
    if (validate(grid) != Status::Ok) return R(std::string("invalid or too large grid"));
    const std::size_t W = grid.width, H = grid.height;
    const double c = grid.cellSize;

    // Triangles with their row spans (cell rows whose centre line may cross them).
    std::vector<Tri> tris;
    tris.reserve(mesh.triangles.size());
    for (const auto& t : mesh.triangles) {
        Tri tr;
        for (int k = 0; k < 3; ++k) {
            const io::Vec3& v = mesh.vertices[t[static_cast<std::size_t>(k)]];
            tr.v[k] = {v.x, v.y};
            tr.z[k] = v.z;
        }
        const double area = rawEdge(tr.v[0], tr.v[1], tr.v[2]);
        if (area == 0.0) continue;  // vertical or degenerate: no top-down footprint
        tr.up = area > 0.0;
        if (!tr.up) {
            std::swap(tr.v[1], tr.v[2]);
            std::swap(tr.z[1], tr.z[2]);
        }
        const double ymin = std::min({tr.v[0].y, tr.v[1].y, tr.v[2].y});
        const double ymax = std::max({tr.v[0].y, tr.v[1].y, tr.v[2].y});
        const double lo = std::floor((ymin - grid.origin.y) / c) - 1.0;
        const double hi = std::ceil((ymax - grid.origin.y) / c) + 1.0;
        if (hi < 0.0 || lo > static_cast<double>(H) - 1.0) continue;
        tr.rowLo = lo < 0.0 ? 0 : static_cast<std::size_t>(lo);
        tr.rowHi = hi > static_cast<double>(H) - 1.0 ? H - 1 : static_cast<std::size_t>(hi);
        tris.push_back(tr);
    }
    std::vector<std::uint32_t> order(tris.size());
    std::iota(order.begin(), order.end(), 0u);
    std::stable_sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b) {
        return tris[a].rowLo < tris[b].rowLo;
    });

    MeshRaster out;
    std::vector<float> topData(W * H, kTransparent), botData;
    if (opt.bothSides) botData.assign(W * H, kTransparent);
    if (opt.detectUndercuts) out.undercut.assign(W * H, 0);

    const double zero = opt.zeroPlane;
    double tolScale = 1.0;
    for (const auto& v : mesh.vertices) tolScale = std::max(tolScale, std::fabs(v.z));
    const double tol = 1e-9 * tolScale;

    std::vector<std::uint32_t> active;
    std::size_t nextTri = 0;
    std::vector<Hit> hits;
    // Hits are bucketed per cell with a counting sort so the order is deterministic.
    std::vector<std::uint32_t> counts(W + 1);
    std::vector<Hit> sorted;
    for (std::size_t j = 0; j < H; ++j) {
        if (opt.cancel && opt.cancel()) return R(std::string("cancelled"));
        while (nextTri < order.size() && tris[order[nextTri]].rowLo <= j) active.push_back(order[nextTri++]);
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [&](std::uint32_t t) { return tris[t].rowHi < j; }),
                     active.end());
        if (active.empty()) continue;
        const double py = grid.origin.y + static_cast<double>(j) * c;
        hits.clear();
        for (std::uint32_t ti : active) {
            const Tri& t = tris[ti];
            const double xmin = std::min({t.v[0].x, t.v[1].x, t.v[2].x});
            const double xmax = std::max({t.v[0].x, t.v[1].x, t.v[2].x});
            double lo = std::floor((xmin - grid.origin.x) / c) - 1.0;
            double hi = std::ceil((xmax - grid.origin.x) / c) + 1.0;
            if (hi < 0.0 || lo > static_cast<double>(W) - 1.0) continue;
            const std::size_t i0 = lo < 0.0 ? 0 : static_cast<std::size_t>(lo);
            const std::size_t i1 = hi > static_cast<double>(W) - 1.0 ? W - 1 : static_cast<std::size_t>(hi);
            for (std::size_t i = i0; i <= i1; ++i) {
                const P p{grid.origin.x + static_cast<double>(i) * c, py};
                double w[3];
                bool inside = true;
                for (int k = 0; k < 3 && inside; ++k) {
                    const P a = t.v[k], b = t.v[(k + 1) % 3];
                    // weight of the opposite vertex (k+2)
                    const double e = edgeValue(a, b, p);
                    w[(k + 2) % 3] = e;
                    if (e < 0.0 || (e == 0.0 && !ownsEdge(a, b))) inside = false;
                }
                if (!inside) continue;
                const double sum = w[0] + w[1] + w[2];
                const double z = (w[0] * t.z[0] + w[1] * t.z[1] + w[2] * t.z[2]) / sum;
                hits.push_back({static_cast<std::uint32_t>(i), z, t.up});
            }
        }
        if (hits.empty()) continue;
        std::fill(counts.begin(), counts.end(), 0u);
        for (const Hit& h : hits) ++counts[h.cell + 1];
        for (std::size_t i = 0; i < W; ++i) counts[i + 1] += counts[i];
        sorted.resize(hits.size());
        {
            std::vector<std::uint32_t> pos(counts.begin(), counts.end() - 1);
            for (const Hit& h : hits) sorted[pos[h.cell]++] = h;
        }
        for (std::size_t i = 0; i < W; ++i) {
            const std::size_t b = counts[i], e = counts[i + 1];
            if (b == e) continue;
            std::sort(sorted.begin() + static_cast<std::ptrdiff_t>(b),
                      sorted.begin() + static_cast<std::ptrdiff_t>(e), [](const Hit& x, const Hit& y) {
                          if (x.z != y.z) return x.z > y.z;
                          return x.up && !y.up;
                      });
            const double zTop = sorted[b].z;
            const double zLow = sorted[e - 1].z;
            double ht = zTop - zero;
            if (opt.discardBelowZero && ht < 0.0) ht = 0.0;
            topData[j * W + i] = static_cast<float>(ht);
            out.maxHeight = std::max(out.maxHeight, ht);
            if (opt.bothSides) {
                double d = zero - zLow;
                if (d < 0.0) d = 0.0;  // the bottom side only keeps what is below the plane
                botData[j * W + i] = static_cast<float>(d);
                out.maxDepth = std::max(out.maxDepth, d);
            }
            if (opt.detectUndercuts) {
                // First exit (down-facing hit) below the top surface bounds the top slab.
                std::size_t k = b + 1;
                while (k < e && sorted[k].up) ++k;
                bool uc = false;
                if (k < e) {
                    const double z2 = sorted[k].z;
                    if (opt.discardBelowZero) {
                        uc = z2 > zero + tol;
                    } else {
                        for (std::size_t m = k + 1; m < e; ++m)
                            if (sorted[m].up) {
                                uc = true;
                                break;
                            }
                    }
                }
                if (uc) {
                    out.undercut[j * W + i] = 1;
                    ++out.undercutCount;
                }
            }
        }
    }
    if (HeightField::fromData(grid, std::move(topData), out.top) != Status::Ok)
        return R(std::string("grid too large"));
    if (opt.bothSides && HeightField::fromData(grid, std::move(botData), out.bottom) != Status::Ok)
        return R(std::string("grid too large"));
    return R(std::move(out));
}

}  // namespace carvetto::relief
