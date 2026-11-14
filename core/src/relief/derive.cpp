#include "carvetto/relief/derive.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <utility>

namespace carvetto::relief {

using geometry::Polyline;
using geometry::Vec2;

namespace {

bool opaqueV(float v) { return !std::isnan(v); }
bool cancelled(const std::atomic<bool>* c) { return c != nullptr && c->load(std::memory_order_relaxed); }
bool finite(double v) { return std::isfinite(v); }

DeriveStatus gridStatus(const Grid& g) {
    return validate(g) == Status::Ok ? DeriveStatus::Ok : DeriveStatus::BadGrid;
}

}  // namespace

const char* deriveStatusMessage(DeriveStatus s) {
    switch (s) {
        case DeriveStatus::Ok: return "ok";
        case DeriveStatus::EmptyModel: return "nothing opaque to work on";
        case DeriveStatus::BadParameter: return "invalid parameter";
        case DeriveStatus::BadVector: return "invalid vector";
        case DeriveStatus::BadGrid: return "invalid or too large grid";
        case DeriveStatus::TooMany: return "too many slices or samples";
        case DeriveStatus::Cancelled: return "cancelled";
    }
    return "unknown";
}

// ---- Contours -------------------------------------------------------------------------------

std::vector<Polyline> opaqueContours(const HeightField& field) {
    std::vector<Polyline> result;
    if (field.empty()) return result;
    const long long w = static_cast<long long>(field.width());
    const long long h = static_cast<long long>(field.height());
    const long long stride = 2 * w + 8;
    // Doubled lattice coordinates (kx, ky) shifted by +2 so keys are non-negative.
    const auto key = [&](long long kx, long long ky) { return (ky + 2) * stride + (kx + 2); };
    const auto val = [&](long long i, long long j) {
        return i >= 0 && j >= 0 && i < w && j < h && opaqueV(field.at(static_cast<std::size_t>(i), static_cast<std::size_t>(j)));
    };
    struct Seg {
        long long end;
        bool used;
    };
    std::map<long long, Seg> segs;
    for (long long j = -1; j < h; ++j) {
        for (long long i = -1; i < w; ++i) {
            const bool corner[4] = {val(i, j), val(i + 1, j), val(i + 1, j + 1), val(i, j + 1)};
            if (corner[0] == corner[1] && corner[1] == corner[2] && corner[2] == corner[3]) continue;
            // Edge crossing keys, CCW from the bottom edge: bottom, right, top, left.
            const long long mid[4] = {key(2 * i + 1, 2 * j), key(2 * i + 2, 2 * j + 1), key(2 * i + 1, 2 * j + 2),
                                      key(2 * i, 2 * j + 1)};
            for (int e = 0; e < 4; ++e) {
                if (!(corner[e] && !corner[(e + 1) % 4])) continue;  // in -> out crossing starts a segment
                for (int k = 1; k <= 4; ++k) {
                    const int f = (e + k) % 4;
                    if (!corner[f] && corner[(f + 1) % 4]) {  // next out -> in crossing ends it
                        segs[mid[e]] = Seg{mid[f], false};
                        break;
                    }
                }
            }
        }
    }
    const double cs = field.cellSize();
    const auto toPoint = [&](long long k) {
        const long long kx = k % stride - 2, ky = k / stride - 2;
        return std::pair<long long, long long>{kx, ky};
    };
    for (auto& entry : segs) {
        if (entry.second.used) continue;
        std::vector<std::pair<long long, long long>> loop;
        long long cur = entry.first;
        while (true) {
            auto it = segs.find(cur);
            if (it == segs.end() || it->second.used) break;
            it->second.used = true;
            loop.push_back(toPoint(cur));
            cur = it->second.end;
        }
        const std::size_t n = loop.size();
        Polyline p;
        p.closed = true;
        for (std::size_t k = 0; k < n; ++k) {
            const auto& a = loop[(k + n - 1) % n];
            const auto& b = loop[k];
            const auto& c = loop[(k + 1) % n];
            const long long cr = (b.first - a.first) * (c.second - b.second) - (b.second - a.second) * (c.first - b.first);
            if (cr == 0) continue;
            p.points.push_back({field.origin().x + static_cast<double>(b.first) * 0.5 * cs,
                                field.origin().y + static_cast<double>(b.second) * 0.5 * cs});
        }
        if (p.points.size() >= 3) result.push_back(std::move(p));
    }
    return result;
}

// ---- Slice Model ----------------------------------------------------------------------------

DeriveStatus sliceModel(const HeightField& model, const SliceParams& params, const std::atomic<bool>* cancel,
                        std::vector<Slice>& out) {
    out.clear();
    if (!finite(params.thickness) || params.thickness <= 0.0) return DeriveStatus::BadParameter;
    if (params.count > kMaxSlices) return DeriveStatus::TooMany;
    float flo = 0.f, fhi = 0.f;
    if (model.empty() || !model.minMax(flo, fhi)) return DeriveStatus::EmptyModel;
    const double lo = flo, hi = fhi, t = params.thickness;
    std::size_t n = params.count;
    if (n == 0) {
        const double need = std::ceil((hi - lo) / t - 1e-9);
        if (need > static_cast<double>(kMaxSlices)) return DeriveStatus::TooMany;
        n = need < 1.0 ? 1 : static_cast<std::size_t>(need);
    }
    std::vector<Slice> slices;
    for (std::size_t k = 0; k < n; ++k) {
        if (cancelled(cancel)) return DeriveStatus::Cancelled;
        Slice s;
        const double kd = static_cast<double>(k);
        if (params.fromTop) {
            s.high = hi - kd * t;
            s.low = s.high - t;
        } else {
            s.low = lo + kd * t;
            s.high = s.low + t;
        }
        // The band that floors the model keeps every cell below it: always the bottom band from the
        // bottom, and the last band from the top when the count covers the whole model.
        const bool floorBand = params.fromTop ? (params.count == 0 && k + 1 == n) : (k == 0);
        HeightField f;
        HeightField::create(model.grid(), kTransparent, f);
        for (std::size_t j = 0; j < model.height(); ++j) {
            for (std::size_t i = 0; i < model.width(); ++i) {
                const float v = model.at(i, j);
                if (!opaqueV(v)) continue;
                const double d = v;
                if (!floorBand && !(d > s.low)) continue;
                f.set(i, j, static_cast<float>(std::min(d, s.high) - s.low));
            }
        }
        s.component.name = "Slice " + std::to_string(k + 1);
        s.component.field = std::move(f);
        s.component.mode = CombineMode::MergeHigh;
        s.component.baseHeight = s.low;
        if (params.boundaryVectors) s.boundary = opaqueContours(s.component.field);
        slices.push_back(std::move(s));
    }
    out = std::move(slices);
    return DeriveStatus::Ok;
}

// ---- Add Draft ------------------------------------------------------------------------------

DeriveStatus addDraft(const HeightField& model, double minDraftRad, const std::atomic<bool>* cancel, Component& out) {
    out = Component{};
    if (!finite(minDraftRad) || minDraftRad < 0.0 || minDraftRad >= 1.5707963267948966) return DeriveStatus::BadParameter;
    float flo = 0.f, fhi = 0.f;
    if (model.empty() || !model.minMax(flo, fhi)) return DeriveStatus::EmptyModel;
    const long long w = static_cast<long long>(model.width());
    const long long h = static_cast<long long>(model.height());
    std::vector<double> v(model.data().size());
    for (std::size_t k = 0; k < v.size(); ++k) v[k] = model.data()[k];
    if (minDraftRad > 0.0) {
        const double s = 1.0 / std::tan(minDraftRad);
        struct Off {
            int dx, dy;
            double drop;
        };
        std::vector<Off> fwd, bwd;  // fwd: neighbours earlier in raster order
        const int moves[16][2] = {{-1, 0}, {1, 0},  {0, -1}, {0, 1},  {-1, -1}, {1, -1}, {-1, 1}, {1, 1},
                                  {-2, -1}, {-2, 1}, {2, -1}, {2, 1}, {-1, -2}, {1, -2}, {-1, 2}, {1, 2}};
        for (const auto& m : moves) {
            const double len = std::sqrt(static_cast<double>(m[0] * m[0] + m[1] * m[1])) * model.cellSize();
            const Off o{m[0], m[1], s * len};
            if (m[1] < 0 || (m[1] == 0 && m[0] < 0)) fwd.push_back(o); else bwd.push_back(o);
        }
        const auto relax = [&](long long i, long long j, const std::vector<Off>& offs) {
            double& cur = v[static_cast<std::size_t>(j * w + i)];
            if (std::isnan(cur)) return;
            for (const Off& o : offs) {
                const long long ni = i + o.dx, nj = j + o.dy;
                if (ni < 0 || nj < 0 || ni >= w || nj >= h) continue;
                const double nv = v[static_cast<std::size_t>(nj * w + ni)];
                if (std::isnan(nv)) continue;
                cur = std::max(cur, nv - o.drop);
            }
        };
        for (long long j = 0; j < h; ++j) {
            if (cancelled(cancel)) return DeriveStatus::Cancelled;
            for (long long i = 0; i < w; ++i) relax(i, j, fwd);
        }
        for (long long j = h - 1; j >= 0; --j) {
            if (cancelled(cancel)) return DeriveStatus::Cancelled;
            for (long long i = w - 1; i >= 0; --i) relax(i, j, bwd);
        }
    }
    HeightField f;
    HeightField::create(model.grid(), kTransparent, f);
    for (std::size_t k = 0; k < v.size(); ++k) {
        if (!std::isnan(v[k])) f.set(k % model.width(), k / model.width(), static_cast<float>(v[k]));
    }
    out.name = "Draft";
    out.field = std::move(f);
    out.mode = CombineMode::Add;
    return DeriveStatus::Ok;
}

// ---- Split Component ------------------------------------------------------------------------

namespace {

bool insidePolygon(const std::vector<Vec2>& pts, double x, double y) {
    bool in = false;
    for (std::size_t a = 0, b = pts.size() - 1; a < pts.size(); b = a++) {
        const Vec2 p = pts[a], q = pts[b];
        if ((p.y > y) != (q.y > y) && x < (q.x - p.x) * (y - p.y) / (q.y - p.y) + p.x) in = !in;
    }
    return in;
}

}  // namespace

DeriveStatus splitComponent(const Component& c, const Polyline& cutter, Component& a, Component& b) {
    a = Component{};
    b = Component{};
    if (c.field.empty()) return DeriveStatus::EmptyModel;
    const std::size_t np = cutter.points.size();
    if (np < 2 || (cutter.closed && np < 3)) return DeriveStatus::BadVector;
    for (const Vec2& p : cutter.points) {
        if (!finite(p.x) || !finite(p.y)) return DeriveStatus::BadVector;
    }
    const std::size_t nseg = cutter.closed ? np : np - 1;
    for (std::size_t k = 0; k < nseg; ++k) {
        const Vec2 d = cutter.points[(k + 1) % np] - cutter.points[k];
        if (!(d.x * d.x + d.y * d.y > 0.0)) return DeriveStatus::BadVector;
    }
    const std::size_t cells = c.field.data().size();
    if (static_cast<double>(cells) * static_cast<double>(nseg) > 2147483648.0) return DeriveStatus::TooMany;

    std::vector<Vec2> pts = cutter.points;
    if (!cutter.closed) {
        // Extend both ends far beyond every cell so the open vector divides the whole plane.
        const Grid& g = c.field.grid();
        double reach = std::abs(g.origin.x) + std::abs(g.origin.y) +
                       static_cast<double>(g.width + g.height) * g.cellSize;
        for (const Vec2& p : pts) reach += std::abs(p.x) + std::abs(p.y);
        reach = reach * 4.0 + 1.0;
        Vec2 d0 = pts[1] - pts[0];
        const double l0 = std::sqrt(d0.x * d0.x + d0.y * d0.y);
        pts[0] = pts[0] - d0 * (reach / l0);
        Vec2 d1 = pts[np - 1] - pts[np - 2];
        const double l1 = std::sqrt(d1.x * d1.x + d1.y * d1.y);
        pts[np - 1] = pts[np - 1] + d1 * (reach / l1);
    }
    // Unit left normals per segment (open case: on the extended segments).
    std::vector<Vec2> nrm(nseg);
    for (std::size_t k = 0; k < nseg; ++k) {
        const Vec2 d = pts[(k + 1) % np] - pts[k];
        const double l = std::sqrt(d.x * d.x + d.y * d.y);
        nrm[k] = Vec2{-d.y / l, d.x / l};
    }
    const Grid& g = c.field.grid();
    Component ra = c, rb = c;
    for (std::size_t j = 0; j < g.height; ++j) {
        const double y = g.origin.y + static_cast<double>(j) * g.cellSize;
        for (std::size_t i = 0; i < g.width; ++i) {
            const double x = g.origin.x + static_cast<double>(i) * g.cellSize;
            bool inA = false;
            if (cutter.closed) {
                inA = insidePolygon(pts, x, y);
            } else {
                double best = 0.0;
                std::size_t bk = 0;
                double bt = 0.0;
                for (std::size_t k = 0; k < nseg; ++k) {
                    const Vec2 p = pts[k], d = pts[k + 1] - p;
                    const double tt = std::clamp(((x - p.x) * d.x + (y - p.y) * d.y) / (d.x * d.x + d.y * d.y), 0.0, 1.0);
                    const double ex = x - (p.x + tt * d.x), ey = y - (p.y + tt * d.y);
                    const double d2 = ex * ex + ey * ey;
                    if (k == 0 || d2 < best) { best = d2; bk = k; bt = tt; }
                }
                double side;
                const Vec2 p = pts[bk], d = pts[bk + 1] - p;
                side = d.x * (y - p.y) - d.y * (x - p.x);
                const bool interiorVertex = (bt <= 0.0 && bk > 0) || (bt >= 1.0 && bk + 1 < nseg + 1 - 1);
                if (interiorVertex) {
                    const std::size_t vi = bt <= 0.0 ? bk : bk + 1;
                    const Vec2 nn = nrm[vi - 1] + nrm[vi];
                    if (nn.x * nn.x + nn.y * nn.y > 1e-18) side = nn.x * (x - pts[vi].x) + nn.y * (y - pts[vi].y);
                }
                inA = side >= 0.0;
            }
            const float v = c.field.at(i, j);
            if (opaqueV(v)) {
                (inA ? rb : ra).field.set(i, j, kTransparent);
            }
        }
    }
    ra.name = c.name + " \xE2\x80\x93" " A";
    rb.name = c.name + " \xE2\x80\x93" " B";
    a = std::move(ra);
    b = std::move(rb);
    return DeriveStatus::Ok;
}

// ---- Create Vector Boundary -----------------------------------------------------------------

DeriveStatus vectorBoundary(const std::vector<const Component*>& components, std::vector<Polyline>& out) {
    out.clear();
    bool any = false;
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0, cell = 0;
    for (const Component* c : components) {
        if (c == nullptr || c->field.empty()) continue;
        const Grid& g = c->field.grid();
        const double ex = g.origin.x + static_cast<double>(g.width - 1) * g.cellSize;
        const double ey = g.origin.y + static_cast<double>(g.height - 1) * g.cellSize;
        if (!any) { x0 = g.origin.x; y0 = g.origin.y; x1 = ex; y1 = ey; cell = g.cellSize; any = true; }
        else {
            x0 = std::min(x0, g.origin.x); y0 = std::min(y0, g.origin.y);
            x1 = std::max(x1, ex); y1 = std::max(y1, ey);
            cell = std::min(cell, g.cellSize);
        }
    }
    if (!any) return DeriveStatus::EmptyModel;
    const double fw = std::floor((x1 - x0) / cell + 0.5) + 1.0, fh = std::floor((y1 - y0) / cell + 0.5) + 1.0;
    if (fw * fh > static_cast<double>(kMaxCells)) return DeriveStatus::BadGrid;
    Grid grid;
    grid.cellSize = cell;
    grid.origin = Point{x0, y0};
    grid.width = static_cast<std::size_t>(fw);
    grid.height = static_cast<std::size_t>(fh);
    if (gridStatus(grid) != DeriveStatus::Ok) return DeriveStatus::BadGrid;
    HeightField mask;
    HeightField::create(grid, kTransparent, mask);
    bool opaqueAny = false;
    for (std::size_t j = 0; j < grid.height; ++j) {
        const double y = y0 + static_cast<double>(j) * cell;
        for (std::size_t i = 0; i < grid.width; ++i) {
            const double x = x0 + static_cast<double>(i) * cell;
            for (const Component* c : components) {
                if (c == nullptr || c->field.empty()) continue;
                if (opaqueV(c->field.sample(x, y))) { mask.set(i, j, 0.0f); opaqueAny = true; break; }
            }
        }
    }
    if (!opaqueAny) return DeriveStatus::EmptyModel;
    out = opaqueContours(mask);
    return DeriveStatus::Ok;
}

// ---- Cross section --------------------------------------------------------------------------

DeriveStatus crossSection(const HeightField& model, Point a, Point b, double spacing, double simplifyTol,
                          std::vector<Polyline>& out) {
    out.clear();
    if (!finite(a.x) || !finite(a.y) || !finite(b.x) || !finite(b.y) || !finite(spacing) || spacing <= 0.0 ||
        !finite(simplifyTol) || simplifyTol < 0.0) {
        return DeriveStatus::BadParameter;
    }
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    if (!(len > 0.0)) return DeriveStatus::BadParameter;
    if (model.empty()) return DeriveStatus::EmptyModel;
    const double steps = std::ceil(len / spacing);
    if (steps + 1.0 > static_cast<double>(kMaxProfileSamples)) return DeriveStatus::TooMany;
    const std::size_t n = static_cast<std::size_t>(steps);
    std::vector<Polyline> runs;
    Polyline cur;
    const auto flush = [&]() {
        if (!cur.points.empty()) runs.push_back(std::move(cur));
        cur = Polyline{};
    };
    for (std::size_t k = 0; k <= n; ++k) {
        const double t = k == n ? 1.0 : static_cast<double>(k) / steps;
        const double x = a.x + (b.x - a.x) * t, y = a.y + (b.y - a.y) * t;
        const float v = model.sample(x, y);
        if (!opaqueV(v)) { flush(); continue; }
        cur.points.push_back({len * t, static_cast<double>(v)});
    }
    flush();
    if (runs.empty()) return DeriveStatus::EmptyModel;
    if (simplifyTol > 0.0) {
        for (Polyline& r : runs) {
            // Douglas-Peucker, iterative, deterministic.
            const std::size_t m = r.points.size();
            if (m < 3) continue;
            std::vector<bool> keep(m, false);
            keep[0] = keep[m - 1] = true;
            std::vector<std::pair<std::size_t, std::size_t>> stack{{0, m - 1}};
            while (!stack.empty()) {
                const auto [lo, hi] = stack.back();
                stack.pop_back();
                const Vec2 p = r.points[lo], q = r.points[hi];
                const double dx = q.x - p.x, dy = q.y - p.y, dl = std::hypot(dx, dy);
                double worst = -1.0;
                std::size_t wi = lo;
                for (std::size_t i = lo + 1; i < hi; ++i) {
                    const double d = std::abs(dx * (r.points[i].y - p.y) - dy * (r.points[i].x - p.x)) / dl;
                    if (d > worst) { worst = d; wi = i; }
                }
                if (worst > simplifyTol) {
                    keep[wi] = true;
                    stack.push_back({lo, wi});
                    stack.push_back({wi, hi});
                }
            }
            std::vector<Vec2> kept;
            for (std::size_t i = 0; i < m; ++i) if (keep[i]) kept.push_back(r.points[i]);
            r.points = std::move(kept);
        }
    }
    out = std::move(runs);
    return DeriveStatus::Ok;
}

// ---- Distort --------------------------------------------------------------------------------

namespace {

double cross2(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

/// Inverse of the bilinear map (u,v) -> p00 + u*B + v*C + u*v*D for a convex quad; false outside it.
bool invertQuad(const Quad& q, double px, double py, double& u, double& v) {
    const Vec2 A{q.p00.x, q.p00.y};
    const Vec2 B = Vec2{q.p10.x, q.p10.y} - A;
    const Vec2 C = Vec2{q.p01.x, q.p01.y} - A;
    const Vec2 D = A - Vec2{q.p10.x, q.p10.y} + Vec2{q.p11.x, q.p11.y} - Vec2{q.p01.x, q.p01.y};
    const Vec2 Q = Vec2{px, py} - A;
    const double a2 = cross2(B, D), a1 = cross2(B, C) - cross2(Q, D), a0 = -cross2(Q, C);
    const double scale = std::max({std::abs(cross2(B, C)), std::abs(a1), 1e-300});
    double roots[2];
    int nr = 0;
    if (std::abs(a2) <= 1e-12 * scale) {
        if (a1 == 0.0) return false;
        roots[nr++] = -a0 / a1;
    } else {
        const double disc = a1 * a1 - 4.0 * a2 * a0;
        if (disc < 0.0) {
            if (disc < -1e-12 * a1 * a1 - 1e-300) return false;
            roots[nr++] = -a1 / (2.0 * a2);
        } else {
            const double sq = std::sqrt(disc);
            roots[nr++] = (-a1 - sq) / (2.0 * a2);
            roots[nr++] = (-a1 + sq) / (2.0 * a2);
        }
    }
    const double eps = 1e-9;
    bool found = false;
    for (int r = 0; r < nr; ++r) {
        const double uu = roots[r];
        const double ex = C.x + uu * D.x, ey = C.y + uu * D.y;
        const double rx = Q.x - uu * B.x, ry = Q.y - uu * B.y;
        double vv;
        if (std::abs(ex) >= std::abs(ey)) {
            if (ex == 0.0) continue;
            vv = rx / ex;
        } else {
            vv = ry / ey;
        }
        if (uu >= -eps && uu <= 1.0 + eps && vv >= -eps && vv <= 1.0 + eps) {
            u = std::clamp(uu, 0.0, 1.0);
            v = std::clamp(vv, 0.0, 1.0);
            found = true;
            break;
        }
    }
    return found;
}

/// Bilinear sample in cell units; coordinates within 1e-9 of a cell centre snap to it exactly.
float sampleCells(const HeightField& f, double fx, double fy) {
    const double rx = std::round(fx), ry = std::round(fy);
    if (std::abs(fx - rx) < 1e-9) fx = rx;
    if (std::abs(fy - ry) < 1e-9) fy = ry;
    const std::size_t w = f.width(), h = f.height();
    std::size_t i = static_cast<std::size_t>(fx), j = static_cast<std::size_t>(fy);
    if (i >= w - 1) i = w - 2;
    if (j >= h - 1) j = h - 2;
    const double tx = fx - static_cast<double>(i), ty = fy - static_cast<double>(j);
    const std::size_t i1 = tx > 0.0 ? i + 1 : i, j1 = ty > 0.0 ? j + 1 : j;
    const struct { double w; float v; } taps[4] = {{(1 - tx) * (1 - ty), f.at(i, j)},
                                                   {tx * (1 - ty), f.at(i1, j)},
                                                   {(1 - tx) * ty, f.at(i, j1)},
                                                   {tx * ty, f.at(i1, j1)}};
    double sum = 0.0;
    for (const auto& t : taps) {
        if (t.w == 0.0) continue;
        if (!opaqueV(t.v)) return kTransparent;
        sum += t.w * static_cast<double>(t.v);
    }
    return static_cast<float>(sum);
}

}  // namespace

DeriveStatus distortComponent(const Component& c, const Quad& q, const std::atomic<bool>* cancel, Component& out) {
    out = Component{};
    const Point corners[4] = {q.p00, q.p10, q.p11, q.p01};
    for (const Point& p : corners) {
        if (!finite(p.x) || !finite(p.y)) return DeriveStatus::BadParameter;
    }
    if (c.field.empty()) return DeriveStatus::EmptyModel;
    const Grid& g = c.field.grid();
    if (g.width < 2 || g.height < 2) return DeriveStatus::BadParameter;
    // Convex, non-degenerate: all four corner turns strictly the same sign.
    int pos = 0, neg = 0;
    double minx = corners[0].x, maxx = corners[0].x, miny = corners[0].y, maxy = corners[0].y;
    for (int k = 0; k < 4; ++k) {
        const Point& p = corners[k];
        const Point& n1 = corners[(k + 1) % 4];
        const Point& n2 = corners[(k + 2) % 4];
        const double cr = (n1.x - p.x) * (n2.y - n1.y) - (n1.y - p.y) * (n2.x - n1.x);
        if (cr > 0.0) ++pos; else if (cr < 0.0) ++neg;
        minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
    }
    if (!(pos == 4 || neg == 4)) return DeriveStatus::BadParameter;
    const double cs = g.cellSize;
    const double fi0 = std::ceil((minx - g.origin.x) / cs - 1e-9), fi1 = std::floor((maxx - g.origin.x) / cs + 1e-9);
    const double fj0 = std::ceil((miny - g.origin.y) / cs - 1e-9), fj1 = std::floor((maxy - g.origin.y) / cs + 1e-9);
    if (fi1 < fi0 || fj1 < fj0) return DeriveStatus::BadGrid;
    const double fw = fi1 - fi0 + 1.0, fh = fj1 - fj0 + 1.0;
    if (fw * fh > static_cast<double>(kMaxCells)) return DeriveStatus::BadGrid;
    Grid og;
    og.cellSize = cs;
    og.width = static_cast<std::size_t>(fw);
    og.height = static_cast<std::size_t>(fh);
    og.origin = Point{g.origin.x + fi0 * cs, g.origin.y + fj0 * cs};
    HeightField f;
    if (HeightField::create(og, kTransparent, f) != Status::Ok) return DeriveStatus::BadGrid;
    const double sw = static_cast<double>(g.width - 1), sh = static_cast<double>(g.height - 1);
    for (std::size_t j = 0; j < og.height; ++j) {
        if (cancelled(cancel)) return DeriveStatus::Cancelled;
        const double y = og.origin.y + static_cast<double>(j) * cs;
        for (std::size_t i = 0; i < og.width; ++i) {
            const double x = og.origin.x + static_cast<double>(i) * cs;
            double u = 0, v = 0;
            if (!invertQuad(q, x, y, u, v)) continue;
            f.set(i, j, sampleCells(c.field, u * sw, v * sh));
        }
    }
    out = c;
    out.field = std::move(f);
    return DeriveStatus::Ok;
}

}  // namespace carvetto::relief
