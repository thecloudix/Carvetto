#include "carvetto/relief/shape.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace carvetto::relief {

namespace {

constexpr std::size_t kMaxPolygons = 20000;
constexpr std::size_t kMaxSegments = std::size_t{1} << 21;
constexpr std::size_t kMaxCrossings = std::size_t{1} << 24;
constexpr std::size_t kMaxBorderWork = std::size_t{1} << 28;
constexpr int kMaxRounds = 16;
constexpr double kPi = 3.14159265358979323846;

struct Seg {
    double ax = 0, ay = 0, bx = 0, by = 0;
    int prev = 0, next = 0;       // neighbours in the same polygon (indices into the set)
    bool reflexA = false, reflexB = false;
};

struct Poly {
    std::vector<geometry::Vec2> pts;
    bool ccw = true;
    int depth = 0;
};

bool cancelled(const std::atomic<bool>* c) { return c != nullptr && c->load(std::memory_order_relaxed); }

bool finite2(const geometry::Vec2& p) { return std::isfinite(p.x) && std::isfinite(p.y); }

// Even-odd containment of a point in a polygon (ray towards +x).
bool contains(const Poly& poly, double px, double py) {
    bool in = false;
    const std::size_t n = poly.pts.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const auto& a = poly.pts[j];
        const auto& b = poly.pts[i];
        if ((a.y > py) != (b.y > py)) {
            const double x = a.x + (py - a.y) * (b.x - a.x) / (b.y - a.y);
            if (px < x) in = !in;
        }
    }
    return in;
}

// A set of segments with per-polygon neighbour links.
struct SegSet {
    std::vector<Seg> segs;
};

// `evenOnly`: only even-depth polygons, interior = the polygon itself (blend mode).
SegSet buildSet(const std::vector<Poly>& polys, bool evenOnly) {
    SegSet set;
    for (const Poly& p : polys) {
        if (evenOnly && (p.depth % 2) != 0) continue;
        const bool interiorLeft = evenOnly ? p.ccw : (p.ccw == (p.depth % 2 == 0));
        const int base = static_cast<int>(set.segs.size());
        const int n = static_cast<int>(p.pts.size());
        for (int i = 0; i < n; ++i) {
            Seg s;
            const auto& a = p.pts[static_cast<std::size_t>(i)];
            const auto& b = p.pts[static_cast<std::size_t>((i + 1) % n)];
            s.ax = a.x; s.ay = a.y; s.bx = b.x; s.by = b.y;
            s.prev = base + (i + n - 1) % n;
            s.next = base + (i + 1) % n;
            set.segs.push_back(s);
        }
        for (int i = 0; i < n; ++i) {
            Seg& s = set.segs[static_cast<std::size_t>(base + i)];
            const Seg& nx = set.segs[static_cast<std::size_t>(s.next)];
            const double cr = (s.bx - s.ax) * (nx.by - nx.ay) - (s.by - s.ay) * (nx.bx - nx.ax);
            const bool reflex = interiorLeft ? cr < 0.0 : cr > 0.0;
            s.reflexB = reflex;
            set.segs[static_cast<std::size_t>(s.next)].reflexA = reflex;
        }
    }
    return set;
}

double lineDist(const Seg& s, double px, double py) {
    const double dx = s.bx - s.ax, dy = s.by - s.ay;
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len == 0.0) return std::hypot(px - s.ax, py - s.ay);
    return std::fabs(dx * (py - s.ay) - dy * (px - s.ax)) / len;
}

// Distance of p to segment s (Euclidean), or the mitred value when the nearest feature is a
// reflex vertex and `preserve` is set.
double segValue(const SegSet& set, const Seg& s, double px, double py, bool preserve) {
    const double dx = s.bx - s.ax, dy = s.by - s.ay;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0.0 ? ((px - s.ax) * dx + (py - s.ay) * dy) / l2 : 0.0;
    const double tc = std::min(1.0, std::max(0.0, t));
    const double d = std::hypot(px - (s.ax + tc * dx), py - (s.ay + tc * dy));
    if (preserve) {
        if (t <= 0.0 && s.reflexA)
            return std::max(lineDist(s, px, py), lineDist(set.segs[static_cast<std::size_t>(s.prev)], px, py));
        if (t >= 1.0 && s.reflexB)
            return std::max(lineDist(s, px, py), lineDist(set.segs[static_cast<std::size_t>(s.next)], px, py));
    }
    return d;
}

// Marks cells whose centre is inside (even-odd) in `mask` (w*h bytes, row-major).
ShapeStatus fillMask(const SegSet& set, const Grid& g, std::vector<std::uint8_t>& mask,
                     const std::atomic<bool>* cancel) {
    const std::size_t w = g.width, h = g.height;
    const double cs = g.cellSize, ox = g.origin.x, oy = g.origin.y;
    auto rowRange = [&](const Seg& s, std::size_t& j0, std::size_t& j1) {  // [j0, j1)
        const double lo = std::min(s.ay, s.by), hi = std::max(s.ay, s.by);
        double a = std::ceil((lo - oy) / cs), b = std::ceil((hi - oy) / cs);
        a = std::max(a, 0.0);
        b = std::min(b, static_cast<double>(h));
        if (!(b > a)) { j0 = j1 = 0; return; }
        j0 = static_cast<std::size_t>(a);
        j1 = static_cast<std::size_t>(b);
    };
    std::vector<std::size_t> start(h + 1, 0);
    std::size_t total = 0;
    for (const Seg& s : set.segs) {
        if (s.ay == s.by) continue;
        std::size_t j0, j1;
        rowRange(s, j0, j1);
        total += j1 - j0;
        if (total > kMaxCrossings) return ShapeStatus::TooMuchInput;
        for (std::size_t j = j0; j < j1; ++j) ++start[j + 1];
    }
    for (std::size_t j = 0; j < h; ++j) start[j + 1] += start[j];
    std::vector<double> xs(total);
    std::vector<std::size_t> fillPos(start.begin(), start.end() - 1);
    for (const Seg& s : set.segs) {
        if (s.ay == s.by) continue;
        std::size_t j0, j1;
        rowRange(s, j0, j1);
        for (std::size_t j = j0; j < j1; ++j) {
            const double yc = oy + static_cast<double>(j) * cs;
            xs[fillPos[j]++] = s.ax + (yc - s.ay) * (s.bx - s.ax) / (s.by - s.ay);
        }
    }
    for (std::size_t j = 0; j < h; ++j) {
        if (cancelled(cancel)) return ShapeStatus::Cancelled;
        auto first = xs.begin() + static_cast<std::ptrdiff_t>(start[j]);
        auto last = xs.begin() + static_cast<std::ptrdiff_t>(start[j + 1]);
        std::sort(first, last);
        for (auto it = first; it != last && it + 1 != last; it += 2) {  // spans [x_2k, x_2k+1)
            double a = std::ceil((*it - ox) / cs), b = std::ceil((*(it + 1) - ox) / cs);
            a = std::max(a, 0.0);
            b = std::min(b, static_cast<double>(w));
            for (double i = a; i < b; i += 1.0) mask[j * w + static_cast<std::size_t>(i)] = 1;
        }
    }
    return ShapeStatus::Ok;
}

// Nearest-edge field: exact distance to the best known edge per cell. Seeds: every cell within two
// cells of a sampled point of an edge gets the exact distance to that edge; the edge ids then
// propagate between 8-neighbours in alternating raster sweeps until nothing improves.
struct EdgeField {
    const SegSet& set;
    const Grid& g;
    bool preserve;
    std::vector<std::int32_t> id;
    std::vector<float> dist;

    EdgeField(const SegSet& s, const Grid& grid, bool pres)
        : set(s), g(grid), preserve(pres), id(grid.width * grid.height, -1),
          dist(grid.width * grid.height, std::numeric_limits<float>::infinity()) {}

    double cellX(std::size_t i) const { return g.origin.x + static_cast<double>(i) * g.cellSize; }
    double cellY(std::size_t j) const { return g.origin.y + static_cast<double>(j) * g.cellSize; }

    bool offer(std::size_t i, std::size_t j, std::int32_t sid) {
        const std::size_t k = j * g.width + i;
        const float d = static_cast<float>(
            segValue(set, set.segs[static_cast<std::size_t>(sid)], cellX(i), cellY(j), preserve));
        if (d < dist[k]) { dist[k] = d; id[k] = sid; return true; }
        return false;
    }

    void seed(std::int32_t sid) {
        const Seg& s = set.segs[static_cast<std::size_t>(sid)];
        const double cs = g.cellSize;
        const double ax = (s.ax - g.origin.x) / cs, ay = (s.ay - g.origin.y) / cs;
        const double dx = (s.bx - s.ax) / cs, dy = (s.by - s.ay) / cs;
        const double xmin = -3.0, xmax = static_cast<double>(g.width) + 2.0;
        const double ymin = -3.0, ymax = static_cast<double>(g.height) + 2.0;
        double t0 = 0.0, t1 = 1.0;
        const double p[4] = {-dx, dx, -dy, dy};
        const double q[4] = {ax - xmin, xmax - ax, ay - ymin, ymax - ay};
        for (int k = 0; k < 4; ++k) {
            if (p[k] == 0.0) {
                if (q[k] < 0.0) return;
            } else {
                const double r = q[k] / p[k];
                if (p[k] < 0.0) t0 = std::max(t0, r); else t1 = std::min(t1, r);
            }
        }
        if (t0 > t1) return;
        const double len = std::hypot(dx, dy) * (t1 - t0);
        const std::size_t n = static_cast<std::size_t>(std::ceil(len / 0.5)) + 1;
        for (std::size_t k = 0; k <= n; ++k) {
            const double t = t0 + (t1 - t0) * static_cast<double>(k) / static_cast<double>(n);
            const long ci = std::lround(ax + t * dx), cj = std::lround(ay + t * dy);
            for (long j = cj - 2; j <= cj + 2; ++j) {
                if (j < 0 || j >= static_cast<long>(g.height)) continue;
                for (long i = ci - 2; i <= ci + 2; ++i) {
                    if (i < 0 || i >= static_cast<long>(g.width)) continue;
                    offer(static_cast<std::size_t>(i), static_cast<std::size_t>(j), sid);
                }
            }
        }
    }

    bool relax(std::size_t i, std::size_t j, int di, int dj) {
        const long ni = static_cast<long>(i) + di, nj = static_cast<long>(j) + dj;
        if (ni < 0 || nj < 0 || ni >= static_cast<long>(g.width) || nj >= static_cast<long>(g.height)) return false;
        const std::int32_t sid = id[static_cast<std::size_t>(nj) * g.width + static_cast<std::size_t>(ni)];
        if (sid < 0 || sid == id[j * g.width + i]) return false;
        return offer(i, j, sid);
    }

    ShapeStatus run(const std::atomic<bool>* cancel) {
        // Edges not fully inside the grid: seed the grid border exactly (brute force, bounded).
        bool allInside = true;
        for (const Seg& s : set.segs) {
            for (int e = 0; e < 2 && allInside; ++e) {
                const double x = e ? s.bx : s.ax, y = e ? s.by : s.ay;
                if (x < g.origin.x || y < g.origin.y || x > cellX(g.width - 1) || y > cellY(g.height - 1))
                    allInside = false;
            }
            if (!allInside) break;
        }
        if (!allInside) {
            const std::size_t perim = 2 * (g.width + g.height);
            if (perim * set.segs.size() > kMaxBorderWork) return ShapeStatus::TooMuchInput;
            for (std::size_t j = 0; j < g.height; ++j) {
                if (cancelled(cancel)) return ShapeStatus::Cancelled;
                const bool edgeRow = j == 0 || j + 1 == g.height;
                for (std::size_t i = 0; i < g.width; i += (edgeRow ? 1 : (g.width > 1 ? g.width - 1 : 1)))
                    for (std::size_t k = 0; k < set.segs.size(); ++k) offer(i, j, static_cast<std::int32_t>(k));
            }
        }
        for (std::size_t k = 0; k < set.segs.size(); ++k) {
            if ((k & 63u) == 0 && cancelled(cancel)) return ShapeStatus::Cancelled;
            seed(static_cast<std::int32_t>(k));
        }
        static const int fwd[4][2] = {{-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
        static const int bwd[4][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}};
        for (int round = 0; round < kMaxRounds; ++round) {
            bool changed = false;
            for (std::size_t j = 0; j < g.height; ++j) {
                if (cancelled(cancel)) return ShapeStatus::Cancelled;
                for (std::size_t i = 0; i < g.width; ++i)
                    for (const auto& o : fwd) changed = relax(i, j, o[0], o[1]) || changed;
            }
            for (std::size_t j = g.height; j-- > 0;) {
                if (cancelled(cancel)) return ShapeStatus::Cancelled;
                for (std::size_t i = g.width; i-- > 0;)
                    for (const auto& o : bwd) changed = relax(i, j, o[0], o[1]) || changed;
            }
            if (!changed) break;
        }
        // Cells no edge reached (only when nothing was seeded near them): brute force, bounded.
        std::size_t missing = 0;
        for (std::int32_t v : id) if (v < 0) ++missing;
        if (missing * set.segs.size() > kMaxBorderWork) return ShapeStatus::TooMuchInput;
        if (missing > 0)
            for (std::size_t j = 0; j < g.height; ++j) {
                if (cancelled(cancel)) return ShapeStatus::Cancelled;
                for (std::size_t i = 0; i < g.width; ++i)
                    if (id[j * g.width + i] < 0)
                        for (std::size_t k = 0; k < set.segs.size(); ++k) offer(i, j, static_cast<std::int32_t>(k));
            }
        return ShapeStatus::Ok;
    }
};

double profileHeight(const ShapeParams& p, double d, double dMax) {
    double h = 0.0;
    switch (p.kind) {
        case ShapeKind::Plane: h = 0.0; break;
        case ShapeKind::Angled: h = d * std::tan(p.angleRad); break;
        case ShapeKind::Round: {
            const double s = std::sin(p.angleRad);
            const double r = dMax / s;
            const double dd = std::min(d, dMax);
            const double q = r * r - (dMax - dd) * (dMax - dd);
            h = std::sqrt(std::max(q, 0.0)) - r * std::cos(p.angleRad);
            break;
        }
        case ShapeKind::Smooth: {
            const double u = std::min(d / dMax, 1.0);
            h = std::pow(u, std::pow(4.0, -p.smoothing)) * dMax;
            break;
        }
        case ShapeKind::Custom: {
            const double u = std::min(d / dMax, 1.0);
            const auto& pr = p.profile;
            double v;
            if (u <= pr.front().u) {
                v = pr.front().h;
            } else if (u >= pr.back().u) {
                v = pr.back().h;
            } else {
                std::size_t k = 1;
                while (pr[k].u < u) ++k;
                const double t = (u - pr[k - 1].u) / (pr[k].u - pr[k - 1].u);
                v = pr[k - 1].h + t * (pr[k].h - pr[k - 1].h);
            }
            h = v * dMax;
            break;
        }
    }
    if (p.limitEnabled) h = std::min(h, p.limit);
    return h;
}

ShapeStatus validateParams(const ShapeParams& p) {
    if (!std::isfinite(p.scalePercent) || p.scalePercent <= 0.0 || !std::isfinite(p.baseHeight))
        return ShapeStatus::BadParameter;
    if (p.limitEnabled && !(std::isfinite(p.limit) && p.limit > 0.0)) return ShapeStatus::BadParameter;
    switch (p.kind) {
        case ShapeKind::Plane: break;
        case ShapeKind::Angled:
            if (!(std::isfinite(p.angleRad) && p.angleRad > 0.0 && p.angleRad < kPi / 2.0))
                return ShapeStatus::BadParameter;
            break;
        case ShapeKind::Round:
            if (!(std::isfinite(p.angleRad) && p.angleRad > 0.0 && p.angleRad <= kPi / 2.0))
                return ShapeStatus::BadParameter;
            break;
        case ShapeKind::Smooth:
            if (!(std::isfinite(p.smoothing) && p.smoothing >= -1.0 && p.smoothing <= 1.0))
                return ShapeStatus::BadParameter;
            break;
        case ShapeKind::Custom: {
            if (p.profile.size() < 2) return ShapeStatus::BadParameter;
            for (std::size_t k = 0; k < p.profile.size(); ++k) {
                const auto& q = p.profile[k];
                if (!std::isfinite(q.u) || !std::isfinite(q.h) || q.u < 0.0 || q.u > 1.0)
                    return ShapeStatus::BadParameter;
                if (k > 0 && !(q.u > p.profile[k - 1].u)) return ShapeStatus::BadParameter;
            }
            break;
        }
    }
    return ShapeStatus::Ok;
}

}  // namespace

const char* shapeStatusMessage(ShapeStatus s) {
    switch (s) {
        case ShapeStatus::Ok: return "ok";
        case ShapeStatus::BadGrid: return "invalid or oversized grid";
        case ShapeStatus::NoVectors: return "no vectors, or no cell lies inside them";
        case ShapeStatus::BadVector: return "vector is open, degenerate or not finite";
        case ShapeStatus::BadParameter: return "shape parameter out of range";
        case ShapeStatus::TooMuchInput: return "too many polygons, segments or crossings";
        case ShapeStatus::Cancelled: return "cancelled";
    }
    return "unknown";
}

ShapeStatus buildShape(const Shape& shape, const std::atomic<bool>* cancel, Component& out) {
    const ShapeParams& prm = shape.params;
    if (ShapeStatus st = validateParams(prm); st != ShapeStatus::Ok) return st;

    // Vectors -> cleaned polygons.
    if (shape.vectors.size() > kMaxPolygons) return ShapeStatus::TooMuchInput;
    std::vector<Poly> polys;
    std::size_t segCount = 0;
    for (const auto& pl : shape.vectors) {
        if (!pl.closed) return ShapeStatus::BadVector;
        Poly p;
        for (const auto& v : pl.points) {
            if (!finite2(v)) return ShapeStatus::BadVector;
            if (!p.pts.empty() && p.pts.back().x == v.x && p.pts.back().y == v.y) continue;
            p.pts.push_back(v);
        }
        while (p.pts.size() > 1 && p.pts.front().x == p.pts.back().x && p.pts.front().y == p.pts.back().y)
            p.pts.pop_back();
        if (p.pts.size() < 3) return ShapeStatus::BadVector;
        double area = 0.0;
        for (std::size_t i = 0, j = p.pts.size() - 1; i < p.pts.size(); j = i++)
            area += p.pts[j].x * p.pts[i].y - p.pts[i].x * p.pts[j].y;
        p.ccw = area >= 0.0;
        segCount += p.pts.size();
        if (segCount > kMaxSegments) return ShapeStatus::TooMuchInput;
        polys.push_back(std::move(p));
    }

    // Grid.
    Grid grid;
    if (shape.grid) {
        grid = *shape.grid;
    } else {
        if (polys.empty()) return ShapeStatus::NoVectors;
        double x0 = std::numeric_limits<double>::infinity(), y0 = x0;
        double x1 = -x0, y1 = -x0;
        for (const Poly& p : polys)
            for (const auto& v : p.pts) {
                x0 = std::min(x0, v.x); x1 = std::max(x1, v.x);
                y0 = std::min(y0, v.y); y1 = std::max(y1, v.y);
            }
        const double cs = shape.cellSize;
        if (!(std::isfinite(cs) && cs > 0.0)) return ShapeStatus::BadGrid;
        const double nw = std::floor((x1 - x0) / cs) + 4.0, nh = std::floor((y1 - y0) / cs) + 4.0;
        if (!(nw * nh <= static_cast<double>(kMaxCells))) return ShapeStatus::BadGrid;
        grid = Grid{static_cast<std::size_t>(nw), static_cast<std::size_t>(nh), cs, {x0 - 1.5 * cs, y0 - 1.5 * cs}};
    }
    if (validate(grid) != Status::Ok) return ShapeStatus::BadGrid;
    if (polys.empty() && prm.kind != ShapeKind::Plane) return ShapeStatus::NoVectors;

    HeightField field;
    if (HeightField::create(grid, kTransparent, field) != Status::Ok) return ShapeStatus::BadGrid;
    const std::size_t w = grid.width, h = grid.height;

    if (polys.empty()) {  // Plane without vectors: the whole grid at height 0.
        for (std::size_t j = 0; j < h; ++j)
            for (std::size_t i = 0; i < w; ++i) field.set(i, j, 0.0f);
    } else {
        // Nesting depth by containment of each polygon's first vertex.
        for (std::size_t i = 0; i < polys.size(); ++i) {
            if (cancelled(cancel)) return ShapeStatus::Cancelled;
            int d = 0;
            for (std::size_t j = 0; j < polys.size(); ++j)
                if (i != j && contains(polys[j], polys[i].pts[0].x, polys[i].pts[0].y)) ++d;
            polys[i].depth = d;
        }
        const bool blend = prm.blendToInnerVectors && (prm.kind == ShapeKind::Smooth || prm.kind == ShapeKind::Custom);

        SegSet full = buildSet(polys, false);
        std::vector<std::uint8_t> mainMask(w * h, 0);
        if (ShapeStatus st = fillMask(full, grid, mainMask, cancel); st != ShapeStatus::Ok) return st;
        if (prm.kind == ShapeKind::Plane) {
            for (std::size_t k = 0; k < w * h; ++k)
                if (mainMask[k]) field.set(k % w, k / w, 0.0f);
        } else {
            SegSet outerOnly;
            if (blend) outerOnly = buildSet(polys, true);
            const SegSet& edges = blend ? outerOnly : full;
            EdgeField ef(edges, grid, prm.preserveInternalCorners);
            if (ShapeStatus st = ef.run(cancel); st != ShapeStatus::Ok) return st;
            double dMax = 0.0;
            for (std::size_t k = 0; k < w * h; ++k)
                if (mainMask[k]) dMax = std::max(dMax, static_cast<double>(ef.dist[k]));
            if (!(dMax > 0.0)) return ShapeStatus::NoVectors;
            for (std::size_t j = 0; j < h; ++j) {
                if (cancelled(cancel)) return ShapeStatus::Cancelled;
                for (std::size_t i = 0; i < w; ++i)
                    if (mainMask[j * w + i])
                        field.set(i, j, static_cast<float>(profileHeight(
                                            prm, static_cast<double>(ef.dist[j * w + i]), dMax)));
            }
        }
    }
    float lo, hi;
    if (!field.minMax(lo, hi)) return ShapeStatus::NoVectors;

    Component c;
    c.name = prm.name;
    c.field = std::move(field);
    c.mode = prm.mode;
    c.shapeHeightPercent = prm.scalePercent;
    c.baseHeight = prm.baseHeight;
    c.fade = prm.fade;
    c.tilt = prm.tilt;
    out = std::move(c);
    return ShapeStatus::Ok;
}

double totalHeight(double rawMax, double scalePercent, double baseHeight) {
    return baseHeight + rawMax * scalePercent / 100.0;
}

ShapeStatus setTotalHeight(double rawMax, double total, TotalHeightAdjust adjust, double& scalePercent,
                           double& baseHeight) {
    if (!std::isfinite(rawMax) || !std::isfinite(total) || !std::isfinite(scalePercent) || !std::isfinite(baseHeight))
        return ShapeStatus::BadParameter;
    if (adjust == TotalHeightAdjust::Scale) {
        if (!(rawMax > 0.0) || !(total > baseHeight)) return ShapeStatus::BadParameter;
        scalePercent = (total - baseHeight) / rawMax * 100.0;
    } else {
        baseHeight = total - rawMax * scalePercent / 100.0;
    }
    return ShapeStatus::Ok;
}

double totalHeight(const Component& c) {
    float lo, hi;
    if (!c.field.minMax(lo, hi)) return c.baseHeight;
    return totalHeight(static_cast<double>(hi), c.shapeHeightPercent, c.baseHeight);
}

ShapeStatus setTotalHeight(Component& c, double total, TotalHeightAdjust adjust) {
    float lo, hi;
    if (!c.field.minMax(lo, hi)) return ShapeStatus::BadParameter;
    double s = c.shapeHeightPercent, b = c.baseHeight;
    if (ShapeStatus st = setTotalHeight(static_cast<double>(hi), total, adjust, s, b); st != ShapeStatus::Ok) return st;
    c.shapeHeightPercent = s;
    c.baseHeight = b;
    return ShapeStatus::Ok;
}

}  // namespace carvetto::relief
