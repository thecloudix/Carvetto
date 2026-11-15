#include "carvetto/toolpath/rough3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/simulate/stock.hpp"

namespace carvetto::toolpath {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::size_t kBlock = 16;
constexpr std::size_t kMaxReachCells = 2000;

bool cancelled(const std::atomic<bool>* c) {
    return c != nullptr && c->load(std::memory_order_relaxed);
}

/// Rounds a double up to the next float (never below the double).
float roundUp(double v) {
    if (std::isnan(v)) {
        return -std::numeric_limits<float>::infinity();
    }
    if (v >= static_cast<double>(std::numeric_limits<float>::max())) {
        return std::numeric_limits<float>::max();
    }
    if (v <= -static_cast<double>(std::numeric_limits<float>::max())) {
        return -std::numeric_limits<float>::infinity();
    }
    float f = static_cast<float>(v);
    if (static_cast<double>(f) < v) {
        f = std::nextafter(f, std::numeric_limits<float>::infinity());
    }
    return f;
}

void addDiag(Rough3dResult& r, ToolpathIssue issue, std::string msg, std::optional<std::size_t> tool = {}) {
    ToolpathDiagnostic d;
    d.issue = issue;
    d.message = std::move(msg);
    d.tool = tool;
    r.diagnostics.push_back(std::move(d));
}

}  // namespace

DropStatus dropToolSurface(const relief::HeightField& model, const simulate::ToolShape& tool, double zOffset,
                           std::vector<float>& out, std::uint64_t workLimit, std::uint64_t& work,
                           const std::atomic<bool>* cancel) {
    const std::size_t w = model.width();
    const std::size_t h = model.height();
    const double cs = model.cellSize();
    out.clear();
    if (w == 0 || h == 0 || !tool.valid()) {
        return DropStatus::TooLarge;
    }
    const double reach = tool.radius() * (1.0 + 1e-9) + 1e-9;
    const double rc = std::ceil(reach / cs);
    if (!(rc <= static_cast<double>(kMaxReachCells))) {
        return DropStatus::TooLarge;
    }
    const long R = static_cast<long>(rc);
    const long side = 2 * R + 1;
    // Profile height at every cell offset (infinity beyond the tool's reach).
    std::vector<double> prof(static_cast<std::size_t>(side * side), kInf);
    for (long dy = -R; dy <= R; ++dy) {
        for (long dx = -R; dx <= R; ++dx) {
            const double d = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * cs;
            if (d <= reach) {
                prof[static_cast<std::size_t>((dy + R) * side + (dx + R))] = tool.heightAt(std::min(d, tool.radius()));
            }
        }
    }
    // Block maxima of the (offset) model; -inf for blocks without opaque cells.
    const std::size_t bw = (w + kBlock - 1) / kBlock;
    const std::size_t bh = (h + kBlock - 1) / kBlock;
    std::vector<double> bmax(bw * bh, -kInf);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            const float v = model.at(i, j);
            if (!std::isnan(v)) {
                double& m = bmax[(j / kBlock) * bw + (i / kBlock)];
                m = std::max(m, static_cast<double>(v) + zOffset);
            }
        }
    }
    try {
        out.assign(w * h, -std::numeric_limits<float>::infinity());
    } catch (const std::bad_alloc&) {
        out.clear();
        return DropStatus::TooLarge;
    }
    for (std::size_t j = 0; j < h; ++j) {
        if (cancelled(cancel)) {
            out.clear();
            return DropStatus::Cancelled;
        }
        if (work > workLimit) {
            out.clear();
            return DropStatus::WorkLimit;
        }
        for (std::size_t i = 0; i < w; ++i) {
            const long ci = static_cast<long>(i);
            const long cj = static_cast<long>(j);
            const long i0 = std::max(0L, ci - R);
            const long i1 = std::min(static_cast<long>(w) - 1, ci + R);
            const long j0 = std::max(0L, cj - R);
            const long j1 = std::min(static_cast<long>(h) - 1, cj + R);
            double best = -kInf;
            for (long by = j0 / static_cast<long>(kBlock); by <= j1 / static_cast<long>(kBlock); ++by) {
                for (long bx = i0 / static_cast<long>(kBlock); bx <= i1 / static_cast<long>(kBlock); ++bx) {
                    ++work;
                    const double bm = bmax[static_cast<std::size_t>(by) * bw + static_cast<std::size_t>(bx)];
                    if (bm == -kInf) {
                        continue;
                    }
                    const long ki0 = std::max(i0, bx * static_cast<long>(kBlock));
                    const long ki1 = std::min(i1, (bx + 1) * static_cast<long>(kBlock) - 1);
                    const long kj0 = std::max(j0, by * static_cast<long>(kBlock));
                    const long kj1 = std::min(j1, (by + 1) * static_cast<long>(kBlock) - 1);
                    // Nearest offset of this part of the block: a lower bound on the profile height.
                    const long ndx = ci < ki0 ? ki0 - ci : (ci > ki1 ? ci - ki1 : 0);
                    const long ndy = cj < kj0 ? kj0 - cj : (cj > kj1 ? cj - kj1 : 0);
                    const double lo = prof[static_cast<std::size_t>((ndy + R) * side + (ndx + R))];
                    if (lo == kInf || bm - lo <= best) {
                        continue;
                    }
                    for (long qj = kj0; qj <= kj1; ++qj) {
                        for (long qi = ki0; qi <= ki1; ++qi) {
                            ++work;
                            const float v = model.at(static_cast<std::size_t>(qi), static_cast<std::size_t>(qj));
                            if (std::isnan(v)) {
                                continue;
                            }
                            const double ph = prof[static_cast<std::size_t>((qj - cj + R) * side + (qi - ci + R))];
                            if (ph == kInf) {
                                continue;
                            }
                            best = std::max(best, static_cast<double>(v) + zOffset - ph);
                        }
                    }
                }
            }
            out[j * w + i] = best == -kInf ? -std::numeric_limits<float>::infinity() : roundUp(best);
        }
    }
    return DropStatus::Ok;
}

namespace {

using geometry::Vec2;

/// Grid indexing in the raster frame: rows run along `axis`, columns across them.
struct Frame {
    std::size_t w{0};
    std::size_t h{0};
    bool alongX{true};
    std::size_t rows() const { return alongX ? h : w; }
    std::size_t cols() const { return alongX ? w : h; }
    std::size_t node(std::size_t row, std::size_t col) const { return alongX ? row * w + col : col * w + row; }
};

struct Grid3 {
    const relief::HeightField* model{nullptr};
    double cs{1.0};
    Vec2 origin;
    Vec2 xy(std::size_t node) const {
        const std::size_t w = model->width();
        return {origin.x + static_cast<double>(node % w) * cs, origin.y + static_cast<double>(node / w) * cs};
    }
};

struct Ctx {
    const Rough3dParams& p;
    const ToolpathRunControl& control;
    Rough3dResult& result;
    std::uint64_t workLimit{0};
    std::uint64_t moveLimit{0};
    std::uint64_t work{0};
    bool abort{false};
    bool isCancelled() const { return cancelled(control.cancel); }
};

double pointSegDist(Vec2 p, Vec2 a, Vec2 b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double ex = a.x + t * dx - p.x;
    const double ey = a.y + t * dy - p.y;
    return std::sqrt(ex * ex + ey * ey);
}

bool insideEvenOdd(const std::vector<geometry::Polyline>& polys, Vec2 p) {
    bool in = false;
    for (const geometry::Polyline& pl : polys) {
        const std::size_t n = pl.points.size();
        for (std::size_t a = 0, b = n - 1; a < n; b = a++) {
            const Vec2 pa = pl.points[a];
            const Vec2 pb = pl.points[b];
            if ((pa.y > p.y) != (pb.y > p.y) && p.x < (pb.x - pa.x) * (p.y - pa.y) / (pb.y - pa.y) + pa.x) {
                in = !in;
            }
        }
    }
    return in;
}

/// Machining region mask. Returns false when refused (work limit / cancel).
bool buildMask(Ctx& c, const Grid3& g, std::vector<std::uint8_t>& mask) {
    const relief::HeightField& m = *g.model;
    const std::size_t w = m.width();
    const std::size_t h = m.height();
    mask.assign(w * h, 0);
    const Rough3dParams& p = c.p;
    if (p.boundary == Rough3dBoundary::Material) {
        std::fill(mask.begin(), mask.end(), std::uint8_t{1});
        return true;
    }
    if (p.boundary == Rough3dBoundary::Model) {
        const double offc = std::ceil(std::fabs(p.boundaryOffset) / g.cs);
        if (!(offc <= 200.0)) {
            return false;
        }
        const long O = static_cast<long>(offc);
        const double off = std::fabs(p.boundaryOffset);
        const bool grow = p.boundaryOffset > 0.0;
        for (std::size_t j = 0; j < h; ++j) {
            if (c.isCancelled()) {
                c.abort = true;
                return false;
            }
            for (std::size_t i = 0; i < w; ++i) {
                const bool own = !std::isnan(m.at(i, j));
                if (p.boundaryOffset == 0.0) {
                    mask[j * w + i] = own ? 1 : 0;
                    continue;
                }
                // Grow: any opaque cell within `off`. Shrink: opaque and no transparent/outside cell within `off`.
                bool found = !grow;
                for (long dj = -O; dj <= O && (grow ? !found : found); ++dj) {
                    for (long di = -O; di <= O; ++di) {
                        ++c.work;
                        const double d = std::sqrt(static_cast<double>(di * di + dj * dj)) * g.cs;
                        if (d > off) {
                            continue;
                        }
                        const long qi = static_cast<long>(i) + di;
                        const long qj = static_cast<long>(j) + dj;
                        const bool opaque = qi >= 0 && qj >= 0 && qi < static_cast<long>(w) &&
                                            qj < static_cast<long>(h) &&
                                            !std::isnan(m.at(static_cast<std::size_t>(qi), static_cast<std::size_t>(qj)));
                        if (grow && opaque) {
                            found = true;
                            break;
                        }
                        if (!grow && !opaque) {
                            found = false;
                            break;
                        }
                    }
                }
                mask[j * w + i] = (grow ? found : (own && found)) ? 1 : 0;
            }
            if (c.work > c.workLimit) {
                return false;
            }
        }
        return true;
    }
    // Vectors.
    std::vector<geometry::Polyline> polys;
    std::size_t edges = 0;
    for (const geometry::Polyline& pl : p.vectors) {
        if (pl.points.size() >= 3) {
            polys.push_back(pl);
            edges += pl.points.size();
        }
    }
    for (std::size_t j = 0; j < h; ++j) {
        if (c.isCancelled()) {
            c.abort = true;
            return false;
        }
        for (std::size_t i = 0; i < w; ++i) {
            const Vec2 pt = g.xy(j * w + i);
            bool in = insideEvenOdd(polys, pt);
            c.work += 1 + edges;
            if (p.boundaryOffset != 0.0) {
                double dmin = kInf;
                for (const geometry::Polyline& pl : polys) {
                    const std::size_t n = pl.points.size();
                    for (std::size_t a = 0, b = n - 1; a < n; b = a++) {
                        dmin = std::min(dmin, pointSegDist(pt, pl.points[a], pl.points[b]));
                    }
                }
                in = p.boundaryOffset > 0.0 ? (in || dmin <= p.boundaryOffset) : (in && dmin > -p.boundaryOffset);
            }
            mask[j * w + i] = in ? 1 : 0;
        }
        if (c.work > c.workLimit) {
            return false;
        }
    }
    return true;
}

/// Emits one chain (a walk over 4-neighbour grid nodes) of the toolpath. `zn` is the tip height at
/// each node. Segments between adjacent nodes are cut at a constant height (the higher of the two
/// node heights, then merged within the tolerance): the exact-at-grid argument of ADR-0071.
struct Emitter {
    const Grid3& g;
    const Rough3dParams& p;
    Toolpath& tp;
    std::uint64_t moveLimit;
    std::optional<geometry::Vec3> pos;
    bool overLimit{false};

    void add(Move m) {
        if (tp.moves.size() >= moveLimit) {
            overLimit = true;
            return;
        }
        tp.moves.push_back(m);
        pos = m.to;
    }
    /// Feed to (x, y, z); a feed collinear with the previous feed at the same Z extends it.
    void feedTo(Vec2 xy, double z) {
        if (!pos) {
            return;
        }
        if (pos->x == xy.x && pos->y == xy.y && pos->z == z) {
            return;
        }
        if (!tp.moves.empty() && tp.moves.back().kind == MoveKind::Feed && tp.moves.size() >= 2) {
            const geometry::Vec3 a = tp.moves[tp.moves.size() - 2].to;
            const geometry::Vec3 b = tp.moves.back().to;
            if (tp.moves[tp.moves.size() - 2].kind != MoveKind::Rapid && a.z == z && b.z == z && pos->z == z &&
                ((a.x == b.x && b.x == xy.x) || (a.y == b.y && b.y == xy.y)) &&
                ((b.x - a.x) * (xy.x - b.x) + (b.y - a.y) * (xy.y - b.y)) > 0.0) {
                tp.moves.back().to = geometry::Vec3{xy, z};
                pos = tp.moves.back().to;
                return;
            }
        }
        add(Move::feed(geometry::Vec3{xy, z}));
    }
    void toSafe() {
        if (pos && pos->z < p.safeZ) {
            add(Move::retract(geometry::Vec3{pos->x, pos->y, p.safeZ}));
        }
    }
    void emitChain(const std::vector<std::uint32_t>& nodes, const std::vector<double>& zn,
                   const std::vector<float>& scur) {
        const std::size_t n = nodes.size();
        if (n == 0 || overLimit) {
            return;
        }
        const Vec2 first = g.xy(nodes[0]);
        toSafe();
        add(Move::rapid(geometry::Vec3{first, p.safeZ}));
        if (n == 1) {
            add(Move::plunge(geometry::Vec3{first, zn[0]}));
            toSafe();
            return;
        }
        // Segment heights, merged within the tolerance (raised to the group maximum).
        std::vector<double> seg(n - 1);
        for (std::size_t k = 0; k + 1 < n; ++k) {
            seg[k] = std::max(zn[k], zn[k + 1]);
        }
        if (p.tolerance > 0.0) {
            std::size_t s = 0;
            while (s < seg.size()) {
                double lo = seg[s];
                double hi = seg[s];
                std::size_t e = s + 1;
                while (e < seg.size() && std::max(hi, seg[e]) - std::min(lo, seg[e]) <= p.tolerance) {
                    hi = std::max(hi, seg[e]);
                    lo = std::min(lo, seg[e]);
                    ++e;
                }
                for (std::size_t k = s; k < e; ++k) {
                    seg[k] = hi;
                }
                s = e;
            }
        }
        auto adj = [&](std::size_t k) {
            double v = -kInf;
            if (k > 0) {
                v = std::max(v, seg[k - 1]);
            }
            if (k + 1 < n) {
                v = std::max(v, seg[k]);
            }
            return v;
        };
        // At a node the tip may go down to the node's own height (a vertical move there is exact),
        // so the surface at the node reaches it; skipped within the tolerance of the segment height.
        auto dip = [&](std::size_t k) {
            if (pos && zn[k] + p.tolerance < pos->z) {
                feedTo(g.xy(nodes[k]), zn[k]);
            }
        };
        // Walks nodes a..b (either direction) as a staircase of constant-height segments.
        auto stair = [&](std::size_t a, std::size_t b) {
            if (a == b) {
                return;
            }
            const bool up = b > a;
            std::size_t k = a;
            while (k != b) {
                const std::size_t s = up ? k : k - 1;
                const std::size_t nx = up ? k + 1 : k - 1;
                const Vec2 cur = g.xy(nodes[k]);
                dip(k);
                feedTo(cur, seg[s]);
                feedTo(g.xy(nodes[nx]), seg[s]);
                k = nx;
            }
            dip(b);
        };
        double zFrom = -kInf;
        std::size_t m = 0;
        if (p.rampPlunge) {
            const double tanA = std::tan(p.rampAngle);
            zFrom = std::min(p.safeZ, static_cast<double>(scur[nodes[0]]));
            for (int it = 0; it < 4; ++it) {
                const double drop = std::max(0.0, zFrom - seg[0]);
                const std::size_t kmax = std::min(n - 1, static_cast<std::size_t>(std::ceil(drop / (tanA * g.cs))) + 2);
                double mx = zFrom;
                for (std::size_t k = 0; k <= kmax; ++k) {
                    mx = std::max(mx, static_cast<double>(scur[nodes[k]]));
                }
                mx = std::min(mx, p.safeZ);
                if (mx == zFrom) {
                    break;
                }
                zFrom = mx;
            }
            if (zFrom > seg[0]) {
                // Ramp forward to the first node whose ramp height has reached the profile.
                m = n - 1;
                for (std::size_t k = 0; k < n; ++k) {
                    if (zFrom - tanA * static_cast<double>(k) * g.cs <= adj(k)) {
                        m = k;
                        break;
                    }
                }
                add(Move::plunge(geometry::Vec3{first, zFrom}));
                for (std::size_t k = 1; k <= m; ++k) {
                    const double zr = std::max(adj(k), zFrom - tanA * static_cast<double>(k) * g.cs);
                    add(Move::feed(geometry::Vec3{g.xy(nodes[k]), zr}));
                }
                stair(m, 0);
                stair(0, n - 1);
                toSafe();
                return;
            }
        }
        add(Move::plunge(geometry::Vec3{first, zn[0] + p.tolerance < seg[0] ? zn[0] : seg[0]}));
        stair(0, n - 1);
        toSafe();
    }
};

}  // namespace

Rough3dResult rough3dToolpath(const relief::HeightField& model, const std::vector<Rough3dTool>& tools,
                              const Rough3dParams& p, const ToolpathRunControl& control) {
    Rough3dResult res;
    auto bad = [&](const std::string& msg) {
        addDiag(res, ToolpathIssue::InvalidParameters, msg);
        return res;
    };
    auto fin = [](double v) { return std::isfinite(v); };
    if (tools.empty()) {
        addDiag(res, ToolpathIssue::NoTool, "3D rough needs at least one tool");
        return res;
    }
    res.toolpaths.resize(tools.size());
    for (std::size_t k = 0; k < tools.size(); ++k) {
        res.toolpaths[k].name = tools.size() == 1 ? p.name : p.name + " [" + tools[k].tool.name + "]";
    }
    if (model.empty() || relief::validate(model.grid()) != relief::Status::Ok) {
        return bad("the model has no valid height field");
    }
    if (!fin(p.zOffset) || !fin(p.stockTop) || !fin(p.stockBottom) || !fin(p.allowance) || !fin(p.safeZ) ||
        !fin(p.boundaryOffset) || !fin(p.rampAngle) || !fin(p.tolerance) || !fin(p.restTolerance)) {
        return bad("a parameter is not finite");
    }
    if (!(p.stockBottom < p.stockTop) || p.allowance < 0.0 || !(p.safeZ > p.stockTop) || p.tolerance < 0.0 ||
        p.restTolerance < 0.0 || !(p.rampAngle > 0.0 && p.rampAngle < 1.5)) {
        return bad("stock top/bottom, safe Z, allowance, tolerance or ramp angle out of range");
    }
    if (p.boundary == Rough3dBoundary::Vectors) {
        bool any = false;
        for (const geometry::Polyline& pl : p.vectors) {
            any = any || pl.points.size() >= 3;
            for (const Vec2& v : pl.points) {
                if (!fin(v.x) || !fin(v.y)) {
                    return bad("a boundary vector has a non-finite point");
                }
            }
        }
        if (!any) {
            addDiag(res, ToolpathIssue::NoRegion, "no closed boundary vector selected");
            return res;
        }
    }
    Ctx c{p, control, res};
    c.workLimit = control.workLimit != 0 ? control.workLimit : kRough3dDefaultWorkLimit;
    c.moveLimit = control.moveLimit != 0 ? control.moveLimit : kRough3dDefaultMoveLimit;
    const std::size_t w = model.width();
    const std::size_t h = model.height();
    const std::size_t N = w * h;
    Grid3 g;
    g.model = &model;
    g.cs = model.cellSize();
    g.origin = {model.origin().x, model.origin().y};
    if (N >= static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
        return bad("the model is too large");
    }
    auto cancelOut = [&]() {
        for (Toolpath& t : res.toolpaths) {
            t.moves.clear();
        }
        addDiag(res, ToolpathIssue::Cancelled, "3D rough cancelled");
        return res;
    };
    auto limitOut = [&](const std::string& why) {
        for (Toolpath& t : res.toolpaths) {
            t.moves.clear();
        }
        addDiag(res, ToolpathIssue::WorkLimitExceeded, why);
        return res;
    };

    std::vector<std::uint8_t> mask;
    if (!buildMask(c, g, mask)) {
        return c.abort ? cancelOut() : limitOut("the boundary needs more work than allowed");
    }
    {
        bool any = false;
        for (std::uint8_t v : mask) {
            any = any || v != 0;
        }
        if (!any) {
            addDiag(res, ToolpathIssue::NoRegion, "the machining boundary encloses no model cell");
            return res;
        }
    }

    const Frame fr{w, h, p.axis == RasterAxis::AlongX};
    std::vector<float> scur(N, static_cast<float>(p.stockTop));
    std::vector<float> T;
    std::vector<std::uint8_t> elig(N, 0);
    std::vector<simulate::ToolShape> shapes;

    for (std::size_t ti = 0; ti < tools.size(); ++ti) {
        const Rough3dTool& rt = tools[ti];
        Toolpath& tp = res.toolpaths[ti];
        const auto shape = simulate::ToolShape::fromTool(rt.tool, rt.fluteLength);
        if (!shape) {
            addDiag(res, ToolpathIssue::InvalidTool, "tool '" + rt.tool.name + "' has no usable shape", ti);
            continue;
        }
        const tooling::CuttingData& cd = effectiveCuttingData(rt.tool, rt.cuttingData);
        if (!(cd.stepover > 0.0) || !fin(cd.stepover) || !(cd.passDepth > 0.0) || !fin(cd.passDepth)) {
            addDiag(res, ToolpathIssue::InvalidTool, "tool '" + rt.tool.name + "' needs a stepover and pass depth > 0", ti);
            continue;
        }
        tp.tool = toolSnapshot(rt.tool, cd);
        shapes.push_back(*shape);

        // Rest machining: the surface the earlier tools left, by simulation.
        if (ti > 0) {
            simulate::Stock st;
            st.width = (static_cast<double>(w) - 0.25) * g.cs;
            st.height = (static_cast<double>(h) - 0.25) * g.cs;
            st.thickness = p.stockTop - p.stockBottom;
            st.cellSize = g.cs;
            st.origin = geometry::Vec2{g.origin.x - 0.5 * g.cs, g.origin.y - 0.5 * g.cs};
            st.topZ = p.stockTop;
            st.trackPassIds = false;
            const simulate::StockCheck chk = simulate::checkStock(st);
            if (!chk.ok() || chk.nx != w || chk.ny != h) {
                return limitOut("rest machining cannot simulate the stock: " + chk.message);
            }
            simulate::SimOptions so;
            so.maxCellWork = static_cast<double>(c.workLimit) * 4.0;
            simulate::Simulator sim(st, so);
            if (!sim.ok()) {
                return limitOut("rest machining simulation: " + sim.error().message);
            }
            std::uint32_t id = 1;
            for (std::size_t q = 0; q < ti; ++q) {
                if (res.toolpaths[q].moves.empty()) {
                    continue;
                }
                if (!sim.addToolpath(res.toolpaths[q], shapes[q], id++)) {
                    return limitOut("rest machining simulation: " + sim.error().message);
                }
            }
            for (std::size_t mv = 0; mv < sim.moveCount();) {
                if (c.isCancelled()) {
                    return cancelOut();
                }
                mv = std::min(sim.moveCount(), mv + 2048);
                if (!sim.advanceToMove(mv)) {
                    return limitOut("rest machining simulation: " + sim.error().message);
                }
            }
            const simulate::Heightfield& hf = sim.heightfield();
            for (std::size_t j = 0; j < h; ++j) {
                for (std::size_t i = 0; i < w; ++i) {
                    scur[j * w + i] = hf.height(i, j);
                }
            }
        }

        // Drop surface and tip targets.
        if (p.tip == Rough3dTipMode::DropTool) {
            const DropStatus ds = dropToolSurface(model, *shape, p.zOffset, T, c.workLimit, c.work, control.cancel);
            if (ds == DropStatus::Cancelled) {
                return cancelOut();
            }
            if (ds != DropStatus::Ok) {
                return limitOut("the drop-tool surface needs more work than allowed (tool too large for the grid "
                                "resolution, or work limit)");
            }
        } else {
            T.assign(N, -std::numeric_limits<float>::infinity());
            for (std::size_t k = 0; k < N; ++k) {
                const float v = model.data()[k];
                if (!std::isnan(v)) {
                    T[k] = roundUp(static_cast<double>(v) + p.zOffset);
                }
            }
        }
        const float bottom = roundUp(p.stockBottom);
        for (std::size_t k = 0; k < N; ++k) {
            const double t = static_cast<double>(T[k]) + p.allowance;
            T[k] = std::max(roundUp(t), bottom);
        }

        // Nodes this tool must clear.
        const bool rest = ti > 0;
        std::size_t eligible = 0;
        float lowest = std::numeric_limits<float>::infinity();
        float highestScur = -std::numeric_limits<float>::infinity();
        for (std::size_t k = 0; k < N; ++k) {
            bool e = mask[k] != 0;
            if (e && rest) {
                e = static_cast<double>(scur[k]) > static_cast<double>(T[k]) + p.restTolerance;
            }
            elig[k] = e ? 1 : 0;
            if (e) {
                ++eligible;
                lowest = std::min(lowest, T[k]);
                highestScur = std::max(highestScur, scur[k]);
            }
        }
        if (eligible == 0) {
            addDiag(res, rest ? ToolpathIssue::NoRestMaterial : ToolpathIssue::NoRegion,
                    rest ? "the earlier tools left nothing for this tool" : "nothing to cut", ti);
            continue;
        }
        if (!(static_cast<double>(lowest) < static_cast<double>(highestScur))) {
            addDiag(res, ToolpathIssue::NoRegion, "the model is not below the stock top: nothing to rough", ti);
            continue;
        }

        const double cs = g.cs;
        const std::size_t kStep = std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(cd.stepover / cs + 1e-9)));
        // Row sequence.
        std::vector<std::size_t> rowSeq;
        for (std::size_t r = 0; r < fr.rows(); r += kStep) {
            rowSeq.push_back(r);
        }
        if (rowSeq.back() != fr.rows() - 1) {
            rowSeq.push_back(fr.rows() - 1);
        }
        // Climb: material stays on the right of the +column travel: AlongX steps towards -Y
        // (descending row index), AlongY towards +X (ascending row index).
        const bool descending = (p.axis == RasterAxis::AlongX) == (p.direction == CutDirection::Climb);
        bool flip = descending;
        if (p.reverseSteps) {
            flip = !flip;
        }
        if (flip) {
            std::reverse(rowSeq.begin(), rowSeq.end());
        }

        Emitter em{g, p, tp, c.moveLimit, std::nullopt, false};
        std::vector<std::uint32_t> chain;
        std::vector<double> zn;
        auto flush = [&]() {
            if (!chain.empty()) {
                em.emitChain(chain, zn, scur);
                chain.clear();
                zn.clear();
            }
        };
        // One pass of rows with eligibility `okNode`, link nodes `okLink` and node heights `zOf`.
        // In zig-zag travel a run that reaches the row's far end continues into the next row along
        // the column when every node between the two rows may be visited.
        std::size_t chainEndRow = 0;
        std::size_t chainEndPos = 0;
        auto pass = [&](auto&& okNode, auto&& okLink, auto&& zOf, std::size_t r0, std::size_t r1, std::size_t c0,
                        std::size_t c1) -> bool {
            std::vector<std::size_t> rowsHere;
            for (std::size_t rw : rowSeq) {
                if (rw >= r0 && rw <= r1) {
                    rowsHere.push_back(rw);
                }
            }
            for (std::size_t s = 0; s < rowsHere.size(); ++s) {
                if ((s & 63u) == 0 && c.isCancelled()) {
                    return false;
                }
                const std::size_t row = rowsHere[s];
                const bool back = p.travel == RasterTravel::ZigZag && (s & 1u) != 0;
                const std::size_t nc = c1 - c0 + 1;
                c.work += nc;
                std::size_t col = 0;  // index in travel order
                while (col < nc) {
                    auto posOf = [&](std::size_t t) { return back ? c1 - t : c0 + t; };
                    if (!okNode(fr.node(row, posOf(col)))) {
                        ++col;
                        continue;
                    }
                    const std::size_t startPos = posOf(col);
                    std::vector<std::uint32_t> run;
                    while (col < nc && okNode(fr.node(row, posOf(col)))) {
                        run.push_back(static_cast<std::uint32_t>(fr.node(row, posOf(col))));
                        ++col;
                    }
                    bool linked = false;
                    if (!chain.empty() && p.travel == RasterTravel::ZigZag && s > 0 && chainEndRow == rowsHere[s - 1] &&
                        chainEndPos == startPos) {
                        const bool up = row > chainEndRow;
                        bool ok = true;
                        for (std::size_t rr = chainEndRow; rr != row && ok;) {
                            rr = up ? rr + 1 : rr - 1;
                            ok = okLink(fr.node(rr, startPos));
                        }
                        if (ok) {
                            for (std::size_t rr = up ? chainEndRow + 1 : chainEndRow - 1; rr != row;
                                 rr = up ? rr + 1 : rr - 1) {
                                chain.push_back(static_cast<std::uint32_t>(fr.node(rr, startPos)));
                                zn.push_back(zOf(chain.back()));
                            }
                            linked = true;
                        }
                    }
                    if (!linked) {
                        flush();
                    }
                    for (std::uint32_t nd : run) {
                        chain.push_back(nd);
                        zn.push_back(zOf(nd));
                    }
                    chainEndRow = row;
                    chainEndPos = posOf(col - 1);
                    // Only a run reaching the far end of the row can link onward.
                    if (col < nc || p.travel == RasterTravel::OneWay) {
                        flush();
                    }
                    if (em.overLimit) {
                        return true;
                    }
                }
            }
            flush();
            return true;
        };

        bool okRun = true;
        if (p.strategy == Rough3dStrategy::Raster) {
            okRun = pass([&](std::size_t nd) { return elig[nd] != 0; }, [&](std::size_t nd) { return mask[nd] != 0; },
                         [&](std::size_t nd) { return static_cast<double>(T[nd]); }, 0, fr.rows() - 1, 0,
                         fr.cols() - 1);
        } else {
            const double sd = cd.passDepth;
            const double top = std::min(p.stockTop, static_cast<double>(highestScur));
            const double floorZ = static_cast<double>(lowest);
            std::vector<double> levels;
            for (std::size_t i = 1;; ++i) {
                const double L = top - static_cast<double>(i) * sd;
                if (!(L > floorZ + 1e-9)) {
                    break;
                }
                levels.push_back(L);
                if (levels.size() > 1'000'000) {
                    break;
                }
            }
            levels.push_back(floorZ);
            const std::size_t nl = levels.size();
            std::vector<float> levelsF(nl);
            for (std::size_t i = 0; i < nl; ++i) {
                levelsF[i] = roundUp(levels[i]);
            }
            // Node n is visited at the levels startLv[n] .. endLv[n] (T <= level, stock above the level).
            std::vector<std::uint32_t> startLv(N, 1);
            std::vector<std::uint32_t> endLv(N, 0);
            std::vector<std::uint32_t> nextLv(N, 0);
            for (std::size_t nd = 0; nd < N; ++nd) {
                if (elig[nd] == 0) {
                    continue;
                }
                std::size_t lo = 0;
                std::size_t hi = nl;  // first i with scur > L_i + 1e-9 (monotone)
                while (lo < hi) {
                    const std::size_t mid = (lo + hi) / 2;
                    if (static_cast<double>(scur[nd]) > levels[mid] + 1e-9) {
                        hi = mid;
                    } else {
                        lo = mid + 1;
                    }
                }
                const std::size_t st = lo;
                lo = 0;
                hi = nl;  // first i with levelsF[i] < T (T <= level fails)
                while (lo < hi) {
                    const std::size_t mid = (lo + hi) / 2;
                    if (T[nd] <= levelsF[mid]) {
                        lo = mid + 1;
                    } else {
                        hi = mid;
                    }
                }
                if (lo > st) {
                    startLv[nd] = static_cast<std::uint32_t>(st);
                    endLv[nd] = static_cast<std::uint32_t>(lo - 1);
                    nextLv[nd] = static_cast<std::uint32_t>(st);
                }
            }
            auto active = [&](std::size_t nd, std::size_t li) {
                return elig[nd] != 0 && startLv[nd] <= endLv[nd] && nextLv[nd] == li && li <= endLv[nd];
            };
            const bool offsetClear = p.clearing == Rough3dClearing::Offset;
            const std::size_t Wd = w;
            std::vector<std::uint32_t> mark(N, 0);
            std::uint32_t stamp = 0;
            std::vector<std::int32_t> dist(N, -1);
            std::vector<std::uint32_t> pend(N, 0);
            std::uint32_t pendStamp = 0;
            std::vector<std::uint32_t> seen(N, 0);
            std::vector<std::uint32_t> par(N, 0);
            std::uint32_t seenStamp = 0;
            struct Dir {
                long dx, dy;
            };
            // The first four are the 4-neighbours (walk and link steps); all eight give the ring distance.
            const Dir dirs[8] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
            auto nbr = [&](std::uint32_t nd, int k, std::uint32_t& out) {  // k < 8
                const long i = static_cast<long>(nd % Wd) + dirs[k].dx;
                const long j = static_cast<long>(nd / Wd) + dirs[k].dy;
                if (i < 0 || j < 0 || i >= static_cast<long>(w) || j >= static_cast<long>(h)) {
                    return false;
                }
                out = static_cast<std::uint32_t>(static_cast<std::size_t>(j) * w + static_cast<std::size_t>(i));
                return true;
            };
            const std::size_t linkDepth = 2 * kStep + 2;
            // Contour rings of a unit: ring k = unit nodes at 8-neighbour (Chebyshev) distance k from outside the
            // unit, so the ring around a rectangle or an island is a 4-connected loop (ADR-0082).
            std::vector<std::vector<std::uint32_t>> rings;
            auto prepareRings = [&](const std::vector<std::uint32_t>& nodes) {
                std::vector<std::uint32_t> q;
                for (std::uint32_t nd : nodes) {
                    dist[nd] = -1;
                }
                for (std::uint32_t nd : nodes) {
                    bool edge = false;
                    for (int k = 0; k < 8 && !edge; ++k) {
                        std::uint32_t o = 0;
                        edge = !nbr(nd, k, o) || mark[o] != stamp;
                    }
                    if (edge) {
                        dist[nd] = 0;
                        q.push_back(nd);
                    }
                }
                std::int32_t maxD = 0;
                for (std::size_t hd = 0; hd < q.size(); ++hd) {
                    const std::uint32_t nd = q[hd];
                    maxD = std::max(maxD, dist[nd]);
                    for (int k = 0; k < 8; ++k) {
                        std::uint32_t o = 0;
                        if (nbr(nd, k, o) && mark[o] == stamp && dist[o] < 0) {
                            dist[o] = dist[nd] + 1;
                            q.push_back(o);
                        }
                    }
                }
                c.work += nodes.size() * 4;
                rings.assign(static_cast<std::size_t>(maxD) + 1, {});
                for (std::uint32_t nd : nodes) {
                    rings[static_cast<std::size_t>(dist[nd])].push_back(nd);
                }
            };
            // Cuts ring `k` of the prepared unit at level height `Lf`, wall-following with the region
            // interior on the left of the travel (climb) or the right (conventional).
            auto walkRing = [&](std::size_t k, double Lf) {
                const std::vector<std::uint32_t>& rn = rings[k];
                if (rn.empty()) {
                    return;
                }
                const std::uint32_t ps = ++pendStamp;
                std::size_t remaining = rn.size();
                for (std::uint32_t nd : rn) {
                    pend[nd] = ps;
                }
                auto okLinkNode = [&](std::uint32_t nd) {
                    if (p.avoidMachined) {
                        return mark[nd] == stamp;
                    }
                    return mask[nd] != 0 && static_cast<double>(T[nd]) <= Lf;
                };
                auto visit = [&](std::uint32_t nd) {
                    chain.push_back(nd);
                    zn.push_back(Lf);
                    if (pend[nd] == ps) {
                        pend[nd] = 0;
                        --remaining;
                    }
                };
                std::size_t startPtr = 0;
                std::vector<std::uint32_t> bq;
                while (remaining > 0 && !em.overLimit) {
                    bool linked = false;
                    if (!chain.empty()) {
                        // Shortest link through allowed nodes to a pending node of this ring.
                        const std::uint32_t from = chain.back();
                        const std::uint32_t sd2 = ++seenStamp;
                        bq.assign(1, from);
                        seen[from] = sd2;
                        std::size_t head = 0;
                        std::size_t levelEnd = 1;
                        std::size_t depth = 0;
                        std::uint32_t found = 0;
                        bool got = false;
                        while (head < bq.size() && !got && depth < linkDepth) {
                            for (; head < levelEnd && !got; ++head) {
                                const std::uint32_t cur = bq[head];
                                ++c.work;
                                for (int d = 0; d < 4 && !got; ++d) {
                                    std::uint32_t o = 0;
                                    if (!nbr(cur, d, o) || seen[o] == sd2 || !okLinkNode(o)) {
                                        continue;
                                    }
                                    seen[o] = sd2;
                                    par[o] = cur;
                                    bq.push_back(o);
                                    if (pend[o] == ps) {
                                        found = o;
                                        got = true;
                                    }
                                }
                            }
                            levelEnd = bq.size();
                            ++depth;
                        }
                        if (got) {
                            std::vector<std::uint32_t> path;
                            for (std::uint32_t x = found; x != from; x = par[x]) {
                                path.push_back(x);
                            }
                            for (std::size_t pi = path.size(); pi-- > 0;) {
                                visit(path[pi]);
                            }
                            linked = true;
                        } else {
                            flush();
                        }
                    }
                    if (!linked) {
                        while (startPtr < rn.size() && pend[rn[startPtr]] != ps) {
                            ++startPtr;
                        }
                        visit(rn[startPtr]);
                    }
                    // Greedy wall-following walk over pending ring nodes.
                    while (remaining > 0) {
                        const std::uint32_t cur = chain.back();
                        long ox = 0;
                        long oy = 0;
                        for (int d = 0; d < 8; ++d) {
                            std::uint32_t o = 0;
                            if (!nbr(cur, d, o) || mark[o] != stamp || dist[o] < dist[cur]) {
                                ox += dirs[d].dx;
                                oy += dirs[d].dy;
                            }
                        }
                        long hx = -oy;
                        long hy = ox;
                        if (hx == 0 && hy == 0 && chain.size() >= 2) {
                            const std::uint32_t pr = chain[chain.size() - 2];
                            hx = static_cast<long>(cur % Wd) - static_cast<long>(pr % Wd);
                            hy = static_cast<long>(cur / Wd) - static_cast<long>(pr / Wd);
                        } else if (p.direction != CutDirection::Climb) {
                            hx = -hx;
                            hy = -hy;
                        }
                        long bestScore = std::numeric_limits<long>::min();
                        std::uint32_t bestNode = 0;
                        bool any = false;
                        for (int d = 0; d < 4; ++d) {
                            std::uint32_t o = 0;
                            if (!nbr(cur, d, o) || pend[o] != ps) {
                                continue;
                            }
                            const long sc = dirs[d].dx * hx + dirs[d].dy * hy;
                            if (!any || sc > bestScore) {
                                any = true;
                                bestScore = sc;
                                bestNode = o;
                            }
                        }
                        if (!any) {
                            break;
                        }
                        visit(bestNode);
                    }
                    ++c.work;
                }
            };
            // Cuts one unit (a set of nodes) at level `li`.
            auto cutUnit = [&](std::size_t li, const std::vector<std::uint32_t>& nodes, bool boxed) -> bool {
                if (nodes.empty()) {
                    return true;
                }
                const float Lf = levelsF[li];
                ++stamp;
                for (std::uint32_t nd : nodes) {
                    mark[nd] = stamp;
                }
                c.work += nodes.size();
                std::size_t r0 = 0;
                std::size_t r1 = fr.rows() - 1;
                std::size_t c0 = 0;
                std::size_t c1 = fr.cols() - 1;
                if (boxed) {
                    r0 = c0 = std::numeric_limits<std::size_t>::max();
                    r1 = c1 = 0;
                    for (std::uint32_t nd : nodes) {
                        const std::size_t rw = fr.alongX ? nd / w : nd % w;
                        const std::size_t cl = fr.alongX ? nd % w : nd / w;
                        r0 = std::min(r0, rw);
                        r1 = std::max(r1, rw);
                        c0 = std::min(c0, cl);
                        c1 = std::max(c1, cl);
                    }
                }
                const bool avoid = p.avoidMachined;
                auto okNode = [&](std::size_t nd) { return mark[nd] == stamp; };
                auto okLink = [&](std::size_t nd) {
                    return avoid ? mark[nd] == stamp : (mask[nd] != 0 && T[nd] <= Lf);
                };
                auto zOf = [&](std::size_t) { return static_cast<double>(Lf); };
                bool ok = true;
                const bool profileOnly = !offsetClear && p.profile != Rough3dProfile::None;
                if (offsetClear || profileOnly) {
                    prepareRings(nodes);
                }
                if (offsetClear) {
                    std::vector<std::size_t> order;
                    for (std::size_t k = 0; k < rings.size(); ++k) {
                        if (k == 0 || k % kStep == 0 || k + 1 == rings.size()) {
                            order.push_back(k);
                        }
                    }
                    if (p.profile == Rough3dProfile::Last && order.size() > 1) {
                        std::rotate(order.begin(), order.begin() + 1, order.end());
                    }
                    for (std::size_t k : order) {
                        if (c.isCancelled()) {
                            return false;
                        }
                        walkRing(k, static_cast<double>(Lf));
                    }
                } else {
                    if (profileOnly && p.profile == Rough3dProfile::First) {
                        walkRing(0, static_cast<double>(Lf));
                    }
                    ok = pass(okNode, okLink, zOf, r0, r1, c0, c1);
                    if (ok && profileOnly && p.profile == Rough3dProfile::Last) {
                        walkRing(0, static_cast<double>(Lf));
                    }
                }
                flush();
                for (std::uint32_t nd : nodes) {
                    scur[nd] = Lf;
                    nextLv[nd] = static_cast<std::uint32_t>(li + 1);
                }
                return ok;
            };
            if (p.order == Rough3dOrder::LevelByLevel) {
                for (std::size_t li = 0; li < nl && okRun && !em.overLimit; ++li) {
                    std::vector<std::uint32_t> nodes;
                    for (std::size_t nd = 0; nd < N; ++nd) {
                        if (active(nd, li)) {
                            nodes.push_back(static_cast<std::uint32_t>(nd));
                        }
                    }
                    okRun = cutUnit(li, nodes, false);
                }
            } else {
                // Depth-first: components of the nodes that start at a level are cut, then their
                // sub-components at the next levels before the next component (explicit stack).
                struct Frame2 {
                    std::size_t level;
                    std::vector<std::uint32_t> nodes;
                };
                std::vector<std::uint32_t> inS(N, 0);
                std::uint32_t inStamp = 0;
                // Components of `cand` (ascending), pushed so the first pops first.
                auto components = [&](const std::vector<std::uint32_t>& cand, std::size_t level,
                                      std::vector<Frame2>& stack) {
                    const std::uint32_t sm = ++inStamp;
                    for (std::uint32_t nd : cand) {
                        inS[nd] = sm;
                    }
                    std::vector<std::vector<std::uint32_t>> comps;
                    for (std::uint32_t s0 : cand) {
                        if (inS[s0] != sm) {
                            continue;
                        }
                        std::vector<std::uint32_t> comp{s0};
                        inS[s0] = 0;
                        for (std::size_t hd = 0; hd < comp.size(); ++hd) {
                            for (int k = 0; k < 4; ++k) {
                                std::uint32_t o = 0;
                                if (nbr(comp[hd], k, o) && inS[o] == sm) {
                                    inS[o] = 0;
                                    comp.push_back(o);
                                }
                            }
                        }
                        std::sort(comp.begin(), comp.end());
                        comps.push_back(std::move(comp));
                        c.work += comps.back().size();
                    }
                    for (std::size_t ci = comps.size(); ci-- > 0;) {
                        stack.push_back(Frame2{level, std::move(comps[ci])});
                    }
                };
                for (std::size_t li = 0; li < nl && okRun && !em.overLimit; ++li) {
                    std::vector<std::uint32_t> roots;
                    for (std::size_t nd = 0; nd < N; ++nd) {
                        if (active(nd, li)) {
                            roots.push_back(static_cast<std::uint32_t>(nd));
                        }
                    }
                    std::vector<Frame2> stack;
                    components(roots, li, stack);
                    while (!stack.empty() && okRun && !em.overLimit) {
                        if (c.isCancelled()) {
                            okRun = false;
                            break;
                        }
                        Frame2 f = std::move(stack.back());
                        stack.pop_back();
                        okRun = cutUnit(f.level, f.nodes, true);
                        if (!okRun || f.level + 1 >= nl) {
                            continue;
                        }
                        std::vector<std::uint32_t> next;
                        for (std::uint32_t nd : f.nodes) {
                            if (active(nd, f.level + 1)) {
                                next.push_back(nd);
                            }
                        }
                        components(next, f.level + 1, stack);
                    }
                }
            }
        }
        if (!okRun) {
            return cancelOut();
        }
        if (em.overLimit) {
            return limitOut("the toolpath would exceed the move limit");
        }
        if (c.work > c.workLimit) {
            return limitOut("the toolpath needs more work than allowed");
        }
        em.toSafe();
    }
    return res;
}

}  // namespace carvetto::toolpath
