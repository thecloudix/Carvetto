#include "carvetto/toolpath/finish3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <utility>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/simulate/stock.hpp"

namespace carvetto::toolpath {

double ballScallopHeight(double toolRadius, double spacing) {
    const double half = 0.5 * spacing;
    if (!(toolRadius > 0.0) || !(half < toolRadius)) {
        return toolRadius;
    }
    return toolRadius - std::sqrt(toolRadius * toolRadius - half * half);
}

namespace {

using geometry::Vec2;
using geometry::Vec3;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::size_t kBlock = 16;
constexpr int kMaxDepth = 5;

bool cancelled(const std::atomic<bool>* c) {
    return c != nullptr && c->load(std::memory_order_relaxed);
}

/// Rounds a double up to the next float (never below the double); returned as double.
double roundUp(double v) {
    if (std::isnan(v)) {
        return -kInf;
    }
    constexpr double fmax = static_cast<double>(std::numeric_limits<float>::max());
    if (v >= fmax) {
        return fmax;
    }
    if (v <= -fmax) {
        return -fmax;
    }
    float f = static_cast<float>(v);
    if (static_cast<double>(f) < v) {
        f = std::nextafter(f, std::numeric_limits<float>::infinity());
    }
    return static_cast<double>(f);
}

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

/// Squared Euclidean distance (cells^2) from every cell to the nearest source cell (exact; 1e20 when
/// there is no source). Felzenszwalb-Huttenlocher lower envelopes.
std::vector<double> edtSq(const std::vector<std::uint8_t>& src, std::size_t W, std::size_t H) {
    constexpr double big = 1e20;
    std::vector<double> g(W * H);
    for (std::size_t k = 0; k < W * H; ++k) {
        g[k] = src[k] != 0 ? 0.0 : big;
    }
    const std::size_t m = std::max(W, H);
    std::vector<double> f(m);
    std::vector<double> d(m);
    std::vector<double> z(m + 1);
    std::vector<std::size_t> v(m);
    auto line = [&](std::size_t n) {
        std::size_t k = 0;
        v[0] = 0;
        z[0] = -kInf;
        z[1] = kInf;
        for (std::size_t q = 1; q < n; ++q) {
            const double qd = static_cast<double>(q);
            auto cross = [&](std::size_t p) {
                const double pd = static_cast<double>(p);
                return ((f[q] + qd * qd) - (f[p] + pd * pd)) / (2.0 * qd - 2.0 * pd);
            };
            double s = cross(v[k]);
            while (s <= z[k]) {
                --k;
                s = cross(v[k]);
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = kInf;
        }
        k = 0;
        for (std::size_t q = 0; q < n; ++q) {
            while (z[k + 1] < static_cast<double>(q)) {
                ++k;
            }
            const double dq = static_cast<double>(q) - static_cast<double>(v[k]);
            d[q] = dq * dq + f[v[k]];
        }
    };
    for (std::size_t x = 0; x < W; ++x) {
        for (std::size_t y = 0; y < H; ++y) {
            f[y] = g[y * W + x];
        }
        line(H);
        for (std::size_t y = 0; y < H; ++y) {
            g[y * W + x] = d[y];
        }
    }
    for (std::size_t y = 0; y < H; ++y) {
        for (std::size_t x = 0; x < W; ++x) {
            f[x] = g[y * W + x];
        }
        line(W);
        for (std::size_t x = 0; x < W; ++x) {
            g[y * W + x] = d[x];
        }
    }
    return g;
}

struct Grid3 {
    const relief::HeightField* model{nullptr};
    double cs{1.0};
    Vec2 origin;
    std::size_t w{0};
    std::size_t h{0};
    Vec2 xy(std::size_t node) const {
        return {origin.x + static_cast<double>(node % w) * cs, origin.y + static_cast<double>(node / w) * cs};
    }
    /// Nearest node of a point inside the node rectangle (tolerance 1e-9), else -1.
    long nodeAt(Vec2 p) const {
        const double fx = (p.x - origin.x) / cs;
        const double fy = (p.y - origin.y) / cs;
        constexpr double e = 1e-9;
        if (fx < -e || fy < -e || fx > static_cast<double>(w - 1) + e || fy > static_cast<double>(h - 1) + e) {
            return -1;
        }
        const long i = std::clamp(std::lround(fx), 0L, static_cast<long>(w) - 1);
        const long j = std::clamp(std::lround(fy), 0L, static_cast<long>(h) - 1);
        return j * static_cast<long>(w) + i;
    }
};

struct Ctx {
    const Finish3dParams& p;
    const ToolpathRunControl& control;
    std::uint64_t workLimit{0};
    std::uint64_t moveLimit{0};
    std::uint64_t work{0};
    bool abort{false};  // cancelled
    bool over{false};   // work or move limit
    bool stop() {
        if (!abort && cancelled(control.cancel)) {
            abort = true;
        }
        if (work > workLimit) {
            over = true;
        }
        return abort || over;
    }
};

/// Machining region mask over the model nodes.
bool buildMask(Ctx& c, const Grid3& g, std::vector<std::uint8_t>& mask) {
    const relief::HeightField& m = *g.model;
    const std::size_t w = g.w;
    const std::size_t h = g.h;
    const Finish3dParams& p = c.p;
    mask.assign(w * h, 0);
    if (p.boundary == Rough3dBoundary::Material) {
        std::fill(mask.begin(), mask.end(), std::uint8_t{1});
        return true;
    }
    if (p.boundary == Rough3dBoundary::Model) {
        const bool grow = p.boundaryOffset > 0.0;
        const double off = std::fabs(p.boundaryOffset);
        if (p.boundaryOffset == 0.0) {
            for (std::size_t k = 0; k < w * h; ++k) {
                mask[k] = std::isnan(m.data()[k]) ? 0 : 1;
            }
            return true;
        }
        // Padded by one cell so that the outside of the grid counts as transparent when shrinking.
        const std::size_t W = w + 2;
        const std::size_t H = h + 2;
        std::vector<std::uint8_t> src(W * H, grow ? 0 : 1);
        for (std::size_t j = 0; j < h; ++j) {
            for (std::size_t i = 0; i < w; ++i) {
                const bool opaque = !std::isnan(m.at(i, j));
                src[(j + 1) * W + (i + 1)] = (grow ? opaque : !opaque) ? 1 : 0;
            }
        }
        c.work += W * H * 8;
        const std::vector<double> d2 = edtSq(src, W, H);
        const double lim = off * off * (1.0 + 1e-9);
        for (std::size_t j = 0; j < h; ++j) {
            for (std::size_t i = 0; i < w; ++i) {
                const double dd = d2[(j + 1) * W + (i + 1)] * g.cs * g.cs;
                const bool opaque = !std::isnan(m.at(i, j));
                mask[j * w + i] = (grow ? dd <= lim : (opaque && dd > lim)) ? 1 : 0;
            }
        }
        return true;
    }
    std::vector<geometry::Polyline> polys;
    std::size_t edges = 0;
    for (const geometry::Polyline& pl : p.vectors) {
        if (pl.points.size() >= 3) {
            polys.push_back(pl);
            edges += pl.points.size();
        }
    }
    for (std::size_t j = 0; j < h; ++j) {
        if (c.stop()) {
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
    }
    return !c.stop();
}

/// Exact drop heights of points and straight pieces over the model (ADR-0072).
class DropEval {
public:
    DropEval(const Grid3& g, const simulate::ToolShape& tool, const Finish3dParams& p, Ctx& c)
        : g_(g), tool_(tool), p_(p), c_(c) {
        reach_ = tool.radius() * (1.0 + 1e-9) + 1e-9;
        bw_ = (g.w + kBlock - 1) / kBlock;
        bh_ = (g.h + kBlock - 1) / kBlock;
        bmax_.assign(bw_ * bh_, -kInf);
        for (std::size_t j = 0; j < g.h; ++j) {
            for (std::size_t i = 0; i < g.w; ++i) {
                const float v = g.model->at(i, j);
                if (!std::isnan(v)) {
                    double& m = bmax_[(j / kBlock) * bw_ + (i / kBlock)];
                    m = std::max(m, static_cast<double>(v) + p.zOffset);
                }
            }
        }
        bottom_ = roundUp(p.stockBottom);
    }

    /// Lowest tip Z (allowance included, clamped to the floor, rounded up to float) for a tool swept at
    /// constant height from a to b (a == b: a point).
    double seg(Vec2 a, Vec2 b) {
        const double cs = g_.cs;
        const double minx = std::min(a.x, b.x);
        const double maxx = std::max(a.x, b.x);
        const double miny = std::min(a.y, b.y);
        const double maxy = std::max(a.y, b.y);
        const long i0 = std::max(0L, static_cast<long>(std::floor((minx - reach_ - g_.origin.x) / cs)));
        const long i1 = std::min(static_cast<long>(g_.w) - 1, static_cast<long>(std::ceil((maxx + reach_ - g_.origin.x) / cs)));
        const long j0 = std::max(0L, static_cast<long>(std::floor((miny - reach_ - g_.origin.y) / cs)));
        const long j1 = std::min(static_cast<long>(g_.h) - 1, static_cast<long>(std::ceil((maxy + reach_ - g_.origin.y) / cs)));
        double best = -kInf;
        const long B = static_cast<long>(kBlock);
        for (long by = j0 / B; j0 <= j1 && by <= j1 / B; ++by) {
            for (long bx = i0 / B; i0 <= i1 && bx <= i1 / B; ++bx) {
                ++c_.work;
                const double bm = bmax_[static_cast<std::size_t>(by) * bw_ + static_cast<std::size_t>(bx)];
                if (bm == -kInf) {
                    continue;
                }
                const long ki0 = std::max(i0, bx * B);
                const long ki1 = std::min(i1, (bx + 1) * B - 1);
                const long kj0 = std::max(j0, by * B);
                const long kj1 = std::min(j1, (by + 1) * B - 1);
                const double rx0 = g_.origin.x + static_cast<double>(ki0) * cs;
                const double rx1 = g_.origin.x + static_cast<double>(ki1) * cs;
                const double ry0 = g_.origin.y + static_cast<double>(kj0) * cs;
                const double ry1 = g_.origin.y + static_cast<double>(kj1) * cs;
                const double gx = std::max({0.0, rx0 - maxx, minx - rx1});
                const double gy = std::max({0.0, ry0 - maxy, miny - ry1});
                const double lb = std::sqrt(gx * gx + gy * gy);
                if (lb > reach_ || bm - tool_.heightAt(std::min(lb, tool_.radius())) <= best) {
                    continue;
                }
                for (long qj = kj0; qj <= kj1; ++qj) {
                    for (long qi = ki0; qi <= ki1; ++qi) {
                        ++c_.work;
                        const float v = g_.model->at(static_cast<std::size_t>(qi), static_cast<std::size_t>(qj));
                        if (std::isnan(v)) {
                            continue;
                        }
                        const Vec2 q{g_.origin.x + static_cast<double>(qi) * cs, g_.origin.y + static_cast<double>(qj) * cs};
                        const double d = pointSegDist(q, a, b);
                        if (d > reach_) {
                            continue;
                        }
                        best = std::max(best, static_cast<double>(v) + p_.zOffset - tool_.heightAt(std::min(d, tool_.radius())));
                    }
                }
            }
        }
        if (best == -kInf) {
            return bottom_;
        }
        return std::max(roundUp(best + p_.allowance), bottom_);
    }
    double point(Vec2 a) { return seg(a, a); }

private:
    const Grid3& g_;
    const simulate::ToolShape& tool_;
    const Finish3dParams& p_;
    Ctx& c_;
    double reach_{0.0};
    double bottom_{0.0};
    std::size_t bw_{0};
    std::size_t bh_{0};
    std::vector<double> bmax_;
};

/// Turns polylines into moves: refinement of the pieces, exact heights, linking.
class PathWriter {
public:
    PathWriter(const Grid3& g, const Finish3dParams& p, Ctx& c, DropEval& drop, Toolpath& tp,
               const std::vector<std::uint8_t>& act, double linkMax)
        : g_(g), p_(p), c_(c), drop_(drop), tp_(tp), act_(act), linkMax_(linkMax) {
        minLen_ = g.cs / static_cast<double>(1 << kMaxDepth);
    }

    bool active(Vec2 q) const {
        const long n = g_.nodeAt(q);
        return n >= 0 && act_[static_cast<std::size_t>(n)] != 0;
    }

    /// Samples of a polyline at most one cell apart (closed polylines carry their start as the last point).
    std::vector<Vec2> densify(const std::vector<Vec2>& pts) const {
        std::vector<Vec2> out;
        for (std::size_t k = 0; k < pts.size(); ++k) {
            if (k == 0) {
                out.push_back(pts[0]);
                continue;
            }
            const Vec2 a = pts[k - 1];
            const Vec2 b = pts[k];
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            if (!(len > 1e-12)) {
                continue;
            }
            const std::size_t n = static_cast<std::size_t>(std::ceil(len / g_.cs - 1e-9));
            for (std::size_t s = 1; s <= std::max<std::size_t>(n, 1); ++s) {
                const double t = static_cast<double>(s) / static_cast<double>(std::max<std::size_t>(n, 1));
                out.push_back(s == std::max<std::size_t>(n, 1) ? b : Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
            }
        }
        return out;
    }

    /// Adds a polyline (already densified); cut only where it is active. `closed`: the last point
    /// equals the first.
    void addPolyline(std::vector<Vec2> pts, bool closed) {
        if (pts.empty()) {
            return;
        }
        std::vector<char> on(pts.size());
        bool all = true;
        std::size_t firstOff = pts.size();
        for (std::size_t k = 0; k < pts.size(); ++k) {
            on[k] = active(pts[k]) ? 1 : 0;
            if (on[k] == 0) {
                all = false;
                firstOff = std::min(firstOff, k);
            }
        }
        c_.work += pts.size();
        if (all) {
            addRun(pts);
            return;
        }
        if (closed && firstOff < pts.size()) {
            // Rotate so that the sequence starts at an inactive sample (then no run wraps around).
            std::vector<Vec2> r(pts.begin() + static_cast<long>(firstOff), pts.end() - 1);
            r.insert(r.end(), pts.begin(), pts.begin() + static_cast<long>(firstOff) + 1);
            std::vector<char> ron(r.size());
            for (std::size_t k = 0; k < r.size(); ++k) {
                ron[k] = active(r[k]) ? 1 : 0;
            }
            pts = std::move(r);
            on = std::move(ron);
        }
        std::size_t k = 0;
        while (k < pts.size()) {
            if (on[k] == 0) {
                ++k;
                continue;
            }
            std::vector<Vec2> run;
            while (k < pts.size() && on[k] != 0) {
                run.push_back(pts[k++]);
            }
            addRun(run);
        }
    }

    void addRun(const std::vector<Vec2>& run) {
        if (run.empty() || c_.stop()) {
            return;
        }
        if (!chain_.empty() && canLink(chain_.back(), run.front())) {
            std::size_t from = 0;
            if (chain_.back().x == run.front().x && chain_.back().y == run.front().y) {
                from = 1;
            }
            chain_.insert(chain_.end(), run.begin() + static_cast<long>(from), run.end());
            return;
        }
        flush();
        chain_ = run;
    }

    Vec2 lastPoint() const { return chain_.empty() ? lastEnd_ : chain_.back(); }
    bool hasLast() const { return !chain_.empty() || haveEnd_; }

    void flush() {
        if (chain_.empty()) {
            return;
        }
        lastEnd_ = chain_.back();
        haveEnd_ = true;
        emit(chain_);
        chain_.clear();
    }

    void finish() {
        flush();
        toSafe();
    }

private:
    bool canLink(Vec2 a, Vec2 b) const {
        const double len = std::hypot(b.x - a.x, b.y - a.y);
        if (len > linkMax_) {
            return false;
        }
        const std::size_t n = static_cast<std::size_t>(std::ceil(len / g_.cs));
        for (std::size_t s = 1; s < n; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(n);
            if (!active({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t})) {
                return false;
            }
        }
        return true;
    }

    void add(Move m) {
        if (tp_.moves.size() >= c_.moveLimit) {
            c_.over = true;
            return;
        }
        tp_.moves.push_back(m);
        pos_ = m.to;
    }
    void feedTo(Vec2 xy, double z) {
        if (!pos_) {
            return;
        }
        if (pos_->x == xy.x && pos_->y == xy.y && pos_->z == z) {
            return;
        }
        auto& mv = tp_.moves;
        if (mv.size() >= 2 && mv.back().kind == MoveKind::Feed && mv[mv.size() - 2].kind != MoveKind::Rapid) {
            const Vec3 a = mv[mv.size() - 2].to;
            const Vec3 b = mv.back().to;
            const double cr = (b.x - a.x) * (xy.y - b.y) - (b.y - a.y) * (xy.x - b.x);
            const double dt = (b.x - a.x) * (xy.x - b.x) + (b.y - a.y) * (xy.y - b.y);
            if (a.z == z && b.z == z && pos_->z == z && cr == 0.0 && dt > 0.0) {
                mv.back().to = Vec3{xy, z};
                pos_ = mv.back().to;
                return;
            }
        }
        add(Move::feed(Vec3{xy, z}));
    }
    void toSafe() {
        if (pos_ && pos_->z < p_.safeZ) {
            add(Move::retract(Vec3{pos_->x, pos_->y, p_.safeZ}));
        }
    }

    // Splits a piece until it leaves at most the tolerance above the drop surface at its ends and middle.
    void refine(Vec2 a, double za, Vec2 b, double zb, int depth) {
        const double zs = drop_.seg(a, b);
        const double len = std::hypot(b.x - a.x, b.y - a.y);
        if (depth < kMaxDepth && len > minLen_ && !c_.stop()) {
            const Vec2 mid{0.5 * (a.x + b.x), 0.5 * (a.y + b.y)};
            const double zm = drop_.point(mid);
            if (zs - std::min({za, zb, zm}) > p_.tolerance) {
                refine(a, za, mid, zm, depth + 1);
                refine(mid, zm, b, zb, depth + 1);
                return;
            }
        }
        pts_.push_back(b);
        zv_.push_back(zb);
        zs_.push_back(zs);
    }

    void emit(const std::vector<Vec2>& in) {
        if (c_.stop() || c_.over) {
            return;
        }
        pts_.clear();
        zv_.clear();
        zs_.clear();
        pts_.push_back(in[0]);
        zv_.push_back(drop_.point(in[0]));
        zs_.push_back(0.0);
        for (std::size_t k = 1; k < in.size(); ++k) {
            const double zb = drop_.point(in[k]);
            refine(in[k - 1], zv_.back(), in[k], zb, 0);
        }
        if (c_.stop()) {
            return;
        }
        toSafe();
        add(Move::rapid(Vec3{pts_[0], p_.safeZ}));
        add(Move::plunge(Vec3{pts_[0], zv_[0]}));
        const std::size_t n = pts_.size();
        for (std::size_t k = 1; k < n; ++k) {
            feedTo(pts_[k - 1], zs_[k]);
            feedTo(pts_[k], zs_[k]);
        }
        toSafe();
    }

    const Grid3& g_;
    const Finish3dParams& p_;
    Ctx& c_;
    DropEval& drop_;
    Toolpath& tp_;
    const std::vector<std::uint8_t>& act_;
    double linkMax_;
    double minLen_{0.0};
    std::optional<Vec3> pos_;
    std::vector<Vec2> chain_;
    Vec2 lastEnd_;
    bool haveEnd_{false};
    std::vector<Vec2> pts_;
    std::vector<double> zv_;
    std::vector<double> zs_;
};

/// Surface distance from the region edge: Dijkstra over 16 neighbours with 3D step lengths on the tip
/// surface `z`. Returns the distances (-1 outside the region) and their maximum.
double surfaceDistance(Ctx& c, const Grid3& g, const std::vector<std::uint8_t>& mask, const std::vector<float>& z,
                       std::vector<double>& dist) {
    static const int kN[16][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1}, {1, 1},  {1, -1}, {-1, 1}, {-1, -1},
                                  {2, 1},  {2, -1}, {-2, 1}, {-2, -1}, {1, 2}, {1, -2}, {-1, 2}, {-1, -2}};
    const long w = static_cast<long>(g.w);
    const long h = static_cast<long>(g.h);
    dist.assign(g.w * g.h, kInf);
    using Item = std::pair<double, std::size_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    auto in = [&](long i, long j) {
        return i >= 0 && j >= 0 && i < w && j < h && mask[static_cast<std::size_t>(j * w + i)] != 0;
    };
    for (long j = 0; j < h; ++j) {
        for (long i = 0; i < w; ++i) {
            if (in(i, j) && !(in(i - 1, j) && in(i + 1, j) && in(i, j - 1) && in(i, j + 1))) {
                dist[static_cast<std::size_t>(j * w + i)] = 0.0;
                pq.push({0.0, static_cast<std::size_t>(j * w + i)});
            }
        }
    }
    double dmax = 0.0;
    std::size_t popped = 0;
    while (!pq.empty()) {
        const auto [d, k] = pq.top();
        pq.pop();
        if (d > dist[k]) {
            continue;
        }
        if ((++popped & 4095u) == 0 && c.stop()) {
            return 0.0;
        }
        c.work += 16;
        dmax = std::max(dmax, d);
        const long i = static_cast<long>(k) % w;
        const long j = static_cast<long>(k) / w;
        for (const auto& o : kN) {
            const long ni = i + o[0];
            const long nj = j + o[1];
            if (!in(ni, nj)) {
                continue;
            }
            const std::size_t nk = static_cast<std::size_t>(nj * w + ni);
            const double dz = static_cast<double>(z[nk]) - static_cast<double>(z[k]);
            const double step = std::sqrt((static_cast<double>(o[0] * o[0] + o[1] * o[1])) * g.cs * g.cs + dz * dz);
            if (d + step < dist[nk]) {
                dist[nk] = d + step;
                pq.push({dist[nk], nk});
            }
        }
    }
    for (double& v : dist) {
        if (v == kInf) {
            v = -1.0;
        }
    }
    return dmax;
}

/// Closed iso-lines of the distance field at `level` (marching squares on the field padded with -1).
/// Each loop is oriented with the higher field on its right.
std::vector<std::vector<Vec2>> isoLoops(const Grid3& g, const std::vector<double>& dist, double level) {
    const long w = static_cast<long>(g.w);
    const long h = static_cast<long>(g.h);
    auto f = [&](long i, long j) {
        return (i < 0 || j < 0 || i >= w || j >= h) ? -1.0 : dist[static_cast<std::size_t>(j * w + i)];
    };
    auto nodeXY = [&](long i, long j) {
        return Vec2{g.origin.x + static_cast<double>(i) * g.cs, g.origin.y + static_cast<double>(j) * g.cs};
    };
    struct Seg {
        std::uint64_t e0, e1;
    };
    std::vector<Seg> segs;
    std::map<std::uint64_t, Vec2> pos;
    std::map<std::uint64_t, std::vector<std::size_t>> at;
    const std::uint64_t stride = static_cast<std::uint64_t>(w + 2);
    auto hId = [&](long i, long j) { return 2 * (static_cast<std::uint64_t>(j + 1) * stride + static_cast<std::uint64_t>(i + 1)); };
    auto vId = [&](long i, long j) { return hId(i, j) + 1; };
    auto crossing = [&](std::uint64_t id, long ai, long aj, long bi, long bj) {
        if (pos.find(id) == pos.end()) {
            const double fa = f(ai, aj);
            const double fb = f(bi, bj);
            const double t = std::clamp((level - fa) / (fb - fa), 0.0, 1.0);
            const Vec2 pa = nodeXY(ai, aj);
            const Vec2 pb = nodeXY(bi, bj);
            pos[id] = Vec2{pa.x + (pb.x - pa.x) * t, pa.y + (pb.y - pa.y) * t};
        }
        return id;
    };
    for (long cj = -1; cj < h; ++cj) {
        for (long ci = -1; ci < w; ++ci) {
            const double v0 = f(ci, cj);
            const double v1 = f(ci + 1, cj);
            const double v2 = f(ci + 1, cj + 1);
            const double v3 = f(ci, cj + 1);
            const int idx = (v0 >= level ? 1 : 0) | (v1 >= level ? 2 : 0) | (v2 >= level ? 4 : 0) | (v3 >= level ? 8 : 0);
            if (idx == 0 || idx == 15) {
                continue;
            }
            auto e = [&](int k) {
                switch (k) {
                    case 0: return crossing(hId(ci, cj), ci, cj, ci + 1, cj);
                    case 1: return crossing(vId(ci + 1, cj), ci + 1, cj, ci + 1, cj + 1);
                    case 2: return crossing(hId(ci, cj + 1), ci, cj + 1, ci + 1, cj + 1);
                    default: return crossing(vId(ci, cj), ci, cj, ci, cj + 1);
                }
            };
            auto put = [&](int a, int b) {
                const Seg s{e(a), e(b)};
                at[s.e0].push_back(segs.size());
                at[s.e1].push_back(segs.size());
                segs.push_back(s);
            };
            const bool centreHigh = 0.25 * (v0 + v1 + v2 + v3) >= level;
            switch (idx) {
                case 1: case 14: put(3, 0); break;
                case 2: case 13: put(0, 1); break;
                case 3: case 12: put(3, 1); break;
                case 4: case 11: put(1, 2); break;
                case 6: case 9: put(0, 2); break;
                case 7: case 8: put(3, 2); break;
                case 5:
                    if (centreHigh) { put(0, 1); put(3, 2); } else { put(3, 0); put(1, 2); }
                    break;
                default:  // 10
                    if (centreHigh) { put(3, 0); put(1, 2); } else { put(0, 1); put(3, 2); }
                    break;
            }
        }
    }
    auto fAt = [&](Vec2 p) {
        const double fx = (p.x - g.origin.x) / g.cs;
        const double fy = (p.y - g.origin.y) / g.cs;
        const long i = static_cast<long>(std::floor(fx));
        const long j = static_cast<long>(std::floor(fy));
        const double tx = fx - static_cast<double>(i);
        const double ty = fy - static_cast<double>(j);
        return (f(i, j) * (1 - tx) + f(i + 1, j) * tx) * (1 - ty) + (f(i, j + 1) * (1 - tx) + f(i + 1, j + 1) * tx) * ty;
    };
    std::vector<std::vector<Vec2>> loops;
    std::vector<char> used(segs.size(), 0);
    for (std::size_t s0 = 0; s0 < segs.size(); ++s0) {
        if (used[s0] != 0) {
            continue;
        }
        std::vector<Vec2> loop;
        std::size_t s = s0;
        std::uint64_t from = segs[s0].e0;
        std::uint64_t to = segs[s0].e1;
        loop.push_back(pos[from]);
        for (;;) {
            used[s] = 1;
            loop.push_back(pos[to]);
            std::size_t next = segs.size();
            for (std::size_t cand : at[to]) {
                if (used[cand] == 0) {
                    next = cand;
                }
            }
            if (next == segs.size()) {
                break;
            }
            from = to;
            to = segs[next].e0 == from ? segs[next].e1 : segs[next].e0;
            s = next;
        }
        // Remove repeated points; drop the closing duplicate (the loop is closed by construction).
        std::vector<Vec2> clean;
        for (const Vec2& q : loop) {
            if (clean.empty() || q.x != clean.back().x || q.y != clean.back().y) {
                clean.push_back(q);
            }
        }
        if (clean.size() > 1 && clean.front().x == clean.back().x && clean.front().y == clean.back().y) {
            clean.pop_back();
        }
        if (clean.size() < 2) {
            continue;
        }
        // Orientation: the higher field must be on the right of the travel direction.
        const Vec2 a = clean[0];
        const Vec2 b = clean[1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len = std::hypot(dx, dy);
        const Vec2 m{0.5 * (a.x + b.x), 0.5 * (a.y + b.y)};
        const double eps = 0.3 * g.cs;
        const Vec2 right{m.x + dy / len * eps, m.y - dx / len * eps};
        const Vec2 left{m.x - dy / len * eps, m.y + dx / len * eps};
        if (fAt(right) < fAt(left)) {
            std::reverse(clean.begin(), clean.end());
        }
        loops.push_back(std::move(clean));
    }
    return loops;
}

/// Simulated stock left by the given toolpaths on the model grid (cell centres at the nodes).
bool simulateLeft(Ctx& c, const Grid3& g, const Finish3dParams& p,
                  const std::vector<std::pair<const Toolpath*, const simulate::ToolShape*>>& paths,
                  std::vector<float>& left, std::string& err) {
    simulate::Stock st;
    st.width = (static_cast<double>(g.w) - 0.25) * g.cs;
    st.height = (static_cast<double>(g.h) - 0.25) * g.cs;
    st.thickness = p.stockTop - p.stockBottom;
    st.cellSize = g.cs;
    st.origin = geometry::Vec2{g.origin.x - 0.5 * g.cs, g.origin.y - 0.5 * g.cs};
    st.topZ = p.stockTop;
    st.trackPassIds = false;
    const simulate::StockCheck chk = simulate::checkStock(st);
    if (!chk.ok() || chk.nx != g.w || chk.ny != g.h) {
        err = "rest machining cannot simulate the stock: " + chk.message;
        return false;
    }
    simulate::SimOptions so;
    so.maxCellWork = static_cast<double>(c.workLimit) * 4.0;
    simulate::Simulator sim(st, so);
    if (!sim.ok()) {
        err = "rest machining simulation: " + sim.error().message;
        return false;
    }
    std::uint32_t id = 1;
    for (const auto& pr : paths) {
        if (pr.first->moves.empty()) {
            continue;
        }
        if (!sim.addToolpath(*pr.first, *pr.second, id++)) {
            err = "rest machining simulation: " + sim.error().message;
            return false;
        }
    }
    for (std::size_t mv = 0; mv < sim.moveCount();) {
        if (c.stop()) {
            return false;
        }
        mv = std::min(sim.moveCount(), mv + 2048);
        if (!sim.advanceToMove(mv)) {
            err = "rest machining simulation: " + sim.error().message;
            return false;
        }
    }
    const simulate::Heightfield& hf = sim.heightfield();
    left.assign(g.w * g.h, 0.0f);
    for (std::size_t j = 0; j < g.h; ++j) {
        for (std::size_t i = 0; i < g.w; ++i) {
            left[j * g.w + i] = hf.height(i, j);
        }
    }
    return true;
}

void addDiag(Finish3dResult& r, ToolpathIssue issue, std::string msg, std::optional<std::size_t> tool = {}) {
    ToolpathDiagnostic d;
    d.issue = issue;
    d.message = std::move(msg);
    d.tool = tool;
    r.diagnostics.push_back(std::move(d));
}

}  // namespace

Finish3dResult finish3dToolpath(const relief::HeightField& model, const std::vector<Rough3dTool>& tools,
                                const Finish3dParams& p, const ToolpathRunControl& control) {
    Finish3dResult res;
    auto bad = [&](const std::string& msg) {
        addDiag(res, ToolpathIssue::InvalidParameters, msg);
        return res;
    };
    auto fin = [](double v) { return std::isfinite(v); };
    if (tools.empty()) {
        addDiag(res, ToolpathIssue::NoTool, "3D finish needs at least one tool");
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
        !fin(p.boundaryOffset) || !fin(p.rasterAngle) || !fin(p.tolerance) || !fin(p.restTolerance)) {
        return bad("a parameter is not finite");
    }
    if (!(p.stockBottom < p.stockTop) || p.allowance < 0.0 || !(p.safeZ > p.stockTop) || !(p.tolerance > 0.0) ||
        p.restTolerance < 0.0) {
        return bad("stock top/bottom, safe Z, allowance, tolerance or rest tolerance out of range");
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
    Grid3 g;
    g.model = &model;
    g.cs = model.cellSize();
    g.origin = {model.origin().x, model.origin().y};
    g.w = model.width();
    g.h = model.height();
    const std::size_t N = g.w * g.h;
    if (N >= static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
        return bad("the model is too large");
    }
    Ctx c{p, control};
    c.workLimit = control.workLimit != 0 ? control.workLimit : kRough3dDefaultWorkLimit;
    c.moveLimit = control.moveLimit != 0 ? control.moveLimit : kRough3dDefaultMoveLimit;
    auto clearAll = [&]() {
        for (Toolpath& t : res.toolpaths) {
            t.moves.clear();
        }
    };
    auto finishOut = [&](const std::string& why) {
        clearAll();
        if (c.abort) {
            addDiag(res, ToolpathIssue::Cancelled, "3D finish cancelled");
        } else {
            addDiag(res, ToolpathIssue::WorkLimitExceeded, why);
        }
        return res;
    };

    std::vector<std::uint8_t> mask;
    if (!buildMask(c, g, mask)) {
        return finishOut("the boundary needs more work than allowed");
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

    // Shapes of the priors and the tools.
    std::vector<simulate::ToolShape> priorShapes;
    for (const Finish3dPrior& pr : p.priors) {
        const auto sh = simulate::ToolShape::fromTool(pr.tool.tool, pr.tool.fluteLength);
        if (!sh) {
            return bad("a prior tool has no usable shape");
        }
        priorShapes.push_back(*sh);
    }
    std::vector<simulate::ToolShape> shapes;  // shapes[k] belongs to tools[k] once it has run

    std::vector<float> scur(N, static_cast<float>(p.stockTop));
    for (std::size_t ti = 0; ti < tools.size(); ++ti) {
        const Rough3dTool& rt = tools[ti];
        Toolpath& tp = res.toolpaths[ti];
        const auto shape = simulate::ToolShape::fromTool(rt.tool, rt.fluteLength);
        if (!shape) {
            addDiag(res, ToolpathIssue::InvalidTool, "tool '" + rt.tool.name + "' has no usable shape", ti);
            shapes.push_back(simulate::ToolShape{});
            continue;
        }
        const tooling::CuttingData& cd = effectiveCuttingData(rt.tool, rt.cuttingData);
        if (!(cd.stepover > 0.0) || !fin(cd.stepover)) {
            addDiag(res, ToolpathIssue::InvalidTool, "tool '" + rt.tool.name + "' needs a stepover > 0", ti);
            shapes.push_back(simulate::ToolShape{});
            continue;
        }
        tp.tool = toolSnapshot(rt.tool, cd);
        shapes.push_back(*shape);
        double stepover = cd.stepover;
        if (stepover > shape->diameter()) {
            stepover = shape->diameter();
            addDiag(res, ToolpathIssue::StepoverLimited, "the stepover was limited to the tool diameter", ti);
        }

        const bool rest = ti > 0 || !p.priors.empty();
        if (rest) {
            std::vector<std::pair<const Toolpath*, const simulate::ToolShape*>> before;
            for (std::size_t q = 0; q < p.priors.size(); ++q) {
                before.emplace_back(&p.priors[q].toolpath, &priorShapes[q]);
            }
            for (std::size_t q = 0; q < ti; ++q) {
                if (shapes[q].valid()) {
                    before.emplace_back(&res.toolpaths[q], &shapes[q]);
                }
            }
            std::string err;
            if (!simulateLeft(c, g, p, before, scur, err)) {
                return finishOut(err);
            }
        }

        DropEval drop(g, *shape, p, c);
        // Tip target per node (the grid drop surface), used for rest eligibility and surface distances.
        std::vector<float> T;
        {
            std::vector<float> ds;
            const DropStatus st = dropToolSurface(model, *shape, p.zOffset, ds, c.workLimit, c.work, control.cancel);
            if (st == DropStatus::Cancelled) {
                c.abort = true;
                return finishOut("");
            }
            if (st != DropStatus::Ok) {
                return finishOut("the drop-tool surface needs more work than allowed (tool too large for the grid "
                                 "resolution, or work limit)");
            }
            T.assign(N, 0.0f);
            const float bottom = static_cast<float>(roundUp(p.stockBottom));
            for (std::size_t k = 0; k < N; ++k) {
                T[k] = std::max(static_cast<float>(roundUp(static_cast<double>(ds[k]) + p.allowance)), bottom);
            }
        }

        // Nodes where this tool may cut: in the region and (rest) near material the earlier tools left.
        std::vector<std::uint8_t> act = mask;
        if (rest) {
            std::vector<std::uint8_t> elig(N, 0);
            std::size_t eligible = 0;
            for (std::size_t k = 0; k < N; ++k) {
                if (mask[k] != 0 && static_cast<double>(scur[k]) > static_cast<double>(T[k]) + p.restTolerance) {
                    elig[k] = 1;
                    ++eligible;
                }
            }
            if (eligible == 0) {
                addDiag(res, ToolpathIssue::NoRestMaterial, "the earlier tools left nothing for this tool", ti);
                continue;
            }
            c.work += N * 8;
            const std::vector<double> d2 = edtSq(elig, g.w, g.h);
            const double lim = stepover * stepover * (1.0 + 1e-9);
            for (std::size_t k = 0; k < N; ++k) {
                act[k] = (mask[k] != 0 && d2[k] * g.cs * g.cs <= lim) ? 1 : 0;
            }
        }

        PathWriter writer(g, p, c, drop, tp, act, 2.0 * stepover);
        const bool climb = p.direction == CutDirection::Climb;
        if (p.strategy == Finish3dStrategy::Raster) {
            const double ca = std::cos(p.rasterAngle);
            const double sa = std::sin(p.rasterAngle);
            // Frame: local coordinates relative to the grid origin.
            const double wx = static_cast<double>(g.w - 1) * g.cs;
            const double wy = static_cast<double>(g.h - 1) * g.cs;
            double umin = kInf, umax = -kInf, vmin = kInf, vmax = -kInf;
            for (int k = 0; k < 4; ++k) {
                const double x = (k & 1) != 0 ? wx : 0.0;
                const double y = (k & 2) != 0 ? wy : 0.0;
                const double u = x * ca + y * sa;
                const double v = -x * sa + y * ca;
                umin = std::min(umin, u);
                umax = std::max(umax, u);
                vmin = std::min(vmin, v);
                vmax = std::max(vmax, v);
            }
            const std::size_t lines = static_cast<std::size_t>(std::ceil((vmax - vmin) / stepover - 1e-9)) + 1;
            if (lines > 10'000'000) {
                return finishOut("the stepover is far too small for the model");
            }
            // Climb: lines step towards the right of the travel direction (descending v).
            const bool descending = climb != p.reverseSteps;
            constexpr double slabEps = 1e-11;
            for (std::size_t s = 0; s < lines; ++s) {
                if (c.stop()) {
                    break;
                }
                const std::size_t kk = descending ? lines - 1 - s : s;
                const double v = kk + 1 == lines ? vmax : vmin + static_cast<double>(kk) * stepover;
                const bool back = p.travel == RasterTravel::ZigZag && (s & 1u) != 0;
                // The part of the line inside the node rectangle (slab clipping), so that lines grazing a
                // corner still reach it.
                double t0 = umin;
                double t1 = umax;
                bool empty = false;
                const double ox[2] = {ca, -sa};   // d(x)/du, d(x)/dv
                const double oy[2] = {sa, ca};    // d(y)/du, d(y)/dv
                for (int axis = 0; axis < 2 && !empty; ++axis) {
                    const double lim = axis == 0 ? wx : wy;
                    const double dir = axis == 0 ? ox[0] : oy[0];
                    const double base = (axis == 0 ? ox[1] : oy[1]) * v;
                    if (std::fabs(dir) < 1e-15) {
                        empty = base < -slabEps || base > lim + slabEps;
                        continue;
                    }
                    double a = (-slabEps - base) / dir;
                    double b = (lim + slabEps - base) / dir;
                    if (a > b) {
                        std::swap(a, b);
                    }
                    t0 = std::max(t0, a);
                    t1 = std::min(t1, b);
                }
                if (empty || t0 > t1) {
                    continue;
                }
                const std::size_t samples = static_cast<std::size_t>(std::ceil((t1 - t0) / g.cs - 1e-9)) + 1;
                std::vector<Vec2> line;
                line.reserve(samples);
                for (std::size_t q = 0; q < samples; ++q) {
                    const std::size_t qq = back ? samples - 1 - q : q;
                    const double u = samples == 1 ? t0 : t0 + (t1 - t0) * static_cast<double>(qq) / static_cast<double>(samples - 1);
                    line.push_back(Vec2{g.origin.x + u * ca - v * sa, g.origin.y + u * sa + v * ca});
                }
                c.work += samples;
                writer.addPolyline(std::move(line), false);
                if (p.travel == RasterTravel::OneWay) {
                    writer.flush();
                }
            }
        } else {
            std::vector<double> dist;
            const double dmax = surfaceDistance(c, g, mask, T, dist);
            if (c.stop()) {
                return finishOut("the offset passes need more work than allowed");
            }
            std::vector<double> levels;
            for (std::size_t k = 0; static_cast<double>(k) * stepover <= dmax + 1e-12; ++k) {
                levels.push_back(static_cast<double>(k) * stepover);
                if (levels.size() > 1'000'000) {
                    return finishOut("the stepover is far too small for the model");
                }
            }
            if (dmax - levels.back() > 0.5 * stepover) {
                levels.push_back(dmax - 1e-6 * g.cs);
            }
            if (p.reverseSteps) {
                std::reverse(levels.begin(), levels.end());
            }
            for (double L : levels) {
                if (c.stop()) {
                    break;
                }
                std::vector<std::vector<Vec2>> loops = isoLoops(g, dist, L);
                c.work += N;
                for (auto& lp : loops) {
                    if (!climb) {
                        std::reverse(lp.begin(), lp.end());
                    }
                }
                std::vector<char> done(loops.size(), 0);
                for (std::size_t n = 0; n < loops.size(); ++n) {
                    if (c.stop()) {
                        break;
                    }
                    std::size_t pick = n;
                    std::size_t startAt = 0;
                    if (writer.hasLast() && loops.size() <= 2000) {
                        const Vec2 e = writer.lastPoint();
                        double best = kInf;
                        for (std::size_t q = 0; q < loops.size(); ++q) {
                            if (done[q] != 0) {
                                continue;
                            }
                            for (std::size_t k = 0; k < loops[q].size(); ++k) {
                                const double dd = std::hypot(loops[q][k].x - e.x, loops[q][k].y - e.y);
                                if (dd < best) {
                                    best = dd;
                                    pick = q;
                                    startAt = k;
                                }
                            }
                        }
                    } else {
                        while (done[pick] != 0) {
                            ++pick;
                        }
                    }
                    done[pick] = 1;
                    const std::vector<Vec2>& lp = loops[pick];
                    std::vector<Vec2> rot;
                    rot.reserve(lp.size() + 1);
                    for (std::size_t k = 0; k <= lp.size(); ++k) {
                        rot.push_back(lp[(startAt + k) % lp.size()]);
                    }
                    writer.addPolyline(writer.densify(rot), true);
                }
            }
        }
        writer.finish();
        if (c.abort) {
            return finishOut("");
        }
        if (c.over) {
            return finishOut(tp.moves.size() >= c.moveLimit ? "the toolpath would exceed the move limit"
                                                           : "the toolpath needs more work than allowed");
        }
        if (tp.moves.empty()) {
            addDiag(res, ToolpathIssue::NoRegion, "nothing to cut", ti);
        }
    }
    return res;
}

}  // namespace carvetto::toolpath
