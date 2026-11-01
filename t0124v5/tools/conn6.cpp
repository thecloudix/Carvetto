// T-0124 round-2 verifier (throwaway): independent tab-connectivity oracle.
// For every kept tab (flat run at the tab top on the final level of a primary cut), rasterise the
// layer just below the tab top inside the tab's window (cells whose nearest point on the primary
// final-level path lies on the tab's flat run), compute the exact clearance of every cell to every
// cutting move below that layer (all passes, all cuts, leads, plunges), and check that uncut cells
// join the vector side (beyond the vector) to the waste side (beyond the widest kerf). Reports the
// neck (bottleneck clearance). Also: counts (kept + left out = requested), determinism.
// usage: conn seed N [fam] [sep(-1)] [target(-1)]   env: CELL (0.02), TLMIN/TLMAX, V
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "scene_io.hpp"

static Contour bezBlob(Vec2 c, double R, int n) {
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) {
        double a = 2 * PI * i / n + U(-0.2, 0.2) / n;
        double rr = R * U(0.55, 1.0);
        p.push_back({c.x + rr * std::cos(a), c.y + rr * std::sin(a)});
    }
    Contour o;
    o.closed = true;
    for (int i = 0; i < n; ++i) {
        Vec2 p0 = p[(size_t)((i - 1 + n) % n)], p1 = p[(size_t)i], p2 = p[(size_t)((i + 1) % n)], p3 = p[(size_t)((i + 2) % n)];
        o.append(geometry::BezierSpan{p1, p1 + (p2 - p0) * (1.0 / 6), p2 - (p3 - p1) * (1.0 / 6), p2});
    }
    return o;
}
static Contour bezWave(Vec2 c, double L, int n) {
    Contour o;
    Vec2 a = c;
    for (int i = 0; i < n; ++i) {
        Vec2 b = a + Vec2{L / n, U(-L / n, L / n)};
        double h = U(-0.6, 0.6) * L / n;
        o.append(geometry::BezierSpan{a, a + Vec2{L / (3 * n), h}, b - Vec2{L / (3 * n), -h}, b});
        a = b;
    }
    return o;
}
static Contour ellipse(Vec2 c, double a, double b, double rot) {  // 4 Bezier quarter ellipse
    const double k = 0.5522847498;
    Vec2 ux{std::cos(rot), std::sin(rot)}, uy{-ux.y, ux.x};
    auto P = [&](double x, double y) { return c + ux * x + uy * y; };
    Contour o;
    o.closed = true;
    o.append(geometry::BezierSpan{P(a, 0), P(a, k * b), P(k * a, b), P(0, b)});
    o.append(geometry::BezierSpan{P(0, b), P(-k * a, b), P(-a, k * b), P(-a, 0)});
    o.append(geometry::BezierSpan{P(-a, 0), P(-a, -k * b), P(-k * a, -b), P(0, -b)});
    o.append(geometry::BezierSpan{P(0, -b), P(k * a, -b), P(a, -k * b), P(a, 0)});
    return o;
}

static tooling::Tool endMill(double d) {
    tooling::Tool t;
    t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::EndMill; t.diameter = d;
    t.defaultCuttingData.passDepth = 4.0; t.defaultCuttingData.feedRate = 1000.0; t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 18000.0; t.defaultCuttingData.toolNumber = 1;
    return t;
}
static tooling::Tool vbit(double d, double deg) { auto t = endMill(d); t.type = tooling::ToolType::VBit; t.includedAngle = deg * PI / 180; return t; }

// ---------------- geometry helpers (own)
static double segDist(Vec2 p, Vec2 a, Vec2 b, double* tOut = nullptr) {
    double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0;
    t = std::clamp(t, 0.0, 1.0);
    if (tOut) *tOut = t;
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
static double sweepOf(const geometry::ArcSpan& a) {
    double raw = a.direction == geometry::ArcDirection::CCW ? a.endAngle - a.startAngle : a.startAngle - a.endAngle;
    raw = std::fmod(raw, 2 * PI);
    if (raw < 0) raw += 2 * PI;
    if (raw <= 1e-15) raw = 2 * PI;
    return a.direction == geometry::ArcDirection::CCW ? raw : -raw;
}
static int arcSteps(double R, double sw) {
    double tol = 1e-4;
    double da = R > tol ? 2 * std::acos(1 - tol / R) : PI / 4;
    return std::max(2, (int)std::ceil(std::fabs(sw) / std::min(da, PI / 36)));
}
static std::vector<Vec2> polyOf(const Contour& c) {
    std::vector<Vec2> out;
    for (auto& sp : c.spans) {
        if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { if (out.empty()) out.push_back(l->start); out.push_back(l->end); }
        else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) {
            double sw = sweepOf(*a);
            int n = arcSteps(a->radius, sw);
            for (int k = out.empty() ? 0 : 1; k <= n; ++k) { double t = a->startAngle + sw * k / n; out.push_back(a->centre + Vec2{std::cos(t), std::sin(t)} * a->radius); }
        } else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) {
            int n = 200;
            for (int k = out.empty() ? 0 : 1; k <= n; ++k) { double t = (double)k / n, u = 1 - t; out.push_back(b->p0 * (u * u * u) + b->p1 * (3 * u * u * t) + b->p2 * (3 * u * t * t) + b->p3 * (t * t * t)); }
        }
    }
    return out;
}

struct SegGrid {
    double cs = 1.0;
    std::unordered_map<long long, std::vector<int>> m;
    static long long key(int x, int y) { return ((long long)x << 32) ^ (unsigned)y; }
    void add(int id, double x0, double y0, double x1, double y1) {
        int ax = (int)std::floor(x0 / cs), ay = (int)std::floor(y0 / cs), bx = (int)std::floor(x1 / cs), by = (int)std::floor(y1 / cs);
        for (int x = ax; x <= bx; ++x) for (int y = ay; y <= by; ++y) m[key(x, y)].push_back(id);
    }
    const std::vector<int>* at(double x, double y) const {
        auto it = m.find(key((int)std::floor(x / cs), (int)std::floor(y / cs)));
        return it == m.end() ? nullptr : &it->second;
    }
};

// vectors: polyline segments, distance query
struct Vecs {
    std::vector<std::pair<Vec2, Vec2>> segs;
    SegGrid g;
    double reach;
    Vecs(const std::vector<Contour>& cs, double reach_) : reach(reach_) {
        for (auto& c : cs) { auto p = polyOf(c); for (size_t i = 1; i < p.size(); ++i) segs.push_back({p[i - 1], p[i]}); }
        for (size_t i = 0; i < segs.size(); ++i) {
            auto [a, b] = segs[i];
            g.add((int)i, std::min(a.x, b.x) - reach, std::min(a.y, b.y) - reach, std::max(a.x, b.x) + reach, std::max(a.y, b.y) + reach);
        }
    }
    double dist(Vec2 p) const {  // exact within reach, else reach
        double d = reach;
        if (auto* v = g.at(p.x, p.y)) for (int i : *v) d = std::min(d, segDist(p, segs[(size_t)i].first, segs[(size_t)i].second));
        return d;
    }
};

struct Region {
    std::vector<std::vector<Vec2>> polys;
    explicit Region(const std::vector<Contour>& cs) { for (auto& c : cs) if (c.closed) polys.push_back(polyOf(c)); }
    std::vector<double> rowXs(double y) const {
        std::vector<double> xs;
        for (auto& P : polys) for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++)
            if ((P[i].y > y) != (P[j].y > y)) xs.push_back((P[j].x - P[i].x) * (y - P[i].y) / (P[j].y - P[i].y) + P[i].x);
        std::sort(xs.begin(), xs.end());
        return xs;
    }
    bool inside(Vec2 q) const {
        bool in = false;
        for (auto& P : polys) for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++) {
            if (((P[i].y > q.y) != (P[j].y > q.y)) && (q.x < (P[j].x - P[i].x) * (q.y - P[i].y) / (P[j].y - P[i].y) + P[i].x)) in = !in;
        }
        return in;
    }
};
struct P3 { double x, y, z; size_t mi; };
static std::vector<P3> expand(const Toolpath& tp) {
    std::vector<P3> o;
    for (size_t i = 0; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        if (i == 0) { o.push_back({m.to.x, m.to.y, m.to.z, i}); continue; }
        P3 A = o.back();
        if (isArc(m.kind)) {
            double R = std::hypot(A.x - m.centre.x, A.y - m.centre.y);
            double a0 = std::atan2(A.y - m.centre.y, A.x - m.centre.x), a1 = std::atan2(m.to.y - m.centre.y, m.to.x - m.centre.x);
            double sw = m.kind == MoveKind::ArcCCW ? a1 - a0 : a0 - a1;
            sw = std::fmod(sw, 2 * PI); if (sw < 0) sw += 2 * PI;
            if (sw < 1e-12) sw = 2 * PI;
            if (m.kind == MoveKind::ArcCW) sw = -sw;
            int n = arcSteps(R, sw);
            for (int k = 1; k <= n; ++k) {
                double t = (double)k / n, a = a0 + sw * t;
                if (k == n) o.push_back({m.to.x, m.to.y, m.to.z, i});
                else o.push_back({m.centre.x + R * std::cos(a), m.centre.y + R * std::sin(a), A.z + (m.to.z - A.z) * t, i});
            }
        } else if (m.kind != MoveKind::Dwell) o.push_back({m.to.x, m.to.y, m.to.z, i});
    }
    return o;
}

// cutting segments below a height h, with tool radius at h
struct CutSeg { Vec2 a, b; double rad; size_t mi = 0; };
struct Cuts {
    std::vector<CutSeg> s;
    SegGrid g;
    double maxRad = 0;
    void build(const Toolpath& tp, const std::vector<P3>& pts, double h, double R, double tanHalf, bool vb, double margin) {
        std::unordered_set<long long> seen;
        for (size_t i = 1; i < pts.size(); ++i) {
            if (tp.moves[pts[i].mi].kind == MoveKind::Rapid) continue;
            P3 A = pts[i - 1], B = pts[i];
            if (A.z >= h && B.z >= h) continue;
            if (A.z >= h || B.z >= h) {  // clip to z < h
                double t = (h - A.z) / (B.z - A.z);
                P3 C{A.x + (B.x - A.x) * t, A.y + (B.y - A.y) * t, h, 0};
                if (A.z >= h) A = C; else B = C;
            }
            if (!vb) {
                auto qk = [](double v) { return (long long)std::llround(v * 1e7); };
                long long k1 = qk(A.x) * 1000003LL ^ qk(A.y) * 10007LL ^ qk(B.x) * 101LL ^ qk(B.y);
                if (!seen.insert(k1).second) continue;
                s.push_back({{A.x, A.y}, {B.x, B.y}, R, pts[i].mi}); continue;
            }
            double L = std::hypot(B.x - A.x, B.y - A.y);
            int n = std::max(1, (int)std::ceil(L / 0.02));
            for (int k = 0; k < n; ++k) {
                double t0 = (double)k / n, t1 = (double)(k + 1) / n;
                double z = std::min(A.z + (B.z - A.z) * t0, A.z + (B.z - A.z) * t1);
                double rr = std::min(R, (h - z) * tanHalf);
                if (rr <= 0) continue;
                s.push_back({{A.x + (B.x - A.x) * t0, A.y + (B.y - A.y) * t0}, {A.x + (B.x - A.x) * t1, A.y + (B.y - A.y) * t1}, rr});
            }
        }
        for (size_t i = 0; i < s.size(); ++i) {
            double e = s[i].rad + margin;
            maxRad = std::max(maxRad, s[i].rad);
            g.add((int)i, std::min(s[i].a.x, s[i].b.x) - e, std::min(s[i].a.y, s[i].b.y) - e, std::max(s[i].a.x, s[i].b.x) + e, std::max(s[i].a.y, s[i].b.y) + e);
        }
    }
    double clearanceW(Vec2 p, double cap, size_t& who) const {
        double c = cap; who = SIZE_MAX;
        if (auto* v = g.at(p.x, p.y)) for (int i : *v) { const auto& q = s[(size_t)i]; double d = segDist(p, q.a, q.b) - q.rad; if (d < c) { c = d; who = q.mi; } }
        return c;
    }
    double clearanceExcl(Vec2 p, double cap, size_t excl) const {
        double c = cap;
        if (auto* v = g.at(p.x, p.y)) for (int i : *v) { const auto& q = s[(size_t)i]; if (q.mi == excl) continue; c = std::min(c, segDist(p, q.a, q.b) - q.rad); }
        return c;
    }
    double clearance(Vec2 p, double cap) const {
        double c = cap;
        if (auto* v = g.at(p.x, p.y)) for (int i : *v) { const auto& q = s[(size_t)i]; c = std::min(c, segDist(p, q.a, q.b) - q.rad); }
        return c;
    }
};

// polyline with arc length (final level of a primary cut)
struct PL {
    std::vector<Vec2> p; std::vector<double> s, z;
    SegGrid g;
    void index(double reach) {
        for (size_t i = 1; i < p.size(); ++i)
            g.add((int)i, std::min(p[i - 1].x, p[i].x) - reach, std::min(p[i - 1].y, p[i].y) - reach, std::max(p[i - 1].x, p[i].x) + reach, std::max(p[i - 1].y, p[i].y) + reach);
    }
    // nearest point: distance, arc length, side (cross sign, + = left of travel)
    bool nearest(Vec2 q, double reach, double& d, double& sOut, int& side) const {
        d = reach; bool found = false;
        auto* v = g.at(q.x, q.y);
        if (!v) return false;
        for (int i : *v) {
            double t; double dd = segDist(q, p[(size_t)i - 1], p[(size_t)i], &t);
            if (dd < d) {
                d = dd; found = true; sOut = s[(size_t)i - 1] + t * (s[(size_t)i] - s[(size_t)i - 1]);
                Vec2 a = p[(size_t)i - 1], b = p[(size_t)i];
                Vec2 foot = a + (b - a) * t;
                double cr = (b.x - a.x) * (q.y - foot.y) - (b.y - a.y) * (q.x - foot.x);
                if (std::fabs(dd) < 1e-12) side = 0;
                else if (t > 0 && t < 1) side = cr > 0 ? 1 : -1;
                else {  // at a vertex: use bisector of adjacent segments
                    size_t k = t <= 0 ? (size_t)i - 1 : (size_t)i;
                    Vec2 tin = k > 0 ? p[k] - p[k - 1] : p[k + 1] - p[k];
                    Vec2 tout = k + 1 < p.size() ? p[k + 1] - p[k] : tin;
                    double l1 = std::hypot(tin.x, tin.y), l2 = std::hypot(tout.x, tout.y);
                    (void)l1; (void)l2;
                    Vec2 w = q - p[k];
                    double c1 = tin.x * w.y - tin.y * w.x, c2 = tout.x * w.y - tout.y * w.x, turn = tin.x * tout.y - tin.y * tout.x;
                    if ((c1 > 0) == (c2 > 0)) side = c1 > 0 ? 1 : -1;
                    else side = turn > 0 ? -1 : 1;
                }
            }
        }
        return found;
    }
};

struct TabRes { bool checked = false, joined = false, noSeed = false, noTarget = false; double neck = 0; Vec2 mid; double len; };

static TabRes checkTab(const PL& F, const std::vector<std::pair<double, double>>& runs, int vecSide, double dVec, double dWaste, const Cuts& cuts, double cell, double r) {
    TabRes res;
    // bbox of runs
    double x0 = 1e18, y0 = 1e18, x1 = -1e18, y1 = -1e18;
    double reach = std::max(dVec, dWaste) + 0.7;
    for (auto [a, b] : runs)
        for (size_t i = 0; i < F.p.size(); ++i)
            if (F.s[i] >= a - 1e-9 && F.s[i] <= b + 1e-9) { x0 = std::min(x0, F.p[i].x); y0 = std::min(y0, F.p[i].y); x1 = std::max(x1, F.p[i].x); y1 = std::max(y1, F.p[i].y); }
    // include interpolated ends
    for (auto [a, b] : runs) for (double sv : {a, b, (a + b) / 2}) {
        auto it = std::lower_bound(F.s.begin(), F.s.end(), sv); size_t k = (size_t)(it - F.s.begin()); if (k == 0) k = 1; if (k >= F.s.size()) k = F.s.size() - 1;
        double t = (F.s[k] - F.s[k - 1]) > 0 ? (sv - F.s[k - 1]) / (F.s[k] - F.s[k - 1]) : 0; Vec2 q = F.p[k - 1] + (F.p[k] - F.p[k - 1]) * t;
        x0 = std::min(x0, q.x); y0 = std::min(y0, q.y); x1 = std::max(x1, q.x); y1 = std::max(y1, q.y);
        if (sv == (a + b) / 2) res.mid = q;
    }
    res.len = 0; for (auto [a, b] : runs) res.len = std::max(res.len, b - a);
    x0 -= reach; y0 -= reach; x1 += reach; y1 += reach;
    int nx = (int)std::ceil((x1 - x0) / cell), ny = (int)std::ceil((y1 - y0) / cell);
    PL Rn;
    for (auto [a, b] : runs) {
        auto at = [&](double sv) { auto it = std::lower_bound(F.s.begin(), F.s.end(), sv); size_t k = (size_t)(it - F.s.begin()); if (k == 0) k = 1; if (k >= F.s.size()) k = F.s.size() - 1; double t = (F.s[k] - F.s[k - 1]) > 0 ? (sv - F.s[k - 1]) / (F.s[k] - F.s[k - 1]) : 0; return F.p[k - 1] + (F.p[k] - F.p[k - 1]) * t; };
        double s0 = Rn.s.empty() ? 0 : Rn.s.back() + 1e6;  // separate pieces far apart in s
        Rn.p.push_back(at(a)); Rn.s.push_back(s0); Rn.z.push_back(0);
        for (size_t i = 0; i < F.p.size(); ++i) if (F.s[i] > a && F.s[i] < b && geometry::distance(F.p[i], Rn.p.back()) > 1e-9) { Rn.s.push_back(s0 + F.s[i] - a); Rn.p.push_back(F.p[i]); Rn.z.push_back(0); }
        if (geometry::distance(at(b), Rn.p.back()) > 1e-9) Rn.p.push_back(at(b)); else { Rn.p.pop_back(); Rn.s.pop_back(); Rn.z.pop_back(); Rn.p.push_back(at(b)); } Rn.s.push_back(s0 + b - a); Rn.z.push_back(0);
    }
    Rn.index(reach);
    std::vector<float> clr((size_t)nx * ny, -1.f);   // -1: outside window
    std::vector<char> role((size_t)nx * ny, 0);       // 1 seed, 2 target
    std::vector<char> why((size_t)nx * ny, ' ');
    bool anySeed = false, anyTarget = false;
    for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
        Vec2 q{x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell};
        double d, sv; int side = 0;
        if (!F.nearest(q, reach, d, sv, side)) { why[(size_t)j * nx + i] = 'n'; continue; }
        double dr, svr; int sider = 0;
        if (!Rn.nearest(q, reach, dr, svr, sider)) { why[(size_t)j * nx + i] = 'r'; continue; }
        if (dr > d + 3e-4) { why[(size_t)j * nx + i] = 'x'; continue; }
        side = sider; d = dr;
        bool vs = vecSide == 0 ? side > 0 : side == vecSide;
        double lim = vs ? dVec + 0.6 : dWaste + 0.6;
        if (d > lim) { why[(size_t)j * nx + i] = vs ? 'v' : 'w'; continue; }
        double c = cuts.clearance(q, 0.6);
        if (c <= 0) { clr[(size_t)j * nx + i] = 0.f; continue; }
        clr[(size_t)j * nx + i] = (float)c;
        if (vs && d >= dVec + 0.1) { role[(size_t)j * nx + i] = 1; anySeed = true; }
        if (!vs && d >= dWaste + 0.05) { role[(size_t)j * nx + i] = 2; anyTarget = true; }
    }
    res.noSeed = !anySeed; res.noTarget = !anyTarget;
    if (std::getenv("PROFILE")) {
        // normal line at the middle of the first run
        double sm = (runs[0].first + runs[0].second) / 2;
        auto it = std::lower_bound(F.s.begin(), F.s.end(), sm); size_t k = (size_t)(it - F.s.begin()); if (k == 0) k = 1;
        Vec2 a = F.p[k - 1], b = F.p[k]; double l = geometry::distance(a, b); Vec2 t = (b - a) * (1 / l), nL{-t.y, t.x};
        Vec2 m = a + (b - a) * ((sm - F.s[k - 1]) / l);
        if (vecSide < 0) nL = nL * -1.0;
        for (double d = -(dWaste + 0.5); d <= dVec + 0.5; d += 0.1) { size_t who; double c = cuts.clearanceW(m + nL * d, 5, who); std::printf("     d %+.2f (%.3f,%.3f) clr %.4f mv %zu\n", d, (m + nL * d).x, (m + nL * d).y, c, who); }
        { std::map<size_t, int> hist; for (int j = 0; j < ny; j += 3) for (int i = 0; i < nx; i += 3) { size_t q = (size_t)j * nx + i; if (clr[q] == 0.f) { size_t who; cuts.clearanceW({x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell}, 5, who); hist[who]++; } }
          for (auto [w, n] : hist) std::printf("     cutter mv %zu: %d cells\n", w, n); }
        // ascii map
        int step = std::getenv("STEP") ? std::atoi(std::getenv("STEP")) : std::max(1, (int)(0.1 / cell));
        for (int j = ny - 1; j >= 0; j -= step) { std::string row; for (int i = 0; i < nx; i += step) { size_t q = (size_t)j * nx + i; row += clr[q] < 0 ? why[q] : (clr[q] <= 0 ? '#' : (role[q] == 1 ? 'P' : role[q] == 2 ? 'W' : '.')); } std::printf("     |%s|\n", row.c_str()); }
    }
    if (!anySeed || !anyTarget) return res;
    res.checked = true;
    auto connected = [&](double T) {
        std::vector<char> vis((size_t)nx * ny, 0);
        std::vector<int> q;
        for (int k = 0; k < nx * ny; ++k) if (role[(size_t)k] == 1 && clr[(size_t)k] > T) { vis[(size_t)k] = 1; q.push_back(k); }
        while (!q.empty()) {
            int k = q.back(); q.pop_back();
            if (role[(size_t)k] == 2) return true;
            int i = k % nx, j = k / nx;
            for (int dj = -1; dj <= 1; ++dj) for (int di = -1; di <= 1; ++di) {
                int ii = i + di, jj = j + dj;
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
                int kk = jj * nx + ii;
                if (vis[(size_t)kk] || !(clr[(size_t)kk] > T)) continue;
                vis[(size_t)kk] = 1; q.push_back(kk);
            }
        }
        return false;
    };
    // bisection on the bottleneck clearance
    if (!connected(0.0)) { res.joined = false; res.neck = 0; return res; }
    res.joined = true;
    double lo = 0, hi = 0.6;
    for (int it = 0; it < 12; ++it) { double m = (lo + hi) / 2; if (connected(m)) lo = m; else hi = m; }
    res.neck = 2 * lo;
    (void)r;
    return res;
}

// Region oracle (closed vectors, Outside/Inside): uncut cells in a box around the tab; seeds = part cells
// (by nesting parity) next to the tab, targets = waste cells beyond the widest kerf; all other tabs of
// this primary cut are virtually cut at full depth, so only this tab can bridge.
static TabRes checkTabRegion(const PL& F, std::pair<double, double> run, size_t runIdx, bool partInside, const Region& reg, const Vecs& vecs,
                             double dVec, double dWaste, const Cuts& cuts, const Cuts& virt, double cell) {
    TabRes res;
    double x0 = 1e18, y0 = 1e18, x1 = -1e18, y1 = -1e18;
    auto at = [&](double sv) { auto it = std::lower_bound(F.s.begin(), F.s.end(), sv); size_t k = (size_t)(it - F.s.begin()); if (k == 0) k = 1; if (k >= F.s.size()) k = F.s.size() - 1; double t = (F.s[k] - F.s[k - 1]) > 0 ? (sv - F.s[k - 1]) / (F.s[k] - F.s[k - 1]) : 0; return F.p[k - 1] + (F.p[k] - F.p[k - 1]) * t; };
    for (double sv = run.first; sv <= run.second + 1e-12; sv += 0.05) { Vec2 q = at(std::min(sv, run.second)); x0 = std::min(x0, q.x); y0 = std::min(y0, q.y); x1 = std::max(x1, q.x); y1 = std::max(y1, q.y); }
    res.mid = at((run.first + run.second) / 2); res.len = run.second - run.first;
    double m = dVec + dWaste + 1.5;
    x0 -= m; y0 -= m; x1 += m; y1 += m;
    int nx = (int)std::ceil((x1 - x0) / cell), ny = (int)std::ceil((y1 - y0) / cell);
    std::vector<float> clr((size_t)nx * ny, 0.f);
    std::vector<char> role((size_t)nx * ny, 0);
    bool anySeed = false, anyTarget = false;
    for (int j = 0; j < ny; ++j) {
      std::vector<double> xs = reg.rowXs(y0 + (j + 0.5) * cell);
      size_t ptr = 0;
      for (int i = 0; i < nx; ++i) {
        Vec2 q{x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell};
        while (ptr < xs.size() && xs[ptr] < q.x) ++ptr;
        double c = std::min(cuts.clearance(q, 0.6), virt.clearanceExcl(q, 0.6, runIdx));
        size_t k = (size_t)j * nx + i;
        if (c <= 0) continue;
        clr[k] = (float)c;
        bool part = (ptr % 2 == 1) == partInside;
        // seeds: part cells near the tab (the box), targets: waste beyond the kerf
        if (part) { if (vecs.dist(q) >= 0.05) { role[k] = 1; anySeed = true; } }
        else { double d; double sv; int sd; if (!F.nearest(q, dWaste + 0.05, d, sv, sd) && vecs.dist(q) >= dVec) { role[k] = 2; anyTarget = true; } }
      }
    }
    res.noSeed = !anySeed; res.noTarget = !anyTarget;
    if (!anySeed || !anyTarget) return res;
    res.checked = true;
    auto connected = [&](double T) {
        std::vector<char> vis((size_t)nx * ny, 0);
        std::vector<int> q;
        for (int k = 0; k < nx * ny; ++k) if (role[(size_t)k] == 1 && clr[(size_t)k] > T) { vis[(size_t)k] = 1; q.push_back(k); }
        while (!q.empty()) {
            int k = q.back(); q.pop_back();
            if (role[(size_t)k] == 2) return true;
            int i = k % nx, j = k / nx;
            for (int dj = -1; dj <= 1; ++dj) for (int di = -1; di <= 1; ++di) {
                int ii = i + di, jj = j + dj;
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
                int kk = jj * nx + ii;
                if (vis[(size_t)kk] || !(clr[(size_t)kk] > T)) continue;
                vis[(size_t)kk] = 1; q.push_back(kk);
            }
        }
        return false;
    };
    if (std::getenv("PROFILE") && (!std::getenv("PONLY") || (size_t)std::atoi(std::getenv("PONLY")) == runIdx)) {
        int step = std::getenv("STEP") ? std::atoi(std::getenv("STEP")) : std::max(1, (int)(0.1 / cell));
        for (int j = ny - 1; j >= 0; j -= step) { std::string row; for (int i = 0; i < nx; i += step) { size_t q = (size_t)j * nx + i; row += clr[q] <= 0 ? '#' : (role[q] == 1 ? 'P' : role[q] == 2 ? 'W' : '.'); } std::printf("     R|%s|\n", row.c_str()); }
        std::map<size_t, int> hist; for (int j = 0; j < ny; j += 2) for (int i = 0; i < nx; i += 2) { size_t q = (size_t)j * nx + i; if (clr[q] <= 0) { size_t who; double c1 = cuts.clearanceW({x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell}, 5, who); if (c1 <= 0) hist[who]++; else hist[SIZE_MAX]++; } }
        for (auto [w, n] : hist) std::printf("     cutter mv %zu: %d cells\n", w == SIZE_MAX ? (size_t)0 : w, n);
    }
    if (!connected(0.0)) return res;
    res.joined = true;
    double lo = 0, hi = 0.6;
    for (int it = 0; it < 12; ++it) { double mm = (lo + hi) / 2; if (connected(mm)) lo = mm; else hi = mm; }
    res.neck = 2 * lo;
    return res;
}

int main(int argc, char** argv) {
    unsigned long seed = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1;
    int N = argc > 2 ? std::atoi(argv[2]) : 100;
    int fam = argc > 3 ? std::atoi(argv[3]) : 0;
    int forceSep = argc > 4 ? std::atoi(argv[4]) : -1;
    int target = argc > 5 ? std::atoi(argv[5]) : -1;
    double cell = std::getenv("CELL") ? std::atof(std::getenv("CELL")) : 0.02;
    bool verbose = std::getenv("V") != nullptr;
    rng.seed(seed);
    int scenes = 0, det = 0, tabsKept = 0, tabsChecked = 0, severed = 0, thin = 0, noSeed = 0, noTarget = 0, countBad = 0, gouge = 0, clampBad = 0, sepCls = 0;
    int requestedTot = 0, leftTot = 0;
    double minNeck = 1e9;
    auto t0 = std::chrono::steady_clock::now();
    for (int sc = 0; sc < N; ++sc) {
        ProfileParams p;
        double D = std::vector<double>{2.0, 3.175, 6.0, 8.0, 12.0}[(size_t)I(0, 4)];
        int sideK = I(0, 9);
        p.side = sideK < 5 ? ProfileSide::Outside : ProfileSide::Inside;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = I(0, 2) == 0 ? 0.0 : U(0.0, 0.5);
        p.passes.startDepth = I(0, 3) == 0 ? U(0, 3) : 0.0;
        p.passes.cutDepth = U(2, 20);
        p.passes.passDepth = I(0, 3) == 0 ? U(0.4, 1.5) : U(1, 8);
        if (I(0, 5) == 0) p.passes.lastPassThickness = U(0.2, 1.0);
        p.separateLastPass = forceSep >= 0 ? forceSep : I(0, 1);
        p.lastPassAllowance = I(0, 5) == 0 ? U(0.0, 0.1) : U(0.05, 1.0);
        p.lastPassReverse = I(0, 1);
        p.tabs.enabled = true;
        p.tabs.length = I(0, 1) ? U(0.3, 4) : U(1, 20);
        if (std::getenv("TLMIN")) p.tabs.length = U(std::atof(std::getenv("TLMIN")), std::atof(std::getenv("TLMAX")));
        p.tabs.thickness = I(0, 7) == 0 ? U(p.passes.cutDepth, p.passes.cutDepth + 5) : U(0.3, std::min(6.0, p.passes.cutDepth));
        p.tabs.spacing = I(0, 3) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
        p.tabs.count = I(0, 1) ? 2 * I(0, 4) + 1 : I(1, 9);
        p.tabs.distance = U(10, 120);
        p.tabs.avoidCorners = I(0, 1);
        p.tabs.threeD = I(0, 2) == 0;
        p.ramp.type = (RampType)I(0, 3);
        p.ramp.distance = U(1, 40);
        p.ramp.useAngle = I(0, 2) == 0;
        p.ramp.angle = U(1, 30) * PI / 180;
        p.ramp.onLeadIn = I(0, 3) == 0;
        p.leads.type = (LeadType)I(0, 2);
        p.leads.length = U(0.5, 15);
        p.leads.angle = U(0.05, PI / 2);
        p.leads.radius = U(0.5, 12);
        p.leads.sweep = U(5, 90) * PI / 180;
        p.leads.leadOut = I(0, 1);
        p.leads.overcut = I(0, 1) ? U(0, 20) : 0.0;
        p.startAt = (StartAt)I(0, 2);
        p.startAnchor = (BoxAnchor)I(0, 8);
        p.startPosition = {U(-50, 50), U(-50, 50)};
        p.order.leftToRight = I(0, 3) == 0;
        p.order.grid = I(0, 3) == 0;
        bool useV = false; double vang = 90;
        std::vector<Contour> cs;
        int f = fam == 0 ? I(1, 9) : fam;
        double R = U(8, 35);
        switch (f) {
            case 1: cs.push_back(Contour::circle({0, 0}, U(4, 30), I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW)); break;  // concave (Inside) / convex (Outside) arcs
            case 2: { int k = I(0, 2); if (k == 0) cs.push_back(flower({0, 0}, U(2.5, 10), 2 * I(2, 5))); else if (k == 1) cs.push_back(gear({0, 0}, U(8, 25), I(6, 20), U(1, 6))); else cs.push_back(spikes({0, 0}, U(8, 25), I(3, 8))); break; }
            case 3: {  // nested
                double W = U(50, 120), H = U(40, 90);
                cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {W, H}}));
                int n = I(1, 3);
                for (int i = 0; i < n; ++i) { double rr = U(4, 15); Vec2 c{U(rr + 0.5, W - rr - 0.5), U(rr + 0.5, H - rr - 0.5)}; cs.push_back(I(0, 1) ? Contour::circle(c, rr) : Contour::rectangle(geometry::Box2{{c.x - rr, c.y - rr * 0.7}, {c.x + rr, c.y + rr * 0.7}})); }
                if (I(0, 1)) cs.push_back(Contour::circle({W / 2, H / 2}, std::min(W, H) * 0.1));
                break;
            }
            case 4: cs.push_back(bezBlob({0, 0}, R, I(4, 9))); break;
            case 5: cs.push_back(I(0, 1) ? anyOpen({0, 0}, R) : bezWave({0, 0}, U(30, 120), I(2, 6))); if (I(0, 3) == 0) p.side = ProfileSide::On; break;
            case 6: cs.push_back(I(0, 1) ? Contour::rectangle(geometry::Box2{{0, 0}, {U(10, 120), U(8, 80)}}) : chamfered({0, 0}, U(30, 100), U(20, 70), U(0.5, 10))); if (I(0, 1)) { p.startAt = StartAt::ClosestOnBoundingBox; } break;
            case 7: cs.push_back(ellipse({0, 0}, U(5, 40), U(3, 20), U(0, PI))); break;
            case 8: {  // V-bit
                useV = true; vang = std::vector<double>{60, 90, 120}[(size_t)I(0, 2)];
                cs.push_back(anyClosed({0, 0}, R));
                p.passes.cutDepth = U(1, 6);
                p.tabs.thickness = U(0.2, p.passes.cutDepth);
                p.corners.sharpExternal = I(0, 1); p.corners.sharpInternal = I(0, 1);
                break;
            }
            case 99: { int vd = 0; cs.clear(); p = ProfileParams{}; readScene(std::getenv("SCENE"), cs, p, D, vd); if (vd) { useV = true; vang = vd; D /= 2; } break; }
            case 23: {  // N1: rectangle, 4 tabs centred on the corners
                cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {40, 30}})); p = ProfileParams{};
                D = std::vector<double>{3.175, 6.0, 12.0}[(size_t)(sc % 3)]; p.side = (sc / 3) % 2 ? ProfileSide::Inside : ProfileSide::Outside; p.allowance = (sc / 6) % 2 ? 0.2 : 0.0;
                p.separateLastPass = (sc / 12) % 2; p.lastPassAllowance = 0.3;
                p.tabs.enabled = true; p.tabs.length = 0.25 + 0.25 * ((sc / 24) % 20); p.tabs.thickness = 1.5; p.tabs.count = 4;
                p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomCentre; p.passes.cutDepth = 6; p.passes.passDepth = 3;
                break;
            }
            case 20: {  // round-1 b5circle (band 26010 fam 10 scene 54)
                cs.push_back(Contour::circle({0, 0}, 19.679080419874751)); p = ProfileParams{}; D = 8.0;
                p.side = ProfileSide::Inside; p.allowance = 0.35808273924283024; p.passes.startDepth = 2.5890622067618838; p.passes.cutDepth = 6.6827760460028767; p.passes.passDepth = 5.5108253589101883;
                p.separateLastPass = true; p.lastPassAllowance = 0.89243563469697218; p.tabs.enabled = true; p.tabs.count = 1; p.tabs.length = 0.62026736706984131; p.tabs.thickness = 4.3090084694084512;
                p.ramp.type = RampType::ZigZag; p.ramp.distance = 36.75464170141808; p.leads.type = LeadType::Circular; p.leads.length = 8.2550718220314288; p.leads.angle = 0.69261347705451748; p.leads.radius = 6.3947391786514576; p.leads.sweep = 0.15589211200284489;
                p.startAt = StartAt::Optimise; p.startAnchor = BoxAnchor::TopRight; p.startPosition = {11.026666867948485, -6.302635438741838};
                break;
            }
            case 21: {  // round-1 b5blob
                const double B[8][8] = {
        {24.528297700458356, -0.075131575306200141, 24.589931499642784, 5.5471493574607127, 21.023356708879575, 14.160905540386025, 16.933676131720073, 17.50028364950516},
        {16.933676131720073, 17.50028364950516, 12.843995554560571, 20.839661758624295, 4.6004164157089189, 21.080710980455006, -0.009785762498662717, 19.961137079408608},
        {-0.009785762498662717, 19.961137079408608, -4.6199879407062436, 18.841563178362211, -7.0160096946899637, 14.103805991146414, -10.727536937525418, 10.782840243226778},
        {-10.727536937525418, 10.782840243226778, -14.439064180360873, 7.461874495307141, -21.381104878470005, 4.4245520465845924, -22.278949219511389, 0.035342591890786854},
        {-22.278949219511389, 0.035342591890786854, -23.176793560552774, -4.3538668628030184, -19.86863318804566, -13.108168954594763, -16.114602983773732, -15.552416484936053},
        {-16.114602983773732, -15.552416484936053, -12.360572779501807, -17.996664015277343, -5.2011807139443693, -14.516645013130244, 0.24523200612016866, -14.630142590156956},
        {0.24523200612016866, -14.630142590156956, 5.6916447261847072, -14.743640167183667, 12.516695720890473, -18.65923711623811, 16.563873336613504, -16.233401947096318},
        {16.563873336613504, -16.233401947096318, 20.611050952336534, -13.807566777954527, 24.466663901273929, -5.6974125080731124, 24.528297700458356, -0.075131575306200141}};
                Contour c; c.closed = true;
                for (auto& bb : B) c.append(geometry::BezierSpan{{bb[0], bb[1]}, {bb[2], bb[3]}, {bb[4], bb[5]}, {bb[6], bb[7]}});
                cs.push_back(c); p = ProfileParams{}; D = 12.0;
                p.side = ProfileSide::Inside; p.direction = CutDirection::Climb; p.allowance = 0.49190493333618557; p.passes.cutDepth = 18.165175165326691; p.passes.passDepth = 5.9478725204244043;
                p.separateLastPass = true; p.lastPassAllowance = 0.95809446752021477; p.tabs.enabled = true; p.tabs.count = 1; p.tabs.length = 1.4386404699535462; p.tabs.thickness = 3.576056744455637; p.tabs.avoidCorners = true;
                p.leads.type = LeadType::Straight; p.leads.length = 14.197196335998006; p.leads.angle = 1.1481504767893238; p.leads.leadOut = true;
                p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::TopLeft; p.startPosition = {39.967753505838715, -23.008829543971238}; p.order.leftToRight = true; p.order.grid = true;
                break;
            }
            default: cs.push_back(anyClosed({0, 0}, R)); if (I(0, 2) == 0) cs.push_back(anyClosed({3.5 * 35, 0}, U(8, 25))); break;
        }
        if (std::getenv("ONP") && f != 99) { bool anyOpen0 = false; for (auto& c : cs) anyOpen0 |= !c.closed; if (!anyOpen0 && I(0, std::atoi(std::getenv("ONP")) - 1) == 0) p.side = ProfileSide::On; }
        if (target >= 0 && sc != target) continue;
        ++scenes;
        tooling::Tool tool = useV ? vbit(D * 2, vang) : endMill(D);
        if (target >= 0 && std::getenv("DUMPSCENE")) writeScene(std::getenv("DUMPSCENE"), cs, p, useV ? D * 2 : D, useV ? (int)vang : 0);
        auto res = profileToolpath(cs, tool, p);
        auto res2 = profileToolpath(cs, tool, p);
        std::vector<std::string> errs;
        char b[800];
        if (!(res.toolpath == res2.toolpath) || res.diagnostics.size() != res2.diagnostics.size()) { ++det; errs.push_back("NONDET"); }
        const auto& mv = res.toolpath.moves;
        const double bottom = p.passes.bottomZ();
        const double th = std::min(p.tabs.thickness, p.passes.cutDepth);
        const double zTab = bottom + th;
        if ((p.tabs.thickness > p.passes.cutDepth) != res.has(ProfileIssue::TabThicknessClamped)) { ++clampBad; errs.push_back("clamp diag mismatch"); }
        const double r = profileToolRadius(tool, p.passes);
        const double Rtool = tool.diameter / 2;
        const double tanHalf = useV ? std::tan(vang * PI / 360) : 0;
        // diagnostics
        int left = 0, reqDiag = 0;
        for (auto& d : res.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced) { int n1, m1; if (std::sscanf(std::strstr(d.message.c_str(), ": ") + 2, "%d of %d", &n1, &m1) == 2 && d.message.find("left out") != std::string::npos) { left += n1; reqDiag += m1; } }
        if (std::getenv("FAKE")) {  // sanity: lower the first block's tab plateaus to the bottom
            size_t e = 0; while (e < res.toolpath.moves.size() && res.toolpath.moves[e].kind != MoveKind::Retract) ++e;
            for (size_t k = 0; k < e; ++k) if (std::fabs(res.toolpath.moves[k].to.z - zTab) < 1e-9) res.toolpath.moves[k].to.z = bottom;
        }
        auto pts = expand(res.toolpath);
        if (mv.empty()) { if (verbose || target >= 0) std::printf("scene %d empty\n", sc); continue; }
        if (!p.tabs.threeD) {  // ascending sloped feed that ends at the tab top: the tab is approached on a slope
            int slopes = 0; double worstDz = 0;
            for (size_t i = 1; i < mv.size(); ++i) {
                const auto& A = mv[i - 1].to; const auto& B = mv[i].to;
                if (mv[i].kind != MoveKind::Feed && !isArc(mv[i].kind)) continue;
                if (std::fabs(B.z - zTab) < 1e-9 && A.z < zTab - 1e-6 && std::hypot(B.x - A.x, B.y - A.y) > 0.01 && zTab < -p.passes.startDepth - 1e-9) { ++slopes; worstDz = std::max(worstDz, zTab - A.z); }
            }
            if (slopes) { snprintf(b, sizeof b, "SLOPE-UP-TO-TAB x%d (tool up to %.3f below the tab top over the tab)", slopes, worstDz); errs.push_back(b); }
        }
        // gouge (end mill): any point below 0 within r - 1e-3 of a vector (allowance >= 0)
        Vecs vecs(cs, r + 2);
        Region region(cs);
        Vecs vecsFar(cs, 3 * r + 4);
        if (!useV && p.side != ProfileSide::On) {
            double worst = 1e9;
            for (size_t i = 1; i < pts.size(); ++i) {
                if (res.toolpath.moves[pts[i].mi].kind == MoveKind::Rapid) continue;
                P3 A = pts[i - 1], B = pts[i];
                double L = std::hypot(B.x - A.x, B.y - A.y); int n = std::max(1, (int)std::ceil(L / 0.05));
                for (int k = 0; k <= n; ++k) { double t = (double)k / n; double z = A.z + (B.z - A.z) * t; if (z >= 0) continue; worst = std::min(worst, vecs.dist({A.x + (B.x - A.x) * t, A.y + (B.y - A.y) * t}) - r); }
            }
            if (worst < -1e-3) { ++gouge; snprintf(b, sizeof b, "GOUGE %.5f", worst); errs.push_back(b); }
        }
        // blocks
        struct Blk { size_t a, e; double med; bool fin; };
        std::vector<Blk> bls;
        { size_t a = 0; for (size_t i = 0; i < mv.size(); ++i) { if (mv[i].kind == MoveKind::Rapid) a = i; if (mv[i].kind == MoveKind::Retract) bls.push_back({a, i, 0, true}); } }
        const double dVec0 = p.side == ProfileSide::On ? std::fabs(p.allowance) : r + p.allowance;  // v4: On path runs |allowance| off the vector
        const double lpa = p.separateLastPass ? p.lastPassAllowance : 0.0;
        if (p.separateLastPass) {
            for (auto& bl : bls) {
                std::vector<double> ds;
                for (size_t i = 1; i < pts.size(); ++i) {
                    if (pts[i].mi <= bl.a || pts[i].mi > bl.e) continue;
                    P3 A = pts[i - 1], B = pts[i];
                    if (A.z > bottom + 1e-7 || B.z > bottom + 1e-7) continue;
                    double L = std::hypot(B.x - A.x, B.y - A.y); int n = std::max(1, (int)std::ceil(L / 0.25));
                    for (int k = 0; k < n; ++k) { double t = (k + 0.5) / n; ds.push_back(vecs.dist({A.x + (B.x - A.x) * t, A.y + (B.y - A.y) * t})); }
                }
                if (!ds.empty()) { std::nth_element(ds.begin(), ds.begin() + (long)ds.size() / 2, ds.end()); bl.med = ds[ds.size() / 2]; }
                bl.fin = std::fabs(bl.med - dVec0) < std::fabs(bl.med - dVec0 - lpa);
                if (target >= 0) std::printf("  block %zu-%zu median %.4f (fin %.4f rough %.4f)\n", bl.a, bl.e, bl.med, dVec0, dVec0 + lpa);
            }
            { size_t nf = 0; for (auto& bl : bls) nf += bl.fin; bool anyOpenVec = false; for (auto& c : cs) anyOpenVec |= !c.closed;
              if (bls.size() % 2 == 0 && (useV || (anyOpenVec && nf * 2 != bls.size()))) { for (size_t q = 0; q < bls.size(); ++q) bls[q].fin = q >= bls.size() / 2; if (target >= 0) std::printf("  open+sep: classified by order\n"); } }
            bool seenFin = false;
            for (auto& bl : bls) { if (bl.fin) seenFin = true; else if (seenFin) { ++sepCls; errs.push_back("roughing after finishing / classification"); break; } }
        }
        // cut segments below the tab top
        Cuts cuts;
        cuts.build(res.toolpath, pts, zTab - 1e-4, useV ? Rtool : r, tanHalf, useV, 0.7);
        // primary blocks
        int keptScene = 0, reqScene = 0;
        bool countable = true;
        for (auto& bl : bls) {
            if (!bl.fin) continue;
            if (std::getenv("DUMP")) for (size_t k = bl.a; k <= bl.e; ++k) std::printf("    mv %zu kind %d (%.4f,%.4f,%.4f)\n", k, (int)mv[k].kind, mv[k].to.x, mv[k].to.y, mv[k].to.z);
            size_t i0 = SIZE_MAX;
            for (size_t i = 0; i < pts.size(); ++i) if (pts[i].mi > bl.a && pts[i].mi <= bl.e && pts[i].z <= bottom + 1e-7) { i0 = i; break; }
            if (i0 == SIZE_MAX) continue;
            /* v4: the final level starts at the end of a tab (level changes at tab ends); take in the
               tab-top stretch just before it (that tab, traversed at the end of the previous level) */
            if (!std::getenv("NOSEAM") && zTab > bottom + 1e-7) {
                while (i0 > 0 && pts[i0 - 1].mi > bl.a && pts[i0 - 1].z > pts[i0].z + 1e-9 && pts[i0 - 1].z < zTab - 1e-7) --i0;  /* 3D ramp */
                if (i0 > 0 && pts[i0 - 1].mi > bl.a && std::fabs(pts[i0 - 1].z - zTab) < 1e-7) while (i0 > 0 && pts[i0 - 1].mi > bl.a && std::fabs(pts[i0 - 1].z - zTab) < 1e-7) --i0;
                else while (i0 + 1 < pts.size() && pts[i0].z > bottom + 1e-7) ++i0;  /* not a tab: undo */
            }
            size_t iEnd = i0;
            for (size_t i = i0; i < pts.size() && pts[i].mi <= bl.e; ++i) iEnd = i;
            PL F;
            double s = 0;
            for (size_t i = i0; i <= iEnd; ++i) {
                Vec2 q{pts[i].x, pts[i].y};
                if (!F.p.empty()) s += geometry::distance(F.p.back(), q);
                F.p.push_back(q); F.s.push_back(s); F.z.push_back(pts[i].z);
            }
            // loop end: return to start (closed), else last bottom point
            size_t loopEnd = F.p.size() - 1, loopStart = 0;
            bool closed = false;
            bool anyClosedVec = false; for (auto& c : cs) anyClosedVec |= c.closed;
            if (anyClosedVec) { double best = 0.5;
              for (size_t i = 0; i < F.p.size() && F.s[i] < 40; ++i)
                for (size_t k = F.p.size(); k-- > i + 1;) if (F.s[k] - F.s[i] > best + 1e-3 && geometry::distance(F.p[k], F.p[i]) < 1e-5) { loopStart = i; loopEnd = k; closed = true; best = F.s[k] - F.s[i]; break; } }
            if (!closed) { for (size_t k = F.p.size(); k-- > 0;) if (F.z[k] <= zTab + 1e-7) { loopEnd = k; break; } if (p.tabs.spacing == TabSpacing::ConstantDistance && p.leads.type != LeadType::None) countable = false; }
            double Lloop = F.s[loopEnd] - F.s[loopStart];
            int req = p.tabs.spacing == TabSpacing::ConstantNumber ? p.tabs.count : std::max(1, (int)std::lround(Lloop / p.tabs.distance));
            if (p.tabs.spacing == TabSpacing::ConstantDistance && std::fabs(std::fmod(Lloop / p.tabs.distance, 1.0) - 0.5) < 0.02) countable = false;
            reqScene += req;
            if (target >= 0) std::printf("  primary cut: Lloop %.4f closed %d req %d dist %.3f\n", Lloop, (int)closed, req, p.tabs.distance);
            // flat runs at zTab within the loop
            std::vector<std::pair<double, double>> runs;
            bool inRun = false;
            for (size_t k = loopStart + 1; k <= loopEnd; ++k) {
                bool atTop = std::fabs(F.z[k] - zTab) < 1e-7 && std::fabs(F.z[k - 1] - zTab) < 1e-7 && zTab > bottom + 1e-7;
                if (!atTop) { inRun = false; continue; }
                if (!(F.s[k] > F.s[k - 1])) continue;
                if (inRun) runs.back().second = F.s[k];
                else { runs.push_back({F.s[k - 1], F.s[k]}); inRun = true; }
            }
            // overcut / repeat: not counted (loop only). Discard runs not bounded by bottom on both sides? keep all.
            F.index(std::max(dVec0, r + lpa) + 0.8);
            // classify runs: on the cut? repeat of an earlier one (overcut)?
            struct RunInfo { Vec2 m, nL; bool valid; size_t canon; };
            std::vector<RunInfo> ri(runs.size());
            for (size_t q = 0; q < runs.size(); ++q) {
                double sm = (runs[q].first + runs[q].second) / 2;
                auto it = std::lower_bound(F.s.begin(), F.s.end(), sm); size_t k = (size_t)(it - F.s.begin()); if (k == 0) k = 1;
                while (k + 1 < F.s.size() && !(F.s[k] > F.s[k - 1])) ++k;
                Vec2 a = F.p[k - 1], bb = F.p[k]; double l = geometry::distance(a, bb);
                Vec2 t = (bb - a) * (1 / l);
                ri[q].nL = {-t.y, t.x};
                ri[q].m = a + (bb - a) * ((sm - F.s[k - 1]) / l);
                { int on = 0, tot = 0;
                  for (size_t k2 = 1; k2 < F.p.size(); ++k2) {
                      if (F.s[k2] <= runs[q].first || F.s[k2 - 1] >= runs[q].second) continue;
                      for (int t2 = 0; t2 < 4; ++t2) { Vec2 x = F.p[k2 - 1] + (F.p[k2] - F.p[k2 - 1]) * ((t2 + 0.5) / 4); double dv = vecs.dist(x); ++tot;
                          if (p.side == ProfileSide::On ? dv <= std::fabs(p.allowance) + 0.05 : std::fabs(dv - std::min(dVec0, vecs.reach)) <= 0.05) ++on; } }
                  ri[q].valid = tot > 0 && (on * 10 >= tot * 3 || (closed && on > 0)); /* v4: runs round V-bit mitre corners */ }
                ri[q].canon = q;
                if (!ri[q].valid) { if (target >= 0) std::printf("  run at (%.3f,%.3f) len %.3f off the cut (lead?) skipped: mid dv %.4f expected %.4f\n", ri[q].m.x, ri[q].m.y, runs[q].second - runs[q].first, vecs.dist(ri[q].m), dVec0); continue; }
                for (size_t o = 0; o < q; ++o) if (ri[o].valid && ri[o].canon == o && geometry::distance(ri[o].m, ri[q].m) < 0.5) { ri[q].canon = o; break; }
                if (ri[q].canon != q && target >= 0) std::printf("  run at (%.3f,%.3f) repeats a tab (overcut) skipped\n", ri[q].m.x, ri[q].m.y);
            }
            Cuts virt;
            for (size_t q = 0; q < runs.size(); ++q) {
                if (!ri[q].valid) continue;
                auto& run = runs[q];
                Vec2 prev{};
                for (double sv = run.first; ; sv += 0.05) {
                    double u = std::min(sv, run.second);
                    auto it = std::lower_bound(F.s.begin(), F.s.end(), u); size_t k = (size_t)(it - F.s.begin()); if (k == 0) k = 1; if (k >= F.s.size()) k = F.s.size() - 1;
                    double t = (F.s[k] - F.s[k - 1]) > 0 ? (u - F.s[k - 1]) / (F.s[k] - F.s[k - 1]) : 0; Vec2 qq = F.p[k - 1] + (F.p[k] - F.p[k - 1]) * t;
                    if (sv > run.first) virt.s.push_back({prev, qq, useV ? std::min(Rtool, (zTab - bottom) * tanHalf) : r, ri[q].canon});
                    prev = qq;
                    if (u >= run.second) break;
                }
            }
            for (size_t i = 0; i < virt.s.size(); ++i) { double e = virt.s[i].rad + 0.7; virt.g.add((int)i, std::min(virt.s[i].a.x, virt.s[i].b.x) - e, std::min(virt.s[i].a.y, virt.s[i].b.y) - e, std::max(virt.s[i].a.x, virt.s[i].b.x) + e, std::max(virt.s[i].a.y, virt.s[i].b.y) + e); }
            if (target >= 0) for (auto& run : runs) std::printf("  run s [%.4f, %.4f]\n", run.first, run.second);
            for (size_t q = 0; q < runs.size(); ++q) {
                if (!ri[q].valid || ri[q].canon != q) continue;
                auto& run = runs[q];
                Vec2 m = ri[q].m, nL = ri[q].nL;
                ++keptScene;
                int vecSide = p.side == ProfileSide::On ? 0 : (vecs.dist(m + nL * dVec0) < vecs.dist(m - nL * dVec0) ? 1 : -1);
                double dWaste = r + lpa;
                double dV = p.side == ProfileSide::On ? r : std::max(dVec0, r);
                TabRes tr;
                bool useRegion = p.side != ProfileSide::On && anyClosedVec && !std::getenv("VORONOI");
                for (auto& c : cs) if (!c.closed) useRegion = false;
                if (useRegion) tr = checkTabRegion(F, run, q, p.side == ProfileSide::Outside, region, vecs, dVec0, (useV ? std::min(Rtool, (zTab - bottom) * tanHalf) : r) + lpa, cuts, virt, cell);
                else tr = checkTab(F, {run}, vecSide, dV, dWaste, cuts, cell, r);
                ++tabsKept;
                if (tr.noSeed) ++noSeed;
                if (tr.noTarget) ++noTarget;
                if (!tr.checked) { if (verbose || target >= 0) std::printf("  tab at (%.3f,%.3f) len %.3f: not checkable (seed %d target %d)\n", tr.mid.x, tr.mid.y, tr.len, !tr.noSeed, !tr.noTarget); continue; }
                ++tabsChecked;
                if (!tr.joined) { ++severed; snprintf(b, sizeof b, "SEVERED tab at (%.3f,%.3f) plateau %.3f", tr.mid.x, tr.mid.y, tr.len); errs.push_back(b); }
                else { minNeck = std::min(minNeck, tr.neck); if (tr.neck < 0.1) { ++thin; snprintf(b, sizeof b, "THIN tab at (%.3f,%.3f) plateau %.3f neck %.4f", tr.mid.x, tr.mid.y, tr.len, tr.neck); errs.push_back(b); } }
                if (target >= 0) {
                    auto at = [&](double sv) { auto it = std::lower_bound(F.s.begin(), F.s.end(), sv); size_t k = (size_t)(it - F.s.begin()); if (k == 0) k = 1; if (k >= F.s.size()) k = F.s.size() - 1; double t = (F.s[k] - F.s[k - 1]) > 0 ? (sv - F.s[k - 1]) / (F.s[k] - F.s[k - 1]) : 0; return F.p[k - 1] + (F.p[k] - F.p[k - 1]) * t; };
                    Vec2 ea = at(run.first), eb = at(run.second);
                    std::printf("  tab (%.3f,%.3f) plateau %.3f joined %d neck %.4f | plateau ends (%.4f,%.4f)-(%.4f,%.4f) %.4f apart (2r %.4f)\n", tr.mid.x, tr.mid.y, tr.len, tr.joined, tr.neck, ea.x, ea.y, eb.x, eb.y, geometry::distance(ea, eb), 2 * r);
                }
            }
        }
        requestedTot += reqScene; leftTot += left;
        if (countable && keptScene + left != reqScene) { ++countBad; snprintf(b, sizeof b, "COUNT kept %d + left %d != requested %d (diag M sum %d)", keptScene, left, reqScene, reqDiag); errs.push_back(b); }
        if (!errs.empty() || verbose || target >= 0) {
            std::printf("scene %d fam %d side %d D %.3f V %d allow %.3f sep %d lpa %.3f rev %d n %d sp %d len %.3f th %.3f 3d %d avoid %d ramp %d lead %d out %d oc %.2f start %d depth %.2f/%.2f:", sc, f, (int)p.side, D, (int)useV, p.allowance, p.separateLastPass, p.lastPassAllowance, p.lastPassReverse, p.tabs.count, (int)p.tabs.spacing, p.tabs.length, p.tabs.thickness, p.tabs.threeD, p.tabs.avoidCorners, (int)p.ramp.type, (int)p.leads.type, p.leads.leadOut, p.leads.overcut, (int)p.startAt, p.passes.cutDepth, p.passes.passDepth);
            for (auto& e : errs) std::printf(" [%s]", e.c_str());
            std::printf("\n");
            if (target >= 0) for (auto& d : res.diagnostics) std::printf("   diag %d %s\n", (int)d.issue, d.message.c_str());
        }
    }
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("SUMMARY seed %lu fam %d sep %d cell %.3f: scenes %d det %d gouge %d clampBad %d sepOrder %d | tabs kept %d checked %d severed %d thin(<0.1) %d minNeck %.4f noSeed %d noTarget %d | requested %d left %d countBad %d | %.1fs\n",
                seed, fam, forceSep, cell, scenes, det, gouge, clampBad, sepCls, tabsKept, tabsChecked, severed, thin, minNeck, noSeed, noTarget, requestedTot, leftTot, countBad, secs);
}
