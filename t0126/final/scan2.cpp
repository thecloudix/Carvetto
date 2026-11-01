// T-0125 round-2 verifier: diameter scans with an exact oracle over line AND arc walls.
// Throwaway; independent of pocket_oracle.hpp and of the offset code (except the "raw" subset check,
// which deliberately reproduces Clipper's raw inset to test "only shrinks").
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"

using namespace carvetto;
using geometry::Contour;
using geometry::Vec2;
using Poly = std::vector<Vec2>;
static const double PI = 3.14159265358979323846, TAU = 2 * PI;

static double segPt(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x, dy = b.y - a.y, L = dx * dx + dy * dy;
    double t = L > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / L : 0;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
static double orient(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
static bool properCross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    double o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    return ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0));
}
static double segSeg(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    if (properCross(a, b, c, d)) return 0;
    return std::min({segPt(a, c, d), segPt(b, c, d), segPt(c, a, b), segPt(d, a, b)});
}
struct Arc { Vec2 c; double r, a0, sw; };  // CCW from a0 by sw in (0, 2pi]
static bool inArc(const Arc& A, double th) { double t = std::fmod(th - A.a0, TAU); if (t < 0) t += TAU; return t <= A.sw + 1e-15; }
static Vec2 arcPt(const Arc& A, double th) { return {A.c.x + A.r * std::cos(th), A.c.y + A.r * std::sin(th)}; }
static double ptArc(Vec2 p, const Arc& A) {
    double dx = p.x - A.c.x, dy = p.y - A.c.y, L = std::hypot(dx, dy);
    if (L > 0 && inArc(A, std::atan2(dy, dx))) return std::fabs(L - A.r);
    Vec2 e0 = arcPt(A, A.a0), e1 = arcPt(A, A.a0 + A.sw);
    return std::min(std::hypot(p.x - e0.x, p.y - e0.y), std::hypot(p.x - e1.x, p.y - e1.y));
}
static double segArc(Vec2 a, Vec2 b, const Arc& A) {
    double best = std::min(ptArc(a, A), ptArc(b, A));
    best = std::min(best, segPt(arcPt(A, A.a0), a, b));
    best = std::min(best, segPt(arcPt(A, A.a0 + A.sw), a, b));
    double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
    if (L2 > 0) {
        double t = std::clamp(((A.c.x - a.x) * dx + (A.c.y - a.y) * dy) / L2, 0.0, 1.0);
        Vec2 q{a.x + t * dx, a.y + t * dy};
        double ql = std::hypot(q.x - A.c.x, q.y - A.c.y);
        if (ql > A.r && inArc(A, std::atan2(q.y - A.c.y, q.x - A.c.x))) best = std::min(best, ql - A.r);
        // crossings
        double fx = a.x - A.c.x, fy = a.y - A.c.y;
        double B = 2 * (fx * dx + fy * dy), Cc = fx * fx + fy * fy - A.r * A.r, disc = B * B - 4 * L2 * Cc;
        if (disc >= 0) {
            double s = std::sqrt(disc);
            for (double tt : {(-B - s) / (2 * L2), (-B + s) / (2 * L2)})
                if (tt >= 0 && tt <= 1 && inArc(A, std::atan2(fy + tt * dy, fx + tt * dx))) return 0;
        }
    }
    return best;
}

struct World {
    std::string name;
    std::vector<Contour> input;
    std::vector<std::pair<Vec2, Vec2>> segs;
    std::vector<Arc> arcs;
    std::function<bool(Vec2)> inside;
    double x0, y0, x1, y1;
    // bucket grid over segs
    double gx0, gy0, cell; int nx = 1, ny = 1; std::vector<std::vector<int>> b;
    int ci(double x) const { return std::clamp(int(std::floor((x - gx0) / cell)), 0, nx - 1); }
    int cj(double y) const { return std::clamp(int(std::floor((y - gy0) / cell)), 0, ny - 1); }
    void build() {
        x0 = y0 = 1e300; x1 = y1 = -1e300;
        auto acc = [&](Vec2 p) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); };
        for (auto& s : segs) { acc(s.first); acc(s.second); }
        for (auto& a : arcs) { acc({a.c.x - a.r, a.c.y - a.r}); acc({a.c.x + a.r, a.c.y + a.r}); }
        gx0 = x0; gy0 = y0; cell = std::max(x1 - x0, y1 - y0) / 128;
        nx = int((x1 - x0) / cell) + 1; ny = int((y1 - y0) / cell) + 1;
        b.assign(size_t(nx) * ny, {});
        for (int k = 0; k < (int)segs.size(); ++k) {
            auto [p, q] = segs[k];
            for (int j = cj(std::min(p.y, q.y)); j <= cj(std::max(p.y, q.y)); ++j)
                for (int i = ci(std::min(p.x, q.x)); i <= ci(std::max(p.x, q.x)); ++i) b[size_t(j) * nx + i].push_back(k);
        }
    }
    double segDist(Vec2 a, Vec2 bb, double R) const {  // exact if < R
        double best = R;
        for (auto& A : arcs) best = std::min(best, segArc(a, bb, A));
        static thread_local std::vector<int> seen; static thread_local int stamp = 0;
        if (seen.size() < segs.size()) seen.assign(segs.size(), 0);
        ++stamp;
        for (int j = cj(std::min(a.y, bb.y) - R); j <= cj(std::max(a.y, bb.y) + R); ++j)
            for (int i = ci(std::min(a.x, bb.x) - R); i <= ci(std::max(a.x, bb.x) + R); ++i)
                for (int k : b[size_t(j) * nx + i]) {
                    if (seen[k] == stamp) continue;
                    seen[k] = stamp;
                    best = std::min(best, segSeg(a, bb, segs[k].first, segs[k].second));
                }
        return best;
    }
    double ptDist(Vec2 p) const { for (double R = cell;; R *= 2) { double d = segDist(p, p, R); if (d < R || R > 1e6) return d; } }
};

static bool insideEO(const std::vector<Poly>& ps, Vec2 p) {
    bool in = false;
    for (auto& r : ps)
        for (size_t i = 0, n = r.size(), j = n - 1; i < n; j = i++) {
            Vec2 a = r[i], b = r[j];
            if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
        }
    return in;
}

// ------------------------------------------------ disc union with exact arcs
struct Disc { Vec2 c; double r; };
// Traces the union boundary; returns false on degeneracy.
static bool discUnion(const std::vector<Disc>& D, std::vector<Contour>& loops, std::vector<Arc>& arcsOut) {
    struct A { int i; double a0, a1; Vec2 p0, p1; bool used = false; };
    std::vector<A> arcs;
    for (int i = 0; i < (int)D.size(); ++i) {
        std::vector<std::pair<double, double>> cov;
        bool full = false;
        for (int j = 0; j < (int)D.size(); ++j) {
            if (i == j) continue;
            double dx = D[j].c.x - D[i].c.x, dy = D[j].c.y - D[i].c.y, L = std::hypot(dx, dy);
            if (L >= D[i].r + D[j].r) continue;
            if (L + D[i].r <= D[j].r) { full = true; break; }
            if (L + D[j].r <= D[i].r) continue;
            double ph = std::atan2(dy, dx), al = std::acos(std::clamp((D[i].r * D[i].r + L * L - D[j].r * D[j].r) / (2 * D[i].r * L), -1.0, 1.0));
            if (al < 1e-6 || al > PI - 1e-6) return false;
            double s = ph - al; s = std::fmod(s, TAU); if (s < 0) s += TAU;
            cov.push_back({s, s + 2 * al});
        }
        if (full) continue;
        if (cov.empty()) { arcs.push_back({i, 0, TAU, {}, {}}); continue; }
        // unwrap: split intervals > TAU
        std::vector<std::pair<double, double>> iv;
        for (auto [s, e] : cov) { if (e > TAU) { iv.push_back({s, TAU}); iv.push_back({0, e - TAU}); } else iv.push_back({s, e}); }
        std::sort(iv.begin(), iv.end());
        std::vector<std::pair<double, double>> m;
        for (auto x : iv) { if (!m.empty() && x.first <= m.back().second + 1e-12) m.back().second = std::max(m.back().second, x.second); else m.push_back(x); }
        // free = complement in [0,TAU)
        std::vector<std::pair<double, double>> fr;
        double cur = 0;
        for (auto x : m) { if (x.first > cur + 1e-9) fr.push_back({cur, x.first}); cur = std::max(cur, x.second); }
        if (cur < TAU - 1e-9) fr.push_back({cur, TAU});
        // merge wrap
        if (fr.size() >= 2 && fr.front().first < 1e-12 && fr.back().second > TAU - 1e-12) { fr.front().first = fr.back().first - TAU; fr.pop_back(); }
        for (auto [s, e] : fr) { if (e - s < 1e-7) return false; arcs.push_back({i, s, e, {}, {}}); }
    }
    for (auto& a : arcs) {
        a.p0 = {D[a.i].c.x + D[a.i].r * std::cos(a.a0), D[a.i].c.y + D[a.i].r * std::sin(a.a0)};
        a.p1 = {D[a.i].c.x + D[a.i].r * std::cos(a.a1), D[a.i].c.y + D[a.i].r * std::sin(a.a1)};
    }
    for (size_t k = 0; k < arcs.size(); ++k) {
        if (arcs[k].used) continue;
        Contour c; c.closed = true;
        size_t cur = k;
        int guard = 0;
        while (!arcs[cur].used) {
            auto& a = arcs[cur]; a.used = true;
            geometry::ArcSpan s; s.centre = D[a.i].c; s.radius = D[a.i].r; s.startAngle = a.a0; s.endAngle = a.a1; s.direction = geometry::ArcDirection::CCW;
            if (a.a1 - a.a0 >= TAU - 1e-12) { s = geometry::ArcSpan::fullCircle(D[a.i].c, D[a.i].r); }
            c.spans.push_back(s);
            arcsOut.push_back({D[a.i].c, D[a.i].r, a.a0, a.a1 - a.a0});
            if (a.a1 - a.a0 >= TAU - 1e-12) break;
            size_t nxt = SIZE_MAX; double bd = 1e-6;
            for (size_t q = 0; q < arcs.size(); ++q) { double dd = std::hypot(arcs[q].p0.x - a.p1.x, arcs[q].p0.y - a.p1.y); if (dd < bd && arcs[q].i != a.i) { bd = dd; nxt = q; } }
            if (nxt == SIZE_MAX || ++guard > 1000) return false;
            cur = nxt;
        }
        loops.push_back(c);
    }
    return true;
}

static void addPoly(World& W, const Poly& p) {
    for (size_t i = 0; i < p.size(); ++i) W.segs.push_back({p[i], p[(i + 1) % p.size()]});
    W.input.push_back(Contour::fromPolyline(geometry::Polyline{p, true}));
}
static Poly circP(Vec2 c, double r, int n) { Poly p; for (int i = 0; i < n; ++i) { double t = TAU * i / n; p.push_back({c.x + r * std::cos(t), c.y + r * std::sin(t)}); } return p; }

static bool makeWorld(int kind, uint64_t seed, World& W) {
    std::mt19937_64 R(seed * 1000003ULL + kind * 7777ULL + 12345);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(R); };
    char buf[200];
    std::vector<Disc> D;
    auto finishDiscs = [&](bool asPocket) {
        std::vector<Contour> loops; std::vector<Arc> arcs;
        if (!discUnion(D, loops, arcs)) return false;
        for (auto& l : loops) W.input.push_back(l);
        for (auto& a : arcs) W.arcs.push_back(a);
        (void)asPocket;
        return true;
    };
    switch (kind) {
    case 0: {  // wiggly disc chain
        int M = int(U(3, 9)); Vec2 p{0, 0}; double ang = U(0, TAU), rprev = U(1, 7);
        D.push_back({p, rprev});
        for (int m = 1; m < M; ++m) { double r = U(1, 7); ang += U(-1.2, 1.2); double L = U(0.35, 0.95) * (r + rprev); L = std::max(L, std::fabs(r - rprev) + 0.2); p = {p.x + L * std::cos(ang), p.y + L * std::sin(ang)}; D.push_back({p, r}); rprev = r; }
        if (!finishDiscs(true)) return false;
        W.inside = [D](Vec2 q) { for (auto& d : D) if (std::hypot(q.x - d.c.x, q.y - d.c.y) < d.r) return true; return false; };
        snprintf(buf, sizeof buf, "discchain-arc M=%d", M); break; }
    case 1: {  // random disc cluster (may enclose holes)
        int M = int(U(4, 14)); double S = U(10, 25);
        for (int m = 0; m < M; ++m) D.push_back({{U(0, S), U(0, S)}, U(1.5, 6)});
        if (!finishDiscs(true)) return false;
        W.inside = [D](Vec2 q) { for (auto& d : D) if (std::hypot(q.x - d.c.x, q.y - d.c.y) < d.r) return true; return false; };
        snprintf(buf, sizeof buf, "disccluster-arc M=%d", M); break; }
    case 2: {  // ring of discs (annulus-like, cusps both sides, hole)
        int M = int(U(6, 14)); double Rr = U(8, 18);
        for (int m = 0; m < M; ++m) { double t = TAU * m / M + U(-0.1, 0.1); double r = Rr * std::sin(PI / M) * U(1.05, 1.6); D.push_back({{Rr * std::cos(t), Rr * std::sin(t)}, r}); }
        if (!finishDiscs(true)) return false;
        W.inside = [D](Vec2 q) { for (auto& d : D) if (std::hypot(q.x - d.c.x, q.y - d.c.y) < d.r) return true; return false; };
        snprintf(buf, sizeof buf, "discring-arc M=%d", M); break; }
    case 3: {  // flower of discs: centre + petals (partially collapsing lobes)
        int N = int(U(3, 9)); double Rc = U(4, 12);
        D.push_back({{0, 0}, Rc});
        for (int m = 0; m < N; ++m) { double t = TAU * m / N + U(-0.15, 0.15), rho = U(0.8, 0.9 * Rc), L = Rc + rho * U(-0.2, 0.9); D.push_back({{L * std::cos(t), L * std::sin(t)}, rho}); }
        if (!finishDiscs(true)) return false;
        W.inside = [D](Vec2 q) { for (auto& d : D) if (std::hypot(q.x - d.c.x, q.y - d.c.y) < d.r) return true; return false; };
        snprintf(buf, sizeof buf, "discflower-arc N=%d", N); break; }
    case 4: {  // rectangle with disc-union islands (true arcs on island walls: unsafe sag side)
        double Wd = U(30, 60), H = U(20, 40);
        Poly rect{{0, 0}, {Wd, 0}, {Wd, H}, {0, H}}; addPoly(W, rect);
        std::vector<std::vector<Disc>> isl;
        for (int tries = 0; tries < 200 && isl.size() < 5; ++tries) {
            std::vector<Disc> I; Vec2 c{U(6, Wd - 6), U(6, H - 6)}; int k = int(U(1, 4));
            for (int q = 0; q < k; ++q) I.push_back({{c.x + U(-3, 3), c.y + U(-3, 3)}, U(1, 3)});
            bool ok = true;
            for (auto& d : I) { if (d.c.x - d.r < 0.5 || d.c.y - d.r < 0.5 || d.c.x + d.r > Wd - 0.5 || d.c.y + d.r > H - 0.5) ok = false;
                for (auto& J : isl) for (auto& e : J) if (std::hypot(d.c.x - e.c.x, d.c.y - e.c.y) < d.r + e.r + 0.3) ok = false; }
            if (ok) isl.push_back(I);
        }
        for (auto& I : isl) { D = I; if (!finishDiscs(false)) return false; }
        std::vector<Disc> all; for (auto& I : isl) for (auto& d : I) all.push_back(d);
        W.inside = [all, Wd, H](Vec2 q) { if (q.x <= 0 || q.y <= 0 || q.x >= Wd || q.y >= H) return false; for (auto& d : all) if (std::hypot(q.x - d.c.x, q.y - d.c.y) < d.r) return false; return true; };
        snprintf(buf, sizeof buf, "rect+%zu disc-islands-arc", isl.size()); break; }
    case 5: {  // sawtooth channel (polygon)
        int T = int(U(4, 14)); double w = U(2, 8), amp = U(1, 6), pitch = U(2, 7); Poly lo, hi;
        for (int t = 0; t <= 2 * T; ++t) { double x = t * pitch / 2; double yl = (t % 2 ? -amp : 0) + U(-0.3, 0.3), yh = w + (t % 2 ? amp * U(0.2, 1) : 0) + U(-0.3, 0.3); lo.push_back({x, yl}); hi.push_back({x + U(-0.4, 0.4), yh}); }
        Poly p = lo; for (int i = (int)hi.size() - 1; i >= 0; --i) p.push_back(hi[i]);
        addPoly(W, p);
        std::vector<Poly> ps{p}; W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "sawchannel T=%d w=%.2f", T, w); break; }
    case 6: {  // very spiky random star
        int K = int(U(40, 200)); Poly p;
        for (int i = 0; i < K; ++i) { double t = TAU * (i + U(-0.3, 0.3)) / K, r = (i % 2 ? U(3, 12) : U(10, 22)); p.push_back({r * std::cos(t), r * std::sin(t)}); }
        addPoly(W, p);
        std::vector<Poly> ps{p}; W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "spikystar K=%d", K); break; }
    case 7: {  // Koch-ish jagged triangle
        int lev = int(U(2, 4)); double S = U(20, 40);
        Poly p{{0, 0}, {S, 0}, {S / 2, -S * std::sqrt(3.0) / 2}};
        std::reverse(p.begin(), p.end());
        for (int l = 0; l < lev; ++l) {
            Poly q; double f = U(0.6, 1.0);
            for (size_t i = 0; i < p.size(); ++i) {
                Vec2 a = p[i], b = p[(i + 1) % p.size()]; Vec2 d{(b.x - a.x) / 3, (b.y - a.y) / 3};
                Vec2 m1{a.x + d.x, a.y + d.y}, m2{a.x + 2 * d.x, a.y + 2 * d.y};
                Vec2 tip{m1.x + 0.5 * d.x - f * std::sqrt(3.0) / 2 * d.y, m1.y + 0.5 * d.y + f * std::sqrt(3.0) / 2 * d.x};
                q.push_back(a); q.push_back(m1); q.push_back(tip); q.push_back(m2);
            }
            p = q;
        }
        // ensure CCW
        double A = 0; for (size_t i = 0; i < p.size(); ++i) A += p[i].x * p[(i + 1) % p.size()].y - p[(i + 1) % p.size()].x * p[i].y;
        if (A < 0) std::reverse(p.begin(), p.end());
        addPoly(W, p);
        std::vector<Poly> ps{p}; W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "koch lev=%d", lev); break; }
    case 8: {  // big disc + arms ending in knobs (polygonal, fine): knobs collapse/invert at large d
        int N = int(U(2, 6)); double Rb = U(6, 14); std::vector<Contour> parts;
        geometry::Tolerance ft = geometry::Tolerance::defaults(); ft.chordal = 1e-4;
        parts.push_back(Contour::circle({0, 0}, Rb));
        for (int m = 0; m < N; ++m) {
            double t = TAU * m / N + U(-0.2, 0.2), w = U(0.4, 2.5), L = Rb + U(3, 12), rho = U(w * 1.1, 5);
            Vec2 u{std::cos(t), std::sin(t)}, n{-u.y, u.x};
            Poly arm{{n.x * w, n.y * w}, {L * u.x + n.x * w, L * u.y + n.y * w}, {L * u.x - n.x * w, L * u.y - n.y * w}, {-n.x * w, -n.y * w}};
            std::reverse(arm.begin(), arm.end());
            double A = 0; for (size_t i = 0; i < 4; ++i) A += arm[i].x * arm[(i + 1) % 4].y - arm[(i + 1) % 4].x * arm[i].y;
            if (A < 0) std::reverse(arm.begin(), arm.end());
            parts.push_back(Contour::fromPolyline(geometry::Polyline{arm, true}));
            parts.push_back(Contour::circle({L * u.x, L * u.y}, rho));
        }
        auto u = geometry::unionContours(parts, ft);
        std::vector<Poly> ps;
        for (auto& c : u) { auto pts = c.polygonise(ft).points; ps.push_back(pts); addPoly(W, pts); }
        W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "knobs N=%d Rb=%.1f", N, Rb); break; }
    case 9: {  // comb with sharp teeth + row of triangle and sliver islands
        int M = int(U(3, 10)); double x = 0; Poly p{{0, 0}};
        std::vector<std::array<double, 3>> sl; for (int m = 0; m < M; ++m) { x += U(0.5, 3); double w = U(1, 6), dd = U(3, 15); sl.push_back({x, w, dd}); x += w; }
        double L = x + U(0.5, 3), base = U(6, 14);
        p.push_back({L, 0}); p.push_back({L, base});
        for (int m = M - 1; m >= 0; --m) { auto [sx, w, dd] = sl[m]; double tipx = sx + w / 2 + U(-w / 3, w / 3); p.push_back({sx + w, base}); p.push_back({tipx, base + dd}); p.push_back({sx, base}); }
        p.push_back({0, base});
        addPoly(W, p);
        std::vector<Poly> ps{p};
        int k = int(U(1, 6));
        for (int i = 0; i < k; ++i) {
            double cx = L * (i + 0.5) / k, cy = base / 2, s = std::min(L / k, base) * U(0.1, 0.35);
            Poly t;
            if (i % 2) t = {{cx - s, cy - s * 0.6}, {cx + s, cy - s * 0.6}, {cx + U(-s, s), cy + s}};
            else t = {{cx - s, cy - 0.05}, {cx + s, cy - 0.02}, {cx + s, cy + 0.03}, {cx - s, cy + 0.06}};
            if (U(0, 1) < 0.5) std::reverse(t.begin(), t.end());
            addPoly(W, t); ps.push_back(t);
        }
        W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "combtri M=%d isl=%d", M, k); break; }
    case 10: {  // crescent/annulus with true arcs: circle pocket with eccentric circular island
        double Ro = U(8, 20), ri = U(0.3, 0.9) * Ro, e = U(0.01, 0.98) * (Ro - ri);
        double ph = U(0, TAU); Vec2 ic{e * std::cos(ph), e * std::sin(ph)};
        W.input.push_back(Contour::circle({0, 0}, Ro)); W.input.push_back(Contour::circle(ic, ri, geometry::ArcDirection::CW));
        W.arcs.push_back({{0, 0}, Ro, 0, TAU}); W.arcs.push_back({ic, ri, 0, TAU});
        W.inside = [Ro, ri, ic](Vec2 q) { return std::hypot(q.x, q.y) < Ro && std::hypot(q.x - ic.x, q.y - ic.y) > ri; };
        snprintf(buf, sizeof buf, "crescent-arc Ro=%.2f ri=%.2f e=%.3f", Ro, ri, e); break; }
    case 12: case 13: case 14: {  // disc unions resolved by Clipper and polygonised coarsely (spike-prone)
        std::vector<Contour> discs; int M;
        if (kind == 12) { M = int(U(3, 8)); double x = 0, y = 0, rp = U(1.5, 6); discs.push_back(Contour::circle({0, 0}, rp));
            for (int m = 1; m < M; ++m) { double r = U(1.5, 6); x += U(0.5, 0.9) * (r + rp); y += U(-2, 2); discs.push_back(Contour::circle({x, y}, r)); rp = r; } }
        else if (kind == 13) { M = int(U(4, 14)); double S = U(10, 25); for (int m = 0; m < M; ++m) discs.push_back(Contour::circle({U(0, S), U(0, S)}, U(1.5, 6))); }
        else { M = int(U(6, 14)); double Rr = U(8, 18); for (int m = 0; m < M; ++m) { double t = TAU * m / M + U(-0.1, 0.1); discs.push_back(Contour::circle({Rr * std::cos(t), Rr * std::sin(t)}, Rr * std::sin(PI / M) * U(1.05, 1.6))); } }
        geometry::Tolerance t = geometry::Tolerance::defaults(); t.chordal = U(0.0005, 0.02);
        auto u = geometry::unionContours(discs, t);
        std::vector<Poly> ps;
        for (auto& c : u) { auto pts = c.polygonise(t).points; ps.push_back(pts); addPoly(W, pts); }
        W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "discs-clipper k%d M=%d t=%.4f", kind, M, t.chordal); break; }
    case 15: case 16: {  // box whose walls carry fine inward saw teeth (many arc junctions at shallow angles)
        double L = U(10, 30), H = U(6, 20); Poly p;
        double sp0 = kind == 15 ? 0.05 : 0.3, sp1 = kind == 15 ? 1.0 : 3.0, h0 = 0.005, h1 = kind == 15 ? 0.3 : 1.5;
        auto edge = [&](Vec2 a, Vec2 b) { double len = std::hypot(b.x - a.x, b.y - a.y); Vec2 u{(b.x - a.x) / len, (b.y - a.y) / len}, n{-u.y, u.x};
            for (double t = 0; t < len - sp1;) { double s1 = U(sp0, sp1); p.push_back({a.x + u.x * t, a.y + u.y * t}); double hh = U(h0, h1); double tt = t + s1 * U(0.2, 0.8); p.push_back({a.x + u.x * tt + n.x * hh, a.y + u.y * tt + n.y * hh}); t += s1; } };
        edge({0, 0}, {L, 0}); edge({L, 0}, {L, H}); edge({L, H}, {0, H}); edge({0, H}, {0, 0});
        addPoly(W, p); std::vector<Poly> ps{p}; W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "sawbox k%d n=%zu", kind, p.size()); break; }
    case 11: {  // fixture polygon from a file (env POLY)
        Poly p; FILE* f = fopen(getenv("POLY"), "r"); double x, y; while (fscanf(f, "%lf %lf", &x, &y) == 2) p.push_back({x, y}); fclose(f);
        addPoly(W, p); std::vector<Poly> ps{p}; W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        snprintf(buf, sizeof buf, "file"); break; }
    }
    W.name = buf;
    if (getenv("PD") && !W.arcs.empty()) {  // polygonise the arc walls; the polygon becomes the exact wall
        geometry::Tolerance pt = geometry::Tolerance::defaults(); pt.chordal = atof(getenv("PD"));
        std::vector<Contour> in2; std::vector<Poly> ps; W.segs.clear(); W.arcs.clear();
        for (auto& c : W.input) { auto pts = c.polygonise(pt).points; ps.push_back(pts); }
        W.input.clear();
        for (auto& p : ps) addPoly(W, p);
        W.inside = [ps](Vec2 q) { return insideEO(ps, q); };
        W.name += " PD=" + std::string(getenv("PD"));
    }
    W.build();
    return true;
}

static double ringArea(const Poly& p) { double A = 0; for (size_t i = 0; i < p.size(); ++i) A += p[i].x * p[(i + 1) % p.size()].y - p[(i + 1) % p.size()].x * p[i].y; return A / 2; }

int main(int argc, char** argv) {
    int kind = atoi(argv[1]); uint64_t seed = strtoull(argv[2], 0, 10); int nd = atoi(argv[3]);
    double chordal = argc > 4 ? atof(argv[4]) : 0.01, allowance = argc > 5 ? atof(argv[5]) : 0.0;
    double dlo = getenv("DLO") ? atof(getenv("DLO")) : -1, dhi = getenv("DHI") ? atof(getenv("DHI")) : -1;
    bool verbose = getenv("VERB") != nullptr, checkRaw = getenv("NORAW") == nullptr;
    World W;
    if (!makeWorld(kind, seed, W)) { printf("kind %d seed %llu degenerate\n", kind, (unsigned long long)seed); return 0; }
    double ext = std::max(W.x1 - W.x0, W.y1 - W.y0);
    double h = ext / 400; int nx = int((W.x1 - W.x0) / h) + 2, ny = int((W.y1 - W.y0) / h) + 2;
    std::vector<double> sd(size_t(nx) * ny);
    double maxsd = 0;
    for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
        Vec2 q{W.x0 + i * h, W.y0 + j * h};
        double d = W.ptDist(q); if (!W.inside(q)) d = -d;
        sd[size_t(j) * nx + i] = d; maxsd = std::max(maxsd, d);
    }
    toolpath::PocketParams P; P.tolerance.chordal = chordal; P.tolerance.distance = std::min(P.tolerance.distance, chordal); P.allowance = allowance;
    geometry::Tolerance rt = P.tolerance; rt.chordal = 0.5 * chordal;
    std::vector<Contour> region;
    if (checkRaw) region = geometry::unionContours(W.input, rt, geometry::FillRule::EvenOdd);
    std::mt19937_64 R(seed ^ 0x51ed270b0b0b0b0bULL ^ uint64_t(kind) << 40);
    long msgs = 0, lostAny = 0, lostBig = 0, gouge = 0, marginEat = 0, outsideV = 0, selfX = 0, notSubset = 0, empties = 0;
    double worstLost = -1, worstGouge = 1e9, worstMargin = 1e9, worstNotSub = 0;
    std::string sLost, sGouge, sMsg, sX, sSub, sMargin;
    for (int k = 0; k < nd; ++k) {
        double dmax = 2 * (maxsd - allowance - chordal) + 0.05;
        double lo = dlo > 0 ? dlo : 0.1, hi = dhi > 0 ? dhi : dmax;
        double d = lo + (hi - lo) * (k + std::uniform_real_distribution<double>(0, 1)(R)) / nd;
        if (getenv("DEXACT")) d = atof(getenv("DEXACT"));
        double r = d / 2, Rr = r + allowance + chordal;
        std::vector<std::string> ms;
#ifdef USE_PLAN
        std::vector<Contour> C;
        { toolpath::PocketTool T; T.tool.name = "T"; T.tool.diameter = d; T.stepover = d; T.passDepth = 1; toolpath::PocketParams Q = P; Q.cutDepth = 1; Q.strategy = toolpath::PocketStrategy::Raster; Q.profilePass = toolpath::ProfilePass::None;
          auto res = toolpath::pocketToolpath(W.input, {T}, Q); C = res.plans[0].centreRegion; for (auto& mm : res.messages) if (mm.find("too large") == std::string::npos) ms.push_back(mm); }
#else
        auto C = toolpath::pocketCentreRegion(W.input, d, P, &ms);
#endif
        if (!ms.empty()) { ++msgs; if (sMsg.empty()) sMsg = "d=" + std::to_string(d) + " " + ms[0]; }
        std::vector<Poly> cp;
        for (auto& c : C) cp.push_back(c.polygonise(geometry::Tolerance::defaults()).points);
        if (C.empty()) ++empties;
        // (i) exact edge distances
        for (auto& ring : cp) {
            for (size_t i = 0; i < ring.size(); ++i) {
                Vec2 a = ring[i], b = ring[(i + 1) % ring.size()];
                if (!W.inside(a)) { ++outsideV; }
                double dd = W.segDist(a, b, Rr + 1);
                double ex = dd - (r + allowance);
                if (ex < -1e-7) { ++gouge; if (ex < worstGouge) { worstGouge = ex; char bb[300]; snprintf(bb, sizeof bb, "d=%.12g edge (%.6f,%.6f)-(%.6f,%.6f) dist-(r+allow) %.3e", d, a.x, a.y, b.x, b.y, ex); sGouge = bb; } }
                if (dd - Rr < worstMargin) { worstMargin = dd - Rr; char bb[300]; snprintf(bb, sizeof bb, "d=%.12g edge (%.6f,%.6f)-(%.6f,%.6f)", d, a.x, a.y, b.x, b.y); sMargin = bb; }
                if (dd < Rr - (0.5 * chordal + 2e-6)) ++marginEat;
            }
        }
        // (iv) self-intersections (all edges of C, all rings)
        {
            std::vector<std::pair<Vec2, Vec2>> E;
            for (auto& ring : cp) for (size_t i = 0; i < ring.size(); ++i) E.push_back({ring[i], ring[(i + 1) % ring.size()]});
            std::vector<int> ord(E.size());
            for (size_t i = 0; i < E.size(); ++i) ord[i] = int(i);
            std::sort(ord.begin(), ord.end(), [&](int a, int b) { return std::min(E[a].first.x, E[a].second.x) < std::min(E[b].first.x, E[b].second.x); });
            long xs = 0;
            for (size_t ii = 0; ii < ord.size(); ++ii) {
                auto& e = E[ord[ii]]; double mx = std::max(e.first.x, e.second.x);
                for (size_t jj = ii + 1; jj < ord.size(); ++jj) {
                    auto& f = E[ord[jj]]; if (std::min(f.first.x, f.second.x) > mx) break;
                    if (properCross(e.first, e.second, f.first, f.second)) ++xs;
                }
            }
            if (xs && getenv("VERB")) for (size_t ii = 0; ii < E.size(); ++ii) for (size_t jj = ii + 1; jj < E.size(); ++jj) if (properCross(E[ii].first, E[ii].second, E[jj].first, E[jj].second)) printf("  X (%.6f,%.6f)-(%.6f,%.6f) x (%.6f,%.6f)-(%.6f,%.6f)\n", E[ii].first.x, E[ii].first.y, E[ii].second.x, E[ii].second.y, E[jj].first.x, E[jj].first.y, E[jj].second.x, E[jj].second.y);
            if (xs) { ++selfX; if (sX.empty()) sX = "d=" + std::to_string(d) + " crossings=" + std::to_string(xs); }
        }
        // (v) C subset of raw Clipper inset
        if (checkRaw && !C.empty()) {
            auto raw = geometry::offsetContours(region, -(r + allowance + chordal), geometry::JoinType::Round, rt);
            auto diff = geometry::booleanOp(geometry::BooleanOp::Difference, C, raw, geometry::Tolerance::defaults());
            double A = 0; for (auto& c : diff.closed) A += ringArea(c.polygonise(geometry::Tolerance::defaults()).points);
            if (A > 1e-6) { ++notSubset; if (A > worstNotSub) { worstNotSub = A; sSub = "d=" + std::to_string(d) + " area " + std::to_string(A); } }
        }
        // (ii) lost reachable points
        double ex = -1; Vec2 at{};
        std::vector<std::vector<double>> xsr(ny);
        for (auto& ring : cp) for (size_t i = 0, n = ring.size(), jj = n - 1; i < n; jj = i++) {
            Vec2 a = ring[i], b = ring[jj];
            if (a.y == b.y) continue;
            int j0 = std::max(0, int(std::ceil((std::min(a.y, b.y) - W.y0) / h)) - 1), j1 = std::min(ny - 1, int(std::floor((std::max(a.y, b.y) - W.y0) / h)) + 1);
            for (int j = j0; j <= j1; ++j) { double y = W.y0 + j * h; if ((a.y > y) != (b.y > y)) xsr[j].push_back((b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x); }
        }
        for (int j = 0; j < ny; ++j) {
            std::sort(xsr[j].begin(), xsr[j].end()); size_t ptr = 0; bool in = false;
            for (int i = 0; i < nx; ++i) {
                double x = W.x0 + i * h;
                while (ptr < xsr[j].size() && xsr[j][ptr] < x) { in = !in; ++ptr; }
                double v = sd[size_t(j) * nx + i];
                if (in || v < Rr + chordal + 1e-3) continue;
                if (v - Rr > ex) { ex = v - Rr; at = {x, W.y0 + j * h}; }
            }
        }
        if (ex > 0) { ++lostAny; if (ex > 0.05) ++lostBig; if (ex > worstLost) { worstLost = ex; char bb[300]; snprintf(bb, sizeof bb, "d=%.12g at (%.4f,%.4f) excess %.5f msgs=%zu", d, at.x, at.y, ex, ms.size()); sLost = bb; } }
        if (verbose && (ex > 0 || !ms.empty())) printf("  d=%.12g lostEx %.4g msgs %zu\n", d, ex, ms.size());
    }
    printf("kind %d seed %llu [%s] ch %g al %g: %d diam | LOST %ld (>0.05 %ld) %s | GOUGE %ld worst %.3e %s | marginEat %ld worstMargin %.3e %s | outsideV %ld | SELFX %ld %s | NOTSUB %ld %s | msgs %ld %s | empty %ld\n",
           kind, (unsigned long long)seed, W.name.c_str(), chordal, allowance, nd, lostAny, lostBig, sLost.c_str(), gouge, gouge ? worstGouge : 0.0, sGouge.c_str(), marginEat, worstMargin,
           sMargin.c_str(), outsideV, selfX, sX.c_str(), notSubset, sSub.c_str(), msgs, sMsg.c_str(), empties);
    return 0;
}
