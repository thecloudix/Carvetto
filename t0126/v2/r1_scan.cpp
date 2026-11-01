// T-0125 verifier: diameter scans with an exact oracle (polygonal inputs -> exact segment distances).
// Throwaway; independent of pocket_oracle.hpp.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"

using namespace carvetto;
using geometry::Contour;
using geometry::Vec2;
using Poly = std::vector<Vec2>;
static const double PI = 3.14159265358979323846;

// ---------------------------------------------------------------- exact distance
static double segPt(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x, dy = b.y - a.y, L = dx * dx + dy * dy;
    double t = L > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / L : 0;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
static double orient(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
static double segSeg(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    double o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    if (((o1 > 0) != (o2 > 0)) && ((o3 > 0) != (o4 > 0)) && o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0) return 0;
    return std::min({segPt(a, c, d), segPt(b, c, d), segPt(c, a, b), segPt(d, a, b)});
}
struct Edges {
    std::vector<std::pair<Vec2, Vec2>> e;
    double x0, y0, cell;
    int nx, ny;
    std::vector<std::vector<int>> b;
    void build(const std::vector<Poly>& ps, double c) {
        x0 = y0 = 1e300;
        double x1 = -1e300, y1 = -1e300;
        for (auto& p : ps)
            for (size_t i = 0; i < p.size(); ++i) {
                e.push_back({p[i], p[(i + 1) % p.size()]});
                x0 = std::min(x0, p[i].x); y0 = std::min(y0, p[i].y); x1 = std::max(x1, p[i].x); y1 = std::max(y1, p[i].y);
            }
        cell = c;
        nx = int((x1 - x0) / cell) + 1; ny = int((y1 - y0) / cell) + 1;
        b.assign(size_t(nx) * ny, {});
        for (int k = 0; k < (int)e.size(); ++k) {
            auto [p, q] = e[k];
            int i0 = ci(std::min(p.x, q.x)), i1 = ci(std::max(p.x, q.x)), j0 = cj(std::min(p.y, q.y)), j1 = cj(std::max(p.y, q.y));
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) b[size_t(j) * nx + i].push_back(k);
        }
    }
    int ci(double x) const { return std::clamp(int(std::floor((x - x0) / cell)), 0, nx - 1); }
    int cj(double y) const { return std::clamp(int(std::floor((y - y0) / cell)), 0, ny - 1); }
    // exact min distance from segment (a,b) to edges, assuming answer < R (else returns >= R)
    double segDist(Vec2 a, Vec2 bb, double R) const {
        double best = R;
        int i0 = ci(std::min(a.x, bb.x) - R), i1 = ci(std::max(a.x, bb.x) + R), j0 = cj(std::min(a.y, bb.y) - R), j1 = cj(std::max(a.y, bb.y) + R);
        static thread_local std::vector<int> seen; static thread_local int stamp = 0;
        if (seen.size() < e.size()) seen.assign(e.size(), 0);
        ++stamp;
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                for (int k : b[size_t(j) * nx + i]) {
                    if (seen[k] == stamp) continue;
                    seen[k] = stamp;
                    best = std::min(best, segSeg(a, bb, e[k].first, e[k].second));
                }
        return best;
    }
    double ptDist(Vec2 p) const {  // exact, expanding search
        for (double R = cell; ; R *= 2) {
            double d = segDist(p, p, R);
            if (d < R) return d;
            if (R > 1e6) return d;
        }
    }
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

// ---------------------------------------------------------------- shapes
struct Shape { std::string name; std::vector<Poly> polys; };
static Poly polar(int n, auto f) { Poly p; for (int i = 0; i < n; ++i) { double t = 2 * PI * i / n, r = f(t); p.push_back({r * std::cos(t), r * std::sin(t)}); } return p; }
static Poly circ(Vec2 c, double r, int n, bool cw = false) { Poly p; for (int i = 0; i < n; ++i) { double t = 2 * PI * i / n * (cw ? -1 : 1); p.push_back({c.x + r * std::cos(t), c.y + r * std::sin(t)}); } return p; }

static Shape makeShape(int kind, uint64_t seed) {
    std::mt19937_64 R(seed * 7919 + kind);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(R); };
    Shape s;
    char buf[160];
    switch (kind) {
    case 0: {  // gear
        int N = int(U(5, 14)); double a = U(0.15, 0.4), k = U(1, 6), ph = U(0, 6.28), R0 = U(10, 20); int n = int(U(600, 1500));
        s.polys.push_back(polar(n, [&](double t) { return R0 * (1 + a * std::tanh(k * std::cos(N * t + ph)) / std::tanh(k)); }));
        snprintf(buf, sizeof buf, "gear N=%d a=%.3f k=%.2f n=%d", N, a, k, n); break; }
    case 1: {  // harmonic blob
        double R0 = U(10, 20); double am[8], ph[8]; double tot = 0;
        for (int i = 2; i < 8; ++i) { am[i] = U(0, 0.25); ph[i] = U(0, 6.28); tot += am[i]; }
        for (int i = 2; i < 8; ++i) am[i] *= std::min(1.0, 0.7 / tot);
        int n = int(U(300, 1200));
        s.polys.push_back(polar(n, [&](double t) { double r = 1; for (int i = 2; i < 8; ++i) r += am[i] * std::cos(i * t + ph[i]); return R0 * r; }));
        snprintf(buf, sizeof buf, "blob n=%d", n); break; }
    case 2: {  // slotted comb, rounded slot ends (many vertices on near-wall arcs)
        int M = int(U(3, 9)); double x = 0, base = U(3, 8); Poly p; p.push_back({0, 0});
        int seg = int(U(16, 80));
        std::vector<double> ws;
        for (int m = 0; m < M; ++m) {
            double gap = U(0.8, 4), w = U(1, 7), dep = U(4, 20);
            x += gap; ws.push_back(w);
            // slot from (x, base) up to depth with semicircle end; walk along top edge left->right? We build CCW outline:
            // bottom edge (0,0)->(L,0), right side up, top edge right->left including slots going up. Simpler: store slots, build later
            (void)dep; x += w;
        }
        // rebuild deterministically
        std::mt19937_64 R2(seed * 31 + 5);
        auto U2 = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(R2); };
        struct Sl { double x, w, d; }; std::vector<Sl> sl; x = 0;
        for (int m = 0; m < M; ++m) { double gap = U2(0.8, 4), w = ws[m], d = U2(4, 20); x += gap; sl.push_back({x, w, d}); x += w; }
        double L = x + U2(0.8, 4);
        p.push_back({L, 0}); p.push_back({L, base});
        for (int m = M - 1; m >= 0; --m) {
            const Sl& q = sl[m];
            p.push_back({q.x + q.w, base});
            p.push_back({q.x + q.w, base + q.d});
            for (int i = 1; i < seg; ++i) { double t = PI * i / seg; p.push_back({q.x + q.w / 2 + q.w / 2 * std::cos(t), base + q.d + q.w / 2 * std::sin(t)}); }
            p.push_back({q.x, base + q.d});
            p.push_back({q.x, base});
        }
        p.push_back({0, base});
        s.polys.push_back(p);
        snprintf(buf, sizeof buf, "comb M=%d seg=%d", M, seg); break; }
    case 3: {  // disc chain union (cusps)
        std::vector<Contour> discs; double x = 0;
        int M = int(U(3, 7));
        for (int m = 0; m < M; ++m) { double r = U(1.5, 6); if (m) x += U(0.6, 0.95) * (r + U(1.5, 6) * 0.0) + r * U(0.5, 0.9); discs.push_back(Contour::circle({x, U(-2, 2)}, r)); }
        geometry::Tolerance t = geometry::Tolerance::defaults(); t.chordal = U(0.0005, 0.01);
        auto u = geometry::unionContours(discs, t);
        size_t best = 0; double ba = -1;
        for (size_t i = 0; i < u.size(); ++i) { auto b = u[i].bounds(); double a = b.width() * b.height(); if (a > ba) { ba = a; best = i; } }
        s.polys.push_back(u[best].polygonise(t).points);
        snprintf(buf, sizeof buf, "discchain M=%d", M); break; }
    case 4: {  // irregular sharp star
        int K = int(U(5, 16)); Poly p;
        for (int i = 0; i < 2 * K; ++i) { double t = PI * i / K + U(-0.1, 0.1) / K, r = i % 2 == 0 ? U(10, 20) : U(3, 8); p.push_back({r * std::cos(t), r * std::sin(t)}); }
        s.polys.push_back(p);
        snprintf(buf, sizeof buf, "star K=%d", K); break; }
    case 5: {  // crescent: circle with eccentric hole
        double Ro = U(8, 15), ri = U(0.5, 0.92) * Ro, e = U(0.02, 0.98) * (Ro - ri);
        int n = int(U(200, 1000));
        s.polys.push_back(circ({0, 0}, Ro, n)); s.polys.push_back(circ({e, U(-0.01, 0.01)}, ri, int(n * ri / Ro) + 16, true));
        snprintf(buf, sizeof buf, "crescent Ro=%.2f ri=%.2f e=%.3f", Ro, ri, e); break; }
    case 6: {  // plate with circular islands
        double W = U(30, 50), H = U(20, 35);
        s.polys.push_back({{0, 0}, {W, 0}, {W, H}, {0, H}});
        std::vector<std::pair<Vec2, double>> isl;
        for (int tries = 0; tries < 300 && isl.size() < 12; ++tries) {
            double r = U(1, 6); Vec2 c{U(r + 0.5, W - r - 0.5), U(r + 0.5, H - r - 0.5)};
            bool ok = true; for (auto& [cc, rr] : isl) if (std::hypot(c.x - cc.x, c.y - cc.y) < r + rr + 0.3) ok = false;
            if (ok) isl.push_back({c, r});
        }
        int seg = int(U(24, 160));
        for (auto& [c, r] : isl) s.polys.push_back(circ(c, r, seg));
        snprintf(buf, sizeof buf, "plate+%zu islands", isl.size()); break; }
    case 7: {  // spiral strip of varying width
        Poly in, out; double a = U(1.5, 3), w0 = U(1, 4), wa = U(0.2, 0.9); int n = int(U(400, 1000)); double turns = U(1.5, 3);
        for (int i = 0; i <= n; ++i) { double t = 2 * PI * turns * i / n + 1.0, r = a * t, w = w0 * (1 + wa * std::sin(3 * t)); out.push_back({(r + w / 2) * std::cos(t), (r + w / 2) * std::sin(t)}); in.push_back({(r - w / 2) * std::cos(t), (r - w / 2) * std::sin(t)}); }
        Poly p = out; for (int i = n; i >= 0; --i) p.push_back(in[i]);
        s.polys.push_back(p);
        snprintf(buf, sizeof buf, "spiral n=%d", n); break; }
    case 8: {  // flower with many petals partially collapsing (near-wall arcs), inverted lobes
        int N = int(U(3, 9)); double a = U(0.3, 0.8), R0 = U(8, 20); int n = int(U(720, 2000));
        s.polys.push_back(polar(n, [&](double t) { return R0 * (1 + a * std::cos(N * t)) + 0.5; }));
        snprintf(buf, sizeof buf, "flower N=%d a=%.2f", N, a); break; }
    case 9: {  // nested: pocket inside island inside pocket, all lobed
        double R0 = U(15, 22);
        s.polys.push_back(polar(800, [&](double t) { return R0 * (1 + 0.2 * std::cos(5 * t)); }));
        s.polys.push_back(polar(500, [&](double t) { return R0 * 0.55 * (1 + 0.25 * std::cos(4 * t + 0.4)); }));
        s.polys.push_back(polar(300, [&](double t) { return R0 * 0.3 * (1 + 0.3 * std::cos(3 * t)); }));
        snprintf(buf, sizeof buf, "nested R0=%.1f", R0); break; }
    case 10: {  // round-3 gap polygon
        Poly p; FILE* f = fopen("/tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0121v3/gap_poly.txt", "r"); double x, y; while (fscanf(f, "%lf %lf", &x, &y) == 2) p.push_back({x, y}); fclose(f);
        s.polys.push_back(p); snprintf(buf, sizeof buf, "gap_poly"); break; }
    }
    s.name = buf;
    return s;
}

// ---------------------------------------------------------------- scan
int main(int argc, char** argv) {
    int kind = atoi(argv[1]); uint64_t seed = strtoull(argv[2], 0, 10); int nd = atoi(argv[3]);
    double dlo = getenv("DLO") ? atof(getenv("DLO")) : -1, dhi = getenv("DHI") ? atof(getenv("DHI")) : -1;
    double chordal = argc > 4 ? atof(argv[4]) : 0.01, allowance = argc > 5 ? atof(argv[5]) : 0.0, over = argc > 6 ? atof(argv[6]) : 1.0;
    Shape s = makeShape(kind, seed);
    std::vector<Contour> input;
    for (auto& p : s.polys) input.push_back(Contour::fromPolyline(geometry::Polyline{p, true}));
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (auto& p : s.polys) for (auto v : p) { x0 = std::min(x0, v.x); y0 = std::min(y0, v.y); x1 = std::max(x1, v.x); y1 = std::max(y1, v.y); }
    double ext = std::max(x1 - x0, y1 - y0);
    Edges E; E.build(s.polys, ext / 128);
    // signed-distance grid
    double h = ext / 500; int nx = int((x1 - x0) / h) + 2, ny = int((y1 - y0) / h) + 2;
    std::vector<double> sd(size_t(nx) * ny);
    double maxsd = 0;
    for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
        Vec2 q{x0 + i * h, y0 + j * h};
        double d = E.ptDist(q); if (!insideEO(s.polys, q)) d = -d;
        sd[size_t(j) * nx + i] = d; maxsd = std::max(maxsd, d);
    }
    toolpath::PocketParams P; P.tolerance.chordal = chordal; P.tolerance.distance = std::min(P.tolerance.distance, chordal); P.allowance = allowance;
    const double m = chordal;
    std::mt19937_64 R(seed ^ 0x9e3779b97f4a7c15ULL);
    long msgs = 0, lostAny = 0, lost002 = 0, lost005 = 0, gouge = 0, marginEat = 0, empties = 0, outsideV = 0;
    double worstLostExcess = -1, worstGouge = 1e9, worstMargin = 1e9;
    std::string firstLost, firstGouge, firstMsg;
    for (int k = 0; k < nd; ++k) {
        double dmax = (2 * (maxsd - allowance - m) + 0.05) * over;
        double lo = dlo > 0 ? dlo : 0.1, hi = dhi > 0 ? dhi : dmax;
        double d = lo + (hi - lo) * (k + std::uniform_real_distribution<double>(0, 1)(R)) / nd;
        double r = d / 2, Rr = r + allowance + m;
        std::vector<std::string> ms;
#ifdef USE_PLAN
        std::vector<Contour> C;
        { toolpath::PocketTool T; T.tool.name = "T"; T.tool.diameter = d; T.stepover = d; T.passDepth = 1; toolpath::PocketParams Q = P; Q.cutDepth = 1; Q.strategy = toolpath::PocketStrategy::Raster; Q.profilePass = toolpath::ProfilePass::None;
          auto res = toolpath::pocketToolpath(input, {T}, Q); C = res.plans[0].centreRegion; for (auto& mm : res.messages) if (mm.find("too large") == std::string::npos) ms.push_back(mm); }
#else
        auto C = toolpath::pocketCentreRegion(input, d, P, &ms);
#endif
        if (!ms.empty()) { ++msgs; if (firstMsg.empty()) firstMsg = "d=" + std::to_string(d) + " " + ms[0]; }
        std::vector<Poly> cp;
        for (auto& c : C) cp.push_back(c.polygonise(geometry::Tolerance::defaults()).points);
        if (C.empty()) ++empties;
        // exact edge check: every centre-region edge >= r + allowance from walls; margin kept within budget
        for (auto& ring : cp) {
            if (!ring.empty() && !insideEO(s.polys, ring[0])) ++outsideV;
            for (size_t i = 0; i < ring.size(); ++i) {
                double dd = E.segDist(ring[i], ring[(i + 1) % ring.size()], Rr + 1);
                if (dd < r + allowance - 1e-9) { ++gouge; if (dd - r - allowance < worstGouge) { worstGouge = dd - r - allowance; char b[200]; snprintf(b, sizeof b, "d=%.10g edge (%.5f,%.5f) dist %.7f < r+allow %.7f", d, ring[i].x, ring[i].y, dd, r + allowance); firstGouge = b; } }
                worstMargin = std::min(worstMargin, dd - Rr);
                if (dd < Rr - (0.5 * chordal + 2e-6)) ++marginEat; if (dd < Rr - (0.5 * chordal + 2e-6) && getenv("VERB")) printf("  MARGIN d=%.12g edge (%.5f,%.5f)-(%.5f,%.5f) dist-Rr %.4e\n", d, ring[i].x, ring[i].y, ring[(i+1)%ring.size()].x, ring[(i+1)%ring.size()].y, dd - Rr);
            }
        }
        // lost cells: grid points with sd >= Rr + delta not inside C
        double ex = -1; Vec2 at{};
        std::vector<std::vector<double>> xs(ny);
        for (auto& ring : cp) for (size_t i = 0, n = ring.size(), jj = n - 1; i < n; jj = i++) {
            Vec2 a = ring[i], b = ring[jj];
            if (a.y == b.y) continue;
            double lo = std::min(a.y, b.y), hi = std::max(a.y, b.y);
            int j0 = std::max(0, int(std::ceil((lo - y0) / h)) - 1), j1 = std::min(ny - 1, int(std::floor((hi - y0) / h)) + 1);
            for (int j = j0; j <= j1; ++j) { double y = y0 + j * h; if ((a.y > y) != (b.y > y)) xs[j].push_back((b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x); }
        }
        for (int j = 0; j < ny; ++j) { std::sort(xs[j].begin(), xs[j].end()); size_t ptr = 0; bool in = false;
          for (int i = 0; i < nx; ++i) {
            double x = x0 + i * h;
            while (ptr < xs[j].size() && xs[j][ptr] < x) { in = !in; ++ptr; }
            double v = sd[size_t(j) * nx + i];
            if (v < Rr + 1e-3) continue;
            Vec2 q{x, y0 + j * h};
            if (in) continue;
            // exact recheck
            if (v - Rr > ex) { ex = v - Rr; at = q; }
        } }
        if (ex > 0) {
            ++lostAny; if (ex > 0.002) ++lost002; if (ex > 0.005) ++lost005;
            if (ex > worstLostExcess) { worstLostExcess = ex; char b[200]; snprintf(b, sizeof b, "d=%.12g at (%.4f,%.4f) depth excess %.5f msgs=%zu", d, at.x, at.y, ex, ms.size()); firstLost = b; }
        }
    }
    printf("kind %d seed %llu [%s] chordal %g allow %g: %d diam | lost>1e-3 %ld (>2e-3 %ld, >5e-3 %ld) worst %s | gougeEdges %ld %s | marginEat %ld worstMargin %.2e | outside %ld | msgs %ld %s\n",
           kind, (unsigned long long)seed, s.name.c_str(), chordal, allowance, nd, lostAny, lost002, lost005, firstLost.c_str(), gouge, firstGouge.c_str(), marginEat, worstMargin,
           outsideV, msgs, firstMsg.c_str());
    return 0;
}
