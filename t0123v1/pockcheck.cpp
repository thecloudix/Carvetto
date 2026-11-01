// T-0123 verifier (throwaway): random polygonal pockets, every option, multi-tool.
// Hash output (compare with main) and check safety independently: gouge (exact segment distance to
// polygon walls), engagement (no move engages more than one level of material: links travel over
// cleared floor), floor coverage of everything the smallest tool can reach, retracts vs parts.
// Build with -DNEWAPI for the T-0123 API.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;
using Poly = std::vector<Vec2>;
static const double PI = 3.14159265358979323846, TAU = 2 * PI;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

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
static bool insideEO(const std::vector<Poly>& ps, Vec2 p) {
    bool in = false;
    for (auto& r : ps)
        for (size_t i = 0, n = r.size(), j = n - 1; i < n; j = i++) {
            Vec2 a = r[i], b = r[j];
            if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
        }
    return in;
}
static double area(const Poly& p) { double A = 0; for (size_t i = 0; i < p.size(); ++i) A += p[i].x * p[(i + 1) % p.size()].y - p[(i + 1) % p.size()].x * p[i].y; return A / 2; }

struct World {
    std::vector<Poly> polys;
    std::vector<std::pair<Vec2, Vec2>> segs;
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    double gx0, gy0, cell; int nx = 1, ny = 1; std::vector<std::vector<int>> b;
    int ci(double x) const { return std::clamp(int(std::floor((x - gx0) / cell)), 0, nx - 1); }
    int cj(double y) const { return std::clamp(int(std::floor((y - gy0) / cell)), 0, ny - 1); }
    void add(Poly p) { polys.push_back(p); }
    void build() {
        for (auto& p : polys) for (size_t i = 0; i < p.size(); ++i) { segs.push_back({p[i], p[(i + 1) % p.size()]}); x0 = std::min(x0, p[i].x); y0 = std::min(y0, p[i].y); x1 = std::max(x1, p[i].x); y1 = std::max(y1, p[i].y); }
        gx0 = x0; gy0 = y0; cell = std::max(x1 - x0, y1 - y0) / 128;
        nx = int((x1 - x0) / cell) + 1; ny = int((y1 - y0) / cell) + 1;
        b.assign(size_t(nx) * ny, {});
        for (int k = 0; k < (int)segs.size(); ++k) {
            auto [p, q] = segs[k];
            for (int j = cj(std::min(p.y, q.y)); j <= cj(std::max(p.y, q.y)); ++j)
                for (int i = ci(std::min(p.x, q.x)); i <= ci(std::max(p.x, q.x)); ++i) b[size_t(j) * nx + i].push_back(k);
        }
    }
    double segDist(Vec2 a, Vec2 bb, double R) const {
        double best = R;
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

static Poly rectP(double ax, double ay, double bx, double by) { return {{ax, ay}, {bx, ay}, {bx, by}, {ax, by}}; }
static Poly ngon(Vec2 c, double r, int n, double ph = 0) { Poly p; for (int i = 0; i < n; ++i) { double t = ph + TAU * i / n; p.push_back({c.x + r * std::cos(t), c.y + r * std::sin(t)}); } return p; }
static bool farFrom(const std::vector<std::pair<Vec2, double>>& used, Vec2 c, double r, double gap) { for (auto& u : used) if (std::hypot(u.first.x - c.x, u.first.y - c.y) < u.second + r + gap) return false; return true; }

static void makeScene(int kind, World& W, std::string& name) {
    char buf[200];
    switch (kind) {
    case 0: {  // rect + islands
        double Wd = U(20, 60), H = U(15, 45); W.add(rectP(0, 0, Wd, H));
        std::vector<std::pair<Vec2, double>> used; int n = I(0, 7);
        for (int t = 0; t < 300 && (int)used.size() < n; ++t) {
            double r = U(0.8, 6); Vec2 c{U(r + 0.3, Wd - r - 0.3), U(r + 0.3, H - r - 0.3)};
            if (!farFrom(used, c, r, U(0.2, 3))) continue;
            used.push_back({c, r}); int s = I(0, 2);
            W.add(s == 0 ? ngon(c, r, I(3, 24), U(0, TAU)) : s == 1 ? rectP(c.x - r, c.y - r * 0.5, c.x + r, c.y + r * 0.5) : ngon(c, r, 4, U(0, TAU)));
        }
        snprintf(buf, sizeof buf, "rect+%zu islands", used.size()); break; }
    case 1: {  // comb: base + gaps
        int M = I(3, 25); double x = 0, base = U(4, 12); Poly p{{0, 0}};
        std::vector<std::array<double, 3>> sl; for (int m = 0; m < M; ++m) { x += U(0.3, 2.5); double w = U(0.3, 3.5), dd = U(2, 15); sl.push_back({x, w, dd}); x += w; }
        double L = x + U(0.3, 2.5); p.push_back({L, 0}); p.push_back({L, base});
        for (int m = M - 1; m >= 0; --m) { auto [sx, w, dd] = sl[m]; p.push_back({sx + w, base}); if (I(0, 1)) { p.push_back({sx + w, base + dd}); p.push_back({sx, base + dd}); } else p.push_back({sx + w / 2 + U(-w / 3, w / 3), base + dd}); p.push_back({sx, base}); }
        p.push_back({0, base}); W.add(p);
        snprintf(buf, sizeof buf, "comb M=%d", M); break; }
    case 2: {  // disjoint rooms, some with islands
        int n = I(2, 4); double x = 0;
        for (int i = 0; i < n; ++i) { double w = U(5, 20), h = U(5, 20); W.add(rectP(x, 0, x + w, h)); if (I(0, 1) && w > 6 && h > 6) W.add(ngon({x + w / 2, h / 2}, std::min(w, h) * U(0.1, 0.3), I(3, 12))); x += w + U(0.5, 10); }
        snprintf(buf, sizeof buf, "rooms n=%d", n); break; }
    case 3: {  // spiky star
        int K = I(10, 80); Poly p;
        for (int i = 0; i < K; ++i) { double t = TAU * (i + U(-0.3, 0.3)) / K, r = (i % 2 ? U(3, 12) : U(10, 22)); p.push_back({r * std::cos(t), r * std::sin(t)}); }
        W.add(p); snprintf(buf, sizeof buf, "star K=%d", K); break; }
    case 4: {  // sawtooth channel
        int T = I(4, 14); double w = U(1.5, 8), amp = U(1, 6), pitch = U(2, 7); Poly lo, hi;
        for (int t = 0; t <= 2 * T; ++t) { double xx = t * pitch / 2; lo.push_back({xx, (t % 2 ? -amp : 0) + U(-0.3, 0.3)}); hi.push_back({xx + U(-0.4, 0.4), w + (t % 2 ? amp * U(0.2, 1) : 0) + U(-0.3, 0.3)}); }
        Poly p = lo; for (int i = (int)hi.size() - 1; i >= 0; --i) p.push_back(hi[i]);
        W.add(p); snprintf(buf, sizeof buf, "saw T=%d w=%.2f", T, w); break; }
    case 5: {  // nested: outer rect, island ring, pocket inside island, island inside that
        double S = U(25, 50); W.add(rectP(0, 0, S, S));
        double a = U(0.15, 0.3) * S; W.add(rectP(a, a, S - a, S - a));
        double b = a + U(1.5, 0.12 * S); W.add(ngon({S / 2, S / 2}, (S / 2 - b), I(4, 20), U(0, 1)));
        if (I(0, 1)) W.add(ngon({S / 2, S / 2}, (S / 2 - b) * U(0.2, 0.5), I(3, 10)));
        snprintf(buf, sizeof buf, "nested S=%.1f", S); break; }
    case 6: {  // dumbbell: two rooms joined by a corridor
        double w1 = U(8, 20), h1 = U(8, 20), cw = U(0.6, 5), cl = U(2, 20), w2 = U(8, 20), h2 = U(8, 20);
        double cy = U(cw / 2 + 0.5, std::min(h1, h2) - cw / 2 - 0.5);
        Poly p{{0, 0}, {w1, 0}, {w1, cy - cw / 2}, {w1 + cl, cy - cw / 2}, {w1 + cl, 0}, {w1 + cl + w2, 0}, {w1 + cl + w2, h2}, {w1 + cl, h2}, {w1 + cl, cy + cw / 2}, {w1, cy + cw / 2}, {w1, h1}, {0, h1}};
        W.add(p);
        if (I(0, 1)) W.add(ngon({w1 / 2, h1 / 2}, std::min(w1, h1) * U(0.1, 0.3), I(3, 12)));
        snprintf(buf, sizeof buf, "dumbbell cw=%.2f", cw); break; }
    default: {  // rows of round islands
        double Wd = U(30, 70), H = U(20, 40); W.add(rectP(0, 0, Wd, H));
        int nxI = I(1, 6), nyI = I(1, 3); double r = std::min(Wd / nxI, H / nyI) * U(0.1, 0.4);
        for (int i = 0; i < nxI; ++i) for (int j = 0; j < nyI; ++j) W.add(ngon({Wd * (i + 0.5) / nxI, H * (j + 0.5) / nyI}, r, I(8, 32), U(0, 1)));
        snprintf(buf, sizeof buf, "rows %dx%d", nxI, nyI); break; }
    }
    name = buf;
}

static PocketTool mkTool(const char* nm, int num, double d, double so, double pd) {
    PocketTool t;
#ifdef NEWAPI
    t.tool.id = nm; t.tool.name = nm; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = d;
    auto& cd = t.tool.defaultCuttingData; cd.toolNumber = num; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.stepover = so; cd.passDepth = pd;
#else
    t.tool.name = nm; t.tool.toolNumber = num; t.tool.diameter = d; t.tool.feedRate = 1000; t.tool.plungeRate = 300; t.tool.spindleSpeed = 18000; t.stepover = so; t.passDepth = pd;
#endif
    return t;
}

// 1D squared EDT (Felzenszwalb)
static void edt1(const double* f, double* d, int n, std::vector<int>& v, std::vector<double>& z) {
    int k = 0; v[0] = 0; z[0] = -1e300; z[1] = 1e300;
    for (int q = 1; q < n; ++q) {
        double s;
        for (;;) { int p = v[k]; s = ((f[q] + double(q) * q) - (f[p] + double(p) * p)) / (2.0 * q - 2.0 * p); if (s <= z[k] && k > 0) --k; else break; }
        if (s <= z[k]) { v[0] = q; z[0] = -1e300; z[1] = 1e300; k = 0; continue; }
        ++k; v[k] = q; z[k] = s; z[k + 1] = 1e300;
    }
    k = 0;
    for (int q = 0; q < n; ++q) { while (z[k + 1] < q) ++k; d[q] = double(q - v[k]) * (q - v[k]) + f[v[k]]; }
}

int main(int argc, char** argv) {
    unsigned long seed = std::strtoul(argv[1], nullptr, 10);
    int N = std::atoi(argv[2]);
    int check = std::atoi(argv[3]);
    int target = argc > 4 ? std::atoi(argv[4]) : -1;
    int forceKind = getenv("KIND") ? atoi(getenv("KIND")) : -1;
    int forceTools = getenv("NTOOLS") ? atoi(getenv("NTOOLS")) : -1;
    int forceStrat = getenv("STRAT") ? atoi(getenv("STRAT")) : -1;
    rng.seed(seed);
    for (int sc = 0; sc < N; ++sc) {
        World W; std::string name;
        int kind = forceKind >= 0 ? forceKind : I(0, 7);
        makeScene(kind, W, name);
        PocketParams P;
        P.strategy = I(0, 1) ? PocketStrategy::Raster : PocketStrategy::Offset;
        if (forceStrat >= 0) P.strategy = (PocketStrategy)forceStrat;
        P.profilePass = (ProfilePass)I(0, 2);
        P.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        P.rasterAngle = I(0, 2) == 0 ? 0.0 : U(0, PI);
        P.allowance = I(0, 2) == 0 ? U(0, 0.5) : 0.0;
        P.startDepth = I(0, 3) == 0 ? U(0, 2) : 0.0;
        P.cutDepth = U(1, 8);
        if (I(0, 4) == 0) P.lastPassThickness = U(0.1, 0.5);
        P.rampPlunge = I(0, 1);
        P.rampDistance = U(1, 15);
        if (I(0, 3) == 0) { P.tolerance.chordal = U(0.002, 0.05); P.tolerance.distance = std::min(P.tolerance.distance, P.tolerance.chordal); }
        int nt = forceTools > 0 ? forceTools : I(1, 3);
        std::vector<PocketTool> tools; std::vector<double> rads, pds;
        double D = U(1.5, 8);
        for (int t = 0; t < nt; ++t) {
            double so = D * U(0.15, 1.0), pd = U(0.5, 4);
            char nm[16]; snprintf(nm, sizeof nm, "T%d", t + 1);
            tools.push_back(mkTool(nm, t + 1, D, so, pd)); rads.push_back(D / 2); pds.push_back(pd);
            D *= U(0.15, 0.6); D = std::max(D, 0.3);
        }
        std::vector<Contour> cs; for (auto& p : W.polys) cs.push_back(Contour::fromPolyline(geometry::Polyline{p, true}));
        W.build();
        if (target >= 0 && sc != target) continue;
        if (target >= 0 && getenv("DUMP")) { for (size_t t = 0; t < tools.size(); ++t) std::printf("tool %zu r %.6f pd %.4f\n", t, rads[t], pds[t]); std::printf("allow %.6f start %.4f cut %.4f chordal %.5f dist %.6f last %.4f ramp %d angle %.5f dir %d\n", P.allowance, P.startDepth, P.cutDepth, P.tolerance.chordal, P.tolerance.distance, P.lastPassThickness, (int)P.rampPlunge, P.rasterAngle, (int)P.direction); for (auto& p : W.polys) { std::printf("poly"); for (auto& v : p) std::printf(" %.6f,%.6f", v.x, v.y); std::printf("\n"); } }
        auto t0 = std::chrono::steady_clock::now();
        PocketResult res = pocketToolpath(cs, tools, P);
        double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        unsigned long long h = 1469598103934665603ULL, hd = h;
        auto mix = [&](unsigned long long& hh, const char* s) { for (; *s; ++s) { hh ^= (unsigned char)*s; hh *= 1099511628211ULL; } };
        char b[256]; size_t nmoves = 0; std::string retr;
        for (size_t ti = 0; ti < res.toolpaths.size(); ++ti) { auto& tp = res.toolpaths[ti];
            for (auto& m : tp.moves) { std::snprintf(b, sizeof b, "%d %.6f %.6f %.6f|", (int)m.kind, m.to.x + 0.0, m.to.y + 0.0, m.to.z + 0.0); mix(h, b); if (target >= 0 && getenv("DUMP")) std::printf("t%zu %s\n", ti, b); }
            nmoves += tp.moves.size(); retr += std::to_string(tp.count(MoveKind::Retract)) + ","; }
        std::string diagtxt;
#ifdef NEWAPI
        for (auto& d : res.diagnostics) { mix(hd, d.message.c_str()); diagtxt += "[" + std::to_string((int)d.issue) + "/t" + (d.tool ? std::to_string(*d.tool) : std::string("-")) + "] " + d.message + "; "; }
#else
        for (auto& d : res.messages) { mix(hd, d.c_str()); diagtxt += d + "; "; }
#endif
        std::printf("scene %d kind %d [%s] strat %d pp %d nt %d moves %zu retr %s hash %016llx dhash %016llx t %.3f\n", sc, kind, name.c_str(), (int)P.strategy, (int)P.profilePass, nt, nmoves, retr.c_str(), h, hd, secs);
        if (!diagtxt.empty()) std::printf("  diags: %s\n", diagtxt.c_str());
        if (!check) { std::fflush(stdout); continue; }
        // ---- independent checks ----
        bool refused = false;
        for (auto& d : diagtxt.empty() ? std::string() : diagtxt) (void)d;
        if (diagtxt.find("skipped") != std::string::npos || diagtxt.find("could not be kept") != std::string::npos || diagtxt.find("trimmed") != std::string::npos || diagtxt.find("too detailed") != std::string::npos) refused = true;
        const double top = -P.startDepth, bottom = -(P.startDepth + P.cutDepth), a = P.allowance, m = P.tolerance.chordal;
        double rmin = 1e9; for (size_t ti = 0; ti < tools.size(); ++ti) rmin = std::min(rmin, rads[ti]);
        double ext = std::max(W.x1 - W.x0, W.y1 - W.y0);
        double hcell = std::max(std::min(rmin / 4, ext / 600), ext / 2500);
        int gx = int((W.x1 - W.x0) / hcell) + 3, gy = int((W.y1 - W.y0) / hcell) + 3;
        double ox = W.x0 - hcell, oy = W.y0 - hcell;
        std::vector<float> cleared(size_t(gx) * gy, (float)top);
        auto at = [&](int i, int j) -> float& { return cleared[size_t(j) * gx + i]; };
        long gouge = 0, outside = 0, overload = 0, badRapid = 0; double worstG = 0, worstE = 0; std::string sG, sE;
        for (size_t ti = 0; ti < res.toolpaths.size(); ++ti) {
            const auto& tp = res.toolpaths[ti]; if (tp.moves.empty()) continue;
            const double r = rads[ti];
            const auto& lv = res.plans[ti].levels;
            double maxStep = 0, prevZ = top; for (double z : lv) { maxStep = std::max(maxStep, prevZ - z); prevZ = z; }
            geometry::Vec3 cur = tp.moves.front().to; bool haveCur = false;
            for (size_t mi = 0; mi < tp.moves.size(); ++mi) {
                const auto& mv = tp.moves[mi];
                geometry::Vec3 from = haveCur ? cur : mv.to; haveCur = true; cur = mv.to;
                if (mv.kind == MoveKind::Rapid) { if (std::min(from.z, mv.to.z) < P.safeZ - 1e-9 && !(mi == 0)) ++badRapid; continue; }
                double zlo = std::min(from.z, mv.to.z);
                if (zlo >= P.safeZ - 1e-9) continue;
                Vec2 A{from.x, from.y}, B{mv.to.x, mv.to.y};
                if (zlo <= top + 1e-9) {
                    double dd = W.segDist(A, B, r + a + 1);
                    double ex = dd - (r + a);
                    if (ex < -1e-7) { ++gouge; if (ex < worstG) { worstG = ex; snprintf(b, sizeof b, "t%zu move %zu (%.5f,%.5f)-(%.5f,%.5f) z %.4f ex %.3e", ti, mi, A.x, A.y, B.x, B.y, zlo, ex); sG = b; } }
                    Vec2 mid{(A.x + B.x) / 2, (A.y + B.y) / 2};
                    if (!insideEO(W.polys, mid) || !insideEO(W.polys, A)) ++outside;
                }
                if (zlo >= top - 1e-9) continue;
                // engagement + update
                double bx0 = std::min(A.x, B.x) - r, bx1 = std::max(A.x, B.x) + r, by0 = std::min(A.y, B.y) - r, by1 = std::max(A.y, B.y) + r;
                int i0 = std::max(0, int((bx0 - ox) / hcell)), i1 = std::min(gx - 1, int((bx1 - ox) / hcell) + 1);
                int j0 = std::max(0, int((by0 - oy) / hcell)), j1 = std::min(gy - 1, int((by1 - oy) / hcell) + 1);
                double dx = B.x - A.x, dy = B.y - A.y, L2 = dx * dx + dy * dy; double E = -1e9;
                for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
                    Vec2 q{ox + i * hcell, oy + j * hcell};
                    double t = L2 > 0 ? std::clamp(((q.x - A.x) * dx + (q.y - A.y) * dy) / L2, 0.0, 1.0) : 1.0;
                    double d = std::hypot(q.x - A.x - t * dx, q.y - A.y - t * dy);
                    if (d > r) continue;
                    double zt = from.z + t * (mv.to.z - from.z);
                    float& c = at(i, j);
                    if (d <= r - 1.5 * hcell) E = std::max(E, double(c) - zt);
                    if (zt < c) c = (float)zt;
                }
                if (E > maxStep + 1e-3) { ++overload; if (E - maxStep > worstE) { worstE = E - maxStep; snprintf(b, sizeof b, "t%zu move %zu kind %d (%.4f,%.4f,%.4f)-(%.4f,%.4f,%.4f) engages %.4f > step %.4f", ti, mi, (int)mv.kind, from.x, from.y, from.z, mv.to.x, mv.to.y, mv.to.z, E, maxStep); sE = b; } }
            }
        }
        // coverage
        long miss = 0; double worstMiss = 0; std::string sM; bool scalloped = P.strategy == PocketStrategy::Raster && P.profilePass == ProfilePass::None;
        bool anyRun = false; for (auto& tp : res.toolpaths) anyRun |= !tp.moves.empty();
        // smallest tool that produced moves or reported "no rest"/"cannot reach": use the smallest tool overall
        if (!refused && !scalloped) {
            const double slack = 2 * m + 0.002 + 2 * hcell + 10 * P.tolerance.distance;
            std::vector<double> f(size_t(gx) * gy), dist(size_t(gx) * gy), Dw(size_t(gx) * gy);
            std::vector<char> in(size_t(gx) * gy);
            for (int j = 0; j < gy; ++j) for (int i = 0; i < gx; ++i) { Vec2 q{ox + i * hcell, oy + j * hcell}; size_t k = size_t(j) * gx + i; in[k] = insideEO(W.polys, q); Dw[k] = in[k] ? W.ptDist(q) : -1; f[k] = (in[k] && Dw[k] >= rmin + a + m + slack) ? 0 : 1e30; }
            // 2D EDT
            int nmax = std::max(gx, gy); std::vector<int> v(nmax + 1); std::vector<double> z(nmax + 2), tmp(nmax), out(nmax);
            for (int i = 0; i < gx; ++i) { for (int j = 0; j < gy; ++j) tmp[j] = f[size_t(j) * gx + i]; edt1(tmp.data(), out.data(), gy, v, z); for (int j = 0; j < gy; ++j) dist[size_t(j) * gx + i] = out[j]; }
            for (int j = 0; j < gy; ++j) { edt1(&dist[size_t(j) * gx], out.data(), gx, v, z); for (int i = 0; i < gx; ++i) dist[size_t(j) * gx + i] = std::sqrt(out[i]) * hcell; }
            for (int j = 0; j < gy; ++j) for (int i = 0; i < gx; ++i) { size_t k = size_t(j) * gx + i;
                if (!in[k] || dist[k] > rmin - slack) continue;
                if (cleared[k] > bottom + 1e-6) { ++miss; double depth = Dw[k]; if (miss == 1 || depth > worstMiss) { worstMiss = depth; snprintf(b, sizeof b, "(%.4f,%.4f) wall %.4f floor-z %.4f", ox + i * hcell, oy + j * hcell, Dw[k], cleared[k]); sM = b; } } }
        }
        // retracts vs parts
        std::string sR;
        if (!scalloped) for (size_t ti = 0; ti < res.toolpaths.size(); ++ti) {
            const auto& plan = res.plans[ti]; if (plan.paths.empty()) continue;
            std::vector<Poly> outers; for (auto& c : plan.centreRegion) { auto pts = c.polygonise(geometry::Tolerance::defaults()).points; if (area(pts) > 0) outers.push_back(pts); }
            std::vector<int> used(outers.size(), 0);
            for (auto& pp : plan.paths) { if (pp.path.points.empty()) continue; Vec2 q{0, 0}; for (auto& v : pp.path.points) { q.x += v.x; q.y += v.y; } q.x /= double(pp.path.points.size()); q.y /= double(pp.path.points.size()); if (!insideEO(W.polys, q)) q = pp.path.points.front(); int best = -1; double ba = 1e300;
                for (size_t o = 0; o < outers.size(); ++o) if (insideEO({outers[o]}, q) || W.segDist(q, q, 1) < 0) { double A = area(outers[o]); if (A < ba) { ba = A; best = int(o); } }
                if (best < 0) { for (size_t o = 0; o < outers.size(); ++o) { double dd = 1e9; for (size_t e = 0; e < outers[o].size(); ++e) dd = std::min(dd, segPt(q, outers[o][e], outers[o][(e + 1) % outers[o].size()])); if (dd < 1e-3) { best = int(o); break; } } }
                if (getenv("DBG")) std::printf("   dbg t%zu cluster %zu start (%.4f,%.4f) part %d n=%zu\n", ti, pp.cluster, q.x, q.y, best, pp.path.points.size());
                if (best >= 0) used[best] = 1; }
            int parts = 0; for (int u : used) parts += u;
            size_t rc = res.toolpaths[ti].count(MoveKind::Retract);
            if ((int)rc != parts) sR += "t" + std::to_string(ti) + " retracts " + std::to_string(rc) + " parts " + std::to_string(parts) + "; ";
        }
        std::printf("  CHECK h=%.4f refused %d scal %d | GOUGE %ld %s | OUTSIDE %ld | OVERLOAD %ld %s | MISS %ld %s | RAPID %ld | RETR %s\n", hcell, (int)refused, (int)scalloped, gouge, sG.c_str(), outside, overload, sE.c_str(), miss, sM.c_str(), badRapid, sR.c_str());
        std::fflush(stdout);
    }
}
