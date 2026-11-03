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
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/tooling/tool.hpp"
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

static tooling::Tool mkEngTool(int kind) {
    tooling::Tool t; t.id = "t"; t.name = "Engraver"; 
    auto& cd = t.defaultCuttingData; cd.toolNumber = 4; cd.feedRate = 900; cd.plungeRate = 250; cd.spindleSpeed = 18000; cd.passDepth = 1; cd.stepover = 0.3;
    switch (kind) {
    case 0: t.type = tooling::ToolType::VBit; t.diameter = U(3, 12); t.includedAngle = (I(0,3)==0 ? 30.0 : I(0,2)==0? 60.0 : I(0,1)? 90.0 : U(20,150)) * PI / 180; break;
    case 1: t.type = tooling::ToolType::Engraving; t.diameter = U(3, 6); t.includedAngle = U(10, 90) * PI / 180; t.tipDiameter = U(0.05, 0.6); break;
    case 2: t.type = tooling::ToolType::EndMill; t.diameter = U(0.3, 4); break;
    default: t.type = tooling::ToolType::BallNose; t.diameter = U(0.5, 6); break;
    }
    return t;
}
int main(int argc, char** argv) {
    unsigned long seed = std::strtoul(argv[1], nullptr, 10);
    int N = std::atoi(argv[2]);
    int target = argc > 3 ? std::atoi(argv[3]) : -1;
    rng.seed(seed);
    long tot = 0, ok = 0, bad = 0;
    for (int sc = 0; sc < N; ++sc) {
        World W; std::string name;
        int kind = I(0, 7);
        makeScene(kind, W, name);
        int tk = I(0, 3);
        tooling::Tool tool = mkEngTool(tk);
        EngraveParams E;
        E.strategy = EngraveStrategy::Fill;
        E.fillPattern = I(0, 1) ? PocketStrategy::Raster : PocketStrategy::Offset;
        E.depth = tk == 2 ? U(0.05, 3) : U(0.02, 1.5);
        E.passes = I(0, 3) == 0 ? I(1, 12) : I(1, 3);
        E.safeZ = U(0.5, 10);
        E.rasterAngle = I(0, 2) == 0 ? 0.0 : U(-PI, 2*PI);
        if (I(0, 3) == 0) { E.tolerance.chordal = U(0.002, 0.05); E.tolerance.distance = std::min(E.tolerance.distance, E.tolerance.chordal); }
        double width = tooling::cuttingWidthAtDepth(tool, E.depth);
        int sm = I(0, 4);
        E.stepover = sm == 0 ? 0.0 : sm == 1 ? width : width * U(0.1, 1.0);
        std::vector<Contour> cs; for (auto& p : W.polys) cs.push_back(Contour::fromPolyline(geometry::Polyline{p, true}));
        // scramble in an open vector sometimes
        bool addOpen = I(0, 4) == 0;
        if (addOpen) { Contour o; o.append(geometry::LineSpan{{-50, -50}, {-40, -45}}); o.closed = false; cs.insert(cs.begin() + I(0, (int)cs.size()), o); }
        W.build();
        if (target >= 0 && sc != target) continue;
        ++tot;
        EngraveResult res = engraveToolpath(cs, tool, std::nullopt, E);
        // direct pocket
        std::vector<Contour> closed; for (auto& c : cs) if (c.closed) closed.push_back(c);
        PocketTool pt{tool, tool.defaultCuttingData};
        double so = E.stepover > 0 ? E.stepover : tool.defaultCuttingData.stepover;
        pt.cuttingData->stepover = so; pt.cuttingData->passDepth = E.depth / E.passes;
        PocketParams P; P.name = E.name; P.cutDepth = E.depth; P.maintainExactStep = true; P.safeZ = E.safeZ; P.strategy = E.fillPattern;
        P.direction = CutDirection::Climb; P.rasterAngle = E.rasterAngle; P.profilePass = ProfilePass::Last; P.tolerance = E.tolerance;
        PocketResult pr = pocketToolpath(closed, {pt}, P);
        bool same;
        if (so > width * (1 + 1e-12)) same = !res.ok() && res.has(ToolpathIssue::InvalidTool);
        else same = (pr.toolpaths.front().moves == res.toolpath.moves) && pr.diagnostics.size() + 0 == res.diagnostics.size() - (addOpen ? 1 : 0);
        std::string diag; for (auto& d : res.diagnostics) { diag += "[" + std::to_string((int)d.issue) + "] " + d.message + "; "; if (d.message.find("ocket") != std::string::npos) diag += "<<POCKETWORD>>"; if (d.tool) diag += "<<TOOLIDX>>"; }
        if (!res.ok()) { std::printf("scene %d kind %d tk %d [%s] refused same %d: %s\n", sc, kind, tk, name.c_str(), (int)same, diag.c_str()); if (!same) ++bad; continue; }
        ++ok;
        // independent checks
        const double r = width / 2, depth = E.depth;
        long gouge = 0, gougeTrue = 0, outside = 0, badRapid = 0, below = 0, badPl = 0; double worst = 0; char b[300]; std::string sG;
        std::vector<double> zs;
        geometry::Vec3 cur{}; bool have = false;
        const auto& mv = res.toolpath.moves;
        if (mv.front().kind != MoveKind::Rapid || mv.front().to.z != E.safeZ) ++badRapid;
        if (mv.back().to.z < E.safeZ - 1e-12) ++badRapid;
        for (size_t mi = 0; mi < mv.size(); ++mi) {
            const auto& m = mv[mi]; geometry::Vec3 from = have ? cur : m.to; have = true; cur = m.to;
            if (m.to.z < -depth - 1e-12) ++below;
            if (m.kind == MoveKind::Rapid) { if (std::min(from.z, m.to.z) < E.safeZ - 1e-12) ++badRapid; continue; }
            if (m.kind == MoveKind::Plunge && (std::fabs(m.to.x-from.x)>1e-9||std::fabs(m.to.y-from.y)>1e-9)) ++badPl;
            double zlo = std::min(from.z, m.to.z);
            if (zlo >= 0) continue;
            if ((m.kind == MoveKind::Feed || isArc(m.kind)) && std::fabs(from.z - m.to.z) < 1e-12) zs.push_back(m.to.z);
            Vec2 A{from.x, from.y}, B{m.to.x, m.to.y};
            if (isArc(m.kind)) { // sample arc
                double rr = std::hypot(A.x - m.centre.x, A.y - m.centre.y);
                double a0 = std::atan2(A.y - m.centre.y, A.x - m.centre.x), a1 = std::atan2(B.y - m.centre.y, B.x - m.centre.x);
                double sw = a1 - a0; if (m.kind == MoveKind::ArcCCW) { while (sw <= 1e-12) sw += TAU; } else { while (sw >= -1e-12) sw -= TAU; }
                Vec2 prev = A; int ns = 64;
                for (int k = 1; k <= ns; ++k) { double a = a0 + sw * k / ns; Vec2 q{m.centre.x + rr * std::cos(a), m.centre.y + rr * std::sin(a)}; double dd = W.segDist(prev, q, r + 1); if (dd - r < -1e-7) { ++gouge; } prev = q; }
                continue;
            }
            double dd = W.segDist(A, B, r + 1);
            double ex = dd - r;
            if (ex < -1e-7) { ++gouge; if (ex < worst) { worst = ex; snprintf(b, sizeof b, "move %zu (%.5f,%.5f)-(%.5f,%.5f) z %.4f ex %.3e", mi, A.x, A.y, B.x, B.y, zlo, ex); sG = b; } }
            double rTrue = tooling::cuttingWidthAtDepth(tool, -zlo) / 2;
            if (dd - rTrue < -1e-7) ++gougeTrue;
            Vec2 mid{(A.x + B.x) / 2, (A.y + B.y) / 2};
            if (!insideEO(W.polys, mid) || !insideEO(W.polys, A)) ++outside;
        }
        std::sort(zs.begin(), zs.end()); zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
        bool lv = (int)zs.size() == E.passes && zs.front() == -depth;
        if (lv) { for (int k = 0; k < E.passes; ++k) if (std::fabs(zs[E.passes - 1 - k] + depth * (k + 1) / E.passes) > 1e-9) lv = false; }
        bool fail = gouge || gougeTrue || outside || badRapid || below || badPl || !lv || !same || diag.find("<<") != std::string::npos;
        if (fail) ++bad;
        std::printf("scene %d kind %d tk %d [%s] pat %d d %.3f n %d w %.4f so %.4f moves %zu | %s GOUGE %ld %s TRUE %ld OUT %ld RAPID %ld BELOW %ld PL %ld LEVELS %d(%zu) SAME %d %s\n", sc, kind, tk, name.c_str(), (int)E.fillPattern, depth, E.passes, width, so, mv.size(), fail ? "FAIL" : "ok", gouge, sG.c_str(), gougeTrue, outside, badRapid, below, badPl, (int)lv, zs.size(), (int)same, diag.c_str());
        std::fflush(stdout);
    }
    std::printf("TOTAL %ld ok %ld bad %ld\n", tot, ok, bad);
}
