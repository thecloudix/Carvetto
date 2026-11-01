// T-0127 verifier (throwaway): WHOLE-SCENE FLOOD oracle, independent of the product's raster.
// Layer just below the tab top (h = zTab - 1e-4). Every cutting move is swept as exact capsules
// (arcs densified at 1e-4 mm sagitta; V-bit radius from the cone at h; plunges as discs). Plateau
// moves (z == zTab) are grouped into tabs (runs within lpa + 0.1 of each other, all levels/passes).
// Per tab g: X0 = layer with every plateau lowered to the bottom; B_g = cells uncut with all tabs
// but cut in X0 and covered only by g's lowered plateau. Components of X0 (4-connected) adjacent to
// a component of B_g get merged. N(g) = significant X0 components within dn of g's plateau.
// HOLD: g merges all of N(g); PARTIAL: some; SEVERED: none (|N(g)| >= 2); NOHOLD: |N(g)| <= 1.
// Repeated with the tools widened by 0.05 (THIN: holds exactly but not widened, i.e. neck < 0.1).
// FREE: significant components with all tabs present not touching the frame.
// usage: wsf seed N mode   (mode 0 On, 1 In/Out a in [-r,0], 2 mix, 99 SCENE=file)
// env: CELLMIN (0.02) CAP (8e6) DUMP=dir (dump flagged scenes) V=1 (verbose per tab) SC=k (only scene k)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"

struct Seg { Vec2 a, b; double za, zb; int run; bool arc = false; Vec2 c{}; double rho = 0, a0 = 0, sw = 0; };
static double segDist(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0; t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}

int main(int argc, char** argv) {
    unsigned long seed0 = std::strtoul(argv[1], nullptr, 0); int N = std::atoi(argv[2]); int mode = std::atoi(argv[3]);
    double CELLMIN = std::getenv("CELLMIN") ? std::atof(std::getenv("CELLMIN")) : 0.02;
    double CAP = std::getenv("CAP") ? std::atof(std::getenv("CAP")) : 8e6;
    bool verbose = std::getenv("V") != nullptr; int only = std::getenv("SC") ? std::atoi(std::getenv("SC")) : -1;
    long T_tabs = 0, T_hold = 0, T_nohold = 0, T_sev = 0, T_part = 0, T_thin = 0, T_free = 0, T_left = 0, T_req = 0, T_scn = 0, T_empty = 0;
    for (int sc = 0; sc < N; ++sc) {
        rng.seed(seed0 * 1000003ULL + (unsigned long)sc);
        if (only >= 0 && sc != only) continue;
        auto t0 = std::chrono::steady_clock::now();
        ProfileParams p; std::vector<Contour> cs; double D = 6; int vdeg = 0;
        if (mode == 99) { readScene(std::getenv("SCENE"), cs, p, D, vdeg); }
        else {
            bool useV = I(0, 4) == 0;
            if (useV) { D = std::vector<double>{6.0, 12.0, 25.0}[(size_t)I(0, 2)]; vdeg = std::vector<int>{60, 90, 120}[(size_t)I(0, 2)]; }
            else D = std::vector<double>{2.0, 3.175, 6.0, 8.0, 12.0}[(size_t)I(0, 4)];
            int sk = mode == 0 ? 2 : mode == 1 ? I(0, 1) : I(0, 2);
            p.side = sk == 0 ? ProfileSide::Outside : sk == 1 ? ProfileSide::Inside : ProfileSide::On;
            p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
            p.passes.startDepth = I(0, 4) == 0 ? U(0, 3) : 0.0;
            p.passes.cutDepth = useV ? U(1, 6) : U(2, 15);
            p.passes.passDepth = I(0, 3) == 0 ? U(0.4, 1.5) : U(1, 8);
            if (I(0, 5) == 0) p.passes.lastPassThickness = U(0.2, 1.0);
            p.separateLastPass = I(0, 1);
            p.lastPassAllowance = I(0, 5) == 0 ? U(0.0, 0.1) : U(0.05, 1.0);
            p.lastPassReverse = I(0, 1);
            p.tabs.enabled = true;
            int lk = I(0, 3);
            p.tabs.length = lk < 2 ? U(0.2, 4) : lk == 2 ? U(1, 12) : U(0.1, 1.5);
            p.tabs.thickness = U(0.2, 0.9 * std::min(6.0, p.passes.cutDepth));
            p.tabs.spacing = I(0, 3) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
            p.tabs.count = I(1, 9); p.tabs.distance = U(8, 80);
            p.tabs.avoidCorners = I(0, 1); p.tabs.threeD = I(0, 2) == 0;
            p.ramp.type = (RampType)I(0, 3); p.ramp.distance = U(1, 40); p.ramp.useAngle = I(0, 2) == 0; p.ramp.angle = U(1, 30) * PI / 180; p.ramp.onLeadIn = I(0, 3) == 0;
            p.leads.type = (LeadType)I(0, 2); p.leads.length = U(0.5, 15); p.leads.angle = U(0.05, PI / 2); p.leads.radius = U(0.5, 12); p.leads.sweep = U(5, 90) * PI / 180;
            p.leads.leadOut = I(0, 1); p.leads.overcut = I(0, 2) == 0 ? U(0, 20) : 0.0;
            p.startAt = (StartAt)I(0, 2); p.startAnchor = (BoxAnchor)I(0, 8); p.startPosition = {U(-50, 50), U(-50, 50)};
            p.order.leftToRight = I(0, 3) == 0; p.order.grid = I(0, 3) == 0;
            if (useV) { p.corners.sharpExternal = I(0, 2) == 0; p.corners.sharpInternal = I(0, 2) == 0; }
            double R = U(8, 35);
            int f = I(0, 9);
            if (f <= 4) cs.push_back(anyClosed({0, 0}, R));
            else if (f == 5) { double s = U(10, 60); cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {s, s * U(0.5, 1.5)}})); if (I(0, 1)) p.tabs.count = 4, p.startAt = StartAt::ClosestOnBoundingBox, p.startAnchor = BoxAnchor::BottomCentre; }
            else if (f == 6) { double W = U(40, 100), H = U(30, 80); cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {W, H}}));
                int n = I(1, 3); for (int i = 0; i < n; ++i) { double rr = U(3, 12); Vec2 c{U(rr + 1, W - rr - 1), U(rr + 1, H - rr - 1)}; cs.push_back(I(0, 1) ? Contour::circle(c, rr) : Contour::rectangle(geometry::Box2{{c.x - rr, c.y - rr * 0.7}, {c.x + rr, c.y + rr * 0.7}})); } }
            else if (f == 7) { cs.push_back(anyClosed({0, 0}, R)); double gap = U(0, 3 * D); auto bx = [](const Contour& c) { geometry::Box2 b; for (auto& sp : c.spans) b.expand(geometry::bounds(sp)); return b; }; Contour c2 = anyClosed({0, 0}, U(6, 30)); double sh = bx(cs[0]).max.x + gap - bx(c2).min.x; cs.push_back(translate(c2, {sh, U(-5, 5)})); }
            else if (f == 8) cs.push_back(I(0, 1) ? bezBlob({0, 0}, R, I(4, 9)) : ellipse({0, 0}, U(5, 40), U(3, 20), U(0, PI)));
            else { cs.push_back(letter({0, 0}, U(8, 40))); if (I(0, 1)) cs.push_back(letter({50, 0}, U(8, 40))); }
            tooling::Tool tt = useV ? vbit(D, vdeg) : endMill(D);
            double r = profileToolRadius(tt, p.passes);
            if (p.side == ProfileSide::On) {
                int k = I(0, 7);
                p.allowance = k < 2 ? 0.0 : k < 5 ? U(-0.6, 0.6) : U(-0.95, 0.95) * r;
            } else {
                int k = I(0, 7);
                p.allowance = k == 0 ? -r : k <= 3 ? std::min(0.0, -r + U(0, 0.5)) : k <= 6 ? -r * U(0, 1) : -U(0, 0.02);
            }
        }
        tooling::Tool tool = vdeg ? vbit(D, vdeg) : endMill(D);
        auto res = profileToolpath(cs, tool, p);
        const auto& mv = res.toolpath.moves;
        ++T_scn;
        int leftOut = 0, req = 0; std::set<size_t> allLost;
        for (auto& d : res.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced) { int a, b; if (std::sscanf(d.message.c_str() + d.message.find(": ") + 2, "%d of %d", &a, &b) == 2) { leftOut += a; req += b; if (a == b && d.contour) allLost.insert(*d.contour); } }
        if (mv.empty()) { ++T_empty; std::printf("S %lu %d empty\n", seed0, sc); continue; }
        const double bottom = p.passes.bottomZ();
        const double zTab = bottom + std::min(p.tabs.thickness, p.passes.cutDepth), h = zTab - 1e-4;
        const double Rt = D / 2, tanH = vdeg ? std::tan(vdeg * PI / 360) : 0;
        auto rad = [&](double z) { if (z >= h) return -1.0; return vdeg ? std::min(Rt, (h - z) * tanH) : Rt; };
        const double rLow = rad(bottom);
        const double lpa = p.separateLastPass ? std::max(0.0, p.lastPassAllowance) : 0.0;
        // densify
        std::vector<Seg> segs; std::vector<int> runOfMove(mv.size(), -1); int nRuns = 0;
        geometry::Vec3 pp = mv[0].to; int curRun = -1;
        for (size_t i = 1; i < mv.size(); ++i) {
            const auto& m = mv[i];
            if (!isCutting(m.kind)) { pp = m.to; curRun = -1; continue; }
            bool plateau = (m.kind == MoveKind::Feed || isArc(m.kind)) && std::fabs(pp.z - zTab) < 1e-7 && std::fabs(m.to.z - zTab) < 1e-7;
            int run = -1;
            if (plateau) { if (curRun < 0) curRun = nRuns++; run = curRun; } else curRun = -1;
            std::vector<geometry::Vec3> pts{pp};
            if (isArc(m.kind)) {
                double r0 = std::hypot(pp.x - m.centre.x, pp.y - m.centre.y);
                double a0 = std::atan2(pp.y - m.centre.y, pp.x - m.centre.x), a1 = std::atan2(m.to.y - m.centre.y, m.to.x - m.centre.x);
                double sw = m.kind == MoveKind::ArcCCW ? a1 - a0 : a0 - a1; sw = std::fmod(sw, 2 * PI); if (sw < 0) sw += 2 * PI; if (sw < 1e-12) sw = 2 * PI;
                if (m.kind == MoveKind::ArcCW) sw = -sw;
                double da = std::min(0.4, 1.0 / std::max(r0, 1e-9));
                int n = std::max(1, (int)std::ceil(std::fabs(sw) / da));
                geometry::Vec3 prev = pp;
                for (int k = 1; k <= n; ++k) { double a = a0 + sw * k / n; geometry::Vec3 q{m.centre.x + r0 * std::cos(a), m.centre.y + r0 * std::sin(a), pp.z + (m.to.z - pp.z) * k / n}; if (k == n) q = m.to;
                    Seg sg{{prev.x, prev.y}, {q.x, q.y}, prev.z, q.z, run}; sg.arc = true; sg.c = m.centre; sg.rho = r0; sg.a0 = a0 + sw * (k - 1) / n; sg.sw = sw / n; segs.push_back(sg); prev = q; }
                pp = m.to; continue;
            } else {
                double L = std::hypot(m.to.x - pp.x, m.to.y - pp.y); int n = std::max(1, (int)std::ceil(L / 1.0));
                for (int k = 1; k <= n; ++k) pts.push_back({pp.x + (m.to.x - pp.x) * k / n, pp.y + (m.to.y - pp.y) * k / n, pp.z + (m.to.z - pp.z) * k / n});
            }
            for (size_t k = 1; k < pts.size(); ++k) segs.push_back({{pts[k - 1].x, pts[k - 1].y}, {pts[k].x, pts[k].y}, pts[k - 1].z, pts[k].z, run});
            pp = m.to;
        }
        // a run is a tab plateau only if entered from below and left downwards (not a lead at the tab top)
        {
            std::vector<int> enter(nRuns, 0), leave(nRuns, 0);
            for (size_t k = 0; k < segs.size(); ++k) {
                int r0 = segs[k].run; if (r0 < 0) continue;
                if (k > 0 && segs[k - 1].run != r0 && segs[k - 1].za < zTab - 1e-7) enter[r0] = 1;
                if (k + 1 < segs.size() && segs[k + 1].run != r0 && segs[k + 1].zb < zTab - 1e-7) leave[r0] = 1;
            }
            std::vector<int> remap(nRuns, -1); int nn = 0;
            for (int i = 0; i < nRuns; ++i) if (enter[i] && leave[i]) remap[i] = nn++;
            for (auto& s : segs) if (s.run >= 0) s.run = remap[s.run];
            nRuns = nn;
        }
        // group runs
        std::vector<std::vector<Vec2>> runPts(nRuns); std::vector<geometry::Box2> runBox(nRuns);
        for (auto& s : segs) if (s.run >= 0) { auto& v = runPts[s.run]; double L = s.arc ? std::fabs(s.sw) * s.rho : geometry::distance(s.a, s.b); int n = std::max(1, (int)std::ceil(L / 0.05)); for (int k = v.empty() ? 0 : 1; k <= n; ++k) { double t = (double)k / n; v.push_back(s.arc ? s.c + Vec2{std::cos(s.a0 + s.sw * t), std::sin(s.a0 + s.sw * t)} * s.rho : s.a + (s.b - s.a) * t); } }
        for (int i = 0; i < nRuns; ++i) for (auto& q : runPts[i]) runBox[i].expand(q);
        std::vector<int> par(nRuns); for (int i = 0; i < nRuns; ++i) par[i] = i;
        std::function<int(int)> fnd = [&](int x) { return par[x] == x ? x : par[x] = fnd(par[x]); };
        double thr = lpa + 0.1;
        for (int i = 0; i < nRuns; ++i) for (int j = i + 1; j < nRuns; ++j) {
            if (fnd(i) == fnd(j) || !runBox[i].inflated(thr).intersects(runBox[j])) continue;
            bool close = false;
            for (auto& a : runPts[i]) { for (auto& b : runPts[j]) if (geometry::distance(a, b) <= thr) { close = true; break; } if (close) break; }
            if (close) par[fnd(i)] = fnd(j);
        }
        std::map<int, int> gid; std::vector<int> runG(nRuns);
        for (int i = 0; i < nRuns; ++i) { int r0 = fnd(i); if (!gid.count(r0)) { int k = (int)gid.size(); gid[r0] = k; } runG[i] = gid[r0]; }
        int G = (int)gid.size();
        if (verbose) for (int i = 0; i < nRuns; ++i) { double L = 0; for (size_t k = 1; k < runPts[i].size(); ++k) L += geometry::distance(runPts[i][k - 1], runPts[i][k]); std::printf("  run %d g%d (%.4f,%.4f)->(%.4f,%.4f) len %.4f\n", i, runG[i], runPts[i].front().x, runPts[i].front().y, runPts[i].back().x, runPts[i].back().y, L); }
        // grid
        geometry::Box2 bb;
        for (auto& s : segs) { if (s.run < 0 && s.za >= h && s.zb >= h) continue; bb.expand(s.a); bb.expand(s.b); }
        if (bb.isEmpty()) { std::printf("S %lu %d nocut\n", seed0, sc); continue; }
        bb = bb.inflated(Rt + 2.0);
        double cell = std::max(CELLMIN, std::sqrt(bb.width() * bb.height() / CAP));
        int nx = (int)std::ceil(bb.width() / cell), ny = (int)std::ceil(bb.height() / cell);
        size_t NC = (size_t)nx * ny;
        std::vector<float> fAll(NC, 1e9f), v1(NC, 1e9f), v2(NC, 1e9f); std::vector<short> g1(NC, -1), g2(NC, -1);
        const double DM = 0.06;
        auto cx = [&](int i) { return bb.min.x + (i + 0.5) * cell; }; auto cy = [&](int j) { return bb.min.y + (j + 0.5) * cell; };
        auto segD = [&](Vec2 p, const Seg* sa, Vec2 a, Vec2 b) {
            if (!sa) return segDist(p, a, b);
            Vec2 v = p - sa->c; Vec2 u0{std::cos(sa->a0), std::sin(sa->a0)};
            double ang = std::atan2(u0.x * v.y - u0.y * v.x, u0.x * v.x + u0.y * v.y);
            bool in = sa->sw > 0 ? (ang >= 0 && ang <= sa->sw) : (ang <= 0 && ang >= sa->sw);
            if (in) return std::fabs(std::hypot(v.x, v.y) - sa->rho);
            return std::min(geometry::distance(p, a), geometry::distance(p, b));
        };
        auto stamp = [&](Vec2 a, Vec2 b, double rr, int grp, const Seg* sa = nullptr) {
            double ext = rr + DM + (sa ? sa->rho * (1 - std::cos(sa->sw / 2)) + 1e-9 : 0);
            int i0 = std::max(0, (int)std::floor((std::min(a.x, b.x) - ext - bb.min.x) / cell)), i1 = std::min(nx - 1, (int)std::ceil((std::max(a.x, b.x) + ext - bb.min.x) / cell));
            int j0 = std::max(0, (int)std::floor((std::min(a.y, b.y) - ext - bb.min.y) / cell)), j1 = std::min(ny - 1, (int)std::ceil((std::max(a.y, b.y) + ext - bb.min.y) / cell));
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
                double d = segD({cx(i), cy(j)}, sa, a, b) - rr; if (d > DM) continue;
                size_t k = (size_t)j * nx + i; float df = (float)d;
                if (grp < 0) { fAll[k] = std::min(fAll[k], df); continue; }
                if (g1[k] == grp) v1[k] = std::min(v1[k], df);
                else if (g2[k] == grp) { v2[k] = std::min(v2[k], df); if (v2[k] < v1[k]) { std::swap(v1[k], v2[k]); std::swap(g1[k], g2[k]); } }
                else if (df < v1[k]) { v2[k] = v1[k]; g2[k] = g1[k]; v1[k] = df; g1[k] = (short)grp; }
                else if (df < v2[k]) { v2[k] = df; g2[k] = (short)grp; }
            }
        };
        for (auto& s : segs) {
            if (s.run >= 0) { stamp(s.a, s.b, rLow, runG[s.run], s.arc ? &s : nullptr); continue; }
            if (s.za >= h && s.zb >= h) continue;
            if (s.arc && s.za < h && s.zb < h && (!vdeg || std::fabs(s.za - s.zb) < 1e-9)) { stamp(s.a, s.b, vdeg ? rad(std::max(s.za, s.zb) * 0 + std::min(s.za, s.zb)) : Rt, -1, &s); continue; }
            if (s.arc) {  // helix crossing h or V-bit helix: fine chords
                double L = std::fabs(s.sw) * s.rho; int n = std::max(1, (int)std::ceil(L / 0.005));
                for (int k = 0; k < n; ++k) { double t0 = (double)k / n, t1 = (double)(k + 1) / n; double z0 = s.za + (s.zb - s.za) * t0, z1 = s.za + (s.zb - s.za) * t1; double zm = std::min(z0, z1); if (zm >= h) continue; double rr = rad(zm);
                    Vec2 p0 = s.c + Vec2{std::cos(s.a0 + s.sw * t0), std::sin(s.a0 + s.sw * t0)} * s.rho, p1 = s.c + Vec2{std::cos(s.a0 + s.sw * t1), std::sin(s.a0 + s.sw * t1)} * s.rho;
                    if (rr > 0) stamp(p0, p1, rr, -1); }
                continue;
            }
            Vec2 a = s.a, b = s.b; double za = s.za, zb = s.zb;
            if (za >= h || zb >= h) { double t = (h - za) / (zb - za); Vec2 c = a + (b - a) * t; if (za >= h) { a = c; za = h - 1e-12; } else { b = c; zb = h - 1e-12; } }
            double L = geometry::distance(a, b);
            if (!vdeg || std::fabs(za - zb) < 1e-9 || L < 1e-9) { double rr = vdeg ? rad(std::min(za, zb)) : Rt; if (rr > 0) stamp(a, b, rr, -1); continue; }
            int n = std::max(1, (int)std::ceil(L / 0.005));
            for (int k = 0; k < n; ++k) { double t0 = (double)k / n, t1 = (double)(k + 1) / n; double rr = rad(std::min(za + (zb - za) * t0, za + (zb - za) * t1)); if (rr > 0) stamp(a + (b - a) * t0, a + (b - a) * t1, rr, -1); }
        }
        // per delta
        struct TabRes { int v0 = -1, v5 = -1; Vec2 mid; double len = 0; };
        std::vector<TabRes> tr(G);
        for (int i = 0; i < nRuns; ++i) { double L = 0; for (size_t k = 1; k < runPts[i].size(); ++k) L += geometry::distance(runPts[i][k - 1], runPts[i][k]); if (L > tr[runG[i]].len) { tr[runG[i]].len = L; tr[runG[i]].mid = runPts[i][runPts[i].size() / 2]; } }
        std::vector<int> lab(NC);
        int freeCnt = 0; double freeArea = 0; Vec2 freeAt{};
        for (int pass = 0; pass < 2; ++pass) {
            double del = pass == 0 ? 0.0 : (std::getenv("DEL") ? std::atof(std::getenv("DEL")) : 0.05);
            std::fill(lab.begin(), lab.end(), -1);
            std::vector<long> area; std::vector<float> maxc; std::vector<char> border;
            std::vector<size_t> st;
            for (size_t k0 = 0; k0 < NC; ++k0) {
                if (lab[k0] >= 0 || std::min(fAll[k0], v1[k0]) <= del) continue;
                int L = (int)area.size(); area.push_back(0); maxc.push_back(0); border.push_back(0);
                st.assign(1, k0); lab[k0] = L;
                while (!st.empty()) { size_t k = st.back(); st.pop_back(); ++area[L]; float fl = std::min(fAll[k], v1[k]); maxc[L] = std::max(maxc[L], fl);
                    int i = (int)(k % nx), j = (int)(k / nx); if (i == 0 || j == 0 || i == nx - 1 || j == ny - 1) border[L] = 1;
                    const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
                    for (int e = 0; e < 4; ++e) { int ii = i + di[e], jj = j + dj[e]; if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue; size_t kk = (size_t)jj * nx + ii; if (lab[kk] >= 0 || std::min(fAll[kk], v1[kk]) <= del) continue; lab[kk] = L; st.push_back(kk); } }
            }
            auto signif = [&](int L) { return area[L] * cell * cell >= 0.01 && maxc[L] > del + 0.025; };
            double dn = (vdeg ? std::min(Rt, (h - bottom) * tanH) : Rt) + 0.3 + 2 * cell;
            for (int g = 0; g < G; ++g) {
                geometry::Box2 gb; std::vector<Vec2> gp;
                for (int i = 0; i < nRuns; ++i) if (runG[i] == g) for (auto& q : runPts[i]) { gb.expand(q); gp.push_back(q); }
                std::sort(gp.begin(), gp.end(), [](Vec2 a, Vec2 b) { return a.x != b.x ? a.x < b.x : a.y < b.y; });
                std::vector<Vec2> gq; for (auto& q : gp) { bool dup = false; for (size_t k = gq.size(); k-- > 0 && gq[k].x > q.x - 0.02;) if (geometry::distance(gq[k], q) < 0.02) { dup = true; break; } if (!dup) gq.push_back(q); }
                geometry::Box2 wb = gb.inflated(std::max(rLow + DM, dn) + 2 * cell);
                int i0 = std::max(0, (int)std::floor((wb.min.x - bb.min.x) / cell)), i1 = std::min(nx - 1, (int)std::ceil((wb.max.x - bb.min.x) / cell));
                int j0 = std::max(0, (int)std::floor((wb.min.y - bb.min.y) / cell)), j1 = std::min(ny - 1, (int)std::ceil((wb.max.y - bb.min.y) / cell));
                int wx = i1 - i0 + 1, wy = j1 - j0 + 1;
                std::vector<char> near((size_t)wx * wy, 0), inB((size_t)wx * wy, 0);
                for (auto& q : gq) { int a0 = std::max(i0, (int)std::floor((q.x - dn - bb.min.x) / cell)), a1 = std::min(i1, (int)std::ceil((q.x + dn - bb.min.x) / cell));
                    int b0 = std::max(j0, (int)std::floor((q.y - dn - bb.min.y) / cell)), b1 = std::min(j1, (int)std::ceil((q.y + dn - bb.min.y) / cell));
                    for (int j = b0; j <= b1; ++j) for (int i = a0; i <= a1; ++i) { double dx = cx(i) - q.x, dy = cy(j) - q.y; if (dx * dx + dy * dy <= dn * dn) near[(size_t)(j - j0) * wx + (i - i0)] = 1; } }
                std::set<int> S;
                for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) { size_t k = (size_t)j * nx + i; size_t w = (size_t)(j - j0) * wx + (i - i0);
                    if (near[w] && lab[k] >= 0 && signif(lab[k])) S.insert(lab[k]);
                    if (fAll[k] > del && v1[k] <= del && g1[k] == g && v2[k] > del) inB[w] = 1; }
                // B components -> union adjacent labels
                std::map<int, int> up; std::function<int(int)> uf = [&](int x) { auto it = up.find(x); if (it == up.end() || it->second == x) return x; int r0 = uf(it->second); up[x] = r0; return r0; };
                std::vector<char> seenB((size_t)wx * wy, 0);
                for (size_t w0 = 0; w0 < inB.size(); ++w0) {
                    if (!inB[w0] || seenB[w0]) continue;
                    std::vector<size_t> q{w0}; seenB[w0] = 1; std::set<int> adj;
                    while (!q.empty()) { size_t w = q.back(); q.pop_back(); int i = (int)(w % wx), j = (int)(w / wx);
                        const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
                        for (int e = 0; e < 4; ++e) { int ii = i + di[e], jj = j + dj[e]; if (ii < 0 || jj < 0 || ii >= wx || jj >= wy) continue; size_t ww = (size_t)jj * wx + ii; size_t kk = (size_t)(jj + j0) * nx + (ii + i0);
                            if (lab[kk] >= 0) adj.insert(lab[kk]); else if (inB[ww] && !seenB[ww]) { seenB[ww] = 1; q.push_back(ww); } } }
                    int first = -1; for (int L : adj) { if (first < 0) { first = L; continue; } int ra = uf(first), rb = uf(L); if (ra != rb) up[rb] = ra; }
                }
                int verdict;
                if (S.size() <= 1) verdict = 0;  // NOHOLD
                else { std::map<int, int> cnt; for (int L : S) cnt[uf(L)]++; int mx = 0; for (auto& [k, v] : cnt) mx = std::max(mx, v);
                    verdict = mx == (int)S.size() ? 1 : mx >= 2 ? 2 : 3; }  // HOLD, PARTIAL, SEVERED
                (pass == 0 ? tr[g].v0 : tr[g].v5) = verdict;
                if (verbose) { std::printf("  g%d pass%d mid (%.3f,%.3f) len %.3f S %zu verdict %d :", g, pass, tr[g].mid.x, tr[g].mid.y, tr[g].len, S.size(), verdict); for (int L : S) std::printf(" L%d(a%.2f b%d)", L, area[L] * cell * cell, (int)border[L]); std::printf("\n"); }
            }
            if (pass == 0) {  // FREE with all tabs present
                std::fill(lab.begin(), lab.end(), -1);
                for (size_t k0 = 0; k0 < NC; ++k0) {
                    if (lab[k0] >= 0 || fAll[k0] <= 0) continue;
                    long a = 0; bool bd = false; float mc = 0; double sx = 0, sy = 0; st.assign(1, k0); lab[k0] = 1;
                    while (!st.empty()) { size_t k = st.back(); st.pop_back(); ++a; mc = std::max(mc, fAll[k]); int i = (int)(k % nx), j = (int)(k / nx); sx += cx(i); sy += cy(j); if (i == 0 || j == 0 || i == nx - 1 || j == ny - 1) bd = true;
                        const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
                        for (int e = 0; e < 4; ++e) { int ii = i + di[e], jj = j + dj[e]; if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue; size_t kk = (size_t)jj * nx + ii; if (lab[kk] >= 0 || fAll[kk] <= 0) continue; lab[kk] = 1; st.push_back(kk); } }
                    if (!bd && a * cell * cell >= 0.01 && mc > 0.025 && verbose) std::printf("  FREECOMP area %.3f thick(2*maxclr) %.3f centroid (%.3f,%.3f) seedcell (%.3f,%.3f)\n", a * cell * cell, 2 * mc, sx / a, sy / a, cx((int)(k0 % nx)), cy((int)(k0 / nx)));
                    if (!bd && a * cell * cell >= 0.01 && mc > 0.025) { ++freeCnt; if (a * cell * cell > freeArea) { freeArea = a * cell * cell; freeAt = {sx / a, sy / a}; } }
                }
            }
        }
        int hold = 0, nohold = 0, sev = 0, part = 0, thin = 0;
        std::string flags;
        for (int g = 0; g < G; ++g) {
            auto& t = tr[g]; char b[200];
            if (t.v0 == 1) { ++hold; if (t.v5 == 2 || t.v5 == 3) { ++thin; snprintf(b, sizeof b, " [THIN g%d (%.3f,%.3f) len %.2f]", g, t.mid.x, t.mid.y, t.len); flags += b; } }
            else if (t.v0 == 0) ++nohold;
            else if (t.v0 == 2) { ++part; snprintf(b, sizeof b, " [PARTIAL g%d (%.3f,%.3f) len %.2f]", g, t.mid.x, t.mid.y, t.len); flags += b; }
            else { ++sev; snprintf(b, sizeof b, " [SEVERED g%d (%.3f,%.3f) len %.2f]", g, t.mid.x, t.mid.y, t.len); flags += b; }
        }
        if (freeCnt) { char b[200]; snprintf(b, sizeof b, " [FREE %d biggest %.2f mm2 at (%.2f,%.2f) allLost %zu]", freeCnt, freeArea, freeAt.x, freeAt.y, allLost.size()); flags += b; }
        T_tabs += G; T_hold += hold; T_nohold += nohold; T_sev += sev; T_part += part; T_thin += thin; T_free += freeCnt ? 1 : 0; T_left += leftOut; T_req += req;
        double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("S %lu %d side %d D %g V %d a %.4f sep %d lpa %.3f n %d len %.3f th %.3f 3d %d | G %d hold %d nohold %d sev %d part %d thin %d | left %d req %d | cell %.3f %.1fs%s\n",
                    seed0, sc, (int)p.side, D, vdeg, p.allowance, (int)p.separateLastPass, p.lastPassAllowance, p.tabs.count, p.tabs.length, p.tabs.thickness, (int)p.tabs.threeD, G, hold, nohold, sev, part, thin, leftOut, req, cell, secs, flags.c_str());
        if ((!flags.empty() || std::getenv("DUMPALL")) && std::getenv("DUMP")) { char fn[512]; snprintf(fn, sizeof fn, "%s/m%d_%lu_%d.scn", std::getenv("DUMP"), mode, seed0, sc); writeScene(fn, cs, p, D, vdeg); }
        std::fflush(stdout);
    }
    std::printf("SUMMARY seed %lu mode %d scenes %ld empty %ld | tabs %ld hold %ld nohold %ld severed %ld partial %ld thin %ld | freeScenes %ld | left %ld req %ld\n", seed0, mode, T_scn, T_empty, T_tabs, T_hold, T_nohold, T_sev, T_part, T_thin, T_free, T_left, T_req);
}
