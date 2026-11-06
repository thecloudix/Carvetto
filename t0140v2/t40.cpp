// T-0140 verifier (throwaway). Whole-scene flood oracle of the MOVES at the tab layer (h = zTab - 1e-4), independent of
// the planner (derived from t0158v5/wsf.cpp, tab-verdict machinery removed; only "free" components), plus a hash mode.
// usage: t40 seed N mode   modes: 0 On, 1 In/Out a in [-r,0], 2 mix (wsf), 3 all-mix incl. tabs off (hash),
//   4 In/Out a >= 0 (wsf families), 5 In/Out a >= 0 stress families (grid/waste, nested, tight arcs, V-bit sharp, Bezier,
//   letters, open vectors), 99 SCENE=file.  Leads are random unless NOLEAD=1 (the flood sweeps set it).
// env: HASH=1 (print H lines: moves hash, non-loose diagnostics hash, loose messages; no flood), CELLMIN (0.015), CAP (2.5e7),
//   FORCEIO=0|1 (stored scenes: Outside|Inside, negative allowance -> 0, tabs on), DUMP=dir (scenes with MISS/FALSE), DUMPALL, SC=k, V=1 (list every component), DEL (clearance band, 0).
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

struct Seg { Vec2 a, b; double za, zb; bool arc = false; Vec2 c{}; double rho = 0, a0 = 0, sw = 0; };
static double segDist(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0; t = std::clamp(t, 0.0, 1.0);
    double ex = p.x - a.x - t * dx, ey = p.y - a.y - t * dy; return std::sqrt(ex * ex + ey * ey);
}
static geometry::Box2 bx(const Contour& c) { geometry::Box2 b; for (auto& sp : c.spans) b.expand(geometry::bounds(sp)); return b; }
static unsigned long long fnv(const std::string& s) { unsigned long long h = 1469598103934665603ULL; for (unsigned char ch : s) { h ^= ch; h *= 1099511628211ULL; } return h; }

static void gen(int mode, std::vector<Contour>& cs, ProfileParams& p, double& D, int& vdeg) {
    bool useV = I(0, 4) == 0;
    if (mode == 5 && I(0, 5) == 0) useV = true;
    if (useV) { D = std::vector<double>{6.0, 12.0, 25.0}[(size_t)I(0, 2)]; vdeg = std::vector<int>{60, 90, 120}[(size_t)I(0, 2)]; }
    else D = std::vector<double>{2.0, 3.175, 6.0, 8.0, 12.0}[(size_t)I(0, 4)];
    int sk = mode == 0 ? 2 : (mode == 1 || mode >= 4) ? I(0, 1) : I(0, 2);
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
    tooling::Tool tt0 = useV ? vbit(D, vdeg) : endMill(D);
    double r = profileToolRadius(tt0, p.passes);
    if (mode <= 4 || mode == 99) {
        double R = U(8, 35);
        int f = I(0, 9);
        if (f <= 4) cs.push_back(anyClosed({0, 0}, R));
        else if (f == 5) { double s = U(10, 60); cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {s, s * U(0.5, 1.5)}})); if (I(0, 1)) p.tabs.count = 4, p.startAt = StartAt::ClosestOnBoundingBox, p.startAnchor = BoxAnchor::BottomCentre; }
        else if (f == 6) { double W = U(40, 100), H = U(30, 80); cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {W, H}}));
            int n = I(1, 3); for (int i = 0; i < n; ++i) { double rr = U(3, 12); Vec2 c{U(rr + 1, W - rr - 1), U(rr + 1, H - rr - 1)}; cs.push_back(I(0, 1) ? Contour::circle(c, rr) : Contour::rectangle(geometry::Box2{{c.x - rr, c.y - rr * 0.7}, {c.x + rr, c.y + rr * 0.7}})); } }
        else if (f == 7) { cs.push_back(anyClosed({0, 0}, R)); double gap = U(0, 3 * D); Contour c2 = anyClosed({0, 0}, U(6, 30)); double sh = bx(cs[0]).max.x + gap - bx(c2).min.x; cs.push_back(translate(c2, {sh, U(-5, 5)})); }
        else if (f == 8) cs.push_back(I(0, 1) ? bezBlob({0, 0}, R, I(4, 9)) : ellipse({0, 0}, U(5, 40), U(3, 20), U(0, PI)));
        else { cs.push_back(letter({0, 0}, U(8, 40))); if (I(0, 1)) cs.push_back(letter({50, 0}, U(8, 40))); }
    } else {  // mode 5 stress families
        int f = I(0, 7);
        if (f == 0) {  // grid of small parts: waste between vectors
            int nx = I(2, 4), ny = I(1, 3); double gap = U(0.1, 2.6) * D; double s = U(2.5, 12);
            for (int i = 0; i < nx; ++i) for (int j = 0; j < ny; ++j) {
                Vec2 c{i * (s + gap), j * (s + gap)}; int k = I(0, 3);
                if (k == 0) cs.push_back(Contour::rectangle(geometry::Box2{c, c + Vec2{s, s}}));
                else if (k == 1) cs.push_back(Contour::circle(c + Vec2{s / 2, s / 2}, s / 2));
                else if (k == 2) cs.push_back(translate(anyClosed({0, 0}, s / 2), c + Vec2{s / 2, s / 2}));
                else cs.push_back(translate(bezBlob({0, 0}, s / 2, I(4, 7)), c + Vec2{s / 2, s / 2}));
            }
        } else if (f == 1) {  // nested: plate, holes, a part inside each hole (and maybe a hole in that part)
            double W = U(30, 80), H = U(25, 60); cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {W, H}}));
            int n = I(1, 2);
            for (int i = 0; i < n; ++i) {
                double rr = U(6, 14); Vec2 c{(i + 0.5) * W / n, H / 2}; rr = std::min(rr, std::min(W / (2.0 * n), H / 2) - 1.5);
                if (rr < 3) continue;
                cs.push_back(I(0, 1) ? Contour::circle(c, rr) : Contour::rectangle(geometry::Box2{{c.x - rr, c.y - rr}, {c.x + rr, c.y + rr}}));
                double ri = rr - U(0.3, 2.5) * D; if (ri < 1.0) continue;
                cs.push_back(I(0, 1) ? Contour::circle(c, ri) : translate(anyClosed({0, 0}, ri * 0.8), c));
                double rh = ri - U(0.3, 2.5) * D; if (rh > 0.8 && I(0, 1)) cs.push_back(Contour::circle(c, rh));
            }
        } else if (f == 2) {  // arcs tighter than the band: small round shapes, slots, flowers with small radii
            int n = I(1, 6); double x = 0;
            for (int i = 0; i < n; ++i) { double rr = U(0.05, 1.6) * r + 0.05; int k = I(0, 2);
                Contour c = k == 0 ? Contour::circle({0, 0}, rr) : k == 1 ? slot({0, 0}, U(0.5, 4) * rr, rr, U(0, PI)) : flower({0, 0}, std::max(rr, 1.0) * U(1, 3), 2 * I(2, 4));
                geometry::Box2 b = bx(c); cs.push_back(translate(c, {x - b.min.x, U(-2, 2)})); x += b.width() + U(0.1, 2.5) * D; }
        } else if (f == 3) {  // V-bit sharp corners
            if (!useV) { useV = true; D = std::vector<double>{6.0, 12.0, 25.0}[(size_t)I(0, 2)]; vdeg = std::vector<int>{60, 90, 120}[(size_t)I(0, 2)]; p.passes.cutDepth = U(1, 6); p.tabs.thickness = U(0.2, 0.9 * std::min(6.0, p.passes.cutDepth)); }
            p.corners.sharpExternal = I(0, 1); p.corners.sharpInternal = I(0, 1);
            double R = U(6, 30); int k = I(0, 3);
            cs.push_back(k == 0 ? spikes({0, 0}, R, I(3, 7)) : k == 1 ? gear({0, 0}, R, I(5, 12), R * U(0.1, 0.35)) : k == 2 ? letter({0, 0}, R) : chamfered({0, 0}, 2 * R, R, U(0.3, R * 0.3)));
            if (I(0, 1)) { Contour c2 = anyClosed({0, 0}, U(5, 20)); double sh = bx(cs[0]).max.x + U(0, 2 * D) - bx(c2).min.x; cs.push_back(translate(c2, {sh, U(-5, 5)})); }
        } else if (f == 4) {  // Bezier-heavy
            int n = I(1, 4); double x = 0;
            for (int i = 0; i < n; ++i) { double R = U(3, 25); Contour c = I(0, 1) ? bezBlob({0, 0}, R, I(4, 12)) : ellipse({0, 0}, R, R * U(0.2, 1), U(0, PI));
                geometry::Box2 b = bx(c); cs.push_back(translate(c, {x - b.min.x, U(-3, 3)})); x += b.width() + U(0, 2.5) * D; }
        } else if (f == 5) {  // letters close together (text)
            int n = I(2, 5); double x = 0, s = U(6, 25);
            for (int i = 0; i < n; ++i) { Contour c = letter({0, 0}, s); geometry::Box2 b = bx(c); cs.push_back(translate(c, {x - b.min.x, 0})); x += b.width() + U(0, 2.5) * D; }
        } else if (f == 6) {  // open vectors mixed with closed ones
            cs.push_back(anyClosed({0, 0}, U(8, 30)));
            int n = I(1, 3); for (int i = 0; i < n; ++i) cs.push_back(I(0, 2) ? anyOpen({U(-30, 30), U(-30, 30)}, U(8, 30)) : bezWave({U(-30, 30), U(-30, 30)}, U(10, 50), I(2, 6)));
        } else {  // concentric rings (part inside a hole of a part, many levels)
            int n = I(2, 5); double rr = U(10, 40); bool sq = I(0, 1);
            for (int i = 0; i < n && rr > 1; ++i) { cs.push_back(sq ? Contour::rectangle(geometry::Box2{{-rr, -rr}, {rr, rr}}) : Contour::circle({0, 0}, rr)); rr -= U(0.3, 3) * D; }
        }
    }
    tooling::Tool tt = useV ? vbit(D, vdeg) : endMill(D);
    r = profileToolRadius(tt, p.passes);
    if (mode == 3) {
        int k = I(0, 11);
        p.allowance = k < 4 ? U(0, 1) : k < 6 ? 0.0 : k < 8 ? -r * U(0, 1) : k == 8 ? -r : k == 9 ? -0.0 : k == 10 ? -r + U(0, 0.5) : U(0, 0.05);
        if (I(0, 3) == 0) p.tabs.enabled = false;
    } else if (mode >= 4) {
        int k = I(0, 7);
        p.allowance = k < 2 ? 0.0 : k == 2 ? -0.0 : k == 3 ? U(0, 0.05) : k <= 5 ? U(0, 1.0) : U(0, 3.0);
    } else if (p.side == ProfileSide::On) {
        int k = I(0, 7);
        p.allowance = k < 2 ? 0.0 : k < 5 ? U(-0.6, 0.6) : U(-0.95, 0.95) * r;
    } else {
        int k = I(0, 7);
        p.allowance = k == 0 ? -r : k <= 3 ? std::min(0.0, -r + U(0, 0.5)) : k <= 6 ? -r * U(0, 1) : -U(0, 0.02);
    }
}

int main(int argc, char** argv) {
    unsigned long seed0 = std::strtoul(argv[1], nullptr, 0); int N = std::atoi(argv[2]); int mode = std::atoi(argv[3]);
    double CELLMIN = std::getenv("CELLMIN") ? std::atof(std::getenv("CELLMIN")) : 0.015;
    double CAP = std::getenv("CAP") ? std::atof(std::getenv("CAP")) : 2.5e7;
    bool verbose = std::getenv("V") != nullptr; int only = std::getenv("SC") ? std::atoi(std::getenv("SC")) : -1;
    const double DEL = std::getenv("DEL") ? std::atof(std::getenv("DEL")) : 0.0;
    long T_scn = 0, T_free = 0, T_rep = 0, T_miss = 0, T_false = 0, T_loose = 0, T_thin = 0;
    for (int sc = 0; sc < N; ++sc) {
        rng.seed(seed0 * 1000003ULL + (unsigned long)sc);
        if (only >= 0 && sc != only) continue;
        auto t0 = std::chrono::steady_clock::now();
        ProfileParams p; std::vector<Contour> cs; double D = 6; int vdeg = 0;
        if (mode == 99) readScene(std::getenv("SCENE"), cs, p, D, vdeg);
        else gen(mode, cs, p, D, vdeg);
        if (std::getenv("NOLEAD")) { p.leads.type = LeadType::None; p.leads.overcut = 0; p.leads.leadOut = false; p.ramp.onLeadIn = false; }
        if (std::getenv("NORAMP")) p.ramp.type = (RampType)0;
        if (const char* fs = std::getenv("FORCEIO")) { p.side = std::atoi(fs) ? ProfileSide::Inside : ProfileSide::Outside; if (!(p.allowance >= 0)) p.allowance = 0.0; p.tabs.enabled = true; }
        if (std::getenv("NOSEP")) p.separateLastPass = false;
        tooling::Tool tool = vdeg ? vbit(D, vdeg) : endMill(D);
        auto t1 = std::chrono::steady_clock::now();
        auto res = profileToolpath(cs, tool, p);
        double tPlan = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
        double r = profileToolRadius(tool, p.passes);
        if (std::getenv("HDUMP")) { char fn[512]; snprintf(fn, sizeof fn, "%s/m%d_%lu_%d.scn", std::getenv("HDUMP"), mode, seed0, sc); writeScene(fn, cs, p, D, vdeg); }
        if (std::getenv("HASH")) {
            std::string mo, dg, lo; char b[400]; int nl = 0;
            for (auto& m : res.toolpath.moves) { snprintf(b, sizeof b, "%d %a %a %a %a %a|", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y); mo += b; }
            for (auto& d : res.diagnostics) {
                std::string s = std::to_string((int)d.issue) + "/" + (d.contour ? std::to_string(*d.contour) : std::string("-")) + "/" + d.message + "\n";
                if (d.message.find("loose piece") != std::string::npos) { lo += s; ++nl; } else dg += s;
            }
            const char* cls = p.side == ProfileSide::On ? "On" : (p.allowance < 0 ? "IOneg" : (std::signbit(p.allowance) ? "IOm0" : "IOpos"));
            std::printf("H %lu %d %s V %d tabs %d sep %d nv %zu | %016llx %zu %016llx %d %016llx | %.3fs\n", seed0, sc, cls, vdeg, (int)p.tabs.enabled, (int)p.separateLastPass, cs.size(), fnv(mo), res.toolpath.moves.size(), fnv(dg), nl, fnv(lo), tPlan);
            std::fflush(stdout);
            continue;
        }
        std::vector<double> rep;
        for (auto& d : res.diagnostics) if (d.message.find("comes free") != std::string::npos) { auto q = d.message.find("(about "); rep.push_back(q != std::string::npos ? std::atof(d.message.c_str() + q + 7) : -1); }
        if (std::getenv("DIAG")) for (auto& d : res.diagnostics) std::printf("  DIAG %d v%d %s\n", (int)d.issue, d.contour ? (int)*d.contour : -1, d.message.c_str());
        bool moreMsg = false; for (auto& d : res.diagnostics) if (d.message.find("more loose pieces") != std::string::npos) moreMsg = true;
        const auto& mv = res.toolpath.moves;
        ++T_scn;
        if (mv.empty()) { std::printf("S %lu %d empty\n", seed0, sc); continue; }
        const double bottom = p.passes.bottomZ();
        const double zTab = bottom + std::min(p.tabs.thickness, p.passes.cutDepth), h = zTab - 1e-4;
        const double Rt = D / 2, tanH = vdeg ? std::tan(vdeg * PI / 360) : 0;
        auto rad = [&](double z) { if (z >= h) return -1.0; return vdeg ? std::min(Rt, (h - z) * tanH) : Rt; };
        std::vector<Seg> segs;
        geometry::Vec3 pp = mv[0].to;
        for (size_t i = 1; i < mv.size(); ++i) {
            const auto& m = mv[i];
            if (!isCutting(m.kind)) { pp = m.to; continue; }
            if (isArc(m.kind)) {
                double r0 = std::hypot(pp.x - m.centre.x, pp.y - m.centre.y);
                double a0 = std::atan2(pp.y - m.centre.y, pp.x - m.centre.x), a1 = std::atan2(m.to.y - m.centre.y, m.to.x - m.centre.x);
                double sw = m.kind == MoveKind::ArcCCW ? a1 - a0 : a0 - a1; sw = std::fmod(sw, 2 * PI); if (sw < 0) sw += 2 * PI; if (sw < 1e-12) sw = 2 * PI;
                if (m.kind == MoveKind::ArcCW) sw = -sw;
                double da = std::min(0.4, 1.0 / std::max(r0, 1e-9));
                int n = std::max(1, (int)std::ceil(std::fabs(sw) / da));
                geometry::Vec3 prev = pp;
                for (int k = 1; k <= n; ++k) { double a = a0 + sw * k / n; geometry::Vec3 q{m.centre.x + r0 * std::cos(a), m.centre.y + r0 * std::sin(a), pp.z + (m.to.z - pp.z) * k / n}; if (k == n) q = m.to;
                    Seg sg{{prev.x, prev.y}, {q.x, q.y}, prev.z, q.z}; sg.arc = true; sg.c = m.centre; sg.rho = r0; sg.a0 = a0 + sw * (k - 1) / n; sg.sw = sw / n; segs.push_back(sg); prev = q; }
                pp = m.to; continue;
            }
            double L = std::hypot(m.to.x - pp.x, m.to.y - pp.y); int n = std::max(1, (int)std::ceil(L / 1.0));
            geometry::Vec3 prev = pp;
            for (int k = 1; k <= n; ++k) { geometry::Vec3 q{pp.x + (m.to.x - pp.x) * k / n, pp.y + (m.to.y - pp.y) * k / n, pp.z + (m.to.z - pp.z) * k / n}; segs.push_back({{prev.x, prev.y}, {q.x, q.y}, prev.z, q.z}); prev = q; }
            pp = m.to;
        }
        geometry::Box2 bb;
        for (auto& s : segs) { if (s.za >= h && s.zb >= h) continue; bb.expand(s.a); bb.expand(s.b); }
        if (bb.isEmpty()) { std::printf("S %lu %d nocut\n", seed0, sc); continue; }
        bb = bb.inflated(Rt + 2.0);
        double cell = std::max(CELLMIN, std::sqrt(bb.width() * bb.height() / CAP));
        int nx = (int)std::ceil(bb.width() / cell), ny = (int)std::ceil(bb.height() / cell);
        size_t NC = (size_t)nx * ny;
        std::vector<float> fAll(NC, 1e9f);
        const double DM = 0.06;
        auto cx = [&](int i) { return bb.min.x + (i + 0.5) * cell; }; auto cy = [&](int j) { return bb.min.y + (j + 0.5) * cell; };
        auto segD = [&](Vec2 q, const Seg* sa, Vec2 a, Vec2 b) {
            if (!sa) return segDist(q, a, b);
            Vec2 v = q - sa->c; Vec2 u0{std::cos(sa->a0), std::sin(sa->a0)};
            double ang = std::atan2(u0.x * v.y - u0.y * v.x, u0.x * v.x + u0.y * v.y);
            bool in = sa->sw > 0 ? (ang >= 0 && ang <= sa->sw) : (ang <= 0 && ang >= sa->sw);
            if (in) return std::fabs(std::sqrt(v.x * v.x + v.y * v.y) - sa->rho);
            return std::min(geometry::distance(q, a), geometry::distance(q, b));
        };
        auto stamp = [&](Vec2 a, Vec2 b, double rr, const Seg* sa = nullptr) {
            double ext = rr + DM + (sa ? sa->rho * (1 - std::cos(sa->sw / 2)) + 1e-9 : 0);
            int i0 = std::max(0, (int)std::floor((std::min(a.x, b.x) - ext - bb.min.x) / cell)), i1 = std::min(nx - 1, (int)std::ceil((std::max(a.x, b.x) + ext - bb.min.x) / cell));
            int j0 = std::max(0, (int)std::floor((std::min(a.y, b.y) - ext - bb.min.y) / cell)), j1 = std::min(ny - 1, (int)std::ceil((std::max(a.y, b.y) + ext - bb.min.y) / cell));
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
                double d = segD({cx(i), cy(j)}, sa, a, b) - rr; if (d > DM) continue;
                size_t k = (size_t)j * nx + i; fAll[k] = std::min(fAll[k], (float)d);
            }
        };
        for (auto& s : segs) {
            if (s.za >= h && s.zb >= h) continue;
            if (s.arc && s.za < h && s.zb < h && (!vdeg || std::fabs(s.za - s.zb) < 1e-9)) { stamp(s.a, s.b, vdeg ? rad(std::min(s.za, s.zb)) : Rt, &s); continue; }
            if (s.arc) {
                double L = std::fabs(s.sw) * s.rho; int n = std::max(1, (int)std::ceil(L / 0.005));
                for (int k = 0; k < n; ++k) { double t0a = (double)k / n, t1a = (double)(k + 1) / n; double z0 = s.za + (s.zb - s.za) * t0a, z1 = s.za + (s.zb - s.za) * t1a; double zm = std::min(z0, z1); if (zm >= h) continue; double rr = rad(zm);
                    Vec2 p0 = s.c + Vec2{std::cos(s.a0 + s.sw * t0a), std::sin(s.a0 + s.sw * t0a)} * s.rho, p1 = s.c + Vec2{std::cos(s.a0 + s.sw * t1a), std::sin(s.a0 + s.sw * t1a)} * s.rho;
                    if (rr > 0) stamp(p0, p1, rr); }
                continue;
            }
            Vec2 a = s.a, b = s.b; double za = s.za, zb = s.zb;
            if (za >= h || zb >= h) { double t = (h - za) / (zb - za); Vec2 c = a + (b - a) * t; if (za >= h) { a = c; za = h - 1e-12; } else { b = c; zb = h - 1e-12; } }
            double L = geometry::distance(a, b);
            if (!vdeg || std::fabs(za - zb) < 1e-9 || L < 1e-9) { double rr = vdeg ? rad(std::min(za, zb)) : Rt; if (rr > 0) stamp(a, b, rr); continue; }
            std::vector<double> ts{0.0, 1.0};
            { double zc = h - Rt / tanH; if ((za - zc) * (zb - zc) < 0) ts.insert(ts.begin() + 1, (zc - za) / (zb - za)); }
            for (size_t q = 0; q + 1 < ts.size(); ++q) {
                Vec2 p0 = a + (b - a) * ts[q], p1 = a + (b - a) * ts[q + 1];
                double r0 = std::max(rad(za + (zb - za) * ts[q]), 0.0), r1 = std::max(rad(za + (zb - za) * ts[q + 1]), 0.0);
                double Ls = geometry::distance(p0, p1); if (Ls < 1e-12) { stamp(p0, p1, std::max(r0, r1)); continue; }
                double kap = (r1 - r0) / Ls; Vec2 dir = (p1 - p0) * (1.0 / Ls);
                double rmax = std::max(r0, r1), ext = rmax + DM;
                int i0 = std::max(0, (int)std::floor((std::min(p0.x, p1.x) - ext - bb.min.x) / cell)), i1 = std::min(nx - 1, (int)std::ceil((std::max(p0.x, p1.x) + ext - bb.min.x) / cell));
                int j0 = std::max(0, (int)std::floor((std::min(p0.y, p1.y) - ext - bb.min.y) / cell)), j1 = std::min(ny - 1, (int)std::ceil((std::max(p0.y, p1.y) + ext - bb.min.y) / cell));
                double sh = std::fabs(kap) < 1 ? kap / std::sqrt(1 - kap * kap) : 0;
                for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
                    Vec2 pc{cx(i), cy(j)}; Vec2 v = pc - p0; double u = v.x * dir.x + v.y * dir.y; double w = std::fabs(v.x * dir.y - v.y * dir.x);
                    auto fs = [&](double s2) { double e = s2 - u; return std::sqrt(e * e + w * w) - (r0 + kap * s2); };
                    double d = std::fabs(kap) >= 1 ? std::min(fs(0), fs(Ls)) : fs(std::clamp(u + sh * w, 0.0, Ls));
                    if (d > DM) continue;
                    size_t k = (size_t)j * nx + i; fAll[k] = std::min(fAll[k], (float)d);
                }
            }
        }
        struct Comp { double area, thick, x, y; };
        std::vector<Comp> fr, thin;
        std::vector<char> lab2;
        {
            std::vector<char> lab(NC, 0); std::vector<size_t> st;
            for (size_t k0 = 0; k0 < NC; ++k0) {
                if (lab[k0] || fAll[k0] <= DEL) continue;
                long a = 0; bool bd = false; float mc = 0; double sx = 0, sy = 0; st.assign(1, k0); lab[k0] = 1;
                while (!st.empty()) { size_t k = st.back(); st.pop_back(); ++a; mc = std::max(mc, fAll[k]); int i = (int)(k % nx), j = (int)(k / nx); sx += cx(i); sy += cy(j); if (i == 0 || j == 0 || i == nx - 1 || j == ny - 1) bd = true;
                    const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
                    for (int e = 0; e < 4; ++e) { int ii = i + di[e], jj = j + dj[e]; if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue; size_t kk = (size_t)jj * nx + ii; if (lab[kk] || fAll[kk] <= DEL) continue; lab[kk] = 1; st.push_back(kk); } }
                if (std::getenv("PGM") && !bd) { for (size_t q = 0; q < NC; ++q) if (lab[q] == 1 && fAll[q] > DEL) lab[q] = 2; }
                if (std::getenv("PGM")) for (size_t q = 0; q < NC; ++q) if (lab[q] == 1) lab[q] = 3;
                if (bd) continue;
                double A = a * cell * cell;
                if (verbose) std::printf("  COMP area %.4f thick %.4f at (%.3f,%.3f)\n", A, 2 * mc, sx / a, sy / a);
                if (A >= 0.03 && mc > 0.025) fr.push_back({A, 2.0 * std::min(mc, 50.f), sx / a, sy / a});
                else if (A >= 0.03) thin.push_back({A, 2.0 * mc, sx / a, sy / a});
            }
            if (std::getenv("PGM")) lab2.swap(lab);
        }
        if (const char* pg = std::getenv("PGM")) {  // free pieces gray, other uncut white, cut black (labels: 2 free, 3 not free)
            int ds = std::max(1, (int)std::ceil(std::max(nx, ny) / 1400.0)); int W = nx / ds, Hh = ny / ds; FILE* f = std::fopen(pg, "wb"); std::fprintf(f, "P5 %d %d 255\n", W, Hh);
            for (int j = Hh - 1; j >= 0; --j) for (int i = 0; i < W; ++i) { int c = 0; for (int dj = 0; dj < ds; ++dj) for (int di = 0; di < ds; ++di) { size_t k = (size_t)(j * ds + dj) * nx + (i * ds + di); if (fAll[k] > DEL) c = std::max(c, lab2[k] == 2 ? 1 : 2); } std::fputc(c == 0 ? 0 : c == 1 ? 128 : 255, f); }
            std::fclose(f); std::printf("PGM %s ds %d cell %.4f origin (%.3f,%.3f)\n", pg, ds, cell, bb.min.x, bb.min.y);
        }
        std::sort(fr.begin(), fr.end(), [](const Comp& x, const Comp& y) { return x.area > y.area; });
        // match: tight (3% + 0.05), then loose (reported 70%..103% of the flood, V-bit ramps)
        std::vector<int> used(rep.size(), 0), fm(fr.size(), 0);
        for (int pass = 0; pass < 2; ++pass)
            for (size_t i = 0; i < fr.size(); ++i) { if (fm[i]) continue; double f = fr[i].area; int best = -1; double bd = 1e18;
                for (size_t j = 0; j < rep.size(); ++j) { if (used[j]) continue; double g = rep[j];
                    bool ok = pass == 0 ? std::fabs(g - f) <= 0.03 * f + 0.05 : (g >= 0.7 * f - 0.05 && g <= 1.03 * f + 0.05);
                    if (ok && std::fabs(g - f) < bd) { bd = std::fabs(g - f); best = (int)j; } }
                if (best >= 0) { used[best] = 1 + pass; fm[i] = 1 + pass; } }
        std::string flags; char b[300]; int nMiss = 0, nFalse = 0, nLoose = 0, nFree = 0;
        for (size_t i = 0; i < fr.size(); ++i) {
            if (fr[i].area < 0.04) { if (fm[i]) { snprintf(b, sizeof b, " [SUB04M %.4f]", fr[i].area); flags += b; } continue; }
            ++nFree; if (fm[i] == 2) ++nLoose;
            if (!fm[i]) { ++nMiss; snprintf(b, sizeof b, " [MISS %.4f th %.4f @(%.3f,%.3f)]", fr[i].area, fr[i].thick, fr[i].x, fr[i].y); flags += b; }
        }
        for (size_t j = 0; j < rep.size(); ++j) if (!used[j]) { ++nFalse; snprintf(b, sizeof b, " [FALSE %.3f]", rep[j]); flags += b; }
        for (auto& t : thin) if (t.area >= 0.04) { snprintf(b, sizeof b, " [THIN %.4f th %.4f @(%.3f,%.3f)]", t.area, t.thick, t.x, t.y); flags += b; ++T_thin; }
        if (moreMsg) flags += " [MOREMSG]";
        std::string rl; for (double g : rep) { snprintf(b, sizeof b, " %.3f", g); rl += b; }
        std::string fl; for (auto& c : fr) { snprintf(b, sizeof b, " %.3f/%.3f", c.area, c.thick); fl += b; }
        T_free += nFree; T_rep += (long)rep.size(); T_miss += nMiss; T_false += nFalse; T_loose += nLoose;
        double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("S %lu %d m%d side %d D %g V %d a %.4f sep %d lpa %.3f nv %zu tabs %d len %.3f th %.3f | free %d rep %zu miss %d false %d loose %d | rep:%s | flood:%s | cell %.4f plan %.2fs %.1fs%s\n",
                    seed0, sc, mode, (int)p.side, D, vdeg, p.allowance, (int)p.separateLastPass, p.lastPassAllowance, cs.size(), p.tabs.count, p.tabs.length, p.tabs.thickness,
                    nFree, rep.size(), nMiss, nFalse, nLoose, rl.substr(0, 300).c_str(), fl.substr(0, 300).c_str(), cell, tPlan, secs, flags.c_str());
        if ((nMiss || nFalse || std::getenv("DUMPALL")) && std::getenv("DUMP")) { char fn[512]; snprintf(fn, sizeof fn, "%s/m%d_%lu_%d.scn", std::getenv("DUMP"), mode, seed0, sc); writeScene(fn, cs, p, D, vdeg); }
        std::fflush(stdout);
    }
    std::printf("SUMMARY seed %lu mode %d scenes %ld free %ld rep %ld miss %ld false %ld loose %ld thin %ld\n", seed0, mode, T_scn, T_free, T_rep, T_miss, T_false, T_loose, T_thin);
}
