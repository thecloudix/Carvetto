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
#include "clipper2/clipper.h"
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
    return [&]{ double ex = p.x - a.x - t * dx, ey = p.y - a.y - t * dy; return std::sqrt(ex * ex + ey * ey); }();
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
        if (mode == 99) { readScene(std::getenv("SCENE"), cs, p, D, vdeg); if (std::getenv("NOSHARP")) { std::printf("sharp ext %d int %d ramp %d\n", (int)p.corners.sharpExternal, (int)p.corners.sharpInternal, (int)p.ramp.type); p.corners.sharpExternal = p.corners.sharpInternal = false; } if (std::getenv("NORAMP")) p.ramp.type = (RampType)0; if (std::getenv("NOTABS")) p.tabs.enabled = false; }
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
            if (mode == 3) {
                int k = I(0, 11);
                p.allowance = k < 4 ? U(0, 1) : k < 6 ? 0.0 : k < 8 ? -r * U(0, 1) : k == 8 ? -r : k == 9 ? -0.0 : k == 10 ? -r + U(0, 0.5) : U(0, 0.05);
                if (I(0, 3) == 0) p.tabs.enabled = false;
            } else if (p.side == ProfileSide::On) {
                int k = I(0, 7);
                p.allowance = k < 2 ? 0.0 : k < 5 ? U(-0.6, 0.6) : U(-0.95, 0.95) * r;
            } else {
                int k = I(0, 7);
                p.allowance = k == 0 ? -r : k <= 3 ? std::min(0.0, -r + U(0, 0.5)) : k <= 6 ? -r * U(0, 1) : -U(0, 0.02);
            }
        }
        if (std::getenv("NOLEAD")) { p.leads.type = LeadType::None; p.leads.overcut = 0; p.leads.leadOut = false; p.ramp.onLeadIn = false; }
        if (std::getenv("NOSEP")) p.separateLastPass = false;
        if (std::getenv("FORCESEP")) p.separateLastPass = true;
        if (std::getenv("FORCEIOPOS")) { p.side = I(0, 1) ? ProfileSide::Outside : ProfileSide::Inside; int k = I(0, 3); p.allowance = k == 0 ? 0.0 : k == 1 ? U(0, 0.05) : k == 2 ? U(0, 1.5) : -0.0; p.tabs.enabled = true; p.separateLastPass = true; }
        tooling::Tool tool = vdeg ? vbit(D, vdeg) : endMill(D);
        auto res = profileToolpath(cs, tool, p);
        int nRep = 0; double repMax = 0; std::string repList;
        for (auto& d : res.diagnostics) if (d.message.find("comes free") != std::string::npos) { ++nRep; double a = 0; auto q = d.message.find("(about "); if (q != std::string::npos) a = std::atof(d.message.c_str() + q + 7); repMax = std::max(repMax, a); char rb[64]; snprintf(rb, sizeof rb, " %.2f", a); repList += rb; }
        const auto& mv = res.toolpath.moves;
        ++T_scn;
        int leftOut = 0, req = 0; std::set<size_t> allLost;
        for (auto& d : res.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced) { int a, b; if (std::sscanf(d.message.c_str() + d.message.find(": ") + 2, "%d of %d", &a, &b) == 2) { leftOut += a; req += b; if (a == b && d.contour) allLost.insert(*d.contour); } }
        if (mv.empty()) { ++T_empty; std::printf("S %lu %d empty\n", seed0, sc); continue; }
        const double bottom = p.passes.bottomZ();
        const double zTab = bottom + std::min(p.tabs.thickness, p.passes.cutDepth), h = zTab - 1e-4;
        const double Rt = D / 2, tanH = vdeg ? std::tan(vdeg * PI / 360) : 0;
        const double WID = std::getenv("WIDEN") ? std::atof(std::getenv("WIDEN")) : 0.0; auto rad = [&](double z) { if (z >= h) return -1.0; return (vdeg ? std::min(Rt, (h - z) * tanH) : Rt) + WID; };
        const double SC = 1e5, SAG = 5e-4;
        Clipper2Lib::Paths64 caps;
        auto P = [&](double x, double y) { return Clipper2Lib::Point64((int64_t)std::llround(x * SC), (int64_t)std::llround(y * SC)); };
        // runs of constant radius are inflated as open polylines (round ends = the swept region)
        std::vector<Clipper2Lib::Point64> run; double runR = -1;
        auto flush = [&]() {
            if (run.size() >= 1 && runR > 0) {
                Clipper2Lib::Path64 path(run.begin(), run.end()); if (path.size() == 1) path.push_back(path[0]);
                Clipper2Lib::Paths64 o = Clipper2Lib::InflatePaths(Clipper2Lib::Paths64{path}, (runR + SAG) * SC, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Round, 2.0, SAG * SC * 0.5);
                for (auto& q : o) caps.push_back(std::move(q));
            }
            run.clear(); runR = -1;
        };
        auto capsule = [&](Vec2 a, Vec2 b, double r) {
            if (r <= 0) { flush(); return; }
            Clipper2Lib::Point64 A = P(a.x, a.y), B = P(b.x, b.y);
            if (std::fabs(r - runR) > 1e-12 || run.empty() || !(run.back() == A)) { flush(); run.push_back(A); runR = r; }
            if (!(run.back() == B)) run.push_back(B);
        };
        auto piece = [&](Vec2 a, double za, Vec2 b, double zb) {
            if (za >= h && zb >= h) return;
            if (za >= h || zb >= h) { double t = (h - za) / (zb - za); Vec2 c = a + (b - a) * t; if (za >= h) { a = c; za = h - 1e-12; } else { b = c; zb = h - 1e-12; } }
            if (!vdeg || std::fabs(za - zb) < 1e-9) { capsule(a, b, vdeg ? rad(std::min(za, zb)) : Rt + WID); return; }
            // tapered capsule = convex hull of the two end discs (exact for linear centre and radius); split where the cone caps at Rt
            std::vector<double> ts{0.0, 1.0};
            { double zc = h - Rt / tanH; if ((za - zc) * (zb - zc) < 0) ts.insert(ts.begin() + 1, (zc - za) / (zb - za)); }
            flush();
            for (size_t k = 0; k + 1 < ts.size(); ++k) {
                Vec2 p0 = a + (b - a) * ts[k], p1 = a + (b - a) * ts[k + 1];
                double r0 = std::max(0.0, rad(za + (zb - za) * ts[k])), r1 = std::max(0.0, rad(za + (zb - za) * ts[k + 1]));
                std::vector<std::pair<double, double>> pts;
                for (int e = 0; e < 2; ++e) { Vec2 c = e ? p1 : p0; double r = e ? r1 : r0; if (r <= 0) { pts.push_back({c.x, c.y}); continue; }
                    double da = 2 * std::acos(std::max(-1.0, 1 - SAG / std::max(r, SAG))); da = std::min(da, PI / 8); int n = (int)std::ceil(2 * PI / da); double R = r / std::cos(PI / n);
                    for (int i = 0; i < n; ++i) { double t = 2 * PI * i / n; pts.push_back({c.x + R * std::cos(t), c.y + R * std::sin(t)}); } }
                std::sort(pts.begin(), pts.end());
                auto cr = [](std::pair<double,double> o, std::pair<double,double> a2, std::pair<double,double> b2) { return (a2.first - o.first) * (b2.second - o.second) - (a2.second - o.second) * (b2.first - o.first); };
                std::vector<std::pair<double,double>> H(2 * pts.size()); size_t kk = 0;
                for (size_t i = 0; i < pts.size(); ++i) { while (kk >= 2 && cr(H[kk - 2], H[kk - 1], pts[i]) <= 0) --kk; H[kk++] = pts[i]; }
                for (size_t i = pts.size() - 1, t2 = kk + 1; i-- > 0;) { while (kk >= t2 && cr(H[kk - 2], H[kk - 1], pts[i]) <= 0) --kk; H[kk++] = pts[i]; }
                H.resize(kk > 0 ? kk - 1 : 0);
                if (H.size() >= 3) { Clipper2Lib::Path64 path; for (auto& q2 : H) path.push_back(P(q2.first, q2.second)); caps.push_back(path); }
            }
        };
        geometry::Vec3 pp = mv[0].to;
        for (size_t i = 1; i < mv.size(); ++i) {
            const auto& m = mv[i];
            if (!isCutting(m.kind)) { flush(); pp = m.to; continue; }
            if (isArc(m.kind)) {
                double r0 = std::hypot(pp.x - m.centre.x, pp.y - m.centre.y);
                double a0 = std::atan2(pp.y - m.centre.y, pp.x - m.centre.x), a1 = std::atan2(m.to.y - m.centre.y, m.to.x - m.centre.x);
                double sw = m.kind == MoveKind::ArcCCW ? a1 - a0 : a0 - a1; sw = std::fmod(sw, 2 * PI); if (sw < 0) sw += 2 * PI; if (sw < 1e-12) sw = 2 * PI;
                if (m.kind == MoveKind::ArcCW) sw = -sw;
                double da = r0 > 1e-4 ? 2 * std::acos(std::max(-1.0, 1 - 1e-4 / r0)) : PI / 4;
                int n = std::max(1, (int)std::ceil(std::fabs(sw) / da));
                geometry::Vec3 prev = pp;
                for (int k = 1; k <= n; ++k) { double a = a0 + sw * k / n; geometry::Vec3 q{m.centre.x + r0 * std::cos(a), m.centre.y + r0 * std::sin(a), pp.z + (m.to.z - pp.z) * k / n}; if (k == n) q = m.to;
                    // chord under-covers the arc by <= 1e-4: grow the radius by that
                    if (!(prev.z >= h && q.z >= h)) { double za = prev.z, zb = q.z; Vec2 A{prev.x, prev.y}, B{q.x, q.y};
                        if (!vdeg || std::fabs(za - zb) < 1e-9) { double rr = (vdeg ? rad(std::min(za, zb)) : Rt + WID); if (rr > 0) capsule(A, B, rr + 1e-4); } else piece(A, za, B, zb); }
                    prev = q; }
                pp = m.to; continue;
            }
            piece({pp.x, pp.y}, pp.z, {m.to.x, m.to.y}, m.to.z);
            pp = m.to;
        }
        flush();
        Clipper2Lib::Clipper64 cl; cl.AddSubject(caps); Clipper2Lib::PolyTree64 tree; cl.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, tree);
        int free04 = 0, free01 = 0; std::string fl;
        std::function<void(const Clipper2Lib::PolyPath64&)> walk = [&](const Clipper2Lib::PolyPath64& node) {
            for (size_t c = 0; c < node.Count(); ++c) {
                const auto& ch = *node.Child(c);
                if (ch.IsHole()) {
                    double A = std::fabs(Clipper2Lib::Area(ch.Polygon())) / (SC * SC);
                    Clipper2Lib::Paths64 isl; for (size_t g = 0; g < ch.Count(); ++g) { A -= std::fabs(Clipper2Lib::Area(ch.Child(g)->Polygon())) / (SC * SC); isl.push_back(ch.Child(g)->Polygon()); }
                    if (std::getenv("VALL") && A > 1e-4) { Clipper2Lib::Path64 hp0 = ch.Polygon(); double cx0 = 0, cy0 = 0; for (auto& q : hp0) { cx0 += q.x; cy0 += q.y; } std::printf("  HOLE area %.4f n %zu at (%.3f,%.3f) islands %zu\n", A, hp0.size(), cx0 / hp0.size() / SC, cy0 / hp0.size() / SC, ch.Count()); }
                    if (A >= 0.01) {
                        Clipper2Lib::Path64 hp = ch.Polygon(); if (Clipper2Lib::Area(hp) < 0) std::reverse(hp.begin(), hp.end());
                        Clipper2Lib::Paths64 reg = Clipper2Lib::Difference(Clipper2Lib::Paths64{hp}, isl, Clipper2Lib::FillRule::NonZero);
                        Clipper2Lib::Paths64 er = Clipper2Lib::InflatePaths(reg, -0.025 * SC, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon, 2.0, 0.0001 * SC);
                        double ea = 0; for (auto& q : er) ea += Clipper2Lib::Area(q);
                        if (ea > 0) { ++free01; if (A >= 0.04) { ++free04; double cx0 = 0, cy0 = 0; for (auto& q : hp) { cx0 += q.x; cy0 += q.y; } char b[96]; snprintf(b, sizeof b, " %.3f@(%.2f,%.2f)", A, cx0 / hp.size() / SC, cy0 / hp.size() / SC); fl += b; } }
                    }
                }
                walk(ch);
            }
        };
        walk(tree);
        double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("V %lu %d side %d D %g V %d a %.4f sep %d | free04 %d rep %d%s | %.2fs |REPL%s |FREEL%s\n", seed0, sc, (int)p.side, D, vdeg, p.allowance, (int)p.separateLastPass, free04, nRep, free04 != nRep ? " MISMATCH" : "", secs, repList.substr(0, 300).c_str(), fl.substr(0, 300).c_str());
        if (free04 != nRep && std::getenv("DUMP")) { char fn[512]; snprintf(fn, sizeof fn, "%s/m%d_%lu_%d.scn", std::getenv("DUMP"), mode, seed0, sc); writeScene(fn, cs, p, D, vdeg); }
        std::fflush(stdout);
    }
    std::printf("SUMMARY seed %lu mode %d scenes %ld\n", seed0, mode, T_scn);
}
