// adv seed N outdir : adversarial loose-piece scenes (T-0135 verifier, throwaway)
// kinds: 0 nested holes/islands, 1 open trimmed loops, 2 slugs touching tabs (holes in a part),
//        3 necked shapes On with |a|, 4 multi-hole + islands with mixed sides, 5 open cluster chains
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
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
static Contour rect(double x0, double y0, double x1, double y1) { return Contour::rectangle(geometry::Box2{{x0, y0}, {x1, y1}}); }
static Contour shape(Vec2 c, double h) { // closed shape of half-size h centred at c
    int k = I(0, 2);
    if (k == 0) return Contour::circle(c, h, I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW);
    if (k == 1) return rect(c.x - h, c.y - h * U(0.6, 1), c.x + h, c.y + h * U(0.6, 1));
    return gear(c, h, I(5, 10), h * U(0.1, 0.3));
}
int main(int argc, char** argv) {
    unsigned long seed0 = std::strtoul(argv[1], nullptr, 0); int N = std::atoi(argv[2]); const char* out = argv[3];
    for (int sc = 0; sc < N; ++sc) {
        rng.seed(seed0 * 1000003ULL + (unsigned long)sc);
        int kind = I(0, 5);
        ProfileParams p; std::vector<Contour> cs;
        double D = std::vector<double>{2.0, 3.175, 6.0}[(size_t)I(0, 2)];
        int vdeg = I(0, 5) == 0 ? std::vector<int>{60, 90}[(size_t)I(0, 1)] : 0;
        if (vdeg) D = 6.0;
        double r = D / 2;
        p.side = ProfileSide::On;
        p.allowance = 0;
        if (kind == 0 || kind == 4) {
            double W = U(25, 60);
            int depth = I(2, 4); double h = W / 2;
            Vec2 c{U(-5, 5), U(-5, 5)};
            for (int d = 0; d < depth && h > 2 * D; ++d) { cs.push_back(shape(c, h)); h = h - U(1.6, 3.5) * D; c = c + Vec2{U(-0.5, 0.5), U(-0.5, 0.5)}; }
            if (kind == 4) { // extra holes beside, each with island
                int n = I(1, 3);
                for (int i = 0; i < n; ++i) { double hh = U(2.5, 5) * D / 2 + D; Vec2 cc{c.x + U(-W / 4, W / 4), c.y + U(-W / 4, W / 4)}; cs.push_back(shape(cc, hh)); if (I(0, 1)) cs.push_back(shape(cc, hh - U(1.6, 2.5) * D)); }
            }
        } else if (kind == 1) {
            // square/triangle broken into open pieces with small gaps or overlaps
            double S = U(10, 40); int nv = I(3, 6);
            std::vector<Vec2> P; for (int i = 0; i < nv; ++i) { double a = 2 * PI * i / nv + U(-0.2, 0.2); double rr = S * U(0.7, 1.0); P.push_back({rr * std::cos(a), rr * std::sin(a)}); }
            int cuts = I(2, nv);
            std::vector<int> breaks; for (int i = 0; i < nv; ++i) breaks.push_back(i); std::shuffle(breaks.begin(), breaks.end(), rng); breaks.resize((size_t)cuts); std::sort(breaks.begin(), breaks.end());
            for (int b = 0; b < cuts; ++b) {
                int s0 = breaks[(size_t)b], s1 = breaks[(size_t)((b + 1) % cuts)]; if (s1 <= s0) s1 += nv;
                std::vector<Vec2> pts;
                for (int i = s0; i <= s1; ++i) pts.push_back(P[(size_t)(i % nv)]);
                double g0 = U(-0.4, 0.45) * D, g1 = U(-0.4, 0.45) * D; // positive: gap, negative: overrun
                Vec2 d0 = pts[1] - pts[0]; d0 = d0 * (1.0 / std::hypot(d0.x, d0.y)); pts[0] = pts[0] + d0 * g0;
                Vec2 d1 = pts.back() - pts[pts.size() - 2]; d1 = d1 * (1.0 / std::hypot(d1.x, d1.y)); pts.back() = pts.back() - d1 * g1;
                cs.push_back(poly(pts, false));
            }
            if (I(0, 1)) cs.push_back(shape({U(-S / 3, S / 3), U(-S / 3, S / 3)}, U(2, S / 3)));
            if (I(0, 2) == 0) cs.push_back(rect(-S * 1.4, -S * 1.4, S * 1.4, S * 1.4));
        } else if (kind == 2) {
            double W = U(30, 70), H = U(25, 60);
            cs.push_back(rect(0, 0, W, H));
            int n = I(1, 5);
            for (int i = 0; i < n; ++i) { double hh = U(1.2, 4) * D; Vec2 cc{U(hh + 1, W - hh - 1), U(hh + 1, H - hh - 1)}; cs.push_back(shape(cc, hh)); }
            p.side = I(0, 1) ? ProfileSide::On : ProfileSide::Outside;
        } else if (kind == 3) {
            // dumbbell / H with necks near the tool width
            double L = U(8, 25), t = U(0.6, 2.2) * D, B = U(6, 18);
            cs.push_back(poly({{0, 0}, {B, 0}, {B, B / 2 - t / 2}, {B + L, B / 2 - t / 2}, {B + L, 0}, {2 * B + L, 0}, {2 * B + L, B}, {B + L, B}, {B + L, B / 2 + t / 2}, {B, B / 2 + t / 2}, {B, B}, {0, B}}));
            p.allowance = U(-0.9, 0.9) * r;
            if (I(0, 1)) cs.push_back(rect(-U(2, 8), -U(2, 8), 2 * B + L + U(2, 8), B + U(2, 8)));
        } else {
            // chain of open lines near each other (a cluster) forming a staircase enclosure
            int n = I(3, 8); Vec2 c{0, 0};
            for (int i = 0; i < n; ++i) { cs.push_back(anyOpen(c, U(10, 25))); c = c + Vec2{U(3, 12), U(-6, 6)}; }
            if (I(0, 1)) cs.push_back(poly({{-5, -20}, {c.x + 30, -20}}, false));
        }
        if (cs.empty()) cs.push_back(Contour::circle({0, 0}, 10));
        if (p.side != ProfileSide::On) p.allowance = -r * U(0, 1);
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.passes.cutDepth = vdeg ? U(1, 4) : U(2, 10); p.passes.passDepth = U(1, 5);
        p.separateLastPass = I(0, 3) == 0; p.lastPassAllowance = U(0.05, 0.6); p.lastPassReverse = I(0, 1);
        p.tabs.enabled = true;
        p.tabs.length = I(0, 2) == 0 ? U(6, 30) : U(0.3, 5);
        p.tabs.thickness = U(0.2, 0.9 * p.passes.cutDepth);
        p.tabs.spacing = TabSpacing::ConstantNumber; p.tabs.count = I(1, 5);
        p.tabs.avoidCorners = I(0, 1); p.tabs.threeD = I(0, 3) == 0;
        p.ramp.type = RampType::None; p.leads.type = LeadType::None; p.leads.overcut = 0;
        p.startAt = (StartAt)I(0, 2); p.startAnchor = (BoxAnchor)I(0, 8);
        char fn[512]; snprintf(fn, sizeof fn, "%s/a%d_%lu_%d.scn", out, kind, seed0, sc);
        writeScene(fn, cs, p, D, vdeg);
    }
}
