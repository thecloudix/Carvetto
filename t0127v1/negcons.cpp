// AC5 consistency at r + allowance ~ 0: compare allowance -r exactly with -r +- eps (eps 1e-9, 1e-7, 1e-5).
// Reports per config: same move count/kinds, max xyz diff, diag count, tabs left; also -0.0 vs +0.0 On.
#include <cmath>
#include <cstdio>
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
#include "gen_helpers.inc"
static int leftOf(const ProfileResult& r) { int s = 0; for (auto& d : r.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced) { int a, b; if (std::sscanf(d.message.c_str() + d.message.find(": ") + 2, "%d of %d", &a, &b) == 2) s += a; } return s; }
int main(int argc, char** argv) {
    int N = std::atoi(argv[1]); long bad = 0, tot = 0;
    for (int sc = 0; sc < N; ++sc) {
        rng.seed(777000 + sc);
        bool useV = I(0, 3) == 0; double D; int vd = 0;
        if (useV) { D = std::vector<double>{6.0, 12.0}[(size_t)I(0, 1)]; vd = std::vector<int>{60, 90}[(size_t)I(0, 1)]; } else D = std::vector<double>{3.175, 6.0, 8.0}[(size_t)I(0, 2)];
        auto tool = useV ? vbit(D, vd) : endMill(D);
        ProfileParams p; p.side = I(0, 1) ? ProfileSide::Inside : ProfileSide::Outside; p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.passes.cutDepth = useV ? U(1, 5) : U(3, 10); p.passes.passDepth = U(1, 4); p.separateLastPass = I(0, 1); p.lastPassAllowance = U(0.05, 0.6); p.lastPassReverse = I(0, 1);
        p.tabs.enabled = I(0, 3) != 0; p.tabs.count = I(1, 6); p.tabs.length = U(0.5, 6); p.tabs.thickness = U(0.3, 0.8 * p.passes.cutDepth); p.tabs.threeD = I(0, 2) == 0;
        p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = (BoxAnchor)I(0, 8);
        std::vector<Contour> cs; int f = I(0, 2);
        if (f == 0) cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {U(15, 60), U(15, 60)}}));
        else if (f == 1) cs.push_back(anyClosed({0, 0}, U(10, 30)));
        else { cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {60, 40}})); cs.push_back(Contour::circle({30, 20}, U(6, 12))); }
        double r = profileToolRadius(tool, p.passes);
        p.allowance = -r; auto e = profileToolpath(cs, tool, p);
        for (double eps : {1e-9, -1e-9, 1e-7, 1e-5}) {
            ProfileParams q = p; q.allowance = -r + eps; if (q.allowance < -r) { /* below -r: invalid? */ }
            auto o = profileToolpath(cs, tool, q);
            ++tot;
            bool sameN = e.toolpath.moves.size() == o.toolpath.moves.size(); double worst = 0; bool kinds = true;
            if (sameN) for (size_t i = 0; i < e.toolpath.moves.size(); ++i) { auto& a = e.toolpath.moves[i]; auto& b = o.toolpath.moves[i]; kinds &= a.kind == b.kind; worst = std::max({worst, std::fabs(a.to.x - b.to.x), std::fabs(a.to.y - b.to.y), std::fabs(a.to.z - b.to.z)}); }
            bool ok = sameN && kinds && worst < 1e-3 && leftOf(e) == leftOf(o) && e.diagnostics.size() == o.diagnostics.size();
            if (!ok) ++bad;
            if (!ok && leftOf(e) != leftOf(o) && std::getenv("DUMP")) { char fn[256]; snprintf(fn, sizeof fn, "%s/nc_%d_x.scn", std::getenv("DUMP"), sc); writeScene(fn, cs, p, D, vd); snprintf(fn, sizeof fn, "%s/nc_%d_e%g.scn", std::getenv("DUMP"), sc, eps); writeScene(fn, cs, q, D, vd); }
            if (!ok || std::getenv("V")) std::printf("sc %d side %d V %d sep %d tabs %d eps %g | n %zu/%zu kinds %d worst %.3g left %d/%d diags %zu/%zu%s\n", sc, (int)p.side, vd, (int)p.separateLastPass, (int)p.tabs.enabled, eps, e.toolpath.moves.size(), o.toolpath.moves.size(), (int)kinds, worst, leftOf(e), leftOf(o), e.diagnostics.size(), o.diagnostics.size(), ok ? "" : "  MISMATCH");
        }
    }
    std::printf("TOTAL %ld comparisons, %ld mismatches\n", tot, bad);
}
