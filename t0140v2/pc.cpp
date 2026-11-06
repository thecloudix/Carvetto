// T-0140 r2 verifier: product-level search for leaf-boundary artefacts. Random pages of nested circles (Outside, tabs kept, TC=1) cut through
// by radial open vectors, plus optional rings/rects; profileToolpath with the default leaves vs one leaf (dbg build: T40LEAF); dumps scenes
// whose loose-piece messages differ to DUMP/pc_<seed>_<k>.scn.  usage: pc_dbg SEED N
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
#include "gen_helpers.inc"
#include <algorithm>
static std::string loose(const ProfileResult& r) { std::vector<std::string> v; for (auto& d : r.diagnostics) if (d.message.find("loose piece") != std::string::npos) { auto i = d.message.find("(about "); v.push_back(i == std::string::npos ? d.message : d.message.substr(i, 24)); } std::sort(v.begin(), v.end()); std::string s; for (auto& x : v) s += x + "\n"; return s; }
int main(int argc, char** argv) {
    unsigned long seed = std::strtoul(argv[1], nullptr, 0); int N = std::atoi(argv[2]); int diff = 0;
    for (int k = 0; k < N; ++k) {
        rng.seed(seed * 7919ULL + k);
        double D = I(0, 2) == 0 ? 3.175 : U(1.0, 6.0);
        std::vector<Contour> cs; int groups = std::getenv("ORIGIN") ? 1 : I(1, 3);
        for (int g = 0; g < groups; ++g) {
            bool org = std::getenv("ORIGIN") != nullptr; Vec2 c = org ? Vec2{U(-3, 3), U(-3, 3)} : Vec2{U(-60, 60), U(-60, 60)}; double r = org ? U(1, 4) : U(5, 30); int n = org ? I(4, 12) : I(3, 7);
            for (int i = 0; i < n; ++i) { cs.push_back(Contour::circle(c, r)); r += D * 2 + U(0.3, 4.0); }
            int walls = I(1, 2);
            for (int w = 0; w < walls; ++w) { double a = U(0, 2 * PI); Contour l; l.closed = false; l.spans.push_back(geometry::LineSpan{c + Vec2{std::cos(a), std::sin(a)} * U(0, 2), c + Vec2{std::cos(a), std::sin(a)} * (r + U(0, 3))}); cs.push_back(l); }
        }
        ProfileParams p; p.side = I(0, 1) ? ProfileSide::Outside : ProfileSide::Inside; p.allowance = I(0, 2) ? 0.0 : U(0, 0.5);
        p.passes.cutDepth = 6; p.passes.passDepth = 3; p.tabs.enabled = true; p.tabs.length = U(1, 5); p.tabs.count = I(1, 2); p.tabs.thickness = 1.0;
        p.leads.type = LeadType::None; p.separateLastPass = I(0, 3) == 0; p.lastPassAllowance = 0.3;
        tooling::Tool t = endMill(D);
        unsetenv("T40LEAF"); auto a = profileToolpath(cs, t, p); std::string la = loose(a);
        setenv("T40LEAF", "1000000000", 1); auto b = profileToolpath(cs, t, p); std::string lb = loose(b); unsetenv("T40LEAF");
        if (la != lb) { ++diff; std::printf("DIFF seed %lu k %d\n--default:\n%s--single:\n%s", seed, k, la.c_str(), lb.c_str());
            if (std::getenv("DUMP")) { char fn[512]; std::snprintf(fn, sizeof fn, "%s/pc_%lu_%d.scn", std::getenv("DUMP"), seed, k); writeScene(fn, cs, p, D, 0); } }
        std::fflush(stdout);
    }
    std::printf("SUMMARY seed %lu scenes %d diff %d\n", seed, N, diff);
}
