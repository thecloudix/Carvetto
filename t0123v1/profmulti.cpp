// T-0123 verifier (throwaway): hash Profile output on random multi-vector scenes, every option.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"
int main(int argc, char** argv) {
    unsigned long seed = std::strtoul(argv[1], nullptr, 10);
    int N = std::atoi(argv[2]);
    int mode = std::atoi(argv[3]);  // 0 = harness families, 1 = multi-vector cluster, 2 = grid of many small
    int target = argc > 4 ? std::atoi(argv[4]) : -1;
    rng.seed(seed);
    int fam = 0, forceSep = -1;
    for (int sc = 0; sc < N; ++sc) {
#include "gen_body.inc"
        p.tabs.enabled = I(0, 2) == 0;
        if (mode >= 1) {
            cs.clear();
            int K = mode == 1 ? I(2, 25) : I(30, 400);
            double box = mode == 1 ? U(20, 150) : std::sqrt((double)K) * U(5, 14);
            for (int k = 0; k < K; ++k) {
                Vec2 c{U(0, box), U(0, box)};
                double r = mode == 1 ? U(2, 25) : U(0.8, 5);
                int t = I(0, 9);
                if (t <= 3) cs.push_back(anyClosed(c, r));
                else if (t == 4) cs.push_back(Contour::circle(c, r, I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW));
                else if (t == 5) cs.push_back(Contour::rectangle(geometry::Box2{{c.x - r, c.y - r * 0.6}, {c.x + r, c.y + r * 0.6}}));
                else if (t == 6) cs.push_back(anyOpen(c, r));
                else if (t == 7) { cs.push_back(Contour::circle(c, r)); cs.push_back(Contour::circle(c + Vec2{U(-0.2,0.2)*r, U(-0.2,0.2)*r}, r * U(0.2, 0.6))); }
                else if (t == 8) cs.push_back(bezBlob(c, r, I(4, 8)));
                else cs.push_back(letter(c, r));
            }
            if (mode == 2) { D = std::vector<double>{0.5, 1.0, 2.0, 3.175}[(size_t)I(0, 3)]; useV = useV && I(0,1); }
            if (I(0, 4) == 0) p.side = ProfileSide::On;
        }
        if (target >= 0 && sc != target) continue;
        tooling::Tool tool = useV ? vbit(D * 2, vang) : endMill(D);
        auto res = profileToolpath(cs, tool, p);
        unsigned long long h = 1469598103934665603ULL;
        auto mix = [&](const char* s) { for (; *s; ++s) { h ^= (unsigned char)*s; h *= 1099511628211ULL; } };
        char b[256];
        for (auto& m : res.toolpath.moves) { std::snprintf(b, sizeof b, "%d %.12g %.12g %.12g %.12g %.12g|", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y); mix(b); if (target >= 0) std::printf("%s\n", b); }
        for (auto& d : res.diagnostics) { std::snprintf(b, sizeof b, "%d %ld|", (int)d.issue, d.contour ? (long)*d.contour : -1L); mix(b); mix(d.message.c_str()); if (target >= 0) std::printf("diag %s\n", d.message.c_str()); }
        std::printf("scene %d K %zu side %d V %d tabs %d moves %zu diags %zu hash %016llx\n", sc, cs.size(), (int)p.side, (int)useV, (int)p.tabs.enabled, res.toolpath.moves.size(), res.diagnostics.size(), h);
        std::fflush(stdout);
    }
}
