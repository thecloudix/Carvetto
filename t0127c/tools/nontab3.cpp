// T-0124 v3 verifier (throwaway): hash every scene's move list + diagnostics, tabs on or off (TABS env).
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
    int fam = std::atoi(argv[3]);
    int forceSep = std::atoi(argv[4]);
    int target = argc > 5 ? std::atoi(argv[5]) : -1;
    int tabsMode = std::getenv("TABS") ? std::atoi(std::getenv("TABS")) : 0;
    rng.seed(seed);
    for (int sc = 0; sc < N; ++sc) {
#include "gen_body_neg.inc"
        if (target >= 0 && sc != target) continue;
        p.tabs.enabled = tabsMode != 0;
        tooling::Tool tool = useV ? vbit(D * 2, vang) : endMill(D);
        auto res = profileToolpath(cs, tool, p);
        unsigned long long h = 1469598103934665603ULL;
        auto mix = [&](const char* s) { for (; *s; ++s) { h ^= (unsigned char)*s; h *= 1099511628211ULL; } };
        char b[256];
        for (auto& m : res.toolpath.moves) { std::snprintf(b, sizeof b, "%d %.9f %.9f %.9f %.9f %.9f|", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y); mix(b); if (target >= 0) std::printf("%s\n", b); }
        for (auto& d : res.diagnostics) { mix(d.message.c_str()); if (target >= 0) std::printf("diag %s\n", d.message.c_str()); }
        std::printf("scene %d fam %d side %d V %d allow %.6f moves %zu hash %016llx\n", sc, f, (int)p.side, (int)useV, p.allowance, res.toolpath.moves.size(), h);
    }
}
