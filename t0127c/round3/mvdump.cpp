// mvdump scene.scn [x y radius]: print moves (index kind x y z) optionally near (x,y)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
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
int main(int argc, char** argv) {
    std::vector<Contour> cs; ProfileParams p; double D; int vd = 0; readScene(argv[1], cs, p, D, vd);
    auto t = vd ? vbit(D, vd) : endMill(D);
    auto r = profileToolpath(cs, t, p);
    for (auto& d : r.diagnostics) std::printf("diag %d c%d %s\n", (int)d.issue, d.contour ? (int)*d.contour : -1, d.message.c_str());
    double zTab = p.passes.bottomZ() + std::min(p.tabs.thickness, p.passes.cutDepth);
    std::printf("zTab %.6f bottom %.6f r %.4f allowance %.6f side %d sep %d lpa %.4f\n", zTab, p.passes.bottomZ(), profileToolRadius(t, p.passes), p.allowance, (int)p.side, (int)p.separateLastPass, p.lastPassAllowance);
    for (size_t i = 0; i < r.toolpath.moves.size(); ++i) { auto& m = r.toolpath.moves[i];
        if (argc > 4 && std::hypot(m.to.x - atof(argv[2]), m.to.y - atof(argv[3])) > atof(argv[4])) continue;
        std::printf("%zu k%d %.4f %.4f %.4f%s\n", i, (int)m.kind, m.to.x, m.to.y, m.to.z, std::fabs(m.to.z - zTab) < 1e-7 ? " TAB" : ""); }
}
