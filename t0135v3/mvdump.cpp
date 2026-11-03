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
    std::vector<Contour> cs; ProfileParams p; double D; int vdeg;
    readScene(argv[1], cs, p, D, vdeg);
    if (getenv("NOLEAD")) { p.leads.type = LeadType::None; p.leads.overcut = 0; p.leads.leadOut = false; p.ramp.onLeadIn = false; }
    double x0 = atof(argv[2]), x1 = atof(argv[3]), y0 = atof(argv[4]), y1 = atof(argv[5]);
    auto r = profileToolpath(cs, vdeg ? vbit(D, vdeg) : endMill(D), p);
    for (auto& d : r.diagnostics) std::printf("DIAG %d %s\n", (int)d.issue, d.message.c_str());
    geometry::Vec3 pp = r.toolpath.moves[0].to;
    for (size_t i = 1; i < r.toolpath.moves.size(); ++i) { auto& m = r.toolpath.moves[i];
        bool in = (m.to.x > x0 && m.to.x < x1 && m.to.y > y0 && m.to.y < y1) || (pp.x > x0 && pp.x < x1 && pp.y > y0 && pp.y < y1);
        if (in) std::printf("%zu kind %d (%.4f,%.4f,%.4f)->(%.4f,%.4f,%.4f) c(%.4f,%.4f)\n", i, (int)m.kind, pp.x, pp.y, pp.z, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y);
        pp = m.to; }
    std::printf("bottom %.4f tabs th %.3f lpa %.3f a %.4f side %d sepl %d dir %d\n", p.passes.bottomZ(), p.tabs.thickness, p.lastPassAllowance, p.allowance, (int)p.side, (int)p.separateLastPass, (int)p.direction);
}
