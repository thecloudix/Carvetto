#include <cstdio>
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
#include <random>
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
constexpr double PI = 3.14159265358979323846;
using geometry::Contour; using geometry::Vec2;
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"
int main(int argc, char** argv) {
    std::vector<Contour> cs; ProfileParams p; double D; int vd = 0; readScene(argv[1], cs, p, D, vd);
    if (std::getenv("NOLEAD")) { p.leads.type = LeadType::None; p.leads.overcut = 0; p.leads.leadOut = false; p.ramp.onLeadIn = false; }
    auto r = profileToolpath(cs, vd ? vbit(D, vd) : endMill(D), p);
    for (auto& d : r.diagnostics) std::printf("DIAG %d %s\n", (int)d.issue, d.message.c_str());
    if (std::getenv("MV")) for (auto& m : r.toolpath.moves) std::printf("M %d %.4f %.4f %.4f c %.4f %.4f\n", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y);
}
