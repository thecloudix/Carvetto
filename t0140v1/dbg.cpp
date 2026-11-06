// T-0140 verifier: dbg SCENE=file -> vectors, every move (kind, to, z flag: B below the tab layer, T at the tab top, U above).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
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
int main() {
    ProfileParams p; std::vector<Contour> cs; double D; int vdeg;
    readScene(std::getenv("SCENE"), cs, p, D, vdeg);
    if (std::getenv("NOLEAD")) { p.leads.type = LeadType::None; p.leads.overcut = 0; p.leads.leadOut = false; p.ramp.onLeadIn = false; }
    for (size_t i = 0; i < cs.size(); ++i) { auto b = cs[i].bounds(); std::printf("vec %zu closed %d spans %zu box (%.3f,%.3f)-(%.3f,%.3f)\n", i, (int)cs[i].closed, cs[i].spans.size(), b.min.x, b.min.y, b.max.x, b.max.y); }
    tooling::Tool t = vdeg ? vbit(D, vdeg) : endMill(D);
    auto r = profileToolpath(cs, t, p);
    double zTab = p.passes.bottomZ() + std::min(p.tabs.thickness, p.passes.cutDepth);
    std::printf("bottom %.4f zTab %.4f side %d a %g sep %d lpa %g tabs n %d len %g ramp %d dist %g\n", p.passes.bottomZ(), zTab, (int)p.side, p.allowance, (int)p.separateLastPass, p.lastPassAllowance, p.tabs.count, p.tabs.length, (int)p.ramp.type, p.ramp.distance);
    for (auto& d : r.diagnostics) std::printf("DIAG %d v%d %s\n", (int)d.issue, d.contour ? (int)*d.contour : -1, d.message.c_str());
    for (auto& m : r.toolpath.moves) { char f = m.to.z < zTab - 1e-4 ? 'B' : std::fabs(m.to.z - zTab) < 1e-7 ? 'T' : 'U'; if (std::getenv("ALL") || f != 'U') std::printf("%d %c (%.4f,%.4f,%.4f) c(%.3f,%.3f)\n", (int)m.kind, f, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y); }
}
