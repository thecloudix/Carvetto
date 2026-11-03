// T-0135 r2 verifier: allLinked shortcut probe. Room1 held by L's tab; room2 free except a redundant tab on
// an open slot running 0.3 mm (material) beside L's kerf. args: out.scn [stripGap=0.3] [slotLen=30]
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
    double gap = argc > 2 ? atof(argv[2]) : 0.3, sl = argc > 3 ? atof(argv[3]) : 30;
    int variant = argc > 4 ? atoi(argv[4]) : 0;
    std::vector<Contour> cs;
    cs.push_back(poly({{70,10},{70,20},{30,20},{30,10.5},{20,10.5},{20,20},{0,20},{0,0},{20,0},{20,9.5},{30,9.5},{30,0},{70,0}}, true));
    double y = 1.0 + gap + 1.0;
    if (variant == 0) cs.push_back(poly({{50 - sl / 2, y}, {50 + sl / 2, y}}, false));
    else cs.push_back(poly({{50, 0}, {50, sl}}, false));  // spur from L's bottom wall into room2
    ProfileParams p; p.side = ProfileSide::On; p.allowance = 0; p.passes.cutDepth = 3; p.passes.passDepth = 3;
    p.tabs.enabled = true; p.tabs.count = 1; p.tabs.length = 2; p.tabs.thickness = 1; p.startAt = StartAt::VectorStart;
    p.leads.type = LeadType::None; p.separateLastPass = false;
    writeScene(argv[1], cs, p, 2.0, 0);
    auto r = profileToolpath(cs, endMill(2.0), p);
    for (auto& d : r.diagnostics) std::printf("DIAG %d %s\n", (int)d.issue, d.message.c_str());
    std::printf("moves %zu\n", r.toolpath.moves.size());
}
