// T-0127 verifier repro: On profile of a "lollipop" (16 x 16 body + 30 x 4 arm), 6 mm end mill, allowance 0.
// The arm (4 mm < 2r) is cut away completely; the body leaves a 10 x 10 remnant (the part). argv: count length out.scn
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
    ProfileParams p; p.side = ProfileSide::On; p.passes.cutDepth = 6; p.passes.passDepth = 3;
    p.tabs.enabled = true; p.tabs.count = std::atoi(argv[1]); p.tabs.length = std::atof(argv[2]); p.tabs.thickness = 1.5;
    std::vector<Contour> cs{poly({{0, 0}, {16, 0}, {16, 6}, {46, 6}, {46, 10}, {16, 10}, {16, 16}, {0, 16}})};
    auto r = profileToolpath(cs, endMill(6), p);
    for (auto& d : r.diagnostics) std::printf("diag %d %s\n", (int)d.issue, d.message.c_str());
    if (argc > 3) writeScene(argv[3], cs, p, 6.0, 0);
}
