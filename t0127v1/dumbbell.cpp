// AC3: Inside profile of a dumbbell whose offset splits into two loops; count kept per loop vs diag
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
    ProfileParams p; p.side = (ProfileSide)std::atoi(argv[1]); p.allowance = std::atof(argv[2]); p.separateLastPass = std::atoi(argv[3]); p.lastPassAllowance = 0.3;
    p.passes.cutDepth = 6; p.passes.passDepth = 3; p.tabs.enabled = true; p.tabs.count = std::atoi(argv[4]); p.tabs.length = std::atof(argv[5]); p.tabs.thickness = 1.5;
    std::vector<Contour> cs{poly({{0, 0}, {20, 0}, {20, 8.5}, {30, 8.5}, {30, 0}, {50, 0}, {50, 20}, {30, 20}, {30, 11.5}, {20, 11.5}, {20, 20}, {0, 20}})};
    auto r = profileToolpath(cs, endMill(6), p);
    for (auto& d : r.diagnostics) std::printf("diag %d %s\n", (int)d.issue, d.message.c_str());
    if (argc > 6) writeScene(argv[6], cs, p, 6.0, 0);
}
