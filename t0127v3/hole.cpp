// hole.cpp: outer 64.75x71.4 rect + one hole rect (w x h) at (30,30); Inside; 1 tab per vector.
// args: w h vdeg(0=endmill) D allowance th L 3d sep
#include <cstdio>
#include <cstdlib>
#include <random>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
constexpr double PI = 3.14159265358979323846;
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"
int main(int argc, char** argv) {
    double w = std::atof(argv[1]), hh = std::atof(argv[2]); int vd = std::atoi(argv[3]); double D = std::atof(argv[4]);
    ProfileParams p; p.side = ProfileSide::Inside; p.allowance = std::atof(argv[5]); p.passes.startDepth = 1.565; p.passes.cutDepth = 5.912; p.passes.passDepth = 7.698; p.passes.lastPassThickness = 0.239;
    p.tabs.enabled = true; p.tabs.thickness = std::atof(argv[6]); p.tabs.length = std::atof(argv[7]); p.tabs.threeD = std::atoi(argv[8]); p.tabs.count = 1; p.tabs.avoidCorners = true;
    p.separateLastPass = std::atoi(argv[9]); p.lastPassAllowance = 0.255; p.direction = CutDirection::Conventional;
    std::vector<Contour> cs{Contour::rectangle(geometry::Box2{{0, 0}, {64.75, 71.396}}), Contour::rectangle(geometry::Box2{{30, 30}, {30 + w, 30 + hh}})};
    tooling::Tool t = vd ? vbit(D, vd) : endMill(D);
    double r = profileToolRadius(t, p.passes);
    if (std::getenv("REL")) p.allowance = -r + std::atof(argv[5]);
    auto res = profileToolpath(cs, t, p);
    int left = 0; for (auto& d : res.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced && d.contour && *d.contour == 1) { int a, b; if (std::sscanf(d.message.c_str() + d.message.find(": ") + 2, "%d of %d", &a, &b) == 2) left = a; }
    bool loose = false; for (auto& d : res.diagnostics) if (d.message.find("comes free") != std::string::npos) loose = true;
    std::printf("w %g h %g vd %d D %g a %.4f (r %.4f) th %g L %g 3d %d sep %d -> hole tab left %d loose %d\n", w, hh, vd, D, p.allowance, r, p.tabs.thickness, p.tabs.length, p.tabs.threeD, p.separateLastPass, left, (int)loose);
    if (std::getenv("OUT")) writeScene(std::getenv("OUT"), cs, p, D, vd);
}
