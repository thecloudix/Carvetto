// mv.cpp SCENE.scn [notab] : prints Profile moves (%.9f) and diagnostics for a dl3 scene file
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
  ProfileParams p; std::vector<Contour> cs; double D = 6; int vdeg = 0;
  readScene(argv[1], cs, p, D, vdeg);
  if (getenv("NOLEAD")) { p.leads.type = LeadType::None; p.leads.overcut = 0; p.leads.leadOut = false; p.ramp.onLeadIn = false; }
  if (getenv("FORCESEP")) p.separateLastPass = true;
  if (argc > 2 && !strcmp(argv[2], "notab")) p.tabs.enabled = false;
  tooling::Tool tool = vdeg ? vbit(D, vdeg) : endMill(D);
  auto r = profileToolpath(cs, tool, p);
  for (auto& m : r.toolpath.moves) printf("%d %.9f %.9f %.6f c %.9f %.9f\n", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y);
  for (auto& d : r.diagnostics) printf("D %d %s\n", (int)d.issue, d.message.c_str());
}
