// dbg: run one scene (VAR=vsout|bbc|opt like st), print raw TFS2 lines (stderr) and the first moves of the toolpath.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
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
int main(int argc, char** argv) {
  std::vector<Contour> cs; ProfileParams p; double D = 6; int vd = 0; readScene(argv[1], cs, p, D, vd);
  if (const char* v = getenv("VAR")) { std::string s = v; if (s == "vsout") { p.startAt = StartAt::VectorStart; p.side = ProfileSide::Outside; }
    else if (s == "bbc") { p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::Centre; } else if (s == "opt") p.startAt = StartAt::Optimise; }
  if (getenv("DIR")) p.direction = (CutDirection)atoi(getenv("DIR"));
  double S = getenv("SCALE") ? atof(getenv("SCALE")) : 1.0;
  if (S != 1.0) for (auto& c : cs) for (auto& sp : c.spans) { if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start * S; l->end = l->end * S; } else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = a->centre * S; a->radius *= S; } }
  tooling::Tool tool = vd ? vbit(D, vd) : endMill(D);
  setenv("TFS_LOG", "1", 1);
  auto r = profileToolpath(cs, tool, p);
  int nm = getenv("NM") ? atoi(getenv("NM")) : 8;
  for (int i = 0; i < (int)r.toolpath.moves.size() && i < nm; ++i) { auto& m = r.toolpath.moves[i]; std::printf("M %d kind %d to %.9f %.9f %.6f c %.9f %.9f\n", i, (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y); }
  for (auto& d : r.diagnostics) std::printf("D %s\n", d.message.c_str());
  std::printf("vector start %.9f %.9f side %d dir %d sharp %d limit %g\n", cs[0].startPoint().x, cs[0].startPoint().y, (int)p.side, (int)p.direction, (int)p.corners.sharpExternal, p.corners.mitreLimit);
}
