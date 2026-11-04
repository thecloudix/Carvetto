// T-0163 verifier: run one stored scene, print diagnostics, move bounds vs input bounds, finiteness,
// rapids at safe Z, and (LEADS=0/1/2 override, LO=0/1 lead-out override, TABS=0/1) variants.
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
  std::vector<Contour> cs; ProfileParams p; double D = 6; int vd = 0;
  if (!readScene(argv[1], cs, p, D, vd)) return 1;
  if (getenv("LEADS")) p.leads.type = (LeadType)atoi(getenv("LEADS"));
  if (getenv("LO")) p.leads.leadOut = atoi(getenv("LO"));
  if (getenv("TABS")) p.tabs.enabled = atoi(getenv("TABS"));
  if (getenv("SEP")) p.separateLastPass = atoi(getenv("SEP"));
  if (getenv("SIDE")) p.side = (ProfileSide)atoi(getenv("SIDE"));
  if (getenv("START")) p.startAt = (StartAt)atoi(getenv("START"));
  if (getenv("ANCHOR")) p.startAnchor = (BoxAnchor)atoi(getenv("ANCHOR"));
  if (getenv("OVERCUT")) p.leads.overcut = atof(getenv("OVERCUT"));
  tooling::Tool tool = vd ? vbit(D, vd) : endMill(D);
  auto r = profileToolpath(cs, tool, p);
  geometry::Box2 in; for (auto& c : cs) in.expand(c.bounds());
  geometry::Box2 mv; bool fin = true; int badRapid = 0, retracts = 0;
  for (auto& m : r.toolpath.moves) { fin = fin && std::isfinite(m.to.x) && std::isfinite(m.to.y) && std::isfinite(m.to.z); if (m.kind != MoveKind::Dwell) mv.expand(m.to.xy());
    if (m.kind == MoveKind::Rapid && m.to.z != p.safeZ) ++badRapid; if (m.kind == MoveKind::Retract) ++retracts; }
  std::printf("contours %zu side %d sep %d lead %d lo %d tabs %d start %d D %g vd %d\n", cs.size(), (int)p.side, (int)p.separateLastPass, (int)p.leads.type, (int)p.leads.leadOut, (int)p.tabs.enabled, (int)p.startAt, D, vd);
  std::printf("moves %zu retracts %d finite %d badRapid %d in [%g %g]-[%g %g] moves [%g %g]-[%g %g]\n", r.toolpath.moves.size(), retracts, (int)fin, badRapid, in.min.x, in.min.y, in.max.x, in.max.y, mv.min.x, mv.min.y, mv.max.x, mv.max.y);
  for (auto& d : r.diagnostics) std::printf("  diag %d c%lld: %s\n", (int)d.issue, d.contour ? (long long)*d.contour : -1LL, d.message.c_str());
}
