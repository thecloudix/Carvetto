// t0169v1 repro (pre-existing, main identical): an island whose arc sagitta is below chordal/2 polygonises to
// a zero-area polygon, Clipper drops it, and Pocket cuts straight through the island.
// usage: lens <sagitta> <chordal> ; 20 x 20 square pocket, lens island on a 4 mm chord at the centre, 1 mm end mill.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "oc2.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
int main(int argc, char** argv) {
  double s = atof(argv[1]), ch = atof(argv[2]); double h = 2.0, R = (h * h + s * s) / (2 * s);
  Contour sq = Contour::rectangle(Box2{{-10, -10}, {10, 10}}); Contour is; is.closed = true;
  Vec2 c{0, s - R}; double a0 = std::atan2(0 - c.y, -2 - c.x), a1 = std::atan2(0 - c.y, 2 - c.x);
  is.append(ArcSpan{c, R, a0, a1, ArcDirection::CW}); is.append(LineSpan{{2, 0}, {-2, 0}});
  PocketTool t; t.tool.id = t.tool.name = "E"; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = 1; auto& cd = t.tool.defaultCuttingData; cd.toolNumber = 1; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.stepover = 0.4; cd.passDepth = 3;
  PocketParams p; p.cutDepth = 1; p.tolerance.chordal = ch; p.tolerance.distance = std::min(ch, 0.001); p.strategy = PocketStrategy::Raster;
  auto r = pocketToolpath({sq, is}, {t}, p);
  auto rep = oc2::check(r.toolpaths[0], {sq, is}, 0.5, -1e-5, 0.0, 0.05);
  std::printf("sagitta %g chordal %g: island apex (0,%g); moves %zu; min clearance - radius %.4e (worst move (%.4f,%.4f)->(%.4f,%.4f))\n", s, ch, s, r.toolpaths[0].moves.size(), rep.minc, rep.wa.x, rep.wa.y, rep.wb.x, rep.wb.y);
}
