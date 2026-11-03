#include <cstdio>
#include <cmath>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/drill.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static tooling::Tool drill(double d) { tooling::Tool t; t.id="d"; t.name="D"; t.type=tooling::ToolType::Drill; t.diameter=d; t.includedAngle=118*kPi/180;
  t.defaultCuttingData.toolNumber=1; t.defaultCuttingData.feedRate=600; t.defaultCuttingData.plungeRate=200; t.defaultCuttingData.spindleSpeed=10000; t.defaultCuttingData.passDepth=2; return t; }
int main() {
  Tolerance tol = Tolerance::defaults();
  for (double degA : {0.0, 1.0, 30.0, 45.0, 90.0, 180.0, 270.0}) {
    double a = degA * kPi / 180; Vec2 c{3, 4};
    for (int poly = 0; poly < 2; ++poly) {
      Contour ring; ring.closed = true;
      Vec2 po = c + Vec2::fromAngle(a, 10), pi = c + Vec2::fromAngle(a, 5);
      if (!poly) { ring.append(ArcSpan{c, 10, a, a + 2*kPi, ArcDirection::CCW}); ring.append(LineSpan{po, pi}); ring.append(ArcSpan{c, 5, a, a + 2*kPi, ArcDirection::CW}); ring.append(LineSpan{pi, po}); }
      else { Polyline pl; pl.closed = true; int n = 12; for (int i=0;i<=n;++i) pl.points.push_back(c + Vec2::fromAngle(a + 2*kPi*i/n, 10)); for (int i=0;i<=n;++i) pl.points.push_back(c + Vec2::fromAngle(a - 2*kPi*i/n, 5)); ring = Contour::fromPolyline(pl); }
      auto r = drillToolpath({ring}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
      std::printf("bridge %5.1f deg %s: drilled=%d issue=%d centre=%s\n", degA, poly ? "polygon" : "arcs", r.ok(), r.diagnostics.empty() ? -1 : int(r.diagnostics[0].issue),
        r.holes.empty() ? "-" : (std::to_string(r.holes[0].centre.x) + "," + std::to_string(r.holes[0].centre.y)).c_str());
    }
  }
  // C shapes whose centroid lies at the height of a vertex (symmetric about y = 5, vertex on y = 5)
  { Polyline pl; pl.closed = true; pl.points = {{0,0},{10,0},{10,2},{2,2},{2,5},{2,8},{10,8},{10,10},{0,10}};
    auto r = drillToolpath({Contour::fromPolyline(pl)}, drill(1), std::nullopt, DrillParams{.cutDepth = 1});
    std::printf("C with vertex at centroid height: drilled=%d issue=%d\n", r.ok(), r.diagnostics.empty() ? -1 : int(r.diagnostics[0].issue)); }
  { // arrow/chevron: centroid in notch, vertex at same y
    Polyline pl; pl.closed = true; pl.points = {{0,0},{10,5},{0,10},{8,5}};
    auto c = drillCentre(Contour::fromPolyline(pl), tol); auto r = drillToolpath({Contour::fromPolyline(pl)}, drill(1), std::nullopt, DrillParams{.cutDepth = 1});
    std::printf("chevron centre %g %g drilled=%d issue=%d\n", c->x, c->y, r.ok(), r.diagnostics.empty() ? -1 : int(r.diagnostics[0].issue)); }
  { Polyline pl; pl.closed = true; pl.points = {{10,0},{0,5},{10,10},{2,5}};  // chevron pointing left: centroid at x=5.33? notch to right of 2
    auto c = drillCentre(Contour::fromPolyline(pl), tol); auto r = drillToolpath({Contour::fromPolyline(pl)}, drill(1), std::nullopt, DrillParams{.cutDepth = 1});
    std::printf("chevron2 centre %g %g drilled=%d issue=%d loc=%d\n", c->x, c->y, r.ok(), r.diagnostics.empty() ? -1 : int(r.diagnostics[0].issue), int(locatePoint(Contour::fromPolyline(pl), *c, FillRule::EvenOdd, tol))); }
}
