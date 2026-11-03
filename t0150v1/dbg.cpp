#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/toolpath/drill.hpp"
using namespace carvetto; using namespace carvetto::geometry;
int main() {
  Tolerance tol = Tolerance::defaults();
  Contour ring; ring.closed = true; ring.append(ArcSpan{{0,0}, 10, 0, 2*kPi, ArcDirection::CCW}); ring.append(LineSpan{{10,0},{5,0}}); ring.append(ArcSpan{{0,0}, 5, 0, 2*kPi, ArcDirection::CW}); ring.append(LineSpan{{5,0},{10,0}});
  auto c = toolpath::drillCentre(ring, tol);
  std::printf("centre %g %g signedArea %g loc %d wn %d cont %d\n", c->x, c->y, signedArea(ring), int(locatePoint(ring, *c, FillRule::EvenOdd, tol)), windingNumber(ring, *c, tol), ring.isContinuous(tol));
  for (auto p : {Vec2{0,0}, Vec2{0.1,0.2}, Vec2{7,3}}) std::printf("pt %g %g loc %d wn %d\n", p.x, p.y, int(locatePoint(ring, p, FillRule::EvenOdd, tol)), windingNumber(ring, p, tol));
  std::printf("dist %g\n", distanceTo(ring, *c));
}
