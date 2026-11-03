#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
using namespace carvetto; using namespace carvetto::geometry;
int main() { Tolerance tol;
  for (double C : {1e7, 1e9, 1e12}) for (double a : {0.0, kPi/2, kPi, 1.5*kPi}) {
    Vec2 c{C, -C}; Contour r; r.closed = true; Vec2 po = c + Vec2::fromAngle(a, 10), pi = c + Vec2::fromAngle(a, 5);
    r.append(ArcSpan{c, 10, a, a + 2*kPi, ArcDirection::CCW}); r.append(LineSpan{po, pi}); r.append(ArcSpan{c, 5, a, a + 2*kPi, ArcDirection::CW}); r.append(LineSpan{pi, po});
    std::printf("C %g a %g: centre wn %d loc %d | band(7.5 along -bridge) wn %d | outside wn %d\n", C, a, windingNumber(r, c, tol), (int)locatePoint(r, c, FillRule::EvenOdd, tol), windingNumber(r, c - Vec2::fromAngle(a, 7.5), tol), windingNumber(r, c + Vec2{20, 0}, tol)); }
}
