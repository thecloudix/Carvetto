// t0169v1 repro: sc001 case 1099 (PK_SCALE=0.01, fam 0): where does the 0.005 mm wide island go?
#include "toolpath/pocket.cpp"
#include <cstdio>
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
int main() {
  Contour o; o.closed = true;
  o.append(LineSpan{{0.07888121189718042, 0.061463439617646873}, {-0.061463439617646866, 0.07888121189718042}});
  o.append(BezierSpan{{-0.061463439617646866, 0.07888121189718042}, {-0.086818921077338837, 0.027227544795219251}, {-0.092624845170516681, -0.019554005709723172}, {-0.07888121189718042, -0.061463439617646866}});
  o.append(ArcSpan{{0.034117576012081416, 0.27490423224630828}, 0.35484072024521046, -1.8948895269718793, -1.4936546610286749, ArcDirection::CCW});
  o.append(LineSpan{{0.061463439617646894, -0.078881211897180406}, {0.07888121189718042, 0.061463439617646873}});
  Contour is; is.closed = true;
  is.append(ArcSpan{{-0.021471844566363221, 0.0035042484277013979}, 0.026669627815965203, 0.454985045315216, -0.77853662250020017, ArcDirection::CW});
  is.append(LineSpan{{-0.0024846180074038745, -0.015224186516010634}, {0.0024846180074038745, 0.015224186516010634}});
  Tolerance user; user.chordal = 0.01; user.distance = 1e-5; Tolerance tol = regionTolerance(user);
  std::printf("island arc sweep %.6f mid (%.6f,%.6f)\n", std::get<ArcSpan>(is.spans[0]).sweepAngle(), std::get<ArcSpan>(is.spans[0]).pointAt(0.5).x, std::get<ArcSpan>(is.spans[0]).pointAt(0.5).y);
  auto pr = [](const char* nm, const std::vector<Contour>& cs) { std::printf("%s: %zu contours\n", nm, cs.size()); for (auto& c : cs) { std::printf("  %zu spans:", c.spans.size()); for (auto& s : c.spans) { auto q = startPoint(s); std::printf(" (%.6f,%.6f)", q.x, q.y); } std::printf("\n"); } };
  { auto pl = is.polygonise(tol); std::printf("island polygon %zu pts:", pl.points.size()); for (auto& q : pl.points) std::printf(" (%.9f,%.9f)", q.x, q.y); std::printf("\n"); }
  pr("island alone", unionContours({is}, tol, FillRule::EvenOdd));
  auto region = unionContours({o, is}, tol, FillRule::EvenOdd); pr("region", region);
  InsetReport rep; auto C = centreRegionOf(region, 0.5 * 0.022430994229998218, 0.0, user.chordal, tol, rep); pr("centre", C);
}
