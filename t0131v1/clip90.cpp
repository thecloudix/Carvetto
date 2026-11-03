// T-0131 verifier: where the 90-degree comb raster time goes: Clipper2 line clipping in batches of 256
// (as rasterPattern) vs the comb inset. usage: clip90 teeth angle_deg
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
using namespace carvetto::geometry;
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
int main(int argc, char** argv) {
  int teeth = atoi(argv[1]); double ang = atof(argv[2]) * 3.14159265358979323846 / 180;
  std::vector<Vec2> pts; for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, 20}); pts.push_back({i + 0.5, 20}); pts.push_back({i + 0.5, 0.0}); }
  pts.push_back({double(teeth), -5}); pts.push_back({0, -5}); Contour comb = Contour::fromPolyline(Polyline{pts, true});
  Tolerance tol = Tolerance::defaults();
  double t0 = cpu(); auto inset = offsetContours({comb}, -0.15, JoinType::Round, tol); double t1 = cpu();
  Vec2 u{std::cos(ang), std::sin(ang)}, v = u.perp(); double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
  for (auto& c : inset) { auto b = c.bounds(); for (Vec2 q : {b.min, b.max, Vec2{b.min.x, b.max.y}, Vec2{b.max.x, b.min.y}}) { umin = std::min(umin, q.dot(u)); umax = std::max(umax, q.dot(u)); vmin = std::min(vmin, q.dot(v)); vmax = std::max(vmax, q.dot(v)); } }
  size_t n = (size_t)std::ceil((vmax - vmin) / 0.12); double pitch = (vmax - vmin) / n; size_t pieces = 0;
  for (size_t k0 = 0; k0 < n; k0 += 256) { std::vector<Contour> lines; for (size_t k = k0; k < std::min(n, k0 + 256); ++k) { double vk = vmin + (k + 0.5) * pitch; Contour c; c.append(LineSpan{u * (umin - 1) + v * vk, u * (umax + 1) + v * vk}); lines.push_back(c); }
    auto r = booleanOp(BooleanOp::Intersection, lines, inset, tol, FillRule::EvenOdd); pieces += r.open.size(); }
  double t2 = cpu();
  std::printf("teeth %d ang %g: inset %.3f s, clip %zu lines %zu pieces %.3f s\n", teeth, argv[2] ? atof(argv[2]) : 0, t1 - t0, n, pieces, t2 - t1);
}
