// T-0142 verifier (e): clip90 (t0131v1) with OpenPathClipper: comb inset time and raster clip time separately,
// interleaved n / 2n, min of reps. usage: c90 teeth angle reps
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
using namespace carvetto::geometry;
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
static Contour comb(int teeth) { std::vector<Vec2> pts; for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, 20}); pts.push_back({i + 0.5, 20}); pts.push_back({i + 0.5, 0.0}); }
  pts.push_back({double(teeth), -5}); pts.push_back({0, -5}); return Contour::fromPolyline(Polyline{pts, true}); }
static size_t clipAll(const std::vector<Contour>& inset, double ang) { Tolerance tol = Tolerance::defaults(); Vec2 u{std::cos(ang), std::sin(ang)}, v = u.perp();
  double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
  for (auto& c : inset) { auto b = c.bounds(); for (Vec2 q : {b.min, b.max, Vec2{b.min.x, b.max.y}, Vec2{b.max.x, b.min.y}}) { umin = std::min(umin, q.dot(u)); umax = std::max(umax, q.dot(u)); vmin = std::min(vmin, q.dot(v)); vmax = std::max(vmax, q.dot(v)); } }
  OpenPathClipper clipper(inset, v, tol, FillRule::EvenOdd);
  size_t n = (size_t)std::ceil((vmax - vmin) / 0.12); double pitch = (vmax - vmin) / n; size_t pieces = 0;
  for (size_t k0 = 0; k0 < n; k0 += 256) { std::vector<Contour> lines; for (size_t k = k0; k < std::min(n, k0 + 256); ++k) { double vk = vmin + (k + 0.5) * pitch; Contour c; c.append(LineSpan{u * (umin - 1) + v * vk, u * (umax + 1) + v * vk}); lines.push_back(c); }
    pieces += clipper.clip(BooleanOp::Intersection, lines).size(); }
  return pieces; }
int main(int argc, char** argv) { int teeth = atoi(argv[1]); double ang = atof(argv[2]) * M_PI / 180; int reps = argc > 3 ? atoi(argv[3]) : 3; Tolerance tol = Tolerance::defaults();
  Contour a = comb(teeth), b = comb(2 * teeth); double ia = 1e30, ib = 1e30, ca = 1e30, cb = 1e30; std::vector<Contour> A, B; size_t pa = 0, pb = 0;
  for (int r = 0; r < reps; ++r) { double t0 = cpu(); A = offsetContours({a}, -0.15, JoinType::Round, tol); double t1 = cpu(); B = offsetContours({b}, -0.15, JoinType::Round, tol); double t2 = cpu();
    pa = clipAll(A, ang); double t3 = cpu(); pb = clipAll(B, ang); double t4 = cpu(); ia = std::min(ia, t1 - t0); ib = std::min(ib, t2 - t1); ca = std::min(ca, t3 - t2); cb = std::min(cb, t4 - t3); }
  std::printf("c90 teeth %d->%d ang %s: inset %.4f %.4f (x%.2f) clip %.4f %.4f (x%.2f) pieces %zu %zu\n", teeth, 2 * teeth, argv[2], ia, ib, ib / ia, ca, cb, cb / ca, pa, pb); }
