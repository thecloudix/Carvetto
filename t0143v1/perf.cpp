#include <cstdio>
#include <cmath>
#include <ctime>
#include <random>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
using namespace carvetto; using namespace carvetto::geometry;
int main() {
  Tolerance tol; std::mt19937_64 rng(5);
  for (int kind = 0; kind < 3; ++kind) {
    Contour c; c.closed = true; const int n = 100000;
    if (kind == 0) { Polyline pl; pl.closed = true; for (int i = 0; i < n; ++i) { double a = 2*kPi*i/n; double r = 50 + 5*std::sin(37*a); pl.points.push_back(Vec2::fromAngle(a, r)); } c = Contour::fromPolyline(pl); }
    else if (kind == 1) { for (int i = 0; i < n; ++i) { double a0 = 2*kPi*i/n, a1 = 2*kPi*(i+1)/n; Vec2 m = Vec2::fromAngle(0.5*(a0+a1), 50); c.append(ArcSpan{m, 1.0, a0, a1 + kPi, ArcDirection::CCW}); } }  // junction gaps: many links
    else { for (int i = 0; i < n / 10; ++i) { double a0 = 2*kPi*i/(n/10), a1 = 2*kPi*(i+1)/(n/10); Vec2 p0 = Vec2::fromAngle(a0, 50), p3 = Vec2::fromAngle(a1, 50); c.append(BezierSpan{p0, p0 * 1.01, p3 * 1.01, p3}); } }
    long sum = 0; std::clock_t t0 = std::clock(); int q = kind == 2 ? 20 : 200;
    for (int i = 0; i < q; ++i) { Vec2 p{std::uniform_real_distribution<double>(-60, 60)(rng), std::uniform_real_distribution<double>(-60, 60)(rng)}; sum += windingNumber(c, p, tol); }
    double s = double(std::clock() - t0) / CLOCKS_PER_SEC;
    std::printf("kind %d spans %zu queries %d cpu %.3f s (%.3f ms/query) sum %ld\n", kind, c.spans.size(), q, s, 1000*s/q, sum);
  }
}
