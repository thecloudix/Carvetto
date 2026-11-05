// T-0142 verifier: outset cost with many properly nested holes (resolveUnnestedHoles nesting scan).
// usage: po mode n reps   mode row|grid|text ; interleaved n / 2n CPU min
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static Contour poly(const std::vector<Vec2>& p) { Contour c; c.closed = true; for (size_t i = 0; i < p.size(); ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Contour rc(double x0, double y0, double x1, double y1, bool ccw = true) { std::vector<Vec2> p{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; if (!ccw) std::reverse(p.begin(), p.end()); return poly(p); }
static std::vector<Contour> scene(const char* m, int n) { std::vector<Contour> cs; int k = (int)std::sqrt((double)n);
  for (int i = 0; i < n; ++i) { double x = 2.0 * i, y = 0; if (!strcmp(m, "grid")) { x = 2.0 * (i % k); y = 2.0 * (i / k); }
    cs.push_back(rc(x, y, x + 1.0, y + 1.0)); cs.push_back(rc(x + 0.3, y + 0.3, x + 0.7, y + 0.7, false)); } return cs; }
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
int main(int argc, char** argv) { const char* m = argv[1]; int n = atoi(argv[2]); int reps = argc > 3 ? atoi(argv[3]) : 3; double d = argc > 4 ? atof(argv[4]) : 0.05;
  auto a = scene(m, n), b = scene(m, 2 * n); Tolerance tol{0.001, 1e-9, 0.001}; double t1 = 1e30, t2 = 1e30; size_t r1 = 0, r2 = 0;
  for (int k = 0; k < reps; ++k) { double s = cpu(); r1 = offsetContours(a, d, JoinType::Round, tol).size(); double u = cpu(); r2 = offsetContours(b, d, JoinType::Round, tol).size(); double v = cpu(); t1 = std::min(t1, u - s); t2 = std::min(t2, v - u); }
  std::printf("po %s d %.3f %d->%d: %.4f %.4f s ratio %.3f rings %zu %zu\n", m, d, n, 2 * n, t1, t2, t2 / t1, r1, r2); }
