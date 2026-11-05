// T-0142 verifier (a): booleanOp scaling, interleaved CPU time (min of reps of n and 2n alternately).
// usage: pb mode op n reps   mode row|grid|jit|holed|nest|pair  op union|diff|inter|xor|unionc
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <random>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
using namespace carvetto::geometry;
static Contour poly(const std::vector<Vec2>& p) { Contour c; c.closed = true; for (size_t i = 0; i < p.size(); ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Contour rc(double x0, double y0, double x1, double y1, bool ccw = true) { std::vector<Vec2> p{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; if (!ccw) std::reverse(p.begin(), p.end()); return poly(p); }
static void scene(const char* m, int n, std::vector<Contour>& S, std::vector<Contour>& C) {
  std::mt19937_64 r(7); auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(r); };
  int k = (int)std::sqrt((double)n);
  for (int i = 0; i < n; ++i) {
    double x = 0.7 * i, y = 0, h = i % 2 ? 0.2 : 0.3;
    if (!strcmp(m, "grid")) { x = 0.7 * (i % k); y = 0.7 * (i / k); }
    if (!strcmp(m, "jit")) { y = U(0, 0.1); h += U(0, 0.1); }
    S.push_back(rc(x, y, x + 0.5, y + h));
    if (!strcmp(m, "holed") || !strcmp(m, "nest")) S.push_back(rc(x + 0.1, y + 0.05, x + 0.4, y + h - 0.05, false));
    if (!strcmp(m, "nest")) S.push_back(rc(x + 0.2, y + 0.08, x + 0.3, y + h - 0.08));
    C.push_back(rc(x + 0.2, y + 0.1, x + 0.6, y + 0.25));
  }
}
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
int main(int argc, char** argv) {
  const char* m = argv[1]; const char* op = argv[2]; int n = atoi(argv[3]); int reps = argc > 4 ? atoi(argv[4]) : 3;
  std::vector<Contour> S1, C1, S2, C2; scene(m, n, S1, C1); scene(m, 2 * n, S2, C2); Tolerance tol{0.001, 1e-9, 0.001};
  auto run = [&](std::vector<Contour>& S, std::vector<Contour>& C) { if (!strcmp(op, "unionc")) return unionContours(S, tol).size();
    BooleanOp b = !strcmp(op, "union") ? BooleanOp::Union : !strcmp(op, "diff") ? BooleanOp::Difference : !strcmp(op, "inter") ? BooleanOp::Intersection : BooleanOp::Xor;
    return booleanOp(b, S, C, tol).closed.size(); };
  double t1 = 1e30, t2 = 1e30; size_t r1 = 0, r2 = 0;
  for (int k = 0; k < reps; ++k) { double a = cpu(); r1 = run(S1, C1); double b = cpu(); r2 = run(S2, C2); double c = cpu(); t1 = std::min(t1, b - a); t2 = std::min(t2, c - b); }
  std::printf("%s %s %d->%d: %.4f %.4f s ratio %.3f rings %zu %zu\n", m, op, n, 2 * n, t1, t2, t2 / t1, r1, r2);
}
