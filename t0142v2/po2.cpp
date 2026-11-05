// T-0142 round-2 verifier: outset cost and output of many nested holes (resolveUnnestedHoles per batch).
// usage: po2 layout n reps [delta]   prints interleaved CPU min for n and 2n shapes, ratio, rings, output hash.
// layouts: row grid rand (separated); tcol (vertical column of touching holed squares), tgrid (touching grid),
//          hex (hexagonal packing of holed squares, boxes overlap across rows), comb (vertical column inside the
//          notches of a comb ring), plate (one big outer with n holes, each hole holding an island with a hole),
//          deep (n/levels concentric nests of 8 levels in a row)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <random>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static Contour poly(const std::vector<Vec2>& p) { Contour c; c.closed = true; for (size_t i = 0; i < p.size(); ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Contour rc(double x0, double y0, double x1, double y1, bool ccw = true) { std::vector<Vec2> p{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; if (!ccw) std::reverse(p.begin(), p.end()); return poly(p); }
static void holed(std::vector<Contour>& cs, double x, double y, double s = 1.0) { cs.push_back(rc(x, y, x + s, y + s)); cs.push_back(rc(x + 0.3 * s, y + 0.3 * s, x + 0.7 * s, y + 0.7 * s, false)); }
static std::vector<Contour> scene(const char* m, int n) {
  std::vector<Contour> cs; int k = std::max(1, (int)std::sqrt((double)n));
  if (!strcmp(m, "row")) for (int i = 0; i < n; ++i) holed(cs, 2.0 * i, 0);
  else if (!strcmp(m, "grid")) for (int i = 0; i < n; ++i) holed(cs, 2.0 * (i % k), 2.0 * (i / k));
  else if (!strcmp(m, "rand")) { std::mt19937_64 g(n); std::uniform_real_distribution<double> u(0, 0.8);
    for (int i = 0; i < n; ++i) holed(cs, 2.0 * (i % k) + u(g), 2.0 * (i / k) + u(g)); }  // jittered cells, separated
  else if (!strcmp(m, "tcol")) for (int i = 0; i < n; ++i) holed(cs, 0, 1.0 * i);
  else if (!strcmp(m, "tgrid")) for (int i = 0; i < n; ++i) holed(cs, 1.0 * (i % k), 1.0 * (i / k));
  else if (!strcmp(m, "hex")) for (int i = 0; i < n; ++i) { int r = i / k, c = i % k; holed(cs, 1.2 * c + (r % 2 ? 0.6 : 0.0), 0.9 * r); }
  else if (!strcmp(m, "comb")) {  // comb: spine x 0..1, teeth to x 3 at y 2i..2i+1 ; holed squares at x 1.5 in notches y 2i+1.2
    std::vector<Vec2> p; p.push_back({0, 0}); for (int i = 0; i < n; ++i) { p.push_back({3, 2.0 * i}); p.push_back({3, 2.0 * i + 1}); p.push_back({1, 2.0 * i + 1}); p.push_back({1, 2.0 * i + 2}); }
    p.push_back({3, 2.0 * n}); p.push_back({3, 2.0 * n + 1}); p.push_back({0, 2.0 * n + 1}); cs.push_back(poly(p));
    for (int i = 0; i < n; ++i) holed(cs, 1.5, 2.0 * i + 1.2, 0.6); }
  else if (!strcmp(m, "plate")) { cs.push_back(rc(-1, -1, 4.0 * n + 1, 5)); for (int i = 0; i < n; ++i) { cs.push_back(rc(4.0 * i, 0, 4.0 * i + 3, 3, false)); holed(cs, 4.0 * i + 0.5, 0.5, 2.0); } }
  else if (!strcmp(m, "deep")) { int lv = 8; for (int i = 0; i < n / lv; ++i) for (int l = 0; l < lv; ++l) { double e = 0.05 * l; cs.push_back(rc(2.0 * i + e, e, 2.0 * i + 1.0 - e, 1.0 - e, l % 2 == 0)); } }
  return cs; }
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
struct H { unsigned long long h = 1469598103934665603ULL; void d(double x) { auto* c = (const unsigned char*)&x; for (int i = 0; i < 8; ++i) { h ^= c[i]; h *= 1099511628211ULL; } } };
static unsigned long long hs(const std::vector<Contour>& r) { H h; for (auto& c : r) { h.d(c.closed); for (auto& s : c.spans) { h.d((double)s.index()); auto a = startPoint(s), b = endPoint(s); h.d(a.x); h.d(a.y); h.d(b.x); h.d(b.y); } h.d(-1); } return h.h; }
int main(int argc, char** argv) { const char* m = argv[1]; int n = atoi(argv[2]); int reps = argc > 3 ? atoi(argv[3]) : 3; double d = argc > 4 ? atof(argv[4]) : 0.05;
  auto a = scene(m, n), b = scene(m, 2 * n); Tolerance tol{0.001, 1e-9, 0.001}; double t1 = 1e30, t2 = 1e30; std::vector<Contour> r1, r2;
  for (int k = 0; k < reps; ++k) { double s = cpu(); r1 = offsetContours(a, d, JoinType::Round, tol); double u = cpu(); r2 = offsetContours(b, d, JoinType::Round, tol); double v = cpu(); t1 = std::min(t1, u - s); t2 = std::min(t2, v - u); }
  std::printf("po2 %s d %.3f %d->%d: %.4f %.4f s ratio %.3f rings %zu %zu hash %016llx %016llx\n", m, d, n, 2 * n, t1, t2, t2 / t1, r1.size(), r2.size(), hs(r1), hs(r2)); }
