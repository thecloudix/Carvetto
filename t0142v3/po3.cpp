// T-0142 round-3 verifier (from t0142v2/po2.cpp): outset cost and output of many nested holes.
// New layouts: tcolrev (column, all outers first then holes reversed: the last-outer shortcut always misses),
// tcolshuf (column shuffled), hexrev, cring (n/2 concentric C rings whose boxes hold n/2 central holes but whose
// insides do not, inside a big outer), ycomb (comb whose teeth run along y, a hole in every tooth: each test line
// crosses every tooth), xcombrev (comb teeth along x, holes in teeth, comb listed after the holes).
// Mode 'nest' (5th arg = 1): time only detail::everyHoleNested on the scene scaled by 1e6 (rectilinear: exact), interleaved.
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
#ifdef WITH_NEST
#include "clipper2/clipper.h"
#include "hole_nesting.hpp"
#endif
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
  else if (!strcmp(m, "tcolrev")) { for (int i = 0; i < n; ++i) cs.push_back(rc(0, i, 1, i + 1)); for (int i = n; i-- > 0;) cs.push_back(rc(0.3, i + 0.3, 0.7, i + 0.7, false)); }
  else if (!strcmp(m, "tcolshuf")) { for (int i = 0; i < n; ++i) holed(cs, 0, 1.0 * i); std::mt19937_64 g(n); std::shuffle(cs.begin(), cs.end(), g); }
  else if (!strcmp(m, "hexrev")) { std::vector<Contour> h; for (int i = 0; i < n; ++i) { int r = i / k, c = i % k; holed(cs, 1.2 * c + (r % 2 ? 0.6 : 0.0), 0.9 * r); } for (size_t i = 1; i < cs.size(); i += 2) h.push_back(cs[i]); std::vector<Contour> o; for (size_t i = 0; i < cs.size(); i += 2) o.push_back(cs[i]); std::reverse(h.begin(), h.end()); cs = o; cs.insert(cs.end(), h.begin(), h.end()); }
  else if (!strcmp(m, "cring")) { int kc = n / 2, mh = n / 2; double W = 4.0 * kc + 20; cs.push_back(rc(-1, -1, W + 1, W + 1));
    for (int i = 0; i < kc; ++i) { double e = 2.0 * i, x0 = e, y0 = e, x1 = W - e, y1 = W - e, th = 0.5;
      cs.push_back(poly({{x0, y0}, {x1, y0}, {x1, y0 + th}, {x0 + th, y0 + th}, {x0 + th, y1 - th}, {x1, y1 - th}, {x1, y1}, {x0, y1}})); }
    int q = std::max(1, (int)std::sqrt((double)mh)); double c0 = W / 2 - 8, cell = 16.0 / q; for (int i = 0; i < mh; ++i) { double x = c0 + cell * (i % q), y = c0 + cell * (i / q); cs.push_back(rc(x + 0.2 * cell, y + 0.2 * cell, x + 0.6 * cell, y + 0.6 * cell, false)); } }
  else if (!strcmp(m, "ycomb") || !strcmp(m, "xcombrev")) { bool y = !strcmp(m, "ycomb"); auto Q = [&](double a, double b) { return y ? Vec2{b, a} : Vec2{a, b}; };
    std::vector<Vec2> p; p.push_back(Q(0, 0)); for (int i = 0; i < n; ++i) { p.push_back(Q(3, 2.0 * i)); p.push_back(Q(3, 2.0 * i + 1)); p.push_back(Q(1, 2.0 * i + 1)); p.push_back(Q(1, 2.0 * i + 2)); }
    p.push_back(Q(3, 2.0 * n)); p.push_back(Q(3, 2.0 * n + 1)); p.push_back(Q(0, 2.0 * n + 1)); Contour comb = poly(p); if (y) comb = comb.reversed();
    std::vector<Contour> h; for (int i = 0; i < n; ++i) { Vec2 a = Q(1.5, 2.0 * i + 0.3), b = Q(2.5, 2.0 * i + 0.7); h.push_back(rc(std::min(a.x, b.x), std::min(a.y, b.y), std::max(a.x, b.x), std::max(a.y, b.y), false)); }
    if (y) { cs.push_back(comb); cs.insert(cs.end(), h.begin(), h.end()); } else { cs = h; cs.push_back(comb); } }
  else if (!strcmp(m, "plate")) { cs.push_back(rc(-1, -1, 4.0 * n + 1, 5)); for (int i = 0; i < n; ++i) { cs.push_back(rc(4.0 * i, 0, 4.0 * i + 3, 3, false)); holed(cs, 4.0 * i + 0.5, 0.5, 2.0); } }
  else if (!strcmp(m, "deep")) { int lv = 8; for (int i = 0; i < n / lv; ++i) for (int l = 0; l < lv; ++l) { double e = 0.05 * l; cs.push_back(rc(2.0 * i + e, e, 2.0 * i + 1.0 - e, 1.0 - e, l % 2 == 0)); } }
  return cs; }
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
struct H { unsigned long long h = 1469598103934665603ULL; void d(double x) { auto* c = (const unsigned char*)&x; for (int i = 0; i < 8; ++i) { h ^= c[i]; h *= 1099511628211ULL; } } };
static unsigned long long hs(const std::vector<Contour>& r) { H h; for (auto& c : r) { h.d(c.closed); for (auto& s : c.spans) { h.d((double)s.index()); auto a = startPoint(s), b = endPoint(s); h.d(a.x); h.d(a.y); h.d(b.x); h.d(b.y); } h.d(-1); } return h.h; }
#ifdef WITH_NEST
static Clipper2Lib::Paths64 toPaths(const std::vector<Contour>& cs) { Clipper2Lib::Paths64 P; for (auto& c : cs) { Clipper2Lib::Path64 p; for (auto& sp : c.spans) { auto a = startPoint(sp); p.emplace_back((int64_t)std::llround(a.x * 1e6), (int64_t)std::llround(a.y * 1e6)); } P.push_back(p); } return P; }
static std::vector<int> kinds(const Clipper2Lib::Paths64& P) { std::vector<int> k(P.size()); for (size_t i = 0; i < P.size(); ++i) { double a = Clipper2Lib::Area(P[i]); k[i] = a == 0 ? 0 : (a > 0 ? 1 : 2); } return k; }  // lowest is CCW in all layouts
#endif
int main(int argc, char** argv) { const char* m = argv[1]; int n = atoi(argv[2]); int reps = argc > 3 ? atoi(argv[3]) : 3; double d = argc > 4 ? atof(argv[4]) : 0.05;
  bool nestOnly = argc > 5 && atoi(argv[5]) == 1;
  auto a = scene(m, n), b = scene(m, 2 * n); Tolerance tol{0.001, 1e-9, 0.001}; double t1 = 1e30, t2 = 1e30; std::vector<Contour> r1, r2;
  if (nestOnly) {
#ifdef WITH_NEST
    auto pa = toPaths(a), pb = toPaths(b); auto ka = kinds(pa), kb = kinds(pb); bool na = false, nb = false;
    for (int k = 0; k < reps; ++k) { double s = cpu(); na = carvetto::geometry::detail::everyHoleNested(pa, ka); double u = cpu(); nb = carvetto::geometry::detail::everyHoleNested(pb, kb); double v = cpu(); t1 = std::min(t1, u - s); t2 = std::min(t2, v - u); }
    std::printf("nest %s %d->%d: %.4f %.4f s ratio %.3f allNested %d %d\n", m, n, 2 * n, t1, t2, t2 / t1, na, nb);
#endif
    return 0; }
  for (int k = 0; k < reps; ++k) { double s = cpu(); r1 = offsetContours(a, d, JoinType::Round, tol); double u = cpu(); r2 = offsetContours(b, d, JoinType::Round, tol); double v = cpu(); t1 = std::min(t1, u - s); t2 = std::min(t2, v - u); }
  std::printf("po3 %s d %.3f %d->%d: %.4f %.4f s ratio %.3f rings %zu %zu hash %016llx %016llx\n", m, d, n, 2 * n, t1, t2, t2 / t1, r1.size(), r2.size(), hs(r1), hs(r2)); }
