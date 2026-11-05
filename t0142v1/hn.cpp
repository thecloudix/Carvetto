// T-0142 verifier (b): outsets of properly nested input (no unnested opposite rings) must be byte-identical to main.
// usage: hn seed N
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
using Ring = std::vector<Vec2>;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static Contour poly(const Ring& p) { Contour c; c.closed = true; for (size_t i = 0; i < p.size(); ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Ring star(Vec2 c, double R, double jit, int n, bool ccw) { Ring r; double ph = U(0, 6.28); for (int k = 0; k < n; ++k) { double a = ph + 6.283185307179586 * k / n, rr = R * U(1 - jit, 1.0); r.push_back(c + Vec2{rr * std::cos(a), rr * std::sin(a)}); } if (!ccw) std::reverse(r.begin(), r.end()); return r; }
struct H { unsigned long long h = 1469598103934665603ULL; void b(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; } } void d(double x) { b(&x, 8); } };
int main(int argc, char** argv) { unsigned long seed = strtoul(argv[1], 0, 10); int N = atoi(argv[2]);
  for (int sc = 0; sc < N; ++sc) { rng.seed(seed * 99991 + sc); int G = I(1, 80); bool rev = I(0, 3) == 0; std::vector<Contour> cs;
    for (int g = 0; g < G; ++g) { Vec2 c{5.0 * (g % 12), 5.0 * (g / 12)}; double R = U(1, 2.2); cs.push_back(poly(star(c, R, 0.3, I(3, 40), !rev)));
      int kids = I(0, 2); if (kids >= 1) cs.push_back(poly(star(c, R * 0.3, 0.3, I(3, 20), rev))); if (kids == 2) cs.push_back(poly(star(c, R * 0.1, 0.2, I(3, 12), !rev))); }
    for (double d : {0.02, 0.1, 0.3, 0.9}) for (int j = 0; j < 3; ++j) { H h; auto out = offsetContours(cs, d, (JoinType)j, Tolerance{0.001, 1e-9, 0.001}, 2.0);
      for (auto& o : out) for (auto& s : o.spans) { h.d(startPoint(s).x); h.d(startPoint(s).y); } std::printf("sc %d G %d d %.2f j %d n %zu %016llx\n", sc, G, d, j, out.size(), h.h); } } }
