// T-0142 verifier (a): byte hash of booleanOp/unionContours/offsetContours output on scenes with
// G groups side by side (G < 32: must equal main byte for byte; G >= 32 informational).
// usage: hb seed N   -> one line per scene: G op fill hash
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
using Ring = std::vector<Vec2>;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static Contour poly(const Ring& p, bool closed = true) { Contour c; c.closed = closed; size_t n = closed ? p.size() : p.size() - 1;
  for (size_t i = 0; i < n; ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Ring star(Vec2 c, double R, int n, bool ccw) { Ring r; double ph = U(0, 6.28);
  for (int k = 0; k < n; ++k) { double a = ph + 6.283185307179586 * k / n, rr = R * U(0.45, 1.0); r.push_back(c + Vec2{rr * std::cos(a), rr * std::sin(a)}); }
  if (!ccw) std::reverse(r.begin(), r.end()); return r; }
struct H { unsigned long long h = 1469598103934665603ULL; void b(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; } } void d(double x) { b(&x, 8); } void i(long long x) { b(&x, 8); } };
static void hc(H& h, const std::vector<Contour>& cs) { h.i((long long)cs.size()); for (auto& c : cs) { h.i(c.closed); h.i((long long)c.spans.size()); for (auto& s : c.spans) { Vec2 a = startPoint(s), e = endPoint(s); h.d(a.x); h.d(a.y); h.d(e.x); h.d(e.y); } } }
int main(int argc, char** argv) {
  unsigned long seed = strtoul(argv[1], 0, 10); int N = atoi(argv[2]);
  for (int sc = 0; sc < N; ++sc) {
    rng.seed(seed * 31337 + sc);
    int G = I(1, 3) == 1 ? I(28, 31) : I(2, 64);
    std::vector<Contour> S, C;
    for (int g = 0; g < G; ++g) { Vec2 c{4.0 * g + U(-0.2, 0.2), U(-0.2, 0.2)}; bool o = I(0, 4) != 0; S.push_back(poly(star(c, U(0.5, 1.5), I(3, 30), o)));
      if (I(0, 1)) S.push_back(poly(star(c, 0.3, I(3, 9), !o))); if (I(0, 1)) C.push_back(poly(star(c + Vec2{0.5, 0.3}, U(0.3, 1.2), I(3, 20), I(0, 3) != 0)));
      if (I(0, 2) == 0) S.push_back(poly({c - Vec2{2, U(-1, 1)}, c + Vec2{2, U(-1, 1)}}, false)); }
    Tolerance tol{0.001, 1e-9, 0.001};
    for (int op = 0; op < 4; ++op) for (int f = 0; f < 2; ++f) { H h; BooleanResult r = booleanOp((BooleanOp)op, S, C, tol, f ? FillRule::EvenOdd : FillRule::NonZero); hc(h, r.closed); hc(h, r.open);
      std::printf("sc %d G %d op %d f %d %016llx\n", sc, G, op, f, h.h); }
    { H h; hc(h, unionContours(S, tol)); std::printf("sc %d G %d union %016llx\n", sc, G, h.h); }
    // offsets of the closed subjects: outsets with lone holes (resolve path) and insets
    std::vector<Contour> cl; for (auto& c : S) if (c.closed) cl.push_back(c);
    for (double d : {-0.05, 0.05, 0.4}) { H h; hc(h, offsetContours(cl, d, JoinType::Round, tol)); std::printf("sc %d G %d off %.2f %016llx\n", sc, G, d, h.h); }
  }
}
