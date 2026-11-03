// T-0159 verifier: fuzz the line/circle part of offset.cpp carrierIntersections (main vs branch,
// copied verbatim apart from Vec2) against __float128 (113-bit) reference arithmetic.
// usage: tangent_fuzz [trials]   build twice: -O2 and -O2 -march=x86-64-v3 (FMA contraction)
// Reports per family/scale: count of 1-vs-2 roots for main and branch, max distance error when
// merged (true r-h), max half-chord merged, and wrong "miss" (0 roots when true crossing exists).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <quadmath.h>
#include <random>
#include <vector>
struct V { double x, y; V operator+(V o) const { return {x + o.x, y + o.y}; } V operator-(V o) const { return {x - o.x, y - o.y}; }
  V operator*(double s) const { return {x * s, y * s}; } double dot(V o) const { return x * o.x + y * o.y; }
  double cross(V o) const { return x * o.y - y * o.x; } double l2() const { return x * x + y * y; } };
static std::vector<V> inter(V s, V e, V c, double rad, bool branch) {
  std::vector<V> out; const V u = e - s; const V w = s - c; const double qa = u.l2(); const double qb = 2.0 * w.dot(u);
  const double qc = w.l2() - rad * rad; double disc = qb * qb - 4.0 * qa * qc;
  if (branch) { const double h = std::fabs(w.cross(u)) / std::sqrt(qa);
    const double scale = std::max({1.0, rad, std::fabs(s.x), std::fabs(s.y), std::fabs(e.x), std::fabs(e.y), std::fabs(c.x), std::fabs(c.y)});
    if (std::fabs(h - rad) <= 64.0 * std::numeric_limits<double>::epsilon() * scale) disc = 0.0; }
  if (disc < 0.0) { if (disc < -1e-9 * qa * rad * rad) return out; disc = 0.0; }
  const double root = std::sqrt(disc);
  out.push_back(s + u * ((-qb - root) / (2.0 * qa))); if (root > 0.0) out.push_back(s + u * ((-qb + root) / (2.0 * qa)));
  return out;
}
typedef __float128 Q;
// exact-ish signed (r - h) and half chord
static void ref(V s, V e, V c, double rad, Q& rminush, Q& half) {
  Q ux = (Q)e.x - s.x, uy = (Q)e.y - s.y, wx = (Q)s.x - c.x, wy = (Q)s.y - c.y;
  Q cr = wx * uy - wy * ux; Q h = fabsq(cr) / sqrtq(ux * ux + uy * uy); rminush = (Q)rad - h;
  Q r = rad; half = rminush > 0 ? sqrtq((r - h) * (r + h)) : 0;
}
int main(int argc, char** argv) {
  long trials = argc > 1 ? atol(argv[1]) : 200000;
  std::mt19937_64 rng(159);
  auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
  const double offs[] = {0, 1e2, 1e4, 1e6, 1e8};
  // families: 0 tangent by construction (offset line from collinear vertex, as Profile/offset do),
  //           1 crossing with r-h = 10^k * eps*scale (k in -2..3), 2 miss by same
  printf("%-4s %-6s %-6s %9s %9s %9s %9s %12s %12s %12s %8s\n", "fam", "off", "k", "n", "main2", "br2", "br0", "maxMergedRH", "maxHalfMrg", "maxPtErr", "badMiss");
  for (int fam = 0; fam < 3; ++fam) for (double off : offs) for (int k = (fam == 0 ? 0 : -2); k <= (fam == 0 ? 0 : 3); ++k) {
    long n = 0, m2 = 0, b2 = 0, b0 = 0, badMiss = 0; double maxRH = 0, maxHalf = 0, maxPt = 0;
    for (long t = 0; t < trials / 20; ++t) {
      double rad = std::pow(10.0, U(-2, 2)); double hd = U(-M_PI, M_PI); V dir{std::cos(hd), std::sin(hd)};
      V p1{off + U(-200, 200), off + U(-200, 200)}; double L0 = U(1, 50), L1 = U(1, 50);
      V s, e, c;
      if (fam == 0) { // input vertex p1 collinear: offset line = input line shifted by rad along normal; circle centre p1, radius rad
        V nrm{-dir.y, dir.x}; V p0 = p1 - dir * L0; V p2 = p1 + dir * L1; (void)p2;
        s = p0 + nrm * rad; e = p1 + nrm * rad; c = p1;
      } else {
        double scale = std::max({1.0, rad, std::fabs(p1.x) + 300, std::fabs(p1.y) + 300});
        double target = std::pow(10.0, k) * std::numeric_limits<double>::epsilon() * scale; // desired |r-h|
        V nrm{-dir.y, dir.x}; double h = fam == 1 ? rad - target : rad + target; c = p1;
        s = p1 - dir * L0 + nrm * h; e = p1 + dir * L1 + nrm * h;
      }
      Q rh, half; ref(s, e, c, rad, rh, half);
      auto am = inter(s, e, c, rad, false), ab = inter(s, e, c, rad, true);
      ++n; if (am.size() == 2) ++m2; if (ab.size() == 2) ++b2; if (ab.empty()) ++b0;
      if (ab.size() == 1 && rh > 0) { maxRH = std::max(maxRH, (double)rh); maxHalf = std::max(maxHalf, (double)half); }
      if (ab.empty() && rh > 64 * 2.3e-16 * std::max(1.0, std::fabs(off) + 300) && rh > 1.25e-10 * rad * 2) ++badMiss;
      // point error: each returned point's distance from the circle and from the line (in Q)
      for (V p : ab) { Q dx = (Q)p.x - c.x, dy = (Q)p.y - c.y; Q dc = fabsq(sqrtq(dx * dx + dy * dy) - rad);
        maxPt = std::max(maxPt, (double)dc); }
    }
    printf("%-4d %-6.0e %-6d %9ld %9ld %9ld %9ld %12.3e %12.3e %12.3e %8ld\n", fam, off, k, n, m2, b2, b0, maxRH, maxHalf, maxPt, badMiss);
  }
}
