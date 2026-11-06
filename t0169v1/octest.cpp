// t0169v1 octest: oc2 arc/Bezier distance vs dense sampling (oracle self-check)
#include <cstdio>
#include <random>
#include "oc2.hpp"
using namespace carvetto::geometry;
int main() { std::mt19937_64 g(5); auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(g); }; double worstLow = 0, worstHigh = 0; int bad = 0;
  for (int k = 0; k < 4000; ++k) { double far = k % 4 == 0 ? 1e5 : 0; Vec2 a{far + U(-5, 5), U(-5, 5)}, b{far + U(-5, 5), U(-5, 5)}; if (k % 3 == 0) b = a + Vec2{U(-1e-3, 1e-3), U(-1e-3, 1e-3)};
    Span s; if (k % 2) { double R = std::pow(10, U(-2, 3)); Vec2 c{far + U(-5, 5) - (k % 5 == 0 ? R : 0), U(-5, 5)}; s = ArcSpan{c, R, U(-7, 7), U(-7, 7), k % 4 < 2 ? ArcDirection::CW : ArcDirection::CCW}; }
    else s = BezierSpan{{far + U(-5, 5), U(-5, 5)}, {far + U(-8, 8), U(-8, 8)}, {far + U(-8, 8), U(-8, 8)}, {far + U(-5, 5), U(-5, 5)}};
    bool cert = true; double d = oc2::dist(s, a, b, -1, 1e-10, cert);
    double sm = 1e300; const int N = 200000; for (int i = 0; i <= N; ++i) { double t = double(i) / N; Vec2 p; if (auto* ar = std::get_if<ArcSpan>(&s)) p = ar->pointAt(t); else p = std::get<BezierSpan>(s).pointAt(t); sm = std::min(sm, oc2::dps(p, a, b)); }
    // crossing => 0; sampled min is an upper bound; oracle must be <= sampled min (+1e-12) and close to it
    double hi = d - sm; if (hi > worstHigh) worstHigh = hi; double lo = sm - d; if (lo > worstLow) worstLow = lo; if (hi > 1e-9 || lo > 1e-3) { ++bad; if (bad < 5) std::printf("k %d oracle %.12g sampled %.12g\n", k, d, sm); } }
  std::printf("oracle above sampled by at most %.3e (must be ~0), below by at most %.3e (sampling resolution), bad %d\n", worstHigh, worstLow, bad); }
