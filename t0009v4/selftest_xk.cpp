// self-test of xchk.hpp exact segment distance against brute force sampling
#include "xchk.hpp"
#include <random>
using namespace carvetto::geometry;
int main() {
    std::mt19937_64 g(7); std::uniform_real_distribution<double> U(-1, 1);
    long bad = 0, n = 0; double worst = 0;
    auto rs = [&]() -> Span {
        if (g() % 2) return LineSpan{{U(g), U(g)}, {U(g), U(g)}};
        double r = std::pow(10.0, -3 * (U(g) + 1) / 2) ; ArcSpan a{{U(g), U(g)}, r, U(g) * 3.2, 0, g() % 2 ? ArcDirection::CCW : ArcDirection::CW};
        double sw = (U(g) + 1) * 3.14; a.endAngle = a.direction == ArcDirection::CCW ? a.startAngle + sw : a.startAngle - sw; return a; };
    for (int it = 0; it < 20000; ++it) {
        xk::S A = xk::mk(rs()), B = xk::mk(rs()); n++;
        double e = xk::dSS(A, B);
        const int M = 3000; double br = 1e300;
        for (int i = 0; i <= M; ++i) br = std::min(br, xk::dPt(B, xk::P(A, (double)i / M)));
        for (int i = 0; i <= M; ++i) br = std::min(br, xk::dPt(A, xk::P(B, (double)i / M)));
        double h = std::max(A.len, B.len) / M;
        if (e > br + 1e-12 || e < br - h) { bad++; if (bad < 5) std::printf("mismatch exact %.9g brute %.9g h %.3g kinds %d %d\n", e, br, h, A.k, B.k); }
        worst = std::max(worst, e - br);
    }
    std::printf("selftest dSS: %ld pairs, %ld mismatches, max exact-brute %.3g\n", n, bad, worst);
}
// dPt self-test
struct DP { DP() {
    std::mt19937_64 g(9); std::uniform_real_distribution<double> U(-1, 1); long bad = 0;
    for (int it = 0; it < 5000; ++it) {
        ArcSpan a{{U(g), U(g)}, (U(g) + 1.01) / 2, U(g) * 3.2, 0, g() % 2 ? ArcDirection::CCW : ArcDirection::CW};
        double sw = (U(g) + 1) * 3.14; a.endAngle = a.direction == ArcDirection::CCW ? a.startAngle + sw : a.startAngle - sw;
        xk::S s = xk::mk(a); Vec2 p{U(g) * 2, U(g) * 2};
        double br = 1e300; for (int i = 0; i <= 20000; ++i) br = std::min(br, distance(p, a.pointAt(i / 20000.0)));
        double e = xk::dPt(s, p); if (e > br + 1e-12 || e < br - s.len / 20000) bad++;
    }
    std::printf("selftest dPt arc: 5000, %ld mismatches\n", bad);
} } dp_;
