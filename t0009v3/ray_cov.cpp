// Independent coverage cross-check by rays: from a centre V, for each direction find the first point at
// distance exactly d from the input (bisection on the ray), report its distance to the output.
#include "xchk.hpp"
#include <cstdio>
using namespace carvetto::geometry;
int main(int argc, char** argv) {
#include CASE
    double d = XD; OffsetSide side = XSIDE; JoinType j = XJOIN;
    Vec2 V{std::atof(argv[1]), std::atof(argv[2])}; double a0 = std::atof(argv[3]), a1 = std::atof(argv[4]); int n = std::atoi(argv[5]);
    auto out = offsetOpenContour(in, d, side, j, Tolerance{}, 2.0);
    std::vector<xk::S> I, O; for (auto& s : in.spans) if (length(s) > 0) I.push_back(xk::mk(s)); for (auto& c : out) for (auto& s : c.spans) O.push_back(xk::mk(s));
    auto fI = [&](Vec2 p) { double b = 1e300; for (auto& g : I) b = std::min(b, xk::dPt(g, p)); return b; };
    auto fO = [&](Vec2 p) { double b = 1e300; for (auto& g : O) b = std::min(b, xk::dPt(g, p)); return b; };
    int miss = 0;
    for (int k = 0; k <= n; ++k) {
        double an = a0 + (a1 - a0) * k / n; Vec2 u{std::cos(an), std::sin(an)};
        double lo = 0, hi = 3 * d; if (fI(V + u * hi) < d) { std::printf("ang %.4f: ray never leaves\n", an); continue; }
        for (int it = 0; it < 200; ++it) { double m = (lo + hi) / 2; if (fI(V + u * m) < d) lo = m; else hi = m; }
        Vec2 q = V + u * hi; double o = fO(q); miss += o > 1e-3;
        std::printf("ang %.4f q(%.10f,%.10f) dist-in %.12f dist-out %.4g%s\n", an, q.x, q.y, fI(q), o, o > 1e-3 ? "  MISS" : "");
    }
    std::printf("%d of %d boundary points farther than tol from the output\n", miss, n + 1);
}
