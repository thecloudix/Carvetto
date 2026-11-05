#include "v9lib.cpp"
#include "xchk.hpp"
int main(int, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0);
    C9 cc = make9(seed); Case& c = cc.c;
    auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    Contour ck = polyOf(c.in, c.tol);
    xk::S o = xk::mk(out[0].spans[0]);
    std::printf("out0 k%d (%.17g,%.17g)->(%.17g,%.17g)\n", o.k, o.a.x, o.a.y, o.b.x, o.b.y);
    int i = 0;
    for (auto& sp : ck.spans) { xk::S s = xk::mk(sp); if (s.len <= 0) { ++i; continue; } double dd = xk::dSS(o, s); double ds = 1e300; for (int q = 0; q <= 2000; ++q) ds = std::min(ds, xk::dPt(s, xk::P(o, q / 2000.0))); if (dd < c.d * 0.999) std::printf("in %d k%d (%.17g,%.17g)->(%.17g,%.17g) r %.6g sw %.6g dSS %.6g sampled %.6g\n", i, s.k, s.a.x, s.a.y, s.b.x, s.b.y, s.r, s.sw, dd, ds); ++i; }
}
