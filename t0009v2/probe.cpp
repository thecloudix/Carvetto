// probe a coverage miss: input spans near q, output near q, valid-region thickness along normal
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0); Vec2 q{std::atof(argv[2]), std::atof(argv[3])};
    C9 cc = make9(seed); Case& c = cc.c;
    auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    Contour ck = polyOf(c.in, c.tol);
    std::printf("d=%.17g %s side %d closed %d spans %zu\n", c.d, jn(c.join), (int)c.side, ck.closed, ck.spans.size());
    int i = 0; for (auto& sp : ck.spans) { xk::S s = xk::mk(sp); double dd = xk::dPt(s, q); if (dd < c.d + 0.05) std::printf(" in %d k%d len %.3g r %.4g sw %.4g dist-d %.6g  a(%.12g,%.12g) b(%.12g,%.12g)\n", i, s.k, s.len, s.r, s.sw, dd - c.d, s.a.x, s.a.y, s.b.x, s.b.y); ++i; }
    for (size_t k = 0; k < out.size(); ++k) { int j = 0; for (auto& sp : out[k].spans) { xk::S s = xk::mk(sp); double dd = xk::dPt(s, q); if (dd < 0.5) std::printf(" out ch%zu sp%d k%d len %.4g dist %.6g a(%.12g,%.12g) b(%.12g,%.12g)\n", k, j, s.k, s.len, dd, s.a.x, s.a.y, s.b.x, s.b.y); ++j; } }
}
