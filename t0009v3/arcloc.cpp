// Brute-force: points of the inner offset circle (centre C, radius R) of an input arc, angles a0..a1:
// distance to input and to the branch / main output (no grids).
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0); Vec2 C{std::atof(argv[2]), std::atof(argv[3])}; double R = std::atof(argv[4]), a0 = std::atof(argv[5]), a1 = std::atof(argv[6]); int n = std::atoi(argv[7]);
    C9 cc = make9(seed); Case& c = cc.c; Contour ck = polyOf(c.in, c.tol);
    auto ob = offsetOpenContour(ck, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    auto mp = mainOffset(toP(ck), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    std::vector<xk::S> I, B, M; for (auto& s : ck.spans) if (length(s) > 0) I.push_back(xk::mk(s)); for (auto& k : ob) for (auto& s : k.spans) B.push_back(xk::mk(s)); for (auto& k : om) for (auto& s : k.spans) M.push_back(xk::mk(s));
    auto f = [](const std::vector<xk::S>& v, Vec2 p) { double b = 1e300; for (auto& g : v) b = std::min(b, xk::dPt(g, p)); return b; };
    for (int k = 0; k <= n; ++k) { double a = a0 + (a1 - a0) * k / n; Vec2 p = C + Vec2{std::cos(a), std::sin(a)} * R; double di = f(I, p);
        std::printf("ang %.4f dist-in - d %+.3g  branch-out %.4g  main-out %.4g%s\n", a, di - c.d, f(B, p), f(M, p), (di >= c.d - 1e-9 && f(B, p) > c.tol) ? "  BRANCH MISS" : ""); }
}
