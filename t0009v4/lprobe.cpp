// lprobe <g2|v9> <seed> <join> <side> <k> [N] [u0 u1]: sample the requested-side offset locus of checker span k (zero-length spans
// dropped, as in xk): validity (>= d from all; >= d from all but k), distance to the branch and to main's output.
#include "v9lib.cpp"
#include "xchk.hpp"
#include "adv.inc"
#include "g2gen.inc"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[2], nullptr, 0); Contour in; double d, mitre, tol;
    if (argv[1][0] == 'g') { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else if (argv[1][0] == 'a') { AdvCase c = makeAdv(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; }
    JoinType join = (JoinType)std::atoi(argv[3]); OffsetSide side = (OffsetSide)std::atoi(argv[4]); int k = std::atoi(argv[5]); int N = argc > 6 ? std::atoi(argv[6]) : 200;
    double u0 = argc > 8 ? std::atof(argv[7]) : 0, u1 = argc > 8 ? std::atof(argv[8]) : 1;
    Contour ck = polyOf(in, tol);
    auto ob = offsetOpenContour(ck, d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre);
    auto mp = mainOffset(toP(ck), d, (int)side, (int)join, tol, mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    xk::Checker XB(ck, ob, d, side, join, tol, mitre), XM(ck, om, d, side, join, tol, mitre);
    const xk::S& g = XB.in[k]; int sg = side == OffsetSide::Left ? 1 : -1;
    std::printf("span %d kind %d len %.6g (%.12g,%.12g)->(%.12g,%.12g) inRun %d\n", k, g.k, g.len, g.a.x, g.a.y, g.b.x, g.b.y, (int)XB.inRun(k));
    for (int q = 0; q <= N; ++q) { double u = u0 + (u1 - u0) * q / N; Vec2 p = xk::P(g, u) + xk::T(g, u).perp() * (sg * d);
        double all = XB.fIn(p, d + 1), oth = XB.fIn(p, d + 1, k, -1); double b = XB.fOut(p, d + 1), m = XM.fOut(p, d + 1);
        bool valid = all >= d - 1e-9 && oth >= d + 1e-9;
        if (valid || argc > 9) std::printf("u %.5f p (%.9f,%.9f) all-d %+.2e oth-d %+.2e valid %d branch %.5f main %.5f\n", u, p.x, p.y, all - d, oth - d, (int)valid, b, m); }
}
