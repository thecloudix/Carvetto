// Brute-force confirmation for v9 0xf0003fad Round Left: points on the circle of radius d about the
// tip T of the sub-tol spike (input spans 20/21) between the left normal of span 20 and that of span 22:
// distance to the whole (polygonised) input by brute force over every span, and to the branch output.
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    std::uint64_t seed = argc > 1 ? std::strtoull(argv[1], nullptr, 0) : 0xf0003fad; C9 cc = make9(seed); Case c = cc.c;
    int j = argc > 2 ? std::atoi(argv[2]) : 0, sd = argc > 3 ? std::atoi(argv[3]) : 0; int spike = argc > 4 ? std::atoi(argv[4]) : 20;
    double a0 = argc > 5 ? std::atof(argv[5]) : 145, a1 = argc > 6 ? std::atof(argv[6]) : 81;
    c.join = (JoinType)j; c.side = (OffsetSide)sd;
    Contour ck = polyOf(c.in, c.tol);
    auto out = offsetOpenContour(ck, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    auto mp = mainOffset(toP(ck), c.d, sd, j, c.tol, c.mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    std::vector<xk::S> in, ob, omm; for (auto& s : ck.spans) if (length(s) > 0) in.push_back(xk::mk(s));
    for (auto& o : out) for (auto& s : o.spans) ob.push_back(xk::mk(s));
    for (auto& o : om) for (auto& s : o.spans) omm.push_back(xk::mk(s));
    Vec2 T = in[spike].b; std::printf("tip T (%.12g,%.12g) d %.17g\n", T.x, T.y, c.d);
    for (int q = 0; q <= 16; ++q) { double a = (a0 + (a1 - a0) * q / 16) * 3.14159265358979 / 180; Vec2 p = T + Vec2{std::cos(a), std::sin(a)} * c.d;
        double di = 1e300; int arg = -1; for (size_t i = 0; i < in.size(); ++i) { double v = xk::dPt(in[i], p); if (v < di) { di = v; arg = (int)i; } }
        double dob = 1e300; for (auto& s : ob) dob = std::min(dob, xk::dPt(s, p)); double dom = 1e300; for (auto& s : omm) dom = std::min(dom, xk::dPt(s, p));
        std::printf("ang %7.2f p (%.6f,%.6f) dist-to-input - d = %+.3e (nearest span %d)  dist-to-branch-output %.4f  dist-to-main-output %.4f\n", a * 180 / 3.14159265358979, p.x, p.y, di - c.d, arg, dob, dom); }
}
