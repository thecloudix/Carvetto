// ptq <v9 seed> <join> <side> x y [x y ...]: exact distance of each point to the polygonised input (brute force,
// minus d), to the branch output and to main's output.
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0); C9 cc = make9(seed); Case c = cc.c;
    c.join = (JoinType)std::atoi(argv[2]); c.side = (OffsetSide)std::atoi(argv[3]);
    Contour ck = polyOf(c.in, c.tol);
    auto out = offsetOpenContour(ck, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    auto mp = mainOffset(toP(ck), c.d, (int)c.side, (int)c.join, c.tol, c.mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    std::vector<xk::S> in, ob, omm; for (auto& s : ck.spans) if (length(s) > 0) in.push_back(xk::mk(s));
    for (auto& o : out) for (auto& s : o.spans) ob.push_back(xk::mk(s));
    for (auto& o : om) for (auto& s : o.spans) omm.push_back(xk::mk(s));
    for (int a = 4; a + 1 < argc; a += 2) { Vec2 p{std::atof(argv[a]), std::atof(argv[a + 1])};
        double di = 1e300; int arg = -1; for (size_t i = 0; i < in.size(); ++i) { double v = xk::dPt(in[i], p); if (v < di) { di = v; arg = (int)i; } }
        double dob = 1e300; for (auto& s : ob) dob = std::min(dob, xk::dPt(s, p)); double dom = 1e300; for (auto& s : omm) dom = std::min(dom, xk::dPt(s, p));
        std::printf("p (%.9f,%.9f) dist-to-input - d = %+.3e (nearest span %d)  dist-to-branch %.5f  dist-to-main %.5f\n", p.x, p.y, di - c.d, arg, dob, dom); }
}
