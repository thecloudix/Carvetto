// Point check: v9 seed [join side] x y -> exact distance to the polygonised input (brute force), to the branch
// output and to main's output; plus a sweep along the arc of radius d about input vertex V (end of span k).
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0); C9 cc = make9(seed); Case c = cc.c;
    c.join = (JoinType)std::atoi(argv[2]); c.side = (OffsetSide)std::atoi(argv[3]); int k = std::atoi(argv[4]); double a0 = std::atof(argv[5]), a1 = std::atof(argv[6]);
    Contour ck = polyOf(c.in, c.tol);
    auto out = offsetOpenContour(ck, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    auto mp = mainOffset(toP(ck), c.d, (int)c.side, (int)c.join, c.tol, c.mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    std::vector<xk::S> in, ob, omm; for (auto& s : ck.spans) if (length(s) > 0) in.push_back(xk::mk(s));
    for (auto& o : out) for (auto& s : o.spans) ob.push_back(xk::mk(s));
    for (auto& o : om) for (auto& s : o.spans) omm.push_back(xk::mk(s));
    Vec2 V = in[k].b; std::printf("V = end of span %d (%.12g,%.12g) d %.17g join %d side %d; branch %zu chains, main %zu chains\n", k, V.x, V.y, c.d, (int)c.join, (int)c.side, out.size(), om.size());
    for (int q = 0; q <= 12; ++q) { double a = (a0 + (a1 - a0) * q / 12) * 3.14159265358979 / 180; Vec2 p = V + Vec2{std::cos(a), std::sin(a)} * c.d;
        double di = 1e300; int arg = -1; for (size_t i = 0; i < in.size(); ++i) { double v = xk::dPt(in[i], p); if (v < di) { di = v; arg = (int)i; } }
        double dob = 1e300; for (auto& s : ob) dob = std::min(dob, xk::dPt(s, p)); double dom = 1e300; for (auto& s : omm) dom = std::min(dom, xk::dPt(s, p));
        std::printf("ang %7.2f p (%.6f,%.6f) dist-to-input - d = %+.3e (nearest span %d)  dist-to-branch %.4f  dist-to-main %.4f\n", a * 180 / 3.14159265358979, p.x, p.y, di - c.d, arg, dob, dom); }
}
