// mcover <v9|g2> <seed> <join> <side>: sample main's output; report points that are a true offset point of the
// input (|dist-to-input - d| <= 1e-6, brute force) and farther than 3 tol from the branch output. Stretch = sampled
// length of such points. Independent of the checker's loci and exemptions.
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[2], nullptr, 0); C9 cc = make9(seed); Case c = cc.c;
    int j = std::atoi(argv[3]), s = std::atoi(argv[4]); Contour ck = polyOf(c.in, c.tol);
    auto ob = offsetOpenContour(ck, c.d, (OffsetSide)s, (JoinType)j, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    auto mp = mainOffset(toP(ck), c.d, s, j, c.tol, c.mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    std::vector<xk::S> in, b; for (auto& sp : ck.spans) if (length(sp) > 0) in.push_back(xk::mk(sp)); for (auto& o : ob) for (auto& sp : o.spans) b.push_back(xk::mk(sp));
    double stretch = 0, worst = 0; Vec2 wp{}; int nm = 0;
    for (auto& o : om) for (auto& sp : o.spans) { auto q = xk::mk(sp); int N = std::max(2, (int)std::ceil(q.len / 2e-4)); N = std::min(N, 20000);
        for (int i = 0; i <= N; ++i) { Vec2 p = xk::P(q, (double)i / N); double di = 1e300; for (auto& g : in) di = std::min(di, xk::dPt(g, p));
            if (std::fabs(di - c.d) > 1e-6 * std::max(1.0, c.d)) continue; double db = 1e300; for (auto& g : b) db = std::min(db, xk::dPt(g, p));
            if (db > 3 * c.tol) { stretch += q.len / N; nm++; if (db > worst) { worst = db; wp = p; } } } }
    std::printf("mcover %s %s %s side=%d d=%.6g: main-only true-offset stretch %.4g mm (%d samples), worst %.4g mm from branch output at (%.17g,%.17g)\n", argv[1], argv[2], jn((JoinType)j), s, c.d, stretch, nm, worst, wp.x, wp.y);
}
