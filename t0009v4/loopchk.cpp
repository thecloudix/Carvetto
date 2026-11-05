// loopchk <g2|v9|adv> <seed> <join> <side>: for every closed chain of main's output, sample its points: distance to the
// branch output, distance to the input - d, and whether the point lies in a corner region (kite) the checker excuses.
#include "v9lib.cpp"
#include "xchk.hpp"
#include "adv.inc"
#include "g2gen.inc"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[2], nullptr, 0); Contour in; double d, mitre, tol;
    if (argv[1][0] == 'g') { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else if (argv[1][0] == 'a') { AdvCase c = makeAdv(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; }
    JoinType join = (JoinType)std::atoi(argv[3]); OffsetSide side = (OffsetSide)std::atoi(argv[4]);
    Contour ck = polyOf(in, tol);
    auto ob = offsetOpenContour(ck, d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre);
    auto mp = mainOffset(toP(ck), d, (int)side, (int)join, tol, mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    xk::Checker XB(ck, ob, d, side, join, tol, mitre); auto cs = XB.corners();
    int bcl = 0; for (auto& c : ob) bcl += c.closed;
    std::printf("branch chains %zu closed %d | main chains %zu\n", ob.size(), bcl, om.size());
    for (size_t ci = 0; ci < om.size(); ++ci) { if (!om[ci].closed) continue; long n = 0, far = 0, farExc = 0, farIn = 0; double worst = 0, per = 0, maxBeyond = 0;
        for (auto& sp : om[ci].spans) { xk::S s = xk::mk(sp); per += s.len; int N = std::max(4, (int)std::ceil(s.len / (tol / 4)));
            for (int q = 0; q < N; ++q) { Vec2 p = xk::P(s, (double)q / N); n++; double b = XB.fOut(p, d + 1); double f = XB.fIn(p, d + 1) - d; maxBeyond = std::max(maxBeyond, f);
                if (b > tol) { far++; worst = std::max(worst, b); if (join != JoinType::Round && XB.excused(cs, p)) farExc++; if (f < tol) farIn++; } } }
        { // interior: grid points inside main's loop (polyline approximation), >= d from the input, outside every excused region
            std::vector<Vec2> poly; for (auto& sp : om[ci].spans) { xk::S s = xk::mk(sp); for (int q = 0; q < 16; ++q) poly.push_back(xk::P(s, q / 16.0)); }
            double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300; for (auto& v : poly) { x0 = std::min(x0, v.x); x1 = std::max(x1, v.x); y0 = std::min(y0, v.y); y1 = std::max(y1, v.y); }
            long inside = 0, valid = 0, free_ = 0; double maxF = 0; Vec2 fp{0, 0};
            for (int a = 0; a <= 120; ++a) for (int b = 0; b <= 120; ++b) { Vec2 p{x0 + (x1 - x0) * a / 120, y0 + (y1 - y0) * b / 120}; bool in2 = false;
                for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) if (((poly[i].y > p.y) != (poly[j].y > p.y)) && (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) in2 = !in2;
                if (!in2) continue; inside++; double f = XB.fIn(p, d + 1) - d; if (f < 0) continue; valid++; if (join != JoinType::Round && XB.excused(cs, p)) continue; free_++; if (f > maxF) { maxF = f; fp = p; } }
            std::printf("  interior grid: inside %ld, beyond d %ld, beyond d and outside every excused region %ld (max dist-d %.3g at %.9f,%.9f; branch output %.4g away)\n", inside, valid, free_, maxF, fp.x, fp.y, free_ ? XB.fOut(fp, d + 1) : -1.0);
        }
        std::printf("main closed chain %zu: %zu spans perimeter %.4g, samples %ld, > tol from branch %ld (worst %.4g), of those inside an excused corner region %ld; max dist-d on loop %.3g\n", ci, om[ci].spans.size(), per, n, far, worst, farExc, maxBeyond);
    }
}
