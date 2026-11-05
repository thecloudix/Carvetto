// subnear <g2|v9> <seed> x y: distance of (x,y) to the nearest sub-tol span and to the nearest longer span, minus d,
// plus the largest |turn| among the corners touching sub-tol spans within d + 5 tol, and tiny arc sweeps there.
#include "v9lib.cpp"
#include "xchk.hpp"
#include "adv.inc"
#include "g2gen.inc"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[2], nullptr, 0); Contour in; double d, mitre, tol;
    if (argv[1][0] == 'g') { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; }
    Contour ck = polyOf(in, tol); std::vector<Contour> none; xk::Checker X(ck, none, d, OffsetSide::Left, JoinType::Round, tol, mitre);
    Vec2 p{std::atof(argv[3]), std::atof(argv[4])}; double dt = 1e300, dl = 1e300; int n = (int)X.in.size(); double maxTurn = 0, maxSw = 0; int cnt = 0;
    for (int k = 0; k < n; ++k) { double v = xk::dPt(X.in[k], p) - d; bool t = X.in[k].len <= tol; if (t) dt = std::min(dt, v); else dl = std::min(dl, v);
        if (t && v < 5 * tol) { cnt++; if (X.in[k].k == 1) maxSw = std::max(maxSw, std::fabs(X.in[k].sw)); for (int s = -1; s <= 0; ++s) { int a = k + s, b = a + 1; if (a < 0 || b >= n) continue; Vec2 t1 = xk::T(X.in[a], 1), t2 = xk::T(X.in[b], 0); maxTurn = std::max(maxTurn, std::fabs(std::atan2(t1.cross(t2), t1.dot(t2)))); } } }
    std::printf("subnear %s %s: nearest sub-tol span %.3g beyond d, nearest longer span %.3g beyond d; %d sub-tol spans within d+5tol, max corner turn there %.1f deg, max tiny-arc sweep %.1f deg\n", argv[1], argv[2], dt, dl, cnt, maxTurn * 57.29578, maxSw * 57.29578);
}
