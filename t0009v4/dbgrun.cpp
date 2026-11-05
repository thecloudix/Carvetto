// Debug: print polygonised input with indices, the checker's (c) runs and collapsed corners, and the output.
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0); C9 cc = make9(seed); Case c = cc.c;
    if (argc > 2) c.join = (JoinType)std::atoi(argv[2]); if (argc > 3) c.side = (OffsetSide)std::atoi(argv[3]);
    Contour ck = polyOf(c.in, c.tol);
    auto out = offsetOpenContour(ck, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    xk::Checker X(ck, out, c.d, c.side, c.join, c.tol, c.mitre);
    std::printf("d=%.17g closed=%d\n", c.d, ck.closed);
    for (size_t i = 0; i < X.in.size(); ++i) { auto& s = X.in[i]; Vec2 t0 = xk::T(s, 0), t1 = xk::T(s, 1);
        std::printf("in %zu k%d (%.10g,%.10g)->(%.10g,%.10g) len %.3g r %.4g sw %.4g dir0 %.4f dir1 %.4f run %d\n", i, s.k, s.a.x, s.a.y, s.b.x, s.b.y, s.len, s.r, s.sw, std::atan2(t0.y, t0.x), std::atan2(t1.y, t1.x), X.runOf_[i]); }
    for (size_t r = 0; r < X.runs_.size(); ++r) { auto& R = X.runs_[r]; Vec2 V; double a1 = 0, sw = 0; bool cc2 = X.collapsedCorner(R, V, a1, sw);
        std::printf("run %zu A %d B %d spans", r, R.A, R.B); for (int j : R.spans) std::printf(" %d", j); std::printf(" collapsed %d a1 %.4f sw %.4f\n", cc2, a1, sw); }
    for (auto& o : out) { std::printf("chain closed %d\n", o.closed); for (auto& sp : o.spans) { auto q = xk::mk(sp); std::printf("  k%d (%.10g,%.10g)->(%.10g,%.10g) c(%.10g,%.10g) r %.6g sw %.6g len %.3g\n", q.k, q.a.x, q.a.y, q.b.x, q.b.y, q.c.x, q.c.y, q.r, q.sw, q.len); } }
}
