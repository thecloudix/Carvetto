// closed zig-zag, offset towards the inside by more than its half-thickness: branch vs main, xk metrics
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    int n = atoi(argv[1]); double p = atof(argv[2]), a = atof(argv[3]), d = atof(argv[4]); int side = atoi(argv[5]), join = atoi(argv[6]);
    std::vector<Vec2> pts{{0, 0}}; for (int i = 1; i <= n; ++i) pts.push_back({i * p, (i % 2) * a});
    Vec2 l = pts.back(); pts.push_back({l.x, -1.0}); pts.push_back({0, -1.0});
    Contour in = fromPts(pts, true);
    auto ob = offsetOpenContour(in, d, (OffsetSide)side, (JoinType)join, Tolerance{}, 2.0);
    auto mp = mainOffset(toP(in), d, side, join, 1e-3, 2.0); std::vector<Contour> om; for (auto& q : mp) om.push_back(fromP(q));
    for (int w = 0; w < 2; ++w) { auto& o = w ? om : ob; xk::Res r; xk::Checker(in, o, d, (OffsetSide)side, (JoinType)join, 1e-3, 2.0).run(r);
        size_t ns = 0; for (auto& c : o) ns += c.spans.size();
        std::printf("%s chains %zu spans %zu | ex%ld dv%ld (u %.3g) rv%ld cm%ld tie%ld | %s\n", w ? "main  " : "branch", o.size(), ns, r.excess, r.dv, r.under, r.rv, r.cmiss, r.cmissTie, r.first.c_str());
        if (getenv("OUTS")) for (auto& c : o) { std::printf(" chain closed %d\n", c.closed); for (auto& s : c.spans) { auto q = xk::mk(s); std::printf("   k%d (%.9g,%.9g)->(%.9g,%.9g)\n", q.k, q.a.x, q.a.y, q.b.x, q.b.y); } } }
}
