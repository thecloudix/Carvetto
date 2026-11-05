// T-0009 round-2 verifier repro driver: branch (or merged) vs main on one input, judged by xchk.hpp.
//  case_f0003fad.inc (v9 seed 0xf0003fad, polygonised): Square, Left, d 3.753745644092505, mitre 2.0341221181052518:
//    branch loses >= 1.6 mm of the inner offset of an arc that is split by a sub-tol arc piece (main covers it).
//  case_e0000119.inc (v9 seed 0xe0000119): Round, Right, d 0.91252773458987702: closed loop -> one OPEN chain,
//    0.67 mm gap at a micro-cluster with a near-reversal; the stretch is emitted on the Left offset instead
//    (side-membership rule of ADR-0012; non-blocking, product question).
// Generic driver: -DCASE=<input .inc> -DXD=<d> -DXSIDE=<side> -DXJOIN=<join> -DXMITRE=<mitre>. Cases: case_e0000119.inc (Round, Right, d 0.91252773458987702),
// case_f0003fad.inc (Square, Left, d 3.753745644092505, mitre 2.03...).
#include "xchk.hpp"
#include "pw.hpp"
#include <cstdio>
using namespace carvetto::geometry;
PContour toP2(const Contour& c) { PContour p; p.closed = c.closed; for (auto& s : c.spans) { PSpan q{}; if (auto* l = std::get_if<LineSpan>(&s)) { q.kind = 0; q.a[0] = l->start.x; q.a[1] = l->start.y; q.a[2] = l->end.x; q.a[3] = l->end.y; } else { auto& a = std::get<ArcSpan>(s); q.kind = 1; q.a[0] = a.centre.x; q.a[1] = a.centre.y; q.a[2] = a.radius; q.a[3] = a.startAngle; q.a[4] = a.endAngle; q.a[5] = a.direction == ArcDirection::CCW ? 0 : 1; } p.s.push_back(q); } return p; }
Contour fromP2(const PContour& p) { Contour c; c.closed = p.closed; for (auto& q : p.s) { if (q.kind == 0) c.spans.push_back(LineSpan{{q.a[0], q.a[1]}, {q.a[2], q.a[3]}}); else c.spans.push_back(ArcSpan{{q.a[0], q.a[1]}, q.a[2], q.a[3], q.a[4], q.a[5] == 0 ? ArcDirection::CCW : ArcDirection::CW}); } return c; }
static double g_mitre = 2.0;
static void report(const char* who, const Contour& in, const std::vector<Contour>& out, double d, OffsetSide side, JoinType j) {
    std::printf("%s: %zu chain(s)", who, out.size());
    for (auto& c : out) { double gap = c.closed ? 0 : distance(c.endPoint(), c.startPoint()); std::printf(" [closed %d, %zu spans, end-to-start gap %.4g]", c.closed, c.spans.size(), gap); }
    xk::Res r; xk::Checker(in, out, d, side, j, 1e-3, g_mitre).run(r);
    std::printf("\n  xk: dist viol %ld (under %.3g) round viol %ld (over %.3g) coverage misses %ld core + %ld tie (non-thin; longest stretch %.3g) %ld thin | %s\n", r.dv, r.under, r.rv, r.over, r.cmiss, r.cmissTie, std::max(r.missLen, r.missTieLen), r.cmissThin, r.first.c_str());
}
int main(int argc, char** argv) {
#include CASE
    double d = XD; OffsetSide side = XSIDE; JoinType j = XJOIN;
    if (argc > 1) j = (JoinType)std::atoi(argv[1]);
    double mitre = XMITRE; g_mitre = mitre; auto ob = offsetOpenContour(in, d, side, j, Tolerance{}, mitre);
    report("branch", in, ob, d, side, j);
    auto mp = mainOffset(toP2(in), d, side == OffsetSide::Left ? 0 : 1, (int)j, 1e-3, mitre);
    std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP2(p));
    report("main  ", in, om, d, side, j);
}
