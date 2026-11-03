#include <cstdio>
#include <cmath>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static void run(const char* nm, const Contour& c, double d) {
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
        for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
            auto o = offsetOpenContour(c, d, s, j);
            std::size_t n = 0; for (auto& k : o) n += k.spans.size();
            std::printf("%-40s d=%g j%d s%d: chains %zu spans %zu", nm, d, (int)j, (int)s, o.size(), n);
            bool fin = true; for (auto& k : o) for (auto& sp : k.spans) { Vec2 a = startPoint(sp), b = endPoint(sp); if (!std::isfinite(a.x+a.y+b.x+b.y)) fin = false; }
            std::printf(" finite %d\n", fin);
        }
}
int main() {
    Contour e; run("empty", e, 1);
    Contour z; z.spans.push_back(LineSpan{{1,1},{1,1}}); z.spans.push_back(LineSpan{{1,1},{1,1}}); run("all zero-length", z, 1);
    Contour t; t.spans.push_back(LineSpan{{0,0},{1e-12,0}}); run("single 1e-12 line", t, 1);
    Contour t2; t2.spans.push_back(LineSpan{{0,0},{1e-300,0}}); run("single 1e-300 line", t2, 1);
    Contour t3; t3.spans.push_back(LineSpan{{0,0},{5e-324,0}}); run("single denormal line", t3, 1);
    Contour a; a.spans.push_back(ArcSpan{{0,0},1e-14,0,3,ArcDirection::CCW}); run("single tiny arc r1e-14", a, 1);
    Contour fc; fc.spans.push_back(ArcSpan::fullCircle({0,0},1e-5)); fc.closed = true; run("closed full circle r1e-5", fc, 1);
    Contour tri; tri.closed = true; tri.spans.push_back(LineSpan{{0,0},{3e-4,0}}); tri.spans.push_back(LineSpan{{3e-4,0},{0,3e-4}}); tri.spans.push_back(LineSpan{{0,3e-4},{0,0}}); run("closed tiny triangle", tri, 0.5);
    Contour bz; bz.spans.push_back(BezierSpan{{0,0},{0,0},{0,0},{0,0}}); run("degenerate bezier", bz, 1);
    Contour bz2; bz2.spans.push_back(BezierSpan{{0,0},{1e-4,1e-4},{2e-4,-1e-4},{3e-4,0}}); run("tiny bezier", bz2, 1);
    Contour bz3; bz3.spans.push_back(LineSpan{{-1,0},{0,0}}); bz3.spans.push_back(BezierSpan{{0,0},{1e-4,1e-4},{2e-4,-1e-4},{3e-4,0}}); bz3.spans.push_back(LineSpan{{3e-4,0},{1,0}}); run("line+tiny bezier+line", bz3, 0.2);
    auto o0 = offsetOpenContour(bz3, 0.0, OffsetSide::Left, JoinType::Round);
    std::printf("d=0: %zu chains, %zu spans\n", o0.size(), o0.empty() ? 0 : o0[0].spans.size());
    Contour far; far.spans.push_back(LineSpan{{1e5,1e5},{1e5+1,1e5}}); far.spans.push_back(LineSpan{{1e5+1,1e5},{1e5+1,1e5+1e-15}}); far.spans.push_back(LineSpan{{1e5+1,1e5+1e-15},{1e5+2,1e5+1}}); run("far, 1e-15 jog (below ulp)", far, 1);
    Contour nanc; nanc.spans.push_back(LineSpan{{0,0},{NAN,0}}); nanc.spans.push_back(LineSpan{{NAN,0},{1,1}}); run("NaN span", nanc, 1);
}
