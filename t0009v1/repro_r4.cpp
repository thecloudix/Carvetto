// T-0006 round 3: exact 6-span input (no sub-tolerance spans), offset comes 1.9e-4 mm inside d.
#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
int main() {
    const std::vector<Vec2> pts{{0.0036197263399135272, -0.29193303409146748}, {0.0049012208081527647, -0.2944254877427343}, {0.006583562492423152, -0.29666697825870963}, {0.0086117770255438111, -0.29860111865782218}, {0.010933978192457157, -0.30017017411134422}, {0.013068744511359252, -0.30198602819125325}, {0.015475029509577408, -0.30342281132840071}};
    Contour in;
    for (std::size_t i = 1; i < pts.size(); ++i) in.spans.emplace_back(LineSpan{pts[i - 1], pts[i]});
    const double d = 0.03169564160714932;
    for (JoinType j : {JoinType::Square, JoinType::Sharp, JoinType::Round}) {
        for (OffsetSide side : {OffsetSide::Right, OffsetSide::Left}) {
            auto out = offsetOpenContour(in, d, side, j, Tolerance::defaults(), 2.1158214539163573);
            double worst = 0;
            for (std::size_t ci = 0; ci < out.size(); ++ci)
                for (std::size_t si = 0; si < out[ci].spans.size(); ++si) {
                    const Span& s = out[ci].spans[si];
                    double w = 0;
                    for (int k = 0; k <= 1000; ++k) {
                        Vec2 p = pointAt(s, k / 1000.0);
                        double m = 1e300;
                        for (const Span& q : in.spans) m = std::min(m, testing::distanceToSpanExact(p, q));
                        w = std::max(w, d - m);
                    }
                    if (w > 1e-6 && j == JoinType::Square && side == OffsetSide::Right)
                        std::printf("   chain %zu span %zu len %.4g: %.4g inside d  (%.15g,%.15g)->(%.15g,%.15g)\n", ci, si, length(s), w, startPoint(s).x, startPoint(s).y, endPoint(s).x, endPoint(s).y);
                    worst = std::max(worst, w);
                }
            std::printf("%s %s: chains %zu, worst inside d %.3g mm\n", j == JoinType::Square ? "Square" : j == JoinType::Sharp ? "Sharp" : "Round", side == OffsetSide::Right ? "Right" : "Left", out.size(), worst);
        }
    }
}
