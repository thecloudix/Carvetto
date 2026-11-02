// Opposite strands 2d + lat apart (+ tilt over the length): which copies survive near tol/2.
#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
static Contour poly(const std::vector<Vec2>& p) { Contour c; for (size_t i = 1; i < p.size(); ++i) c.spans.emplace_back(LineSpan{p[i-1], p[i]}); return c; }
int main() {
    const double tol = 0.001;
    for (double tx : {0.0, 1e4}) for (JoinType j : {JoinType::Square, JoinType::Round}) for (double d : {0.3, 2.0})
    for (double tiltF : {0.0, 0.45, 0.55, 1.5}) {
        std::printf("tx %g %s d=%g tilt %.2f*tol/2: ", tx, j == JoinType::Square ? "Square" : "Round", d, tiltF);
        for (double f : {0.0, 0.2, 0.45, 0.49, 0.51, 0.55, 0.8, 1.2}) {
            double lat = f * tol, L = 4.0, up = 20 + 4 * d, tilt = tiltF * tol / 2;
            std::vector<Vec2> p{{0, 0}, {L, 0}, {L + 1, up}, {L + 0.5, up + 1}, {L, 2 * d + lat + tilt}, {0, 2 * d + lat}};
            for (auto& v : p) { v.x += tx; v.y += tx; }
            Contour in = poly(p);
            auto out = offsetOpenContour(in, d, OffsetSide::Left, j, Tolerance::defaults(), 2.0);
            // length of output within 2*tol of the line y = d (+tx), x in [0.5, L-0.5]
            double along = 0; size_t back = 0, ov = 0;
            for (auto& c : out) { back += testing::countBackwardSteps(c, Tolerance::defaults()); for (auto& s : c.spans) { double len = length(s); int k = 200, in_ = 0; for (int q = 0; q < k; ++q) { Vec2 pt = pointAt(s, (q + 0.5) / k); if (std::fabs(pt.y - tx - d - lat / 2) < 2 * tol && pt.x - tx > 0.5 && pt.x - tx < L - 0.5) in_++; } along += len * in_ / k; } }
            ov = testing::countOverlaps(out, 1e-6);
            std::printf(" lat %.2ftol: %.2f%s%s |", f, along / (L - 1), back ? " BACK" : "", ov ? " OVL" : "");
        }
        std::printf("\n");
    }
}
