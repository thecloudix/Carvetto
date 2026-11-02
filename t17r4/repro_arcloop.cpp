// Verifier repro (throwaway): continuous closed 3-arc input at ~1e4 mm, Round, Left.
#include <cstdio>
#include <variant>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main() {
    Contour in;
    in.closed = true;
    in.spans.emplace_back(ArcSpan{{9983.0018315241668, -7985.8356249752496}, 20.899133891249114, -0.27495478339630758, -1.0921978125326512, ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{9998.6116614581879, -8003.1964218735147}, 6.1022177852037727, -2.9453011998537355, -0.96720173187117575, ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{9968.5842862924146, -7997.747088651955}, 35.090437701744214, -0.30308470313606828, 0.17869817934407367, ArcDirection::CCW});
    for (std::size_t i = 0; i < 3; ++i) { Vec2 e = endPoint(in.spans[i]), s = startPoint(in.spans[(i + 1) % 3]); std::printf("input junction %zu identical=%d\n", i, (int)(e == s)); }
    auto out = offsetOpenContour(in, 0.072003004263852685, OffsetSide::Left, JoinType::Round, Tolerance{}, 2.0);
    for (const Contour& c : out) {
        std::size_t n = c.spans.size();
        for (std::size_t i = 0; i < (c.closed ? n : n - 1); ++i) {
            const Span& a = c.spans[i]; const Span& b = c.spans[(i + 1) % n];
            double dot = (startPoint(b) - endPoint(a)).dot(tangentAt(a, 1.0));
            std::printf("chain closed=%d n=%zu junction %zu: along-travel %.3g mm gap %.3g mm\n", c.closed, n, i, dot, distance(startPoint(b), endPoint(a)));
        }
    }
}
