// T-0006 round 3: exact 8-line input (strands ~2d apart, opposite), Round Left: an offset span lies
// 9.5e-5 mm beyond d (Round must be exactly d up to the documented allowance).
#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
int main() {
    const std::vector<Vec2> p{{-5281.0715789769911, -5268.2784733563021}, {-5284.9414577865855, -5267.8446579537076}, {-5288.7708788228092, -5290.2632742408732}, {-5286.9348942352071, -5291.4753527022858}, {-5284.2200656054292, -5269.286576947201}, {-5280.3502977692388, -5269.7213822953418}, {-5276.2576290492298, -5245.9702232636519}, {-5281.4940880823642, -5268.2313251412743}, {-5285.3638950837394, -5267.7968691687456}};
    Contour in;
    for (std::size_t i = 1; i < p.size(); ++i) in.spans.emplace_back(LineSpan{p[i - 1], p[i]});
    const double d = 0.67641900916302644;
    auto out = offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, Tolerance::defaults(), 2.0);
    double under = 0, over = 0;
    for (const Contour& c : out)
        for (const Span& s : c.spans)
            for (int k = 0; k <= 4000; ++k) {
                Vec2 q = pointAt(s, k / 4000.0);
                double m = 1e300;
                for (const Span& e : in.spans) m = std::min(m, testing::distanceToSpanExact(q, e));
                under = std::max(under, d - m);
                over = std::max(over, m - d);
            }
    std::printf("chains %zu, worst inside d %.3g, worst beyond d (Round) %.3g mm\n", out.size(), under, over);
}
