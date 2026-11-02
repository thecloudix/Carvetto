// T-0006 round 3: exact input, line + two nearly co-circular arcs; Sharp Right offset 2.4e-6 mm inside d.
#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
int main() {
    Contour in;
    in.spans.emplace_back(LineSpan{{-0.54490984941556175, 0.47713618213141934}, {-0.65087324196032126, 0.74692846369896915}});
    in.spans.emplace_back(ArcSpan{{-0.66128590342338933, 0.74283879542663755}, 0.011186997153936269, 0.37424873933798614, 3.7359281215955029, ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{-0.66130088294241851, 0.74282867147309783}, 0.011168917319157794, -2.5472571855840811, -1.6946626886190583, ArcDirection::CCW});
    const double d = 0.1712170592382718;
    auto out = offsetOpenContour(in, d, OffsetSide::Right, JoinType::Sharp, Tolerance::defaults(), 2.3199483387667215);
    double worst = 0;
    for (const Contour& c : out)
        for (const Span& s : c.spans)
            for (int k = 0; k <= 20000; ++k) {
                Vec2 p = pointAt(s, k / 20000.0);
                double m = 1e300;
                for (const Span& q : in.spans) m = std::min(m, testing::distanceToSpanExact(p, q));
                worst = std::max(worst, d - m);
            }
    std::printf("chains %zu, worst inside d %.3g mm (documented bound for snapped extensions %.3g)\n", out.size(), worst, std::min(5e-7 * std::max(1.0, d), 0.1 * 1e-6 / d));
}
