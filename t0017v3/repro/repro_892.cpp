// T-0017 round-3 verifier: fam5 coCircGap 0xf5000892. Continuous exact input (no input gaps).
// The closing junction of a closed chain sits at the wrong one of two crossings of nearly
// tangent circles (overlap 3.52e-9 mm, crossings 6.24e-5 mm apart), so the loop crosses itself
// 6.24e-5 mm from its junction and dips 3.52e-9 mm inside d. Main: junction at the other crossing,
// no self-crossing, under 1.2e-12. Translation-dependent (clean at +1e4,0).
#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main() {
    for (Vec2 sh : {Vec2{0, 0}, Vec2{2e4, 1e4}, Vec2{-0.5, 0.25}, Vec2{1e4, 0}}) {
        auto T = [&](Vec2 p) { return Vec2{p.x + sh.x, p.y + sh.y}; };
        Contour in;
        in.spans.push_back(ArcSpan{T({-10000, -1594.7850028979628}), 1.8569824617641819, 6.4262107396587735, 5.9068986469164431, ArcDirection::CW});
        std::vector<Vec2> P = {{-9998.2729402326077, -1595.4673872641051}, {-9977.4259129997026, -1603.7043259809141},
                               {-9983.0260522973149, -1612.1002727819391}, {-9998.7726747695579, -1595.7428089432765}};
        P[0] = endPoint(in.spans[0]) - sh;  // keep the input exactly continuous
        for (std::size_t i = 1; i < P.size(); ++i) in.spans.push_back(LineSpan{i == 1 ? endPoint(in.spans[0]) : T(P[i - 1]), T(P[i])});
        ArcSpan last{T({-10000, -1594.7850028979628}), 1.5568299976939231, 5.6205096403315613, 6.3121596973040859, ArcDirection::CCW};
        std::get<LineSpan>(in.spans.back()).end = startPoint(Span{last});
        in.spans.push_back(last);
        auto out = offsetOpenContour(in, 0.15007623379778232, OffsetSide::Right, JoinType::Round, Tolerance::defaults(), 2.0);
        int selfX = 0;
        for (auto& c : out) {
            if (!c.closed) continue;
            const std::size_t n = c.spans.size();
            auto r = intersect(c.spans[0], c.spans[n - 1], Tolerance{1e-12, 1e-9, 0.01});
            for (auto& h : r.points) if (h.tA > 1e-9 || h.tB < 1 - 1e-9) { selfX++; std::printf("  self-crossing of closed chain at (%.12f, %.12f), %.3g mm from its junction\n", h.point.x, h.point.y, distance(h.point, startPoint(c.spans[0]))); }
        }
        std::printf("shift (%g,%g): %zu chains, self-crossings %d\n", sh.x, sh.y, out.size(), selfX);
    }
}
