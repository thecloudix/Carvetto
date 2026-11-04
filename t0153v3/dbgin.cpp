#include <cstdio>
#include "gen3.hpp"
#include "acc.hpp"
using namespace carvetto::geometry;
int main() {
    std::vector<Contour> cs{g2::circleAt({0, 0}, 8, 0), vs::rev(g2::circleAt({4.99, 0}, 3, 0))};
    va::Acc A; A.build(cs);
    for (double y : {0.0, -0.0, 1e-17, -1e-17, 1e-9}) { Vec2 p{-4.3205, y}; std::printf("y=%g acc=%d exact=%d\n", y, (int)A.inside(p), (int)vo::insideExact(cs, p)); }
    for (auto& c : cs) { auto& a = std::get<ArcSpan>(c.spans[0]); std::printf("arc start %.17g end %.17g dir %d  p0=(%.17g,%.17g) p1=(%.17g,%.17g)\n", a.startAngle, a.endAngle, (int)a.direction, vo::evalSpan(c.spans[0],0).x, vo::evalSpan(c.spans[0],0).y, vo::evalSpan(c.spans[0],1).x, vo::evalSpan(c.spans[0],1).y); }
}
