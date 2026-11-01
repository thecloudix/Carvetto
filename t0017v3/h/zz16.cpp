#include <cstdio>
#include <ctime>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main() {
    Contour c; Vec2 prev{0, 0};
    for (int i = 1; i <= 1000; ++i) { Vec2 q{i * 0.02, (i % 2) * 1.0}; c.spans.push_back(LineSpan{prev, q}); prev = q; }
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        double t0 = (double)std::clock() / CLOCKS_PER_SEC;
        auto out = offsetOpenContour(c, 7.0, OffsetSide::Left, j, Tolerance::defaults(), 2.0);
        std::printf("join %d: %.2f s CPU, %zu chains\n", (int)j, (double)std::clock() / CLOCKS_PER_SEC - t0, out.size());
    }
}
