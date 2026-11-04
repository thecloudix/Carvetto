#include <cstdio>
#include <ctime>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    int n = std::atoi(argv[1]);
    Contour c; Vec2 p{0, 0};
    for (int i = 0; i < n; ++i) { Vec2 q = p + Vec2{2e-4, (i % 2 ? 1 : -1) * 3e-4}; c.spans.push_back(LineSpan{p, q}); p = q; }
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
    for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
        double t0 = (double)std::clock() / CLOCKS_PER_SEC;
        auto o = offsetOpenContour(c, 1.0, s, j);
        double t = (double)std::clock() / CLOCKS_PER_SEC - t0;
        double len = 0; for (auto& k : o) for (auto& sp : k.spans) len += length(sp);
        std::printf("n=%d j%d s%d: %.2fs chains %zu total length %.4f (input x-extent %.4f)\n", n, (int)j, (int)s, t, o.size(), len, n * 2e-4);
    }
}
