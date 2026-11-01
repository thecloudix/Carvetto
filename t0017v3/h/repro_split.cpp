// T-0017 verifier: closed rotated zig-zag (exact up to rotation rounding), d = 88.59, Square, Right.
// Main: 2 chains (one continuous chain along the bottom); branch: 4 chains, the bottom line cut into 3 chains.
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main() {
    const std::vector<Vec2> p{
#include "pts124.txt"
    };
    Contour in; in.closed = true;
    for (std::size_t i = 0; i < p.size(); ++i) in.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]});
    auto out = offsetOpenContour(in, 88.588646191724834, OffsetSide::Right, JoinType::Square, Tolerance::defaults(), 2.0);
    std::printf("%zu chains\n", out.size());
    for (auto& c : out) { std::printf("chain closed=%d:", c.closed); for (auto& s : c.spans) std::printf(" (%.17g,%.17g)->(%.17g,%.17g)", startPoint(s).x, startPoint(s).y, endPoint(s).x, endPoint(s).y); std::printf("\n"); }
}
