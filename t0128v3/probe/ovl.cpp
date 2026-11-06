// Does the tests' noise filter hide an overlap of two adjacent arcs on nearly the same circle (b runs on past a's start)?
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>
#include <variant>
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/intersect.hpp"
using namespace carvetto::geometry;
struct Rng { unsigned long long s; double uni() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (s >> 11) * (1.0 / 9007199254740992.0); } double range(double a, double b) { return a + (b - a) * uni(); } };
#include "sxcount.inc"
int main() {
    for (double off : {1e-15, 1e-13, 1e-12, 1e-11, 1e-10, 1e-9, 1e-8, 1e-7, 1e-5}) for (double sc : {1.0, 1e4}) {
        Vec2 c0{sc, sc};
        ArcSpan a{c0, 1.0, 0.0, 4.0, ArcDirection::CCW};
        Vec2 c1 = c0 + Vec2{std::cos(2.2) * off, std::sin(2.2) * off};
        // b starts where a ends (on its own circle about c1, through a's end), runs 3 rad on: past a's start
        Vec2 e = a.endPoint(); double r1 = distance(e, c1); double s = std::atan2(e.y - c1.y, e.x - c1.x);
        ArcSpan b{c1, r1, s, s + 3.0, ArcDirection::CCW};
        Contour k; k.closed = false; k.spans = {a, b};
        SX r; sxCount({k}, r, true);
        std::printf("off %.0e scale %.0e: sxNew %ld sxOld %ld maxNoise %.3g\n", off, sc, r.sxNew, r.sxOld, r.maxNoise);
    }
}
