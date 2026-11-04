// Bounded-cost probe for dense sub-tolerance input (T-0009 note, tracked in T-0016).
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstdint>
#include <cstdlib>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double cpu() { return (double)std::clock() / CLOCKS_PER_SEC; }
struct R { std::uint64_t s; double u() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return (double)(s >> 11) * 0x1.0p-53; } };
static void run(const char* nm, const Contour& c, double d) {
    double worst = 0; const char* wj = "";
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
        for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
            double t0 = cpu(); auto o = offsetOpenContour(c, d, s, j); double t = cpu() - t0;
            if (t > worst) { worst = t; wj = j == JoinType::Round ? "Round" : j == JoinType::Sharp ? "Sharp" : "Square"; }
            std::printf("  %s j%d s%d %.2fs chains %zu\n", nm, (int)j, (int)s, t, o.size()); std::fflush(stdout);
        }
    std::printf("%-50s spans %zu d=%g worst %.2fs (%s)\n", nm, c.spans.size(), d, worst, wj); std::fflush(stdout);
}
int main(int argc, char** argv) {
    int which = argc > 1 ? std::atoi(argv[1]) : -1;
    int n = argc > 2 ? std::atoi(argv[2]) : 8000;
    R g{12345};
    if (which == 0 || which < 0) {  // micro cluster: n spans of 1e-7 random walk, then a long line
        Contour c; Vec2 p{0, 0};
        for (int i = 0; i < n; ++i) { double a = g.u() * 6.283; Vec2 q = p + Vec2{std::cos(a), std::sin(a)} * 1e-7; c.spans.push_back(LineSpan{p, q}); p = q; }
        c.spans.push_back(LineSpan{p, p + Vec2{5, 1}});
        run("micro cluster walk 1e-7 + line", c, 1.0);
    }
    if (which == 1 || which < 0) {  // dense random walk 0.5 um steps
        Contour c; Vec2 p{0, 0}; double h = 0;
        for (int i = 0; i < n; ++i) { h += (g.u() - 0.5) * 1.6; Vec2 q = p + Vec2{std::cos(h), std::sin(h)} * 5e-4; c.spans.push_back(LineSpan{p, q}); p = q; }
        run("walk 0.5um steps", c, 1.0);
    }
    if (which == 2 || which < 0) {  // dense circle closed 0.4um
        Contour k; Vec2 p0{0.5, 0}, pv = p0; double r = n * 4e-4 / 6.2832;
        p0 = {r, 0}; pv = p0;
        for (int i = 1; i <= n; ++i) { double a = 6.283185307179586 * i / n; Vec2 q = i == n ? p0 : Vec2{r * std::cos(a), r * std::sin(a)}; k.spans.push_back(LineSpan{pv, q}); pv = q; }
        k.closed = true; run("dense circle 0.4um closed", k, 1.0);
    }
    if (which == 3 || which < 0) {  // zig-zag of tiny lines along a long path (hatch-like fuzz)
        Contour c; Vec2 p{0, 0};
        for (int i = 0; i < n; ++i) { Vec2 q = p + Vec2{2e-4, (i % 2 ? 1 : -1) * 3e-4}; c.spans.push_back(LineSpan{p, q}); p = q; }
        run("tiny zig-zag 2e-4 x 3e-4", c, 1.0);
    }
}
