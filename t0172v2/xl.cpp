// Pairs spansMeet accepts whose grown boxes would be apart without the line's 1e-7 x len growth (mutant XL), and the
// margin of the real growth on them. Long line, tiny arc tangent near the line's far end.
#include "offset.cpp"
#include <cstdio>
#include <random>
using namespace carvetto::geometry;
static double gapOf(const Box2& a, const Box2& b) { return std::max({0.0, a.min.x - b.max.x, b.min.x - a.max.x, a.min.y - b.max.y, b.min.y - a.max.y}); }
int main() {
    std::mt19937_64 r(5); auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(r); };
    long found = 0, meets = 0; double worstReal = 0;
    for (int t = 0; t < 2000000; ++t) {
        const double L = std::exp(U(std::log(10.0), std::log(1e5)));
        const double rad = std::exp(U(std::log(1e-12), std::log(1e-3)));
        const double th = U(0, 2 * kPi);
        const Vec2 o{U(-1, 1) * 1e3, U(-1, 1) * 1e3};
        const Vec2 dir = Vec2::fromAngle(th, 1.0);
        const Vec2 s1 = o + dir * L;
        const double g = U(0, 1) * 3e-8 * L;
        const Vec2 c = s1 + dir.perp() * (rad + g);
        const double ac = (s1 - c).angle();
        const ArcSpan A{c, rad, ac - 0.5, ac + 0.5, ArcDirection::CCW};
        const Span a = LineSpan{o, s1}, b = A;
        if (!spansMeet(a, b)) continue;
        ++meets;
        const Box2 ba = bounds(a), bb = bounds(b);
        const double need = gapOf(ba, bb);
        const double real = meetReach(a, ba) + meetReach(b, bb);
        const double xl = real - 1e-7 * L;
        worstReal = std::max(worstReal, need / real);
        if (need > xl && ++found <= 3)
            std::printf("XL miss: line (%.17g,%.17g)-(%.17g,%.17g) arc c(%.17g,%.17g) r %.17g a %.17g..%.17g | true gap %.3g need %.3g XLreach %.3g real %.3g\n",
                        o.x, o.y, s1.x, s1.y, c.x, c.y, rad, A.startAngle, A.endAngle, g, need, xl, real);
    }
    std::printf("meets %ld, XL misses %ld, worst need/real %.3g\n", meets, found, worstReal);
}
