// Emits predicate cases as hex floats with the implementation's sign; checked exactly in Python.
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <random>
#include "robust_predicates_old.hpp"
using carvetto::geometry::Vec2;
using namespace carvetto::geometry::detail;
static std::mt19937_64 rng(12345);
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double nudge(double x, int k) { for (int i = 0; i < std::abs(k); ++i) x = std::nextafter(x, k > 0 ? INFINITY : -INFINITY); return x; }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static void O(Vec2 a, Vec2 b, Vec2 c) { std::printf("O %a %a %a %a %a %a %d\n", a.x, a.y, b.x, b.y, c.x, c.y, orient2dSign(a, b, c)); }
static void C(Vec2 a, Vec2 b, Vec2 c, Vec2 d) { std::printf("C %a %a %a %a %a %a %a %a %d\n", a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y, incircleSign(a, b, c, d)); }
int main(int argc, char** argv) {
    int n = argc > 1 ? std::atoi(argv[1]) : 20000;
    for (int i = 0; i < n; ++i) {
        int kind = i % 10;
        double s = std::pow(10.0, I(-6, 7)); Vec2 off{U(-1, 1) * std::pow(10.0, I(0, 8)), U(-1, 1) * std::pow(10.0, I(0, 8))};
        if (kind == 0) { O({U(-1,1),U(-1,1)}, {U(-1,1),U(-1,1)}, {U(-1,1),U(-1,1)}); C({U(-1,1),U(-1,1)}, {U(-1,1),U(-1,1)}, {U(-1,1),U(-1,1)}, {U(-1,1),U(-1,1)}); }
        else if (kind == 1 || kind == 2) { // near-collinear: c on line ab, nudged by ulps
            Vec2 a{off.x + U(-1,1)*s, off.y + U(-1,1)*s}, b{off.x + U(-1,1)*s, off.y + U(-1,1)*s}; double t = U(-2, 3);
            Vec2 c{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)}; c.x = nudge(c.x, I(-3,3)); c.y = nudge(c.y, I(-3,3)); O(a, b, c); O(b, c, a); O(c, a, b);
        } else if (kind == 3) { // Shewchuk-style grid
            Vec2 a{12, 12}, b{24, 24}; Vec2 c{nudge(0.5, I(0, 256)), nudge(0.5, I(0, 256))}; O(a, b, c);
            Vec2 a2{nudge(0.5, I(0,64)), nudge(0.5, I(0,64))}; O(a2, b, a);
        } else if (kind >= 4 && kind <= 7) { // near-cocircular: pythagorean points on a circle, translated, ulp-nudged
            static const int tr[][3] = {{3,4,5},{5,12,13},{8,15,17},{7,24,25},{20,21,29},{12,35,37},{9,40,41},{28,45,53}};
            auto pt = [&](void) { auto& t = tr[I(0,7)]; int sx = I(0,1)?1:-1, sy = I(0,1)?1:-1; bool sw = I(0,1); double r = 1.0 / t[2];
                double x = (sw ? t[0] : t[1]) * r * sx, y = (sw ? t[1] : t[0]) * r * sy; return Vec2{x, y}; };
            // exact co-circular points with power-of-two scale (exact), plus offset
            double sc = std::ldexp(1.0, I(-20, 25)); Vec2 o{std::ldexp(std::round(off.x), I(-10,10)), std::ldexp(std::round(off.y), I(-10,10))};
            auto P = [&](Vec2 p){ Vec2 q{o.x + p.x * sc, o.y + p.y * sc}; if (kind >= 6) { q.x = nudge(q.x, I(-2,2)); q.y = nudge(q.y, I(-2,2)); } return q; };
            C(P(pt()), P(pt()), P(pt()), P(pt()));
            // axis-aligned exact circle points (always cocircular): (±r,0),(0,±r)
            double r = std::ldexp(1.0, I(-30, 30)) * (1 + I(0, 1000)); Vec2 cc{nudge(o.x, I(-5,5)), o.y};
            C({cc.x + r, cc.y}, {cc.x, cc.y + r}, {cc.x - r, cc.y}, {nudge(cc.x, I(-1,1)), nudge(cc.y - r, I(-1,1))});
        } else if (kind == 8) { // huge / tiny magnitudes and mixtures
            double m = std::pow(10.0, I(-60, 60)); auto R = [&]{ return Vec2{U(-1,1)*m, U(-1,1)*m}; };
            Vec2 a = R(), b = R(); Vec2 c{a.x + 0.5 * (b.x - a.x), a.y + 0.5 * (b.y - a.y)}; O(a, b, c); O(a, b, R());
            C(R(), R(), R(), R());
        } else { // mixed magnitudes incl. points that differ in the last ulp (sliver triangles)
            Vec2 a{off.x, off.y}; Vec2 b{nudge(off.x, I(1,4)), nudge(off.y, I(-4,4))}; Vec2 c{nudge(off.x, I(-4,4)), nudge(off.y, I(1,4))}; Vec2 d{nudge(off.x, I(-4,4)), nudge(off.y, I(-4,4))};
            O(a, b, c); C(a, b, c, d); C(a, {a.x + s, a.y}, {a.x, a.y + s}, {nudge(a.x + s, I(-2,2)), nudge(a.y + s, I(-2,2))});
        }
    }
    // claimed-range boundary probes
    for (double m : {1e-140, 1e-100, 1e-80, 1e-70, 1e60, 1e70, 1e75}) {
        C({m, 0}, {0, m}, {-m, 0}, {0, -m}); C({m, 0}, {0, m}, {-m, 0}, {0, -m * 0.999}); C({m, 0}, {0, m}, {-m, 0}, {0, nudge(-m, 1)});
        O({0, 0}, {m, m}, {nudge(2 * m, 1), 2 * m}); O({0, 0}, {m, m}, {2*m, 2*m});
    }
}
