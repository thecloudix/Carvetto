// BoxTree query work on a hatch with random x-extents (rhatch-like boxes) vs uniform-x hatch: small square queries.
#include "box_tree.hpp"
#include <cstdio>
#include <cmath>
#include <random>
using namespace carvetto::geometry;
int main() {
    std::mt19937_64 r(1); auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(r); };
    for (int randx = 0; randx < 2; ++randx)
    for (size_t n = 1000; n <= 256000; n *= 4) {
        std::vector<Box2> v;
        for (size_t i = 0; i < n; ++i) { double y = 0.1 * (double)i, a = randx ? U(0, 4) : 0, b = randx ? U(6, 10) : 10; v.push_back(Box2{Vec2{a, y}, Vec2{b, y}}); }
        detail::BoxTree t(v); size_t rep = 0, q = 20000;
        size_t w0 = t.work();
        for (size_t k = 0; k < q; ++k) { Vec2 p{U(0, 10), U(0, 0.1 * (double)n)}; double c = 0.0165; t.query(Box2{p - Vec2{c, c}, p + Vec2{c, c}}, [&](size_t) { ++rep; }); }
        printf("%s n %7zu work/query %8.1f reported/query %.2f  n^.5 %.0f n^.75 %.0f\n", randx ? "random x" : "uniform x", n, double(t.work() - w0) / q, double(rep) / q, std::sqrt((double)n), std::pow((double)n, .75));
    }
}
