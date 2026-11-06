// k-d tree worst-case probe: point/box grid with slab queries between columns (output 0).
#include "box_tree.hpp"
#include <cstdio>
#include <cmath>
#include <cstdlib>
using namespace carvetto::geometry;
int main(int, char** argv) {
    for (size_t m = 100; m <= (size_t)atol(argv[1]); m *= 2) {
        std::vector<Box2> v;
        for (size_t i = 0; i < m; ++i) for (size_t j = 0; j < m; ++j) {
            double x = (double)i, y = (double)j, s = atof(argv[2]);
            v.push_back(Box2{Vec2{x, y}, Vec2{x + s, y + s}});
        }
        detail::BoxTree t(v);
        size_t worst = 0;
        for (size_t c = 0; c + 1 < m; c += m / 16 + 1) {
            for (int dir = 0; dir < 2; ++dir) {
                double a = (double)c + 0.6;
                Box2 Q = dir ? Box2{Vec2{a, -1}, Vec2{a + 0.1, (double)m}} : Box2{Vec2{-1, a}, Vec2{(double)m, a + 0.1}};
                size_t b = t.work(), k = 0; t.query(Q, [&](size_t) { ++k; });
                if (k) printf("nonzero %zu\n", k);
                worst = std::max(worst, t.work() - b);
            }
        }
        printf("grid n %zu worst empty-slab work %zu  sqrt(n) %.0f  n^.75 %.0f\n", m * m, worst, (double)m, std::pow((double)(m * m), .75));
    }
}
