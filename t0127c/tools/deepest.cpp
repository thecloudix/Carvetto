// deepest scene : deepest point inside the closed vectors (even-odd), its distance to the vectors, and a far-outside point
#include <cmath>
#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto;
int main(int argc, char** argv) {
    std::vector<geometry::Contour> cs; toolpath::ProfileParams p; double D; int vd = 0;
    readScene(argv[1], cs, p, D, vd);
    geometry::Box2 b; for (auto& c : cs) b.expand(c.bounds());
    geometry::Tolerance tol; double best = -1; geometry::Vec2 bp;
    for (double y = b.min.y; y <= b.max.y; y += 0.05) for (double x = b.min.x; x <= b.max.x; x += 0.05) {
        geometry::Vec2 q{x, y}; if (!geometry::isInside(cs, q, geometry::FillRule::EvenOdd, tol)) continue;
        double d = 1e18; for (auto& c : cs) d = std::min(d, geometry::distanceTo(c, q)); if (d > best) { best = d; bp = q; } }
    std::printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", bp.x, bp.y, best, b.min.x - D - 5, b.min.y - D - 5, b.min.x - D - 8, b.min.y - D - 8, b.max.x + D + 8, b.max.y + D + 8);
}
