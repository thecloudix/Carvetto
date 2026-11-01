// tangent scene mx my -> prints D allowance and a unit tangent of the nearest vector at the nearest point
#include <cstdio>
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto;
int main(int argc, char** argv) {
    std::vector<geometry::Contour> cs; toolpath::ProfileParams p; double D; int vd = 0;
    readScene(argv[1], cs, p, D, vd);
    geometry::Vec2 m{std::atof(argv[2]), std::atof(argv[3])};
    double best = 1e18; geometry::Vec2 t{1, 0};
    for (auto& c : cs) { auto cp = geometry::closestPoint(c, m); if (cp.distance < best) { best = cp.distance; t = geometry::tangentAt(c.spans[cp.spanIndex], cp.t); } }
    std::printf("%.17g %.17g %.17g %.17g\n", D, p.allowance, t.x, t.y);
}
