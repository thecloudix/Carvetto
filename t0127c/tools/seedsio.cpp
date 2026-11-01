// seedsio scene mx my : In/Out tab at (mx,my) (a point near the vector): prints D, allowance, part seed, waste seed
// part seed: |min(a,0)| + 1.0 into the part from the nearest vector point; waste seed: 2r + a + lpa + 1.0 into the waste
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto;
int main(int argc, char** argv) {
    std::vector<geometry::Contour> cs; toolpath::ProfileParams p; double D; int vd = 0;
    readScene(argv[1], cs, p, D, vd);
    geometry::Vec2 m{std::atof(argv[2]), std::atof(argv[3])};
    double best = 1e18; geometry::Vec2 t{1, 0}, q = m;
    for (auto& c : cs) { auto cp = geometry::closestPoint(c, m); if (cp.distance < best) { best = cp.distance; q = cp.point; t = geometry::tangentAt(c.spans[cp.spanIndex], cp.t); } }
    geometry::Vec2 n{-t.y, t.x};
    geometry::Tolerance tol;
    std::vector<geometry::Contour> closed; for (auto& c : cs) if (c.closed) closed.push_back(c);
    bool leftInside = geometry::isInside(closed, q + n * 0.01, geometry::FillRule::EvenOdd, tol);
    bool partInside = p.side == toolpath::ProfileSide::Outside;
    geometry::Vec2 toPart = (leftInside == partInside) ? n : n * -1.0;
    double r = D / 2, lpa = p.separateLastPass ? p.lastPassAllowance : 0.0;
    geometry::Vec2 ps = q + toPart * (std::fabs(std::min(p.allowance, 0.0)) + 1.0);
    geometry::Vec2 ws = q - toPart * (2 * r + p.allowance + lpa + 1.0);
    std::printf("%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", D, p.allowance, ps.x, ps.y, ws.x, ws.y, t.x, t.y);
}
