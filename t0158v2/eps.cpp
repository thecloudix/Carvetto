// T-0158 verifier: the eps threshold as a knife-edge. Circle (centre c, radius 10), Outside, D 4, Optimise
// from startPosition = c + t*(cos a, sin a) for t around the distance tolerance (1e-3): the start is the
// circle's arc start for t <= eps and the point towards the start position for t > eps.
#include <cmath>
#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main() {
  Vec2 c{37.3, -12.9}; double a = 2.0;
  tooling::Tool tool; tool.type = tooling::ToolType::EndMill; tool.diameter = 4;
  for (double t : {0.0, 1e-9, 0.5e-3, std::nextafter(1e-3, 0.0), 1e-3, std::nextafter(1e-3, 1.0), 1.0000001e-3, 1.001e-3, 2e-3}) {
    ProfileParams p; p.side = ProfileSide::Outside; p.passes.cutDepth = 3; p.passes.passDepth = 3; p.startAt = StartAt::Optimise;
    p.startPosition = {c.x + t * std::cos(a), c.y + t * std::sin(a)};
    auto r = profileToolpath({geometry::Contour::circle(c, 10)}, tool, p);
    Vec2 s = r.toolpath.moves.empty() ? Vec2{0, 0} : r.toolpath.moves.front().to.xy();
    std::printf("t %.17g  start angle %.6f rad (target angle %.1f)\n", t, std::atan2(s.y - c.y, s.x - c.x), a);
  }
}
