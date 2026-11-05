// mv4.cpp: print the first cut's moves (XY, kind, distance to the vector start T and to the carrier line of the move) for a scene.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
#include "scene_io.hpp"
int main(int argc, char** argv) {
  std::vector<Contour> cs; ProfileParams p; double D = 6; int vd = 0; if (!readScene(argv[1], cs, p, D, vd)) return 1;
  if (getenv("FLIP")) p.direction = p.direction == CutDirection::Climb ? CutDirection::Conventional : CutDirection::Climb;
  if (getenv("VSOUT")) { p.startAt = StartAt::VectorStart; p.side = ProfileSide::Outside; }
  tooling::Tool t; if (vd) { t.type = tooling::ToolType::VBit; t.includedAngle = vd * PI / 180; } else t.type = tooling::ToolType::EndMill; t.diameter = D;
  auto r = profileToolpath(cs, t, p); Vec2 T = geometry::startPoint(cs[0].spans.front());
  std::printf("d %.17g T %.17g %.17g dir %d sharp %d limit %g\n", profileToolRadius(t, p.passes) + p.allowance, T.x, T.y, (int)p.direction, (int)p.corners.sharpExternal, p.corners.mitreLimit);
  Vec2 prev{}; int n = 0, lim = getenv("N") ? atoi(getenv("N")) : 400;
  for (auto& m : r.toolpath.moves) { Vec2 q = m.to.xy(); double ld = -1; Vec2 dd = q - prev; if (dd.length() > 0) ld = std::fabs((T - prev).cross(dd)) / dd.length();
    if (geometry::distance(q, T) < (getenv("NEAR") ? atof(getenv("NEAR")) : 1e9)) std::printf("%4d k %d %.12g %.12g z %.4g |q-T| %.9g lineDist %.9g\n", n, (int)m.kind, q.x, q.y, m.to.z, geometry::distance(q, T), ld);
    prev = q; if (++n > lim) break; }
}
