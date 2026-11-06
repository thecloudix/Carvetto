// rect4.cpp: 20 x 10 rectangle started at its corner (0,0), CCW, with one extra exactly collinear vertex on the closing edge at
// distance L before the start corner (L env, default 0.05 mm). V-bit 90 deg sharp external corners, depth 3 (d = 3), Outside,
// VectorStart, Climb and Conventional. Prints the start and its distance from the corner's mitre (-3,-3). DUMP=prefix.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
int main() {
  double L = getenv("L") ? atof(getenv("L")) : 0.05;
  std::vector<Vec2> v{{0, 0}, {20, 0}, {20, 10}, {0, 10}, {0, L}};
  Contour c; c.closed = true; for (size_t i = 0; i < v.size(); ++i) c.spans.push_back(geometry::LineSpan{v[i], v[(i + 1) % v.size()]});
  tooling::Tool t; t.type = tooling::ToolType::VBit; t.diameter = 20; t.includedAngle = PI / 2;
  for (int dir = 0; dir < 2; ++dir) {
    ProfileParams p; p.side = ProfileSide::Outside; p.startAt = StartAt::VectorStart; p.passes.cutDepth = 3; p.passes.passDepth = 3; p.safeZ = 5;
    p.direction = (CutDirection)dir; p.corners.sharpExternal = true; p.corners.mitreLimit = 50;
    auto r = profileToolpath({c}, t, p); Vec2 s = r.toolpath.moves.front().to.xy();
    std::printf("L %g dir %d start (%.9f, %.9f) |start - mitre(-3,-3)| %.6g\n", L, dir, s.x, s.y, geometry::distance(s, Vec2{-3, -3}));
    if (const char* d = getenv("DUMP")) { std::string f = std::string(d) + "_L" + std::to_string(L) + "_dir" + std::to_string(dir) + ".scn"; writeScene(f.c_str(), {c}, p, 20.0, 90); }
  }
}
