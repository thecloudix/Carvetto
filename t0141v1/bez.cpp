#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour;
int main(int argc, char** argv) {
  double dist = std::atof(argv[1]);
  tooling::Tool t; t.id = "e"; t.name = "e"; t.type = tooling::ToolType::EndMill; t.diameter = 6;
  Contour c; c.closed = true;
  c.spans.emplace_back(geometry::BezierSpan{{0, 0}, {10, -5}, {20, 5}, {30, 0}});
  c.spans.emplace_back(geometry::LineSpan{{30, 0}, {30, 20}});
  c.spans.emplace_back(geometry::BezierSpan{{30, 20}, {20, 25}, {10, 15}, {0, 20}});
  c.spans.emplace_back(geometry::LineSpan{{0, 20}, {0, 0}});
  ProfileParams p; p.passes.cutDepth = 3; p.safeZ = 5; p.tolerance = {dist, 1e-9, 1e-4};
  std::clock_t t0 = std::clock(); auto r = profileToolpath({c}, t, p);
  std::printf("dist %g ok %d moves %zu diags %zu cpu %.2f\n", dist, (int)r.ok(), r.toolpath.moves.size(), r.diagnostics.size(), double(std::clock() - t0) / CLOCKS_PER_SEC);
}
