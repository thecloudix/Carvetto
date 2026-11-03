#include <cstdio>
#include "carvetto/geometry/box2.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour;
int main() {
  tooling::Tool t; t.id = "e"; t.name = "e"; t.type = tooling::ToolType::EndMill; t.diameter = 6;
  std::vector<Contour> sc{Contour::rectangle(geometry::Box2{{0, 0}, {40, 30}}), Contour::circle({80, 15}, 12)};
  for (double d : {1e-4, 1e-6, 1e-8, 1e-9, 1e-10, 1e-12, 1e-15, 1e-20}) {
    ProfileParams p; p.passes.cutDepth = 3; p.safeZ = 5; p.tolerance = {d, 1e-9, 1e-4};
    auto r = profileToolpath(sc, t, p); std::printf("dist %g ok %d moves %zu diags %zu\n", d, (int)r.ok(), r.toolpath.moves.size(), r.diagnostics.size()); }
}
