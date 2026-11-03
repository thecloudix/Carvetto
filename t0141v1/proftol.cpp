#include <cstdio>
#include <limits>
#include "carvetto/geometry/box2.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour;
int main() {
  tooling::Tool t; t.id = "e"; t.name = "e"; t.type = tooling::ToolType::EndMill; t.diameter = 6;
  auto run = [&](const char* nm, std::vector<Contour> cs, ProfileParams p) { auto r = profileToolpath(cs, t, p);
    std::printf("%-28s ok %d moves %zu diags %zu:", nm, (int)r.ok(), r.toolpath.moves.size(), r.diagnostics.size());
    for (auto& d : r.diagnostics) std::printf(" [%d] %s;", (int)d.issue, d.message.c_str()); std::printf("\n"); };
  ProfileParams p; p.passes.cutDepth = 3; p.safeZ = 5; p.tolerance = {1e-5, 1e-9, 1e-5};
  std::vector<Contour> sc{Contour::rectangle(geometry::Box2{{0, 0}, {40, 30}})};
  run("empty input, bad tol", {}, p);
  run("bad tol", sc, p);
  ProfileParams q = p; q.safeZ = std::numeric_limits<double>::quiet_NaN(); run("bad tol + NaN safeZ", sc, q);
  q = p; q.safeZ = -10; run("bad tol + bad safeZ", sc, q);
  q = p; q.tolerance = {0.0001, 1e-9, 0.0001}; q.safeZ = -10; run("good tol + bad safeZ", sc, q);
  Contour open; open.closed = false; open.spans.emplace_back(geometry::LineSpan{{0, 0}, {10, 0}});
  run("open contour, bad tol", {open}, p);
  q = p; q.tolerance = {std::numeric_limits<double>::denorm_min(), 1e-9, 1e-4}; run("denorm distance", sc, q);
  q = p; q.tolerance = {1e-4, 1e-9, std::nextafter(1e-4, 0.0)}; run("chordal nextbelow 1e-4", sc, q);
  q = p; q.tolerance = {1.0, 1e-9, std::nextafter(1.0, 2.0)}; run("chordal nextabove 1", sc, q);
  q = p; q.tolerance = {-0.0, 1e-9, 0.01}; run("distance -0", sc, q);
}
