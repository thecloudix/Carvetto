#include <cstdio>
#include <limits>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/drill.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
int main() {
  tooling::Tool t; t.id="d"; t.name="D"; t.type=tooling::ToolType::Drill; t.diameter=3; t.includedAngle=2; t.defaultCuttingData = {2.5,0,0,0,10000,600,250,0,2};
  double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
  for (double bad : {nan, inf, 1e300, 1e154}) {
    Polyline pl; pl.closed = true; pl.points = {{0,0},{bad,0},{bad,bad},{0,bad}};
    auto r = drillToolpath({Contour::fromPolyline(pl), Contour::circle({bad, 1}, 2), Contour::circle({1, 1}, bad)}, t, std::nullopt, DrillParams{.cutDepth = 3});
    std::printf("bad=%g ok=%d holes=%zu:", bad, r.ok(), r.holes.size()); for (auto& h : r.holes) std::printf(" (%g,%g)", h.centre.x, h.centre.y); for (auto& d : r.diagnostics) std::printf(" [%d %s]", int(d.issue), d.message.c_str()); std::printf("\n");
  }
  // pass count cap boundary
  for (double cut : {9999.0, 10000.0, 10001.0}) { tooling::Tool u = t; u.defaultCuttingData.passDepth = 1; auto r = drillToolpath({Contour::circle({0,0},3)}, u, std::nullopt, DrillParams{.cutDepth = cut, .peck = true});
    std::printf("cut %g pass 1: ok=%d moves=%zu %s\n", cut, r.ok(), r.toolpath.moves.size(), r.diagnostics.empty() ? "" : r.diagnostics[0].message.c_str()); }
  // safeZ just above start, startDepth > 0 with safe Z below surface
  { auto r = drillToolpath({Contour::circle({0,0},3)}, t, std::nullopt, DrillParams{.startDepth = 3, .cutDepth = 3, .safeZ = -2.5, .peck = true, .peckRetract = PeckRetract::AbovePreviousPass, .retractGap = 1});
    std::printf("safeZ below surface: ok=%d:", r.ok()); for (auto& m : r.toolpath.moves) std::printf(" %d@%g", int(m.kind), m.to.z); std::printf("\n"); }
}
