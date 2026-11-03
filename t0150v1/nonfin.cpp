#include <cstdio>
#include <limits>
#include <memory>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/document/commands.hpp"
using namespace carvetto; using namespace carvetto::document;
int main() {
  int fails = 0; const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
  struct F { const char* key; double toolpath::DrillParams::* m; } fs[] = {{"start_depth", &toolpath::DrillParams::startDepth}, {"cut_depth", &toolpath::DrillParams::cutDepth}, {"safe_z", &toolpath::DrillParams::safeZ}, {"retract_gap", &toolpath::DrillParams::retractGap}, {"dwell_time", &toolpath::DrillParams::dwellTime}};
  for (double bad : {nan, inf, -inf}) {
    for (auto& f : fs) { toolpath::DrillParams p; p.*(f.m) = bad; auto k = nonFiniteParam(ToolpathParams{p}); if (k != f.key) { ++fails; std::printf("key %s gave '%s'\n", f.key, k.c_str()); }
      Document doc; ToolpathRecord r; r.name = "x"; r.params = p; bool added = doc.execute(std::make_unique<AddToolpathCommand>(r)); if (added) { ++fails; std::printf("AddToolpath accepted %s\n", f.key); } }
    toolpath::DrillParams p; p.startPosition.x = bad; std::printf("sp.x -> %s\n", nonFiniteParam(ToolpathParams{p}).c_str());
    p = {}; p.startPosition.y = bad; std::printf("sp.y -> %s\n", nonFiniteParam(ToolpathParams{p}).c_str());
    p = {}; p.tolerance.distance = bad; std::printf("tol.d -> %s\n", nonFiniteParam(ToolpathParams{p}).c_str());
    p = {}; p.tolerance.angle = bad; std::printf("tol.a -> %s\n", nonFiniteParam(ToolpathParams{p}).c_str());
    p = {}; p.tolerance.chordal = bad; std::printf("tol.c -> %s\n", nonFiniteParam(ToolpathParams{p}).c_str());
  }
  toolpath::DrillParams ok; if (!nonFiniteParam(ToolpathParams{ok}).empty()) { ++fails; std::printf("default flagged\n"); }
  std::printf("fails=%d\n", fails); return fails;
}
