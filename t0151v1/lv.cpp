#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
  for (double d : {1e-6, 1e-5, 1e-4, 1e-3, 0.01}) for (int n : {2, 10, 100, 1000}) {
    auto lv = engraveLevels(d, n);
    tooling::Tool t; t.id="e"; t.name="e"; t.type=tooling::ToolType::EndMill; t.diameter=1; auto& c=t.defaultCuttingData; c.toolNumber=1; c.feedRate=1000; c.plungeRate=300; c.spindleSpeed=10000; c.passDepth=1; c.stepover=0.4;
    EngraveParams p; p.depth = d; p.passes = n; p.safeZ = 5;
    auto r = engraveToolpath({geometry::Contour::circle({0,0},1)}, t, std::nullopt, p);
    size_t cutLevels = 0; double prev = 1; for (auto& m : r.toolpath.moves) if (isArc(m.kind)) { if (m.to.z != prev) ++cutLevels; prev = m.to.z; }
    size_t pl = r.toolpath.count(MoveKind::Plunge);
    if (lv.size() != (size_t)n || pl != (size_t)n || cutLevels != (size_t)n) std::printf("depth %g passes %d: levels %zu plunges %zu cut-levels %zu arcs %zu\n", d, n, lv.size(), pl, cutLevels, r.toolpath.count(MoveKind::ArcCCW));
  }
  // where in 1e-6/1000
  tooling::Tool t; t.id="e"; t.name="e"; t.type=tooling::ToolType::EndMill; t.diameter=1; auto& c=t.defaultCuttingData; c.toolNumber=1; c.feedRate=1000; c.plungeRate=300; c.spindleSpeed=10000; c.passDepth=1; c.stepover=0.4;
  EngraveParams p; p.depth = 1e-6; p.passes = 1000; p.safeZ = 5;
  auto r = engraveToolpath({geometry::Contour::circle({0,0},1)}, t, std::nullopt, p);
  for (size_t i = 0; i < 6; ++i) std::printf("k%d z %.17g\n", (int)r.toolpath.moves[i].kind, r.toolpath.moves[i].to.z);
}
