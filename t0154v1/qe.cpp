#include <cstdio>
#include <random>
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/geometry/contour.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Box2; using geometry::Vec2;
static tooling::Tool mk(tooling::ToolType ty, double dia, double so) { tooling::Tool t; t.id="x"; t.name="T"; t.type=ty; t.diameter=dia; t.includedAngle = ty==tooling::ToolType::DiamondDrag ? 1.5707963 : 0.0;
  t.defaultCuttingData.toolNumber=2; t.defaultCuttingData.feedRate=1500; t.defaultCuttingData.plungeRate=400; t.defaultCuttingData.spindleSpeed=16000; t.defaultCuttingData.passDepth=0.5; t.defaultCuttingData.stepover=so; return t; }
int main() {
  std::mt19937 rng(154); std::uniform_real_distribution<double> U(0,1);
  int cases=0, diff=0, flagbad=0, okmismatch=0;
  for (int it=0; it<400; ++it) {
    double dia = 0.05 + U(rng)*3.0;
    std::vector<Contour> cs;
    int n = 1 + int(U(rng)*4);
    for (int k=0;k<n;++k) { double x=U(rng)*50, y=U(rng)*50, w=0.2+U(rng)*20, h=0.2+U(rng)*20;
      if (U(rng)<0.4) cs.push_back(Contour::circle({x,y}, 0.1+U(rng)*8)); else cs.push_back(Contour::rectangle(Box2{Vec2{x,y},Vec2{x+w,y+h}})); }
    EngraveParams p; p.depth = 0.01 + U(rng)*1.0; p.passes = 1 + int(U(rng)*3); p.strategy = U(rng)<0.5? EngraveStrategy::Outline: EngraveStrategy::Fill;
    p.fillPattern = U(rng)<0.5? PocketStrategy::Offset : PocketStrategy::Raster; p.rasterAngle = U(rng)*3.14;
    p.stepover = U(rng)<0.3 ? 0.0 : dia * (0.1 + U(rng)*1.1);  // sometimes > dia
    double so = dia*(0.2+U(rng)*0.9);
    auto drag = mk(tooling::ToolType::DiamondDrag, dia, so), mill = mk(tooling::ToolType::EndMill, dia, so);
    auto r = engraveToolpath(cs, drag, std::nullopt, p), m = engraveToolpath(cs, mill, std::nullopt, p);
    ++cases;
    if (r.ok() != m.ok()) { ++okmismatch; printf("ok mismatch it=%d\n", it); }
    if (r.toolpath.moves != m.toolpath.moves) { ++diff; printf("moves differ it=%d\n", it); }
    if (r.diagnostics.size()!=m.diagnostics.size()) printf("diag count differ it=%d\n", it);
    else for (size_t i=0;i<r.diagnostics.size();++i) if (r.diagnostics[i].message != m.diagnostics[i].message) printf("diag msg differ it=%d: %s | %s\n", it, r.diagnostics[i].message.c_str(), m.diagnostics[i].message.c_str());
    if (r.ok() && !(r.toolpath.tool.spindleOff && r.toolpath.tool.spindleSpeed==0.0)) ++flagbad;
    if (m.toolpath.tool.spindleOff) ++flagbad;
    double eff = p.stepover>0? p.stepover : so;
    if (p.strategy==EngraveStrategy::Fill && eff > dia*(1+1e-12) && r.ok()) printf("stepover > dia accepted it=%d\n", it);
  }
  // explicit cap: stepover == dia ok, dia*1.001 refused
  std::vector<Contour> sq{Contour::rectangle(Box2{Vec2{0,0},Vec2{10,10}})};
  EngraveParams p; p.depth=0.1; p.strategy=EngraveStrategy::Fill; p.stepover=0.3;
  auto a = engraveToolpath(sq, mk(tooling::ToolType::DiamondDrag,0.3,0.1), std::nullopt, p); printf("so=dia ok=%d\n", a.ok());
  p.stepover=0.3003; auto b = engraveToolpath(sq, mk(tooling::ToolType::DiamondDrag,0.3,0.1), std::nullopt, p); printf("so>dia ok=%d %s\n", b.ok(), b.diagnostics.empty()?"":b.diagnostics[0].message.c_str());
  // drag result with empty input / zero-size contour
  auto e = engraveToolpath({}, mk(tooling::ToolType::DiamondDrag,0.3,0.1), std::nullopt, p); printf("empty: ok=%d diags=%zu spindleOff=%d\n", e.ok(), e.diagnostics.size(), e.toolpath.tool.spindleOff);
  printf("cases=%d movesdiff=%d okmismatch=%d flagbad=%d\n", cases, diff, okmismatch, flagbad);
}
