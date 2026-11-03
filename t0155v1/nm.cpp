#include <cstdio>
#include <cmath>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
int main() {
  std::vector<Contour> sq{Contour::rectangle(geometry::Box2{Vec2{0,0}, Vec2{20,20}})};
  struct C { double depth, stepover, dia; int type; } cs[] = {{0.5,1.01,6,0},{1e-6,5,6,0},{5e-5,5,6,0},{0.0001,5,6,0},{0.001,5,6,0},{2,3,1,1},{2,1234567.25,1,1},{2,999999.5,1,1},{2,1e20,1,1},{1e9,2e9,1e9,1},{3,7.5,6,1},{0.25,10,6,1},{1.0/3,2.54,6,0}};
  for (auto c : cs) { tooling::Tool t; t.id="t"; t.name="T"; t.type = c.type? tooling::ToolType::EndMill : tooling::ToolType::VBit; t.diameter=c.dia; t.includedAngle=M_PI/2;
    auto& cd=t.defaultCuttingData; cd.toolNumber=1; cd.feedRate=1000; cd.plungeRate=300; cd.spindleSpeed=12000; cd.passDepth=1; cd.stepover=0.4;
    EngraveParams E; E.depth=c.depth; E.strategy=EngraveStrategy::Fill; E.stepover=c.stepover; auto r = engraveToolpath(sq, t, std::nullopt, E);
    for (auto& d : r.diagnostics) std::printf("%s\n", d.message.c_str()); }
}
