#include <cstdio>
#include <cmath>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
int main(){ tooling::Tool t; t.id="em"; t.name="EM"; t.type=tooling::ToolType::EndMill; t.diameter=0.5; t.defaultCuttingData.passDepth=2; t.defaultCuttingData.feedRate=1000; t.defaultCuttingData.plungeRate=300; t.defaultCuttingData.spindleSpeed=1; t.defaultCuttingData.toolNumber=1;
 for(double m: {1e8,3e8}) for(int j=0;j<4;++j){ Vec2 o{m*(1+0.037*j), m*0.7*(1+0.051*j)}; if(j%4==1) o.x=-o.x; if(j%4==2) o.y=-o.y;
  ProfileParams p; p.passes.cutDepth=1; p.passes.passDepth=1; auto r=profileToolpath({Contour::circle(o+Vec2{1,1},1.2)},t,p);
  std::printf("m %g j %d moves %zu", m,j,r.toolpath.moves.size()); for(auto&d:r.diagnostics) std::printf(" | %s", d.message.c_str()); std::printf("\n"); } }
