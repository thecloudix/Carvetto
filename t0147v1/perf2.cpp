#include <cstdio>
#include <cmath>
#include <ctime>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
int main(){ tooling::Tool t; t.id="em"; t.name="EM"; t.type=tooling::ToolType::EndMill; t.diameter=0.5; t.defaultCuttingData.passDepth=2; t.defaultCuttingData.feedRate=1000; t.defaultCuttingData.plungeRate=300; t.defaultCuttingData.spindleSpeed=1; t.defaultCuttingData.toolNumber=1;
 std::vector<Contour> cs; for(int i=0;i<3000;++i){ Vec2 o{(i%60)*5.0,(i/60)*5.0}; Contour c; c.closed=true; int n=40; for(int k=0;k<n;++k){ double a0=2*kPi*k/n,a1=2*kPi*(k+1)/n; Vec2 p0=o+Vec2{std::cos(a0),std::sin(a0)}*1.5, p1=o+Vec2{std::cos(a1),std::sin(a1)}*1.5; c.append(ArcSpan::fromCentre(o,p0,p1,ArcDirection::CCW)); } cs.push_back(c);} 
 for(int r=0;r<3;++r){ std::clock_t t0=std::clock(); ProfileParams p; p.passes.cutDepth=1; p.passes.passDepth=1; auto res=profileToolpath(cs,t,p); std::printf("moves %zu cpu %.3f s\n",res.toolpath.moves.size(), double(std::clock()-t0)/CLOCKS_PER_SEC);} }
