// far-origin behaviour: small parts at 0 vs 1e6..1e8 (translated); compare move count and shape
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <string>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "pocket_test_util.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static tooling::Tool em(double d, tooling::ToolType ty=tooling::ToolType::EndMill){ tooling::Tool t; t.id="em"; t.name="EM"; t.type=ty; t.diameter=d; if(ty==tooling::ToolType::Drill) t.includedAngle=118.0*kPi/180; t.defaultCuttingData.passDepth=2; t.defaultCuttingData.feedRate=1000; t.defaultCuttingData.plungeRate=300; t.defaultCuttingData.spindleSpeed=18000; t.defaultCuttingData.toolNumber=1; return t; }
static std::vector<Contour> shape(int k, Vec2 o){
  std::vector<Contour> v;
  auto P=[&](std::vector<Vec2> p){ for(auto& q:p) q=q+o; return Contour::fromPolyline(Polyline{p,true}); };
  switch(k){
    case 0: v.push_back(P({{0,0},{3,0},{3,1.5},{0,1.5}})); break;
    case 1: v.push_back(P({{0,0},{1,0},{1,0.5},{0,0.5}})); break;
    case 2: v.push_back(Contour::circle(o+Vec2{1,1},1.2)); break;
    case 3: { Contour c; c.closed=true; Vec2 a=o, b=o+Vec2{4,0}; double w=0.8;
      c.append(LineSpan{a+Vec2{0,-w},b+Vec2{0,-w}}); c.append(ArcSpan::fromCentre(b,b+Vec2{0,-w},b+Vec2{0,w},ArcDirection::CCW));
      c.append(LineSpan{b+Vec2{0,w},a+Vec2{0,w}}); c.append(ArcSpan::fromCentre(a,a+Vec2{0,w},a+Vec2{0,-w},ArcDirection::CCW)); v.push_back(c); break; }
    case 4: { std::vector<Vec2> p; for(int i=0;i<10;++i){ double r=i%2?0.8:2.0; double a=kPi*i/5; p.push_back({r*std::cos(a),r*std::sin(a)});} v.push_back(P(p)); break; }
    case 5: v.push_back(P({{0,0},{6,0},{6,4},{0,4}})); v.push_back(P({{2,1.5},{2,2.5},{4,2.5},{4,1.5}})); break; // hole CW
    case 6: { std::vector<Vec2> p; for(int i=0;i<6;++i){ double a=kPi*i/3; p.push_back({0.75*std::cos(a),0.75*std::sin(a)});} v.push_back(P(p)); std::vector<Vec2> q=p; for(auto& x:q) x=x+Vec2{5,0}; v.push_back(P(q)); break; }
  }
  return v;
}
struct R{ size_t n; bool empty; double dev; };
static R cmp(const Toolpath& a, const Toolpath& b, Vec2 o){ R r{b.moves.size(), b.moves.empty(), 0};
  if(a.moves.size()!=b.moves.size()) { r.dev=1e300; return r; }
  for(size_t i=0;i<a.moves.size();++i){ r.dev=std::max({r.dev, std::fabs(b.moves[i].to.x-o.x-a.moves[i].to.x), std::fabs(b.moves[i].to.y-o.y-a.moves[i].to.y), std::fabs(b.moves[i].to.z-a.moves[i].to.z)}); }
  return r; }
int main(){
  const double mags[]={1e6,1e7,1e8,3e8};
  for(int op=0; op<4; ++op) for(int k=0;k<7;++k){
    auto run=[&](Vec2 o)->Toolpath{ auto cs=shape(k,o);
      if(op==0){ PocketParams p; p.cutDepth=1; p.safeZ=5; auto r=pocketToolpath(cs,{carvetto::test::pocket::endMill("EM",1,0.5,0.2,1.0)},p); return r.toolpaths.empty()?Toolpath{}:r.toolpaths.front(); }
      if(op==1||op==2){ ProfileParams p; p.side= op==1?ProfileSide::Outside:ProfileSide::Inside; p.passes.cutDepth=1; p.passes.passDepth=1; auto r=profileToolpath(cs,em(0.5),p); return r.toolpath; }
      DrillParams p; p.cutDepth=2; auto r=drillToolpath(cs,em(0.5,tooling::ToolType::Drill),std::nullopt,p); return r.toolpath; };
    Toolpath ref=run({0,0});
    for(double m: mags){ int same=0, empty=0, cnt=0; double worst=0;
      for(int j=0;j<20;++j){ Vec2 o{m*(1+0.037*j), m*0.7*(1+0.051*j)}; if(j%4==1) o.x=-o.x; if(j%4==2) o.y=-o.y;
        R r=cmp(ref, run(o), o); ++cnt; if(r.empty) ++empty; if(r.n==ref.moves.size()) ++same; if(r.dev<1e300) worst=std::max(worst,r.dev); }
      std::printf("op %d shape %d mag %g refmoves %zu sameCount %d/%d empty %d maxdev %.3g\n", op,k,m,ref.moves.size(),same,cnt,empty,worst); }
  }
}
