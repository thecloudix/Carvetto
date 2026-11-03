#include <cstdio>
#include <cmath>
#include <random>
#include <string>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/drill.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
int main(int argc,char**argv){ std::mt19937_64 g(std::strtoull(argv[1],0,10)); int N=atoi(argv[2]); auto U=[&](double a,double b){return std::uniform_real_distribution<double>(a,b)(g);};
 for(int sc=0;sc<N;++sc){ std::vector<Contour> cs; int n=1+int(U(0,40)); double far = U(0,1)<0.3? std::pow(10,U(3,8)):0;
  for(int i=0;i<n;++i){ Vec2 o{far+U(-100,100), far*0.5+U(-100,100)}; int k=int(U(0,4)); double r=U(0.2,6);
   if(k==0) cs.push_back(Contour::circle(o,r, U(0,1)<0.5?ArcDirection::CCW:ArcDirection::CW));
   else if(k==1){ std::vector<Vec2> p; int m=3+int(U(0,8)); for(int j=0;j<m;++j){double a=2*kPi*j/m; p.push_back(o+Vec2{r*std::cos(a),r*std::sin(a)});} if(U(0,1)<0.5) std::reverse(p.begin(),p.end()); cs.push_back(Contour::fromPolyline(Polyline{p,true})); }
   else if(k==2){ cs.push_back(Contour::circle(o,r)); cs.push_back(Contour::circle(o+Vec2{U(-0.1,0.1),0},r*0.5)); }
   else { std::vector<Vec2> p{o,o+Vec2{r,0},o+Vec2{r,r*0.2},o+Vec2{r*0.2,r*0.2},o+Vec2{r*0.2,r},o+Vec2{0,r}}; cs.push_back(Contour::fromPolyline(Polyline{p,true})); } }
  tooling::Tool t; t.id="d"; t.name="D"; t.type= U(0,1)<0.5?tooling::ToolType::Drill:tooling::ToolType::EndMill; t.diameter=U(0.5,8); if(t.type==tooling::ToolType::Drill) t.includedAngle=118*kPi/180; t.defaultCuttingData.passDepth=U(0.3,4); t.defaultCuttingData.feedRate=500; t.defaultCuttingData.plungeRate=300; t.defaultCuttingData.spindleSpeed=1000; t.defaultCuttingData.toolNumber=1;
  DrillParams p; p.cutDepth=U(0.5,10); p.peck=U(0,1)<0.5; int o=int(U(0,5)); p.order.selectionOrder=o==1; p.order.leftToRight=o==2; p.order.bottomToTop=o==3; p.order.grid=o==4;
  auto r=drillToolpath(cs,t,std::nullopt,p); unsigned long long h=1469598103934665603ULL; char b[256];
  auto add=[&](const char*s){ for(;*s;++s){h^=(unsigned char)*s; h*=1099511628211ULL;} };
  for(auto&m:r.toolpath.moves){ snprintf(b,sizeof b,"%d %a %a %a|",(int)m.kind,m.to.x,m.to.y,m.to.z); add(b);} for(auto&d:r.diagnostics) add(d.message.c_str());
  std::printf("%d %zu %zu %016llx\n",sc,r.toolpath.moves.size(),r.diagnostics.size(),h); } }
