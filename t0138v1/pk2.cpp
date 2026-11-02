// Pocket/Profile near-collapse A/B: features barely wider than the tool.
#include <cmath>
#include <cstdio>
#include <vector>
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/tooling/tool.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
static const double PI=3.14159265358979323846;
static tooling::Tool em(double d){ tooling::Tool t; t.id="T"; t.name="T"; t.type=tooling::ToolType::EndMill; t.diameter=d; auto& cd=t.defaultCuttingData; cd.toolNumber=1; cd.feedRate=1000; cd.plungeRate=300; cd.spindleSpeed=18000; cd.stepover=d*0.4; cd.passDepth=2; return t; }
static Contour stadium(Vec2 c,double L,double w,double ph){ Vec2 u=Vec2::fromAngle(ph), n{-u.y,u.x}; Vec2 a=c-u*(L/2), b=c+u*(L/2); Contour o; o.closed=true;
  o.spans.emplace_back(LineSpan{a-n*w,b-n*w}); o.spans.emplace_back(ArcSpan::fromCentre(b,b-n*w,b+n*w,ArcDirection::CCW)); o.spans.emplace_back(LineSpan{b+n*w,a+n*w}); o.spans.emplace_back(ArcSpan::fromCentre(a,a+n*w,a-n*w,ArcDirection::CCW)); return o; }
static unsigned long long H(const std::vector<Move>& mv){ unsigned long long h=1469598103934665603ULL; char b[200]; for(auto& m:mv){ std::snprintf(b,sizeof b,"%d %.6f %.6f %.6f|",(int)m.kind,m.to.x+0.0,m.to.y+0.0,m.to.z+0.0); for(char* s=b;*s;++s){ h^=(unsigned char)*s; h*=1099511628211ULL; } } return h; }
int main(){
  int sc=0;
  for(double D: {1.0,3.175,6.0}) for(Vec2 at: {Vec2{12.3,-4.5},Vec2{12345.6,-23456.7}}) for(int shape=0;shape<3;++shape) for(int k=-60;k<=60;++k){
    double delta=k*2.5e-5; double r=D/2;
    std::vector<Contour> cs;
    if(shape==0) cs.push_back(Contour::circle(at,r+delta));
    else if(shape==1) cs.push_back(stadium(at,20,r+delta,0.37));
    else { cs.push_back(Contour::circle(at,3*r+delta)); cs.push_back(Contour::circle(at,r,ArcDirection::CW)); } // annulus width 2r+delta
    for(int strat=0;strat<2;++strat){
      PocketParams P; P.strategy= strat?PocketStrategy::Raster:PocketStrategy::Offset; P.cutDepth=2; P.tolerance.chordal= (k&1)?0.0001:0.01; P.tolerance.distance=std::min(P.tolerance.distance,P.tolerance.chordal);
      PocketTool t; t.tool=em(D); auto res=pocketToolpath(cs,{t},P); if(getenv("T") && atoi(getenv("T"))==sc){ for(auto& tp:res.toolpaths){ int cnt[16]={0}; for(auto& m:tp.moves) cnt[(int)m.kind]++; std::printf("kinds:"); for(int q=0;q<8;++q) std::printf(" %d",cnt[q]); std::printf("\n"); for(auto& m:tp.moves) std::printf("  %d %.10f %.10f %.4f\n",(int)m.kind,m.to.x,m.to.y,m.to.z);} for(auto& d:res.diagnostics) std::printf("diag %s\n",d.message.c_str()); } size_t n=0; unsigned long long h=1; for(auto& tp:res.toolpaths){ n+=tp.moves.size(); h^=H(tp.moves);} 
      if(getenv("T")&&atoi(getenv("T"))==sc) return 0; std::printf("scene %d pocket D %g shape %d at %g delta %g strat %d moves %zu diags %zu hash %016llx\n",sc++,D,shape,at.x,delta,strat,n,res.diagnostics.size(),h); std::fflush(stdout);
    }
    { ProfileParams pp; pp.side=ProfileSide::Inside; pp.tolerance.chordal=(k&1)?0.0001:0.01; pp.tolerance.distance=std::min(pp.tolerance.distance,pp.tolerance.chordal);
      auto res=profileToolpath(cs,em(D),pp); std::printf("scene %d profile D %g shape %d at %g delta %g moves %zu diags %zu hash %016llx\n",sc++,D,shape,at.x,delta,res.toolpath.moves.size(),res.diagnostics.size(),H(res.toolpath.moves)); std::fflush(stdout); }
  }
}
