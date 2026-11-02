#include <cstdio>
#include <cmath>
#include <random>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double area(const Contour& c){ double a=0; for(auto& s:c.spans){ Vec2 p=startPoint(s), q=endPoint(s); a+=p.x*q.y-q.x*p.y;} return a/2; }
int main(int argc,char**argv){ std::mt19937_64 g(5); long bad=0, multi=0, n=0;
  for(int i=0;i<4000;++i){ double R=0.5+29.5*((g()>>11)*(1.0/9007199254740992.0)); double cx=-80+160*((g()>>11)*(1.0/9007199254740992.0)), cy=-80+160*((g()>>11)*(1.0/9007199254740992.0));
    double tols[]={0.0002,0.001,0.01}; for(double t: tols) for(double off: {3*t+1e-3, 1e-3, 3e-3, 1e-2}){ double d=R-off; for(int dir=0;dir<2;++dir){ ++n;
      auto out=offsetContours({Contour::circle({cx,cy},R,dir?ArcDirection::CW:ArcDirection::CCW)},-d,JoinType::Round,Tolerance{0.001,1e-9,t});
      if(out.size()!=1){ ++multi; if(multi<=8){ std::printf("R=%.6f c=(%.4f,%.4f) tol=%g d=%.6f dir=%d rings=%zu:",R,cx,cy,t,d,dir,out.size()); for(auto& c:out) std::printf(" [n=%zu area=%.3e]",c.spans.size(),area(c)); std::printf("\n"); } }
      for(auto& c:out) if((area(c)>0)!=(dir==0)) ++bad; } } }
  std::printf("cases %ld multiRing %ld wrongOrient %ld\n",n,multi,bad); }
