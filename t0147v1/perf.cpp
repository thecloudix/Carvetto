#include <chrono>
#include <cstdio>
#include <cmath>
#include <ctime>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
using namespace carvetto::geometry;
int main(){
  for(int kind=0;kind<4;++kind){
    Contour c; c.closed=true; Polyline pl; pl.closed=true; int n=1000;
    for(int i=0;i<n;++i){ double a0=2*kPi*i/n, a1=2*kPi*(i+1)/n; Vec2 p0{10*std::cos(a0)+5,10*std::sin(a0)}, p1{10*std::cos(a1)+5,10*std::sin(a1)}; pl.points.push_back(p0);
      if(kind==0||kind==3) c.append(LineSpan{p0,p1}); else if(kind==1) c.append(ArcSpan::fromCentre({5,0},p0,p1,ArcDirection::CCW)); else c.append(BezierSpan{p0,p0*0.7+p1*0.3,p0*0.3+p1*0.7,p1}); }
    volatile double s=0; std::clock_t t0=std::clock(); int R=20000;
    for(int r=0;r<R;++r) s = s + (kind==3? pl.signedArea() : signedArea(c));
    double ns = double(std::clock()-t0)/CLOCKS_PER_SEC/R/n*1e9;
    std::printf("kind %d ns/span %.2f\n",kind,ns);
  }
}
