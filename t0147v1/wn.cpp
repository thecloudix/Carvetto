// cross-check: integral of winding number over plane == signedArea, gapped contours
#include <cmath>
#include <cstdio>
#include <random>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
using namespace carvetto::geometry;
int main(){
  std::mt19937_64 g(7); auto U=[&](double a,double b){return std::uniform_real_distribution<double>(a,b)(g);};
  double worst=0;
  for(int t=0;t<40;++t){
    Vec2 b = t%2? Vec2{0,0} : Vec2{3e6,-2e6};
    double s=10, gap= t<20? 1.0 : 0.01; // large gaps (beyond tol) and small
    int n=5; Contour c; c.closed=true; std::vector<Vec2> p;
    for(int i=0;i<n;++i){double a=6.2831853*i/n+U(0,0.5); double r=s*U(0.5,1); p.push_back(b+Vec2{r*std::cos(a),r*std::sin(a)});}
    for(int i=0;i<n;++i){ Vec2 a=p[i]+Vec2{U(-gap,gap),U(-gap,gap)}, e=p[(i+1)%n];
      int m=i%3; if(m==0) c.append(LineSpan{a,e}); else if(m==1){Vec2 mid=(a+e)*0.5; Vec2 d=e-a; c.append(ArcSpan::fromCentre(mid+Vec2{-d.y,d.x}*U(-1,1),a,e,ArcDirection::CCW));}
      else { c.append(BezierSpan{a,a+Vec2{U(-3,3),U(-3,3)},e+Vec2{U(-3,3),U(-3,3)},e}); } }
    Box2 bx=c.bounds(); int G=600; double hx=bx.width()/G, hy=bx.height()/G; double sum=0; Tolerance tol{};
    for(int i=0;i<G;++i) for(int j=0;j<G;++j){ Vec2 q{bx.min.x+(i+0.5)*hx, bx.min.y+(j+0.5)*hy}; sum+=windingNumber(c,q,tol);} sum*=hx*hy;
    double A=signedArea(c); double rel=std::fabs(sum-A)/(bx.width()*bx.height());
    if(rel>worst) worst=rel; if(rel>3e-3) std::printf("t=%d A=%g wn=%g rel=%g\n",t,A,sum,rel);
  }
  std::printf("worst rel %g\n",worst);
}
