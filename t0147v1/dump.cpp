// verifier driver: dump contours + library signedArea for an external exact oracle
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include <algorithm>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
using namespace carvetto::geometry;
static std::mt19937_64 rng;
static double U(double a,double b){return std::uniform_real_distribution<double>(a,b)(rng);}
static double LU(double a,double b){return std::exp(U(std::log(a),std::log(b)));}
static int I(int a,int b){return std::uniform_int_distribution<int>(a,b)(rng);}
static Vec2 base(){ int k=I(0,5); if(k==0) return {U(-1,1),U(-1,1)}; double m=LU(1,1e9); if(k==1) m=1e9; double a=U(0,6.283185307179586); return {m*std::cos(a), m*std::sin(a)}; }
static std::vector<Vec2> star(Vec2 b,double s,int n){ std::vector<double> an(n); for(auto&a:an)a=U(0,6.283185307179586); std::sort(an.begin(),an.end()); std::vector<Vec2> p; for(double a:an){double r=s*U(0.2,1); p.push_back({b.x+r*std::cos(a), b.y+r*std::sin(a)});} return p;}
static std::vector<Vec2> rnd(Vec2 b,double s,int n){ std::vector<Vec2> p; for(int i=0;i<n;++i) p.push_back({b.x+s*U(-1,1), b.y+s*U(-1,1)}); return p;}
static void dump(const char* kind, int id, const Contour& c, const Polyline* pl){
  std::printf("C %s %d %d %a\n", kind, id, c.closed?1:0, signedArea(c));
  if(pl) std::printf("Q %a\n", pl->signedArea());
  for(const Span& s: c.spans){
    if(auto* l=std::get_if<LineSpan>(&s)) std::printf("L %a %a %a %a\n", l->start.x,l->start.y,l->end.x,l->end.y);
    else if(auto* a=std::get_if<ArcSpan>(&s)) { Vec2 sp=a->startPoint(), ep=a->endPoint(); std::printf("A %a %a %a %a %a %a %a %a %a\n", a->centre.x,a->centre.y,a->radius,a->startAngle,a->sweepAngle(), sp.x,sp.y,ep.x,ep.y);} 
    else { auto& b=std::get<BezierSpan>(s); std::printf("B %a %a %a %a %a %a %a %a\n", b.p0.x,b.p0.y,b.p1.x,b.p1.y,b.p2.x,b.p2.y,b.p3.x,b.p3.y);} }
  std::printf("E\n");
}
// closed contour through points, each edge line/arc/bezier per mode
static Contour mixed(const std::vector<Vec2>& p, int mode, double gap){
  Contour c; c.closed=true; size_t n=p.size();
  std::vector<Vec2> q=p; // junction points possibly perturbed per span start
  for(size_t i=0;i<n;++i){ Vec2 a=p[i], b=p[(i+1)%n];
    if(gap>0 && I(0,2)==0){ a=a+Vec2{U(-gap,gap),U(-gap,gap)}; }
    int m = mode==3? I(0,2): mode;
    if(m==0) c.append(LineSpan{a,b});
    else if(m==1){ Vec2 mid=(a+b)*0.5; Vec2 d=b-a; Vec2 nrm{-d.y,d.x}; double t=U(-1.5,1.5); Vec2 ce=mid+nrm*t; ArcDirection dir = I(0,1)?ArcDirection::CCW:ArcDirection::CW; c.append(ArcSpan::fromCentre(ce,a,b,dir)); }
    else { Vec2 d=b-a; double L=std::hypot(d.x,d.y); c.append(BezierSpan{a, a+Vec2{U(-L,L),U(-L,L)}, b+Vec2{U(-L,L),U(-L,L)}, b}); }
  }
  return c;
}
int main(int argc,char**argv){
  rng.seed(std::strtoull(argv[1],nullptr,10)); int N=std::atoi(argv[2]);
  for(int id=0;id<N;++id){
    int k=I(0,11); Vec2 b=base(); double s=LU(1e-3,10); if(k==11){ s=LU(1e5,1e6);} 
    std::vector<Vec2> p;
    switch(k){
      case 0: case 1: { p = k==0? star(b,s,I(3,20)) : rnd(b,s,I(3,12)); if(I(0,1)) std::reverse(p.begin(),p.end()); Polyline pl{p,true}; Contour c=Contour::fromPolyline(pl); dump(k==0?"star":"selfx",id,c,&pl); break; }
      case 2: { // degenerate: collinear / needle
        Vec2 d{U(-1,1),U(-1,1)}; int n=I(3,6); for(int i=0;i<n;++i){ double t=U(-s,s); p.push_back(b+d*t);} if(I(0,1)) p.push_back(p[0]+Vec2{s*1e-9,0}); Polyline pl{p,true}; dump("degen",id,Contour::fromPolyline(pl),&pl); break; }
      case 3: { Contour c=Contour::circle(b,s, I(0,1)?ArcDirection::CCW:ArcDirection::CW); dump("circle",id,c,nullptr); break; }
      case 4: { // slot
        double w=s*U(0.1,1); Vec2 a0=b, a1=b+Vec2{s,0}; Contour c; c.closed=true;
        c.append(LineSpan{a0+Vec2{0,-w},a1+Vec2{0,-w}}); c.append(ArcSpan::fromCentre(a1,a1+Vec2{0,-w},a1+Vec2{0,w},ArcDirection::CCW));
        c.append(LineSpan{a1+Vec2{0,w},a0+Vec2{0,w}}); c.append(ArcSpan::fromCentre(a0,a0+Vec2{0,w},a0+Vec2{0,-w},ArcDirection::CCW));
        if(I(0,1)) c.reverse(); dump("slot",id,c,nullptr); break; }
      case 5: case 6: case 7: { p=star(b,s,I(3,10)); Contour c=mixed(p,k-4,0); if(I(0,1)) c.reverse(); dump(k==5?"arcs":k==6?"bez":"mix",id,c,nullptr); break; }
      case 8: case 9: { p=star(b,s,I(3,10)); Contour c=mixed(p,3,s*LU(1e-6,1e-2)); if(I(0,1)) c.reverse(); dump("gapped",id,c,nullptr); break; }
      case 10: { p=star(b,s,I(3,8)); Contour c=mixed(p,3,0); c.closed=false; dump("open",id,c,nullptr); break; }
      case 11: { p=star(b,s,I(3,10)); Contour c=mixed(p,3,0); dump("large",id,c,nullptr); break; }
    }
  }
}
