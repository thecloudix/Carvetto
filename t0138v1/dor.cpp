// Verifier oracle T-0138: orientation-vs-nesting near collapse, dumps rings for A/B.
// Usage: dor <seed0> <nseeds> <offsetMag> [dumpfile]
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static const double PI=3.14159265358979323846;
using Ring=std::vector<Vec2>;
struct R{ std::mt19937_64 g; double u(double a,double b){return a+(b-a)*((g()>>11)*(1.0/9007199254740992.0));} int k(int n){return int(g()%uint64_t(n));} };
static Vec2 O;
static Ring verts(const Contour& c){ Ring r; for(const Span& s:c.spans) r.push_back(startPoint(s)-O); return r; }
static double area(const Ring& r){ double a=0; for(size_t i=1,n=r.size();i+1<n;++i) a+=(r[i]-r[0]).cross(r[i+1]-r[0]); return a/2; }
static double perim(const Ring& r){ double l=0; for(size_t i=0,n=r.size();i<n;++i) l+=distance(r[i],r[(i+1)%n]); return l; }
static int wind(const Ring& r, Vec2 p){ int w=0; for(size_t i=0,n=r.size();i<n;++i){ Vec2 a=r[i], b=r[(i+1)%n]; double c=(b.x-a.x)*(p.y-a.y)-(p.x-a.x)*(b.y-a.y); if(a.y<=p.y){ if(b.y>p.y&&c>0) ++w;} else if(b.y<=p.y&&c<0) --w;} return w; }
static double inrad(const Ring& r){ double lx=1e300,ly=1e300,hx=-1e300,hy=-1e300; for(auto q:r){lx=std::min(lx,q.x);ly=std::min(ly,q.y);hx=std::max(hx,q.x);hy=std::max(hy,q.y);} double best=0; int N=120; for(int i=0;i<=N;++i) for(int j=0;j<=N;++j){ Vec2 p{lx+(hx-lx)*i/N, ly+(hy-ly)*j/N}; if(wind(r,p)==0) continue; double b=1e300; for(size_t k=0,n=r.size();k<n;++k){ Vec2 a=r[k], d=r[(k+1)%n]-a; double l=d.dot(d); double t=l>0?std::clamp((p-a).dot(d)/l,0.0,1.0):0; Vec2 e=p-a-d*t; b=std::min(b,e.dot(e)); } best=std::max(best,std::sqrt(b)); } return best; }
static Contour poly(const std::vector<Vec2>& p){ Contour c; c.closed=true; for(size_t i=0;i<p.size();++i) c.spans.emplace_back(LineSpan{p[i],p[(i+1)%p.size()]}); return c; }
static Contour bez(Vec2 c,double r){ Contour o; o.closed=true; for(auto& b: ArcSpan::fullCircle(c,r).toBeziers()) o.spans.emplace_back(b); return o; }
static Contour ngon(Vec2 c,double ap,int n,double ph){ std::vector<Vec2> p; double Rr=ap/std::cos(PI/n); for(int i=0;i<n;++i) p.push_back(c+Vec2::fromAngle(ph+2*PI*i/n,Rr)); return poly(p); }
static Contour stadium(Vec2 c,double L,double w,double ph){ Vec2 u=Vec2::fromAngle(ph), n{-u.y,u.x}; Vec2 a=c-u*(L/2), b=c+u*(L/2); Contour o; o.closed=true;
  o.spans.emplace_back(LineSpan{a-n*w,b-n*w}); o.spans.emplace_back(ArcSpan::fromCentre(b,b-n*w,b+n*w,ArcDirection::CCW)); o.spans.emplace_back(LineSpan{b+n*w,a+n*w}); o.spans.emplace_back(ArcSpan::fromCentre(a,a+n*w,a-n*w,ArcDirection::CCW)); return o; }
static Contour ellipse(Vec2 c,double a,double b,int n,double ph){ std::vector<Vec2> p; for(int i=0;i<n;++i){ double t=2*PI*i/n; Vec2 q{a*std::cos(t),b*std::sin(t)}; p.push_back(c+Vec2{q.x*std::cos(ph)-q.y*std::sin(ph), q.x*std::sin(ph)+q.y*std::cos(ph)}); } return poly(p); }
static Contour starp(R& r,Vec2 c,double rmin,double rmax,int k){ std::vector<Vec2> p; for(int i=0;i<k;++i) p.push_back(c+Vec2::fromAngle(2*PI*(i+r.u(-.3,.3))/k,(i%2)?r.u(rmin,rmax*0.6):r.u(rmax*0.6,rmax))); return poly(p); }
static const char* jn(JoinType j){ return j==JoinType::Round?"round":j==JoinType::Sharp?"sharp":"square"; }
static FILE* dumpF=nullptr; static long nOff=0,nBad=0,nDeg=0,nRings=0,nTiny3=0; static double minHW=1e300;
static long gSeed;
static void check(const std::string& key,const std::vector<Contour>& out,double sign){
  ++nOff; std::vector<Ring> rings; for(auto& c:out) rings.push_back(verts(c)); nRings+=long(rings.size());
  if(dumpF){ std::fprintf(dumpF,"K %s %zu\n",key.c_str(),rings.size()); for(auto& r:rings){ double a=area(r),p=perim(r); double lx=1e300,ly=1e300,hx=-1e300,hy=-1e300; for(auto q:r){lx=std::min(lx,q.x);ly=std::min(ly,q.y);hx=std::max(hx,q.x);hy=std::max(hy,q.y);} std::fprintf(dumpF,"R %zu %.10e %.10e %.9f %.9f %.9f %.9f %.3e\n",r.size(),a,p,lx,ly,hx,hy,(p>0&&std::fabs(a)/p<1e-5&&r.size()<5000)?inrad(r):-1.0);} }
  for(size_t k=0;k<rings.size();++k){ if(rings[k].size()<3){ ++nDeg; continue; } double a=area(rings[k]); double hw=std::fabs(a)/perim(rings[k]); minHW=std::min(minHW,hw); if(rings[k].size()==3 && std::fabs(a)<1e-10) ++nTiny3;
    int depth=0; Vec2 pr=(rings[k][0]+rings[k][1])*0.5; for(size_t m=0;m<rings.size();++m) if(m!=k && wind(rings[m],pr)!=0) ++depth; double want=depth%2==0?sign:-sign;
    if(a*want<=0){ if(nBad++<20) std::printf("BAD seed %ld %s ring %zu n %zu area %.3e depth %d\n",gSeed,key.c_str(),k,rings[k].size(),a,depth); } }
}
int main(int argc,char**argv){
  long s0=std::atol(argv[1]), ns=std::atol(argv[2]); double mag=std::atof(argv[3]); if(argc>4) dumpF=std::fopen(argv[4],"w");
  const double tols[]={0.0002,0.001,0.01,0.05};
  for(long seed=s0;seed<s0+ns;++seed){ gSeed=seed; R rng{std::mt19937_64(uint64_t(seed)*0x9E3779B97F4A7C15ULL+uint64_t(mag))};
    double ang=rng.u(0,2*PI); O=Vec2::fromAngle(ang,mag); O={std::round(O.x*7)/7+0.123456789,std::round(O.y*7)/7-0.987654321}; if(mag==0) O={0,0};
    struct Case{std::string n; std::vector<Contour> cs; double inr;};
    std::vector<Case> cases; auto C=[&]{ return O+Vec2{rng.u(-80,80),rng.u(-80,80)}; };
    double tol=tols[rng.k(4)]; double rmax= tol<0.0005?6:tol<0.005?15:30;
    { double r=rng.u(0.3,rmax); Vec2 c=C(); cases.push_back({"circle",{Contour::circle(c,r)},r}); cases.push_back({"circleCW",{Contour::circle(c,r,ArcDirection::CW)},r}); cases.push_back({"bezCW",{bez(c,r).reversed()},r}); }
    { Vec2 c=C(); double ro=rng.u(2,rmax), ri=ro*rng.u(0.1,0.8); Case k{"annulus",{bez(c,ro),Contour::circle(c,ri,ArcDirection::CW)},(ro-ri)/2}; if(rng.k(2)) for(auto& x:k.cs) x=x.reversed(); cases.push_back(k); }
    { int n=3+rng.k(300); double a=rng.u(0.5,rmax); cases.push_back({"ngon",{ngon(C(),a,n,rng.u(0,6))},a}); }
    { double w=rng.u(0.2,rmax/3); cases.push_back({"stadium",{stadium(C(),rng.u(0,40),w,rng.u(0,6))},w}); }
    { double a=rng.u(2,rmax), b=a*rng.u(0.1,0.9); cases.push_back({"ellipse",{ellipse(C(),a,b,100+rng.k(1500),rng.u(0,6))},-1}); }
    { cases.push_back({"star",{starp(rng,C(),rng.u(1,4),rng.u(5,rmax+5),6+2*rng.k(12))},-1}); }
    { Vec2 c=C(); double ro=rng.u(5,rmax+5); std::vector<Contour> cs{bez(c,ro)}; int nh=1+rng.k(3); for(int i=0;i<nh;++i){ Vec2 p=c+Vec2::fromAngle(2*PI*i/nh,0.5*ro); double rr=rng.u(0.3,0.2*ro)*0.85; cs.push_back(rng.k(2)?Contour::circle(p,rr,ArcDirection::CW):ngon(p,rr,3+rng.k(60),rng.u(0,6)).reversed()); } cases.push_back({"holes",cs,-1}); }
    { // text-like: many small circles and stadiums
      std::vector<Contour> cs; double r=rng.u(0.2,1.0); Vec2 b=C(); for(int i=0;i<12;++i) cs.push_back(i%2?Contour::circle(b+Vec2{i*3.0*r,0},r):stadium(b+Vec2{i*3.0*r,0},r,r,1.57)); cases.push_back({"text",cs,r}); }
    for(auto& cs:cases){
      Tolerance T{0.001,1e-9,tol}; double sign=0; { Ring r0=verts(Contour{}); (void)r0; }
      // sign from first contour, approx via polygonised area of sample points
      { Ring s; for(const Span& sp:cs.cs[0].spans) for(int i=0;i<8;++i) s.push_back(pointAt(sp,i/8.0)-O); sign=area(s)>0?1:-1; }
      // find collapse distance by bisection (round join)
      double lo=0, hi=cs.inr>0?cs.inr*1.05+0.01:60; for(int it=0;it<34;++it){ double m=(lo+hi)/2; if(offsetContours(cs.cs,-m,JoinType::Round,T).empty()) hi=m; else lo=m; }
      std::vector<double> ds; for(double dd: {-3e-3,-1e-3,-3e-4,-1e-4,-3e-5,-1e-5,-3e-6,-1e-6,-1e-7,0.0,1e-7,1e-6}) ds.push_back(lo+dd*(1+rng.u(-0.2,0.2)));
      if(cs.inr>0) for(double dd: {-2e-3,-1.6e-3,-6e-4,0.0,6e-4,1.5e-3}) ds.push_back(cs.inr+dd+rng.u(-1e-5,1e-5));
      for(JoinType j:{JoinType::Round,JoinType::Sharp,JoinType::Square}) for(size_t i=0;i<ds.size();++i){ double d=ds[i]; if(d<=0) continue;
        char key[256]; std::snprintf(key,sizeof key,"s%ld|%s|%s|%g|%zu",seed,cs.n.c_str(),jn(j),tol,i); check(key,offsetContours(cs.cs,-d,j,T),sign); }
    }
  }
  std::printf("mag=%g offsets=%ld rings=%ld bad=%ld degenerate=%ld tiny3=%ld minHalfWidth=%.3e mm\n",mag,nOff,nRings,nBad,nDeg,nTiny3,minHW);
  if(dumpF) std::fclose(dumpF); return nBad?1:0; }
