// Verifier oracle for T-0018 (independent of offset_inradius_test.cpp).
// Usage: inr <seed0> <nseeds> [dumpfile]
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static const double PI = 3.14159265358979323846;
using Ring = std::vector<Vec2>;
struct R { std::mt19937_64 g; double u(double a,double b){return a+(b-a)*((g()>>11)*(1.0/9007199254740992.0));} int k(int n){return int(g()%uint64_t(n));} };

static Ring sample(const Contour& c){ Ring r; for(const Span& s: c.spans){ if(std::holds_alternative<LineSpan>(s)){ r.push_back(startPoint(s)); continue;} int n=200; if(auto*a=std::get_if<ArcSpan>(&s)) n=std::max(16,int(std::ceil(std::fabs(a->sweepAngle())/(2*std::acos(1-std::min(1.0,2e-5/a->radius)))))); n=std::min(n,8000); for(int i=0;i<n;++i) r.push_back(pointAt(s,double(i)/n)); } return r; }
static Ring verts(const Contour& c){ Ring r; for(const Span& s:c.spans) r.push_back(startPoint(s)); return r; }
static double area(const Ring& r){ double a=0; for(size_t i=0,n=r.size();i<n;++i){ a+=r[i].x*r[(i+1)%n].y-r[(i+1)%n].x*r[i].y;} return a/2; }
static int wind(const Ring& r, Vec2 p){ int w=0; for(size_t i=0,n=r.size();i<n;++i){ Vec2 a=r[i], b=r[(i+1)%n]; double c=(b.x-a.x)*(p.y-a.y)-(p.x-a.x)*(b.y-a.y); if(a.y<=p.y){ if(b.y>p.y&&c>0) ++w;} else if(b.y<=p.y&&c<0) --w;} return w; }
struct Src { std::vector<Ring> rings; Vec2 lo{1e300,1e300}, hi{-1e300,-1e300};
  explicit Src(const std::vector<Contour>& cs){ for(auto& c:cs){ rings.push_back(sample(c)); for(auto p:rings.back()){ lo={std::min(lo.x,p.x),std::min(lo.y,p.y)}; hi={std::max(hi.x,p.x),std::max(hi.y,p.y)}; } } }
  bool in(Vec2 p) const { int w=0; for(auto& r:rings) w+=wind(r,p); return w!=0; }
  double dist(Vec2 p) const { double b=1e300; for(auto& r:rings) for(size_t i=0,n=r.size();i<n;++i){ Vec2 a=r[i], d=r[(i+1)%n]-a; double l=d.dot(d); double t=l>0?std::clamp((p-a).dot(d)/l,0.0,1.0):0; Vec2 e=p-a-d*t; b=std::min(b,e.dot(e)); } return std::sqrt(b); }
  double sd(Vec2 p) const { double d=dist(p); return in(p)?d:-d; }
};
static double inradiusBF(const Src& s){ const int N=90; double ext=std::max(s.hi.x-s.lo.x,s.hi.y-s.lo.y); double h=ext/N; double best=0; std::vector<std::pair<double,Vec2>> c;
  for(double y=s.lo.y;y<=s.hi.y;y+=h) for(double x=s.lo.x;x<=s.hi.x;x+=h){ double v=s.sd({x,y}); if(v>0) c.push_back({v,{x,y}}); }
  std::sort(c.begin(),c.end(),[](auto&a,auto&b){return a.first>b.first;}); if(c.empty()) return 0;
  double top=c[0].first; for(auto [v,p]: c){ if(v<top-0.8*h) break; for(double st=h; st>1e-6; st*=0.6){ bool imp=true; while(imp){ imp=false; for(int dj=-1;dj<=1;++dj) for(int di=-1;di<=1;++di){ Vec2 q=p+Vec2{di*st,dj*st}; double w=s.sd(q); if(w>v){v=w;p=q;imp=true;} } } } best=std::max(best,v);} return best; }

struct Case { std::string name; std::vector<Contour> cs; double inr; bool analytic; };
static Contour poly(const std::vector<Vec2>& p){ Contour c; c.closed=true; for(size_t i=0;i<p.size();++i) c.spans.emplace_back(LineSpan{p[i],p[(i+1)%p.size()]}); return c; }
static Contour rev(const Contour& c){ return c.reversed(); }
static Contour arcsCircle(Vec2 c,double r,int n,double phase){ Contour o; o.closed=true; for(int i=0;i<n;++i){ Vec2 a=c+Vec2::fromAngle(phase+2*PI*i/n,r), b=c+Vec2::fromAngle(phase+2*PI*(i+1)/n,r); o.spans.emplace_back(ArcSpan::fromCentre(c,a,b,ArcDirection::CCW)); } return o; }
static Contour bezCircle(Vec2 c,double r){ Contour o; o.closed=true; for(auto& b: ArcSpan::fullCircle(c,r).toBeziers()) o.spans.emplace_back(b); return o; }
static Contour ngon(Vec2 c,double apothem,int n,double ph){ std::vector<Vec2> p; double R_=apothem/std::cos(PI/n); for(int i=0;i<n;++i) p.push_back(c+Vec2::fromAngle(ph+2*PI*i/n,R_)); return poly(p); }
static Contour ellipse(Vec2 c,double a,double b,int n,double ph){ std::vector<Vec2> p; for(int i=0;i<n;++i){ double t=2*PI*i/n; Vec2 q{a*std::cos(t),b*std::sin(t)}; p.push_back(c+Vec2{q.x*std::cos(ph)-q.y*std::sin(ph), q.x*std::sin(ph)+q.y*std::cos(ph)}); } return poly(p); }
static Contour stadium(Vec2 c,double L,double w,double ph){ Vec2 u=Vec2::fromAngle(ph), n{-u.y,u.x}; Vec2 a=c-u*(L/2), b=c+u*(L/2); Contour o; o.closed=true;
  o.spans.emplace_back(LineSpan{a-n*w,b-n*w}); o.spans.emplace_back(ArcSpan::fromCentre(b,b-n*w,b+n*w,ArcDirection::CCW)); o.spans.emplace_back(LineSpan{b+n*w,a+n*w}); o.spans.emplace_back(ArcSpan::fromCentre(a,a+n*w,a-n*w,ArcDirection::CCW)); return o; }
static Contour dumb(Vec2 c1,double r1,Vec2 c2,double r2,double h){ Vec2 u=(c2-c1)*(1/distance(c1,c2)), n{-u.y,u.x}; double a1=std::sqrt(r1*r1-h*h), a2=std::sqrt(r2*r2-h*h);
  Vec2 p1t=c1+u*a1+n*h, p1b=c1+u*a1-n*h, p2t=c2-u*a2+n*h, p2b=c2-u*a2-n*h; Contour o; o.closed=true; o.spans.emplace_back(ArcSpan::fromCentre(c1,p1t,p1b,ArcDirection::CCW)); o.spans.emplace_back(LineSpan{p1b,p2b}); o.spans.emplace_back(ArcSpan::fromCentre(c2,p2b,p2t,ArcDirection::CCW)); o.spans.emplace_back(LineSpan{p2t,p1t}); return o; }
static Contour wobbly(R& r,Vec2 c,double base,int n){ // smooth radial polygon with many nearly flat vertices
  double a1=r.u(0,0.3)*base, a2=r.u(0,0.15)*base; int f1=2+r.k(4), f2=5+r.k(6); double p1=r.u(0,6), p2=r.u(0,6); std::vector<Vec2> p; for(int i=0;i<n;++i){ double t=2*PI*i/n; p.push_back(c+Vec2::fromAngle(t, base+a1*std::sin(f1*t+p1)+a2*std::sin(f2*t+p2))); } return poly(p); }
static Contour blob(R& r,Vec2 c,double rmin,double rmax,int k){ std::vector<Vec2> p; for(int i=0;i<k;++i) p.push_back(c+Vec2::fromAngle(2*PI*i/k,r.u(rmin,rmax))); Contour o; o.closed=true; for(int i=0;i<k;++i){ Vec2 a=p[i], b=p[(i+1)%k], pa=p[(i+k-1)%k], nb=p[(i+2)%k]; o.spans.emplace_back(BezierSpan{a,a+(b-pa)*(1.0/6),b-(nb-a)*(1.0/6),b}); } return o; }
static Contour starp(R& r,Vec2 c,double rmin,double rmax,int k){ std::vector<Vec2> p; for(int i=0;i<k;++i) p.push_back(c+Vec2::fromAngle(2*PI*(i+r.u(-.3,.3))/k,(i%2)?r.u(rmin,rmax*0.6):r.u(rmax*0.6,rmax))); return poly(p); }
static Contour sector(Vec2 c,double ro,double ri,double a0,double sw){ Contour o; o.closed=true; Vec2 A=c+Vec2::fromAngle(a0,ro), B=c+Vec2::fromAngle(a0+sw,ro), C=c+Vec2::fromAngle(a0+sw,ri), D=c+Vec2::fromAngle(a0,ri);
  o.spans.emplace_back(ArcSpan::fromCentre(c,A,B,ArcDirection::CCW)); o.spans.emplace_back(LineSpan{B,C}); o.spans.emplace_back(ArcSpan::fromCentre(c,C,D,ArcDirection::CW)); o.spans.emplace_back(LineSpan{D,A}); return o; }

static FILE* dumpF=nullptr;
static long gSeed=0; static const char* gOnly=nullptr; static long nFail=0, nOff=0, nVanish=0, nVanishOK=0;
static void fail(const std::string& s){ if(nFail++<(getenv("ALLFAIL")?100000:25)) {std::printf("FAIL seed %ld %s\n",gSeed,s.c_str()); std::fflush(stdout);} }
static const char* jn(JoinType j){ return j==JoinType::Round?"round":j==JoinType::Sharp?"sharp":"square"; }

static void dump(const std::string& key,const std::vector<Contour>& out){ if(!dumpF) return; std::fprintf(dumpF,"K %s %zu\n",key.c_str(),out.size()); for(auto& c:out){ auto v=verts(c); std::fprintf(dumpF,"R %zu\n",v.size()); for(auto p:v) std::fprintf(dumpF,"%.12g %.12g\n",p.x,p.y);} }

static bool segx(Vec2 a,Vec2 b,Vec2 c,Vec2 d){ auto o=[](Vec2 p,Vec2 q,Vec2 r){ return (q-p).cross(r-p); }; double d1=o(a,b,c),d2=o(a,b,d),d3=o(c,d,a),d4=o(c,d,b); return ((d1>0)!=(d2>0)) && ((d3>0)!=(d4>0)) && d1!=0 && d2!=0 && d3!=0 && d4!=0; }
static void runCase(R& rng, const Case& cs, const Tolerance& tol){
  if(getenv("SHOW") && cs.cs.size()>1) if(auto*a=std::get_if<ArcSpan>(&cs.cs[1].spans[0])) std::printf("SHOWHOLE %s c=(%.17g,%.17g) r=%.17g\n",cs.name.c_str(),a->centre.x,a->centre.y,a->radius);
  if(getenv("SHOW")) if(auto*b=std::get_if<BezierSpan>(&cs.cs[0].spans[0])) std::printf("SHOWBEZ p0=(%.17g,%.17g)\n",b->p0.x,b->p0.y);
  if(getenv("SHOW")) if(auto*a=std::get_if<ArcSpan>(&cs.cs[0].spans[0])) std::printf("SHOW %s c=(%.17g,%.17g) r=%.17g tol=%g\n",cs.name.c_str(),a->centre.x,a->centre.y,a->radius,tol.chordal);
  Src s(cs.cs); if(getenv("SELFX")){ long x=0; for(auto& r:s.rings){ size_t n=r.size(); for(size_t i=0;i<n;++i) for(size_t j=i+2;j<n;++j){ if(i==0&&j==n-1) continue; if(segx(r[i],r[(i+1)%n],r[j],r[(j+1)%n])) ++x; } } std::printf("case %s seed %ld selfx %ld\n",cs.name.c_str(),gSeed,x); if(getenv("SELFXONLY")) return; } double sign= area(s.rings[0])>0?1:-1; double inr=cs.inr; bool ana=cs.analytic; if(inr<0) inr=inradiusBF(s);
  const double slack=2*tol.chordal+2e-5;
  std::vector<double> ds={0.1*inr,0.4*inr,0.75*inr,0.95*inr,inr-3*tol.chordal-1e-3, inr+1e-3+2.5*tol.chordal, inr*1.01+0.01+2.5*tol.chordal, 1.3*inr, 2*inr, 5*inr, rng.u(0.02,3)*inr};
  // coverage grid
  std::vector<std::pair<Vec2,double>> nodes; { int N=60; double ext=std::max(s.hi.x-s.lo.x,s.hi.y-s.lo.y)/N; for(double y=s.lo.y+ext*rng.u(0,1);y<=s.hi.y;y+=ext) for(double x=s.lo.x+ext*rng.u(0,1);x<=s.hi.x;x+=ext){ double v=s.sd({x,y}); if(v>0) nodes.push_back({{x,y},v}); } }
  for(JoinType j: {JoinType::Round,JoinType::Sharp,JoinType::Square}){
    double reach = j==JoinType::Round?1.0: j==JoinType::Square?std::sqrt(2.0):2.0;
    for(size_t di=0; di<ds.size(); ++di){ double d=ds[di]; if(d<=0) continue; ++nOff;
      auto out=offsetContours(cs.cs,-d,j,tol);
      char tag[512]; std::snprintf(tag,sizeof tag,"%s %s tol=%g d=%.6g inr=%.6g",cs.name.c_str(),jn(j),tol.chordal,d,inr);
      if(di<4) dump(std::string(tag).substr(0,0)+cs.name+"|"+jn(j)+"|"+std::to_string(tol.chordal)+"|in"+std::to_string(di),out);
      bool mustVanish = ana ? d>=inr+1e-3+2.5*tol.chordal-1e-12 : d>=inr*1.01+0.01+2.5*tol.chordal-1e-12;
      if(mustVanish){ ++nVanish; if(out.empty()) ++nVanishOK; else fail(std::string(tag)+" expected empty, got "+std::to_string(out.size())+" rings"); }
      std::vector<Ring> rings; for(auto& c:out) rings.push_back(verts(c));
      // (a) depth of output points
      bool bad=false; for(auto& r:rings){ size_t n=r.size(); size_t step=std::max<size_t>(1,n/800); for(size_t i=0;i<n && !bad;i+=step) for(double t: {0.0,0.37,0.81}){ Vec2 p=r[i]+(r[(i+1)%n]-r[i])*t; double v=s.sd(p); if(v<d-slack){ char b[200]; std::snprintf(b,sizeof b," point (%.5f,%.5f) depth %.6f < d",p.x,p.y,v); fail(std::string(tag)+b); bad=true; break; } } }
      // (b) coverage
      int checked=0; for(auto& [p,v]: nodes){ if(v< reach*d+slack) continue; if(++checked>1500) break; int w=0; for(auto& r:rings) w+=wind(r,p); if(w==0){ char b[200]; std::snprintf(b,sizeof b," node (%.5f,%.5f) depth %.6f uncovered",p.x,p.y,v); fail(std::string(tag)+b); break; } }
      // (b2) no node shallower than d-slack covered
      for(auto& [p,v]: nodes){ if(v>=d-slack) continue; int w=0; for(auto& r:rings) w+=wind(r,p); if(w!=0){ char b[200]; std::snprintf(b,sizeof b," shallow node (%.5f,%.5f) depth %.6f covered",p.x,p.y,v); fail(std::string(tag)+b); break; } }
      // (c) orientation vs nesting
      for(size_t k=0;k<rings.size();++k){ if(rings[k].size()<3){ fail(std::string(tag)+" degenerate ring"); continue;} int depth=0; Vec2 pr=(rings[k][0]+rings[k][1])*0.5; for(size_t m=0;m<rings.size();++m) if(m!=k && wind(rings[m],pr)!=0) ++depth; double want= depth%2==0?sign:-sign; if(area(rings[k])*want<=0) fail(std::string(tag)+" ring "+std::to_string(k)+" wrong orientation at depth "+std::to_string(depth)+" area "+std::to_string(area(rings[k])*1e12)+"e-12 n "+std::to_string(rings[k].size())); }
    }
    // outward offsets, for A/B drift only
    for(double f: {0.05,0.5,2.0}){ auto out=offsetContours(cs.cs,f*inr,j,tol); dump(cs.name+"|"+jn(j)+"|"+std::to_string(tol.chordal)+"|out"+std::to_string(f),out); ++nOff; }
  }
}

int main(int argc,char** argv){
  long s0=argc>1?std::atol(argv[1]):1, ns=argc>2?std::atol(argv[2]):3; if(argc>3) dumpF=std::fopen(argv[3],"w");
  const double tols[]={0.0002,0.001,0.01,0.05,0.3};
  gOnly=getenv("ONLY"); for(long seed=s0; seed<s0+ns; ++seed){ gSeed=seed; R rng{std::mt19937_64(uint64_t(seed)*0x9E3779B97F4A7C15ULL)};
    std::vector<Case> cases; auto C=[&]{ return Vec2{rng.u(-80,80),rng.u(-80,80)}; };
    { double r=rng.u(0.5,30); Vec2 c=C(); cases.push_back({"circle",{Contour::circle(c,r)},r,true}); cases.push_back({"circleCW",{Contour::circle(c,r,ArcDirection::CW)},r,true});
      int n=2+rng.k(9); cases.push_back({"arcs"+std::to_string(n),{arcsCircle(c,r,n,rng.u(0,6))},r,true}); cases.push_back({"bezcircleCW",{rev(bezCircle(c,r))},r,true}); }
    { int n=3+rng.k(400); double a=rng.u(1,25); cases.push_back({"ngon"+std::to_string(n),{ngon(C(),a,n,rng.u(0,6))},a,true}); }
    { double a=rng.u(5,30), b=a*rng.u(0.1,0.9); int n=200+rng.k(3000); cases.push_back({"ellipse"+std::to_string(n),{ellipse(C(),a,b,n,rng.u(0,6))},b*std::cos(PI/n)*0+(-1),false}); }
    { double w=rng.u(0.5,10); cases.push_back({"stadium",{rng.k(2)?stadium(C(),rng.u(0,40),w,rng.u(0,6)):rev(stadium(C(),rng.u(0,40),w,rng.u(0,6)))},w,true}); }
    { Vec2 c=C(); double ro=rng.u(5,30), ri=ro*rng.u(0.1,0.8); cases.push_back({"annulus",{bezCircle(c,ro),Contour::circle(c,ri,ArcDirection::CW)},(ro-ri)/2,true}); }
    { Vec2 c1=C(); double r1=rng.u(4,20), r2=rng.u(2,r1*0.8); double h=rng.u(0.1,0.7)*r2; Vec2 c2=c1+Vec2::fromAngle(rng.u(0,6),(r1+r2)*rng.u(1.1,2.0));
      Case k{"dumbbell",{dumb(c1,r1,c2,r2,h)},r1,true}; if(rng.k(2)) k.cs[0]=rev(k.cs[0]); cases.push_back(k);
      // lobe check
      for(JoinType j:{JoinType::Round,JoinType::Sharp,JoinType::Square}){ double d=rng.u(std::max(h,0.0)*1.05+0.05, r2-0.05); if(d>0 && d<r2-0.05){ auto o=offsetContours(k.cs,-d,j,Tolerance{}); if(o.size()!=2) fail("dumbbell both-lobes d="+std::to_string(d)+" got "+std::to_string(o.size())); }
        double d2=rng.u(r2+0.05,r1-0.05); auto o2=offsetContours(k.cs,-d2,j,Tolerance{}); if(o2.size()!=1) fail("dumbbell big-lobe d="+std::to_string(d2)+" got "+std::to_string(o2.size())); else { Box2 b=o2[0].bounds(); if(distance((b.min+b.max)*0.5,c1)>r1) fail("dumbbell surviving lobe misplaced"); } } }
    { double base=rng.u(5,25); cases.push_back({"wobbly",{wobbly(rng,C(),base,100+rng.k(1500))},-1,false}); }
    { cases.push_back({"blob",{blob(rng,C(),rng.u(3,10),rng.u(12,25),4+rng.k(10))},-1,false}); }
    { cases.push_back({"star",{starp(rng,C(),rng.u(2,6),rng.u(10,25),6+2*rng.k(15))},-1,false}); }
    { Vec2 c=C(); double ro=rng.u(10,30); Contour o=bezCircle(c,ro); std::vector<Contour> cs{o}; int nh=1+rng.k(3); for(int i=0;i<nh;++i){ double ph0=rng.u(0,6); Vec2 p=c+Vec2::fromAngle(ph0*0+2*PI*i/nh,0.5*ro); double rr=rng.u(0.5,0.2*ro)*(nh>1?0.85:1.0); cs.push_back(rng.k(2)?Contour::circle(p,rr,ArcDirection::CW):rev(ngon(p,rr,3+rng.k(60),rng.u(0,6)))); }
      // ensure holes disjoint: drop overlapping ones crudely by union-free check
      if(rng.k(2)) for(auto& x:cs) x=rev(x); cases.push_back({"holes"+std::to_string(nh),cs,-1,false}); }
    { Vec2 c=C(); double ro=rng.u(10,30), ri=ro*rng.u(0.3,0.8); cases.push_back({"sector",{sector(c,ro,ri,rng.u(0,6),rng.u(1.0,5.0))},-1,false}); }
    for(auto& cs: cases){ double t=tols[rng.k(5)]; if(gOnly && cs.name.find(gOnly)==std::string::npos) continue; if(cs.name.rfind("circle",0)==0 || (gOnly && getenv("ALLTOL"))) for(double tt: tols) runCase(rng,cs,Tolerance{0.001,1e-9,tt}); else runCase(rng,cs,Tolerance{0.001,1e-9,t}); }
  }
  std::printf("offsets=%ld vanishChecks=%ld vanishOK=%ld failures=%ld\n",nOff,nVanish,nVanishOK,nFail);
  if(dumpF) std::fclose(dumpF);
  return nFail?1:0;
}
