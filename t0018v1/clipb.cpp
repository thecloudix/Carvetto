#include <cstdio>
#include <cmath>
#include <random>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double len(const std::vector<Contour>& cs){ double l=0; for(auto& c:cs) for(auto& s:c.spans) l+=distance(startPoint(s),endPoint(s)); return l; }
int main(){ std::mt19937_64 g(11); auto U=[&](double a,double b){ return a+(b-a)*((g()>>11)*(1.0/9007199254740992.0)); };
  long n=0, both=0, neither=0, in=0, out=0, partial=0, grownOK=0, grownN=0;
  for(int it=0; it<3000; ++it){ int k=3+int(g()%7); std::vector<Vec2> p; Vec2 c{U(-50,50),U(-50,50)}; for(int i=0;i<k;++i){ double a=6.283185307179586*i/k+U(-0.2,0.2); double r=U(5,20); Vec2 q=c+Vec2::fromAngle(a,r); if(g()%2){ q.x=std::round(q.x*100)/100; q.y=std::round(q.y*100)/100; } p.push_back(q);} 
    Contour clip; clip.closed=true; for(int i=0;i<k;++i) clip.spans.emplace_back(LineSpan{p[i],p[(i+1)%k]}); if(g()%2) clip=clip.reversed();
    int e=int(g()%k); Vec2 a=p[e], b=p[(e+1)%k]; Vec2 dir=(b-a)*(1.0/distance(a,b)); double L=distance(a,b);
    Vec2 s=a-dir*5.0, t=b+dir*5.0; if(g()%2) std::swap(s,t); Contour path; path.append(LineSpan{s,t});
    auto I=booleanOp(BooleanOp::Intersection,{path},{clip}).open; auto D=booleanOp(BooleanOp::Difference,{path},{clip}).open;
    double li=len(I), lo=len(D); ++n;
    bool inF=std::fabs(li-L)<1e-3, inZ=li<1e-3, outF=std::fabs(lo-(L+10))<1e-3, outZ=std::fabs(lo-10)<1e-3;
    if(inF&&outF) ++both; else if(inZ&&outZ) { ++neither; if(neither<5) std::printf("NEITHER k=%d edge (%.4f,%.4f)-(%.4f,%.4f) li %.6f lo %.6f\n",k,a.x,a.y,b.x,b.y,li,lo);} else if(inF&&outZ) ++in; else if(inZ&&outF) ++out; else { ++partial; if(partial<5) std::printf("PARTIAL li %.6f lo %.6f L %.6f\n",li,lo,L);} 
    auto grown=offsetContours({clip},0.001,JoinType::Sharp); double lg=len(booleanOp(BooleanOp::Intersection,{path},grown).open); ++grownN; if(lg>=L-1e-6) ++grownOK; }
  std::printf("n %ld in %ld out %ld both %ld neither %ld partial %ld | grown-clip keeps edge %ld/%ld\n",n,in,out,both,neither,partial,grownOK,grownN); }
