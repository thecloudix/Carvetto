#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double area(const Contour& c){ double a=0; for(auto& s:c.spans){ Vec2 p=startPoint(s), q=endPoint(s); a+=p.x*q.y-q.x*p.y;} return a/2; }
int main(int argc,char**argv){ double ro=atof(argv[1]), ri=atof(argv[2]), d=atof(argv[3]); double t=argc>4?atof(argv[4]):0.01;
  Contour o; o.closed=true; for(auto& b: ArcSpan::fullCircle({0,0},ro).toBeziers()) o.spans.emplace_back(b);
  for(int bez=0;bez<2;++bez){ std::vector<Contour> cs{bez?o:Contour::circle({0,0},ro), Contour::circle({0,0},ri,ArcDirection::CW)};
  auto out=offsetContours(cs,d,JoinType::Round,Tolerance{0.001,1e-9,t});
  std::printf("%s rings %zu:",bez?"bez":"arc",out.size()); for(auto& c:out){ double rmin=1e9,rmax=0; for(auto&s:c.spans){ double r=startPoint(s).length(); rmin=std::min(rmin,r); rmax=std::max(rmax,r);} std::printf(" [area %.3e r %.4f..%.4f]",area(c),rmin,rmax);} std::printf("\n"); } }
