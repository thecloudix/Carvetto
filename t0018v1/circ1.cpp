#include <cstdio>
#include <cstdlib>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double area(const Contour& c){ double a=0; for(auto& s:c.spans){ Vec2 p=startPoint(s), q=endPoint(s); a+=p.x*q.y-q.x*p.y;} return a/2; }
int main(int argc,char**argv){ double R=atof(argv[1]), d=atof(argv[2]), t=atof(argv[3]); double cx=argc>4?atof(argv[4]):0, cy=argc>5?atof(argv[5]):0;
  for(JoinType j:{JoinType::Round,JoinType::Sharp,JoinType::Square}){ auto out=offsetContours({Contour::circle({cx,cy},R)},-d,j,Tolerance{0.001,1e-9,t});
  std::printf("rings %zu:",out.size()); for(auto& c:out){ Box2 b=c.bounds(); std::printf(" [n=%zu area=%.4e box (%.5f,%.5f)-(%.5f,%.5f)]",c.spans.size(),area(c),b.min.x,b.min.y,b.max.x,b.max.y);} std::printf("\n"); } }
