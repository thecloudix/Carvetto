// Standalone repro: reversed nanometre sliver from offsetContours (annulus, Bezier outer).
#include <cstdio>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double area(const Contour& c){ double a=0; for(auto& s:c.spans){ Vec2 p=startPoint(s), q=endPoint(s); a+=p.x*q.y-q.x*p.y;} return a/2; }
int main(){ const Vec2 c{32.939392279875733,-76.249769660635266}; const double ro=43.277926783499645-c.x, ri=6.7009639741650435;
  Contour outer; outer.closed=true; for(auto& b: ArcSpan::fullCircle(c,ro).toBeziers()) outer.spans.emplace_back(b);
  std::vector<Contour> annulus{outer, Contour::circle(c,ri,ArcDirection::CW)};
  const double d=(ro-ri)/2+0.0015;  // = 1.82029
  auto out=offsetContours(annulus,-d,JoinType::Round,Tolerance{0.001,1e-9,0.0002});
  std::printf("ro=%.17g ri=%.17g d=%.17g rings=%zu\n",ro,ri,d,out.size());
  for(size_t i=0;i<out.size();++i) std::printf(" ring %zu n=%zu area=%.3e\n",i,out[i].spans.size(),area(out[i])); }
