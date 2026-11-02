#include <cstdio>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main(){ const Vec2 c{32.939392279875733,-76.249769660635266}; const double ro=43.277926783499645-c.x, ri=6.7009639741650435;
  Contour outer; outer.closed=true; for(auto& b: ArcSpan::fullCircle(c,ro).toBeziers()) outer.spans.emplace_back(b);
  auto out=offsetContours({outer, Contour::circle(c,ri,ArcDirection::CW)},-((ro-ri)/2+0.0015),JoinType::Round,Tolerance{0.001,1e-9,0.0002});
  for(int i: {0,7}) { std::printf("ring %d:",i); for(auto& s: out[i].spans){ Vec2 p=startPoint(s); std::printf(" (%.9f,%.9f)",p.x,p.y);} std::printf("\n"); } }
