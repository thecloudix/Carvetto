// Bit-continuous 6-arc loop at (3e4,7e4): needs the walk-from-another-arc retry.
#include <cstdio>
#include <variant>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static Contour in; static const auto ccw = ArcDirection::CCW, cw = ArcDirection::CW;
static void arc(double x,double y,double r,double s,double e,ArcDirection d){ in.spans.emplace_back(ArcSpan{{x,y},r,s,e,d}); }
int main(){ in.closed=true;
#include "retry_case.inc"
 size_t n=in.spans.size(); for(size_t i=0;i<n;++i) std::printf("input junction %zu identical=%d\n",i,(int)(endPoint(in.spans[i])==startPoint(in.spans[(i+1)%n])));
 auto out=offsetOpenContour(in,0.031527132982214531,OffsetSide::Right,JoinType::Round,Tolerance{},2.0);
 for(auto&c:out){ size_t m=c.spans.size(); for(size_t i=0;i<m;++i){ const Span&a=c.spans[i]; const Span&b=c.spans[(i+1)%m]; double dot=(startPoint(b)-endPoint(a)).dot(tangentAt(a,1.0)); if(dot<0) std::printf("closed=%d n=%zu junction %zu behind by %.3g mm\n",c.closed,m,i,-dot);} std::printf("chain closed=%d n=%zu\n",c.closed,m);} }
