#include <cstdio>
#include <cmath>
#include <variant>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static Contour in; static const auto ccw = ArcDirection::CCW, cw = ArcDirection::CW;
static void arc(double x,double y,double r,double s,double e,ArcDirection d){ in.spans.emplace_back(ArcSpan{{x,y},r,s,e,d}); }
int main(){ in.closed=true;
#include "gap_case.inc"
 size_t n=in.spans.size(); for(size_t i=0;i<n;++i){ double g=distance(endPoint(in.spans[i]),startPoint(in.spans[(i+1)%n])); if(g>0) std::printf("in gap %zu %.3g\n",i,g);}
 auto out=offsetOpenContour(in,0.012458969356304214,OffsetSide::Left,JoinType::Round,Tolerance{},2.0);
 for(auto&c:out){ size_t m=c.spans.size(); for(size_t i=0;i<m;++i){ const Span&a=c.spans[i]; const Span&b=c.spans[(i+1)%m]; if(!c.closed&&i+1==m)break; double g=distance(endPoint(a),startPoint(b)); if(g>1e-12){ auto&A=std::get<ArcSpan>(a); auto&B=std::get<ArcSpan>(b); double D=distance(A.centre,B.centre); std::printf("junction %zu gap %.3g rA %.6g rB %.6g D %.17g rA+rB %.17g |rA-rB| %.17g ptA (%.9f,%.9f)\n",i,g,A.radius,B.radius,D,A.radius+B.radius,std::fabs(A.radius-B.radius),endPoint(a).x,endPoint(a).y);} } } }
