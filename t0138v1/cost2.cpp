#include <chrono>
#include <cstdio>
#include <vector>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static Contour poly(const std::vector<Vec2>& p){ Contour c; c.closed=true; for(size_t i=0;i<p.size();++i) c.spans.emplace_back(LineSpan{p[i],p[(i+1)%p.size()]}); return c; }
int main(){ const Tolerance T{0.001,1e-9,0.001};
  for(int N: {2000,10000,20000}){ std::vector<Contour> hs; for(int i=0;i<N;++i){ double x=(i%100)*0.7, y=(i/100)*0.7; double w= i%2? 0.2:0.3; hs.push_back(poly({{x,y},{x+0.5,y},{x+0.5,y+w},{x,y+w}})); }
    double best=1e9; size_t n=0; for(int k=0;k<3;++k){ auto t0=std::chrono::steady_clock::now(); n=offsetContours(hs,-(0.1-3e-6),JoinType::Round,T).size(); best=std::min(best,std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count()); }
    std::printf("N=%d mixed dust/genuine rects: rings=%zu %.4f s\n",N,n,best); std::fflush(stdout); } }
