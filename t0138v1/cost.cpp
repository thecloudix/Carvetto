// Cost of dust filtering: many collapsing rings.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static const double PI=3.14159265358979323846;
static Contour poly(const std::vector<Vec2>& p){ Contour c; c.closed=true; for(size_t i=0;i<p.size();++i) c.spans.emplace_back(LineSpan{p[i],p[(i+1)%p.size()]}); return c; }
template<class F> static void timeit(const char* name,F f){ double best=1e9; size_t n=0; for(int k=0;k<3;++k){ auto t0=std::chrono::steady_clock::now(); n=f(); best=std::min(best,std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count()); } std::printf("%-58s rings=%6zu  %.4f s\n",name,n,best); }
int main(){
  const Tolerance T{0.001,1e-9,0.001};
  for(double dd: {-3e-6,-1e-6,0.0,1e-6,-0.0016}){
    // 2000 circles r=0.5 in a grid ("text")
    std::vector<Contour> cs; for(int i=0;i<2000;++i) cs.push_back(Contour::circle({(i%50)*1.3,(i/50)*1.3},0.5));
    char nm[128]; std::snprintf(nm,sizeof nm,"2000 circles r0.5 inset r%+g",dd); timeit(nm,[&]{ return offsetContours(cs,-(0.5+dd),JoinType::Round,T).size(); });
    // 2000 thin rectangles (hatch) 0.2 wide, inset 0.1
    std::vector<Contour> hs; for(int i=0;i<2000;++i){ double y=i*0.5; hs.push_back(poly({{0,y},{40,y},{40,y+0.2},{0,y+0.2}})); }
    std::snprintf(nm,sizeof nm,"2000 hatch strips w0.2 inset 0.1%+g",dd); timeit(nm,[&]{ return offsetContours(hs,-(0.1+dd),JoinType::Round,T).size(); });
    // wavy band, 40000 vertices, half-width 0.05
    std::vector<Vec2> top,bot; int N=20000; for(int i=0;i<N;++i){ double x=i*0.005; double y=0.3*std::sin(x*2); top.push_back({x,y+0.05}); bot.push_back({x,y-0.05}); }
    std::vector<Vec2> band(bot.begin(),bot.end()); for(int i=N-1;i>=0;--i) band.push_back(top[i]);
    std::snprintf(nm,sizeof nm,"wavy band 40k verts inset 0.05%+g",dd); timeit(nm,[&]{ return offsetContours({poly(band)},-(0.05+dd),JoinType::Round,T).size(); });
  }
}
