#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
int main(){ Tolerance tol; for(double o: {1e6,1e7,1e8,3e8,1e9}) for(double w: {40.0,5.0,1.0}){ int ign=0,emp=0,cap=0,N=0; for(int j=0;j<20;++j){ double ox=o*(1+0.037*j), oy=0.7*o*(1+0.051*j);
  std::vector<Contour> cs={vs::poly({{ox,oy},{ox+w,oy},{ox+w,oy+w/2},{ox,oy+w/2}})}; MedialAxis ax=medialAxis(cs,tol); ++N; ign+=ax.ignoredContours>0; emp+=ax.nodes.empty(); cap+=ax.accuracyCapped; }
  std::printf("o=%.0e w=%g: ignored %d/%d empty %d capped %d\n",o,w,ign,N,emp,cap);} }
