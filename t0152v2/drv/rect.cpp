#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
int main(){ Tolerance tol; for(double o: {5e9, 6e9, 1e10, 3e10, 1e11, 5e11, 1e12, 1e13}){ for(int k=0;k<3;++k){ std::vector<Contour> cs;
  if(k==0) cs={vs::poly({{o,0.7*o},{o+40,0.7*o},{o+40,0.7*o+20},{o,0.7*o+20}})};
  if(k==1) cs={vs::poly({{o,o},{o+40,o},{o+40,o+20},{o,o+20}})};
  if(k==2) cs={vs::poly({{o,0},{o+40,0},{o+40,20},{o,20}})};
  MedialAxis ax=medialAxis(cs,tol); std::printf("o=%.0e k=%d capped=%d ignored=%zu samples=%zu nodes=%zu\n",o,k,(int)ax.accuracyCapped,ax.ignoredContours,ax.boundarySamples,ax.nodes.size()); }}}
