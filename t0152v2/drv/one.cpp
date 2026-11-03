#include <cstdio>
#include "shapes.hpp"
#include "carvetto/geometry/contour_ops.hpp"
using namespace carvetto::geometry;
int main(){ Tolerance tol; double ox=1e8*(1+0.037), oy=0.7e8*(1+0.051), w=1;
 std::vector<Contour> cs={vs::poly({{ox,oy},{ox+w,oy},{ox+w,oy+w/2},{ox,oy+w/2}})}; MedialAxis ax=medialAxis(cs,tol);
 std::printf("ox=%.17g oy=%.17g signedArea=%g ignored=%zu nodes=%zu capped=%d\n",ox,oy,signedArea(cs[0]),ax.ignoredContours,ax.nodes.size(),(int)ax.accuracyCapped);}
