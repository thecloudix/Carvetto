#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
int main(){ for(double ch: {0.01, 0.002, 0.1}){ Tolerance tol; tol.chordal=ch; double lim=0.5*ch/1e-12; for(double d: {-1024.0, 1024.0}){ double o=lim-40+d;
 std::vector<Contour> cs={vs::poly({{o,0},{o+40,0},{o+40,20},{o,20}})}; MedialAxis ax=medialAxis(cs,tol);
 std::printf("chordal=%g scale=%.6e (limit %.6e) capped=%d ign=%zu nodes=%zu\n",ch,o+40,lim,(int)ax.accuracyCapped,ax.ignoredContours,ax.nodes.size()); } } }
