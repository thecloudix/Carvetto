#include <chrono>
#include <cstdio>
#include <cstdlib>
#include "shapes.hpp"
using namespace carvetto::geometry;
static unsigned long long h=1469598103934665603ULL; static void H(const void* p,size_t n){auto c=(const unsigned char*)p;for(size_t i=0;i<n;++i){h^=c[i];h*=1099511628211ULL;}}
static void HD(double d){H(&d,8);} static void HZ(size_t z){H(&z,8);}
int main(int argc,char**argv){ double chordal=argc>1?atof(argv[1]):0.01; double off=argc>2?atof(argv[2]):0; double sc=argc>3?atof(argv[3]):1;
 Tolerance tol; tol.chordal=chordal;
 for(auto& sh: vs::all()){ auto cs=sh.cs; for(auto&c:cs){ c.transform(Transform2{sc,0,0,sc,0,0}); c.transform(Transform2::translation({off,off*0.7})); }
  h=1469598103934665603ULL; auto t0=std::chrono::steady_clock::now(); MedialAxis ax=medialAxis(cs,tol); double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
  for(auto&n:ax.nodes){HD(n.point.x);HD(n.point.y);HD(n.radius);for(auto b:n.branches)HZ(b);}
  for(auto&b:ax.branches){HZ(b.startNode);HZ(b.endNode);for(auto&p:b.points){HD(p.point.x);HD(p.point.y);HD(p.radius);} for(auto&s:b.sides)for(auto&r:s){HZ(r.contour);HZ(r.span);HD(r.t);}}
  std::printf("%-20s hash=%016llx nodes=%zu br=%zu smp=%zu capped=%d t=%.3f\n",sh.name.c_str(),h,ax.nodes.size(),ax.branches.size(),ax.boundarySamples,(int)ax.accuracyCapped,t);}
}
