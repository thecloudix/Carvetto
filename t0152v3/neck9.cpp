#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
int main(){ Tolerance tol; auto S=vs::all(); for(auto& sh:S){ if(sh.name.find("neck")==std::string::npos) continue;
 for(double o:{0.0,1e8,3e8,1e9,2e9,3e9}){ auto cs=sh.cs; for(auto&c:cs)c.transform(Transform2::translation({o,0.7*o})); MedialAxis ax=medialAxis(cs,tol);
  double minNeck=1e9; int inNeck=0; double xmin=1e9,xmax=-1e9; for(auto&b:ax.branches) for(auto&p:b.points){ double x=p.point.x-o; if(x>5.2&&x<6.8){++inNeck; minNeck=std::min(minNeck,p.radius);} }
  std::printf("%-16s o=%g nodes=%zu br=%zu neckPts=%d neckMinR=%.5f\n",sh.name.c_str(),o,ax.nodes.size(),ax.branches.size(),inNeck,minNeck);
  if(o==1e9||o==0) for(auto&n:ax.nodes) std::printf("   node %.4f %.4f r=%.4f deg=%zu\n",n.point.x-o,n.point.y-0.7*o,n.radius,n.branches.size()); }}}
