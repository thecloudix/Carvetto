#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
// polygon vertex angle sweep with varying size/rotation; report branch shortfall per corner and coverage
int main(){ Tolerance tol; for(int n: {6,12,36,60,90,180}) for(double R:{10.0,25.0,3.0}) for(double rot:{0.0,0.37}){ std::vector<Vec2> p; for(int i=0;i<n;++i){double a=rot+2*vo::PI*i/n; p.push_back({R*std::cos(a),R*std::sin(a)});}
 MedialAxis ax=medialAxis({vs::poly(p)},tol); double worst=0,best0=1e9; int ends=0; for(auto&nd:ax.nodes) if(nd.branches.size()==1){++ends;}
 for(auto&v:p){ double best=1e9; for(auto&b:ax.branches) for(auto&m:b.points) best=std::min(best,vo::dist(v,m.point)); for(auto&nd:ax.nodes) best=std::min(best,vo::dist(v,nd.point)); worst=std::max(worst,best); best0=std::min(best0,best);} 
 vo::Boundary B=vo::makeBoundary({vs::poly(p)},tol.chordal/8); double cov=0; 
 // coverage: sample interior points near boundary, check union of inscribed disks reaches within cov of boundary
 for(int i=0;i<n;++i) for(int k=0;k<=20;++k){ Vec2 a=p[i],b=p[(i+1)%n]; Vec2 q{a.x+(b.x-a.x)*k/20.0,a.y+(b.y-a.y)*k/20.0}; double best=1e9; for(auto&br:ax.branches) for(auto&m:br.points) best=std::min(best,vo::dist(q,m.point)-m.radius); for(auto&nd:ax.nodes) best=std::min(best,vo::dist(q,nd.point)-nd.radius); cov=std::max(cov,best);} 
 std::printf("angle=%5.1f R=%4.0f rot=%.2f ends=%3d tip-to-vertex min %.1f max %.1f x chordal  uncovered %.3f x chordal\n",180-360.0/n,R,rot,ends,best0/tol.chordal,worst/tol.chordal,cov/tol.chordal);} }
