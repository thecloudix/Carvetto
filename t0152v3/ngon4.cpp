#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
// polygon vertex angle sweep with varying size/rotation; report branch shortfall per corner and coverage
int main(){ Tolerance tol; for(int n: {6,12,36,60,90,180}) for(double R:{10.0,25.0,3.0}) for(double rot:{0.0,0.37}){ std::vector<Vec2> p; for(int i=0;i<n;++i){double a=rot+2*vo::PI*i/n; p.push_back({R*std::cos(a),R*std::sin(a)});}
 MedialAxis ax=medialAxis({vs::poly(p)},tol); double worst=0,best0=1e9; int ends=0; for(auto&nd:ax.nodes) if(nd.branches.size()==1){++ends;}
 for(auto&v:p){ double best=1e9; for(auto&b:ax.branches) for(auto&m:b.points) best=std::min(best,vo::dist(v,m.point)); for(auto&nd:ax.nodes) best=std::min(best,vo::dist(v,nd.point)); worst=std::max(worst,best); best0=std::min(best0,best);} 
 std::vector<Contour> PC{vs::poly(p)}; vo::Boundary B=vo::makeBoundary(PC,tol.chordal/8); double cov=0; 
 // coverage: sample interior points near boundary, check union of inscribed disks reaches within cov of boundary
 double covV=0; std::vector<std::pair<Vec2,double>> D; for(auto&br:ax.branches) for(size_t j=0;j+1<br.points.size();++j) for(int u=0;u<=10;++u){ auto&A=br.points[j]; auto&Bp=br.points[j+1]; double t=u/10.0; D.push_back({Vec2{A.point.x+(Bp.point.x-A.point.x)*t,A.point.y+(Bp.point.y-A.point.y)*t},A.radius+(Bp.radius-A.radius)*t}); } for(auto&nd:ax.nodes) D.push_back({nd.point,nd.radius});
 for(auto&d:D) d.second=vo::exactDist(B,d.first).d;
 for(int i=0;i<n;++i) for(int k=0;k<=20;++k){ Vec2 a=p[i],b=p[(i+1)%n]; Vec2 q{a.x+(b.x-a.x)*k/20.0,a.y+(b.y-a.y)*k/20.0}; double best=1e9; for(auto&d:D) best=std::min(best,vo::dist(q,d.first)-d.second); cov=std::max(cov,best); if(k==0) covV=std::max(covV,best);} 
 std::printf("angle=%5.1f R=%4.0f rot=%.2f ends=%3d tip-to-vertex min %.1f max %.1f x chordal  uncovered edges %.3f vertices %.3f x chordal\n",180-360.0/n,R,rot,ends,best0/tol.chordal,worst/tol.chordal,cov/tol.chordal,covV/tol.chordal);} }
