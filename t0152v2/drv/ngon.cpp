#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
int main(){ Tolerance tol; for(int n: {6,12,36,60,180}){ double R=10; std::vector<Vec2> p; for(int i=0;i<n;++i){double a=2*vo::PI*i/n; p.push_back({R*std::cos(a),R*std::sin(a)});}
 MedialAxis ax=medialAxis({vs::poly(p)},tol); double worst=0; int ends=0; for(auto&nd:ax.nodes) if(nd.branches.size()==1){++ends; double best=1e9; for(auto&v:p) best=std::min(best,vo::dist(v,nd.point)); worst=std::max(worst,best);} 
 // fraction of vertices reached within 3 chordal + coverage depth
 vo::Boundary B=vo::makeBoundary({vs::poly(p)},tol.chordal/4); double cov=0; for(auto&v:p){ Vec2 q=v*0.9999; double best=1e9; for(auto&b:ax.branches) for(auto&m:b.points) best=std::min(best,vo::dist(q,m.point)-m.radius); for(auto&nd:ax.nodes) best=std::min(best,vo::dist(q,nd.point)-nd.radius); cov=std::max(cov,best);} 
 std::printf("n=%3d angle=%.0f endnodes=%d worstTipToVertex=%.4f (%.1f x chordal) nearVertexUncovered=%.4f (%.2f x chordal)\n",n,180-360.0/n,ends,worst,worst/tol.chordal,cov,cov/tol.chordal);} }
