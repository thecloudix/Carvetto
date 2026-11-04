#include <cstdio>
#include <random>
#include <algorithm>
#include "shapes.hpp"
#include "carvetto/geometry/transform2.hpp"
using namespace carvetto::geometry;
struct R { size_t ign, nodes, br; bool capped; double rmax, rerr; std::vector<double> nr; };
static R run(const std::vector<Contour>& cs0, double s, double rot, double o, const Tolerance& tol, FillRule fr){
  std::vector<Contour> cs=cs0; Transform2 T=Transform2::rotation(rot); for(auto&c:cs){ c.transform(Transform2{s,0,0,s,0,0}); c.transform(T);}
  std::vector<Contour> loc=cs; Vec2 off{o,0.7*o}; for(auto&c:cs) c.transform(Transform2::translation(off));
  MedialAxisOptions opt; opt.fillRule=fr; MedialAxis ax=medialAxis(cs,tol,opt);
  R r{ax.ignoredContours,ax.nodes.size(),ax.branches.size(),ax.accuracyCapped,0,0,{}};
  vo::Boundary B=vo::makeBoundary(loc,tol.chordal/8);
  auto chk=[&](Vec2 p,double rad){ r.rmax=std::max(r.rmax,rad); Vec2 q{p.x-off.x,p.y-off.y}; double d=vo::exactDist(B,q).d; r.rerr=std::max(r.rerr,std::fabs(d-rad)); };
  for(auto&n:ax.nodes){chk(n.point,n.radius); r.nr.push_back(n.radius);} for(auto&b:ax.branches) for(auto&p:b.points) chk(p.point,p.radius);
  std::sort(r.nr.begin(),r.nr.end()); return r; }
int main(int argc,char**argv){ Tolerance tol; std::mt19937 rng(argc>1?atoi(argv[1]):1); std::uniform_real_distribution<double> U(0,1);
  std::vector<vs::Shape> sh=vs::all();
  // random star-shaped polygons
  for(int k=0;k<6;++k){ int n=5+int(U(rng)*20); std::vector<double> a; for(int i=0;i<n;++i)a.push_back(U(rng)*2*vo::PI); std::sort(a.begin(),a.end()); std::vector<Vec2> p; for(double t:a){double rr=3+7*U(rng); p.push_back({rr*std::cos(t),rr*std::sin(t)});}
    Contour c=vs::poly(p); if(U(rng)<0.5) c=vs::rev(c); sh.push_back({"rand"+std::to_string(k),{c}}); }
  // holes, mixed orientations
  sh.push_back({"rect_hole_same_orient",{vs::poly({{0,0},{10,0},{10,6},{0,6}}),vs::poly({{3,2},{7,2},{7,4},{3,4}})}});
  sh.push_back({"rect_hole_cw_outer",{vs::rev(vs::poly({{0,0},{10,0},{10,6},{0,6}})),vs::poly({{3,2},{7,2},{7,4},{3,4}})}});
  sh.push_back({"circle_bezhole_island",{vs::circle({0,0},10),vs::rev(vs::bezEllipse({0,0},7,5)),vs::circle({0,0},2)}});
  sh.push_back({"two_disjoint",{vs::poly({{0,0},{4,0},{4,2},{0,2}}),vs::rev(vs::circle({8,1},1))}});
  int bad=0;
  for(FillRule fr:{FillRule::EvenOdd,FillRule::NonZero})
  for(auto& S: sh) for(double s:{1.0,0.05,0.005}) { if(s*20<4*tol.chordal) continue; double rot=U(rng)*6.283;
    R base=run(S.cs,s,rot,0,tol,fr);
    for(double o:{1e3,1e5,1e7,1e8,3e8,1e9,3e9,4.9e9,6e9,1e10}){ R r=run(S.cs,s,rot,o,tol,fr);
      bool expectCap = 1e-12*(o+20*s+1)>tol.chordal/2; // approx
      bool ok = r.ign==base.ign && (r.nodes==0)==(base.nodes==0) && std::fabs(r.rmax-base.rmax)<=tol.chordal && r.rerr<=tol.chordal*(expectCap?50:1.0);
      bool topo = r.nodes==base.nodes && r.br==base.br;
      if(!ok || (!topo && o<=1e9) || (r.capped && o<4.9e9) || (!r.capped && o>=6e9)){ ++bad;
        std::printf("%s fr=%d %s s=%g o=%g: ign %zu/%zu nodes %zu/%zu br %zu/%zu rmax %.5f/%.5f rerr %.2e capped %d\n", ok?"TOPO":"BAD ",(int)fr,S.name.c_str(),s,o,r.ign,base.ign,r.nodes,base.nodes,r.br,base.br,r.rmax,base.rmax,r.rerr,(int)r.capped);} }
  }
  std::printf("done bad=%d shapes=%zu\n",bad,sh.size()); }
