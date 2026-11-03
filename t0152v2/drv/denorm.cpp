#include <cstdio>
#include <cmath>
#include <random>
#include "shapes.hpp"
#include "delaunay.hpp"
#include "robust_predicates.hpp"
using namespace carvetto::geometry; using carvetto::geometry::detail::Delaunay;
int main(){ std::mt19937_64 rng(5); auto U=[&](double a,double b){return std::uniform_real_distribution<double>(a,b)(rng);};
 const double dn=5e-324;
 // 1. Delaunay with denormal/tiny/normal mixtures
 for(int trial=0;trial<40;++trial){ double big = trial%4==0?1.0: trial%4==1?1e-300: trial%4==2?1e-200:1e60;
  Delaunay dt(Box2{{-big*2,-big*2},{big*2,big*2}}); int n=0;
  for(int i=0;i<400;++i){ Vec2 p{U(-1,1)*big,U(-1,1)*big}; int k=i%5; if(k==1)p.x=dn*std::floor(U(-50,50)); if(k==2){p.y=dn*std::floor(U(-50,50));} if(k==3)p={dn*std::floor(U(-5,5)),dn*std::floor(U(-5,5))}; dt.insert(p); ++n; }
  int alive=0; for(size_t t=0;t<dt.triangleSlots();++t) alive+=dt.alive((int)t);
  std::printf("dt trial %d big=%g verts=%zu alive=%d\n",trial,big,dt.vertexCount(),alive); }
 // 2. medialAxis with denormal coordinate mixes and huge coords
 Tolerance tol;
 std::vector<std::vector<Vec2>> polys={ {{dn,0},{10,0},{10,5},{0,5}}, {{0,dn},{10,-dn},{10,5},{-dn,5}}, {{dn,dn},{3*dn,0},{2*dn,4*dn}}, {{0,0},{1e-300,0},{1e-300,1e-300},{0,1e-300}},
   {{1e75,1e75},{1e75*1.0000001,1e75},{1e75*1.0000001,1e75*1.0000001}}, {{-1e200,-1e200},{1e200,-1e200},{1e200,1e200},{-1e200,1e200}}, {{0,0},{1e200,0},{1e200,1},{0,1}} };
 for(auto& p: polys){ MedialAxis ax=medialAxis({vs::poly(p)},tol); std::printf("medial: nodes=%zu br=%zu smp=%zu capped=%d ign=%zu\n",ax.nodes.size(),ax.branches.size(),ax.boundarySamples,(int)ax.accuracyCapped,ax.ignoredContours); }
 // rectangle with one denormal-perturbed vertex mixed with mm scale (realistic garbage)
 for(int i=0;i<50;++i){ std::vector<Vec2> p={{0,0},{10,dn*i},{10,5},{dn*(i%7),5},{5,2.5+dn}}; MedialAxis ax=medialAxis({vs::poly(p)},tol); if(i%10==0) std::printf("mix %d: nodes=%zu capped=%d\n",i,ax.nodes.size(),(int)ax.accuracyCapped);}
 std::puts("done");
}
