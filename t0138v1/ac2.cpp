// AC2 adversarial: genuine thin/tiny regions near the dust threshold.
#include <cmath>
#include <cstdio>
#include <vector>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static const double G=1e-6; static const double PI=3.14159265358979323846;
static Contour poly(const std::vector<Vec2>& p){ Contour c; c.closed=true; for(size_t i=0;i<p.size();++i) c.spans.emplace_back(LineSpan{p[i],p[(i+1)%p.size()]}); return c; }
static Contour rect(Vec2 a,Vec2 b){ return poly({a,{b.x,a.y},b,{a.x,b.y}}); }
static double area(const Contour& c, Vec2 o){ double a=0; for(auto& s:c.spans){ Vec2 p=startPoint(s)-o,q=endPoint(s)-o; a+=p.cross(q);} return a/2; }
static double tot(const std::vector<Contour>& v, Vec2 o){ double a=0; for(auto& c:v) a+=area(c,o); return a; }
static const Tolerance F{1e-9,1e-9,0.01};
int main(){
  for(Vec2 at: {Vec2{0,0},Vec2{1e4,-1e4},Vec2{123456.7,-765432.1},Vec2{1e6,1e6}}){
    std::printf("== at (%g,%g)\n",at.x,at.y);
    // strips of width w units, rotated at angle th
    for(double th: {0.0,0.3,PI/4,1.1}){ Vec2 u=Vec2::fromAngle(th), n{-u.y,u.x};
      for(double w: {6.0,7.0,8.0,8.5,9.0,10.0,12.0}){ double L=50; double wm=w*G; auto c=poly({at,at+u*L,at+u*L+n*wm,at+n*wm});
        auto o=offsetContours({c},0.0,JoinType::Round,F); std::printf(" strip th=%.2f w=%4.1fu rings=%zu area/expected=%.3f\n",th,w,o.size(),o.empty()?0:std::fabs(tot(o,at))/(L*wm)); } }
    // tiny genuine disks of radius r units (polygon 64 gon), offset 0 and also produced by inset
    for(double r: {3.0,4.0,4.5,5.0,6.0,8.0,20.0}){ std::vector<Vec2> p; for(int i=0;i<64;++i) p.push_back(at+Vec2::fromAngle(2*PI*i/64,r*G)); auto o=offsetContours({poly(p)},0.0,JoinType::Round,F); std::printf(" disk r=%4.1fu rings=%zu\n",r,o.size()); }
    // inset of a circle leaving a tool-centre disk of radius e
    for(double e: {2e-6,4e-6,6e-6,1e-5,1e-4,5e-4}){ for(double r: {0.5,3.0,20.0}) { auto o=offsetContours({Contour::circle(at,r)},-(r-e),JoinType::Round,Tolerance{0.001,1e-9,0.0002}); std::printf(" circle r=%g leave e=%g rings=%zu",r,e,o.size()); } std::printf("\n"); }
    // square with long hair-thin tail (w=2 units, 6 units), and ring with hair inside a genuine outer
    for(double h: {1.0,2.0,6.0}){ double s=0.01; auto c=poly({at,at+Vec2{s,0},at+Vec2{s,s/2},at+Vec2{s+100,s/2},at+Vec2{s+100,s/2+h*G},at+Vec2{s,s/2+h*G},at+Vec2{s,s},at+Vec2{0,s}}); auto o=offsetContours({c},0.0,JoinType::Round,F); std::printf(" hairy h=%gu rings=%zu spans=%zu\n",h,o.size(),o.empty()?0:o[0].spans.size()); }
    // tiny square (10 units) whose area/perim = 2.5 < 4 -> candidate; inradius 5 -> must be kept
    for(double s: {7.0,8.0,9.0,10.0,12.0}){ auto o=offsetContours({rect(at,at+Vec2{s*G,s*G})},0.0,JoinType::Round,F); std::printf(" square %gu rings=%zu\n",s,o.size()); }
    // thin genuine hole (frame) and thin hole in big block: hole width w
    for(double w: {3.0,6.0,8.0,9.0,12.0}){ auto o=offsetContours({rect(at,at+Vec2{50,1}),rect(at+Vec2{1,0.5},at+Vec2{49,0.5+w*G}).reversed()},0.0,JoinType::Round,F); std::printf(" block+hole w=%gu rings=%zu\n",w,o.size()); }
    // genuine ring nested in genuine hole (island in hole), small island 20 units
    { auto o=offsetContours({rect(at,at+Vec2{10,10}),rect(at+Vec2{2,2},at+Vec2{8,8}).reversed(),rect(at+Vec2{5,5},at+Vec2{5+20*G,5+20*G})},0.0,JoinType::Round,F); std::printf(" island20u rings=%zu\n",o.size()); }
    { auto o=offsetContours({rect(at,at+Vec2{10,10}),rect(at+Vec2{2,2},at+Vec2{8,8}).reversed(),rect(at+Vec2{5,5},at+Vec2{5+5*G,5+5*G})},0.0,JoinType::Round,F); std::printf(" island5u rings=%zu (5u square: dust)\n",o.size()); }
    // thin frame region between outer and hole
    for(double t: {1.0,3.0,6.0}){ auto o=offsetContours({rect(at,at+Vec2{0.1,0.1}),rect(at+Vec2{t*G,t*G},at+Vec2{0.1-t*G,0.1-t*G}).reversed()},0.0,JoinType::Round,F); std::printf(" frame t=%gu rings=%zu\n",t,o.size()); }
    // inset of rectangle 2a wide leaving strip of width 2e
    for(double e: {3e-6,5e-6,1e-5}){ double a=1.0; auto o=offsetContours({rect(at,at+Vec2{30,2*a})},-(a-e),JoinType::Round,Tolerance{0.001,1e-9,0.001}); std::printf(" rect inset leaving 2e=%gu rings=%zu\n",2*e/G,o.size()); }
  }
}
