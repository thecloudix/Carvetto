#include <cmath>
#include <cstdio>
#include <random>
#include "robust_predicates.hpp"
using carvetto::geometry::Vec2; using namespace carvetto::geometry::detail;
static std::mt19937_64 rng(777);
static double U(double a,double b){return std::uniform_real_distribution<double>(a,b)(rng);}
static int I(int a,int b){return std::uniform_int_distribution<int>(a,b)(rng);}
static double nud(double x,int k){for(int i=0;i<std::abs(k);++i)x=std::nextafter(x,k>0?INFINITY:-INFINITY);return x;}
static void O(Vec2 a,Vec2 b,Vec2 c){std::printf("O %a %a %a %a %a %a %d\n",a.x,a.y,b.x,b.y,c.x,c.y,orient2dSign(a,b,c));}
static void C(Vec2 a,Vec2 b,Vec2 c,Vec2 d){std::printf("C %a %a %a %a %a %a %a %a %d\n",a.x,a.y,b.x,b.y,c.x,c.y,d.x,d.y,incircleSign(a,b,c,d));}
int main(int argc,char**argv){ int n=argc>1?atoi(argv[1]):20000; int lo=argc>2?atoi(argv[2]):-70; int hi=argc>3?atoi(argv[3]):70; int rel=argc>4?atoi(argv[4]):70;
 static const int tr[][3]={{3,4,5},{5,12,13},{8,15,17},{7,24,25},{20,21,29},{12,35,37}};
 for(int i=0;i<n;++i){ double M=std::pow(10.0,I(lo,hi)); double s=M*std::pow(10.0,-I(0,rel));
  auto cp=[&]{auto&t=tr[I(0,5)];double r=s/t[2];bool sw=I(0,1);return Vec2{(sw?t[0]:t[1])*r*(I(0,1)?1:-1),(sw?t[1]:t[0])*r*(I(0,1)?1:-1)};};
  auto nz=[&](Vec2 p){return Vec2{nud(p.x,I(-2,2)),nud(p.y,I(-2,2))};};
  Vec2 far{U(-1,1)*M,U(-1,1)*M};
  // cocircular cluster at origin (circle radius s), plus far point variants
  C(nz(cp()),nz(cp()),nz(cp()),nz(cp())); C(cp(),cp(),cp(),cp());
  C(cp(),cp(),cp(),far);
  // collinear with far point: a=0-ish, b=far, c=t*far nudged
  double t=U(-2,2); O(nz(Vec2{0,0}),far,nz(Vec2{t*far.x,t*far.y})); O(cp(),far,Vec2{nud(-far.x,I(-2,2)),nud(-far.y,I(-2,2))});
  O(cp(),cp(),Vec2{s*U(-1,1),s*U(-1,1)}); O(nz(cp()),Vec2{0,0},far);
  // tiny triangle near far point offset by sub-ulp? use far + multiple of ulp
  Vec2 a=far,b{nud(far.x,I(1,3)),nud(far.y,I(-3,3))},c{nud(far.x,I(-3,3)),nud(far.y,I(1,3))},d{nud(far.x,I(-3,3)),nud(far.y,I(-3,3))};
  O(a,b,c); C(a,b,c,d);
 }}
