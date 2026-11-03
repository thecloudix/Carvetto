#include <cmath>
#include <cstdio>
#include <random>
#include "robust_predicates.hpp"
using carvetto::geometry::Vec2; using namespace carvetto::geometry::detail;
static std::mt19937_64 rng(7); static double U(double a,double b){return std::uniform_real_distribution<double>(a,b)(rng);} static int I(int a,int b){return std::uniform_int_distribution<int>(a,b)(rng);}
static double nudge(double x,int k){for(int i=0;i<std::abs(k);++i)x=std::nextafter(x,k>0?INFINITY:-INFINITY);return x;}
static void O(const char*t,Vec2 a,Vec2 b,Vec2 c){std::printf("O%s %a %a %a %a %a %a %d\n",t,a.x,a.y,b.x,b.y,c.x,c.y,orient2dSign(a,b,c));}
static void C(const char*t,Vec2 a,Vec2 b,Vec2 c,Vec2 d){std::printf("C%s %a %a %a %a %a %a %a %a %d\n",t,a.x,a.y,b.x,b.y,c.x,c.y,d.x,d.y,incircleSign(a,b,c,d));}
int main(){
 for(int i=0;i<6000;++i){ double M=std::pow(10.0,I(-250,69)); double mn=1e-68*M; // claimed-in-range: coords in [1e-68 M, M] or 0, near-degenerate
   auto co=[&]{ int k=I(0,9); if(k==0) return 0.0; double v=std::pow(10.0,U(std::log10(mn),std::log10(M))); return I(0,1)?v:-v; };
   Vec2 a{co(),co()}, b{co(),co()}; double t=U(-1,2); Vec2 c{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)}; c.x=nudge(c.x,I(-2,2));
   O("in",a,b,c);
   // near-cocircular: axis circle with centre and radius
   double r=std::pow(10.0,U(std::log10(mn)+2,std::log10(M))); Vec2 o{co()*1e-3,co()*1e-3};
   C("in",{o.x+r,o.y},{o.x,o.y+r},{o.x-r,o.y},{nudge(o.x,I(-1,1)),nudge(o.y-r,I(-1,1))});
   Vec2 d{co(),co()}; C("in",a,b,{co(),co()},d);
 }
 // single tiny coordinate ~1e-150..1e-200 M, large differences
 for(int i=0;i<6000;++i){ double tiny=std::pow(10.0,U(-200,-140)); 
   double r=1+I(0,100); Vec2 o{tiny*(I(0,1)?1:-1),0};
   C("tiny",{o.x+r,o.y},{o.x,o.y+r},{o.x-r,o.y},{nudge(o.x,I(-1,1)),nudge(o.y-r,I(-1,1))});
   Vec2 a{U(-1,1),U(-1,1)}, b{U(-1,1),U(-1,1)}; double t=U(-1,2); Vec2 c{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)}; a.x+=0; Vec2 a2{a.x, a.y}; 
   O("tiny",a2,b,{c.x+tiny*(I(0,1)?1:-1),c.y});
   C("tiny",{1,tiny},{-1,0},{0,1},{0,nudge(-1.0,I(-1,1))});
   C("tiny",{1,0},{-1,0},{0,1},{tiny,nudge(-1.0,I(-1,1))});
 }
}
