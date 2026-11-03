#include <chrono>
#include <cstdio>
#include <cmath>
#include "robust_predicates.hpp"
using carvetto::geometry::Vec2; using namespace carvetto::geometry::detail;
int main(){ // exact cocircular points (fall back to exact), and random (filter)
 volatile long acc=0; Vec2 a{0.6,0.8},b{-0.8,0.6},c{-0.6,-0.8},d{0.8,-0.6};
 auto t0=std::chrono::steady_clock::now();
 for(int i=0;i<300000;++i){ double s=std::ldexp(1.0,i%40-20)*(1+(i%7)); acc+=incircleSign({a.x*s+1,a.y*s},{b.x*s+1,b.y*s},{c.x*s+1,c.y*s},{d.x*s+1,d.y*s}); acc+=orient2dSign({0.1*s,0.1*s},{0.3*s,0.3*s},{0.7*s,0.7*s}); }
 auto t1=std::chrono::steady_clock::now();
 for(int i=0;i<3000000;++i){ double s=1+i*1e-7; acc+=incircleSign({s,0.1},{0.2,s},{-s,0.3},{0.1,0.2}); }
 auto t2=std::chrono::steady_clock::now();
 std::printf("exact-path %.1f ns/call  filter %.1f ns/call\n", std::chrono::duration<double,std::nano>(t1-t0).count()/600000, std::chrono::duration<double,std::nano>(t2-t1).count()/3e6); }
