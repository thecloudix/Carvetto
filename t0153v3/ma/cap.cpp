#include <cstdio>
#include <limits>
#include "shapes.hpp"
using namespace carvetto::geometry;
static bool capAt(double o,double ch,double w){ Tolerance t; t.chordal=ch; return medialAxis({vs::poly({{o,0},{o+w,0},{o+w,w/2},{o,w/2}})},t).accuracyCapped; }
static void search(double ch,double w){ double lo=1e6,hi=1e12; if(capAt(lo,ch,w)||!capAt(hi,ch,w)){std::printf("bad bracket\n");return;} for(int i=0;i<80;++i){double m=std::sqrt(lo*hi); if(hi-lo<1) break; if(capAt(m,ch,w))hi=m; else lo=m;} 
  for(int i=0;i<60;++i){double m=0.5*(lo+hi); if(capAt(m,ch,w))hi=m; else lo=m;}
  std::printf("chordal=%g w=%g: flag starts at maxcoord %.6e (expected %.6e)\n",ch,w,hi+w,ch/2/1e-12); }
int main(){ search(0.01,20); search(0.01,0.1); search(0.1,20); search(0.001,1);
 Tolerance t; double far=1e11; double nan=std::numeric_limits<double>::quiet_NaN();
 auto pr=[&](const char*n,std::vector<Contour> cs,Tolerance tt){ MedialAxis a=medialAxis(cs,tt); std::printf("%-34s nodes=%zu ign=%zu capped=%d\n",n,a.nodes.size(),a.ignoredContours,(int)a.accuracyCapped); };
 pr("collinear 1e11",{vs::poly({{far,far},{far+5,far},{far+10,far}})},t);
 pr("collinear 6e9",{vs::poly({{6e9,0},{6e9+5,0},{6e9+10,0}})},t);
 pr("collinear 4e9",{vs::poly({{4e9,0},{4e9+5,0},{4e9+10,0}})},t);
 pr("bowtie 1e11",{vs::poly({{far,far},{far+10,far+10},{far+10,far},{far,far+10}})},t);
 pr("bowtie 1e9",{vs::poly({{1e9,1e9},{1e9+10,1e9+10},{1e9+10,1e9},{1e9,1e9+10}})},t);
 pr("narrow rect (0.005 wide) 1e11",{vs::poly({{far,0},{far+10,0},{far+10,0.005},{far,0.005}})},t);
 pr("narrow rect (0.005 wide) 1e9",{vs::poly({{1e9,0},{1e9+10,0},{1e9+10,0.005},{1e9,0.005}})},t);
 pr("tiny circle r=0.004 at 1e11",{vs::circle({far,far},0.004)},t);
 pr("tiny circle r=0.004 at 1e9",{vs::circle({1e9,1e9},0.004)},t);
 pr("dup contour 1e11 (annul. self)",{vs::poly({{far,0},{far+10,0},{far+10,5},{far,5}}),vs::rev(vs::poly({{far,0},{far+10,0},{far+10,5},{far,5}}))},t);
 Contour open=vs::poly({{far,0},{far+10,0},{far+10,5},{far,5}}); open.closed=false; pr("open contour 1e11",{open},t);
 pr("zero-length spans 1e11",{vs::poly({{far,far},{far,far},{far,far}})},t);
 pr("NaN contour + nothing",{vs::poly({{nan,0},{1,0},{1,1}})},t);
 pr("empty input",{},t);
 pr("negative far -1e11 collinear",{vs::poly({{-far,0},{-far+5,0},{-far+10,0}})},t);
 pr("rect near origin + bowtie at 1e11",{vs::poly({{0,0},{10,0},{10,5},{0,5}}),vs::poly({{far,far},{far+10,far+10},{far+10,far},{far,far+10}})},t);
 Tolerance c; c.chordal=0.1; pr("collinear 1e11 chordal .1 (lim 5e10)",{vs::poly({{far,far},{far+5,far},{far+10,far}})},c);
 pr("collinear 4e10 chordal .1",{vs::poly({{4e10,0},{4e10+5,0},{4e10+10,0}})},c);
 pr("huge size: rect 0..6e9",{vs::poly({{0,0},{6e9,0},{6e9,3e9},{0,3e9}})},c.chordal==0.1?t:t);
}
