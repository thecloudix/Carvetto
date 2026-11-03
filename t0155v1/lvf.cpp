#include <cstdio>
#include <cmath>
#include <random>
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto::toolpath;
static long bad=0;
static void chk(double d, int n){
  auto lv = engraveLevels(d,n);
  bool allowed = d/n >= kEngraveMinPassDepth;
  if (lv.empty()) { if (allowed) { ++bad; std::printf("allowed but empty d %.17g n %d\n", d, n);} return; }
  if (!allowed) { ++bad; std::printf("refused-range but got %zu d %.17g n %d\n", lv.size(), d, n); }
  if (lv.size()!=(size_t)n) { ++bad; std::printf("count %zu != %d d %.17g\n", lv.size(), n, d); }
  if (lv.back()!=-d) { ++bad; std::printf("last %.17g != %.17g\n", lv.back(), -d); }
  for (size_t i=1;i<lv.size();++i) if (!(lv[i]<lv[i-1])) { ++bad; std::printf("non-decreasing at %zu d %.17g n %d\n", i, d, n); break; }
}
int main(){
  std::mt19937_64 g(7); long tot=0, empt=0;
  for (int i=0;i<2000000;++i){ double d=std::pow(10.0,std::uniform_real_distribution<double>(-7,1.5)(g)); int n=std::uniform_int_distribution<int>(1,1000)(g); chk(d,n); ++tot; if (engraveLevels(d,n).empty()) ++empt; }
  // at the floor and one ulp either side, every n
  long floorOk=0, belowEmpty=0;
  for (int n=1;n<=1000;++n){
    double d = n*1e-6;  // may not be exactly n*1e-6 / n == 1e-6
    for (double x : {std::nextafter(d,0.0), d, std::nextafter(d,1.0), std::nextafter(std::nextafter(d,1.0),1.0)}) { chk(x,n); }
    // smallest double d with d/n >= 1e-6
    double lo = d; while (lo/n >= 1e-6) lo = std::nextafter(lo,0.0); double at = std::nextafter(lo,1.0);
    auto a = engraveLevels(at,n); auto b = engraveLevels(lo,n);
    if (a.size()==(size_t)n && a.back()==-at) ++floorOk; else std::printf("at-floor fail n %d d %.17g -> %zu\n", n, at, a.size());
    if (b.empty()) ++belowEmpty; else std::printf("below-floor not empty n %d\n", n);
    chk(at,n); chk(lo,n);
  }
  // Pass depth just above floor with big depth? n max 1000 so d>=1e-3 always fine.
  for (double d: {0.0,-1.0,(double)NAN,(double)INFINITY,1e-300}) chk(d,1);
  std::printf("random tot %ld empty %ld; floorOk %ld/1000 belowEmpty %ld/1000; bad %ld\n", tot, empt, floorOk, belowEmpty, bad);
}
