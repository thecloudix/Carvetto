#include "oracle.hpp"
using namespace v; using namespace carvetto::toolpath;
// T-0120 round-3 repro: Inside profile of a 100x50 rectangle, 6 mm end mill, 12 deep in 2 passes,
// separate last pass (0.1), 7 tabs 4 x 3: the finishing cut gets 7 tabs, the roughing cut only 6,
// with no diagnostic, so the roughing cuts through one finishing tab at full depth.
int main(){
  ProfileParams p; p.side=ProfileSide::Inside; p.passes.cutDepth=12; p.passes.passDepth=6;
  p.separateLastPass=true; p.lastPassAllowance=0.1; p.tabs.enabled=true; p.tabs.count=7; p.tabs.length=4; p.tabs.thickness=3;
  auto r=profileToolpath({Contour::rectangle(geometry::Box2{{0,0},{100,50}})},endMill(6),p);
  dump(r.toolpath); printf("diagnostics: %zu\n", r.diagnostics.size()); for(auto&d:r.diagnostics) printf("diag %s\n",d.message.c_str());
}
