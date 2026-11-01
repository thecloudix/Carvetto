#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main(){
  ProfileParams p; p.side=ProfileSide::Inside; p.passes.cutDepth=12; p.passes.passDepth=6;
  p.separateLastPass=true; p.lastPassAllowance=0.1; p.tabs.enabled=true; p.tabs.count=7; p.tabs.length=4; p.tabs.thickness=3;
  writeScene("tabmin1.scn", {geometry::Contour::rectangle(geometry::Box2{{0,0},{100,50}})}, p, 6.0);
}
