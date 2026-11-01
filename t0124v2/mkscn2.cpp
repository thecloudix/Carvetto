#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main(int argc, char** argv){
  ProfileParams p; p.side=ProfileSide::Inside; p.passes.cutDepth=6; p.passes.passDepth=3;
  p.tabs.enabled=true; p.tabs.count=4; p.tabs.length=std::atof(argv[1]); p.tabs.thickness=1.5; p.separateLastPass=std::atoi(argv[2]); p.lastPassAllowance=0.3;
  p.startAt=StartAt::ClosestOnBoundingBox; p.startAnchor=BoxAnchor::BottomCentre;
  writeScene(argv[3], {geometry::Contour::rectangle(geometry::Box2{{0,0},{30,30}})}, p, 6.0);
}
