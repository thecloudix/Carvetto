// mkscn out.scn side(0 out,1 in,2 on) D allowance sep L n [shape: 0 square30, 1 square30+circle] [lpa] [3d]
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main(int argc, char** argv) {
    ProfileParams p; p.side = (ProfileSide)std::atoi(argv[2]); double D = std::atof(argv[3]); p.allowance = std::atof(argv[4]);
    p.passes.cutDepth = 8; p.passes.passDepth = 3; p.tabs.enabled = true; p.separateLastPass = std::atoi(argv[5]); p.tabs.length = std::atof(argv[6]);
    p.tabs.count = std::atoi(argv[7]); p.tabs.thickness = 1.5; p.lastPassAllowance = argc > 9 ? std::atof(argv[9]) : 0.3; p.tabs.threeD = argc > 10 && std::atoi(argv[10]);
    p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomLeft;
    std::vector<geometry::Contour> cs{geometry::Contour::rectangle(geometry::Box2{{0, 0}, {100, 60}})};
    if (argc > 8 && std::atoi(argv[8]) == 1) cs.push_back(geometry::Contour::circle({60, 15}, 10));
    writeScene(argv[1], cs, p, D, 0);
}
