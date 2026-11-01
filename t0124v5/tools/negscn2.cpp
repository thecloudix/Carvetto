#include <cstdio>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
    for (int side : {0, 1}) for (double da : {0.01, 0.1, 0.5, 1.0, 2.0}) {
        ProfileParams p; p.side = side ? ProfileSide::Inside : ProfileSide::Outside; p.allowance = -3.0 + da; p.passes.cutDepth = 6; p.passes.passDepth = 3;
        p.tabs.enabled = true; p.tabs.count = 4; p.tabs.length = 1.0; p.tabs.thickness = 1.5;
        p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomCentre;
        std::vector<geometry::Contour> cs{geometry::Contour::rectangle(geometry::Box2{{0, 0}, {30, 30}})};
        char name[128]; std::snprintf(name, sizeof name, "scn/neg2_s%d_da%g.scn", side, da);
        writeScene(name, cs, p, 6.0, 0);
    }
}
