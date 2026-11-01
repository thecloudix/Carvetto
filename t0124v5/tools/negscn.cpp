#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
    for (int side : {0, 1}) for (double D : {3.175, 6.0}) for (int dai : {0, 1, 2}) for (int sep : {0, 1}) for (double L : {1.0, 5.0}) {
        double da = dai == 0 ? 0.0 : dai == 1 ? 1e-9 : -1e-9;
        ProfileParams p; p.side = side ? ProfileSide::Inside : ProfileSide::Outside; p.allowance = -D / 2 + da; p.passes.cutDepth = 6; p.passes.passDepth = 3;
        p.tabs.enabled = true; p.tabs.count = 4; p.tabs.length = L; p.tabs.thickness = 1.5; p.separateLastPass = sep; p.lastPassAllowance = 0.3;
        p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomCentre;
        std::vector<geometry::Contour> cs{geometry::Contour::rectangle(geometry::Box2{{0, 0}, {30, 30}})};
        char name[128]; std::snprintf(name, sizeof name, "scn/neg_s%d_D%g_da%d_sep%d_L%g.scn", side, D, dai, sep, L);
        writeScene(name, cs, p, D, 0);
    }
}
