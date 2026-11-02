// mkabc out.scn count anchor side(2 On / 0 Outside with a=-r) D : three pieces A-B-C joined by 3 mm necks; C sits in B's inner corner (bbox(C) inside bbox(B))
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main(int argc, char** argv) {
    std::vector<Vec2> P{{0,0},{40,0},{40,18.5},{50,18.5},{50,0},{90,0},{90,12},{80.5,12},{80.5,22},{86,22},{86,36},{72,36},{72,22},{77.5,22},{77.5,12},{62,12},{62,40},{50,40},{50,21.5},{40,21.5},{40,40},{0,40}};
    geometry::Contour c; c.closed = true; for (size_t i = 0; i < P.size(); ++i) c.append(geometry::LineSpan{P[i], P[(i + 1) % P.size()]});
    ProfileParams p; p.side = (ProfileSide)std::atoi(argv[4]); double D = std::atof(argv[5]); p.allowance = p.side == ProfileSide::On ? 0.0 : -D / 2;
    p.passes.cutDepth = 6; p.passes.passDepth = 3; p.tabs.enabled = true; p.tabs.count = std::atoi(argv[2]); p.tabs.length = 4; p.tabs.thickness = 1.5;
    p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = (BoxAnchor)std::atoi(argv[3]);
    writeScene(argv[1], {c}, p, D, 0);
}
