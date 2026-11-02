// mkh out.scn sep lpa D startAnchor ramp(0/1) : the H shape of 401007_45, On, a=0, 2 tabs 0.68 long, no leads
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main(int argc, char** argv) {
    std::vector<Vec2> P{{-7.552,-7.552},{-2.225,-7.552},{-2.225,-1.153},{5.246,-1.153},{5.246,-7.552},{10.573,-7.552},{10.573,10.573},{5.246,10.573},{5.246,4.174},{-2.225,4.174},{-2.225,10.573},{-7.552,10.573}};
    geometry::Contour c; c.closed = true; for (size_t i = 0; i < P.size(); ++i) c.append(geometry::LineSpan{P[i], P[(i + 1) % P.size()]});
    ProfileParams p; p.side = ProfileSide::On; double D = std::atof(argv[4]);
    p.passes.cutDepth = 14.4; p.passes.passDepth = 1.2; p.tabs.enabled = true; p.tabs.count = 2; p.tabs.length = 0.68; p.tabs.thickness = 4.1;
    p.separateLastPass = std::atoi(argv[2]); p.lastPassAllowance = std::atof(argv[3]);
    p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = (BoxAnchor)std::atoi(argv[5]);
    p.leads.type = LeadType::None; p.ramp.type = std::atoi(argv[6]) ? RampType::Smooth : RampType::None; p.ramp.distance = 5;
    writeScene(argv[1], {c}, p, D, 0);
}
