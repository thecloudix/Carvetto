// writes the perf2 kind-2 scene with n vectors to OUT
#include <cmath>
#include <cstdlib>
#include <random>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main(int argc, char** argv) {
    int n = std::atoi(argv[1]); int sep = std::atoi(argv[2]); double allow = std::atof(argv[3]); int side = std::atoi(argv[4]);
    std::vector<geometry::Contour> cs; int sd = (int)std::ceil(std::sqrt((double)n));
    for (int i = 0; i < n; ++i) { double pitchX = 18 + 2.5 * 1.5875, pitchY = 14 + 2.5 * 1.5875; double x = (i % sd) * pitchX, y = (i / sd) * pitchY; cs.push_back(geometry::Contour::rectangle(geometry::Box2{{x, y}, {x + 18, y + 14}})); }
    ProfileParams p; p.passes.cutDepth = 6; p.passes.passDepth = 2; p.separateLastPass = sep; p.lastPassAllowance = 0.2;
    p.tabs.enabled = true; p.tabs.count = 4; p.tabs.length = 3; p.tabs.thickness = 1.5;
    p.leads.type = LeadType::Circular; p.leads.radius = 2; p.ramp.type = RampType::Smooth; p.ramp.distance = 5;
    p.side = (ProfileSide)side; p.allowance = allow;
    writeScene(argv[5], cs, p, 3.175, 0);
}
