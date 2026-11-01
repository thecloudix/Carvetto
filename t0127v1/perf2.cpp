// T-0124 verifier (throwaway): profile 500 small vectors with tabs; CPU time.
#include <chrono>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main(int argc, char** argv) {
    int n = argc > 1 ? std::atoi(argv[1]) : 500;
    int sep = argc > 2 ? std::atoi(argv[2]) : 0;
    int kind = argc > 3 ? std::atoi(argv[3]) : 0;
    std::vector<geometry::Contour> cs;
    int side = (int)std::ceil(std::sqrt((double)n));
    for (int i = 0; i < n; ++i) {
        double pitchX = kind == 2 ? 18 + 2.5 * 1.5875 : 30.0, pitchY = kind == 2 ? 14 + 2.5 * 1.5875 : 30.0;
        double x = (i % side) * pitchX, y = (i / side) * pitchY;
        if (kind == 2) cs.push_back(geometry::Contour::rectangle(geometry::Box2{{x, y}, {x + 18, y + 14}}));
        else if (kind == 0) cs.push_back(i % 2 ? geometry::Contour::circle({x + 10, y + 10}, 8) : geometry::Contour::rectangle(geometry::Box2{{x, y}, {x + 18, y + 14}}));
        else { // letter-ish polygon with 8 corners
            geometry::Contour c; c.closed = true;
            std::vector<geometry::Vec2> p{{x, y}, {x + 18, y}, {x + 18, y + 6}, {x + 12, y + 6}, {x + 12, y + 18}, {x + 6, y + 18}, {x + 6, y + 6}, {x, y + 6}};
            for (size_t k = 0; k < p.size(); ++k) c.append(geometry::LineSpan{p[k], p[(k + 1) % p.size()]});
            cs.push_back(c);
        }
    }
    tooling::Tool t; t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::EndMill; t.diameter = 3.175;
    t.defaultCuttingData.passDepth = 2; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.toolNumber = 1;
    ProfileParams p; p.passes.cutDepth = 6; p.passes.passDepth = 2; p.separateLastPass = sep; p.lastPassAllowance = 0.2;
    p.tabs.enabled = true; p.tabs.count = 4; p.tabs.length = 3; p.tabs.thickness = 1.5;
    p.leads.type = LeadType::Circular; p.leads.radius = 2; p.ramp.type = RampType::Smooth; p.ramp.distance = 5;
    if (std::getenv("NOTABS")) p.tabs.enabled = false;
    if (std::getenv("SIDE")) p.side = (ProfileSide)std::atoi(std::getenv("SIDE"));
    if (std::getenv("ALLOW")) p.allowance = std::atof(std::getenv("ALLOW"));
    std::clock_t c0 = std::clock(); auto w0 = std::chrono::steady_clock::now();
    auto r = profileToolpath(cs, t, p);
    double cpu = double(std::clock() - c0) / CLOCKS_PER_SEC, wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    size_t tn = 0; for (auto& d : r.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced) ++tn;
    std::printf("n %d sep %d kind %d: moves %zu diags %zu (tab diags %zu) cpu %.2fs wall %.2fs\n", n, sep, kind, r.toolpath.moves.size(), r.diagnostics.size(), tn, cpu, wall);
}
