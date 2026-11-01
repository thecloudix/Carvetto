// T-0124 round-3 verifier (v4) repro (throwaway): ON profile of a 30 x 30 square, 6 mm end mill, 4 tabs 1.0 x 1.5,
// 1.0 x 1.5 starting at the bottom centre, so each tab is centred on a (sharp, 90-degree) corner of the cut.
// Each raised stretch is 7 mm (3.5 mm along each edge), so the tool at its two ends - at full depth - stands
// 3.5*sqrt(2) = 4.95 mm apart (< 2r = 6): the discs overlap across the corner and, with the edge kerfs,
// leave no material joining part and waste. Expected: kept and joined, or left out and reported.
// argv[1] = tab length (default 1.0), argv[2] = separate last pass (0/1), argv[3] = allowance (default 0.3).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main(int argc, char** argv) {
    ProfileParams p;
    p.side = ProfileSide::On; p.allowance = argc > 3 ? std::atof(argv[3]) : 0.3; p.passes.cutDepth = 6; p.passes.passDepth = 3;
    p.tabs.enabled = true; p.tabs.count = 4; p.tabs.length = argc > 1 ? std::atof(argv[1]) : 1.0; p.tabs.thickness = 1.5;
    p.separateLastPass = argc > 2 && std::atoi(argv[2]); p.lastPassAllowance = 0.3;
    p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomCentre;
    tooling::Tool t; t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::EndMill; t.diameter = 6.0;
    t.defaultCuttingData.passDepth = 3; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.toolNumber = 1;
    auto res = profileToolpath({geometry::Contour::rectangle(geometry::Box2{{0, 0}, {30, 30}})}, t, p);
    for (auto& d : res.diagnostics) std::printf("diag %d %s\n", (int)d.issue, d.message.c_str());
    const double zTab = p.passes.bottomZ() + p.tabs.thickness, r = 3.0;
    const auto& mv = res.toolpath.moves;
    int tabs = 0, severed = 0;
    for (size_t i = 1; i + 1 < mv.size(); ++i) {
        if (!(mv[i].to.z == zTab && mv[i - 1].to.z < zTab && mv[i].to.x == mv[i - 1].to.x && mv[i].to.y == mv[i - 1].to.y)) continue;
        if (mv[i - 1].to.z != p.passes.bottomZ()) continue;
        size_t j = i + 1; while (j < mv.size() && mv[j].to.z == zTab) ++j;
        Vec2 a = mv[i - 1].to.xy(), b = mv[j].to.xy();
        double d = std::hypot(a.x - b.x, a.y - b.y);
        ++tabs; if (d < 2 * r) ++severed;
        std::printf("tab: tool at bottom before (%.4f,%.4f) and after (%.4f,%.4f): %.4f apart (2r = %.1f)%s\n", a.x, a.y, b.x, b.y, d, 2 * r, d < 2 * r ? "  -> discs overlap: tab cut through" : "");
    }
    std::printf("bottom-level tab plateaus %d, cut through %d\n", tabs, severed);
}
