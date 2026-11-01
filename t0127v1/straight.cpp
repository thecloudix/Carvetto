// On profile, rectangle 100 x 60 and a hexagon-ish polygon; count raised runs at tab top on the final level
// and classify each tab midpoint's distance to the nearest corner of the source shape.
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main() {
    long tot = 0, keptAll = 0, keptStraight = 0, reqStraight = 0;
    for (double D : {3.175, 6.0, 12.0}) for (double a : {-0.3, 0.0, 0.3}) for (int sep : {0, 1}) for (double L : {1.0, 2.0, 5.0, 10.0}) for (int n : {2, 4, 6}) for (int td : {0, 1}) {
        ProfileParams p; p.side = ProfileSide::On; p.allowance = a; p.passes.cutDepth = 8; p.passes.passDepth = 3;
        p.tabs.enabled = true; p.tabs.count = n; p.tabs.length = L; p.tabs.thickness = 1.5; p.tabs.threeD = td;
        p.separateLastPass = sep; p.lastPassAllowance = 0.3; p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomLeft;
        tooling::Tool t; t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::EndMill; t.diameter = D;
        t.defaultCuttingData.passDepth = 3; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.toolNumber = 1;
        auto res = profileToolpath({geometry::Contour::rectangle(geometry::Box2{{0, 0}, {100, 60}})}, t, p);
        const double zTab = p.passes.bottomZ() + p.tabs.thickness;
        const auto& mv = res.toolpath.moves;
        // count plateau runs at zTab (consecutive moves with to.z == zTab) on the last cut segment
        int runs = 0, straight = 0; 
        size_t lastRetract = 0; for (size_t i = 0; i + 1 < mv.size(); ++i) if (mv[i].kind == MoveKind::Retract || mv[i].kind == MoveKind::Rapid) { if (i + 2 < mv.size()) lastRetract = i; }
        size_t start = 0; // final level of final cut: find from the end the last run of moves at bottomZ region
        std::vector<std::pair<Vec2,Vec2>> plateaus;
        for (size_t i = 1; i < mv.size(); ++i) {
            if (std::fabs(mv[i].to.z - zTab) < 1e-9 && std::fabs(mv[i-1].to.z - zTab) < 1e-9 && mv[i].kind != MoveKind::Rapid) {
                Vec2 a0 = mv[i-1].to.xy(), b0 = mv[i].to.xy();
                if (!plateaus.empty() && plateaus.back().second.x == a0.x && plateaus.back().second.y == a0.y) plateaus.back().second = b0; else plateaus.push_back({a0, b0});
            }
        }
        (void)start; (void)lastRetract;
        for (auto& pl : plateaus) {
            Vec2 m{(pl.first.x + pl.second.x) / 2, (pl.first.y + pl.second.y) / 2};
            double dc = 1e9; for (Vec2 c : {Vec2{0,0}, Vec2{100,0}, Vec2{100,60}, Vec2{0,60}}) dc = std::min(dc, std::hypot(m.x-c.x, m.y-c.y));
            ++runs; if (dc > D + L) ++straight;
        }
        int left = 0; for (auto& d : res.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced) { int a1, b1; if (std::sscanf(d.message.c_str() + d.message.find(": ") + 2, "%d of %d", &a1, &b1) == 2) left += a1; }
        std::printf("D %.3f a %.1f sep %d L %.0f n %d 3d %d plateaus %d straight %d left %d\n", D, a, sep, L, n, td, runs, straight, left);
        ++tot; keptAll += runs; keptStraight += straight;
    }
    std::printf("TOTAL configs %ld plateaus %ld onStraight %ld\n", tot, keptAll, keptStraight);
}
