#include <cmath>
#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
    for (int side : {0, 1}) for (double D : {3.175, 6.0}) for (double da : {0.0, 1e-9, -1e-9, 0.3}) for (int sep : {0, 1}) for (double L : {1.0, 5.0}) {
        ProfileParams p; p.side = side ? ProfileSide::Inside : ProfileSide::Outside; p.allowance = -D / 2 + da; p.passes.cutDepth = 6; p.passes.passDepth = 3;
        p.tabs.enabled = true; p.tabs.count = 4; p.tabs.length = L; p.tabs.thickness = 1.5; p.separateLastPass = sep; p.lastPassAllowance = 0.3;
        p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomCentre;
        tooling::Tool t; t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::EndMill; t.diameter = D;
        t.defaultCuttingData.passDepth = 3; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.toolNumber = 1;
        auto res = profileToolpath({geometry::Contour::rectangle(geometry::Box2{{0, 0}, {30, 30}}), geometry::Contour::circle({60, 15}, 10)}, t, p);
        unsigned long long h = 1469598103934665603ULL; char b[256];
        auto mix = [&](const char* s) { for (; *s; ++s) { h ^= (unsigned char)*s; h *= 1099511628211ULL; } };
        for (auto& m : res.toolpath.moves) { std::snprintf(b, sizeof b, "%d %.9f %.9f %.9f %.9f %.9f|", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y); mix(b); }
        for (auto& d : res.diagnostics) mix(d.message.c_str());
        std::printf("side %d D %.3f da %g sep %d L %.0f moves %zu diags %zu hash %016llx\n", side, D, da, sep, L, res.toolpath.moves.size(), res.diagnostics.size(), h);
    }
}
