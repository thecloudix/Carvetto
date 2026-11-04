// round 4: minch with a usable stepover (0.15 < 0.346 cutting width at 0.3 deep) and an end-mill clearance variant
#include <cstdio>
#include <ctime>
#include "gen3.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
int main() {
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.includedAngle = 3.14159265358979 / 3; t.diameter = 12.7;
    auto& c = t.defaultCuttingData; c.feedRate = 1000; c.plungeRate = 300; c.passDepth = 1; c.stepover = 0.15; c.toolNumber = 1; c.spindleSpeed = 10000;
    struct J { const char* n; std::vector<Contour> cs; };
    std::vector<J> jobs{{"glyphA 6mm", g2::glyph(0, 6, {0, 0})}, {"glyphS 6mm", g2::glyph(1, 6, {0, 0})}, {"square 50mm", {vs::poly({{0, 0}, {50, 0}, {50, 50}, {0, 50}})}}, {"circle r25", {g2::circleAt({0, 0}, 25, 0)}}, {"star 40mm", {vs::star({0, 0}, 20, 8, 7, 0)}}};
    tooling::Tool em; em.id = em.name = "EM"; em.type = tooling::ToolType::EndMill; em.diameter = 3; em.defaultCuttingData = c; em.defaultCuttingData.toolNumber = 2; em.defaultCuttingData.stepover = 1.2;
    for (auto& j : jobs) for (double ch : {0.00029, 0.0003, 0.000300001, 0.001}) for (double fl : {0.0, 0.3}) for (int useEm = 0; useEm < (fl > 0 ? 2 : 1); ++useEm) {
        VCarvingParams p; p.tolerance.chordal = ch; p.tolerance.distance = std::min(p.tolerance.distance, ch); p.flatDepth = fl;
        double t0 = double(std::clock()) / CLOCKS_PER_SEC; std::vector<PocketTool> cl; if (useEm) cl.push_back(PocketTool{em, std::nullopt}); auto r = vCarvingToolpath(j.cs, t, std::nullopt, cl, p); double t1 = double(std::clock()) / CLOCKS_PER_SEC;
        std::printf("%-12s em=%d ch=%.9g flat=%g cpu=%.2fs ok=%d moves=%zu", j.n, useEm, ch, fl, t1 - t0, (int)r.ok(), r.toolpath.moves.size()); for (auto& tp : r.toolpaths()) std::printf(" tp%s:%zu", tp.tool.name.c_str(), tp.moves.size());
        for (auto& d : r.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.substr(0, 110).c_str()); std::printf("\n"); }
}
