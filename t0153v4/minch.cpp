// Minimum chordal 0.0003: refusal just below, timing and oracle-free sanity at the minimum on glyphs and a 50 mm part.
#include <cstdio>
#include <ctime>
#include "gen3.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
int main() {
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.includedAngle = 3.14159265358979 / 3; t.diameter = 12.7;
    auto& c = t.defaultCuttingData; c.feedRate = 1000; c.plungeRate = 300; c.passDepth = 1; c.stepover = 1; c.toolNumber = 1; c.spindleSpeed = 10000;
    struct J { const char* n; std::vector<Contour> cs; };
    std::vector<J> jobs{{"glyphA 6mm", g2::glyph(0, 6, {0, 0})}, {"glyphS 6mm", g2::glyph(1, 6, {0, 0})}, {"square 50mm", {vs::poly({{0, 0}, {50, 0}, {50, 50}, {0, 50}})}}, {"circle r25", {g2::circleAt({0, 0}, 25, 0)}}, {"star 40mm", {vs::star({0, 0}, 20, 8, 7, 0)}}};
    for (auto& j : jobs) for (double ch : {0.00029, 0.0003, 0.001}) for (double fl : {0.0, 0.3}) {
        VCarvingParams p; p.tolerance.chordal = ch; p.tolerance.distance = std::min(p.tolerance.distance, ch); p.flatDepth = fl;
        double t0 = double(std::clock()) / CLOCKS_PER_SEC; auto r = vCarvingToolpath(j.cs, t, std::nullopt, {}, p); double t1 = double(std::clock()) / CLOCKS_PER_SEC;
        std::printf("%-12s ch=%g flat=%g cpu=%.2fs ok=%d moves=%zu", j.n, ch, fl, t1 - t0, (int)r.ok(), r.toolpath.moves.size());
        for (auto& d : r.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.substr(0, 110).c_str()); std::printf("\n"); }
}
