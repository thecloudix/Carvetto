// Tiny / degenerate contours next to a normal job: does a speck refuse the whole V-carve?
#include <cstdio>
#include "gen3.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static void run(const char* name, std::vector<Contour> cs, double ch) {
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.includedAngle = 3.14159265358979 / 2; t.diameter = 12;
    auto& c = t.defaultCuttingData; c.feedRate = 1000; c.plungeRate = 300; c.passDepth = 1; c.stepover = 1; c.toolNumber = 1; c.spindleSpeed = 10000;
    VCarvingParams p; p.tolerance.chordal = ch; p.tolerance.distance = std::min(p.tolerance.distance, ch);
    auto r = vCarvingToolpath(cs, t, std::nullopt, {}, p);
    std::printf("%-40s ch=%g ok=%d moves=%zu regions=%zu", name, ch, (int)r.ok(), r.toolpath.moves.size(), r.regions);
    for (auto& d : r.diagnostics) std::printf("\n     [%d] %s", (int)d.issue, d.message.substr(0, 150).c_str()); std::printf("\n");
}
int main() {
    for (double ch : {0.01, 0.1}) {
        for (double rr : {1e-5, 1e-4, 1e-3, 3e-3, 0.01, 0.03}) {
            char nm[100];
            std::snprintf(nm, sizeof nm, "square + speck island r=%g", rr); run(nm, {vs::poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), g2::circleAt({20, 5}, rr, 0)}, ch);
            std::snprintf(nm, sizeof nm, "square + speck hole r=%g", rr); run(nm, {vs::poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), vs::rev(g2::circleAt({5, 5}, rr, 0))}, ch);
            std::snprintf(nm, sizeof nm, "square + speck triangle h=%g", rr); run(nm, {vs::poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), vs::poly({{20, 0}, {20 + rr, 0}, {20, rr}})}, ch);
        }
        run("square + sliver 5mm x 1e-6", {vs::poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), vs::poly({{20, 0}, {25, 0}, {25, 1e-6}, {20, 1e-6}})}, ch);
        run("square + degenerate A-B-A", {vs::poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), vs::poly({{2, 2}, {5, 5}})}, ch);
        run("square + collinear triangle inside", {vs::poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), vs::poly({{2, 2}, {5, 5}, {8, 8}})}, ch);
        run("square + degenerate inside + D-hole", {vs::poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), vs::poly({{2, 2}, {5, 5}, {8, 8}}), vs::rev(vs::poly({{4, 1e-3}, {6, 1e-3}, {6, 1}}))}, ch);
    }
}
