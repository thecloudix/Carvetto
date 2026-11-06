// t0160v2 perf3: one chain run with sharpening on (for perf record). usage: perf3 <n>
#include <cstdio>
#include "shapes.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
int main(int argc, char** argv) { int n = atoi(argv[1]); std::vector<Vec2> lo, hi; for (int k = 0; k < n; ++k) { double x = 12.0 * k; lo.push_back({x, 0}); lo.push_back({x + 10, 0}); if (k + 1 < n) { lo.push_back({x + 10, 4.25}); lo.push_back({x + 12, 4.25}); } }
    for (int k = n - 1; k >= 0; --k) { double x = 12.0 * k; if (k + 1 < n) { hi.push_back({x + 12, 5.75}); hi.push_back({x + 10, 5.75}); } hi.push_back({x + 10, 10}); hi.push_back({x, 10}); }
    std::vector<Vec2> p = lo; p.insert(p.end(), hi.begin(), hi.end()); std::vector<Contour> cs{vs::poly(p)};
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.diameter = 12.7; t.includedAngle = vo::PI / 2; t.defaultCuttingData.toolNumber = 1; t.defaultCuttingData.feedRate = 1500; t.defaultCuttingData.plungeRate = 500; t.defaultCuttingData.spindleSpeed = 18000; t.defaultCuttingData.stepover = 0.5; t.defaultCuttingData.passDepth = 3;
    tooling::Tool e = t; e.id = e.name = "EM"; e.type = tooling::ToolType::EndMill; e.diameter = 3; e.includedAngle = 0; e.defaultCuttingData.toolNumber = 2; e.defaultCuttingData.stepover = 1.2;
    VCarvingParams pr; pr.flatDepth = 1.0; pr.tolerance.chordal = 0.05; auto r = vCarvingToolpath(cs, t, std::nullopt, {PocketTool{e, std::nullopt}}, pr); std::printf("%zu %d\n", r.toolpath.moves.size(), (int)r.ok()); }
