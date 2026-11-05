// t0160v1 repro (vc3 fam 7 seed 607 case 22, overpoly): with cornerSharpen on, the cuts of a LATER region are emitted in a
// different order (nearest-first from where the corner cuts left the tool), so on != off + inserted cuts.
// usage: order_run repro/order_fam7_607_22.txt
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <tuple>
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::ifstream in(argv[1]); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; } else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); } }
    tooling::Tool v; v.id = v.name = "Carve"; v.type = tooling::ToolType::VBit; v.diameter = 1.0444184374203789; v.includedAngle = 126.56807094017771 * M_PI / 180;
    v.defaultCuttingData.toolNumber = 1; v.defaultCuttingData.feedRate = 1000; v.defaultCuttingData.plungeRate = 300; v.defaultCuttingData.spindleSpeed = 18000; v.defaultCuttingData.passDepth = 1.586622871403903; v.defaultCuttingData.stepover = 0.41776737496815158;
    tooling::Tool e = v; e.id = e.name = "EM0"; e.type = tooling::ToolType::EndMill; e.diameter = 5.0699874097944901; e.includedAngle = 0; e.defaultCuttingData.toolNumber = 2; e.defaultCuttingData.stepover = 0.4 * e.diameter;
    VCarvingParams on; on.flatDepth = 0.25289822537933671; on.startDepth = 0.020472249626880358; on.safeZ = 2.6792918851477463; on.tolerance.chordal = 0.001; on.tolerance.distance = 0.001;
    on.clearanceStrategy = PocketStrategy::Offset; on.clearanceRasterAngle = 0.3455166377283827; on.rampPlunge = true;
    VCarvingParams off = on; off.cornerSharpen = false;
    auto A = vCarvingToolpath(cs, v, std::nullopt, {PocketTool{e, std::nullopt}}, on), B = vCarvingToolpath(cs, v, std::nullopt, {PocketTool{e, std::nullopt}}, off);
    const auto& ma = A.toolpath.moves; const auto& mb = B.toolpath.moves; size_t q = 0; while (q < ma.size() && ma[q] == mb[q]) ++q;
    std::printf("ok %d/%d, on %zu moves, off %zu; first difference at move %zu\n", (int)A.ok(), (int)B.ok(), ma.size(), mb.size(), q);
    // strip inserted cuts (rapid .. retract blocks not matching) greedily
    size_t i = 0, j = 0; int inserted = 0; while (i < ma.size()) { if (j < mb.size() && ma[i] == mb[j]) { ++i; ++j; continue; } if (ma[i].kind != MoveKind::Rapid) break; ++inserted; ++i; while (i < ma.size() && ma[i].kind != MoveKind::Retract) ++i; ++i; }
    std::printf("after skipping %d inserted cut(s): matched %zu of %zu off moves -> %s\n", inserted, j, mb.size(), j == mb.size() ? "only insertions" : "LATER MOVES REORDERED");
    std::map<std::tuple<int, double, double, double>, int> cnt; for (auto& m : mb) cnt[{(int)m.kind, m.to.x, m.to.y, m.to.z}]++; for (auto& m : ma) { auto& c = cnt[{(int)m.kind, m.to.x, m.to.y, m.to.z}]; if (c > 0) --c; } int missing = 0; for (auto& [k, c] : cnt) missing += c;
    std::printf("off moves absent from on (as a multiset): %d\n", missing);
}
