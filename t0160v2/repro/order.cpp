// t0160v1 repro: corner cuts at the end of one region change where the tool is, so the NEXT region's
// cuts (ordered nearest-first from the tool position) come in a different order: on != off + insertions.
// Prints, for two squares side by side (gap g), whether on == off + complete inserted cuts, and
// whether the moves are the same multiset.
#include <cstdio>
#include <cmath>
#include <map>
#include <tuple>
#include "carvetto/toolpath/v_carve.hpp"
#include "carvetto/geometry/contour.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
static tooling::Tool mk(int type, double D, double ang, const char* n, int num, double so) { tooling::Tool t; t.id = t.name = n; t.diameter = D; t.type = type == 0 ? tooling::ToolType::VBit : tooling::ToolType::EndMill; if (type == 0) t.includedAngle = ang * 3.14159265358979323846 / 180; auto& c = t.defaultCuttingData; c.toolNumber = num; c.feedRate = 1000; c.plungeRate = 300; c.spindleSpeed = 18000; c.passDepth = 3; c.stepover = so; return t; }
int main() {
    int shown = 0;
    for (double w2 : {4.0, 6.0, 8.0, 10.0, 14.0, 20.0}) for (double g : {1.0, 5.0, 12.0}) for (double dy : {0.0, 7.0, 15.0}) {
        Contour st; st.closed = true; { std::vector<Vec2> p; for (int k = 0; k < 10; ++k) { double r = k % 2 ? w2 * 0.2 : w2 * 0.5, a = 3.14159265358979323846 * k / 5; p.push_back({20 + g + w2 * 0.5 + r * std::cos(a), dy + w2 * 0.5 + r * std::sin(a)}); } for (int k = 0; k < 10; ++k) st.append(LineSpan{p[k], p[(k + 1) % 10]}); }
        std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {20, 20}}), st};
        tooling::Tool v = mk(0, 12.7, 90, "V", 1, 0.5); std::vector<PocketTool> cl{PocketTool{mk(2, 6, 0, "EM", 2, 2.4), std::nullopt}};
        VCarvingParams on; on.flatDepth = 1.0; on.tolerance.chordal = 0.05; VCarvingParams off = on; off.cornerSharpen = false;
        auto A = vCarvingToolpath(cs, v, std::nullopt, cl, on), B = vCarvingToolpath(cs, v, std::nullopt, cl, off);
        const auto& ma = A.toolpath.moves; const auto& mb = B.toolpath.moves; size_t i = 0, j = 0; bool ins = true;
        while (i < ma.size()) { if (j < mb.size() && ma[i] == mb[j]) { ++i; ++j; continue; } if (ma[i].kind != MoveKind::Rapid) { ins = false; break; } ++i; while (i < ma.size() && ma[i].kind != MoveKind::Retract) ++i; ++i; }
        if (j != mb.size()) ins = false;
        std::map<std::tuple<int, double, double, double>, int> cnt; for (auto& m : mb) cnt[{(int)m.kind, m.to.x, m.to.y, m.to.z}]++; int missing = 0; for (auto& m : ma) { auto& c = cnt[{(int)m.kind, m.to.x, m.to.y, m.to.z}]; if (c > 0) --c; } for (auto& [k, c] : cnt) missing += c;
        if (!ins && shown++ < 5) std::printf("w2=%g gap=%g dy=%g: on %zu moves, off %zu: on == off + insertions? %s; off moves missing from on: %d\n", w2, g, dy, ma.size(), mb.size(), ins ? "yes" : "NO (order of later cuts changed)", missing);
    }
    std::printf("layouts with reordering shown: %d\n", shown);
}
