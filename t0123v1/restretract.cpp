// T-0123 verifier (throwaway): rest areas in one region: retracts between them? (owner decision 2026-11-01)
#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
    auto mk = [](const char* n, double d, double so) { PocketTool t; t.tool.name = n; t.tool.diameter = d; t.tool.defaultCuttingData.stepover = so; t.tool.defaultCuttingData.passDepth = 3; return t; };
    for (int s = 0; s < 2; ++s) {
        PocketParams p; p.cutDepth = 6; p.startDepth = 1; p.strategy = (PocketStrategy)s;
        auto r = pocketToolpath({geometry::Contour::rectangle(geometry::Box2{{0, 0}, {100, 60}})}, {mk("B", 12, 4.8), mk("S", 2, 0.8)}, p);
        const auto& tp = r.toolpaths[1]; size_t topFeeds = 0; double topLen = 0;
        for (size_t i = 1; i < tp.moves.size(); ++i) { auto a = tp.startOf(i), b = tp.moves[i].to; if (tp.moves[i].kind == MoveKind::Feed && a.z == -1.0 && b.z == -1.0) { ++topFeeds; topLen += std::hypot(b.x - a.x, b.y - a.y); } }
        std::printf("strategy %d: rest clusters %zu retracts %zu rapids %zu feeds at top-of-cut z=-1: %zu (%.1f mm)\n", s, r.plans[1].paths.back().cluster + 1, tp.count(MoveKind::Retract), tp.count(MoveKind::Rapid), topFeeds, topLen);
    }
}
