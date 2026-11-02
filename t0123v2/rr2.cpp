#include <cstdio>
#include <cmath>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
    auto mk = [](const char* n, double d, double so) { PocketTool t; t.tool.name = n; t.tool.diameter = d; t.tool.defaultCuttingData.stepover = so; t.tool.defaultCuttingData.passDepth = 3; return t; };
    PocketParams p; p.cutDepth = 6; p.startDepth = 1; p.strategy = (PocketStrategy)1;
    auto r = pocketToolpath({geometry::Contour::rectangle(geometry::Box2{{0, 0}, {100, 60}})}, {mk("B", 12, 4.8), mk("S", 2, 0.8)}, p);
    const auto& tp = r.toolpaths[1];
    for (size_t i = 0; i < tp.moves.size(); ++i) { auto a = i ? tp.startOf(i) : tp.moves[i].to; auto b = tp.moves[i].to;
      std::printf("%zu k%d (%.3f,%.3f,%.3f)->(%.3f,%.3f,%.3f)%s\n", i, (int)tp.moves[i].kind, a.x,a.y,a.z,b.x,b.y,b.z, (tp.moves[i].kind==MoveKind::Feed && a.z==-1.0 && b.z==-1.0)?" TOP":""); }
    for (auto& pp : r.plans[1].paths) std::printf("path cluster %zu first (%.3f,%.3f) n %zu\n", pp.cluster, pp.path.points.front().x, pp.path.points.front().y, pp.path.points.size());
}
