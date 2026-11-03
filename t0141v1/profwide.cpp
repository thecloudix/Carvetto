#include <cstdio>
#include "carvetto/geometry/box2.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scenes.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
  tooling::Tool t; t.id = "e"; t.name = "e"; t.type = tooling::ToolType::EndMill; t.diameter = 3;
  for (int sc = 0; sc < 6; ++sc) for (int side = 0; side < 3; ++side) {
    std::vector<Contour> cs; double gap = sc < 2 ? 4.0 : sc < 4 ? 3.0 : 2.9;
    if (sc % 2) cs.push_back(rectC({-5, -5}, {60 * (5 + gap) + 5, 15}));
    for (int i = 0; i < 60; ++i) { double x = i * (5 + gap); cs.push_back(i % 3 ? rectC({x, 0}, {x + 5, 5 + (i % 4)}) : Contour::circle({x + 2.5, 4}, 2.5)); }
    ProfileParams p; p.passes.cutDepth = 3; p.safeZ = 5; p.side = side == 0 ? ProfileSide::Outside : side == 1 ? ProfileSide::Inside : ProfileSide::On;
    auto r = profileToolpath(cs, t, p); std::vector<Contour> dummy;
    uint64_t h = 1469598103934665603ull; for (auto& m : r.toolpath.moves) { const double v[] = {m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y}; auto b = reinterpret_cast<const unsigned char*>(v); for (size_t i = 0; i < sizeof v; ++i) { h ^= b[i]; h *= 1099511628211ull; } h ^= (uint64_t)m.kind; }
    std::printf("sc %d side %d moves %zu diags %zu hash %016llx\n", sc, side, r.toolpath.moves.size(), r.diagnostics.size(), (unsigned long long)h); }
}
