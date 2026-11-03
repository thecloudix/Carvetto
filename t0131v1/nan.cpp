// T-0131 verifier: Profile/Pocket with NaN/inf/huge vectors among normal ones (sep last pass, tabs,
// rest tools, raster): prints a hash per case, compare main vs branch.
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2; using geometry::Box2;
static unsigned long long H = 1469598103934665603ULL;
static void hb(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { H ^= c[i]; H *= 1099511628211ULL; } }
static void htp(const Toolpath& tp) { for (auto& m : tp.moves) { int k = (int)m.kind; hb(&k, 4); hb(&m.to, sizeof m.to); hb(&m.centre, sizeof m.centre); } }
int main() {
  const double nan_ = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
  std::vector<std::vector<Contour>> scenes;
  for (int k = 0; k < 6; ++k) {
    std::vector<Contour> cs;
    for (int i = 0; i < 30; ++i) { double x = (i % 6) * 8.0, y = (i / 6) * 8.0; cs.push_back(i % 3 ? Contour::rectangle(Box2{{x, y}, {x + 5, y + 4}}) : Contour::circle({x + 2.5, y + 2}, 2)); }
    Contour bad;
    if (k == 0) bad = Contour::fromPolyline(geometry::Polyline{{{0, 0}, {nan_, 3}, {4, 4}}, true});
    if (k == 1) bad = Contour::fromPolyline(geometry::Polyline{{{0, 0}, {inf, 3}, {4, 4}}, true});
    if (k == 2) bad = Contour::rectangle(Box2{{-1e300, -1e300}, {1e300, 1e300}});
    if (k == 3) bad = Contour::circle({nan_, 0}, 3);
    if (k == 4) bad = Contour::rectangle(Box2{{-1e8, -1e8}, {1e8 + 0.5, 1e8}});
    if (k == 5) bad = Contour::fromPolyline(geometry::Polyline{{{1, 1}, {1, 1}, {1, 1}}, true});
    cs.insert(cs.begin() + 7, bad); scenes.push_back(cs);
  }
  tooling::Tool t; t.id = "e"; t.name = "E"; t.diameter = 1.0; t.defaultCuttingData.passDepth = 3; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.stepover = 0.4;
  for (size_t s = 0; s < scenes.size(); ++s) for (int o = 0; o < 4; ++o) {
    H = 1469598103934665603ULL;
    ProfileParams p; p.passes.cutDepth = 3; p.separateLastPass = o & 1; p.tabs.enabled = o & 2; p.tabs.length = 1; p.tabs.count = 2; p.tabs.thickness = 1;
    auto r = profileToolpath(scenes[s], t, p); htp(r.toolpath); for (auto& d : r.diagnostics) hb(d.message.data(), d.message.size());
    PocketParams q; q.cutDepth = 3; q.strategy = o & 1 ? PocketStrategy::Raster : PocketStrategy::Offset; q.rasterAngle = o * 0.4;
    PocketTool a; a.tool = t; PocketTool b; b.tool = t; b.tool.diameter = 0.3; b.tool.name = "S"; b.tool.defaultCuttingData.stepover = 0.12;
    auto pr = pocketToolpath(scenes[s], o & 2 ? std::vector<PocketTool>{a, b} : std::vector<PocketTool>{a}, q); for (auto& tp : pr.toolpaths) htp(tp); for (auto& d : pr.diagnostics) hb(d.message.data(), d.message.size());
    std::fprintf(stderr, ""); std::printf("scene %zu opt %d prof %zu moves %zu diags | pock %zu diags hash %016llx\n", s, o, r.toolpath.moves.size(), r.diagnostics.size(), pr.diagnostics.size(), H); std::fflush(stdout);
  }
}
