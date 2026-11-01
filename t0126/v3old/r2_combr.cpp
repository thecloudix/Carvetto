#include "../t0121v2/common.hpp"
#include <sys/resource.h>
int main(int argc, char** argv) {
  double pd = atof(argv[1]), so = atof(argv[2]); int strat = atoi(argv[3]); double d = argc > 4 ? atof(argv[4]) : 0.2;
  std::vector<Vec2> pts; int teeth = 200; for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, 20}); pts.push_back({i + 0.5, 20}); pts.push_back({i + 0.5, 0.0}); } pts.push_back({double(teeth), -5}); pts.push_back({0, -5});
  PocketParams p; p.cutDepth = 9; p.strategy = strat ? PocketStrategy::Raster : PocketStrategy::Offset; p.rasterAngle = 0;
  auto t0 = std::chrono::steady_clock::now();
  auto r = pocketToolpath({Contour::fromPolyline(geometry::Polyline{pts, true})}, {tool(d, so, pd)}, p);
  double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  struct rusage ru; getrusage(RUSAGE_SELF, &ru);
  printf("pd=%g so=%g strat=%d d=%g: %.1fs moves=%zu paths=%zu levels=%zu maxrss=%ld MB msgs=%zu %s\n", pd, so, strat, d, el, r.toolpaths[0].moves.size(), r.plans[0].paths.size(), r.plans[0].levels.size(), ru.ru_maxrss / 1024, r.messages.size(), r.messages.empty() ? "" : r.messages[0].c_str());
}
