// T-0142 verifier (e): Pocket raster on the T-0121 comb: CPU time and move hash per (teeth, angle).
// usage: pc teeth angle_deg [tool_d stepover]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_test_util.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
struct H { unsigned long long h = 1469598103934665603ULL; void b(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; } } void d(double x) { b(&x, 8); } };
int main(int argc, char** argv) { int teeth = atoi(argv[1]); double ang = atof(argv[2]); double td = argc > 3 ? atof(argv[3]) : 0.3, so = argc > 4 ? atof(argv[4]) : 0.12;
  std::vector<Vec2> pts; for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, 20}); pts.push_back({i + 0.5, 20}); pts.push_back({i + 0.5, 0.0}); }
  pts.push_back({double(teeth), -5}); pts.push_back({0, -5}); Contour comb = Contour::fromPolyline(geometry::Polyline{pts, true});
  PocketParams p; p.cutDepth = 9.0; p.strategy = PocketStrategy::Raster; p.rasterAngle = ang * M_PI / 180.0;
  std::vector<PocketTool> tools{carvetto::test::pocket::endMill("B", 1, td, so, 9.0)};
  std::clock_t t0 = std::clock(); PocketResult r = pocketToolpath({comb}, tools, p); double t = double(std::clock() - t0) / CLOCKS_PER_SEC;
  H h; size_t mv = 0; for (auto& tp : r.toolpaths) for (auto& m : tp.moves) { ++mv; h.d((double)(int)m.kind); h.d(m.to.x); h.d(m.to.y); h.d(m.to.z); }
  std::printf("teeth %d ang %g: %.3f s moves %zu diags %zu hash %016llx\n", teeth, ang, t, mv, r.diagnostics.size(), h.h); }
