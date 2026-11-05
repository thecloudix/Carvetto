// T-0142 round-2 verifier: Pocket (Offset strategy, stepover > radius so the reach outset runs) on hex-packed
// holed squares (boxes overlap across rows: one separableBatches batch). usage: pkh n reps ; interleaved n/2n CPU.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_test_util.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
static Contour rc(double x0, double y0, double x1, double y1, bool ccw) { Contour c = Contour::rectangle(Box2{{x0, y0}, {x1, y1}}); if (!ccw) c.reverse(); return c; }
static std::vector<Contour> scene(int n, int layout) { std::vector<Contour> cs; int k = std::max(1, (int)std::sqrt((double)n));
  for (int i = 0; i < n; ++i) { int r = i / k, c = i % k; double x = layout ? 2.0 * c : 1.2 * c + (r % 2 ? 0.6 : 0.0), y = layout ? 2.0 * r : 0.9 * r;
    cs.push_back(rc(x, y, x + 1, y + 1, true)); cs.push_back(rc(x + 0.35, y + 0.35, x + 0.65, y + 0.65, false)); } return cs; }
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
struct H { unsigned long long h = 1469598103934665603ULL; void d(double x) { auto* c = (const unsigned char*)&x; for (int i = 0; i < 8; ++i) { h ^= c[i]; h *= 1099511628211ULL; } } };
int main(int argc, char** argv) { int n = atoi(argv[1]), reps = argc > 2 ? atoi(argv[2]) : 1, layout = argc > 3 ? atoi(argv[3]) : 0;
  PocketParams p; p.cutDepth = 1.0; p.strategy = PocketStrategy::Offset; p.tolerance = Tolerance{0.001, 1e-9, 0.001};
  auto tool = carvetto::test::pocket::endMill("EM", 1, 0.1, 0.07, 1.0);
  auto a = scene(n, layout), b = scene(2 * n, layout); double t1 = 1e30, t2 = 1e30; unsigned long long h1 = 0, h2 = 0; size_t m1 = 0, m2 = 0;
  for (int k = 0; k < reps; ++k) { double s = cpu(); auto ra = pocketToolpath(a, {tool}, p); double u = cpu(); auto rb = pocketToolpath(b, {tool}, p); double v = cpu();
    t1 = std::min(t1, u - s); t2 = std::min(t2, v - u); H x, y; for (auto& mv : ra.toolpaths.at(0).moves) { x.d(mv.to.x); x.d(mv.to.y); x.d(mv.to.z); } for (auto& mv : rb.toolpaths.at(0).moves) { y.d(mv.to.x); y.d(mv.to.y); y.d(mv.to.z); }
    h1 = x.h; h2 = y.h; m1 = ra.toolpaths[0].moves.size(); m2 = rb.toolpaths[0].moves.size(); if (!ra.diagnostics.empty()) std::printf("diag %s\n", ra.diagnostics[0].message.c_str()); }
  std::printf("pkh layout %d %d->%d: %.3f %.3f s ratio %.3f moves %zu %zu hash %016llx %016llx\n", layout, n, 2 * n, t1, t2, t2 / t1, m1, m2, h1, h2); }
