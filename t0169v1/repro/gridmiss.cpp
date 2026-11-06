// t0169v1 repro: EdgeGrid::anyCloser vs brute force on the wb seed-21 case (rings in grid_miss_rings.txt)
#include "toolpath/pocket.cpp"
#include <cstdio>
#include <fstream>
#include <sstream>
using namespace carvetto::toolpath;
int main(int argc, char** argv) {
  std::ifstream in(argv[1]); std::string l; std::vector<Ring> rs; double cellArg = 0;
  while (std::getline(in, l)) { std::istringstream is(l); std::string t; is >> t; if (t == "RINGS") { is >> t >> cellArg; } else if (t == "ring") { Ring r; double x, y; while (is >> x >> y) r.push_back({x, y}); rs.push_back(r); } }
  Vec2 a{0.44956935713374679, 0.0070003571337468214}, b{0.47199624902709314, 0.029427249027093174}; double d = 1.9199167225117231e-05;
  for (size_t ri = 0; ri < rs.size(); ++ri) for (size_t i = 0; i < rs[ri].size(); ++i) { Vec2 c = rs[ri][i], e = rs[ri][(i + 1) % rs[ri].size()]; double ds = distSegments(a, b, c, e); if (ds < 1e-3) std::printf("ring %zu edge %zu (%.17g,%.17g)-(%.17g,%.17g) distSegments %.17g dps a %.3g b %.3g\n", ri, i, c.x, c.y, e.x, e.y, ds, distToSegment(a, c, e), distToSegment(b, c, e)); }
  EdgeGrid g(rs, cellArg); std::printf("anyCloser %d nearestWithin %.17g\n", (int)g.anyCloser(a, b, d), g.nearestWithin(a, b, d));
}
