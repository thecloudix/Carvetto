#include "../t0121v2/common.hpp"
static double area(const Contour& c) { double A = 0; for (auto& sp : c.spans) { auto& l = std::get<LineSpan>(sp); A += l.start.x * l.end.y - l.end.x * l.start.y; } return A / 2; }
int main(int argc, char** argv) {
  double w = atof(argv[1]); int strat = atoi(argv[2]); double d2 = argc > 3 ? atof(argv[3]) : 3;
  Scene s; s.addPoly({{0, 0}, {25, 0}, {25, 15 - w / 2}, {40, 15 - w / 2}, {40, 5}, {60, 5}, {60, 25}, {40, 25}, {40, 15 + w / 2}, {25, 15 + w / 2}, {25, 30}, {0, 30}});
  PocketParams p; p.cutDepth = 6; p.safeZ = 5; p.strategy = strat ? PocketStrategy::Raster : PocketStrategy::Offset; p.rasterAngle = 0.4;
  auto r = pocketToolpath(s.contours, {tool(12, 4.8, 2, "T12"), tool(d2, d2 * 0.4, 2, "Tr")}, p);
  for (auto& m : r.messages) printf("msg %s\n", m.c_str());
  for (int t = 0; t < 2; ++t) {
    auto& pl = r.plans[t];
    printf("tool %d: centre contours %zu:", t, pl.centreRegion.size()); for (auto& c : pl.centreRegion) printf(" %.2f", area(c)); printf("\n");
    printf("  rest contours %zu:", pl.restMaterial.size()); for (auto& c : pl.restMaterial) { Box2 b = c.bounds(); printf(" [%.2f (%.2f,%.2f)-(%.2f,%.2f)]", area(c), b.min.x, b.min.y, b.max.x, b.max.y); } printf("\n");
    int inNeck = 0; for (auto& pa : pl.paths) for (auto q : pa.path.points) if (q.x > 26 && q.x < 39) ++inNeck;
    printf("  paths %zu, points in neck %d, moves %zu\n", pl.paths.size(), inNeck, r.toolpaths[t].moves.size());
  }
}
