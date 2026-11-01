#include "../t0121v2/common.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double area(const Contour& c) { double A = 0; for (auto& sp : c.spans) { auto& l = std::get<LineSpan>(sp); A += l.start.x * l.end.y - l.end.x * l.start.y; } return A / 2; }
static void show(const char* n, const std::vector<Contour>& cs) { printf("%s: %zu:", n, cs.size()); for (auto& c : cs) { Box2 b = c.bounds(); printf(" [%.3f (%.2f,%.2f)-(%.2f,%.2f)]", area(c), b.min.x, b.min.y, b.max.x, b.max.y); } printf("\n"); }
int main(int argc, char** argv) {
  double w = atof(argv[1]);
  Scene s; s.addPoly({{0, 0}, {25, 0}, {25, 15 - w / 2}, {40, 15 - w / 2}, {40, 5}, {60, 5}, {60, 25}, {40, 25}, {40, 15 + w / 2}, {25, 15 + w / 2}, {25, 30}, {0, 30}});
  Tolerance tol; tol.chordal = 0.005; Tolerance fine = tol; fine.chordal = 1e-4;
  auto region = unionContours(s.contours, tol, FillRule::EvenOdd);
  auto C0 = offsetContours(region, -6.01, JoinType::Round, tol); show("C0 raw", C0);
  auto target = offsetContours(region, -0.01, JoinType::Round, tol); show("target", target);
  auto pr = PocketParams{}; pr.cutDepth = 1;
  auto r = pocketToolpath(s.contours, {tool(12, 4.8, 1)}, pr);
  show("plan C0", r.plans[0].centreRegion);
  auto swept = offsetContours(r.plans[0].centreRegion, 6, JoinType::Round, fine); show("swept", swept);
  auto diff = booleanOp(BooleanOp::Difference, target, swept, tol).closed; show("diff", diff);
  auto in = offsetContours(diff, -0.005, JoinType::Round, fine); show("diff inset .005", in);
  auto op = offsetContours(in, 0.005, JoinType::Round, fine); show("opened", op);
}
