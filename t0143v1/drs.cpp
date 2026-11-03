// Verifier T-0143: drill scenes + orderCuts nesting; prints per-scene hashes for new/old diff.
#include <cstdio>
#include <cmath>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/cut_order.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static tooling::Tool tool(double d, bool mill) { tooling::Tool t; t.id = "d"; t.name = "D"; t.type = mill ? tooling::ToolType::EndMill : tooling::ToolType::Drill; t.diameter = d; t.includedAngle = 118 * kPi / 180;
  t.defaultCuttingData.toolNumber = 1; t.defaultCuttingData.feedRate = 600; t.defaultCuttingData.plungeRate = 200; t.defaultCuttingData.spindleSpeed = 10000; t.defaultCuttingData.passDepth = U(0.3, 3); return t; }
static Contour ringLoop(Vec2 c, double ro, double ri, double a, int style) {
  Contour r; r.closed = true; Vec2 po = c + Vec2::fromAngle(a, ro), pi = c + Vec2::fromAngle(a, ri);
  if (style == 0) { r.append(ArcSpan{c, ro, a, a + 2*kPi, ArcDirection::CCW}); r.append(LineSpan{po, pi}); r.append(ArcSpan{c, ri, a, a + 2*kPi, ArcDirection::CW}); r.append(LineSpan{pi, po}); }
  else if (style == 1) { Polyline pl; pl.closed = true; int n = I(6, 24); for (int i = 0; i <= n; ++i) pl.points.push_back(c + Vec2::fromAngle(a + 2*kPi*i/n, ro)); for (int i = 0; i <= n; ++i) pl.points.push_back(c + Vec2::fromAngle(a - 2*kPi*i/n, ri)); r = Contour::fromPolyline(pl); }
  else { for (auto& b : ArcSpan{c, ro, a, a + 2*kPi, ArcDirection::CCW}.toBeziers()) r.append(b); r.append(LineSpan{po, pi}); for (auto& b : ArcSpan{c, ri, a, a + 2*kPi, ArcDirection::CW}.toBeziers()) r.append(b); r.append(LineSpan{pi, po}); }
  return r;
}
static unsigned long long fnv(const std::string& o) { unsigned long long h = 1469598103934665603ULL; for (unsigned char c : o) { h ^= c; h *= 1099511628211ULL; } return h; }
int main(int argc, char** argv) {
  unsigned long seed = std::strtoul(argv[1], nullptr, 10); int N = std::atoi(argv[2]);
  for (int sc = 0; sc < N; ++sc) {
    rng.seed(seed * 1000003ULL + sc);
    std::vector<Contour> cs; std::string desc;
    int n = I(1, 8);
    for (int k = 0; k < n; ++k) {
      Vec2 c = I(0, 1) ? Vec2{(double)I(-5, 5) * 10, (double)I(-5, 5) * 10} : Vec2{U(-60, 60), U(-60, 60)};
      int kind = I(0, 6); double R = I(0, 1) ? (double)I(2, 15) : U(2, 15);
      double a = I(0, 2) == 0 ? U(0, 2*kPi) : I(0, 7) * kPi / 4;
      if (kind <= 2) { cs.push_back(ringLoop(c, R, R * U(0.2, 0.8), a, kind)); desc += "R" + std::to_string(kind); }
      else if (kind == 3) { cs.push_back(Contour::circle(c, R)); if (I(0, 1)) { cs.push_back(Contour::circle(c, R * 0.5, ArcDirection::CW)); desc += "rr"; } desc += "c"; }
      else if (kind == 4) { cs.push_back(Contour::rectangle(Box2{{c.x - R, c.y - R * 0.6}, {c.x + R, c.y + R * 0.6}})); desc += "b"; }
      else if (kind == 5) { Polyline pl; pl.closed = true; pl.points = {{c.x, c.y}, {c.x + R, c.y}, {c.x + R, c.y + 0.2*R}, {c.x + 0.2*R, c.y + 0.2*R}, {c.x + 0.2*R, c.y + 0.5*R}, {c.x + 0.2*R, c.y + 0.8*R}, {c.x + R, c.y + 0.8*R}, {c.x + R, c.y + R}, {c.x, c.y + R}}; cs.push_back(Contour::fromPolyline(pl)); desc += "C"; }
      else { Polyline pl; pl.closed = true; int m = I(3, 9); for (int i = 0; i < m; ++i) pl.points.push_back(c + Vec2::fromAngle(2*kPi*i/m, R * U(0.4, 1))); cs.push_back(Contour::fromPolyline(pl)); desc += "p"; }
    }
    DrillParams p; p.cutDepth = U(1, 10); p.peck = I(0, 1); p.peckRetract = I(0, 1) ? PeckRetract::AboveStart : PeckRetract::AbovePreviousPass; p.dwellTime = I(0, 2) ? 0 : U(0.1, 2);
    p.order.selectionOrder = I(0, 3) == 0; p.order.leftToRight = I(0, 3) == 0; p.order.grid = I(0, 4) == 0; p.startPosition = {U(-50, 50), U(-50, 50)};
    auto r = drillToolpath(cs, tool(U(1, 8), I(0, 3) == 0), std::nullopt, p);
    std::string o, dg; char b[200];
    for (auto& m : r.toolpath.moves) { snprintf(b, sizeof b, "%d %a %a %a|", (int)m.kind, m.to.x, m.to.y, m.to.z); o += b; }
    for (auto& d : r.diagnostics) dg += std::to_string((int)d.issue) + ":" + (d.contour ? std::to_string(*d.contour) : "-") + "|";
    // orderCuts on the same contours as closed items, starts at first vertex
    std::vector<OrderItem> items; for (size_t i = 0; i < cs.size(); ++i) items.push_back({cs[i], i, cs[i].startPoint(), cs[i].endPoint()});
    // add small probe items whose first-span midpoint lies on the ray of container vertices
    for (size_t i = 0; i < cs.size(); ++i) { Vec2 s = cs[i].startPoint(); Contour sq = Contour::rectangle(Box2{{s.x - U(-30, 30) - 0.5, s.y - 0.2}, {s.x + 0.0, s.y + 0.2}}); items.push_back({sq, items.size(), sq.startPoint(), sq.startPoint()}); }
    auto ord = orderCuts(items, p.order, p.startPosition, Tolerance::defaults());
    std::string os; for (auto k : ord) os += std::to_string(k) + ",";
    std::printf("D %lu %d [%s] holes %zu moves %zu mh %016llx dg %s | order %s\n", seed, sc, desc.c_str(), r.holes.size(), r.toolpath.moves.size(), fnv(o), dg.c_str(), os.c_str());
  }
}
