// T-0158 round-3 verifier: families aimed at the round-3 rule (sharpest junction of the near run, kTipTurnTie = 1e-3 rad,
// merged-region slack). Writes .scn into DIR (argv[1]), N per family (argv[2]), seed argv[3]. V-bit sharpExternal unless noted.
//  fpoly : fine convex polyline (N-gon / ellipse polygon, 200..6000 vertices) started at a vertex (neighbours within tol)
//  split : star whose start tip is split into two corners (turn shares 0.3..0.7) joined by an edge of 1e-3..5e-2 mm
//  sharpnb: start vertex turning 0.02..0.4 rad next to a neighbour turning 1..3x as much at 1e-3..2e-2 mm
//  rmrg2 : two overlapping stars, end mill (merged region, polygonal round join), Climb/Conventional, start at a tip
//  vmrg2 : same, V-bit sharp (region mitre / squared), mitre limit 1..50
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static std::mt19937_64 rng; static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double LU(double a, double b) { return std::exp(U(std::log(a), std::log(b))); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static const double PI = 3.14159265358979323846;
static Contour poly(const std::vector<Vec2>& v) { Contour c; c.closed = true; for (size_t i = 0; i < v.size(); ++i) c.spans.push_back(geometry::LineSpan{v[i], v[(i + 1) % v.size()]}); return c; }
static std::vector<Vec2> star(Vec2 c, double ro, double ri, int n, double rot) { std::vector<Vec2> v;
  for (int j = 0; j < 2 * n; ++j) { double a = rot + j * PI / n; double r = j % 2 ? ri : ro; v.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)}); } return v; }
static ProfileParams base() { ProfileParams p; p.passes.cutDepth = U(1, 4); p.passes.passDepth = U(0.5, 4); p.safeZ = 5; p.startAt = StartAt::VectorStart; p.side = ProfileSide::Outside;
  p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional; p.separateLastPass = I(0, 2) == 0; p.lastPassAllowance = U(0.05, 0.5);
  p.allowance = I(0, 2) == 0 ? 0.0 : U(-0.3, 0.6);
  p.tabs.enabled = I(0, 1); p.tabs.count = I(2, 8); p.tabs.length = U(1, 6); p.tabs.thickness = U(0.3, p.passes.cutDepth);
  p.corners.sharpExternal = true; const double L[] = {1.0, 1.3, 2.0, 3.0, 50.0}; p.corners.mitreLimit = L[I(0, 4)]; return p; }
static int vdeg() { return std::vector<int>{60, 90, 120}[I(0, 2)]; }
// insert a split at vertex 0 of a CCW polygon: replace v0 by two vertices L apart whose turns share the corner's turn
static std::vector<Vec2> splitTip(std::vector<Vec2> v, double share, double L) {
  Vec2 a = v.back(), t = v[0], b = v[1]; Vec2 ui = (t - a) * (1.0 / (t - a).length()), uo = (b - t) * (1.0 / (b - t).length());
  double turn = std::atan2(ui.cross(uo), ui.dot(uo)); double mid = std::atan2(ui.y, ui.x) + share * turn; Vec2 um = Vec2::fromAngle(mid, 1.0);
  // vertices P (on a->t side) and Q (on t->b side) with Q - P = L um and P, Q on the original edges
  // P = t - s ui, Q = t + r uo, Q - P = s ui + r uo = L um -> solve
  double det = ui.cross(uo); double s = L * um.cross(uo) / det, r = L * ui.cross(um) / det;
  std::vector<Vec2> w{t - ui * s, t + uo * r}; w.insert(w.end(), v.begin() + 1, v.end()); return w; }
int main(int argc, char** argv) {
  std::string D = argv[1]; int N = atoi(argv[2]); rng.seed(strtoull(argv[3], 0, 10));
  auto W = [&](const std::string& n, int k, const std::vector<Contour>& cs, const ProfileParams& p, double dia, int vd) {
    char f[512]; std::snprintf(f, sizeof f, "%s/%s_%03d.scn", D.c_str(), n.c_str(), k); writeScene(f, cs, p, dia, vd); };
  for (int k = 0; k < N; ++k) {
    Vec2 c{U(-80, 80), U(-80, 80)}; double ro = U(10, 40); int n = I(3, 8);
    { int m = I(200, 6000); double R = U(10, 200), e = I(0, 1) ? 1.0 : U(0.4, 0.9), rot = U(0, 2 * PI); std::vector<Vec2> v;
      for (int j = 0; j < m; ++j) { double a = 2 * PI * j / m; Vec2 q{R * std::cos(a), e * R * std::sin(a)}; v.push_back(c + Vec2{q.x * std::cos(rot) - q.y * std::sin(rot), q.x * std::sin(rot) + q.y * std::cos(rot)}); }
      std::rotate(v.begin(), v.begin() + I(0, m - 1), v.end()); ProfileParams p = base(); p.tabs.enabled = false; W("fpoly", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { auto v = star(c, ro, ro * U(0.3, 0.7), n, U(0, 2 * PI)); auto w = splitTip(v, U(0.3, 0.7), LU(1e-3, 5e-2)); ProfileParams p = base(); W("split", k, {poly(w)}, p, U(6, 25), vdeg()); }
    { // big convex blob with a start vertex turning tA and a neighbour turning tB at distance L
      double tA = LU(0.02, 0.4), tB = tA * U(1.0, 3.0), L = LU(1e-3, 2e-2); bool before = I(0, 1); Vec2 A{c.x, c.y}; Vec2 ein{1, 0};
      std::vector<Vec2> v;
      if (before) { Vec2 eP = Vec2::fromAngle(-tB, 1.0), eout = Vec2::fromAngle(tA, 1.0); Vec2 B = A - ein * L, P = B - eP * 30.0, Q = A + eout * 30.0; v = {A, Q, (P + Q) * 0.5 + Vec2{0, 25}, P, B}; }
      else { Vec2 eQ = Vec2::fromAngle(tA + tB, 1.0), e1 = Vec2::fromAngle(tA, 1.0); Vec2 P = A - ein * 30.0, B = A + e1 * L, Q = B + eQ * 30.0; v = {A, B, Q, (P + Q) * 0.5 + Vec2{0, 25}, P}; }
      double rot = U(0, 2 * PI); for (auto& q : v) { Vec2 r = q - A; q = A + Vec2{r.x * std::cos(rot) - r.y * std::sin(rot), r.x * std::sin(rot) + r.y * std::cos(rot)}; }
      ProfileParams p = base(); p.corners.mitreLimit = 50; W("sharpnb", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); Vec2 c2 = c + (c - v[0]) * U(0.3, 0.8); auto w = star(c2, ro * U(0.6, 1.0), ro * U(0.3, 0.6), I(3, 7), U(0, 2 * PI));
      ProfileParams q = base(); q.corners.sharpExternal = false; W("rmrg2", k, {poly(v), poly(w)}, q, U(1, 10), 0);
      ProfileParams p = base(); W("vmrg2", k, {poly(v), poly(w)}, p, U(6, 25), vdeg()); }
  }
}
