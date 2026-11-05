// T-0158 verifier: write tie-prone .scn scenes into DIR (argv[1]), N per family (argv[2]), seed argv[3].
// Families: star (lines, tip start, end mill, Outside), bez (Bezier edges meeting at the start tip), inref (Inside,
// start at a reflex vertex), mitre (V-bit sharp external corners, tip start), circ (circle, bbox Centre / Optimise from
// centre), dup (start vertex followed by a tiny edge of length near the tolerance), gap (closing gap near the tolerance),
// flat (start vertex turning by a tiny angle: near-zero-sweep join), poly (random polygon, start at random vertex).
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
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static const double PI = 3.14159265358979323846;
static Contour poly(const std::vector<Vec2>& v, bool bez = false) { Contour c; c.closed = true;
  for (size_t i = 0; i < v.size(); ++i) { Vec2 a = v[i], b = v[(i + 1) % v.size()];
    if (bez && I(0, 1)) { Vec2 d{b.x - a.x, b.y - a.y}; Vec2 n{d.y, -d.x}; double s = U(-0.3, 0.3);
      c.spans.push_back(geometry::BezierSpan{a, {a.x + d.x / 3 + n.x * s, a.y + d.y / 3 + n.y * s}, {a.x + 2 * d.x / 3 + n.x * s * U(0, 1), a.y + 2 * d.y / 3 + n.y * s * U(0, 1)}, b}); }
    else c.spans.push_back(geometry::LineSpan{a, b}); }
  return c; }
static std::vector<Vec2> star(Vec2 c, double ro, double ri, int n, double rot) { std::vector<Vec2> v;
  for (int j = 0; j < 2 * n; ++j) { double a = rot + j * PI / n; double r = j % 2 ? ri : ro; v.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)}); } return v; }
static ProfileParams base() { ProfileParams p; p.passes.cutDepth = U(2, 8); p.passes.passDepth = U(1, 4); p.safeZ = 5; p.startAt = StartAt::VectorStart;
  p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional; p.separateLastPass = I(0, 2) == 0; p.lastPassAllowance = U(0.05, 0.5);
  p.allowance = I(0, 2) == 0 ? 0.0 : U(-0.3, 0.6);
  p.tabs.enabled = I(0, 1); p.tabs.count = I(2, 8); p.tabs.length = U(1, 6); p.tabs.thickness = U(0.5, p.passes.cutDepth);
  p.leads.type = (LeadType)I(0, 2); p.leads.length = U(0.5, 5); p.leads.radius = U(0.5, 4); p.ramp.type = (RampType)I(0, 3); p.ramp.distance = U(1, 20); return p; }
int main(int argc, char** argv) {
  std::string D = argv[1]; int N = atoi(argv[2]); rng.seed(strtoull(argv[3], 0, 10));
  auto W = [&](const std::string& n, int k, const std::vector<Contour>& cs, const ProfileParams& p, double dia, int vd) {
    char f[512]; std::snprintf(f, sizeof f, "%s/%s_%03d.scn", D.c_str(), n.c_str(), k); writeScene(f, cs, p, dia, vd); };
  for (int k = 0; k < N; ++k) {
    Vec2 c{U(-80, 80), U(-80, 80)}; double ro = U(10, 40); int n = I(3, 8);
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); ProfileParams p = base(); p.side = ProfileSide::Outside; W("star", k, {poly(v)}, p, U(1, 10), 0); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); ProfileParams p = base(); p.side = ProfileSide::Outside; W("bez", k, {poly(v, true)}, p, U(1, 10), 0); }
    { auto v = star(c, ro, ro * U(0.5, 0.75), n, U(0, 2 * PI)); std::rotate(v.begin(), v.begin() + 1, v.end()); ProfileParams p = base(); p.side = ProfileSide::Inside; W("inref", k, {poly(v)}, p, U(1, 4), 0); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); ProfileParams p = base(); p.side = ProfileSide::Outside; p.corners.sharpExternal = true; p.corners.mitreLimit = I(0,1) ? 2.0 : 50.0; W("mitre", k, {poly(v)}, p, U(6, 20), std::vector<int>{60, 90, 120}[I(0, 2)]); }
    { ProfileParams p = base(); p.side = I(0, 1) ? ProfileSide::Outside : ProfileSide::Inside; double r = U(4, 30);
      if (I(0, 1)) { p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::Centre; } else { p.startAt = StartAt::Optimise; p.startPosition = c; }
      W("circ", k, {Contour::circle(c, r, I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW)}, p, U(1, 6), 0); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); const double Ls[] = {0.2e-3, 0.5e-3, 0.9e-3, 0.999e-3, 1.0e-3, 1.001e-3, 1.1e-3, 2e-3, 5e-3};
      double L = Ls[k % 9]; Vec2 d{v[1].x - v[0].x, v[1].y - v[0].y}; double dl = std::hypot(d.x, d.y); double a = std::atan2(d.y, d.x) + U(-1.2, 1.2);
      v.insert(v.begin() + 1, Vec2{v[0].x + L * std::cos(a), v[0].y + L * std::sin(a)}); (void)dl;
      ProfileParams p = base(); p.side = ProfileSide::Outside; W("dup", k, {poly(v)}, p, U(1, 10), 0); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); Contour cc = poly(v); const double G[] = {0.1e-3, 0.5e-3, 0.9e-3, 0.999e-3};
      auto& l = std::get<geometry::LineSpan>(cc.spans.back()); double g = G[k % 4], a = U(0, 2 * PI); l.end = {l.end.x + g * std::cos(a), l.end.y + g * std::sin(a)};
      ProfileParams p = base(); p.side = ProfileSide::Outside; W("gap", k, {cc}, p, U(1, 10), 0); }
    { std::vector<Vec2> v; int m = I(4, 9); for (int j = 0; j < m; ++j) { double a = 2 * PI * j / m; v.push_back({c.x + ro * std::cos(a), c.y + ro * std::sin(a)}); }
      // the start vertex turns by a tiny angle: put it (nearly) on the chord between its neighbours
      Vec2 a = v.back(), b = v[1]; const double T[] = {1e-3, 1e-5, 1e-7, 1e-9}; double t = T[k % 4];
      Vec2 mid{(a.x + b.x) / 2, (a.y + b.y) / 2}; Vec2 o{mid.x - c.x, mid.y - c.y}; double ol = std::hypot(o.x, o.y);
      v[0] = {mid.x + o.x / ol * t * std::hypot(b.x - a.x, b.y - a.y), mid.y + o.y / ol * t * std::hypot(b.x - a.x, b.y - a.y)};
      ProfileParams p = base(); p.side = ProfileSide::Outside; W("flat", k, {poly(v)}, p, U(1, 10), 0); }
    { std::vector<Vec2> v; int m = I(5, 12); for (int j = 0; j < m; ++j) { double a = 2 * PI * j / m + U(-0.2, 0.2); double r = ro * U(0.5, 1.0); v.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)}); }
      ProfileParams p = base(); p.side = I(0, 1) ? ProfileSide::Outside : ProfileSide::Inside; W("poly", k, {poly(v, I(0, 1))}, p, U(1, 10), 0); }
  }
}
