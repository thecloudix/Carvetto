// T-0158 round-2 verifier: sharp-corner (mitre / chamfer) families, written as .scn into DIR (argv[1]), N per family
// (argv[2]), seed argv[3]. All V-bit with corners.sharpExternal unless noted.
//  mt   : star, start at a tip, Outside, mitre limit from {1.0,1.1,1.3,1.6,2,3,50} (many chamfers), Climb/Conv
//  mtc  : regular polygon/star tips whose mitre ratio sits close to the limit (both sides of it)
//  mref : star, start at a reflex vertex, Outside (no join span; on-vector target with a junction tie)
//  min  : star, start at a tip, Inside (convex tip is an inside corner for the cut)
//  mon  : star, start at a tip, On (cut distance 0)
//  mloop: 2- and 3-span loops: D-shape (arc+line, either order), lens (two arcs), D with low limit (chamfer at start)
//  mtri : triangle (3 spans), start at a vertex
//  mshort: tip followed / preceded by an edge of length 1e-4..5e-3
//  mflat: start vertex turning by a tiny angle (1e-2..1e-8 rad), convex or reflex
//  marc : star with arc edges (sag +-), start at a tip (arcs meeting at a mitre)
//  mmrg : two overlapping stars (merged region -> Clipper offset with mitre/square), start at a tip
//  rmrg : two overlapping stars, end mill (merged region, round join as polygon), start at a tip
//  mopen: open polyline, V-bit sharp, VectorStart, Outside/On
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
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static const double PI = 3.14159265358979323846;
static geometry::Span edge(Vec2 a, Vec2 b, double sag) {
  if (sag == 0.0) return geometry::LineSpan{a, b};
  Vec2 d{b.x - a.x, b.y - a.y}; double L = std::hypot(d.x, d.y), half = 0.5 * L; Vec2 left{-d.y / L, d.x / L};
  double s = sag * L, t = (half * half - s * s) / (2 * s); Vec2 c{(a.x + b.x) / 2 + left.x * t, (a.y + b.y) / 2 + left.y * t};
  return geometry::ArcSpan::fromCentre(c, a, b, s > 0 ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW); }
static Contour poly(const std::vector<Vec2>& v, double sagMax = 0, bool closed = true) { Contour c; c.closed = closed;
  size_t m = closed ? v.size() : v.size() - 1;
  for (size_t i = 0; i < m; ++i) c.spans.push_back(edge(v[i], v[(i + 1) % v.size()], sagMax > 0 && I(0, 1) ? U(-sagMax, sagMax) : 0.0)); return c; }
static std::vector<Vec2> star(Vec2 c, double ro, double ri, int n, double rot) { std::vector<Vec2> v;
  for (int j = 0; j < 2 * n; ++j) { double a = rot + j * PI / n; double r = j % 2 ? ri : ro; v.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)}); } return v; }
static ProfileParams base() { ProfileParams p; p.passes.cutDepth = U(1, 4); p.passes.passDepth = U(0.5, 4); p.safeZ = 5; p.startAt = StartAt::VectorStart;
  p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional; p.separateLastPass = I(0, 2) == 0; p.lastPassAllowance = U(0.05, 0.5);
  p.allowance = I(0, 2) == 0 ? 0.0 : U(-0.3, 0.6);
  p.tabs.enabled = I(0, 1); p.tabs.count = I(2, 8); p.tabs.length = U(1, 6); p.tabs.thickness = U(0.3, p.passes.cutDepth);
  p.leads.type = (LeadType)I(0, 2); p.leads.length = U(0.5, 5); p.leads.radius = U(0.5, 4); p.ramp.type = (RampType)I(0, 3); p.ramp.distance = U(1, 20);
  p.corners.sharpExternal = true; const double L[] = {1.0, 1.1, 1.3, 1.6, 2.0, 3.0, 50.0}; p.corners.mitreLimit = L[I(0, 6)]; return p; }
static int vdeg() { return std::vector<int>{60, 90, 120}[I(0, 2)]; }
int main(int argc, char** argv) {
  std::string D = argv[1]; int N = atoi(argv[2]); rng.seed(strtoull(argv[3], 0, 10));
  auto W = [&](const std::string& n, int k, const std::vector<Contour>& cs, const ProfileParams& p, double dia, int vd) {
    char f[512]; std::snprintf(f, sizeof f, "%s/%s_%03d.scn", D.c_str(), n.c_str(), k); writeScene(f, cs, p, dia, vd); };
  for (int k = 0; k < N; ++k) {
    Vec2 c{U(-80, 80), U(-80, 80)}; double ro = U(10, 40); int n = I(3, 8);
    { auto v = star(c, ro, ro * U(0.3, 0.8), n, U(0, 2 * PI)); ProfileParams p = base(); p.side = ProfileSide::Outside; W("mt", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { // tip angle chosen so the mitre ratio 1/sin(half) is within +-2% of the limit
      ProfileParams p = base(); p.side = ProfileSide::Outside; p.corners.mitreLimit = U(1.05, 4.0); double r = p.corners.mitreLimit * U(0.98, 1.02);
      double half = std::asin(std::min(1.0, 1.0 / r)); // tip interior half-angle
      int m = I(3, 7); double rot = U(0, 2 * PI); // star whose tips have interior angle 2*half: tip + two points along edges
      std::vector<Vec2> v; for (int j = 0; j < m; ++j) { double a = rot + 2 * PI * j / m; Vec2 tip{c.x + ro * std::cos(a), c.y + ro * std::sin(a)}; v.push_back(tip);
        double ia = a + PI - half; // edge direction leaving the tip
        double L = ro * U(0.3, 0.5); Vec2 q{tip.x + L * std::cos(ia), tip.y + L * std::sin(ia)}; v.push_back(q); }
      // fix ordering: we need CCW; build via angles anyway (not exact interior angles for every tip, but tip 0 is)
      W("mtc", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { auto v = star(c, ro, ro * U(0.4, 0.8), n, U(0, 2 * PI)); std::rotate(v.begin(), v.begin() + 1, v.end()); ProfileParams p = base(); p.side = ProfileSide::Outside; W("mref", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { auto v = star(c, ro, ro * U(0.5, 0.8), n, U(0, 2 * PI)); ProfileParams p = base(); p.side = ProfileSide::Inside; W("min", k, {poly(v)}, p, U(6, 20), vdeg()); }
    { auto v = star(c, ro, ro * U(0.4, 0.8), n, U(0, 2 * PI)); ProfileParams p = base(); p.side = ProfileSide::On; W("mon", k, {poly(v)}, p, U(6, 20), vdeg()); }
    { ProfileParams p = base(); p.side = I(0, 3) ? ProfileSide::Outside : ProfileSide::Inside; double R = U(15, 40), a0 = U(0, 2 * PI), sw = U(0.5, 1.7) * PI; int kind = k % 4; Contour d; d.closed = true;
      geometry::ArcSpan arc{c, R, a0, a0 + sw, geometry::ArcDirection::CCW};
      if (kind == 0) { d.spans.push_back(arc); d.spans.push_back(geometry::LineSpan{arc.endPoint(), arc.startPoint()}); }
      else if (kind == 1) { d.spans.push_back(geometry::LineSpan{arc.endPoint(), arc.startPoint()}); d.spans.push_back(arc); }
      else if (kind == 2) { Vec2 A = arc.startPoint(), B = arc.endPoint(); d.spans.push_back(edge(A, B, -U(0.1, 0.4))); d.spans.push_back(edge(B, A, -U(0.1, 0.4))); }
      else { d.spans.push_back(arc); d.spans.push_back(geometry::LineSpan{arc.endPoint(), arc.startPoint()}); p.corners.mitreLimit = U(1.0, 1.2); }
      W("mloop", k, {d}, p, U(6, 25), vdeg()); }
    { std::vector<Vec2> v; for (int j = 0; j < 3; ++j) { double a = U(0, 2 * PI) ; (void)a; }
      double a0 = U(0, 2 * PI); v = {{c.x + ro * std::cos(a0), c.y + ro * std::sin(a0)}, {c.x + ro * std::cos(a0 + U(1.5, 2.5)), c.y + ro * std::sin(a0 + U(1.5, 2.5))}};
      double a2 = a0 + U(3.6, 5.0); v.push_back({c.x + ro * std::cos(a2), c.y + ro * std::sin(a2)});
      ProfileParams p = base(); p.side = I(0, 2) ? ProfileSide::Outside : ProfileSide::Inside; W("mtri", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); const double Ls[] = {1e-4, 0.5e-3, 0.9e-3, 0.999e-3, 1.0e-3, 1.001e-3, 1.5e-3, 2e-3, 5e-3};
      double L = Ls[k % 9]; bool after = I(0, 1); Vec2 nb = after ? v[1] : v.back(); Vec2 dd{nb.x - v[0].x, nb.y - v[0].y}; double a = std::atan2(dd.y, dd.x) + U(-1.0, 1.0);
      Vec2 q{v[0].x + L * std::cos(a), v[0].y + L * std::sin(a)};
      if (after) v.insert(v.begin() + 1, q); else v.push_back(q);
      ProfileParams p = base(); p.side = ProfileSide::Outside; W("mshort", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { std::vector<Vec2> v; int m = I(4, 9); for (int j = 0; j < m; ++j) { double a = 2 * PI * j / m; v.push_back({c.x + ro * std::cos(a), c.y + ro * std::sin(a)}); }
      Vec2 a = v.back(), b = v[1]; const double T[] = {1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-8}; double t = T[k % 6] * (I(0, 1) ? 1 : -1);
      Vec2 mid{(a.x + b.x) / 2, (a.y + b.y) / 2}; Vec2 o{mid.x - c.x, mid.y - c.y}; double ol = std::hypot(o.x, o.y), L = std::hypot(b.x - a.x, b.y - a.y);
      v[0] = {mid.x + o.x / ol * t * L, mid.y + o.y / ol * t * L};
      ProfileParams p = base(); p.side = ProfileSide::Outside; W("mflat", k, {poly(v)}, p, U(6, 25), vdeg()); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); ProfileParams p = base(); p.side = ProfileSide::Outside; W("marc", k, {poly(v, 0.08)}, p, U(6, 25), vdeg()); }
    { auto v = star(c, ro, ro * U(0.4, 0.75), n, U(0, 2 * PI)); Vec2 c2{c.x + ro * U(0.5, 1.2), c.y + ro * U(-0.5, 0.5)}; auto w = star(c2, ro * U(0.6, 1.0), ro * U(0.3, 0.6), I(3, 7), U(0, 2 * PI));
      ProfileParams p = base(); p.side = ProfileSide::Outside; W("mmrg", k, {poly(v), poly(w)}, p, U(6, 25), vdeg());
      ProfileParams q = base(); q.side = ProfileSide::Outside; q.corners.sharpExternal = false; W("rmrg", k, {poly(v), poly(w)}, q, U(1, 10), 0); }
    { std::vector<Vec2> v{c}; int m = I(2, 6); for (int j = 0; j < m; ++j) v.push_back({v.back().x + U(-20, 20), v.back().y + U(-20, 20)});
      ProfileParams p = base(); p.side = I(0, 1) ? ProfileSide::Outside : ProfileSide::On; W("mopen", k, {poly(v, 0, false)}, p, U(6, 25), vdeg()); }
  }
}
