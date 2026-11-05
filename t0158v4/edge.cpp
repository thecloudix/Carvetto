// T-0158 r2 verifier edge probes for the sharp-corner rule (V-bit, sharpExternal, Outside, VectorStart).
//  chamfer: tip squared off (limit 1.0x..), Climb and Conventional: is the start the chamfer corner that comes first in
//           travel (first feed runs along the chamfer to its other corner)?
//  shortpre/shortpost: a short edge of length L (convex turn) just before / after the start tip: where does the cut start
//           (distance to the tip's mitre corner) and is it stable under ulp / 1e-9 scaling?
//  gap: start vertex a gap g off the closing span's end (g around the 1e-3 tolerance)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static Contour poly(const std::vector<Vec2>& v) { Contour c; c.closed = true; for (size_t i = 0; i < v.size(); ++i) c.spans.push_back(geometry::LineSpan{v[i], v[(i + 1) % v.size()]}); return c; }
static Contour scale(Contour c, double s) { for (auto& sp : c.spans) if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start * s; l->end = l->end * s; } return c; }
static tooling::Tool vb() { tooling::Tool t; t.type = tooling::ToolType::VBit; t.diameter = 20; t.includedAngle = PI / 2; return t; }
static ProfileParams pp(int dir, double limit) { ProfileParams p; p.side = ProfileSide::Outside; p.startAt = StartAt::VectorStart; p.passes.cutDepth = 3; p.passes.passDepth = 3; p.safeZ = 5;
  p.direction = (CutDirection)dir; p.corners.sharpExternal = true; p.corners.mitreLimit = limit; return p; }
struct S { Vec2 at, next; bool ok = false; size_t moves = 0; };
static S run(const std::vector<Contour>& cs, const ProfileParams& p) { auto r = profileToolpath(cs, vb(), p); S s; s.moves = r.toolpath.moves.size();
  if (r.toolpath.moves.empty()) return s; s.at = r.toolpath.moves.front().to.xy();
  for (auto& m : r.toolpath.moves) if ((m.kind == MoveKind::Feed || isArc(m.kind)) && geometry::distance(m.to.xy(), s.at) > 1e-9) { s.next = m.to.xy(); s.ok = true; break; }
  return s; }
static std::vector<double> scales() { std::vector<double> o; double u = 1, d = 1; for (int k = 0; k < 4; ++k) { u = std::nextafter(u, 2.0); d = std::nextafter(d, 0.0); o.push_back(u); o.push_back(d); }
  for (double r : {1e-12, 1e-10, 1e-9}) { o.push_back(1 + r); o.push_back(1 - r); } return o; }
static double maxMove(const std::vector<Contour>& cs, const ProfileParams& p, Vec2 at0) { double mx = 0;
  for (double k : scales()) { std::vector<Contour> c2; for (auto& c : cs) c2.push_back(scale(c, k)); ProfileParams q = p; S s = run(c2, q); mx = std::max(mx, geometry::distance(s.at * (1.0 / k), at0)); } return mx; }
int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "all";
  const double d = 3.0; // V-bit 90deg at depth 3: radius 3 (allowance 0)
  // star-ish: tip T at (40,10) with interior angle a; polygon T, B, C, A (CCW)
  auto tipPoly = [&](double half, Vec2 T) { // edges leave T at angles PI+-half from +x (tip pointing +x)
    Vec2 A = T + Vec2::fromAngle(PI - half, 30), B = T + Vec2::fromAngle(PI + half, 30); Vec2 C = T + Vec2{-45, 0};
    return std::vector<Vec2>{T, A, C, B}; }; // CCW? T -> A (upper-left) -> C -> B (lower-left): that's CCW for tip pointing +x
  if (mode == "all" || mode == "chamfer") {
    for (int dir = 0; dir < 2; ++dir) for (double halfDeg : {20.0, 35.0, 50.0}) for (double lim : {1.0, 1.2, 1.5}) {
      double half = halfDeg * PI / 180; Vec2 T{40.3, 10.7}; auto v = tipPoly(half, T); ProfileParams p = pp(dir, lim);
      bool squared = 1.0 / std::sin(half) > lim;
      S s = run({poly(v)}, p);
      // chamfer corners: on the line x = T.x + d (perpendicular to the bisector +x), y = +-(d/sin... ) : corner where offset line meets x = T.x + d
      // offset of edge T->A (direction PI-half), outward normal: for CCW polygon outward is right of travel
      Vec2 uA = Vec2::fromAngle(PI - half, 1), nA{uA.y, -uA.x}; Vec2 uB = Vec2::fromAngle(PI + half, 1), nB{-uB.y, uB.x};
      // offset line of T->A: points T + nA*d + t*uA; meets x = T.x + d at t = (d - nA.x*d)/uA.x
      double tA = (d - nA.x * d) / uA.x, tB = (d - nB.x * d) / uB.x; Vec2 cA = T + nA * d + uA * tA, cB = T + nB * d + uB * tB;
      Vec2 mit = T + Vec2{d / std::sin(half), 0};
      std::printf("chamfer dir %d half %.0f limit %.1f squared %d start (%.6f,%.6f) | dist to mitre %.3g cornerA %.3g cornerB %.3g | next (%.6f,%.6f) next-is-otherCorner %d\n",
        dir, halfDeg, lim, (int)squared, s.at.x, s.at.y, geometry::distance(s.at, mit), geometry::distance(s.at, cA), geometry::distance(s.at, cB), s.next.x, s.next.y,
        (int)((geometry::distance(s.at, cA) < 1e-6 && geometry::distance(s.next, cB) < 1e-6) || (geometry::distance(s.at, cB) < 1e-6 && geometry::distance(s.next, cA) < 1e-6)));
    }
  }
  if (mode == "all" || mode == "short") {
    std::vector<double> Ls{0.0005, 0.0009, 0.00099, 0.001, 0.00101, 0.0011, 0.0015, 0.002, 0.003, 0.01, 0.02, 0.03, 0.04, 0.06, 0.1, 0.3};
    if (getenv("LS")) Ls = {atof(getenv("LS"))};
    int turnSel = getenv("TURN") ? atoi(getenv("TURN")) : -1; int preSel = getenv("PRE") ? atoi(getenv("PRE")) : -1;
    for (int pre = 0; pre < 2; ++pre) for (double turnDeg : {2.0, 10.0, 30.0}) for (double L : Ls) for (int dir = 0; dir < 2; ++dir) { if (getenv("DIRSEL") && atoi(getenv("DIRSEL")) != dir) continue;
      if ((turnSel >= 0 && turnSel != (int)turnDeg) || (preSel >= 0 && preSel != pre)) continue;
      double half = 35 * PI / 180; Vec2 T{40.3, 10.7}; auto v = tipPoly(half, T);
      // insert a vertex at distance L from T along the edge, then bend: the short edge T->Q (post) or Q->T (pre), with a convex turn of turnDeg at Q
      std::vector<Vec2> w;
      if (!pre) { Vec2 Q = T + Vec2::fromAngle(PI - half - turnDeg * PI / 180, L); w = {T, Q, v[1], v[2], v[3]}; }
      else { Vec2 Q = T + Vec2::fromAngle(PI + half + turnDeg * PI / 180, L); w = {T, v[1], v[2], v[3], Q}; }
      ProfileParams p = pp(dir, 50); S s = run({poly(w)}, p);
      if (const char* dp = getenv("DUMP")) writeScene(dp, {poly(w)}, p, 20.0, 90);
      // actual mitre corners of the split tip: at T (turn = tip turn - turnDeg) and at Q
      Vec2 mit = T + Vec2{d / std::sin(half), 0};
      if (getenv("DUMPMOVES")) { auto r = profileToolpath({poly(w)}, vb(), p); for (int i = 0; i < 6 && i < (int)r.toolpath.moves.size(); ++i) std::printf("   M %d %.9f %.9f %.4f\n", i, r.toolpath.moves[i].to.x, r.toolpath.moves[i].to.y, r.toolpath.moves[i].to.z); }
      double mv = maxMove({poly(w)}, p, s.at);
      std::printf("short %s turn %.0f L %.3f dir %d start-to-tipMitre %.4g |start-T|-d %.4g maxMoveUnderPert %.3g\n", pre ? "pre " : "post", turnDeg, L, dir, geometry::distance(s.at, mit), geometry::distance(s.at, T) - d, mv);
    }
  }
  if (mode == "all" || mode == "gap") {
    std::vector<double> Gs{0.0, 1e-6, 1e-5, 1e-4, 0.5e-3, 0.9e-3, 0.99e-3, 0.999e-3, 1e-3, 1.001e-3, 1.01e-3, 1.1e-3, 2e-3}; if (getenv("GS")) Gs = {atof(getenv("GS"))};
    for (double g : Gs) for (double ga : {0.0, PI / 2, PI}) {
      double half = 35 * PI / 180; Vec2 T{40.3, 10.7}; auto v = tipPoly(half, T); Contour c = poly(v);
      auto& l = std::get<geometry::LineSpan>(c.spans.back()); l.end = T + Vec2::fromAngle(ga, g);
      ProfileParams p = pp(0, 50); S s = run({c}, p); if (const char* dp = getenv("DUMP")) writeScene(dp, {c}, p, 20.0, 90); Vec2 mit = T + Vec2{d / std::sin(half), 0};
      std::printf("gap %.6g ang %.2f start-to-tipMitre %.4g |start-T|-d %.4g maxMoveUnderPert %.3g\n", g, ga, geometry::distance(s.at, mit), geometry::distance(s.at, T) - d, maxMove({c}, p, s.at));
    }
  }
}
