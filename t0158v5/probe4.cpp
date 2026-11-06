// probe4.cpp (T-0158 round 4 knife-edge probes). Outside, VectorStart, start vertex A of a CCW polygon.
//  col  : V-bit sharp (d = 3). Neighbour vertex B at distance L BEFORE A (contour order; ORDER=1 mirrors it AFTER A), turning tB
//         (0 = exactly collinear, redundant vertex). Prints |S - mitre(A)|, |S - corner(B)| and max start move under 14 scalings.
//         Owner 2026-11-05: start exactly at the tip's mitre.
//  thr  : as col, bisect tB in [0, 1e-3] for the flip of the start between B's corner and A's mitre (L env, default 20 um), then
//         the start move under +-ulp scaling at the threshold.
//  mrg  : end mill (D 6) merged region (two overlapping polygons): B at x before A (x in 0.3..12 um, turn TB default 0.02).
//         Prints |S - F| where F = the region's first join point in travel (foot of A on the incoming offset edge, at d+chordal),
//         |S-A| - (d+chordal) and max move under scaling and translation.
//  chord: end mill (D 2*d) exact offset, start vertex turning theta around tol/d (join arc length = tol: arc vs chord): |S - F_in|,
//         |S - F_out| (feet on the contour-incoming / outgoing offset edges), the start's arc-length position, max move.
//  OFF=<x> translates every scene by (x, 0.7x) (large coordinates).
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
static double OFFX = getenv("OFF") ? atof(getenv("OFF")) : 0.0;
static Contour poly(const std::vector<Vec2>& v) { Contour c; c.closed = true; for (size_t i = 0; i < v.size(); ++i) c.spans.push_back(geometry::LineSpan{v[i], v[(i + 1) % v.size()]}); return c; }
static Contour xf(Contour c, double s, Vec2 o) { for (auto& sp : c.spans) if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start * s + o; l->end = l->end * s + o; } return c; }
static tooling::Tool vb() { tooling::Tool t; t.type = tooling::ToolType::VBit; t.diameter = 20; t.includedAngle = PI / 2; return t; }
static tooling::Tool em(double d) { tooling::Tool t; t.type = tooling::ToolType::EndMill; t.diameter = d; return t; }
static ProfileParams pp(int dir, bool sharp) { ProfileParams p; p.side = ProfileSide::Outside; p.startAt = StartAt::VectorStart; p.passes.cutDepth = 3; p.passes.passDepth = 3; p.safeZ = 5;
  p.direction = (CutDirection)dir; p.corners.sharpExternal = sharp; p.corners.mitreLimit = 50; return p; }
// start nearest the first contour's start (the first rapid whose XY is nearest T)
static Vec2 startNear(const std::vector<Contour>& cs, const tooling::Tool& t, const ProfileParams& p, Vec2 T) {
  auto r = profileToolpath(cs, t, p); double bd = 1e300; Vec2 b{};
  for (auto& m : r.toolpath.moves) if (m.kind == MoveKind::Rapid && geometry::distance(m.to.xy(), T) < bd) { bd = geometry::distance(m.to.xy(), T); b = m.to.xy(); }
  return b; }
struct X { double s; Vec2 o; };
static std::vector<X> xforms(bool frames) { std::vector<X> v; double u = 1, d = 1; for (int k = 0; k < 4; ++k) { u = std::nextafter(u, 2.0); d = std::nextafter(d, 0.0); v.push_back({u, {0, 0}}); v.push_back({d, {0, 0}}); }
  for (double r : {1e-12, 1e-10, 1e-9}) { v.push_back({1 + r, {0, 0}}); v.push_back({1 - r, {0, 0}}); }
  if (frames) for (Vec2 o : {Vec2{3, -7}, Vec2{0.1, 0.3}, Vec2{1000.25, -999.75}}) v.push_back({1, o});
  return v; }
static double maxMove(const std::vector<Contour>& cs, const tooling::Tool& t, const ProfileParams& p, Vec2 T, Vec2 s0, bool frames = false) { double mx = 0;
  for (auto x : xforms(frames)) { std::vector<Contour> c2; for (auto& c : cs) c2.push_back(xf(c, x.s, x.o)); Vec2 s = startNear(c2, t, p, T * x.s + x.o);
    mx = std::max(mx, geometry::distance((s - x.o) * (1.0 / x.s), s0)); } return mx; }
static Vec2 rightOf(Vec2 u) { return Vec2{u.y, -u.x}; }
static Vec2 mitre(Vec2 T, Vec2 uIn, Vec2 uOut, double d) { Vec2 p0 = T + rightOf(uIn) * d, q0 = T + rightOf(uOut) * d; double t = (q0 - p0).cross(uOut) / uIn.cross(uOut); return p0 + uIn * t; }
static Vec2 U(Vec2 a, Vec2 b) { Vec2 w = b - a; return w * (1.0 / w.length()); }
// polygon with A at index 0, B before A (order 0) or after A (order 1, mirrored), CCW, offset by OFF
static std::vector<Vec2> shape(double tA, double tB, double L, int order, Vec2 A0 = {11.25, 3.5}) {
  Vec2 A = A0 + Vec2{OFFX, 0.7 * OFFX}; Vec2 ein{1, 0}; Vec2 eout = Vec2::fromAngle(tA, 1.0); Vec2 eP = Vec2::fromAngle(-tB, 1.0);
  Vec2 B = A - ein * L, P = B - eP * 30.0, Q = A + eout * 30.0, F = (P + Q) * 0.5 + Vec2{0, 25};
  if (order == 0) return {A, Q, F, P, B};
  auto m = [&](Vec2 q) { return Vec2{2 * A.x - q.x, q.y}; }; return {A, m(B), m(P), m(F), m(Q)}; }
int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "col"; const double d = 3.0;
  if (mode == "col" || mode == "thr") {
    std::vector<double> Ls{2e-3, 5e-3, 2e-2, 5e-2, 7e-2, 0.1}; std::vector<double> tBs{0.0, 1e-12, 1e-9, 1e-7, 1e-5, 1e-3, 0.05}; std::vector<double> tAs{0.3, 1.2};
    if (getenv("L")) Ls = {atof(getenv("L"))}; if (getenv("TB")) tBs = {atof(getenv("TB"))}; if (getenv("TA")) tAs = {atof(getenv("TA"))};
    for (double tA : tAs) for (double L : Ls) for (int order = 0; order < 2; ++order) for (int dir = 0; dir < 2; ++dir) {
      auto eval = [&](double tB, Vec2& s, double& dA, double& dB, std::vector<Vec2>& v) {
        v = shape(tA, tB, L, order); ProfileParams p = pp(dir, true); s = startNear({poly(v)}, vb(), p, v[0]);
        size_t n = v.size(); Vec2 A = v[0]; Vec2 MA = mitre(A, U(v[n - 1], A), U(A, v[1]), d);
        Vec2 B = order == 0 ? v[n - 1] : v[1]; Vec2 MB = order == 0 ? mitre(B, U(v[n - 2], B), U(B, A), d) : mitre(B, U(A, B), U(B, v[2]), d);
        dA = geometry::distance(s, MA); dB = geometry::distance(s, MB); };
      if (mode == "col") for (double tB : tBs) {
        Vec2 s; double dA, dB; std::vector<Vec2> v; eval(tB, s, dA, dB, v);
        double mm = getenv("NOPERT") ? -1 : maxMove({poly(v)}, vb(), pp(dir, true), v[0], s, true);
        if (const char* dp = getenv("DUMP")) { char nm[256]; std::snprintf(nm, sizeof nm, "%s_tA%g_L%g_tB%g_o%d_d%d.scn", dp, tA, L, tB, order, dir); writeScene(nm, {poly(v)}, pp(dir, true), 20.0, 90); }
        std::printf("col tA %.3g L %.4g tB %.3g order %d dir %d | S-mitreA %.4g S-cornerB %.4g maxMove %.3g\n", tA, L, tB, order, dir, dA, dB, mm);
      } else {
        double lo = 0, hi = 1e-3; Vec2 s; double dA, dB; std::vector<Vec2> v; eval(lo, s, dA, dB, v); bool loA = dA < 1e-9; eval(hi, s, dA, dB, v); bool hiA = dA < 1e-9;
        if (loA == hiA) { std::printf("thr tA %.3g L %.4g order %d dir %d | no flip in [0,1e-3] (atMitre %d)\n", tA, L, order, dir, (int)loA); continue; }
        for (int it = 0; it < 200 && hi - lo > 0; ++it) { double mid = 0.5 * (lo + hi); if (mid <= lo || mid >= hi) break; eval(mid, s, dA, dB, v); if ((dA < 1e-9) == loA) lo = mid; else hi = mid; }
        eval(lo, s, dA, dB, v); double mm = maxMove({poly(v)}, vb(), pp(dir, true), v[0], s, true);
        std::printf("thr tA %.3g L %.4g order %d dir %d | flip at tB %.17g (L sin %.3g) | at lo: S-mitreA %.4g S-cornerB %.4g maxMove %.4g", tA, L, order, dir, lo, L * std::sin(lo), dA, dB, mm);
        eval(hi, s, dA, dB, v); mm = maxMove({poly(v)}, vb(), pp(dir, true), v[0], s, true); std::printf(" | at hi: S-mitreA %.4g maxMove %.4g\n", dA, mm);
        if (const char* dp = getenv("DUMP")) { eval(lo, s, dA, dB, v); std::string f = std::string(dp) + "_L" + std::to_string(L) + "_o" + std::to_string(order) + "_d" + std::to_string(dir) + ".scn"; writeScene(f.c_str(), {poly(v)}, pp(dir, true), 20.0, 90); }
      }
    }
  }
  if (mode == "mrg") {
    std::vector<double> xs{3e-4, 1e-3, 2e-3, 3e-3, 4e-3, 4.5e-3, 5e-3, 6e-3, 8e-3, 1.2e-2, 3e-2}; if (getenv("XS")) { xs.clear(); for (double x = atof(getenv("XS")); x <= atof(getenv("XE")); x *= atof(getenv("XF"))) xs.push_back(x); } double tB = getenv("TB") ? atof(getenv("TB")) : 0.02;
    const double tAm = getenv("TA") ? atof(getenv("TA")) : 0.9;
    for (double x : xs) for (int order = 0; order < 2; ++order) for (int dir = 0; dir < 2; ++dir) {
      if (getenv("ORDER") && atoi(getenv("ORDER")) != order) continue; if (getenv("DIR") && atoi(getenv("DIR")) != dir) continue;
      auto v = shape(tAm, tB, x, order); Vec2 A = v[0]; Vec2 c{0, 0}; for (auto q : v) c = c + q * (1.0 / v.size());
      std::vector<Vec2> w; for (auto q : v) w.push_back(q + (c - A) * 0.5);
      std::vector<Contour> cs{poly(v), poly(w)}; ProfileParams p = pp(dir, false); const double R = 3.0 + 0.01;
      Vec2 s = startNear(cs, em(6), p, A); size_t n = v.size();
      Vec2 Fin = A + rightOf(U(v[n - 1], A)) * R, Fout = A + rightOf(U(A, v[1])) * R;
      double mm = maxMove(cs, em(6), p, A, s, true);
      if (const char* dp = getenv("DUMP")) { std::string f = std::string(dp) + "_x" + std::to_string(x) + "_o" + std::to_string(order) + "_d" + std::to_string(dir) + ".scn"; writeScene(f.c_str(), cs, p, 6.0, 0); }
      std::printf("mrg x %.3g tB %.3g order %d dir %d | |S-A|-R %.3g S-Fin %.4g S-Fout %.4g maxMove %.3g\n", x, tB, order, dir, geometry::distance(s, A) - R, geometry::distance(s, Fin), geometry::distance(s, Fout), mm);
    }
  }
  if (mode == "chord") {
    const double tol = 1e-3;
    std::vector<double> fs{0.5, 0.9, 0.999, 1.0, 1.001, 1.1, 2.0};
    for (double f : fs) for (int dir = 0; dir < 2; ++dir) {
      double th = f * tol / d; auto v = shape(th, 0.4, 5.0, 0); ProfileParams p = pp(dir, false); Vec2 A = v[0]; size_t n = v.size();
      Vec2 s = startNear({poly(v)}, em(2 * d), p, A);
      Vec2 Fin = A + rightOf(U(v[n - 1], A)) * d, Fout = A + rightOf(U(A, v[1])) * d;
      double mm = maxMove({poly(v)}, em(2 * d), p, A, s, true);
      std::printf("chord f %.4g theta %.4g dir %d | |S-A|-d %.3g S-Fin %.4g S-Fout %.4g maxMove %.3g\n", f, th, dir, geometry::distance(s, A) - d, geometry::distance(s, Fin), geometry::distance(s, Fout), mm);
    }
  }
}
