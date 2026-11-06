// probe5.cpp (T-0158 round-5 verifier): knife-edges of the own-edge corner rule. Outside, VectorStart, start vertex A (index 0)
// of a CCW polygon, d = 3. OFF=x translates by (x, 0.7x). Prints the start's distance to the expected corner point and the max
// start move under 4 ulp up/down, 1+-1e-12/1e-10/1e-9 scaling and 3 translations (maxMove).
//  split: extra exactly collinear vertex at s before A on the in-edge (order 0) or after A on the out-edge (order 1): a sliver of
//         A's own edge offset next to the mitre (labelling band / on-circle threshold). V-bit sharp: |S - mitre(A)|;
//         end mill (TOOL=em): |S - F_in|, |S - F_out| (feet of A on its two edge offsets).
//  short: neighbour B at L before A (order 0) / after (order 1) turning tB (negative = reflex: consumes A's short edge offset),
//         V-bit sharp and end mill.
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
static double OFFX = getenv("OFF") ? atof(getenv("OFF")) : 0.0;
static Contour poly(const std::vector<Vec2>& v) { Contour c; c.closed = true; for (size_t i = 0; i < v.size(); ++i) c.spans.push_back(geometry::LineSpan{v[i], v[(i + 1) % v.size()]}); return c; }
static Contour xf(Contour c, double s, Vec2 o) { for (auto& sp : c.spans) { if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start * s + o; l->end = l->end * s + o; } else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = a->centre * s + o; a->radius *= s; } else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = b->p0 * s + o; b->p1 = b->p1 * s + o; b->p2 = b->p2 * s + o; b->p3 = b->p3 * s + o; } } return c; }
static tooling::Tool vb() { tooling::Tool t; t.type = tooling::ToolType::VBit; t.diameter = 20; t.includedAngle = 3.14159265358979323846 / 2; return t; }
static tooling::Tool em(double d) { tooling::Tool t; t.type = tooling::ToolType::EndMill; t.diameter = d; return t; }
static ProfileParams pp(int dir, bool sharp) { ProfileParams p; p.side = ProfileSide::Outside; p.startAt = StartAt::VectorStart; p.passes.cutDepth = 3; p.passes.passDepth = 3; p.safeZ = 5;
  p.direction = (CutDirection)dir; p.corners.sharpExternal = sharp; p.corners.mitreLimit = 50; return p; }
static Vec2 startNear(const std::vector<Contour>& cs, const tooling::Tool& t, const ProfileParams& p, Vec2 T) {
  auto r = profileToolpath(cs, t, p); double bd = 1e300; Vec2 b{};
  for (auto& m : r.toolpath.moves) if (m.kind == MoveKind::Rapid && geometry::distance(m.to.xy(), T) < bd) { bd = geometry::distance(m.to.xy(), T); b = m.to.xy(); }
  return b; }
struct X { double s; Vec2 o; };
static std::vector<X> xforms() { std::vector<X> v; double u = 1, d = 1; for (int k = 0; k < 4; ++k) { u = std::nextafter(u, 2.0); d = std::nextafter(d, 0.0); v.push_back({u, {0, 0}}); v.push_back({d, {0, 0}}); }
  for (double r : {1e-12, 1e-10, 1e-9}) { v.push_back({1 + r, {0, 0}}); v.push_back({1 - r, {0, 0}}); }
  for (Vec2 o : {Vec2{3, -7}, Vec2{0.1, 0.3}, Vec2{1000.25, -999.75}}) v.push_back({1, o});
  return v; }
static double maxMove(const std::vector<Contour>& cs, const tooling::Tool& t, const ProfileParams& p, Vec2 T, Vec2 s0, std::string* worst = nullptr) { double mx = 0;
  for (auto x : xforms()) { std::vector<Contour> c2; for (auto& c : cs) c2.push_back(xf(c, x.s, x.o)); Vec2 s = startNear(c2, t, p, T * x.s + x.o);
    double m = geometry::distance((s - x.o) * (1.0 / x.s), s0); if (m > mx) { mx = m; if (worst) { char b[96]; std::snprintf(b, sizeof b, "s-1=%.3g o=%g", x.s - 1, x.o.x); *worst = b; } } } return mx; }
static Vec2 rightOf(Vec2 u) { return Vec2{u.y, -u.x}; }
static Vec2 mitre(Vec2 T, Vec2 uIn, Vec2 uOut, double d) { Vec2 p0 = T + rightOf(uIn) * d, q0 = T + rightOf(uOut) * d; double t = (q0 - p0).cross(uOut) / uIn.cross(uOut); return p0 + uIn * t; }
static Vec2 U(Vec2 a, Vec2 b) { Vec2 w = b - a; return w * (1.0 / w.length()); }
// A at index 0; in-edge direction (1,0); out-edge at tA; neighbour B at L before A (order 0) or after (order 1, mirrored) turning tB
static std::vector<Vec2> shape(double tA, double tB, double L, int order, Vec2 A0 = {11.25, 3.5}) {
  Vec2 A = A0 + Vec2{OFFX, 0.7 * OFFX}; Vec2 ein{1, 0}; Vec2 eout = Vec2::fromAngle(tA, 1.0); Vec2 eP = Vec2::fromAngle(-tB, 1.0);
  Vec2 B = A - ein * L, P = B - eP * 30.0, Q = A + eout * 30.0, F = (P + Q) * 0.5 + Vec2{0, 25};
  if (order == 0) return {A, Q, F, P, B};
  auto m = [&](Vec2 q) { return Vec2{2 * A.x - q.x, q.y}; }; return {A, m(B), m(P), m(F), m(Q)}; }
int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "split"; const double d = 3.0; const bool useEm = getenv("TOOL") && std::string(getenv("TOOL")) == "em";
  const bool nopert = getenv("NOPERT") != nullptr;
  auto envList = [](const char* k, std::vector<double> def) { if (!getenv(k)) return def; std::vector<double> v; std::string s = getenv(k); size_t p = 0;
    while (p < s.size()) { size_t q = s.find(',', p); if (q == std::string::npos) q = s.size(); v.push_back(atof(s.substr(p, q - p).c_str())); p = q + 1; } return v; };
  if (mode == "split" || mode == "short") {
    std::vector<double> tAs = envList("TA", {1e-3, 0.05, 0.3, 1.2, 2.5});
    std::vector<double> Ls = envList("L", mode == "split" ? std::vector<double>{1e-7, 3e-7, 1e-6, 3e-6, 1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 3e-3} : std::vector<double>{1e-3, 1.5e-3, 5e-3, 0.05, 0.5, 2});
    std::vector<double> tBs = envList("TB", mode == "split" ? std::vector<double>{0.4} : std::vector<double>{-2, -1, -0.3, -0.05, 0.05, 0.3, 1, 2});
    for (double tA : tAs) for (double L : Ls) for (double tB : tBs) for (int order = 0; order < 2; ++order) for (int dir = 0; dir < 2; ++dir) {
      std::vector<Vec2> v;
      if (mode == "split") { v = shape(tA, tB, 5.0, order); Vec2 A = v[0];
        if (order == 0) v.push_back(A - Vec2{1, 0} * L); else v.insert(v.begin() + 1, A + U(A, v[1]) * L); }
      else v = shape(tA, tB, L, order);
      // A = v[0]; its own edges: in from v.back(), out to v[1] (both lines along the in/out directions of A by construction)
      Vec2 A = v[0]; size_t n = v.size(); Vec2 uIn = U(v[n - 1], A), uOut = U(A, v[1]);
      ProfileParams p = pp(dir, !useEm); tooling::Tool t = useEm ? em(2 * d) : vb();
      Vec2 s = startNear({poly(v)}, t, p, A);
      Vec2 M = mitre(A, uIn, uOut, d), Fin = A + rightOf(uIn) * d, Fout = A + rightOf(uOut) * d;
      std::string w; double mm = nopert ? -1 : maxMove({poly(v)}, t, p, A, s, &w);
      if (const char* dp = getenv("DUMP")) { char nm[256]; std::snprintf(nm, sizeof nm, "%s_%s_tA%g_L%g_tB%g_o%d_d%d_off%g.scn", dp, mode.c_str(), tA, L, tB, order, dir, OFFX); writeScene(nm, {poly(v)}, p, useEm ? 2 * d : 20.0, useEm ? 0 : 90); }
      std::printf("%s %s tA %.3g L %.3g tB %.3g order %d dir %d off %g | S-mitre %.4g S-Fin %.4g S-Fout %.4g |S-A|-d %.3g maxMove %.3g %s\n", mode.c_str(), useEm ? "em" : "vb", tA, L, tB, order, dir, OFFX,
                  geometry::distance(s, M), geometry::distance(s, Fin), geometry::distance(s, Fout), geometry::distance(s, A) - d, mm, w.c_str());
      std::fflush(stdout);
    }
  }

  if (mode == "tiny") {
    // tiny edge (length e, direction phi relative to the in-edge) at the start vertex A: order 0 = A -> A + e dir (out-edge tiny),
    // order 1 = A - e dir -> A (in-edge tiny). The real corner is between the in-edge (1,0) and the out-edge at tA.
    // LIM = mitre limit (default 50; 1 = squared off). Prints |S - mitre|, |S - chamfer corners| (chamfer at distance d).
    std::vector<double> tAs = envList("TA", {0.3, 1.2, 2.0});
    std::vector<double> es = envList("E", {0.0, 1e-12, 1e-9, 1e-7, 1e-5, 5e-4, 9.99e-4});
    std::vector<double> phis = envList("PHI", {0.0, 0.7, 1.6, 3.0, -1.6});
    const double lim = getenv("LIM") ? atof(getenv("LIM")) : 50.0;
    for (double tA : tAs) for (double e : es) for (double phi : phis) for (int order = 0; order < 2; ++order) for (int dir = 0; dir < 2; ++dir) {
      std::vector<Vec2> v = shape(tA, 0.4, 5.0, 0); Vec2 A = v[0]; Vec2 uIn = U(v.back(), A), uOut = U(A, v[1]);
      Vec2 w = Vec2::fromAngle(phi, 1.0);
      if (order == 0) v.insert(v.begin() + 1, A + w * e); else v.push_back(A - w * e);
      ProfileParams p = pp(dir, !useEm); p.corners.mitreLimit = lim; tooling::Tool t = useEm ? em(2 * d) : vb();
      Vec2 s = startNear({poly(v)}, t, p, A);
      Vec2 M = mitre(A, uIn, uOut, d), Fin = A + rightOf(uIn) * d, Fout = A + rightOf(uOut) * d;
      const double th = std::fabs(std::atan2(uIn.cross(uOut), uIn.dot(uOut))); const double tt = d * std::tan(th / 4);
      Vec2 Ci = Fin + uIn * tt, Co = Fout - uOut * tt;
      std::string wst; double mm = nopert ? -1 : maxMove({poly(v)}, t, p, A, s, &wst);
      if (const char* dp = getenv("DUMP")) { char nm[256]; std::snprintf(nm, sizeof nm, "%s_tiny_tA%g_e%g_phi%g_o%d_d%d_lim%g.scn", dp, tA, e, phi, order, dir, lim); writeScene(nm, {poly(v)}, p, useEm ? 2 * d : 20.0, useEm ? 0 : 90); }
      std::printf("tiny %s lim %g tA %.3g e %.3g phi %.3g order %d dir %d | S-mitre %.4g S-chIn %.4g S-chOut %.4g S-Fin %.4g S-Fout %.4g |S-A|-d %.3g maxMove %.3g %s\n", useEm ? "em" : "vb", lim, tA, e, phi, order, dir,
                  geometry::distance(s, M), geometry::distance(s, Ci), geometry::distance(s, Co), geometry::distance(s, Fin), geometry::distance(s, Fout), geometry::distance(s, A) - d, mm, wst.c_str());
      std::fflush(stdout);
    }
  }

  if (mode == "bez") {
    // start vertex A between two cubic Bezier edges (curvature SAG relative, both edges bulging out), V-bit; LIM mitre limit.
    std::vector<double> tAs = envList("TA", {0.3, 1.2, 2.5});
    std::vector<double> sags = envList("SAG", {-0.1, -0.01, 0.01, 0.1});
    const double lim = getenv("LIM") ? atof(getenv("LIM")) : 50.0;
    for (double tA : tAs) for (double sag : sags) for (int dir = 0; dir < 2; ++dir) {
      std::vector<Vec2> v = shape(tA, 0.4, 12.0, 0); Vec2 A = v[0];
      Contour c; c.closed = true; size_t n = v.size();
      for (size_t i = 0; i < n; ++i) { Vec2 a = v[i], b = v[(i + 1) % n];
        if (i == 0 || i == n - 1) { Vec2 dd = b - a; Vec2 nn = Vec2{dd.y, -dd.x} * sag; c.spans.push_back(geometry::BezierSpan{a, a + dd * (1.0 / 3) + nn, a + dd * (2.0 / 3) + nn, b}); }
        else c.spans.push_back(geometry::LineSpan{a, b}); }
      ProfileParams p = pp(dir, true); p.corners.mitreLimit = lim; tooling::Tool t = vb();
      Vec2 s = startNear({c}, t, p, A);
      double mx = 0; std::string wst;
      if (!nopert) for (auto x : xforms()) { Contour c2 = c; for (auto& sp : c2.spans) { if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start * x.s + x.o; l->end = l->end * x.s + x.o; } else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = b->p0 * x.s + x.o; b->p1 = b->p1 * x.s + x.o; b->p2 = b->p2 * x.s + x.o; b->p3 = b->p3 * x.s + x.o; } }
        Vec2 s2 = startNear({c2}, t, p, A * x.s + x.o); double m = geometry::distance((s2 - x.o) * (1.0 / x.s), s); if (m > mx) { mx = m; char b[64]; std::snprintf(b, sizeof b, "s-1=%.3g o=%g", x.s - 1, x.o.x); wst = b; } }
      if (const char* dp = getenv("DUMP")) { char nm[256]; std::snprintf(nm, sizeof nm, "%s_bez_tA%g_sag%g_d%d_lim%g.scn", dp, tA, sag, dir, lim); writeScene(nm, {c}, p, 20.0, 90); }
      std::printf("bez lim %g tA %.3g sag %.3g dir %d | |S-A|-d %.4g maxMove %.3g %s\n", lim, tA, sag, dir, geometry::distance(s, A) - d, mx, wst.c_str()); std::fflush(stdout);
    }
  }

  if (mode == "scan") {
    // jump finder: start vertex A between an arc in-edge (radius R, from P, bulging out) and a line out-edge at tA. Scans R
    // over [R0, R1] in NS steps (TOOL=em round joins / vb sharp with LIM; MRG=1 adds an overlapping second polygon: merged
    // region). Prints consecutive starts that jump by more than JUMP (default 0.02 mm), then bisects each jump to 1e-15 relative
    // and prints the start move under the xforms at the jump.
    const double tA = getenv("TA") ? atof(getenv("TA")) : 1.0, R0 = getenv("R0") ? atof(getenv("R0")) : 8.0, R1 = getenv("R1") ? atof(getenv("R1")) : 200.0;
    const int NS = getenv("NS") ? atoi(getenv("NS")) : 400; const double jump = getenv("JUMP") ? atof(getenv("JUMP")) : 0.02;
    const double lim = getenv("LIM") ? atof(getenv("LIM")) : 50.0; const bool mrg = getenv("MRG") != nullptr;
    for (int dir = 0; dir < 2; ++dir) {
      auto build = [&](double R) { Vec2 A = Vec2{11.25, 3.5} + Vec2{OFFX, 0.7 * OFFX}; Vec2 Q = A + Vec2::fromAngle(tA, 1.0) * 30.0; Vec2 P = A - Vec2{1, 0} * 30.0;
        Vec2 F = (P + Q) * 0.5 + Vec2{0, 25};
        // arc P -> A, CCW polygon interior above: bulge downward (outward): centre above the chord midpoint
        Vec2 mid = (P + A) * 0.5; double h = std::sqrt(std::max(0.0, R * R - 225.0)); Vec2 cen = mid + Vec2{0, h};
        double a0 = std::atan2(P.y - cen.y, P.x - cen.x), a1 = std::atan2(A.y - cen.y, A.x - cen.x); if (a1 < a0) a1 += 2 * 3.14159265358979323846;
        Contour c; c.closed = true; c.spans.push_back(geometry::LineSpan{A, Q}); c.spans.push_back(geometry::LineSpan{Q, F}); c.spans.push_back(geometry::LineSpan{F, P});
        c.spans.push_back(geometry::ArcSpan{cen, R, a0, a1, geometry::ArcDirection::CCW});
        std::vector<Contour> cs{c};
        if (mrg) { Vec2 g = (A + Q + F + P) * 0.25; std::vector<Vec2> w{A + (g - A) * 0.5, Q + (g - A) * 0.5, F + (g - A) * 0.5, P + (g - A) * 0.5}; cs.push_back(poly(w)); }
        return std::make_pair(cs, A); };
      ProfileParams p = pp(dir, !useEm); p.corners.mitreLimit = lim; tooling::Tool t = useEm ? em(2 * d) : vb();
      auto startAtR = [&](double R) { auto [cs, A] = build(R); return startNear(cs, t, p, A); };
      Vec2 prev = startAtR(R0); double pr = R0;
      for (int k = 1; k <= NS; ++k) { double R = R0 + (R1 - R0) * k / NS; Vec2 s = startAtR(R);
        if (geometry::distance(s, prev) > jump) {
          double lo = pr, hi = R; Vec2 slo = prev, shi = s;
          for (int it = 0; it < 200; ++it) { double mid = 0.5 * (lo + hi); if (mid <= lo || mid >= hi) break; Vec2 sm = startAtR(mid);
            if (geometry::distance(sm, slo) < geometry::distance(sm, shi)) { lo = mid; slo = sm; } else { hi = mid; shi = sm; } }
          auto [cs, A] = build(lo); std::string w; double mm = maxMove(cs, t, p, A, slo, &w);
          if (const char* dp = getenv("DUMP")) { char nm[256]; std::snprintf(nm, sizeof nm, "%s_scan_%s%s_tA%g_R%.17g_d%d_lim%g.scn", dp, useEm ? "em" : "vb", mrg ? "_mrg" : "", tA, lo, dir, lim); writeScene(nm, cs, p, useEm ? 2 * d : 20.0, useEm ? 0 : 90); }
          std::printf("scan %s%s tA %g lim %g dir %d | jump %.4g between R %.17g and %.17g (gap %.3g) | at lo |S-A|-d %.3g maxMove %.4g %s\n", useEm ? "em" : "vb", mrg ? " mrg" : "", tA, lim, dir,
                      geometry::distance(slo, shi), lo, hi, hi - lo, geometry::distance(slo, A) - d, mm, w.c_str());
          std::fflush(stdout);
        }
        prev = s; pr = R; }
      std::printf("scan done dir %d\n", dir); std::fflush(stdout);
    }
  }
}
