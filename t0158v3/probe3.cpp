// T-0158 round-3 verifier probes (V-bit sharp unless noted, Outside, VectorStart).
//  ngon  : regular N-gon (fine polyline approximating a circle; every vertex turns 2pi/N): does the cut start at the mitre
//          corner of the start vertex (owner 2026-11-05)? prints |start - M0|, |start - nearest foot|, and the max move under
//          ulp/1e-9 scaling. Climb and Conventional.
//  tie   : start vertex A turning tA preceded (in contour order) by B turning tB = tA - 1e-3 + delta at distance L
//          (L sin tB < tol, so B's edges are near): which mitre takes the start as delta crosses 0 (kTipTurnTie knife-edge)?
//          env TB=<tB> overrides tB (delta added), LFAC=f sets L = f * tol / sin(tB) (default 0.5): L sin(tB) = tol knife-edge.
//  join  : merged region (two overlapping stars), end mill: where on the region's polygonal round join does the cut start?
//          prints the angle of start-tip from the travel-first normal, in units of the join's chord step.
//  DUMP=path writes the scene of the last run (tie/ngon: env SEL to pick one).
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
static tooling::Tool em(double d) { tooling::Tool t; t.type = tooling::ToolType::EndMill; t.diameter = d; return t; }
static ProfileParams pp(int dir, double limit, bool sharp = true) { ProfileParams p; p.side = ProfileSide::Outside; p.startAt = StartAt::VectorStart; p.passes.cutDepth = 3; p.passes.passDepth = 3; p.safeZ = 5;
  p.direction = (CutDirection)dir; p.corners.sharpExternal = sharp; p.corners.mitreLimit = limit; return p; }
struct S { Vec2 at, next; bool ok = false; };
static S run(const std::vector<Contour>& cs, const tooling::Tool& t, const ProfileParams& p) { auto r = profileToolpath(cs, t, p); S s;
  if (r.toolpath.moves.empty()) return s; s.at = r.toolpath.moves.front().to.xy();
  for (auto& m : r.toolpath.moves) if ((m.kind == MoveKind::Feed || isArc(m.kind)) && geometry::distance(m.to.xy(), s.at) > 1e-9) { s.next = m.to.xy(); s.ok = true; break; }
  return s; }
static std::vector<double> scales() { std::vector<double> o; double u = 1, d = 1; for (int k = 0; k < 4; ++k) { u = std::nextafter(u, 2.0); d = std::nextafter(d, 0.0); o.push_back(u); o.push_back(d); }
  for (double r : {1e-12, 1e-10, 1e-9}) { o.push_back(1 + r); o.push_back(1 - r); } return o; }
static double maxMove(const std::vector<Contour>& cs, const tooling::Tool& t, const ProfileParams& p, Vec2 at0) { double mx = 0;
  for (double k : scales()) { std::vector<Contour> c2; for (auto& c : cs) c2.push_back(scale(c, k)); S s = run(c2, t, p); mx = std::max(mx, geometry::distance(s.at * (1.0 / k), at0)); } return mx; }
int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "ngon";
  const double d = 3.0;
  if (mode == "ngon") {
    std::vector<int> Ns{100, 300, 1000, 2000, 5000}; std::vector<double> Rs{20.0, 60.0}; if (getenv("NGON_N")) Ns = {atoi(getenv("NGON_N"))}; if (getenv("NGON_R")) Rs = {atof(getenv("NGON_R"))};
    for (int N : Ns) for (double R : Rs) for (int dir = 0; dir < 2; ++dir) {
      Vec2 c{7.3, -4.1}; double rot = 0.3; std::vector<Vec2> v; for (int j = 0; j < N; ++j) v.push_back(c + Vec2::fromAngle(rot + 2 * PI * j / N, R));
      ProfileParams p = pp(dir, 50); S s = run({poly(v)}, vb(), p);
      double turn = 2 * PI / N; Vec2 radial = Vec2::fromAngle(rot, 1.0); Vec2 M0 = v[0] + radial * (d / std::cos(turn / 2));
      double foot = d * std::tan(turn / 2);  // distance mitre - foot along the edge
      if (const char* dp = getenv("DUMP")) if (getenv("SEL") && atoi(getenv("SEL")) == N * 10 + dir + (R > 30 ? 100000 : 0)) writeScene(dp, {poly(v)}, p, 20.0, 90);
      std::printf("ngon N %d R %.0f dir %d turn %.3g edge %.4g start-M0 %.4g (mitre-foot %.3g) maxMovePert %.3g\n", N, R, dir, turn, 2 * R * std::sin(turn / 2),
                  geometry::distance(s.at, M0), foot, maxMove({poly(v)}, vb(), p, s.at));
    }
  }
  if (mode == "tie") {
    std::vector<double> tAs{2e-3, 5e-3, 0.02, 0.3, 1.0};
    std::vector<double> deltas{-1e-4, -1e-7, -1e-10, -1e-13, 0.0, 1e-13, 1e-10, 1e-7, 1e-4};
    if (getenv("TA")) tAs = {atof(getenv("TA"))};
    if (getenv("DELTA")) deltas = {atof(getenv("DELTA"))};
    for (double tA : tAs) for (int order = 0; order < 2; ++order) for (double delta : deltas) for (int dir = 0; dir < 2; ++dir) {
      if ((getenv("ORDER") && atoi(getenv("ORDER")) != order) || (getenv("DIR") && atoi(getenv("DIR")) != dir)) continue;
      double tB = getenv("TB") ? atof(getenv("TB")) + delta : tA - 1e-3 + delta; double L = std::min(5.0, (getenv("LFAC") ? atof(getenv("LFAC")) : 0.5) * 1e-3 / std::sin(tB));
      // contour order ... P -> B -> A -> Q -> F (CCW, left turns); order 1: mirror so B follows A in contour order
      Vec2 A{11.25, 3.5}; Vec2 ein{1, 0};
      Vec2 eout = Vec2::fromAngle(tA, 1.0); Vec2 eP = Vec2::fromAngle(-tB, 1.0);
      Vec2 B = A - ein * L; Vec2 P = B - eP * 30.0; Vec2 Q = A + eout * 30.0; Vec2 F = (P + Q) * 0.5 + Vec2{0, 25};
      std::vector<Vec2> v;
      if (order == 0) v = {A, Q, F, P, B};
      else { auto m = [&](Vec2 q) { return Vec2{2 * A.x - q.x, q.y}; }; v = {A, m(B), m(P), m(F), m(Q)}; }  // mirrored about x = A.x, reversed order keeps CCW
      ProfileParams p = pp(dir, 50); S s = run({poly(v)}, vb(), p);
      // mitres: at A between edge B->A (dir ein) and A->Q (eout); at B between P->B and B->A
      auto mit = [&](Vec2 T, Vec2 uIn, Vec2 uOut) { Vec2 nIn{uIn.y, -uIn.x}, nOut{uOut.y, -uOut.x}; // outward = right of travel for CCW
        nIn = nIn * -1.0; nOut = nOut * -1.0; // left turns -> polygon interior on the left, outward on the right: (uy,-ux) is right; keep right
        nIn = nIn * -1.0; nOut = nOut * -1.0; Vec2 p0 = T + nIn * d, q0 = T + nOut * d; double t = (q0 - p0).cross(uOut) / uIn.cross(uOut); return p0 + uIn * t; };
      Vec2 MA, MB;
      if (order == 0) { MA = mit(A, ein, eout); MB = mit(B, eP, ein); }
      else { auto m = [&](Vec2 q) { return Vec2{2 * A.x - q.x, q.y}; }; Vec2 Bm = m(B);
        // mirrored polygon: A -> Bm (dir -ein mirrored = ein? ) compute from vertices directly
        auto u = [](Vec2 a, Vec2 b) { Vec2 w = b - a; return w * (1.0 / w.length()); };
        MA = mit(A, u(v[4], A), u(A, Bm)); MB = mit(Bm, u(A, Bm), u(Bm, v[2])); }
      if (const char* dp = getenv("DUMP")) writeScene(dp, {poly(v)}, p, 20.0, 90);
      double mm = getenv("NOPERT") ? -1 : maxMove({poly(v)}, vb(), p, s.at);
      std::printf("tie tA %.3g order %d delta %.3g dir %d L %.4g | start-MA %.4g start-MB %.4g |MA-MB| %.4g maxMovePert %.3g\n", tA, order, delta, dir, L,
                  geometry::distance(s.at, MA), geometry::distance(s.at, MB), geometry::distance(MA, MB), mm);
    }
  }
  if (mode == "join") {
    for (int k = 0; k < 12; ++k) for (int dir = 0; dir < 2; ++dir) {
      int n = 4 + k % 4; double rot = 0.37 * k; Vec2 c{3.0 * k, -2.0 * k}; double ro = 20, ri = 9;
      std::vector<Vec2> v; for (int j = 0; j < 2 * n; ++j) v.push_back(c + Vec2::fromAngle(rot + j * PI / n, j % 2 ? ri : ro));
      Vec2 tip = v[0]; Vec2 sh = (c - tip) * 0.7; std::vector<Vec2> w; for (auto q : v) w.push_back(q + sh);
      double D = 2.0 + k % 5; ProfileParams p = pp(dir, 50, false);
      S s; { auto r = profileToolpath({poly(v), poly(w)}, em(D), p); double bd = 1e300; size_t bi = 0;
        for (size_t i = 0; i < r.toolpath.moves.size(); ++i) { auto& m = r.toolpath.moves[i]; if (m.kind == MoveKind::Rapid && geometry::distance(m.to.xy(), tip) < bd) { bd = geometry::distance(m.to.xy(), tip); bi = i; } }
        s.at = r.toolpath.moves[bi].to.xy(); for (size_t i = bi; i < r.toolpath.moves.size(); ++i) { auto& m = r.toolpath.moves[i]; if ((m.kind == MoveKind::Feed || isArc(m.kind)) && geometry::distance(m.to.xy(), s.at) > 1e-9) { s.next = m.to.xy(); break; } } }
      double R = D / 2 + 0.01;
      Vec2 uIn = (tip - v.back()) * (1.0 / (tip - v.back()).length()), uOut = (v[1] - tip) * (1.0 / (v[1] - tip).length());
      Vec2 nIn{uIn.y, -uIn.x}, nOut{uOut.y, -uOut.x};  // right of travel = outward (CCW)
      auto ang = [&](Vec2 a, Vec2 b) { return std::fabs(std::atan2(a.cross(b), a.dot(b))); };
      Vec2 r0 = (s.at - tip) * (1.0 / (s.at - tip).length()); Vec2 r1 = (s.next - tip) * (1.0 / std::max(1e-12, (s.next - tip).length()));
      double total = ang(nIn, nOut);
      // travel along the join: if next is closer (in angle) to nOut than start is, travel nIn -> nOut
      bool fwd = ang(r1, nOut) < ang(r0, nOut);
      double fromFirst = fwd ? ang(nIn, r0) : ang(nOut, r0);
      double chord = geometry::distance(s.at, s.next); double step = 2 * std::asin(std::min(1.0, chord / (2 * R)));
      if (const char* dp = getenv("DUMP")) if (getenv("SEL") && atoi(getenv("SEL")) == k * 2 + dir) writeScene(dp, {poly(v), poly(w)}, p, D, 0);
      std::printf("join k %d dir %d D %.0f |start-tip|-R %.3g joinAngle %.4f fromTravelFirstNormal %.5f firstMoveStep %.5f (=%.2f steps) maxMovePert %.3g\n", k, dir, D,
                  geometry::distance(s.at, tip) - R, total, fromFirst, step, fromFirst / step, 0.0);
    }
  }

  if (mode == "dch") {
    // D-shape (arc + closing line), V-bit 90 deg at depth 3 (d = 3), squared off (limit 1.0 / 1.1): start vertex = the arc's
    // start (also the line's end). Chamfer = line perpendicular to the tip's tangent bisector at distance d. Its corners:
    // cl on the line's offset, ca on the arc's offset circle. Owner 2026-11-05: start at the chamfer's first corner in travel.
    for (double sweepDeg : {120.0, 180.0, 240.0}) for (double lim : {1.0, 1.1}) for (int dir = 0; dir < 2; ++dir) for (int order = 0; order < 2; ++order) {
      Vec2 c{4.25, -3.5}; double R = 20, a0 = 0.4, sw = sweepDeg * PI / 180;
      geometry::ArcSpan arc{c, R, a0, a0 + sw, geometry::ArcDirection::CCW};
      Contour dsh; dsh.closed = true; Vec2 tip;
      if (order == 0) { dsh.spans.push_back(arc); dsh.spans.push_back(geometry::LineSpan{arc.endPoint(), arc.startPoint()}); tip = arc.startPoint(); }
      else { dsh.spans.push_back(geometry::LineSpan{arc.endPoint(), arc.startPoint()}); dsh.spans.push_back(arc); tip = arc.endPoint(); }
      ProfileParams p = pp(dir, lim); auto r = profileToolpath({dsh}, vb(), p);
      Vec2 at = r.toolpath.moves.front().to.xy(), next{}; for (auto& m : r.toolpath.moves) if ((m.kind == MoveKind::Feed || isArc(m.kind)) && geometry::distance(m.to.xy(), at) > 1e-9) { next = m.to.xy(); break; }
      // tangents at the tip, outward directions: line direction into/out of tip, arc tangent at tip
      Vec2 radial = (tip - c) * (1.0 / (tip - c).length());
      Vec2 tArc = Vec2{-radial.y, radial.x};  // CCW tangent at tip
      Vec2 other = order == 0 ? arc.endPoint() : arc.startPoint(); Vec2 uLine = (tip - other) * (1.0 / (tip - other).length());
      // outward bisector of the corner: away from the interior, i.e. - (normalised sum of the edge directions leaving the tip)
      Vec2 l1 = order == 0 ? tArc : tArc * -1.0;  // direction leaving the tip along the arc
      Vec2 l2 = uLine * -1.0;                    // direction leaving the tip along the line
      Vec2 bis = (l1 + l2) * (-1.0 / (l1 + l2).length());
      // chamfer corners: points q with (q - tip).bis = d on the line offset (distance d from the line, outside) and on circle R + d
      Vec2 nL{-uLine.y, uLine.x}; if (nL.dot(bis) < 0) nL = nL * -1.0;  // line offset normal (outside)
      Vec2 bperp{-bis.y, bis.x};
      // line offset: tip + nL*d + s*uLine ; solve (nL*d + s*uLine).bis = d
      double s = (d - d * nL.dot(bis)) / uLine.dot(bis); Vec2 cl = tip + nL * d + uLine * s;
      // circle |q - c| = R + d with q = tip + bis*d + t*bperp
      Vec2 q0 = tip + bis * d - c; double B = q0.dot(bperp), C = q0.dot(q0) - (R + d) * (R + d); double disc = std::sqrt(std::max(0.0, B * B - C));
      Vec2 ca1 = tip + bis * d + bperp * (-B + disc), ca2 = tip + bis * d + bperp * (-B - disc);
      Vec2 ca = geometry::distance(ca1, cl) < geometry::distance(ca2, cl) ? ca1 : ca2;
      bool atL = geometry::distance(at, cl) < 1e-6, atA = geometry::distance(at, ca) < 1e-6;
      bool nextOther = (atL && geometry::distance(next, ca) < 1e-6) || (atA && geometry::distance(next, cl) < 1e-6);
      std::printf("dch sweep %.0f limit %.1f dir %d order %d | start %s | first move runs along the chamfer (start = first corner in travel) %d | |cl-ca| %.4g\n",
                  sweepDeg, lim, dir, order, atL ? "lineCorner" : atA ? "arcCorner" : "OTHER", (int)nextOther, geometry::distance(cl, ca));
      if (const char* dp = getenv("DUMP")) if (getenv("SEL") && atoi(getenv("SEL")) == (int)sweepDeg * 100 + (lim > 1.05) * 10 + dir * 2 + order) writeScene(dp, {dsh}, p, 20.0, 90);
    }
  }
}
