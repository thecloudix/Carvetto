// Verifier T-0143: independent winding oracles.
//  Part A: integer polygons (lines only, incl. retraces, horizontals, gaps, open) - exact
//          Sunday winding (__int128) over the implicit chain of all span endpoints.
//  Part B: rings/arcs/Beziers vs long-double angle summation AND rotated-ray crossing on dense samples.
//  Part C: boundary points: locatePoint printed as hash for new/old comparison.
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
#include <string>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
using namespace carvetto; using namespace carvetto::geometry;
static Tolerance tol = Tolerance::defaults();
typedef __int128 i128;
struct IP { long long x, y; };
// exact winding of polygon (closed chain) about p, all coords scaled integers; returns INT_MIN if on boundary
static int sunday(const std::vector<IP>& v, IP p, bool& onB) {
  int wn = 0; onB = false; size_t n = v.size();
  for (size_t i = 0; i < n; ++i) {
    IP a = v[i], b = v[(i + 1) % n];
    i128 cr = (i128)(b.x - a.x) * (p.y - a.y) - (i128)(p.x - a.x) * (b.y - a.y);
    if (cr == 0 && std::min(a.x,b.x) <= p.x && p.x <= std::max(a.x,b.x) && std::min(a.y,b.y) <= p.y && p.y <= std::max(a.y,b.y)) onB = true;
    if (a.y <= p.y) { if (b.y > p.y && cr > 0) ++wn; }
    else { if (b.y <= p.y && cr < 0) --wn; }
  }
  return wn;
}
static long double angleSum(const std::vector<Vec2>& pts, Vec2 p) {
  long double s = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    const Vec2 a = pts[i], b = pts[(i + 1) % pts.size()];
    long double ax = (long double)a.x - p.x, ay = (long double)a.y - p.y, bx = (long double)b.x - p.x, by = (long double)b.y - p.y;
    s += atan2l(ax * by - ay * bx, ax * bx + ay * by);
  }
  return s / (2 * 3.14159265358979323846264338327950288L);
}
static int rotRay(const std::vector<Vec2>& pts, Vec2 p, long double th) {
  long double c = cosl(th), s = sinl(th); int w = 0;
  auto rot = [&](Vec2 q) { long double x = (long double)q.x - p.x, y = (long double)q.y - p.y; return std::pair<long double,long double>{c*x + s*y, -s*x + c*y}; };
  for (size_t i = 0; i < pts.size(); ++i) {
    auto [ax, ay] = rot(pts[i]); auto [bx, by] = rot(pts[(i + 1) % pts.size()]);
    if ((ay >= 0) == (by >= 0)) continue;
    long double x = ax + (bx - ax) * (0 - ay) / (by - ay);
    if (x > 0) w += by >= 0 ? 1 : -1;
  }
  return w;
}
static void sample(const Contour& c, std::vector<Vec2>& pts) {
  for (const Span& s : c.spans) {
    size_t n = 1;
    if (auto* a = std::get_if<ArcSpan>(&s)) n = (size_t)std::ceil(std::fabs(a->sweepAngle()) / (2*kPi) * 3000) + 4;
    else if (std::holds_alternative<BezierSpan>(s)) n = 600;
    for (size_t i = 0; i < n; ++i) pts.push_back(pointAt(s, double(i) / double(n)));
    pts.push_back(endPoint(s));
  }
}
static unsigned long long H = 1469598103934665603ULL;
static void mix(long long v) { H ^= (unsigned long long)v; H *= 1099511628211ULL; }
int main(int argc, char** argv) {
  unsigned seed = argc > 1 ? (unsigned)atoi(argv[1]) : 1;
  std::mt19937_64 rng(seed);
  auto ri = [&](int lo, int hi) { return (long long)std::uniform_int_distribution<int>(lo, hi)(rng); };
  auto ru = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
  long long aChecked = 0, aFail = 0, aGapped = 0, aOnRay = 0;
  // ---------------- Part A
  for (int it = 0; it < 4000; ++it) {
    const int mode = it % 4;  // 0 closed continuous, 1 open, 2 gapped closed, 3 huge offset continuous
    const long long off = mode == 3 ? (1LL << 40) : 0;  // scaled-by-2 coordinates stay exact
    int n = (int)ri(2, 12);
    std::vector<IP> V; for (int i = 0; i < n; ++i) V.push_back({ri(-6,6) * 2, ri(-6,6) * 2});
    // retraces and horizontals
    std::vector<IP> W;
    for (int i = 0; i < n; ++i) { W.push_back(V[i]); int k = (int)ri(0,9);
      if (k == 0 && i) { W.push_back(V[i-1]); W.push_back(V[i]); }
      if (k == 1 && i + 1 < n) W.push_back({V[i+1].x, V[i].y}); }
    Contour c; c.closed = mode != 1;
    std::vector<IP> chain;  // implicit polygon of all span endpoints in order
    for (size_t i = 0; i + 1 < W.size() + (c.closed ? 1 : 0); ++i) {
      IP a = W[i], b = W[(i + 1) % W.size()];
      if (mode == 2 && ri(0,3) == 0) { a.x += ri(-3,3) * 2; a.y += ri(-3,3) * 2; ++aGapped; }  // gap before this span
      c.append(LineSpan{{0.5 * double(a.x + off), 0.5 * double(a.y + off)}, {0.5 * double(b.x + off), 0.5 * double(b.y + off)}});
      chain.push_back(a); chain.push_back(b);
    }
    if (c.spans.empty()) continue;
    for (int q = 0; q < 60; ++q) {
      IP p;
      if (q % 2 == 0) { p.y = chain[(size_t)ri(0, (int)chain.size() - 1)].y; ++aOnRay; } else p.y = ri(-14, 14);
      p.x = ri(-14, 14);
      bool onB; int ex = sunday(chain, p, onB);
      if (onB) continue;
      Vec2 pp{0.5 * double(p.x + off), 0.5 * double(p.y + off)};
      int got = windingNumber(c, pp, tol);
      mix(got); ++aChecked;
      // also EvenOdd/NonZero classification when not within tol of boundary
      if (got != ex) { if (aFail < 8) std::printf("A FAIL mode %d it %d p(%g,%g) got %d exact %d\n", mode, it, pp.x, pp.y, got, ex); ++aFail; }
    }
  }
  std::printf("A: checked %lld (on vertex rays ~%lld) fail %lld gappedSpans %lld\n", aChecked, aOnRay, aFail, aGapped);
  // ---------------- Part B: curves
  long long bChecked = 0, bFail = 0, bOracleDisagree = 0, bOnRay = 0;
  for (int it = 0; it < 600; ++it) {
    const int kind = it % 6;
    Vec2 cen = (it % 5 == 0) ? Vec2{ru(-1e6, 1e6), ru(-1e6, 1e6)} : (it % 3 == 0 ? Vec2{0, 0} : Vec2{(double)ri(-10,10), (double)ri(-10,10)});
    double ro = (it % 4 == 0) ? (double)ri(2, 20) : ru(1, 30); double rin = ro * ru(0.2, 0.8);
    double th;
    int sel = (int)ri(0, 3);
    if (sel == 0) th = ri(0, 23) * 15.0 * kPi / 180.0; else if (sel == 1) th = ri(0, 4) * 0.5 * kPi; else if (sel == 2) th = ru(0, 2*kPi); else th = -kPi + ri(0,8) * kPi / 4;
    Contour c; c.closed = true;
    Vec2 po = cen + Vec2::fromAngle(th, ro), pi = cen + Vec2::fromAngle(th, rin);
    ArcSpan outer{cen, ro, th, th + 2*kPi, ArcDirection::CCW}, inner{cen, rin, th, th + 2*kPi, ArcDirection::CW};
    if (kind == 0) { c.append(outer); c.append(LineSpan{po, pi}); c.append(inner); c.append(LineSpan{pi, po}); }
    else if (kind == 1) { // outer split at pi relative, inner split, bezier versions
      c.append(ArcSpan{cen, ro, th, th + kPi, ArcDirection::CCW}); c.append(ArcSpan{cen, ro, th + kPi, th + 2*kPi, ArcDirection::CCW});
      c.append(LineSpan{po, pi}); c.append(ArcSpan{cen, rin, th, th - kPi, ArcDirection::CW}); c.append(ArcSpan{cen, rin, th - kPi, th - 2*kPi, ArcDirection::CW}); c.append(LineSpan{pi, po}); }
    else if (kind == 2) { for (auto& b : outer.toBeziers()) c.append(b); c.append(LineSpan{po, pi}); for (auto& b : inner.toBeziers()) c.append(b); c.append(LineSpan{pi, po}); }
    else if (kind == 3) { // half annulus with bridges at th and th+pi (opposite bridges)
      Vec2 qo = cen + Vec2::fromAngle(th + kPi, ro), qi = cen + Vec2::fromAngle(th + kPi, rin);
      c.append(ArcSpan{cen, ro, th, th + kPi, ArcDirection::CCW}); c.append(LineSpan{qo, qi}); c.append(ArcSpan{cen, rin, th + kPi, th, ArcDirection::CW}); c.append(LineSpan{pi, po}); }
    else if (kind == 4) { // double ring: two rings in one loop sharing the bridge (outer twice) -> winding 2 in band
      c.append(outer); c.append(outer); c.append(LineSpan{po, pi}); c.append(inner); c.append(LineSpan{pi, po}); }
    else { // figure-eight lines + arc mix, self-intersecting; plus arc ending at 0/pi
      c.append(ArcSpan{cen, ro, 0, kPi, ArcDirection::CCW}); c.append(LineSpan{cen + Vec2{-ro, 0}, cen + Vec2{ro, -ro}}); c.append(LineSpan{cen + Vec2{ro, -ro}, cen + Vec2{-ro, -ro}}); c.append(LineSpan{cen + Vec2{-ro, -ro}, cen + Vec2{ro, 0}}); }
    std::vector<Vec2> pts; sample(c, pts);
    // vertex ys
    std::vector<double> ys{cen.y}; for (auto& s : c.spans) { ys.push_back(startPoint(s).y); ys.push_back(endPoint(s).y); }
    double scale = ro;
    for (int q = 0; q < 40; ++q) {
      Vec2 p;
      int m = q % 4;
      double y = ys[(size_t)ri(0, (int)ys.size() - 1)];
      if (m == 0) { p = {cen.x + ru(-1.3, 1.3) * ro, y}; ++bOnRay; }
      else if (m == 1) { p = {cen.x + ru(-1.3, 1.3) * ro, std::nextafter(y, (q & 4) ? 1e300 : -1e300)}; ++bOnRay; }
      else if (m == 2 && q < 8) p = cen;
      else p = {cen.x + ru(-1.3, 1.3) * ro, cen.y + ru(-1.3, 1.3) * ro};
      double d = distanceTo(c, p);
      if (d < 2e-3 * scale + 0.05) continue;
      long double as = angleSum(pts, p); int o1 = (int)lroundl(as); int o2 = rotRay(pts, p, ru(0, 2*kPi));
      if (o1 != o2 || fabsl(as - o1) > 1e-6) { ++bOracleDisagree; continue; }
      int got = windingNumber(c, p, tol); mix(got); ++bChecked;
      PointLocation le = locatePoint(c, p, FillRule::EvenOdd, tol), ln = locatePoint(c, p, FillRule::NonZero, tol);
      bool ok = got == o1 && (le == ((o1 % 2) ? PointLocation::Inside : PointLocation::Outside)) && (ln == (o1 ? PointLocation::Inside : PointLocation::Outside));
      if (!ok) { if (bFail < 10) std::printf("B FAIL kind %d it %d th %.17g cen(%.17g,%.17g) ro %.17g p(%.17g,%.17g) got %d oracle %d le %d ln %d\n", kind, it, th, cen.x, cen.y, ro, p.x, p.y, got, o1, int(le), int(ln)); ++bFail; }
    }
  }
  std::printf("B: checked %lld (on/near rays ~%lld) fail %lld oracleDisagree(skipped) %lld\n", bChecked, bOnRay, bFail, bOracleDisagree);
  // ---------------- Part C: boundary points, hash of locatePoint
  unsigned long long HC = 1469598103934665603ULL; long long cN = 0, cNotOnB = 0;
  for (int it = 0; it < 300; ++it) {
    Vec2 cen{(double)ri(-10,10), (double)ri(-10,10)}; double ro = ru(2, 20), rin = ro * 0.5, th = ri(0, 23) * kPi / 12;
    Contour c; c.closed = true; Vec2 po = cen + Vec2::fromAngle(th, ro), pi = cen + Vec2::fromAngle(th, rin);
    c.append(ArcSpan{cen, ro, th, th + 2*kPi, ArcDirection::CCW}); c.append(LineSpan{po, pi}); c.append(ArcSpan{cen, rin, th, th + 2*kPi, ArcDirection::CW}); c.append(LineSpan{pi, po});
    for (int q = 0; q < 50; ++q) { Vec2 p = c.pointAt(ru(0, 1)); if (q % 5 == 0) p = startPoint(c.spans[(size_t)ri(0,3)]);
      for (auto r : {FillRule::EvenOdd, FillRule::NonZero}) { int l = int(locatePoint(c, p, r, tol)); HC ^= (unsigned)l; HC *= 1099511628211ULL; ++cN; if (l != int(PointLocation::OnBoundary)) ++cNotOnB; } }
  }
  std::printf("C: boundary queries %lld notOnBoundary %lld hash %016llx\n", cN, cNotOnB, HC);
  std::printf("winding hash %016llx\n", H);
}
