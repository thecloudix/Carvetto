// Verifier perturbation driver (throwaway). Compiled against a given core build.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
static const double PI = 3.14159265358979323846;
struct Rng { std::uint64_t s; explicit Rng(std::uint64_t x) : s(x * 0x9E3779B97F4A7C15ULL + 7) {}
  std::uint64_t next() { std::uint64_t z = (s += 0x9E3779B97F4A7C15ULL); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL; return z ^ (z >> 31); }
  double uni() { return (double)(next() >> 11) * 0x1.0p-53; }
  double range(double a, double b) { return a + (b - a) * uni(); } };
static double ulps(double v, int k) { while (k > 0) { v = std::nextafter(v, INFINITY); --k; } while (k < 0) { v = std::nextafter(v, -INFINITY); ++k; } return v; }
static ArcSpan arcThrough(Vec2 p0, Vec2 p1, double sweep) {
  Vec2 c = p1 - p0; double L = c.length(); double r = L / (2 * std::sin(std::fabs(sweep) / 2));
  Vec2 m = (p0 + p1) * 0.5; Vec2 n = Vec2{-c.y, c.x} / L; double h = std::sqrt(std::max(0.0, r * r - L * L / 4));
  // centre left of chord for CCW with |sweep|<pi
  double sgn = (sweep > 0) == (std::fabs(sweep) < PI) ? 1.0 : -1.0;
  Vec2 ctr = m + n * (sgn * h);
  double sa = std::atan2(p0.y - ctr.y, p0.x - ctr.x), ea = std::atan2(p1.y - ctr.y, p1.x - ctr.x);
  return ArcSpan{ctr, r, sa, ea, sweep > 0 ? ArcDirection::CCW : ArcDirection::CW};
}
// family 0: closed loop of arcs only; 1: closed line/arc mix; 2: open line/arc mix; 3: open, big-radius arcs
static Contour gen(Rng& g, int fam, Vec2 T) {
  Contour c; int n = 3 + (int)(g.uni() * 5); double R = std::exp(g.range(std::log(0.3), std::log(30.0)));
  std::vector<Vec2> pts; double a0 = g.range(0, 2 * PI);
  for (int i = 0; i < n; ++i) { double a = a0 + 2 * PI * i / n + g.range(-0.3, 0.3) * 2 * PI / n; double rr = R * g.range(0.6, 1.4); pts.push_back(T + Vec2{rr * std::cos(a), rr * std::sin(a)}); }
  bool closed = fam <= 1; int m = closed ? n : n - 1;
  for (int i = 0; i < m; ++i) {
    Vec2 p0 = pts[i], p1 = pts[(i + 1) % n];
    bool arc = fam == 0 || g.uni() < 0.6;
    if (arc) {
      double sw;
      double u = g.uni();
      if (fam == 3 || u < 0.25) sw = g.range(1e-4, 0.03); else sw = g.range(0.2, 2.6);
      if (g.uni() < 0.3) sw = -sw;
      c.spans.push_back(arcThrough(p0, p1, sw));
    } else c.spans.push_back(LineSpan{p0, p1});
  }
  // stitch line ends to arc points
  for (std::size_t i = 0; i < c.spans.size(); ++i) {
    std::size_t j = (i + 1) % c.spans.size(); if (!closed && j == 0) break;
    if (auto* l = std::get_if<LineSpan>(&c.spans[j])) l->start = endPoint(c.spans[i]);
    else if (auto* l2 = std::get_if<LineSpan>(&c.spans[i])) l2->end = startPoint(c.spans[j]);
  }
  c.closed = closed; return c;
}

// make an all-arc closed contour bit-continuous by ulp nudges; false if impossible
static bool exactify(Contour& c) {
  std::size_t n = c.spans.size();
  for (std::size_t i = 1; i < n; ++i) {
    Vec2 j = endPoint(c.spans[i - 1]); auto& b = std::get<ArcSpan>(c.spans[i]);
    if (b.startPoint() == j) continue;
    ArcSpan m = b; m.centre = j - Vec2::fromAngle(b.startAngle, b.radius);
    bool ok = false; ArcSpan base = m;
    for (int dr = -6; dr <= 6 && !ok; ++dr) for (int dx = -6; dx <= 6 && !ok; ++dx) for (int dy = -6; dy <= 6 && !ok; ++dy) {
      ArcSpan t = base; t.radius = ulps(base.radius, dr); t.centre = Vec2{ulps(base.centre.x, dx), ulps(base.centre.y, dy)};
      if (t.startPoint() == j) { b = t; ok = true; } }
    if (!ok) return false;
  }
  Vec2 j0 = startPoint(c.spans[0]); auto& last = std::get<ArcSpan>(c.spans[n - 1]);
  if (last.endPoint() == j0) return true;
  double ta = std::atan2(j0.y - last.centre.y, j0.x - last.centre.x);
  double w = std::remainder(ta - last.endAngle, 2 * PI); double e0 = last.endAngle + w;
  for (int k = -64; k <= 64; ++k) { ArcSpan t = last; t.endAngle = ulps(e0, k); if (t.endPoint() == j0) { last = t; return true; } }
  return false;
}
static void perturb(Contour& c, Rng& g, int K) {
  auto p = [&](double& v) { int k = 1 + (int)(g.uni() * K); if (g.uni() < 0.5) k = -k; if (g.uni() < 0.7) v = ulps(v, k); };
  for (Span& s : c.spans) {
    if (auto* l = std::get_if<LineSpan>(&s)) { p(l->start.x); p(l->start.y); p(l->end.x); p(l->end.y); }
    else if (auto* a = std::get_if<ArcSpan>(&s)) { p(a->centre.x); p(a->centre.y); p(a->radius); p(a->startAngle); p(a->endAngle); }
  }
}
static const Contour* gIn = nullptr;
static double inGap() { double g = 0; const auto& sp = gIn->spans; std::size_t n = sp.size(); for (std::size_t i = 0; i < n; ++i) { std::size_t j = (i + 1) % n; if (!gIn->closed && j == 0) break; g = std::max(g, distance(endPoint(sp[i]), startPoint(sp[j]))); } return g; }
struct Stat { long skipped = 0, offs = 0, chains = 0, arcLoops = 0, junc = 0, behind0 = 0, behind12 = 0, lineNI = 0, arcGapBig = 0, cbs = 0, behindArcLoop = 0; double worstBehind = 0, worstGapRel = 0; };
static void measure(const std::vector<Contour>& out, Stat& st, std::uint64_t id, const char* tag) {
  const Tolerance tol{};
  for (const Contour& c : out) {
    st.chains++; std::size_t n = c.spans.size();
    bool allArcs = std::all_of(c.spans.begin(), c.spans.end(), [](const Span& s) { return std::holds_alternative<ArcSpan>(s); });
    bool arcLoop = c.closed && n >= 2 && allArcs; if (arcLoop) st.arcLoops++;
    st.cbs += (long)testing::countBackwardSteps(c, tol);
    std::size_t J = c.closed && n > 1 ? n : (n ? n - 1 : 0);
    for (std::size_t i = 0; i < J; ++i) {
      const Span& a = c.spans[i]; const Span& b = c.spans[(i + 1) % n]; st.junc++;
      Vec2 e = endPoint(a), s = startPoint(b); double dot = (s - e).dot(tangentAt(a, 1.0));
      if (dot < 0) { st.behind0++; if (arcLoop) st.behindArcLoop++; st.worstBehind = std::max(st.worstBehind, -dot); }
      if (dot < -1e-12) { st.behind12++; if (std::getenv("V")) std::printf("BEHIND %s id=%llx dot=%.3g arcLoop=%d n=%zu i=%zu gap=%.3g closeGap=%.3g inGap=%.3g\n", tag, (unsigned long long)id, dot, arcLoop, n, i, distance(e, s), c.closed ? distance(endPoint(c.spans[n-1]), startPoint(c.spans[0])) : -1.0, inGap()); }
      bool line = std::holds_alternative<LineSpan>(a) || std::holds_alternative<LineSpan>(b);
      if (line && !(e == s)) st.lineNI++;
      if (!line) { const auto& ab = std::get<ArcSpan>(b); double sc = std::max({1.0, std::fabs(e.x), std::fabs(e.y), ab.radius}); double gr = distance(e, s) / sc; st.worstGapRel = std::max(st.worstGapRel, gr); if (gr > 1e-12) { st.arcGapBig++; if (std::getenv("V2")) std::printf("GAP %s id=%llx gap=%.3g rel=%.3g inGap=%.3g closed=%d i=%zu n=%zu\n", tag, (unsigned long long)id, distance(e, s), gr, inGap(), c.closed, i, n); } }
    }
  }
}

#include <chrono>
int main(int argc, char** argv) {
  int n = std::atoi(argv[1]); long N = std::atol(argv[2]); int K = std::atoi(argv[3]);
  const Vec2 Ts[] = {{1e5, -4e4}, {2e5, 1.3e5}, {1e4,-8e3}};
  double worst = 0; Stat S; long gapsBig=0;
  for (long s = 0; s < N; ++s) {
    Rng g(1000 + s); Vec2 T = Ts[s % 3];
    Contour c; c.closed = true; double R = g.range(5, 50);
    std::vector<Vec2> pts; for (int i = 0; i < n; ++i) { double a = 2*PI*i/n; double rr = R*(1+0.02*std::sin(7*a)); pts.push_back(T + Vec2{rr*std::cos(a), rr*std::sin(a)}); }
    for (int i = 0; i < n; ++i) { double sw = g.range(0.05, 1.0); if (g.uni()<0.2) sw=-sw; c.spans.push_back(arcThrough(pts[i], pts[(i+1)%n], sw)); }
    exactify(c); if (K > 0) { Rng pg(s*7+3); perturb(c, pg, K); }
    gIn = &c;
    for (JoinType j : {JoinType::Round, JoinType::Sharp}) for (double d : {0.01, 0.3}) {
      auto t0 = std::chrono::steady_clock::now();
      auto out = offsetOpenContour(c, d, g.uni()<0.5?OffsetSide::Left:OffsetSide::Right, j, Tolerance{}, 2.0);
      double dt = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count(); worst = std::max(worst, dt);
      measure(out, S, s, "perf");
    }
  }
  std::printf("n=%d K=%d worst time %.3fs behind0 %ld behind12 %ld arcGapBig %ld worstGapRel %.3g arcLoops %ld\n", n, K, worst, S.behind0, S.behind12, S.arcGapBig, S.worstGapRel, S.arcLoops);
}
