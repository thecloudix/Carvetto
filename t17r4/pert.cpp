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
int main(int argc, char** argv) {
  long N = std::atol(argv[1]); std::uint64_t seed0 = std::strtoull(argv[2], nullptr, 0);
  const Vec2 Ts[] = {{1e4, -8e3}, {-1e5, -4e4}, {3e4, 7e4}, {-6304, -8204}, {2e5, 1.3e5}, {0.5, -0.25}};
  const int Ks[] = {1, 2, 4, 8, 16, 32, 64};
  const JoinType Js[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};
  Stat S[5];
  for (long k = 0; k < N; ++k) {
    std::uint64_t seed = seed0 + (std::uint64_t)k; Rng g(seed);
    int fam = (int)(k % 5); int gfam = fam == 4 ? 0 : fam; Vec2 T = Ts[(k / 5) % 6];
    Contour base = gen(g, gfam, T);
    double d = std::exp(g.range(std::log(0.01), std::log(3.0)));
    OffsetSide side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    JoinType j = gfam == 0 && g.uni() < 0.6 ? JoinType::Round : Js[(int)(g.uni() * 3) % 3];
    for (int ki = 0; ki < 7; ++ki) for (int rep = 0; rep < 3; ++rep) {
      Contour c = base; Rng pg(seed * 131 + (std::uint64_t)(ki * 7 + rep)); perturb(c, pg, Ks[ki]); if (fam == 4 && !exactify(c)) { S[4].skipped++; continue; }
      double dd = rep == 2 ? ulps(d, (int)(pg.uni() * 2 * Ks[ki]) - Ks[ki]) : d;
      if (std::getenv("ONE") && std::strtoull(std::getenv("ONE"), nullptr, 0) != seed * 100 + (std::uint64_t)(ki * 7 + rep)) continue;
      if (std::getenv("ONE")) { std::printf("// d=%.17g side=%d join=%d closed=%d\n", dd, (int)side, (int)j, c.closed); for (auto& sp : c.spans) { if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("arc(%.17g, %.17g, %.17g, %.17g, %.17g, %s);\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "ccw" : "cw"); else { auto& l = std::get<LineSpan>(sp); std::printf("line(%.17g, %.17g, %.17g, %.17g);\n", l.start.x, l.start.y, l.end.x, l.end.y); } } }
      gIn = &c; auto out = offsetOpenContour(c, dd, side, j, Tolerance{}, 2.0);
      if (std::getenv("ONE")) for (auto& oc : out) { std::printf("// out closed=%d\n", oc.closed); for (auto& sp : oc.spans) if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("//  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle); else std::printf("//  L\n"); }
      S[fam].offs++; measure(out, S[fam], seed * 100 + (std::uint64_t)(ki * 7 + rep), "rnd");
    }
  }
  // lens base from the test with ±1..64 ulps per single value
  Stat L;
  if (seed0 == 0) {
    const double b[16] = {-6308.8745429246874, 31581.569216038675, 39785.30676902362, -1.570920458202786, -1.5706721953870071, -6303.9359367948146, -8203.7372464668806, -6302.9359367948146, -8193.37889961566, -6305.2879140935202, -8192.37889961566, -6304.1211345246902, -8203.558166433495, -6313.6279513246845, -8203.558166433495, 0.089633408917393007};
    for (int w = 0; w < 16; ++w) for (int k = -64; k <= 64; ++k) {
      double v[16]; std::copy(b, b + 16, v); v[w] = ulps(v[w], k);
      Contour in; in.spans.emplace_back(ArcSpan{{v[0], v[1]}, v[2], v[3], v[4], ArcDirection::CCW});
      in.spans.emplace_back(LineSpan{{v[5], v[6]}, {v[7], v[8]}}); in.spans.emplace_back(LineSpan{{v[7], v[8]}, {v[9], v[10]}});
      in.spans.emplace_back(LineSpan{{v[9], v[10]}, {v[11], v[12]}}); in.spans.emplace_back(LineSpan{{v[11], v[12]}, {v[13], v[14]}});
      gIn = &in; for (JoinType jj : Js) { auto out = offsetOpenContour(in, v[15], OffsetSide::Left, jj, Tolerance{}, 2.0); L.offs++; measure(out, L, (std::uint64_t)(w * 1000 + k + 500), "lens"); }
    }
  }
  const char* names[] = {"arcLoopClosed", "mixClosed", "mixOpen", "bigRadiusOpen", "arcLoopExact"};
  auto pr = [](const char* nm, const Stat& s) { std::printf("%-14s offs %ld chains %ld arcLoops %ld junc %ld behind(<0) %ld behind(<-1e-12) %ld [in arc loops %ld] worstBehind %.3g lineNotIdent %ld arcGap>1e-12rel %ld worstGapRel %.3g countBackwardSteps %ld\n", nm, s.offs, s.chains, s.arcLoops, s.junc, s.behind0, s.behind12, s.behindArcLoop, s.worstBehind, s.lineNI, s.arcGapBig, s.worstGapRel, s.cbs); };
  for (int f = 0; f < 5; ++f) pr(names[f], S[f]); std::printf("arcLoopExact skipped (not exactifiable) %ld\n", S[4].skipped);
  if (seed0 == 0) pr("lens+-64", L);
}
