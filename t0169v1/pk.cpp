// t0169v1 pk: Pocket adversarial families + own exact clearance oracle (oc2.hpp) + hashes.
// usage: pk <family> <seed0> <n> ; env PK_FAR (translation, mm), PK_SCALE (geometry and tools scaled),
//   PK_DFR (distance = chordal * DFR, floored at 1e-6), PK_CH (chordal), PK_STRAT (0 offset 1 raster),
//   PK_REST (1 force a rest tool, 0 none), PK_ALLOW (allowance), PK_NOCHECK (hash only), PK_VERB.
// families: 0 branch junction generator (copy), 1 letters with filleted reflex corners (concave arcs) and
//   arc/bezier edges, 2 bitten squares (concave arcs/beziers into the pocket), 3 dumbbells / thin necks
//   (circle-line reflex junctions, neck ~ tool diameter), 4 islands (circles/beziers/D shapes) at gaps
//   near the tool diameter, 5 wavy Bezier blobs with inflections, 6 the T-0153 A (nest_frame) with arcs,
//   7 gears (alternating convex/concave arcs), 8 many reflex vertices (saw-tooth with arc teeth)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "oc2.hpp"
#ifdef PK_XO
#include "/home/chris/wt-T0169/core/tests/toolpath/pocket_curve_oracle.hpp"
#endif
using namespace carvetto; using namespace carvetto::toolpath;
using geometry::ArcDirection; using geometry::ArcSpan; using geometry::BezierSpan; using geometry::Contour; using geometry::LineSpan; using geometry::Span; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846, TAU = 2 * PI;
struct SM { std::uint64_t s; std::uint64_t next() { std::uint64_t z = (s += 0x9E3779B97F4A7C15ULL); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL; return z ^ (z >> 31); }
  double U(double a, double b) { return a + (b - a) * double(next() >> 11) * 0x1.0p-53; } int I(int a, int b) { return a + int(next() % std::uint64_t(b - a + 1)); } };
static double envd(const char* n, double d) { const char* v = getenv(n); return v ? atof(v) : d; }
static Span edge(Vec2 P, Vec2 Q, int kind, double s, double skew) {  // s>0 bulges left of P->Q
  if (kind == 0 || s == 0.0) return LineSpan{P, Q};
  Vec2 d = Q - P; double len = d.length(); Vec2 n = (d * (1.0 / len)).perp();
  if (kind == 1) { double h = 0.5 * len, R = (h * h + s * s) / (2 * std::fabs(s)); Vec2 c = (P + Q) * 0.5 - n * ((R - std::fabs(s)) * (s > 0 ? 1 : -1));
    double a0 = std::atan2(P.y - c.y, P.x - c.x), a1 = std::atan2(Q.y - c.y, Q.x - c.x); Vec2 apex = (P + Q) * 0.5 + n * s;
    ArcSpan ccw{c, R, a0, a1, ArcDirection::CCW}, cw{c, R, a0, a1, ArcDirection::CW};
    return distance(ccw.pointAt(0.5), apex) < distance(cw.pointAt(0.5), apex) ? ccw : cw; }
  return BezierSpan{P, P + d * (1.0 / 3 + skew) + n * (4.0 / 3 * s), P + d * (2.0 / 3 + skew) + n * (4.0 / 3 * s), Q};
}
static Contour closedOf(const std::vector<Vec2>& v, SM& r, double curveP, double bulge) {  // random edge kinds
  Contour c; c.closed = true; for (size_t i = 0; i < v.size(); ++i) { Vec2 P = v[i], Q = v[(i + 1) % v.size()];
    int k = r.U(0, 1) < curveP ? r.I(1, 2) : 0; c.append(edge(P, Q, k, r.U(-bulge, bulge) * distance(P, Q), r.U(-0.1, 0.1))); } return c; }
// a rounded polygon: each corner replaced by a fillet arc of radius rf (reflex corners get concave arcs)
static Contour filleted(const std::vector<Vec2>& v, const std::vector<double>& rf, SM& r, double curveP, double bulge) {
  size_t n = v.size(); std::vector<Vec2> a(n), b(n); std::vector<Span> fil(n); std::vector<bool> has(n);
  double area = 0; for (size_t i = 0; i < n; ++i) area += v[i].cross(v[(i + 1) % n]); double sg = area > 0 ? 1 : -1;
  for (size_t i = 0; i < n; ++i) { Vec2 P = v[(i + n - 1) % n], V = v[i], Q = v[(i + 1) % n]; Vec2 u1 = (P - V).normalized(), u2 = (Q - V).normalized();
    double ang = std::acos(std::clamp(u1.dot(u2), -1.0, 1.0)); double t = rf[i] / std::tan(ang / 2);
    double lim = 0.4 * std::min(distance(P, V), distance(Q, V)); if (rf[i] <= 0 || t > lim || ang < 1e-3 || ang > PI - 1e-3) { a[i] = b[i] = V; has[i] = false; continue; }
    a[i] = V + u1 * t; b[i] = V + u2 * t; Vec2 bis = (u1 + u2).normalized(); Vec2 c = V + bis * (rf[i] / std::sin(ang / 2));
    double s0 = std::atan2(a[i].y - c.y, a[i].x - c.x), s1 = std::atan2(b[i].y - c.y, b[i].x - c.x);
    bool convex = sg * (V - P).cross(Q - V) > 0; ArcDirection dir = (convex == (sg > 0)) ? ArcDirection::CCW : ArcDirection::CW;
    fil[i] = ArcSpan{c, rf[i], s0, s1, dir}; has[i] = true; }
  Contour c; c.closed = true;
  for (size_t i = 0; i < n; ++i) { if (has[i]) c.append(fil[i]); size_t j = (i + 1) % n; Vec2 P = b[i], Q = a[j];
    if (distance(P, Q) > 1e-9) { int k = r.U(0, 1) < curveP ? r.I(1, 2) : 0; c.append(edge(P, Q, k, r.U(-bulge, bulge) * distance(P, Q), r.U(-0.1, 0.1))); } }
  return c;
}
static Contour circleC(Vec2 c, double R, bool asBez) {
  Contour k; k.closed = true; if (!asBez) { k.append(ArcSpan{c, R, 0.0, PI, ArcDirection::CCW}); k.append(ArcSpan{c, R, PI, TAU, ArcDirection::CCW}); return k; }
  for (auto& b : ArcSpan{c, R, 0.3, 0.3 + TAU - 1e-12, ArcDirection::CCW}.toBeziers()) k.append(b); return k;
}
static PocketTool tool(const char* nm, int num, double d, double so, double pd) { PocketTool t; t.tool.id = nm; t.tool.name = nm; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = d;
  auto& cd = t.tool.defaultCuttingData; cd.toolNumber = num; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.stepover = so; cd.passDepth = pd; return t; }
struct Case { std::vector<Contour> cs; std::vector<PocketTool> tools; PocketParams p; std::string what; };
static std::vector<Vec2> letterPts(int which, double s) {
  switch (which) {
    case 0: return {{0, 0}, {1.125, 0}, {1.5, 1.5}, {3, 1.5}, {3.375, 0}, {4.5, 0}, {2.8125, 6}, {1.6875, 6}};  // A outer
    case 1: return {{0, 0}, {4, 0}, {4, 1.2}, {1.3, 1.2}, {1.3, 6}, {0, 6}};                                     // L
    case 2: return {{0, 5}, {5, 5}, {5, 6}, {0, 6}, {0, 5}, {2, 5}, {2, 0}, {3, 0}, {3, 5}}; // T (fixed below)
    case 3: return {{0, 0}, {1.2, 0}, {1.2, 2.4}, {3.8, 2.4}, {3.8, 0}, {5, 0}, {5, 6}, {3.8, 6}, {3.8, 3.6}, {1.2, 3.6}, {1.2, 6}, {0, 6}};  // H
    default: return {{0, 0}, {5, 0}, {5, 1.2}, {3.1, 1.2}, {3.1, 4.8}, {5, 4.8}, {5, 6}, {0, 6}};  // C-ish
  }
}
static Case gen(int fam, std::uint64_t seed) {
  SM r{seed * 7919 + std::uint64_t(fam) * 1000003}; Case c; double toolD = 1.0;
  if (fam == 0) {  // branch generator semantics (re-implemented)
    int n = 4 + int(r.next() % 4); bool star = r.next() % 2 == 0; double inner = star ? r.U(0.55, 0.8) : 1.0, ph = r.U(0, TAU);
    std::vector<Vec2> v; int m = star ? 2 * n : n; for (int i = 0; i < m; ++i) { double a = ph + TAU * i / m, rr = (star && i % 2) ? 10 * inner : 10; v.push_back({rr * std::cos(a), rr * std::sin(a)}); }
    c.cs.push_back(closedOf(v, r, 0.67, star ? 0.08 : 0.15));
    if (r.next() % 2 == 0) { double ir = r.U(1, 2.5) * inner, ia = r.U(0, TAU); Vec2 P = Vec2::fromAngle(ia, ir), Q = P * -1.0; Contour is; is.closed = true;
      is.append(edge(P, Q, 1 + int(r.next() % 2), r.U(0.3, 1) * ir, r.U(-0.1, 0.1))); is.append(edge(Q, P, int(r.next() % 3), r.U(-0.3, 0.3) * ir, 0)); c.cs.push_back(is); }
    toolD = r.U(0.4, 3.0) * inner; c.what = star ? "star" : "poly";
  } else if (fam == 1) {
    int w = r.I(0, 4); double s = r.U(0.5, 3); auto v = letterPts(w == 2 ? 4 : w, s); for (auto& q : v) q = q * s;
    double rot = r.U(0, TAU); for (auto& q : v) q = Vec2{q.x * std::cos(rot) - q.y * std::sin(rot), q.x * std::sin(rot) + q.y * std::cos(rot)};
    std::vector<double> rf(v.size()); for (auto& x : rf) x = r.U(0, 1) < 0.7 ? r.U(0.0, 0.5) * s : 0.0;
    c.cs.push_back(filleted(v, rf, r, 0.4, 0.06)); toolD = r.U(0.1, 0.9) * s; c.what = "letter" + std::to_string(w);
    if (w == 0) { std::vector<Vec2> h{{1.8, 2.4375}, {2.25, 4.3125}, {2.7, 2.4375}}; for (auto& q : h) { q = q * s; q = Vec2{q.x * std::cos(rot) - q.y * std::sin(rot), q.x * std::sin(rot) + q.y * std::cos(rot)}; } c.cs.push_back(closedOf(h, r, 0.5, 0.1)); toolD = r.U(0.05, 0.4) * s; }
  } else if (fam == 2) {
    double L = r.U(5, 20); int k = r.I(1, 6); Contour sq; sq.closed = true; std::vector<Vec2> cr{{0, 0}, {L, 0}, {L, L}, {0, L}};
    for (int e = 0; e < 4; ++e) { Vec2 P = cr[e], Q = cr[(e + 1) % 4]; int bites = r.I(0, k); std::vector<double> ts{0}; for (int b = 0; b < bites; ++b) ts.push_back(r.U(0.05, 0.95)); ts.push_back(1); std::sort(ts.begin(), ts.end());
      for (size_t j = 0; j + 1 < ts.size(); ++j) { Vec2 a = P + (Q - P) * ts[j], b = P + (Q - P) * ts[j + 1]; if (distance(a, b) < 1e-6) continue; bool bite = j % 2 == 1 || r.U(0, 1) < 0.3;
        sq.append(edge(a, b, bite ? r.I(1, 2) : 0, bite ? r.U(0.05, 0.5) * distance(a, b) : 0, r.U(-0.1, 0.1))); } }
    c.cs.push_back(sq); toolD = r.U(0.2, 0.25 * L); c.what = "bitten";
  } else if (fam == 3) {
    double R1 = r.U(2, 6), R2 = r.U(2, 6), gap = r.U(1, 8); toolD = r.U(0.3, 2.0); double w = toolD * r.U(1.0001, 1.6) + r.U(0, 0.01);
    if (w >= 2 * std::min(R1, R2) * 0.9) w = std::min(R1, R2); Vec2 c1{0, 0}, c2{R1 + gap + R2, 0}; double h = w / 2;
    double a1 = std::asin(h / R1), a2 = std::asin(h / R2); Contour k; k.closed = true;
    Vec2 p1t = c1 + Vec2::fromAngle(a1, R1), p2t = c2 + Vec2::fromAngle(PI - a2, R2), p2b = c2 + Vec2::fromAngle(PI + a2, R2), p1b = c1 + Vec2::fromAngle(-a1, R1);
    int nk = r.I(0, 2); auto neck = [&](Vec2 P, Vec2 Q) { return edge(P, Q, nk, nk ? r.U(-0.3, 0.3) * h : 0.0, 0.0); };
    k.append(neck(p1t, p2t)); k.append(ArcSpan{c2, R2, PI - a2, -(PI - a2), ArcDirection::CW}); k.append(neck(p2b, p1b)); k.append(ArcSpan{c1, R1, -a1, a1 - TAU, ArcDirection::CW});
    // fix arc directions: we go p1t -> p2t (top neck, left to right), around c2 the long way (clockwise from PI-a2 to PI+a2)
    k = Contour{}; k.closed = true; k.append(neck(p1t, p2t)); k.append(ArcSpan{c2, R2, PI - a2, PI + a2 - TAU, ArcDirection::CW}); k.append(neck(p2b, p1b)); k.append(ArcSpan{c1, R1, -a1, a1 - TAU, ArcDirection::CW});
    c.cs.push_back(k); c.what = "dumbbell";
    if (r.U(0, 1) < 0.5) c.cs.push_back(circleC(c1 + Vec2{r.U(-0.3, 0.3), r.U(-0.3, 0.3)} * R1, R1 * r.U(0.1, 0.35), r.U(0, 1) < 0.5));
  } else if (fam == 4) {
    double L = r.U(8, 25); toolD = r.U(0.3, 2.5); c.cs.push_back(closedOf({{0, 0}, {L, 0}, {L, L * r.U(0.5, 1)}, {0, L * r.U(0.5, 1)}}, r, 0.5, 0.05));
    int ni = r.I(1, 6); for (int i = 0; i < ni; ++i) { Vec2 cc{r.U(0.2, 0.8) * L, r.U(0.2, 0.5) * L}; double R = r.U(0.5, 2.5); int t = r.I(0, 2);
      if (t < 2) c.cs.push_back(circleC(cc, R, t == 1)); else { double ia = r.U(0, TAU); Vec2 P = cc + Vec2::fromAngle(ia, R), Q = cc - Vec2::fromAngle(ia, R); Contour is; is.closed = true; is.append(edge(P, Q, r.I(1, 2), r.U(0.3, 1) * R, 0)); is.append(LineSpan{Q, P}); c.cs.push_back(is); } }
    // pairs at gap ~ tool diameter
    if (r.U(0, 1) < 0.6) { double R = r.U(0.5, 2); Vec2 a{0.3 * L, 0.75 * L * 0.5}; double g = toolD * r.U(0.99, 1.2) + 2 * R; c.cs.push_back(circleC(a, R, false)); c.cs.push_back(circleC(a + Vec2{g + 0.002, 0}, R, true)); }
    c.what = "islands";
  } else if (fam == 5) {
    int n = r.I(5, 12); double R = r.U(3, 10); std::vector<Vec2> v; for (int i = 0; i < n; ++i) { double a = TAU * i / n; double rr = R * r.U(0.6, 1.0); v.push_back(Vec2::fromAngle(a, rr)); }
    Contour k; k.closed = true; for (int i = 0; i < n; ++i) { Vec2 P = v[i], Q = v[(i + 1) % n]; Vec2 d = Q - P, nn = d.normalized().perp(); double s1 = r.U(-0.4, 0.4) * d.length(), s2 = r.U(-0.4, 0.4) * d.length();
      k.append(BezierSpan{P, P + d * 0.33 + nn * s1, P + d * 0.67 + nn * s2, Q}); }
    c.cs.push_back(k); toolD = r.U(0.2, 0.3 * R); c.what = "wavy";
  } else if (fam == 6) {
    double s = r.U(0.3, 4); auto v = letterPts(0, s); for (auto& q : v) q = q * s; c.cs.push_back(closedOf(v, r, 0.3, 0.03));
    std::vector<Vec2> h{{1.8, 2.4375}, {2.25, 4.3125}, {2.7, 2.4375}}; for (auto& q : h) q = q * s; c.cs.push_back(closedOf(h, r, 0.3, 0.05));
    toolD = s * r.U(0.05, 0.4); c.what = "A";
  } else if (fam == 7) {
    int n = r.I(6, 24); double R = r.U(4, 15), dep = r.U(0.05, 0.3) * R; Contour k; k.closed = true; std::vector<Vec2> v;
    for (int i = 0; i < 2 * n; ++i) { double a = TAU * i / (2 * n); v.push_back(Vec2::fromAngle(a, i % 2 ? R - dep : R)); }
    for (int i = 0; i < 2 * n; ++i) { Vec2 P = v[i], Q = v[(i + 1) % (2 * n)]; double s = (i % 2 ? 1 : -1) * r.U(0.05, 0.3) * distance(P, Q); k.append(edge(P, Q, r.I(0, 2), s, 0)); }
    c.cs.push_back(k); toolD = r.U(0.1, 0.25) * R; c.what = "gear";
  } else if (fam == 9) {  // line-only: stars, letters, saw teeth, polygon islands (MoveCheck margin 0 path)
    int t = r.I(0, 2);
    if (t == 0) { int n = r.I(4, 12); double inner = r.U(0.3, 0.9), ph = r.U(0, TAU); std::vector<Vec2> v; for (int i = 0; i < 2 * n; ++i) { double a = ph + TAU * i / (2 * n); v.push_back(Vec2::fromAngle(a, i % 2 ? 10 * inner : 10)); } c.cs.push_back(closedOf(v, r, 0, 0)); toolD = r.U(0.2, 2.5) * inner; }
    else if (t == 1) { int w = r.I(0, 4); double s = r.U(0.5, 3); auto v = letterPts(w == 2 ? 4 : w, s); for (auto& q : v) q = q * s; c.cs.push_back(closedOf(v, r, 0, 0)); toolD = r.U(0.05, 0.6) * s;
      if (w == 0) { std::vector<Vec2> h{{1.8, 2.4375}, {2.25, 4.3125}, {2.7, 2.4375}}; for (auto& q : h) q = q * s; c.cs.push_back(closedOf(h, r, 0, 0)); toolD = r.U(0.05, 0.4) * s; } }
    else { int n = r.I(5, 40); double L = r.U(10, 60), hgt = r.U(0.5, 4), pitch = L / n; std::vector<Vec2> v{{0, -5}, {L, -5}}; for (int i = n; i > 0; --i) { v.push_back({i * pitch, 0}); v.push_back({(i - 0.5) * pitch, hgt}); } v.push_back({0, 0}); c.cs.push_back(closedOf(v, r, 0, 0)); toolD = r.U(0.1, 0.9) * pitch + r.U(0, 0.5);
      if (r.U(0, 1) < 0.5) { std::vector<Vec2> h; int m = r.I(3, 8); for (int j = 0; j < m; ++j) h.push_back(Vec2{L / 2, -2.5} + Vec2::fromAngle(TAU * j / m + r.U(-0.2, 0.2), r.U(0.5, 1.5))); c.cs.push_back(closedOf(h, r, 0, 0)); } }
    c.what = "lines" + std::to_string(t);
  } else {
    int n = r.I(5, 40); double L = r.U(10, 60), hgt = r.U(0.5, 4); Contour k; k.closed = true; double pitch = L / n;
    std::vector<Vec2> v{{0, -5}, {L, -5}}; for (int i = n; i > 0; --i) { v.push_back({i * pitch, 0}); v.push_back({(i - 0.5) * pitch, hgt}); } v.push_back({0, 0});
    for (size_t i = 0; i < v.size(); ++i) { Vec2 P = v[i], Q = v[(i + 1) % v.size()]; int kind = i >= 2 ? r.I(0, 2) : 0; k.append(edge(P, Q, kind, kind ? r.U(-0.25, 0.25) * distance(P, Q) : 0, 0)); }
    c.cs.push_back(k); toolD = r.U(0.1, 0.9) * pitch + r.U(0, 0.5); c.what = "saw";
  }
  const double chords[] = {1e-4, 3e-4, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1}; const double dfr[] = {1, 1, 1, 1.0 / 3, 0.1, 1e-3};
  double ch = chords[r.next() % 9], df = dfr[r.next() % 6]; ch = envd("PK_CH", ch); df = envd("PK_DFR", df);
  c.p.tolerance.chordal = ch; c.p.tolerance.distance = std::max(1e-6, ch * df);
  double sc = envd("PK_SCALE", 1.0), far = envd("PK_FAR", 0.0); toolD *= sc;
  if (sc != 1.0 || far != 0.0) { geometry::Transform2 m = geometry::Transform2::translation({far, -0.7 * far}) * geometry::Transform2::scaling(sc, sc); for (auto& k : c.cs) k = k.transformed(m); }
  c.tools.push_back(tool("T1", 1, toolD, r.U(0.2, 0.9) * toolD, 2.0 * sc));
  bool rest = r.next() % 3 == 0; if (getenv("PK_REST")) rest = atoi(getenv("PK_REST")) != 0;
  if (rest) { double d2 = r.U(0.2, 0.7) * toolD; c.tools.push_back(tool("T2", 2, d2, r.U(0.2, 0.9) * d2, 2.0 * sc)); }
  if (r.next() % 4 == 0 && rest) { double d3 = r.U(0.2, 0.7) * c.tools.back().tool.diameter; c.tools.push_back(tool("T3", 3, d3, r.U(0.2, 0.9) * d3, 2.0 * sc)); }
  PocketParams& p = c.p; p.strategy = r.next() % 2 ? PocketStrategy::Raster : PocketStrategy::Offset; if (getenv("PK_STRAT")) p.strategy = atoi(getenv("PK_STRAT")) ? PocketStrategy::Raster : PocketStrategy::Offset;
  p.direction = r.next() % 2 ? CutDirection::Climb : CutDirection::Conventional; p.rasterAngle = r.U(0, 3.2); p.profilePass = static_cast<ProfilePass>(r.next() % 3);
  int al = int(r.next() % 3); p.allowance = al == 0 ? 0.0 : al == 1 ? r.U(0, 0.05) * toolD : r.U(0, 0.3) * toolD; p.allowance = envd("PK_ALLOW", p.allowance);
  p.cutDepth = (r.next() % 2 ? 1.0 : 3.0) * sc; p.safeZ = 5.0 * sc; p.rampPlunge = r.next() % 4 == 0; p.rampDistance = r.U(0.5, 10) * sc;
  return c;
}
static unsigned long long HH;
static void hb(const void* p, size_t n) { auto* q = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { HH ^= q[i]; HH *= 1099511628211ULL; } }
int main(int argc, char** argv) {
  int fam = atoi(argv[1]); std::uint64_t s0 = strtoull(argv[2], 0, 10); int n = atoi(argv[3]); bool check = !getenv("PK_NOCHECK"); bool verb = getenv("PK_VERB");
  double worstAll = 1e300; int viol = 0, neg = 0, ncase = 0;
  for (int k = 0; k < n; ++k) {
    std::uint64_t seed = s0 + k; Case c = gen(fam, seed);
    auto t0 = std::chrono::steady_clock::now(); double c0 = double(std::clock()) / CLOCKS_PER_SEC;
    PocketResult res = pocketToolpath(c.cs, c.tools, c.p);
    double cpu = double(std::clock()) / CLOCKS_PER_SEC - c0; (void)t0;
    HH = 1469598103934665603ULL; size_t moves = 0, plunges = 0, retracts = 0, rapids = 0; double cutlen = 0;
    for (auto& tp : res.toolpaths) { for (size_t i = 0; i < tp.moves.size(); ++i) { auto& m = tp.moves[i]; int kk = (int)m.kind; hb(&kk, 4); hb(&m.to.x, 8); hb(&m.to.y, 8); hb(&m.to.z, 8); double f = m.feedOverride.value_or(-1); hb(&f, 8);
        ++moves; plunges += m.kind == MoveKind::Plunge; retracts += m.kind == MoveKind::Retract; rapids += m.kind == MoveKind::Rapid; if (m.kind != MoveKind::Rapid) { auto s = tp.startOf(i); cutlen += std::hypot(m.to.x - s.x, m.to.y - s.y); } } int sep = -7; hb(&sep, 4); }
    for (auto& d : res.diagnostics) hb(d.message.data(), d.message.size());
    double worst = 1e300; bool cert = true; std::string wdesc;
    if (check) for (size_t ti = 0; ti < res.toolpaths.size(); ++ti) { auto& tp = res.toolpaths[ti]; if (tp.isEmpty()) continue; double rad = 0.5 * c.tools[ti].tool.diameter;
      oc2::Rep rp = oc2::check(tp, c.cs, rad, c.p.allowance - 1e-5, -c.p.startDepth, 0.05 * envd("PK_SCALE", 1.0));
      double v = rp.minc - c.p.allowance; if (v < worst) { worst = v; char b[300]; std::snprintf(b, sizeof b, "tool %zu move %zu kind %d (%.9f,%.9f)->(%.9f,%.9f)", ti, rp.worst, rp.worstKind, rp.wa.x, rp.wa.y, rp.wb.x, rp.wb.y); wdesc = b; }
      if (!rp.cert) cert = false;
#ifdef PK_XO
      { auto xr = carvetto::test::pocket_curve::wallClearance(tp, c.cs, rad, c.p.allowance - 1e-5, -c.p.startDepth); std::printf("XO tool %zu branchOracle min-allow %.4e certified %d | oc2 %.4e cert %d\n", ti, xr.minClearance - c.p.allowance, (int)xr.certified, v, (int)rp.cert); }
#endif
      }
    if (getenv("PK_DUMP")) { for (auto& k : c.cs) { std::printf("contour\n"); for (auto& sp : k.spans) { if (auto* l = std::get_if<LineSpan>(&sp)) std::printf("  L %.17g %.17g %.17g %.17g\n", l->start.x, l->start.y, l->end.x, l->end.y);
          else if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("  A %.17g %.17g r %.17g %.17g %.17g dir %d\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CW ? 1 : 0);
          else { auto& z = std::get<BezierSpan>(sp); std::printf("  B %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", z.p0.x, z.p0.y, z.p1.x, z.p1.y, z.p2.x, z.p2.y, z.p3.x, z.p3.y); } } }
      for (size_t ti = 0; ti < res.toolpaths.size(); ++ti) { std::printf("toolpath %zu D %.17g\n", ti, c.tools[ti].tool.diameter); for (auto& m : res.toolpaths[ti].moves) std::printf("  M %d %.9f %.9f %.9f\n", (int)m.kind, m.to.x, m.to.y, m.to.z); }
      for (auto& d : res.diagnostics) std::printf("DIAG %s\n", d.message.c_str());
      for (size_t ti = 0; ti < res.plans.size(); ++ti) for (auto& k : res.plans[ti].centreRegion) { std::printf("centre %zu:", ti); for (auto& sp : k.spans) { auto q = geometry::startPoint(sp); std::printf(" %.6f,%.6f", q.x, q.y); } std::printf("\n"); } }
    ++ncase; worstAll = std::min(worstAll, worst); if (worst < -1e-5) ++viol; if (worst < 0) ++neg;
    std::printf("case %llu fam %d %s ch %g dist %g D %g al %g strat %d pass %d tools %zu ramp %d hash %016llx moves %zu rapid %zu plunge %zu retract %zu cut %.6f cpu %.4f worst %.4e %s%s\n",
      (unsigned long long)seed, fam, c.what.c_str(), c.p.tolerance.chordal, c.p.tolerance.distance, c.tools[0].tool.diameter, c.p.allowance, (int)c.p.strategy, (int)c.p.profilePass, c.tools.size(), (int)c.p.rampPlunge,
      HH, moves, rapids, plunges, retracts, cutlen, cpu, worst, worst < -1e-5 ? "VIOL " : worst < 0 ? "NEG " : "", worst < 0 || verb ? wdesc.c_str() : "");
  }
  std::printf("SUMMARY fam %d cases %d worst %.4e neg %d viol %d\n", fam, ncase, worstAll, neg, viol);
}
