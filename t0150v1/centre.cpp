// T-0150 verifier: drillCentre oracle + AC1 behaviour checks.
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <optional>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/cut_order.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static int fails = 0;
#define CHK(c, ...) do { if (!(c)) { ++fails; std::printf("FAIL %s:%d: ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)
using LD = long double;
struct Acc { LD a=0, mx=0, my=0; };
static void seg(Acc& m, LD ax, LD ay, LD bx, LD by) { LD c = ax*by - ay*bx; m.a += c/2; m.mx += c*(ax+bx)/6; m.my += c*(ay+by)/6; }
static void polyMoments(const Contour& c, int N, Vec2 ref, Acc& m) {
  for (auto& s : c.spans) {
    int n = std::holds_alternative<LineSpan>(s) ? 1 : N;
    Vec2 prev = startPoint(s) - ref;
    for (int i = 1; i <= n; ++i) { Vec2 p = (i == n ? endPoint(s) : pointAt(s, double(i)/n)) - ref; seg(m, prev.x, prev.y, p.x, p.y); prev = p; }
  }
  Vec2 a = c.endPoint() - ref, b = c.startPoint() - ref; seg(m, a.x, a.y, b.x, b.y);
}
static std::optional<Vec2> oracle(const Contour& c) {
  Vec2 ref = c.bounds().centre();
  Acc m1, m2; polyMoments(c, 4000, ref, m1); polyMoments(c, 8000, ref, m2);
  LD a = m2.a + (m2.a - m1.a)/3, mx = m2.mx + (m2.mx - m1.mx)/3, my = m2.my + (m2.my - m1.my)/3;
  if (std::fabs((double)a) < 1e-12) return std::nullopt;
  return Vec2{ref.x + double(mx/a), ref.y + double(my/a)};
}
static Vec2 rot(Vec2 p, double t, Vec2 o) { Vec2 d = p - o; return o + Vec2{d.x*std::cos(t) - d.y*std::sin(t), d.x*std::sin(t) + d.y*std::cos(t)}; }
static Span xform(const Span& s, double t, Vec2 o, Vec2 off) {
  if (auto* l = std::get_if<LineSpan>(&s)) return LineSpan{rot(l->start,t,o)+off, rot(l->end,t,o)+off};
  if (auto* a = std::get_if<ArcSpan>(&s)) { ArcSpan r = *a; r.centre = rot(a->centre,t,o)+off; r.startAngle += t; r.endAngle += t; return r; }
  auto b = std::get<BezierSpan>(s); return BezierSpan{rot(b.p0,t,o)+off, rot(b.p1,t,o)+off, rot(b.p2,t,o)+off, rot(b.p3,t,o)+off};
}
static Contour xform(const Contour& c, double t, Vec2 o, Vec2 off) { Contour r; r.closed = c.closed; for (auto& s : c.spans) r.spans.push_back(xform(s,t,o,off)); return r; }
static Contour randomShape(std::mt19937& rng, double size) {
  std::uniform_real_distribution<double> U(0,1);
  int n = 3 + int(U(rng)*10); std::vector<double> ang; for (int i=0;i<n;++i) ang.push_back(U(rng)*2*kPi); std::sort(ang.begin(), ang.end());
  std::vector<Vec2> pts; for (double a : ang) { double r = size*(0.4+0.6*U(rng)); pts.push_back({r*std::cos(a), r*std::sin(a)}); }
  Contour c; c.closed = true;
  for (int i=0;i<n;++i) { Vec2 p = pts[i], q = pts[(i+1)%n]; double k = U(rng);
    if (k < 0.4 || distance(p,q) < 1e-6) c.append(LineSpan{p,q});
    else if (k < 0.7) { Vec2 m = (p+q)*0.5, d = q-p; Vec2 nrm{-d.y, d.x}; double h = (U(rng)-0.5)*1.5; Vec2 ctr = m + nrm*h;
      c.append(ArcSpan::fromCentre(ctr, p, q, U(rng) < 0.5 ? ArcDirection::CCW : ArcDirection::CW)); }
    else { Vec2 d = q-p; c.append(BezierSpan{p, p + d*0.33 + Vec2{(U(rng)-0.5)*size, (U(rng)-0.5)*size}*0.5, p + d*0.66 + Vec2{(U(rng)-0.5)*size,(U(rng)-0.5)*size}*0.5, q}); }
  }
  if (U(rng) < 0.5) { Contour r; r.closed = true; for (auto it = c.spans.rbegin(); it != c.spans.rend(); ++it) r.spans.push_back(reversed(*it)); return r; }
  return c;
}
static Contour circleBeziers(Vec2 c, double r, double a0) {
  const double k = 0.5522847498307936; Contour out; out.closed = true;
  for (int i=0;i<4;++i) { double t0 = a0 + i*kPi/2, t1 = t0 + kPi/2;
    Vec2 p0 = c + Vec2{r*std::cos(t0), r*std::sin(t0)}, p3 = c + Vec2{r*std::cos(t1), r*std::sin(t1)};
    Vec2 d0{-std::sin(t0), std::cos(t0)}, d1{-std::sin(t1), std::cos(t1)};
    out.append(BezierSpan{p0, p0 + d0*(k*r), p3 - d1*(k*r), p3}); }
  return out;
}
static Contour ngon(Vec2 c, double r, int n, double a0) { Polyline pl; pl.closed = true; for (int i=0;i<n;++i) pl.points.push_back(c + Vec2{r*std::cos(a0+2*kPi*i/n), r*std::sin(a0+2*kPi*i/n)}); return Contour::fromPolyline(pl); }
static Contour arcsCircle(Vec2 c, double r, int n, double a0, ArcDirection dir) { Contour out; out.closed = true;
  for (int i=0;i<n;++i) { double s = dir==ArcDirection::CCW ? 1 : -1; Vec2 p = c + Vec2{r*std::cos(a0+s*2*kPi*i/n), r*std::sin(a0+s*2*kPi*i/n)}, q = c + Vec2{r*std::cos(a0+s*2*kPi*(i+1)/n), r*std::sin(a0+s*2*kPi*(i+1)/n)};
    out.append(ArcSpan::fromCentre(c, p, q, dir)); } return out; }
static tooling::Tool drill(double d) { tooling::Tool t; t.id="d"; t.name="D"; t.type=tooling::ToolType::Drill; t.diameter=d; t.includedAngle=118*kPi/180;
  t.defaultCuttingData.toolNumber=1; t.defaultCuttingData.feedRate=600; t.defaultCuttingData.plungeRate=200; t.defaultCuttingData.spindleSpeed=10000; t.defaultCuttingData.passDepth=2; return t; }
int main() {
  std::mt19937 rng(12345); std::uniform_real_distribution<double> U(0,1);
  Tolerance tol = Tolerance::defaults();
  // 1. random shapes vs oracle, rotation & translation covariance
  double worst = 0, worstRot = 0;
  for (int it = 0; it < 3000; ++it) {
    double size = std::pow(10.0, U(rng)*4 - 1); Contour c = randomShape(rng, size);
    auto o = oracle(c); auto d = drillCentre(c, tol);
    if (!o) continue;
    if (!d) { CHK(false, "no centre where oracle has one (it %d) size %g area %g spans %zu", it, size, signedArea(c), c.spans.size()); continue; }
    double e = distance(*o, *d) / size; worst = std::max(worst, e);
    CHK(e < 1e-8, "it %d size %g err %g", it, size, e);
    double t = U(rng)*2*kPi; Vec2 off{(U(rng)-0.5)*2e6, (U(rng)-0.5)*2e6}; Vec2 org{U(rng)*10, U(rng)*10};
    Contour c2 = xform(c, t, org, off); auto d2 = drillCentre(c2, tol);
    if (!d2) { CHK(false, "rotated: no centre (it %d)", it); continue; }
    Vec2 want = rot(*d, t, org) + off; double e2 = distance(want, *d2) / size; worstRot = std::max(worstRot, e2 * size / (1 + std::hypot(off.x, off.y)));
    CHK(distance(want, *d2) < 1e-9 * (size + 2e6), "it %d rotated err %g", it, distance(want,*d2));
  }
  std::printf("random shapes: worst rel err %.3g, worst rotated/far err (rel to coord) %.3g\n", worst, worstRot);
  // 2. circles many ways, exactness
  double worstCirc = 0; int notExact = 0, total = 0;
  for (int it = 0; it < 2000; ++it) {
    double mag = std::pow(10.0, U(rng)*7 - 1);
    Vec2 c{std::round((U(rng)-0.5)*2*mag*1000)/1000, std::round((U(rng)-0.5)*2*mag*1000)/1000};
    double r = std::pow(10.0, U(rng)*4 - 2); double a0 = U(rng)*2*kPi;
    std::vector<Contour> forms{Contour::circle(c, r), Contour::circle(c, r, ArcDirection::CW), arcsCircle(c, r, 2, a0, ArcDirection::CCW), arcsCircle(c, r, 3, a0, ArcDirection::CW),
                               arcsCircle(c, r, 4, a0, ArcDirection::CCW), circleBeziers(c, r, a0), circleBeziers(c, r, 0), ngon(c, r, 6, a0), ngon(c, r, 7, 0), ngon(c, r, 64, a0), ngon(c, r, 360, a0)};
    for (std::size_t f = 0; f < forms.size(); ++f) {
      auto d = drillCentre(forms[f], tol); ++total;
      if (!d) { if (r > 1e-2) CHK(false, "circle form %zu r %g no centre", f, r); continue; }
      double e = distance(*d, c); worstCirc = std::max(worstCirc, e / (r + 1e-300));
      if (!(d->x == c.x && d->y == c.y)) ++notExact;
      char b1[64], b2[64], b3[64], b4[64]; std::snprintf(b1, 64, "%.3f", d->x); std::snprintf(b2, 64, "%.3f", c.x); std::snprintf(b3, 64, "%.3f", d->y); std::snprintf(b4, 64, "%.3f", c.y);
      CHK(std::string(b1) == b2 && std::string(b3) == b4, "circle form %zu c (%.17g,%.17g) r %g printed %s %s", f, c.x, c.y, r, b1, b3);
      CHK(e <= 1e-9 * (std::fabs(c.x) + std::fabs(c.y) + r), "circle form %zu c (%.17g,%.17g) r %g err %g", f, c.x, c.y, r, e);
    }
  }
  std::printf("circles: worst err/r %.3g, not bit-exact %d of %d\n", worstCirc, notExact, total);
  // 3. C / U shape, ring self-overlapping
  { Polyline pl; pl.closed = true; pl.points = {{0,0},{10,0},{10,2},{2,2},{2,8},{10,8},{10,10},{0,10}}; Contour C = Contour::fromPolyline(pl);
    auto r = drillToolpath({C, Contour::circle({50,50}, 3)}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    CHK(r.has(ToolpathIssue::CentreOutsideVector) && r.holes.size() == 1 && r.holes[0].contour == 1, "C shape");
    for (auto& d : r.diagnostics) std::printf("  C diag %d contour %zu: %s\n", int(d.issue), d.contour.value_or(999), d.message.c_str());
    Contour C2 = xform(C, 1.234, {0,0}, {1e6, -3e5}); auto r2 = drillToolpath({C2}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    CHK(r2.has(ToolpathIssue::CentreOutsideVector) && !r2.ok(), "C shape rotated far");
    // ring as one loop: outer CCW circle, bridge, inner CW circle, bridge back
    Contour ring; ring.closed = true; ring.append(ArcSpan{{0,0}, 10, 0, 2*kPi, ArcDirection::CCW}); ring.append(LineSpan{{10,0},{5,0}}); ring.append(ArcSpan{{0,0}, 5, 0, 2*kPi, ArcDirection::CW}); ring.append(LineSpan{{5,0},{10,0}});
    auto r3 = drillToolpath({ring}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    for (auto& d : r3.diagnostics) std::printf("  ring diag %d: %s\n", int(d.issue), d.message.c_str());
    CHK(!r3.ok() && r3.has(ToolpathIssue::CentreOutsideVector), "ring loop drilled? ok=%d", r3.ok());
    // U shape with centroid just inside the bar? A thick U: centroid inside
    Polyline pu; pu.closed = true; pu.points = {{0,0},{10,0},{10,10},{7,10},{7,3},{3,3},{3,10},{0,10}}; auto r4 = drillToolpath({Contour::fromPolyline(pu)}, drill(1), std::nullopt, DrillParams{.cutDepth = 1});
    std::printf("  thick U: ok=%d holes=%zu centre=(%g,%g)\n", r4.ok(), r4.holes.size(), r4.holes.empty()?0:r4.holes[0].centre.x, r4.holes.empty()?0:r4.holes[0].centre.y);
  }
  // 4. open/empty/zero-area/duplicate/discontinuous
  { Contour open = Contour::circle({0,0},3); open.closed = false; Contour open2 = open; Contour empty; empty.closed = true; Contour empty2;
    Polyline coll; coll.closed = true; coll.points = {{0,0},{5,0},{10,0}}; Contour zero = Contour::fromPolyline(coll);
    Contour gap; gap.closed = true; gap.append(LineSpan{{0,0},{10,0}}); gap.append(LineSpan{{10,0},{10,10}}); gap.append(LineSpan{{9,10},{0,10}}); gap.append(LineSpan{{0,10},{0,0}});
    Contour a = Contour::circle({20,20}, 3), b = Contour::circle({20,20}, 6), c3 = ngon({20,20}, 4, 6, 0.3), dup = Contour::circle({20,20}, 3);
    Contour near = Contour::circle({20 + 0.5*tol.distance, 20}, 2); Contour far2 = Contour::circle({20 + 3*tol.distance, 20}, 2);
    std::vector<Contour> in{open, empty, a, zero, gap, b, open2, c3, dup, near, far2, empty2};
    auto r = drillToolpath(in, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    for (auto& d : r.diagnostics) std::printf("  diag %d contour %zu: %s\n", int(d.issue), d.contour.value_or(999), d.message.c_str());
    std::printf("  holes:"); for (auto& h : r.holes) std::printf(" %zu(%.9g,%.9g)", h.contour, h.centre.x, h.centre.y); std::printf("\n");
    int opens = 0, empties = 0, dups = 0, zeros = 0, gaps = 0; for (auto& d : r.diagnostics) { opens += d.issue == ToolpathIssue::OpenContourIgnored; empties += d.issue == ToolpathIssue::EmptyContour; dups += d.issue == ToolpathIssue::DuplicateHole; zeros += d.issue == ToolpathIssue::NoRegion; gaps += d.issue == ToolpathIssue::DiscontinuousContour; }
    CHK(opens == 1 && empties == 2 && zeros == 1 && gaps == 1 && dups == 4 && r.holes.size() == 2, "diag counts o%d e%d z%d g%d d%d holes %zu", opens, empties, zeros, gaps, dups, r.holes.size());
    auto r0 = drillToolpath({}, drill(3), std::nullopt, DrillParams{.cutDepth = 3}); CHK(!r0.ok() && !r0.diagnostics.empty(), "no input gives a diagnostic");
    auto r1 = drillToolpath({open}, drill(3), std::nullopt, DrillParams{.cutDepth = 3}); CHK(!r1.ok() && r1.diagnostics.size() == 1 && r1.has(ToolpathIssue::OpenContourIgnored), "only open");
    // tiny circle (area just above / below tol^2)
    double rr = tol.distance / std::sqrt(kPi) * 0.9; auto rt = drillToolpath({Contour::circle({1,1}, rr)}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    std::printf("  tiny circle r=%g ok=%d diag=%d\n", rr, rt.ok(), rt.diagnostics.empty() ? -1 : int(rt.diagnostics[0].issue));
    double rr2 = tol.distance * 2; auto rt2 = drillToolpath({Contour::circle({1,1}, rr2)}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    std::printf("  small circle r=%g ok=%d holes=%zu diag=%d\n", rr2, rt2.ok(), rt2.holes.size(), rt2.diagnostics.empty() ? -1 : int(rt2.diagnostics[0].issue));
    // hole wider: narrow slot 2mm wide with 3mm drill
    Polyline sl; sl.closed = true; sl.points = {{0,0},{40,0},{40,2},{0,2}}; auto rs = drillToolpath({Contour::fromPolyline(sl)}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    CHK(rs.ok() && rs.has(ToolpathIssue::HoleWiderThanVector), "narrow slot");
    Polyline sl2 = sl; sl2.points = {{0,0},{40,0},{40,3.0},{0,3.0}}; auto rs2 = drillToolpath({Contour::fromPolyline(sl2)}, drill(3), std::nullopt, DrillParams{.cutDepth = 3});
    CHK(rs2.ok() && !rs2.has(ToolpathIssue::HoleWiderThanVector), "slot exactly tool width flagged");
  }
  // 5. tool types
  for (auto type : tooling::allToolTypes()) { tooling::Tool t = drill(3); t.type = type; t.includedAngle = (type == tooling::ToolType::Drill || type == tooling::ToolType::VBit) ? 1.5 : 0; t.cornerRadius = 0.5; t.tipDiameter = 0.5;
    auto r = drillToolpath({Contour::circle({0,0},3)}, t, std::nullopt, DrillParams{.cutDepth = 3});
    bool allowed = type == tooling::ToolType::Drill || type == tooling::ToolType::EndMill;
    CHK(allowed ? r.ok() : (r.has(ToolpathIssue::InvalidTool) && !r.ok()), "tool type %d ok=%d %s", int(type), r.ok(), r.diagnostics.empty() ? "" : r.diagnostics[0].message.c_str()); }
  // 6. ordering equals orderCuts
  for (int it = 0; it < 200; ++it) {
    std::vector<Contour> cs; int n = 1 + int(U(rng)*40); for (int i=0;i<n;++i) cs.push_back(Contour::circle({std::round(U(rng)*100), std::round(U(rng)*100)}, 1 + U(rng)*4));
    DrillParams p; p.cutDepth = 2; int k = int(U(rng)*5); p.order.selectionOrder = k==1; p.order.leftToRight = k==2; p.order.bottomToTop = k==3; p.order.grid = k==4; p.startPosition = {U(rng)*100, U(rng)*100};
    auto r = drillToolpath(cs, drill(1), std::nullopt, p);
    // expected: dedupe first, then orderCuts
    std::vector<OrderItem> items; std::vector<std::size_t> idx;
    for (std::size_t i=0;i<cs.size();++i) { Vec2 c = *drillCentre(cs[i], tol); bool dup=false; for (auto j : idx) if (distance(*drillCentre(cs[j], tol), c) <= tol.distance) dup = true; if (dup) continue;
      OrderItem o; o.path = cs[i]; o.path.closed = false; o.selectionIndex = i; o.start = o.end = c; items.push_back(o); idx.push_back(i); }
    auto ord = orderCuts(items, p.order, p.startPosition, tol);
    bool same = ord.size() == r.holes.size(); for (std::size_t i=0; same && i<ord.size(); ++i) same = idx[ord[i]] == r.holes[i].contour;
    CHK(same, "order mismatch it %d k %d", it, k);
    if (k == 1) { bool inc = true; for (std::size_t i=1;i<r.holes.size();++i) inc &= r.holes[i].contour > r.holes[i-1].contour; CHK(inc, "selection order not increasing"); }
    if (k == 0 && !r.holes.empty()) { // nearest: first hole is nearest to start
      double best = 1e300; for (auto& h : r.holes) best = std::min(best, distance(h.centre, p.startPosition)); CHK(distance(r.holes[0].centre, p.startPosition) <= best + 1e-9, "nearest first"); }
  }
  std::printf("fails=%d\n", fails); return fails ? 1 : 0;
}
