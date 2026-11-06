// t0169v1: independent exact clearance oracle (verifier). Lines: closed-form segment-segment.
// Arcs and Beziers: certified 1-D branch and bound over the curve parameter with a Lipschitz bound
// (arc pieces <= 90 deg in their tangent triangle; Bezier pieces in their control hull), so "every curve point >= need from the move"
// is proven, and the minimum is bracketed to `res`. No polygonisation, no core geometry calls
// except span evaluation (pointAt / startAngle arithmetic done here).
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/toolpath.hpp"
namespace oc2 {
using carvetto::geometry::Vec2;
inline double dps(Vec2 p, Vec2 a, Vec2 b) {
  double ax = b.x - a.x, ay = b.y - a.y, l2 = ax * ax + ay * ay;
  double t = l2 > 0 ? ((p.x - a.x) * ax + (p.y - a.y) * ay) / l2 : 0.0; t = std::clamp(t, 0.0, 1.0);
  double dx = p.x - (a.x + ax * t), dy = p.y - (a.y + ay * t); return std::sqrt(dx * dx + dy * dy);
}
inline double orient(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
inline double dss(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
  double o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
  if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return 0.0;
  return std::min({dps(a, c, d), dps(b, c, d), dps(c, a, b), dps(d, a, b)});
}
// lower bound: distance from segment ab to triangle pqr (0 if they meet)
inline bool inTri(Vec2 x, Vec2 p, Vec2 q, Vec2 r) { double a = orient(p, q, x), b = orient(q, r, x), c = orient(r, p, x); return (a >= 0 && b >= 0 && c >= 0) || (a <= 0 && b <= 0 && c <= 0); }
inline double dsTri(Vec2 a, Vec2 b, Vec2 p, Vec2 q, Vec2 r) {
  if (inTri(a, p, q, r) || inTri(b, p, q, r)) return 0.0;
  return std::min({dss(a, b, p, q), dss(a, b, q, r), dss(a, b, r, p)});
}
// certified min distance from segment ab to a Bezier (control points) by subdivision; the curve
// lies in its control hull = union of the 4 triangles of control-point triples.
inline double bezMin(Vec2 a, Vec2 b, Vec2 P0, Vec2 P1, Vec2 P2, Vec2 P3, double res, bool& cert) {
  const double mag = std::max({std::fabs(P0.x), std::fabs(P0.y), std::fabs(a.x), std::fabs(a.y)}); res = std::max(res, 4e-15 * mag); const double stop = 1e-12 + 1e-15 * mag;
  struct Pc { Vec2 p0, p1, p2, p3; };
  double best = std::min(dps(P0, a, b), dps(P3, a, b)); std::vector<Pc> st{{P0, P1, P2, P3}}; long guard = 0;
  while (!st.empty()) {
    Pc c = st.back(); st.pop_back();
    double lb = std::min({dsTri(a, b, c.p0, c.p1, c.p2), dsTri(a, b, c.p0, c.p1, c.p3), dsTri(a, b, c.p0, c.p2, c.p3), dsTri(a, b, c.p1, c.p2, c.p3)});
    if (lb >= best - res) continue;
    if (++guard > 2000000) { cert = false; break; }
    Vec2 q0 = (c.p0 + c.p1) * 0.5, q1 = (c.p1 + c.p2) * 0.5, q2 = (c.p2 + c.p3) * 0.5, r0 = (q0 + q1) * 0.5, r1 = (q1 + q2) * 0.5, m = (r0 + r1) * 0.5;
    best = std::min(best, dps(m, a, b));
    if (std::max({std::hypot(c.p3.x - c.p0.x, c.p3.y - c.p0.y), std::hypot(c.p1.x - c.p0.x, c.p1.y - c.p0.y), std::hypot(c.p2.x - c.p3.x, c.p2.y - c.p3.y)}) < stop) continue;
    st.push_back({c.p0, q0, r0, m}); st.push_back({m, r1, q2, c.p3});
  }
  return best;
}
// arc: pieces of sweep <= pi/2 lie in the triangle (start, tangent intersection, end)
inline double arcMin(Vec2 a, Vec2 b, Vec2 c, double R, double t0, double sw, double res, bool& cert) {
  const double mag = std::max({std::fabs(c.x), std::fabs(c.y), R, std::fabs(a.x), std::fabs(a.y)}); res = std::max(res, 8e-15 * mag); const double stop = 1e-12 + 1e-15 * mag;
  auto pt = [&](double t) { return Vec2{c.x + R * std::cos(t), c.y + R * std::sin(t)}; };
  struct Pc { double u0, u1; };
  double best = std::min(dps(pt(t0), a, b), dps(pt(t0 + sw), a, b)); std::vector<Pc> st; int k = std::max(1, (int)std::ceil(std::fabs(sw) / (3.14159265358979 / 2)));
  for (int i = 0; i < k; ++i) st.push_back({t0 + sw * i / k, t0 + sw * (i + 1) / k}); long guard = 0;
  while (!st.empty()) {
    Pc p = st.back(); st.pop_back(); double h = 0.5 * (p.u1 - p.u0), mid = 0.5 * (p.u0 + p.u1);
    Vec2 A = pt(p.u0), B = pt(p.u1); double ch = std::cos(h); Vec2 T{c.x + R / ch * std::cos(mid), c.y + R / ch * std::sin(mid)};
    // pad the triangle by rounding (relative 1e-15 of coordinates)
    double pad = 4e-16 * (std::fabs(c.x) + std::fabs(c.y) + 2 * R);
    double lb = dsTri(a, b, A, T, B) - pad;
    if (lb >= best - res) continue;
    if (++guard > 2000000) { cert = false; break; }
    best = std::min(best, dps(pt(mid), a, b));
    if (R * std::fabs(h) < stop) continue;
    st.push_back({p.u0, mid}); st.push_back({mid, p.u1});
  }
  return best;
}
inline double dist(const carvetto::geometry::Span& s, Vec2 a, Vec2 b, double need, double res, bool& cert) {
  using namespace carvetto::geometry; double d;
  if (auto* l = std::get_if<LineSpan>(&s)) d = dss(a, b, l->start, l->end);
  else if (auto* ar = std::get_if<ArcSpan>(&s)) d = arcMin(a, b, ar->centre, ar->radius, ar->startAngle, ar->sweepAngle(), res, cert);
  else { auto& z = std::get<BezierSpan>(s); d = bezMin(a, b, z.p0, z.p1, z.p2, z.p3, res, cert); }
  if (d < need) cert = false;
  return d;
}
struct Box { double x0, y0, x1, y1; };
inline Box spanBox(const carvetto::geometry::Span& s) {
  using namespace carvetto::geometry; Box b{1e300, 1e300, -1e300, -1e300};
  auto ex = [&](Vec2 p) { b.x0 = std::min(b.x0, p.x); b.y0 = std::min(b.y0, p.y); b.x1 = std::max(b.x1, p.x); b.y1 = std::max(b.y1, p.y); };
  if (auto* l = std::get_if<LineSpan>(&s)) { ex(l->start); ex(l->end); }
  else if (auto* a = std::get_if<ArcSpan>(&s)) { ex(a->centre - Vec2{a->radius, a->radius}); ex(a->centre + Vec2{a->radius, a->radius}); }
  else { auto& z = std::get<BezierSpan>(s); ex(z.p0); ex(z.p1); ex(z.p2); ex(z.p3); }
  return b;
}
struct Rep { double minc = 1e300; size_t worst = 0; bool cert = true; size_t checked = 0; int worstKind = -1; Vec2 wa, wb; };
// every non-rapid, non-dwell move with any part below `surface`: XY segment vs every span; need = radius + extra
inline Rep check(const carvetto::toolpath::Toolpath& tp, const std::vector<carvetto::geometry::Contour>& walls, double radius, double extra, double surface, double window) {
  using namespace carvetto::toolpath; Rep r; std::vector<std::pair<const carvetto::geometry::Span*, Box>> it;
  for (auto& c : walls) for (auto& s : c.spans) it.push_back({&s, spanBox(s)});
  double need = radius + extra;
  for (size_t i = 0; i < tp.moves.size(); ++i) {
    const Move& m = tp.moves[i]; if (m.kind == MoveKind::Rapid || m.kind == MoveKind::Dwell) continue;
    auto s3 = tp.startOf(i); if (std::min(s3.z, m.to.z) >= surface - 1e-9) continue;
    ++r.checked; Vec2 a = s3.xy(), b = m.to.xy();
    double w = need + window;
    double bx0 = std::min(a.x, b.x) - w, bx1 = std::max(a.x, b.x) + w, by0 = std::min(a.y, b.y) - w, by1 = std::max(a.y, b.y) + w;
    for (auto& [sp, bo] : it) {
      if (bo.x0 > bx1 || bo.x1 < bx0 || bo.y0 > by1 || bo.y1 < by0) continue;
      bool c = true; double d = dist(*sp, a, b, need, 1e-10, c);
      if (!c) r.cert = false;
      if (d - radius < r.minc) { r.minc = d - radius; r.worst = i; r.worstKind = (int)m.kind; r.wa = a; r.wb = b; }
    }
  }
  return r;
}
}  // namespace oc2
