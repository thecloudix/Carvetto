// t0169v1 wb: white-box fuzz of the branch's segmentInside / RegionIndex::segmentInside / EdgeGrid::anyCloser
// (pocket.cpp included into this TU). usage: wb <seed> <n> ; env WB_FAR offset, WB_MODE (0 all).
// For each random ring set + segment: exact max distance outside the region along the segment (crossing split +
// dense sampling + golden refinement); report the worst for strict-accepted (kMoveSlack) and lenient-accepted.
#include "toolpath/pocket.cpp"
#include <cstdio>
#include <cstdlib>
using namespace carvetto; using namespace carvetto::toolpath;
struct SM { std::uint64_t s; std::uint64_t next() { std::uint64_t z = (s += 0x9E3779B97F4A7C15ULL); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL; return z ^ (z >> 31); }
  double U(double a, double b) { return a + (b - a) * double(next() >> 11) * 0x1.0p-53; } int I(int a, int b) { return a + int(next() % std::uint64_t(b - a + 1)); } };
static double dps(Vec2 p, Vec2 a, Vec2 b) { Vec2 ab = b - a; double l2 = ab.lengthSquared(); double t = l2 > 0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0; return geometry::distance(p, a + ab * t); }
static double orient2(Vec2 a, Vec2 b, Vec2 c) { return (b - a).cross(c - a); }
// exact-ish segment distance: 0 only for a proper crossing (strict signs); else min of the 4 end distances
static double exactSS(Vec2 a, Vec2 b, Vec2 c, Vec2 d) { double o1 = orient2(a, b, c), o2 = orient2(a, b, d), o3 = orient2(c, d, a), o4 = orient2(c, d, b);
  if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return 0.0; return std::min({dps(a, c, d), dps(b, c, d), dps(c, a, b), dps(d, a, b)}); }
static bool parity(const std::vector<Ring>& rs, Vec2 p) { bool in = false; for (auto& r : rs) for (size_t i = 0, n = r.size(); i < n; ++i) { Vec2 a = r[i], b = r[(i + 1) % n]; if ((a.y > p.y) != (b.y > p.y)) { double x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x); if (x > p.x) in = !in; } } return in; }
static double bdist(const std::vector<Ring>& rs, Vec2 p) { double d = 1e300; for (auto& r : rs) for (size_t i = 0, n = r.size(); i < n; ++i) d = std::min(d, dps(p, r[i], r[(i + 1) % n])); return d; }
static double outDist(const std::vector<Ring>& rs, Vec2 p) { double d = bdist(rs, p); if (d < 1e-12) return 0; return parity(rs, p) ? 0.0 : d; }
static double maxOut(const std::vector<Ring>& rs, Vec2 a, Vec2 b) {
  const int N = 400; double best = 0, bt = 0; std::vector<double> v(N + 1);
  for (int i = 0; i <= N; ++i) { v[i] = outDist(rs, a + (b - a) * (double(i) / N)); if (v[i] > best) { best = v[i]; bt = double(i) / N; } }
  // refine around the best samples (top 5 local maxima)
  for (int i = 0; i <= N; ++i) { bool lm = v[i] > 0 && (i == 0 || v[i] >= v[i - 1]) && (i == N || v[i] >= v[i + 1]); if (!lm) continue;
    double lo = std::max(0.0, (i - 1.0) / N), hi = std::min(1.0, (i + 1.0) / N);
    for (int it = 0; it < 80; ++it) { double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3; if (outDist(rs, a + (b - a) * m1) < outDist(rs, a + (b - a) * m2)) lo = m1; else hi = m2; }
    double x = outDist(rs, a + (b - a) * (0.5 * (lo + hi))); if (x > best) { best = x; bt = 0.5 * (lo + hi); } }
  // vertex projections
  for (auto& r : rs) for (auto& q : r) { Vec2 ab = b - a; double l2 = ab.lengthSquared(); if (l2 <= 0) continue; double t = std::clamp((q - a).dot(ab) / l2, 0.0, 1.0); double x = outDist(rs, a + ab * t); if (x > best) { best = x; bt = t; } }
  (void)bt; return best;
}
// a region like a Pocket centre region: star polygon whose reflex vertices are replaced by fine "round join" arcs
static std::vector<Ring> mkRings(SM& r, double scale, Vec2 off) {
  std::vector<Ring> rs; int n = r.I(3, 9); double inner = r.U(0.3, 0.9), ph = r.U(0, 6.283); Ring out;
  double jr = r.U(0.01, 0.5) * scale; double sag = std::pow(10.0, r.U(-6, -2)) * scale; int mode = r.I(0, 2);
  for (int i = 0; i < 2 * n; ++i) { double a = ph + 6.283185307179586 * i / (2 * n); double rr = (i % 2 ? inner : 1.0) * scale;
    Vec2 V = off + Vec2{rr * std::cos(a), rr * std::sin(a)};
    if (i % 2 == 1 && mode > 0) { // reflex vertex: round join arc of radius jr around a centre outside (centre at V + outward)
      Vec2 dir = Vec2{std::cos(a), std::sin(a)}; Vec2 c = V + dir * jr * 1.0; double half = r.U(0.3, 1.3);
      double step = 2 * std::acos(std::max(-1.0, 1 - sag / jr)); int k = std::clamp(int(std::ceil(2 * half / std::max(step, 1e-6))), 1, 300);
      for (int j = 0; j <= k; ++j) { double t = a + 3.14159265358979 - half + 2 * half * j / k; // arc on the inner side of c, runs clockwise around c seen from region
        out.push_back(c + Vec2{jr * std::cos(t), jr * std::sin(t)}); }
      if (mode == 2) { std::reverse(out.end() - (k + 1), out.end()); }
    } else out.push_back(V);
  }
  rs.push_back(out);
  if (r.U(0, 1) < 0.4) { Ring h; double hr = r.U(0.05, 0.25) * scale * inner; int m = r.I(3, 40); for (int j = 0; j < m; ++j) h.push_back(off + Vec2{hr * std::cos(6.283185307179586 * j / m), hr * std::sin(6.283185307179586 * j / m)}); rs.push_back(h); }
  // snap to 1e-6 grid like Clipper2 output
  for (auto& q : rs) for (auto& p : q) { p.x = std::round(p.x * 1e6) / 1e6; p.y = std::round(p.y * 1e6) / 1e6; }
  return rs;
}
static Vec2 pickPoint(SM& r, const std::vector<Ring>& rs, double dist, size_t& ri, size_t& ei, int pref) {
  if (pref < 0) { ri = r.next() % rs.size(); ei = r.next() % rs[ri].size(); } else { ei = (ei + size_t(pref)) % rs[ri].size(); }
  const Ring& q = rs[ri]; Vec2 a = q[ei], b = q[(ei + 1) % q.size()]; double t = r.U(0, 1); if (r.U(0, 1) < 0.2) t = r.U(0, 1) < 0.5 ? 0 : 1;
  Vec2 p = a + (b - a) * t; Vec2 n = (b - a).lengthSquared() > 0 ? (b - a).normalized().perp() : Vec2{0, 0};
  const double offs[] = {0, 0, 1e-7, 1e-6, 2e-6, 3.9e-6, 4.1e-6, 6e-6, 1e-5, 0.5, 0.99, 1.0, 1.5, 2.5};
  double o = offs[r.next() % 14]; if (o >= 0.5 && o <= 2.5) o *= dist; o *= (r.next() % 2 ? 1 : -1); return p + n * o;
}
int main(int argc, char** argv) {
  std::uint64_t seed = strtoull(argv[1], 0, 10); long n = atol(argv[2]); double far = getenv("WB_FAR") ? atof(getenv("WB_FAR")) : 0.0;
  SM r{seed}; double worstS = 0, worstL = 0, worstRS = 0; long accS = 0, accL = 0, mism = 0, gridBad = 0, accSfree = 0; char wS[400] = "", wL[400] = "", wR[400] = "", wX[400] = ""; double worstR = 0, worstX = 0;
  const double dists[] = {1e-6, 3e-6, 1e-5, 1e-4, 3.3333e-4, 1e-3, 0.01, 0.05};
  for (long k = 0; k < n; ++k) {
    double scale = std::pow(10.0, r.U(-1, 2)); Vec2 off{far * r.U(0.5, 1), -far * r.U(0.5, 1)}; auto rs = mkRings(r, scale, off);
    double dist = dists[r.next() % 8]; MoveTolerance strict{std::min(dist, kMoveSlack), dist}, len{dist, dist};
    RegionIndex idx(rs);
    for (int s = 0; s < 20; ++s) {
      size_t ri, ei; Vec2 a = pickPoint(r, rs, dist, ri, ei, -1); int pref = r.I(0, 3); Vec2 b = pref == 0 ? pickPoint(r, rs, dist, ri, ei, -1) : pickPoint(r, rs, dist, ri, ei, r.I(1, 4));
      if (r.U(0, 1) < 0.1) b = a + (b - a) * r.U(0.1, 3.0);
      bool fS = segmentInside(a, b, rs, strict), fL = segmentInside(a, b, rs, len);
      auto v = idx.segmentInside(a, b, strict, len);
      if (v.strict != fS || v.lenient != fL) { ++mism; if (mism < 10) std::printf("MISMATCH idx S%d L%d free S%d L%d dist %g a (%.9f,%.9f) b (%.9f,%.9f)\n", v.strict, v.lenient, fS, fL, dist, a.x, a.y, b.x, b.y); }
      if (fS && !fL) std::printf("STRICT_NOT_LENIENT\n");
      if (!fS && !fL) continue;
      double mo = maxOut(rs, a, b); double ed = std::max(outDist(rs, a), outDist(rs, b));
      if (fS && ed <= 1e-6 && mo > worstR) { worstR = mo; std::snprintf(wR, sizeof wR, "dist %g scale %g a (%.9f,%.9f) b (%.9f,%.9f) len %.3g", dist, scale, a.x, a.y, b.x, b.y, geometry::distance(a, b)); }
      if (fS && mo - ed > worstX) { worstX = mo - ed; std::snprintf(wX, sizeof wX, "dist %g scale %g a (%.9f,%.9f) b (%.9f,%.9f) len %.3g ends %.3g", dist, scale, a.x, a.y, b.x, b.y, geometry::distance(a, b), ed); }
      if (fS) { ++accS; if (mo > worstS) { worstS = mo; std::snprintf(wS, sizeof wS, "dist %g scale %g a (%.9f,%.9f) b (%.9f,%.9f) len %.3g", dist, scale, a.x, a.y, b.x, b.y, geometry::distance(a, b)); } worstRS = std::max(worstRS, mo / std::min(dist, kMoveSlack)); }
      else { ++accL; if (mo > worstL) { worstL = mo; std::snprintf(wL, sizeof wL, "dist %g a (%.9f,%.9f) b (%.9f,%.9f)", dist, a.x, a.y, b.x, b.y); } }
      (void)accSfree;
    }
    // EdgeGrid::anyCloser vs brute force on random long segments
    const double cellArg = r.U(0.001, 0.5) * scale; EdgeGrid g(rs, cellArg);
    for (int s = 0; s < 10; ++s) { size_t ri, ei; Vec2 a = pickPoint(r, rs, dist, ri, ei, -1), b = pickPoint(r, rs, dist, ri, ei, -1); if (r.U(0, 1) < 0.3) b = a + (b - a) * r.U(-2, 4);
      double d = r.U(0, 1) < 0.5 ? r.U(1e-6, 0.3) * scale : std::pow(10.0, r.U(-6, 0)) * scale; double bf = 1e300;
      for (auto& q : rs) for (size_t i = 0, m = q.size(); i < m; ++i) bf = std::min(bf, exactSS(a, b, q[i], q[(i + 1) % m]));
      bool got = g.anyCloser(a, b, d); if (got != (bf < d)) { ++gridBad; if (getenv("WB_DUMP")) { std::printf("RINGS cellArg %.17g\n", cellArg); for (auto& q : rs) { std::printf("ring"); for (auto& p : q) std::printf(" %.17g %.17g", p.x, p.y); std::printf("\n"); } }
        if (gridBad < 10) std::printf("GRID got %d bf %.17g d %.17g a (%.17g,%.17g) b (%.17g,%.17g)\n", got, bf, d, a.x, a.y, b.x, b.y); } }
  }
  std::printf("STRICT ends<=1e-6 outside: worst %.4e [%s]\nSTRICT worst beyond the ends' own depth: %.4e [%s]\n", worstR, wR, worstX, wX);
  std::printf("SUMMARY seed %llu n %ld far %g accS %ld accL %ld mism %ld gridBad %ld worstStrict %.4e (x slack %.3f) [%s] worstLenientOnly %.4e [%s]\n", (unsigned long long)seed, n, far, accS, accL, mism, gridBad, worstS, worstRS, wS, worstL, wL);
}
