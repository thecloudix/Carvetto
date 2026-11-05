// T-0142 verifier (a): booleanOp batching exactness. For random scenes of many far-apart groups
// (nested outers/holes/islands, overlapping clips, open subjects, groups whose boxes touch at the
// batch boundary, subjects/clips crossing several groups) compare
//   B = detail::booleanOpBatched(..., minDepth=0)   (batch whenever it splits)
//   D = booleanOp(...)                               (default threshold)
//   O = detail::booleanOpBatched(..., kNever)        (one Clipper2 call = main's code path)
// and an independent winding oracle from the input. usage: bat seed N
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>
#include "boolean_batch.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
using namespace carvetto::geometry;
using Ring = std::vector<Vec2>;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static double q3(double v) { return std::round(v * 1000.0) / 1000.0; }  // exact-ish mm grid
static Contour poly(const Ring& p, bool closed = true) { Contour c; c.closed = closed;
  size_t n = closed ? p.size() : p.size() - 1;
  for (size_t i = 0; i < n; ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Ring ringOf(const Contour& c) { Ring r; for (auto& s : c.spans) r.push_back(startPoint(s)); if (!c.closed && !c.spans.empty()) r.push_back(endPoint(c.spans.back())); return r; }
static int wind(const Ring& r, Vec2 p) { int w = 0; for (size_t i = 0, n = r.size(); i < n; ++i) { Vec2 a = r[i], b = r[(i + 1) % n];
    double c = (b - a).cross(p - a); if (a.y <= p.y) { if (b.y > p.y && c > 0) ++w; } else if (b.y <= p.y && c < 0) --w; } return w; }
static double segD(Vec2 p, Vec2 a, Vec2 b) { Vec2 d = b - a; double l = d.dot(d); double t = l > 0 ? std::clamp((p - a).dot(d) / l, 0.0, 1.0) : 0; return distance(p, a + d * t); }
static double edgeD(const std::vector<Ring>& rs, Vec2 p, bool closed = true) { double b = 1e300; for (auto& r : rs) { size_t n = r.size(); size_t m = closed ? n : n - 1;
    for (size_t i = 0; i < m; ++i) b = std::min(b, segD(p, r[i], r[(i + 1) % n])); } return b; }
static Ring star(Vec2 c, double R, int n, bool ccw) { Ring r; double ph = U(0, 6.28);
  for (int k = 0; k < n; ++k) { double a = ph + 6.283185307179586 * k / n, rr = R * U(0.45, 1.0); r.push_back({q3(c.x + rr * std::cos(a)), q3(c.y + rr * std::sin(a))}); }
  if (!ccw) std::reverse(r.begin(), r.end()); return r; }
static Ring rectR(double x0, double y0, double x1, double y1, bool ccw) { Ring r{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; if (!ccw) std::reverse(r.begin(), r.end()); return r; }

int main(int argc, char** argv) {
  unsigned long seed = strtoul(argv[1], 0, 10); int N = atoi(argv[2]);
  long bad = 0, scenes = 0, batchedScenes = 0, samples = 0, opensamples = 0; double worstX = 0;
  for (int sc = 0; sc < N; ++sc) {
    rng.seed(seed * 1000003 + sc);
    std::vector<Ring> subj, clip, opens; std::vector<Contour> S, C;
    int nx = I(20, 60), ny = I(1, 4); double pitch = 4.0;
    int mode = I(0, 5);  // 0 apart, 1 touching boxes (rect groups sharing x edges), 2 corner touch, 3 crossing subject, 4 mixed, 5 tight gaps of 1 grid unit
    for (int i = 0; i < nx; ++i) for (int j = 0; j < ny; ++j) {
      double cx = pitch * i, cy = pitch * j * 1.1;
      if (mode == 1 || (mode == 4 && I(0, 1))) {  // a rectangle group exactly filling its pitch cell: neighbour boxes touch (shared edge)
        bool ccw = I(0, 3) != 0;
        subj.push_back(rectR(cx, cy, cx + pitch, cy + pitch * 0.5 + 0.001 * I(0, 3), ccw));
        if (I(0, 1)) subj.push_back(rectR(cx + 1, cy + 0.1, cx + 2, cy + 0.3, !ccw));  // hole
        if (I(0, 1)) clip.push_back(rectR(cx + 0.5, cy - 0.2, cx + pitch, cy + 0.25, I(0, 3) != 0));  // clip reaching the shared edge
        continue;
      }
      if (mode == 2) {  // diamonds touching at a vertex
        Ring d{{cx, cy + 1}, {cx + 2, cy}, {cx + 4, cy + 1}, {cx + 2, cy + 2}}; if (I(0, 3) == 0) std::reverse(d.begin(), d.end());
        subj.push_back(d); if (I(0, 1)) clip.push_back(star({cx + 2, cy + 1}, 1.2, I(3, 12), I(0, 3) != 0)); continue; }
      double off = mode == 5 ? 0.0 : U(-0.3, 0.3);
      Vec2 c{cx + 2 + off, cy + 2};
      bool ccw = I(0, 4) != 0;
      double R = mode == 5 ? 2.0 : U(0.6, 1.6);
      if (mode == 5) {  // boxes 1e-6 mm apart (1 grid unit): exact square outers
        subj.push_back(rectR(cx, cy, cx + pitch - 0.000001, cy + 1.5, ccw));
      } else subj.push_back(star(c, R, I(3, 30), ccw));
      int kids = I(0, 3);
      if (kids >= 1) subj.push_back(star(c, R * 0.35, I(3, 12), !ccw));   // hole
      if (kids >= 2) subj.push_back(star(c, R * 0.12, I(3, 8), ccw));     // island in hole
      if (kids == 3) subj.push_back(star(c + Vec2{0.2, 0}, R * 0.5, I(3, 10), ccw));  // overlapping same orientation (EvenOdd differs)
      int nc = I(0, 2);
      for (int k = 0; k < nc; ++k) clip.push_back(star(c + Vec2{U(-0.8, 0.8), U(-0.8, 0.8)}, R * U(0.3, 0.9), I(3, 16), I(0, 4) != 0));
      if (I(0, 1)) { Ring o{{q3(c.x - R * 1.2), q3(c.y + U(-1, 1))}, {q3(c.x + U(-0.5, 0.5)), q3(c.y + U(-1, 1))}, {q3(c.x + R * 1.2), q3(c.y + U(-1, 1))}}; opens.push_back(o); }
      if (I(0, 5) == 0) { Ring o{{q3(c.x - 0.3), c.y}, {q3(c.x + 0.3), c.y}}; opens.push_back(o); }  // short open inside
      if (I(0, 7) == 0) clip.push_back(star({c.x + 30, c.y + 30}, 0.5, 5, true));  // clip-only group far away
    }
    if (mode == 3) {  // a subject crossing several groups and an open path across the whole row
      subj.push_back(rectR(pitch * 3.5, 1.5, pitch * 9.5, 2.5, true));
      opens.push_back({{-1, 2.123}, {pitch * nx + 1, 2.123}});
    }
    for (auto& r : subj) S.push_back(poly(r));
    for (auto& r : opens) S.push_back(poly(r, false));
    for (auto& r : clip) C.push_back(poly(r));
    Tolerance tol{0.001, 1e-9, 0.001};
    for (int op = 0; op < 4; ++op) for (int f = 0; f < 2; ++f) {
      BooleanOp bop = (BooleanOp)op; FillRule fill = f ? FillRule::EvenOdd : FillRule::NonZero;
      BooleanResult O = detail::booleanOpBatched(bop, S, C, tol, fill, std::numeric_limits<size_t>::max());
      BooleanResult B = detail::booleanOpBatched(bop, S, C, tol, fill, 0);
      BooleanResult D = booleanOp(bop, S, C, tol, fill);
      ++scenes; bool batched = !(B.closed == O.closed && B.open == O.open); batchedScenes += batched;
      // D must equal either B (batched) or O (not batched) -- default threshold batches iff depth >= 32
      bool dOk = (D.closed == B.closed && D.open == B.open) || (D.closed == O.closed && D.open == O.open);
      if (!dOk) { ++bad; std::printf("BAD default neither: seed %lu sc %d op %d f %d\n", seed, sc, op, f); }
      std::vector<Ring> rb, ro, rob, roo; for (auto& c : B.closed) rb.push_back(ringOf(c)); for (auto& c : O.closed) ro.push_back(ringOf(c));
      for (auto& c : B.open) rob.push_back(ringOf(c)); for (auto& c : O.open) roo.push_back(ringOf(c));
      if (B.closed.size() != O.closed.size() || B.open.size() != O.open.size()) { ++bad; std::printf("BAD counts seed %lu sc %d op %d f %d mode %d closed %zu/%zu open %zu/%zu\n", seed, sc, op, f, mode, B.closed.size(), O.closed.size(), B.open.size(), O.open.size()); }
      // XOR area of the two closed results
      BooleanResult X = detail::booleanOpBatched(BooleanOp::Xor, B.closed, O.closed, tol, FillRule::NonZero, std::numeric_limits<size_t>::max());
      double xa = 0; for (auto& c : X.closed) xa += std::fabs(area(c));
      double per = 0; for (auto& r : ro) for (size_t i = 0; i < r.size(); ++i) per += distance(r[i], r[(i + 1) % r.size()]);
      worstX = std::max(worstX, per > 0 ? xa / per : xa);
      if (xa > per * 3e-6 + 1e-12) { ++bad; std::printf("BAD xor area %.3g per %.3g seed %lu sc %d op %d f %d mode %d\n", xa, per, seed, sc, op, f, mode); }
      // winding oracle on random points away from all edges
      double xmin = -2, xmax = pitch * nx + 2, ymin = -2, ymax = pitch * ny * 1.1 + 2; if (mode != 3) ymax += 0; 
      int obad = 0;
      for (int k = 0; k < 3000; ++k) {
        Vec2 p{U(xmin, xmax), U(ymin, ymax)};
        if (edgeD(subj, p) < 2e-5 || edgeD(clip, p) < 2e-5 || edgeD(rb, p) < 2e-5 || edgeD(ro, p) < 2e-5) continue;
        int ws = 0, wc = 0; for (auto& r : subj) ws += wind(r, p); for (auto& r : clip) wc += wind(r, p);
        bool is = f ? (ws & 1) : ws != 0, ic = f ? (wc & 1) : wc != 0;
        bool want = op == 0 ? (is || ic) : op == 1 ? (is && !ic) : op == 2 ? (is && ic) : (is != ic);  // Union, Difference, Intersection, Xor
        int wb = 0, wo = 0; for (auto& r : rb) wb += wind(r, p); for (auto& r : ro) wo += wind(r, p);
        ++samples;
        if ((wb != 0) != want || (wo != 0) != want) { if (obad++ < 3) std::printf("BAD oracle seed %lu sc %d op %d f %d p %.6f %.6f want %d B %d O %d\n", seed, sc, op, f, p.x, p.y, want, wb != 0, wo != 0); }
      }
      if (obad) ++bad;
      // open pieces: points along every open input, away from closed edges: covered by B iff by O
      int pbad = 0;
      for (auto& o : opens) for (size_t s = 0; s + 1 < o.size(); ++s) for (int k = 0; k < 40; ++k) {
        Vec2 p = o[s] + (o[s + 1] - o[s]) * U(0, 1);
        if (edgeD(subj, p) < 2e-5 || edgeD(clip, p) < 2e-5) continue;
        bool inB = edgeD(rob, p, false) < 1e-5, inO = edgeD(roo, p, false) < 1e-5; ++opensamples;
        if (inB != inO) { if (pbad++ < 3) std::printf("BAD open seed %lu sc %d op %d f %d p %.6f %.6f B %d O %d\n", seed, sc, op, f, p.x, p.y, inB, inO); }
      }
      if (pbad) ++bad;
    }
  }
  std::printf("seed %lu scenes %ld batched %ld samples %ld opensamples %ld worst xor/perimeter %.3g mm bad %ld\n", seed, scenes, batchedScenes, samples, opensamples, worstX, bad);
  return bad ? 1 : 0;
}
