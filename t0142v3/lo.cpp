// T-0142 verifier (b): outset leftovers / lost regions oracle, independent of Carvetto geometry code.
// Region R of the input = { p : rel * winding(p) > 0 }, rel = orientation of Clipper2's lowest path
// (largest y, then smallest x, first path on ties). dist(p, R) from a dense sampling of R's boundary
// (edge samples whose two sides differ in membership). Checks for offsetContours(input, +d):
//   LOST: p with dist(p,R) <= d - slack not in the result;  FAR: p with dist > reach*d + slack in the result;
//   LEFT: a result ring whose inside-point is farther than reach*d + slack from R (leftover ring).
// usage: lo seed N [family]   families: 0 lone holes + nested (no crossings), 1 crossing rings, 2 tied lowest
// contours, 3 CW-lowest (reversed) groups with lone CCW rings, 4 everything mixed, 5 the card repro family
// (lone circles under a lid, many radii/d/chordal)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
using Ring = std::vector<Vec2>;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static Contour poly(const Ring& p) { Contour c; c.closed = true; for (size_t i = 0; i < p.size(); ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Ring ringOf(const Contour& c) { Ring r; for (auto& s : c.spans) r.push_back(startPoint(s)); return r; }
static double sarea(const Ring& r) { double a = 0; for (size_t i = 0; i < r.size(); ++i) a += r[i].cross(r[(i + 1) % r.size()]); return a / 2; }
static int wind(const Ring& r, Vec2 p) { int w = 0; for (size_t i = 0, n = r.size(); i < n; ++i) { Vec2 a = r[i], b = r[(i + 1) % n];
    double c = (b - a).cross(p - a); if (a.y <= p.y) { if (b.y > p.y && c > 0) ++w; } else if (b.y <= p.y && c < 0) --w; } return w; }
static double segD(Vec2 p, Vec2 a, Vec2 b) { Vec2 d = b - a; double l = d.dot(d); double t = l > 0 ? std::clamp((p - a).dot(d) / l, 0.0, 1.0) : 0; return distance(p, a + d * t); }
static Ring star(Vec2 c, double R, int n, bool ccw, double jit = 0.5) { Ring r; double ph = U(0, 6.28);
  for (int k = 0; k < n; ++k) { double a = ph + 6.283185307179586 * k / n, rr = R * U(1 - jit, 1.0); r.push_back(c + Vec2{rr * std::cos(a), rr * std::sin(a)}); }
  if (!ccw) std::reverse(r.begin(), r.end()); return r; }
static Ring circ(Vec2 c, double R, int n, bool ccw) { return star(c, R, n, ccw, 0.0); }

struct Grid { double h; std::map<std::pair<long, long>, std::vector<Vec2>> cells;
  void add(Vec2 p) { cells[{(long)std::floor(p.x / h), (long)std::floor(p.y / h)}].push_back(p); }
  double nearest(Vec2 p, double cap) const { double best = cap; long cx = (long)std::floor(p.x / h), cy = (long)std::floor(p.y / h); long rad = (long)std::ceil(cap / h) + 1;
    for (long i = cx - rad; i <= cx + rad; ++i) for (long j = cy - rad; j <= cy + rad; ++j) { auto it = cells.find({i, j}); if (it == cells.end()) continue; for (auto& q : it->second) best = std::min(best, distance(p, q)); } return best; } };

int main(int argc, char** argv) {
  unsigned long seed = strtoul(argv[1], 0, 10); int N = atoi(argv[2]); int forceFam = argc > 3 ? atoi(argv[3]) : -1;
  long lost = 0, far = 0, left = 0, scenesBad = 0, totalPts = 0;
  for (int sc = 0; sc < N; ++sc) {
    rng.seed(seed * 1000003 + sc);
    int fam = forceFam >= 0 ? forceFam : I(0, 5);
    std::vector<Ring> rings;
    double lidY = 30;
    bool lidCcw = I(0, 3) != 0;
    if (fam == 6) { // undetected crossing: hole box inside a U-shaped outer's box, first vertex inside, crossing into the notch
      lidCcw = true; double w = U(2, 5), dep = U(4, 9); Ring u{{0, 0}, {10, 0}, {10, 10}, {5 + w / 2, 10}, {5 + w / 2, 10 - dep}, {5 - w / 2, 10 - dep}, {5 - w / 2, 10}, {0, 10}}; rings.push_back(u);
      double R = U(1.0, 3.0), cy = 10 - dep - R * U(-0.6, 0.6); if (cy - R < 0.2) cy = R + 0.2; int n = I(4, 200); Ring h; for (int k = 0; k < n; ++k) { double a = -1.5707963 + 6.283185307179586 * k / n; h.push_back({5 + R * std::cos(a), cy + R * std::sin(a)}); }
      std::reverse(h.begin() + 1, h.end()); rings.push_back(h); Ring lid{{-2, lidY}, {30, lidY}, {30, lidY + 2}, {-2, lidY + 2}}; rings.push_back(lid); }
    else if (fam == 5) { lidCcw = I(0, 1); double R = U(0.2, 3); rings.push_back({{0, 10}, {10, 10}, {10, 20}, {0, 20}}); if (!lidCcw) std::reverse(rings[0].begin(), rings[0].end());
      int nl = I(1, 4); for (int k = 0; k < nl; ++k) rings.push_back(circ({2.0 + 2.5 * k, 5}, R * U(0.3, 1), I(8, 400), !lidCcw)); }
    else {
      int ng = I(1, 6);
      for (int g = 0; g < ng; ++g) {
        Vec2 c{U(0, 25), U(0, 20)}; double R = U(0.3, 4);
        int t = fam == 4 ? I(0, 3) : fam;
        if (t == 0) { bool o = I(0, 1); rings.push_back(star(c, R, I(3, 40), o)); if (I(0, 1)) rings.push_back(star(c, R * 0.3, I(3, 20), !o)); if (I(0, 2) == 0) rings.push_back(star(c + Vec2{R * 2.5, 0}, R * 0.5, I(3, 20), !lidCcw)); }
        else if (t == 1) { // crossing rings of mixed orientation
          rings.push_back(star(c, R, I(3, 40), I(0, 1))); rings.push_back(star(c + Vec2{U(-R, R), U(-R, R)}, R * U(0.3, 1.2), I(3, 40), I(0, 1))); }
        else if (t == 2) { // rings sharing the topmost y (Clipper2's lowest) exactly
          double top = lidY + 2; Ring a{{c.x, top - R}, {c.x + R, top - R}, {c.x + R, top}, {c.x, top}}; Ring b{{c.x + R * U(0.2, 2), top - R * 0.5}, {c.x + R * 2.5, top - R * 0.5}, {c.x + R * 2.5, top}, {c.x + R * U(0.2, 2), top}};
          if (I(0, 1)) std::reverse(a.begin(), a.end()); if (I(0, 1)) std::reverse(b.begin(), b.end()); rings.push_back(a); rings.push_back(b); }
        else { // lone rings of the other orientation, some inside others of the same orientation
          rings.push_back(star(c, R, I(3, 30), !lidCcw)); if (I(0, 1)) rings.push_back(star(c, R * 0.4, I(3, 20), !lidCcw)); if (I(0, 1)) rings.push_back(star(c, R * 0.2, I(3, 20), lidCcw)); }
      }
      if (fam != 2) { Ring lid{{-2, lidY}, {30, lidY}, {30, lidY + 2}, {-2, lidY + 2}}; if (!lidCcw) std::reverse(lid.begin(), lid.end()); rings.push_back(lid); }
    }
    // Clipper2's lowest path (largest y in carvetto coordinates) and rel
    size_t low = 0; double by = -1e300, bx = 1e300; for (size_t i = 0; i < rings.size(); ++i) for (auto& p : rings[i]) { if (p.y < by || (p.y == by && p.x >= bx)) continue; by = p.y; bx = p.x; low = i; }
    int rel = sarea(rings[low]) < 0 ? -1 : 1;
    auto inR = [&](Vec2 p) { int w = 0; for (auto& r : rings) w += wind(r, p); return rel * w > 0; };
    double d = fam == 5 ? (I(0, 2) == 0 ? U(1.0, 4.0) : U(0.05, 1.0)) : U(0.02, 2.0);
    int js = I(0, 2); JoinType join = js == 0 ? JoinType::Round : js == 1 ? JoinType::Square : JoinType::Sharp; double mitre = 2.0;
    double reach = js == 0 ? 1.0 : js == 1 ? std::sqrt(2.0) : mitre;
    double chordal = std::vector<double>{0.01, 0.001, 0.0001}[I(0, 2)]; if (fam != 5 && chordal < 0.001) chordal = 0.001;
    double h = std::max(0.002, d * 0.01);
    double slack = 2 * chordal + 2 * h + 1e-5;
    // boundary samples of R
    Grid G{std::max(0.05, d)};
    for (auto& r : rings) for (size_t i = 0; i < r.size(); ++i) { Vec2 a = r[i], b = r[(i + 1) % r.size()]; double L = distance(a, b); if (L <= 0) continue; Vec2 nrm = (b - a).perp() * (1.0 / L);
      int m = std::max(1, (int)std::ceil(L / h)); for (int k = 0; k <= m; ++k) { Vec2 q = a + (b - a) * (double(k) / m); if (inR(q + nrm * 1e-7) != inR(q - nrm * 1e-7)) G.add(q); } }
    auto distR = [&](Vec2 p, double cap) { return inR(p) ? 0.0 : G.nearest(p, cap); };
    std::vector<Contour> in; for (auto& r : rings) in.push_back(poly(r));
    std::vector<Contour> out = offsetContours(in, d, join, Tolerance{0.001, 1e-9, chordal}, mitre);
    std::vector<Ring> res; for (auto& c : out) res.push_back(ringOf(c));
    auto inRes = [&](Vec2 p) { int w = 0; for (auto& r : res) w += wind(r, p); return w != 0; };
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300; for (auto& r : rings) for (auto& p : r) { xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x); ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y); }
    double m = reach * d + 1; long l0 = lost, f0 = far, le0 = left;
    for (int k = 0; k < 4000; ++k) { Vec2 p{U(xmin - m, xmax + m), U(ymin - m, ymax + m)}; double cap = reach * d + slack + 0.01; double dr = distR(p, cap); ++totalPts;
      bool ir = inRes(p);
      if (dr <= d - slack && !ir) { if (lost - l0 < 2) std::printf("LOST seed %lu sc %d fam %d d %.4f join %d p %.6f %.6f distR %.4g\n", seed, sc, fam, d, js, p.x, p.y, dr); ++lost; }
      if (dr > reach * d + slack && ir) { if (far - f0 < 2) std::printf("FAR seed %lu sc %d fam %d d %.4f join %d p %.6f %.6f\n", seed, sc, fam, d, js, p.x, p.y); ++far; } }
    for (auto& r : res) { if (r.size() < 3) continue; size_t lg = 0; for (size_t i = 0; i < r.size(); ++i) if (distance(r[i], r[(i + 1) % r.size()]) > distance(r[lg], r[(lg + 1) % r.size()])) lg = i;
      Vec2 a = r[lg], b = r[(lg + 1) % r.size()]; double L = distance(a, b); if (L <= 0) continue; Vec2 inw = (b - a).perp() * (1.0 / L) * (sarea(r) > 0 ? 1.0 : -1.0);
      Vec2 q = (a + b) * 0.5 + inw * std::min(1e-4, 0.01 * L); if (!inRes(q)) continue; double cap = reach * d + slack + 0.01;
      if (distR(q, cap) > reach * d + slack) { if (left - le0 < 2) std::printf("LEFT seed %lu sc %d fam %d d %.4f join %d ring of %zu pts area %.3g at %.5f %.5f\n", seed, sc, fam, d, js, r.size(), sarea(r), q.x, q.y); ++left; } }
    if (lost > l0 || far > f0 || left > le0) ++scenesBad;
  }
  std::printf("seed %lu fam %d scenes %d bad %ld lost %ld far %ld leftover %ld points %ld\n", seed, forceFam, N, scenesBad, lost, far, left, totalPts);
  return 0;
}
