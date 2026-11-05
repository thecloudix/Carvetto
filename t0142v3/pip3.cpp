// T-0142 round-3 verifier: RingLocator<Path64> (edge-indexed after the 2nd test) vs Clipper2 PointInPolygon.
// Adversarial rings: many vertices on shared rows, duplicate consecutive vertices, zero-length/back-tracking edges,
// spikes, self-overlapping, all-but-one vertex on one row, combs along x and y, huge coordinates up to +-4.6e18
// (Clipper2's MAX_COORD), rings of 3..2000 vertices. Every ring gets >= 2 queries so the index is used.
// usage: pip3 seed rings   prints mismatches (first 10 in detail) and counts per result.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include "clipper2/clipper.h"
#include "hole_nesting.hpp"
using namespace Clipper2Lib;
namespace cd = carvetto::geometry::detail;
static std::mt19937_64 g;
static int64_t R(int64_t a, int64_t b) { return std::uniform_int_distribution<int64_t>(a, b)(g); }
static cd::RingSide ref(Point64 p, const Path64& r) {
  switch (PointInPolygon(p, r)) { case PointInPolygonResult::IsInside: return cd::RingSide::Inside; case PointInPolygonResult::IsOn: return cd::RingSide::On; default: return cd::RingSide::Outside; } }
int main(int argc, char** argv) {
  g.seed(atoll(argv[1])); int rings = atoi(argv[2]); long long cnt[3] = {0, 0, 0}, bad = 0, idx = 0;
  const int64_t MAXC = INT64_MAX >> 2;
  for (int ri = 0; ri < rings; ++ri) {
    int style = (int)R(0, 9); size_t n = (size_t)(R(0, 3) == 0 ? R(3, 12) : R(48, R(0, 5) ? 300 : 2000));
    int64_t base = 0, step = 1;
    int sc = (int)R(0, 3);
    if (sc == 1) { step = R(1, 1000000000000LL); base = R(-MAXC / 2, MAXC / 2); }
    if (sc == 2) { step = R(1, 3); base = R(0, 1) ? MAXC - 100 * step - 10 : -MAXC + 10; }  // near the limit, small steps
    if (sc == 3) { step = MAXC / 40; base = -MAXC / 2; }                                      // huge steps: cross product rounds
    int64_t span = (int64_t)R(2, 30);
    Path64 r;
    auto P = [&](int64_t i, int64_t j) { return Point64(base + step * i, base + step * j); };
    switch (style) {
      case 0: for (size_t i = 0; i < n; ++i) r.push_back(P(R(0, span), R(0, span))); break;                       // random
      case 1: { int64_t x = R(0, span), y = R(0, span); for (size_t i = 0; i < n; ++i) { r.push_back(P(x, y)); (i % 2 ? y : x) = R(0, span); } } break;  // rectilinear
      case 2: { int64_t row = R(0, span); for (size_t i = 0; i < n; ++i) r.push_back(P(R(0, span), R(0, 3) ? row : R(0, span))); } break;  // mostly on one row
      case 3: { for (size_t i = 0; i < n; ++i) { Point64 q = P(R(0, span), R(0, span)); r.push_back(q); if (R(0, 2) == 0) r.push_back(q); } } break;  // duplicates
      case 4: { size_t t = n / 4 + 1; r.push_back(P(0, 0)); for (size_t i = 0; i < t; ++i) { r.push_back(P(3, 2 * i)); r.push_back(P(3, 2 * i + 1)); r.push_back(P(1, 2 * i + 1)); r.push_back(P(1, 2 * i + 2)); } r.push_back(P(0, 2 * t + 2)); } break;  // comb, teeth along x
      case 5: { size_t t = n / 4 + 1; r.push_back(P(0, 0)); for (size_t i = 0; i < t; ++i) { r.push_back(P(2 * i, 3)); r.push_back(P(2 * i + 1, 3)); r.push_back(P(2 * i + 1, 1)); r.push_back(P(2 * i + 2, 1)); } r.push_back(P(2 * t + 2, 0)); } break;  // comb, teeth along y
      case 6: { for (size_t i = 0; i < n; ++i) { int64_t y = R(0, 1) ? 5 : (i % 2 ? 0 : 10); r.push_back(P((int64_t)i % (span + 1), y)); } } break;  // spikes across row 5
      case 7: { for (size_t i = 0; i + 1 < n; ++i) r.push_back(P(R(0, span), 7)); r.push_back(P(R(0, span), R(0, 1) ? 6 : 8)); } break;  // all but one on a row
      case 8: { Path64 a; for (size_t i = 0; i < n / 2 + 2; ++i) a.push_back(P(R(0, span), R(0, span))); r = a; for (size_t i = a.size(); i-- > 0;) r.push_back(a[i]); } break;  // back-tracking
      default: { size_t m = n / 2 + 2; for (size_t i = 0; i < m; ++i) r.push_back(P(i, R(0, 1) ? 0 : 4)); for (size_t i = m; i-- > 0;) r.push_back(P(i, R(0, 1) ? 4 : 8)); } break;  // wavy band
    }
    // random rotation of the start index (first vertex not on the line moves)
    if (!r.empty()) std::rotate(r.begin(), r.begin() + (long)(R(0, (int64_t)r.size() - 1)), r.end());
    cd::RingLocator<Path64> loc(r);
    if (r.size() >= cd::kRingIndexMinVertices) ++idx;
    int q = (int)R(40, 200);
    for (int k = 0; k < q; ++k) {
      Point64 p; const Point64& a = r[(size_t)R(0, (int64_t)r.size() - 1)]; const Point64& b = r[(size_t)R(0, (int64_t)r.size() - 1)];
      switch (R(0, 6)) {
        case 0: p = a; break;
        case 1: p = Point64(a.x + (b.x - a.x) / 2, a.y + (b.y - a.y) / 2); break;
        case 2: p = Point64(base + step * R(-1, span + 1) + R(-1, 1), a.y); break;
        case 3: p = Point64(a.x + R(-1, 1), base + step * R(-1, span + 1) + R(-1, 1)); break;
        case 4: p = Point64(a.x + R(-2, 2), a.y + R(-2, 2)); break;
        case 5: { // a point on an edge exactly when the edge direction divides
          size_t i = (size_t)R(0, (int64_t)r.size() - 1); const Point64& c = r[i]; const Point64& d = r[(i + 1) % r.size()]; int64_t t = R(0, 4);
          p = Point64(c.x + (d.x - c.x) / 4 * t, c.y + (d.y - c.y) / 4 * t); } break;
        default: p = Point64(base + R(-step, (span + 1) * step), base + R(-step, (span + 1) * step));
      }
      cd::RingSide e = ref(p, r), got = loc.locate(p.x, p.y); ++cnt[(int)e];
      if (e != got) { if (bad < 10) std::printf("MISMATCH ring %d style %d n %zu q %d p (%lld,%lld) ref %d got %d\n", ri, style, r.size(), k, (long long)p.x, (long long)p.y, (int)e, (int)got); ++bad; }
    }
  }
  std::printf("pip3 seed %s rings %d indexed %lld out %lld in %lld on %lld MISMATCHES %lld\n", argv[1], rings, idx, cnt[0], cnt[1], cnt[2], bad);
  return bad != 0;
}
