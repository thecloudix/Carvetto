// T-0142 round-3 verifier: everyHoleNested<Path64> (box hierarchy + edge index, fb08fc9) vs the plain all-pairs scan with
// Clipper2 PointInPolygon, whole-scene and per hole. Scenes: nd.cpp's 7 families + big combs along x/y with holes on
// teeth/vertices, shuffled order (holes before outers), concentric C-rings, duplicates/zero-area, huge coords.
// usage: nd3 seed scenes
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include "clipper2/clipper.h"
#include "hole_nesting.hpp"
using namespace Clipper2Lib;
namespace cd = carvetto::geometry::detail;
struct LP { int64_t x, y; };
static LP lowestOf(const Path64& p) { LP b{INT64_MAX, INT64_MIN}; for (auto& q : p) { if (q.y < b.y || (q.y == b.y && q.x >= b.x)) continue; b = {q.x, q.y}; } return b; }
static bool lowerThan(LP a, LP b) { return a.y > b.y || (a.y == b.y && a.x < b.x); }
static void classify(const Paths64& paths, std::vector<size_t>& with, std::vector<size_t>& against) {
  size_t li = 0; LP lo = lowestOf(paths[0]); for (size_t i = 1; i < paths.size(); ++i) { LP l = lowestOf(paths[i]); if (lowerThan(l, lo)) { lo = l; li = i; } }
  bool rev = Area(paths[li]) < 0; for (size_t i = 0; i < paths.size(); ++i) { double a = Area(paths[i]); if (a == 0) continue; ((a < 0) == rev ? with : against).push_back(i); } }
// round 1: returns allNested
static bool oldNested(const Paths64& paths) { std::vector<size_t> with, against; classify(paths, with, against); if (against.empty()) return true;
  std::vector<Rect64> boxes(paths.size()); for (size_t i : with) boxes[i] = GetBounds(paths[i]);
  std::sort(with.begin(), with.end(), [&](size_t a, size_t b) { return boxes[a].left < boxes[b].left || (boxes[a].left == boxes[b].left && a < b); });
  for (size_t h : against) { Rect64 hb = GetBounds(paths[h]);
    auto last = std::upper_bound(with.begin(), with.end(), hb.left, [&](int64_t v, size_t w) { return v < boxes[w].left; }); bool nested = false;
    for (auto it = with.begin(); it != last && !nested; ++it) { const Rect64& wb = boxes[*it]; if (wb.right < hb.right || wb.top > hb.top || wb.bottom < hb.bottom) continue;
      nested = PointInPolygon(paths[h].front(), paths[*it]) == PointInPolygonResult::IsInside; }
    if (!nested) return false; }
  return true; }
// plain scan over all pairs (round-1 decision, no left-edge pruning): hole nested iff some kind-1 ring's box holds its box and
// Clipper2 PointInPolygon(front) == IsInside.
static bool brute(const Paths64& P, const std::vector<int>& kind) {
  for (size_t h = 0; h < P.size(); ++h) { if (kind[h] != 2) continue; Rect64 hb = GetBounds(P[h]); bool nested = false;
    for (size_t w = 0; w < P.size() && !nested; ++w) { if (kind[w] != 1) continue; Rect64 wb = GetBounds(P[w]);
      if (wb.left <= hb.left && wb.right >= hb.right && wb.top <= hb.top && wb.bottom >= hb.bottom) nested = PointInPolygon(P[h].front(), P[w]) == PointInPolygonResult::IsInside; }
    if (!nested) return false; }
  return true; }
static std::mt19937_64 g;
static int64_t R(int64_t a, int64_t b) { return std::uniform_int_distribution<int64_t>(a, b)(g); }
static Path64 rect(int64_t x0, int64_t y0, int64_t x1, int64_t y1, bool ccw) { Path64 p{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; if (!ccw) std::reverse(p.begin(), p.end()); return p; }
static Path64 ushape(int64_t x0, int64_t y0, int64_t x1, int64_t y1, bool ccw) { int64_t w = x1 - x0, h = y1 - y0; Path64 p{{x0, y0}, {x1, y0}, {x1, y1}, {x1 - w / 3, y1}, {x1 - w / 3, y0 + h / 3}, {x0 + w / 3, y0 + h / 3}, {x0 + w / 3, y1}, {x0, y1}}; if (!ccw) std::reverse(p.begin(), p.end()); return p; }
int main(int argc, char** argv) { g.seed(atoi(argv[1])); int scenes = atoi(argv[2]); long long agree = 0, differ = 0, resolved = 0, nest = 0, holes = 0, holeDiff = 0;
  for (int s = 0; s < scenes; ++s) { Paths64 P; int kindScene = (int)R(0, 11); int n = (int)R(1, 60); int64_t S = R(4, 40);  // grid step
    bool base = R(0, 1);
    if (kindScene == 0) {  // touching cells sharing edges, random orientation, holes inside some
      for (int i = 0; i < n; ++i) { int64_t x = S * (i % 8), y = S * (i / 8); P.push_back(rect(x, y, x + S, y + S, R(0, 5) ? base : !base));
        if (R(0, 1)) { int64_t a = R(0, S / 2), b = R(S / 2, S); if (b > a) P.push_back(rect(x + a, y + a, x + b, y + b, R(0, 5) ? !base : base)); } } }
    else if (kindScene == 1) {  // huge outer, many children, children touching outer edges and each other
      int64_t W = S * 20; P.push_back(rect(0, 0, W, W, base)); for (int i = 0; i < n; ++i) { int64_t x = R(0, W - 1), y = R(0, W - 1), w = R(0, S), h = R(0, S);
        P.push_back(rect(x, y, std::min(W, x + w), std::min(W, y + h), R(0, 4) ? !base : base)); } }
    else if (kindScene == 2) {  // deep nesting with exact shared boxes at some levels
      int lv = (int)R(1, 30); int64_t e = 0; for (int l = 0; l < lv; ++l) { P.push_back(rect(e, e, 4000 - e, 4000 - e, (l % 2 == 0) ? base : !base)); if (R(0, 2)) e += R(0, 60); }
      for (int i = 0; i < n; ++i) { int64_t x = R(4100, 9000); P.push_back(rect(x, 0, x + R(0, 30), R(0, 30), R(0, 1) ? base : !base)); } }
    else if (kindScene == 3) {  // U-shapes whose box holds a hole in the notch, plus an island elsewhere
      for (int i = 0; i < n; ++i) { int64_t x = 3 * S * (i % 6), y = 3 * S * (i / 6); P.push_back(ushape(x, y, x + 3 * S, y + 3 * S, base));
        if (R(0, 1)) P.push_back(rect(x + S + 1, y + 2 * S, x + 2 * S - 1, y + 3 * S - 1, !base)); else P.push_back(rect(x + 1, y + 1, x + S - 1, y + S - 1, !base)); } }
    else if (kindScene == 4) {  // boxes touching exactly at a would-be batch boundary: hole touching neighbour's box
      for (int i = 0; i < n; ++i) { int64_t x = 2 * S * i; P.push_back(rect(x, 0, x + S, S, base)); P.push_back(rect(x + S, 0, x + 2 * S, S, R(0, 1) ? !base : base));
        if (R(0, 1)) P.push_back(rect(x + 1, 1, x + S - 1, S - 1, !base)); } }
    else if (kindScene == 5) {  // random rectangles anywhere, random orientation, plus zero-area slivers
      for (int i = 0; i < n; ++i) { int64_t x = R(0, 400), y = R(0, 400); P.push_back(rect(x, y, x + R(0, 200), y + R(0, 200), R(0, 1))); }
      if (R(0, 1)) P.push_back(Path64{{10, 10}, {50, 10}, {30, 10}}); }
    else if (kindScene == 6) {  // hole whose box equals its outer's box (diamond hole in square), and hole vertex on outer boundary
      for (int i = 0; i < n; ++i) { int64_t x = 2 * S * (i % 10), y = 2 * S * (i / 10); P.push_back(rect(x, y, x + S, y + S, base));
        int64_t m = S / 2; Path64 d{{x + m, y}, {x + S, y + m}, {x + m, y + S}, {x, y + m}}; if (base) std::reverse(d.begin(), d.end()); P.push_back(d); } }
    else if (kindScene == 7) {  // big combs (>= 48 vertices, indexed) teeth along x or y, holes on teeth/notches/edges/vertices
      int t = (int)R(12, 80); bool alongY = R(0, 1); auto Q = [&](int64_t a, int64_t b) { return alongY ? Point64(b, a) : Point64(a, b); };
      Path64 c; c.push_back(Q(0, 0)); for (int i = 0; i < t; ++i) { c.push_back(Q(3 * S, 2 * S * i)); c.push_back(Q(3 * S, 2 * S * i + S)); c.push_back(Q(S, 2 * S * i + S)); c.push_back(Q(S, 2 * S * i + 2 * S)); }
      c.push_back(Q(0, 2 * S * t + 2 * S)); if (Area(c) < 0 != !base) std::reverse(c.begin(), c.end()); P.push_back(c);
      for (int i = 0; i < n; ++i) { int64_t a = R(-1, 3 * S + 1), b = R(-1, 2 * S * t + 2 * S + 1); int64_t w = R(0, 3), h = R(0, 3);
        Path64 hr{Q(a, b), Q(a + w, b), Q(a + w, b + h), Q(a, b + h)}; if (R(0, 3) == 0) { int j = (int)R(0, (int)c.size() - 1); hr[0] = c[j]; }
        if ((Area(hr) < 0) != base) std::reverse(hr.begin(), hr.end()); if (R(0, 9) == 0) std::reverse(hr.begin(), hr.end()); P.push_back(hr); } }
    else if (kindScene == 8) {  // holes listed before their outers, shuffled order, touching column + grid
      for (int i = 0; i < n; ++i) { int64_t x = S * (i % 5), y = S * (i / 5); P.push_back(rect(x, y, x + S, y + S, base)); if (R(0, 3)) { int64_t a = R(0, S / 2), b = R(S / 2, S); P.push_back(rect(x + a, y + a, x + b, y + b, !base)); } }
      std::shuffle(P.begin(), P.end(), g); }
    else if (kindScene == 9) {  // many concentric C-rings (box holds hole, inside does not) + a big outer
      int64_t W = 40 * S; P.push_back(rect(-10, -10, W + 10, W + 10, base));
      for (int i = 0; i < n; ++i) { int64_t e = i * (S / 4 + 1); if (2 * e + 4 >= W) break; int64_t x0 = e, y0 = e, x1 = W - e, y1 = W - e, th = 1 + R(0, 1);
        Path64 cr{{x0, y0}, {x1, y0}, {x1, y0 + th}, {x0 + th, y0 + th}, {x0 + th, y1 - th}, {x1, y1 - th}, {x1, y1}, {x0, y1}}; if ((Area(cr) < 0) != !base) std::reverse(cr.begin(), cr.end()); P.push_back(cr); }
      for (int i = 0; i < (int)R(1, 20); ++i) { int64_t x = R(W / 2 - 3, W / 2 + 3), y = R(W / 2 - 3, W / 2 + 3); P.push_back(rect(x, y, x + R(0, 2), y + R(0, 2), !base)); } }
    else if (kindScene == 10) {  // duplicate rings, duplicate vertices, zero-area outers, hole equal to outer
      for (int i = 0; i < n; ++i) { int64_t x = R(0, 10 * S), y = R(0, 10 * S), w = R(0, S), h = R(0, S); Path64 r = rect(x, y, x + w, y + h, R(0, 1));
        if (R(0, 3) == 0) r.insert(r.begin() + 1, r[1]); P.push_back(r); if (R(0, 3) == 0) P.push_back(r); if (R(0, 4) == 0) { Path64 q = r; std::reverse(q.begin(), q.end()); P.push_back(q); } } }
    else {  // huge coordinates: rectangles and diamonds near MAX_COORD with holes on edges
      const int64_t M = (INT64_MAX >> 2) - 1000; int64_t st = R(0, 1) ? 1 : M / 200;
      for (int i = 0; i < n; ++i) { int64_t x = -M + st * R(0, 300) , y = (R(0,1) ? -M : M - 400 * st) + st * R(0, 300); int64_t w = st * R(0, 40), h = st * R(0, 40);
        P.push_back(rect(x, y, x + w, y + h, R(0, 4) ? base : !base)); } }
    if (P.size() < 2) continue;
    // classification as resolveUnnestedHoles
    std::vector<size_t> with, against; classify(P, with, against); std::vector<int> kind(P.size(), 0); for (size_t i : with) kind[i] = 1; for (size_t i : against) kind[i] = 2;
    bool o = brute(P, kind), w = cd::everyHoleNested(P, kind); (o == w ? agree : differ)++; if (!o) ++resolved; else ++nest;
    if (o != w) std::printf("DIFF seed %s scene %d kind %d brute %d new %d\n", argv[1], s, kindScene, o, w);
    for (size_t h : against) { std::vector<int> one = kind; for (size_t j : against) if (j != h) one[j] = 0;
      bool bo = brute(P, one), bw = cd::everyHoleNested(P, one); ++holes; if (bo != bw) { ++holeDiff; if (holeDiff < 10) std::printf("HOLEDIFF seed %s scene %d kind %d hole %zu brute %d new %d\n", argv[1], s, kindScene, h, bo, bw); } } }
  std::printf("nd3 seed %s: agree %lld differ %lld (resolved %lld, allNested %lld) per-hole %lld differ %lld\n", argv[1], agree, differ, resolved, nest, holes, holeDiff); return (differ || holeDiff) ? 1 : 0; }
