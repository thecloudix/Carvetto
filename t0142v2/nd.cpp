// T-0142 round-2 verifier: nesting decision of resolveUnnestedHoles, round-1 global scan (35b8ca9) vs round-2
// per-separableBatches scan (15848d4), re-implemented here from the two commits' code (only the decision: does
// some hole lack a nesting ring). Adversarial integer scenes: boxes touching exactly at batch boundaries, holes
// whose box equals the outer's, huge outer rings, deep nesting, holes touching outer boundary, zero-area rings,
// U-shaped rings whose box holds a hole they do not hold. Also checks the claim directly: for every hole and every
// ring that passes the box test, both are in one batch. usage: nd seed scenes
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include "clipper2/clipper.h"
#include "offset_batch.hpp"
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
static long long claimViolations = 0;
static bool newNested(const Paths64& paths) { std::vector<size_t> with, against; classify(paths, with, against); if (against.empty()) return true;
  std::vector<Rect64> boxes(paths.size()); std::vector<cd::GridBox> grid(paths.size()); std::vector<int> kind(paths.size(), 0);
  for (size_t i : with) kind[i] = 1; for (size_t i : against) kind[i] = 2;
  for (size_t i = 0; i < paths.size(); ++i) { boxes[i] = GetBounds(paths[i]); grid[i] = {boxes[i].left, boxes[i].top, boxes[i].right, boxes[i].bottom}; }
  auto batches = cd::separableBatches(grid);
  // claim check: hole h and any "with" ring whose box contains h's box share a batch
  std::vector<size_t> batchOf(paths.size()); for (size_t b = 0; b < batches.size(); ++b) for (size_t i : batches[b]) batchOf[i] = b;
  for (size_t h : against) for (size_t w : with) { const Rect64& hb = boxes[h]; const Rect64& wb = boxes[w];
    if (!(wb.left > hb.left || wb.right < hb.right || wb.top > hb.top || wb.bottom < hb.bottom) && batchOf[h] != batchOf[w]) ++claimViolations; }
  bool allNested = true; std::vector<size_t> bw;
  for (auto& batch : batches) { bw.clear(); for (size_t i : batch) if (kind[i] == 1) bw.push_back(i);
    std::sort(bw.begin(), bw.end(), [&](size_t a, size_t b) { return boxes[a].left < boxes[b].left || (boxes[a].left == boxes[b].left && a < b); });
    for (size_t h : batch) { if (kind[h] != 2) continue; const Rect64& hb = boxes[h];
      auto last = std::upper_bound(bw.begin(), bw.end(), hb.left, [&](int64_t v, size_t w) { return v < boxes[w].left; }); bool nested = false;
      for (auto it = bw.begin(); it != last && !nested; ++it) { const Rect64& wb = boxes[*it]; if (wb.right < hb.right || wb.top > hb.top || wb.bottom < hb.bottom) continue;
        nested = PointInPolygon(paths[h].front(), paths[*it]) == PointInPolygonResult::IsInside; }
      if (!nested) { allNested = false; break; } }
    if (!allNested) break; }
  return allNested; }
static std::mt19937_64 g;
static int64_t R(int64_t a, int64_t b) { return std::uniform_int_distribution<int64_t>(a, b)(g); }
static Path64 rect(int64_t x0, int64_t y0, int64_t x1, int64_t y1, bool ccw) { Path64 p{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; if (!ccw) std::reverse(p.begin(), p.end()); return p; }
static Path64 ushape(int64_t x0, int64_t y0, int64_t x1, int64_t y1, bool ccw) { int64_t w = x1 - x0, h = y1 - y0; Path64 p{{x0, y0}, {x1, y0}, {x1, y1}, {x1 - w / 3, y1}, {x1 - w / 3, y0 + h / 3}, {x0 + w / 3, y0 + h / 3}, {x0 + w / 3, y1}, {x0, y1}}; if (!ccw) std::reverse(p.begin(), p.end()); return p; }
int main(int argc, char** argv) { g.seed(atoi(argv[1])); int scenes = atoi(argv[2]); long long agree = 0, differ = 0, resolved = 0, nest = 0;
  for (int s = 0; s < scenes; ++s) { Paths64 P; int kindScene = (int)R(0, 6); int n = (int)R(1, 60); int64_t S = R(4, 40);  // grid step
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
    else {  // hole whose box equals its outer's box (diamond hole in square), and hole vertex on outer boundary
      for (int i = 0; i < n; ++i) { int64_t x = 2 * S * (i % 10), y = 2 * S * (i / 10); P.push_back(rect(x, y, x + S, y + S, base));
        int64_t m = S / 2; Path64 d{{x + m, y}, {x + S, y + m}, {x + m, y + S}, {x, y + m}}; if (base) std::reverse(d.begin(), d.end()); P.push_back(d); } }
    if (P.size() < 2) continue;
    bool o = oldNested(P), w = newNested(P); (o == w ? agree : differ)++; if (!o) ++resolved; else ++nest;
    if (o != w) std::printf("DIFF seed %s scene %d kind %d old %d new %d\n", argv[1], s, kindScene, o, w); }
  std::printf("nd seed %s: agree %lld differ %lld (resolved %lld, allNested %lld) claim violations %lld\n", argv[1], agree, differ, resolved, nest, claimViolations); }
