// T-0131 verifier: BoxIndex vs brute force, sets of only non-finite/huge/NaN boxes and mixed sets.
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>
#include "carvetto/toolpath/box_index.hpp"
using namespace carvetto; using geometry::Box2; using geometry::Vec2;
static std::mt19937_64 rng(131);
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
const double inf = std::numeric_limits<double>::infinity(), nan_ = std::numeric_limits<double>::quiet_NaN(), mx = std::numeric_limits<double>::max();
static double coord(int kind) {
  switch (kind) { case 0: return U(-10, 10); case 1: return U(-1e308, 1e308); case 2: return I(0, 1) ? inf : -inf; case 3: return nan_;
    case 4: return I(0,1) ? mx : -mx; case 5: return U(-1e3, 1e3) + 1e15; case 6: return std::ldexp(U(-1, 1), I(-1074, 1023)); default: return U(-1e-6, 1e-6); }
}
static Box2 rbox(int kinds) {  // kinds: bitmask of allowed coordinate kinds
  auto pick = [&]() { int k; do { k = I(0, 7); } while (!((kinds >> k) & 1)); return coord(k); };
  Box2 b; double x0 = pick(), x1 = pick(), y0 = pick(), y1 = pick();
  if (I(0, 4)) { if (x0 > x1) std::swap(x0, x1); if (y0 > y1) std::swap(y0, y1); }  // sometimes leave reversed (empty)
  if (I(0, 6) == 0) { x1 = x0; y1 = y0; }  // point box
  b.min = {x0, y0}; b.max = {x1, y1}; return b;
}
static std::vector<size_t> brute(const std::vector<Box2>& bs, const Box2& q) { std::vector<size_t> o; for (size_t i = 0; i < bs.size(); ++i) if (bs[i].intersects(q)) o.push_back(i); return o; }
int main(int argc, char** argv) {
  long iters = argc > 1 ? atol(argv[1]) : 200000; long bad = 0, checks = 0, nonEmptyOnly = 0;
  // fixed one-box cases (card)
  Box2 singles[] = {Box2{{0,0},{1,1}}, Box2{{-1e308,0},{1e308,1}}, Box2{{-inf,-1},{inf,1}}, Box2{{-inf,-inf},{inf,inf}}, Box2{{-mx,-mx},{mx,mx}}, Box2{{0,0},{1e308,1e308}}};
  for (auto& s : singles) { toolpath::BoxIndex ix(std::vector<Box2>{s}); Box2 q{{0,0},{1,1}}; if (ix.query(q) != brute({s}, q) || ix.containing({0.5,0.5}) != brute({s}, Box2{{0.5,0.5},{0.5,0.5}})) { ++bad; std::printf("single mismatch\n"); } }
  for (long it = 0; it < iters; ++it) {
    int mode = I(0, 3);
    int kinds = mode == 0 ? 0b11110 /*only non-finite/huge*/ : mode == 1 ? 0b00110 : mode == 2 ? 0xff : 0b1000001 | (1 << I(1, 7));
    int n = I(1, mode == 2 ? 60 : 8);
    std::vector<Box2> bs; for (int i = 0; i < n; ++i) bs.push_back(rbox(kinds));
    if (mode == 3) for (int i = 0; i < I(0, 40); ++i) bs.push_back(rbox(1));  // many finite + a few odd
    toolpath::BoxIndex ix(bs);
    for (int k = 0; k < 6; ++k) {
      Box2 q = k < 3 ? rbox(I(0, 1) ? 1 : 0xff) : rbox(1 | (1 << I(1, 7)));
      auto a = ix.query(q), b = brute(bs, q); ++checks; if (!b.empty()) ++nonEmptyOnly;
      if (a != b) { if (bad++ < 10) { std::printf("MISMATCH it %ld mode %d n %zu: idx %zu brute %zu\n", it, mode, bs.size(), a.size(), b.size()); } }
      Vec2 p{q.min.x, q.min.y}; auto c = ix.containing(p), d = brute(bs, Box2{p, p}); ++checks;
      if (c != d) { if (bad++ < 10) std::printf("MISMATCH containing it %ld mode %d\n", it, mode); }
    }
  }
  std::printf("checks %ld (nonempty brute %ld) mismatches %ld\n", checks, nonEmptyOnly, bad);
  return bad ? 1 : 0;
}
