// markNestedInDust vs brute-force closure; random boxes incl. rows/columns; nested() pseudo-random but pure.
#include <cstdio>
#include <random>
#include "offset_dust.hpp"
using namespace carvetto::geometry::detail;
int main() {
  std::mt19937_64 r(77); long bad = 0, trials = 0, asked_bad = 0;
  for (int t = 0; t < 3000; ++t) {
    int n = 1 + r() % 300; int mode = t % 4; std::vector<GridBox> b(n);
    for (int i = 0; i < n; ++i) {
      int64_t x = r() % 1000, y = r() % 1000, w = r() % 300, h = r() % 300;
      if (mode == 1) x = 0, w = 500;               // column: same x-range
      if (mode == 2) y = 0, h = 500;               // row
      if (mode == 3) { x = (r() % 5) * 10; y = (r() % 5) * 10; w = (r() % 3) * 10; h = (r() % 3) * 10; } // many equal edges
      b[i] = {x, y, x + w, y + h};
    }
    std::vector<bool> usable(n), dust0(n); for (int i = 0; i < n; ++i) { usable[i] = r() % 10 != 0; dust0[i] = r() % 6 == 0; }
    uint64_t salt = r();
    auto nested = [&](size_t o, size_t in) { uint64_t h = (o * 1000003ull + in) ^ salt; h *= 0x9E3779B97F4A7C15ull; return (h >> 60) < 9; };
    auto within = [&](size_t o, size_t in) { return b[in].left >= b[o].left && b[in].right <= b[o].right && b[in].top >= b[o].top && b[in].bottom <= b[o].bottom; };
    std::vector<bool> d1 = dust0;
    markNestedInDust(b, usable, d1, [&](size_t o, size_t in) { if (!usable[o] || !usable[in] || d1[in] || !within(o, in)) ++asked_bad; return nested(o, in); });
    std::vector<bool> d2 = dust0; bool ch = true;
    while (ch) { ch = false; for (int o = 0; o < n; ++o) if (usable[o] && d2[o]) for (int in = 0; in < n; ++in) if (usable[in] && !d2[in] && within(o, in) && nested(o, in)) { d2[in] = true; ch = true; } }
    ++trials; if (d1 != d2) ++bad;
  }
  std::printf("trials %ld mismatches %ld contract-violating asks %ld\n", trials, bad, asked_bad);
}
