#include <cstdio>
#include <cmath>
#include <random>
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto::toolpath;
int main() {
  std::mt19937_64 g(1); long bad = 0, tot = 0, badLast = 0, badStep = 0;
  for (int i = 0; i < 2000000; ++i) {
    double d = std::pow(10.0, std::uniform_real_distribution<double>(-6, 1.5)(g)); int n = std::uniform_int_distribution<int>(1, 1000)(g);
    auto lv = engraveLevels(d, n); ++tot;
    if (lv.size() != (size_t)n) { if (bad < 8) std::printf("depth %.17g passes %d -> %zu levels\n", d, n, lv.size()); ++bad; continue; }
    if (lv.back() != -d) ++badLast;
    for (int k = 0; k < n; ++k) if (std::fabs(lv[k] + d * (k + 1) / n) > 1e-9 * d + 1e-15) { ++badStep; break; }
  }
  std::printf("tot %ld wrong-count %ld last!=depth %ld step-off %ld\n", tot, bad, badLast, badStep);
  // shallow but practical
  long b2 = 0; for (int n = 1; n <= 1000; ++n) for (double d : {0.001, 0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 3.0}) if (engraveLevels(d, n).size() != (size_t)n) { if (b2 < 10) std::printf("  d %g n %d -> %zu\n", d, n, engraveLevels(d, n).size()); ++b2; }
  std::printf("grid wrong %ld\n", b2);
}
