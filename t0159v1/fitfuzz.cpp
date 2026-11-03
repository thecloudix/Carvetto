// fitfuzz.cpp: fitTabs with clearSlack = +-kKeepClearSlack and 0 over random closed/open lines and
// keep-clear sets; every kept centre must be >= reach - max(0,slack) from each keep-clear point,
// inside [reach, len - reach], tabs apart by footprint + minGap; refused (-slack) kept set is a
// subset-or-equal in count of the accepted one when moveMargin = 0 for single centres.
#include <cmath>
#include <cstdio>
#include <random>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "carvetto/toolpath/tab_placement.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour;
int main() {
  std::mt19937_64 rng(1590); auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
  long bad = 0, n = 0, singleWorse = 0;
  for (int t = 0; t < 200000; ++t) {
    double off = std::pow(10.0, U(0, 6)) * (t % 2); double L = U(20, 300);
    Contour c; if (t % 3 == 0) c = Contour::rectangle(geometry::Box2{{off, off}, {off + L / 4, off + L / 4}}); else c.append(geometry::LineSpan{{off, off}, {off + L, off}});
    ArcLengthPath lp(c, geometry::Tolerance::defaults()); double len = lp.length();
    double fp = U(0.5, 8); int k = std::uniform_int_distribution<int>(0, 4)(rng);
    std::vector<double> keep; for (int i = 0; i < k; ++i) keep.push_back(U(0, len));
    std::vector<double> cen; int m = std::uniform_int_distribution<int>(1, 6)(rng);
    for (int i = 0; i < m; ++i) { double base = keep.empty() || U(0,1) < 0.3 ? U(0, len) : keep[i % keep.size()] + U(-1, 1) * (0.5 * fp + kMinTabGap); cen.push_back(base); }
    double mm = U(0,1) < 0.5 ? 0.0 : U(0, 0.05); double minGap = U(0, 3);
    for (double slack : {-kKeepClearSlack, 0.0, kKeepClearSlack}) {
      TabPlacement tp = fitTabs(lp, cen, fp, keep, minGap, mm, slack); ++n;
      double reach = 0.5 * fp + kMinTabGap;
      if (tp.centres.size() + tp.dropped != cen.size()) ++bad;
      for (size_t i = 0; i < tp.centres.size(); ++i) { double x = tp.centres[i];
        for (double f : keep) if (std::fabs(x - f) < reach - std::max(0.0, slack)) { ++bad; if (bad < 10) printf("keep-clear violated slack %g d %.17g reach %.17g\n", slack, std::fabs(x - f), reach); }
        if (x < reach - 1e-12 || x > len - reach + 1e-12) { ++bad; if (bad < 10) printf("range slack %g\n", slack); }
        if (i && tp.centres[i] - tp.centres[i - 1] < fp + minGap - 1e-12) { ++bad; if (bad < 10) printf("overlap\n"); } }
    }
    if (cen.size() == 1) { auto r = fitTabs(lp, cen, fp, keep, minGap, mm, -kKeepClearSlack), a = fitTabs(lp, cen, fp, keep, minGap, mm, kKeepClearSlack); if (r.centres.size() > a.centres.size()) ++singleWorse; }
  }
  printf("runs %ld bad %ld refused-keeps-more(single) %ld\n", n, bad, singleWorse);
}
