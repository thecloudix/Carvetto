#include <cstdio>
#include <cmath>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/pass_plan.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static tooling::Tool tool(double pd) { tooling::Tool t; t.id = "d"; t.name = "D"; t.type = tooling::ToolType::Drill; t.diameter = 3; t.includedAngle = 118 * kPi / 180;
  t.defaultCuttingData.toolNumber = 1; t.defaultCuttingData.feedRate = 600; t.defaultCuttingData.plungeRate = 200; t.defaultCuttingData.spindleSpeed = 10000; t.defaultCuttingData.passDepth = pd; return t; }
int main() {
  long cases = 0, bad = 0, at10000ok = 0, at10001ref = 0, refused = 0;
  for (double pd : {0.5, 0.1, 0.3, 1.0 / 3.0, 0.7, 1e-3, 0.123456789, 2.5, 1e-4, 0.0625, 0.2}) {
    for (int N : {9998, 9999, 10000, 10001, 10002}) {
      double base = N * pd;
      std::vector<double> cuts{base, std::nextafter(base, 0.0), std::nextafter(base, 1e300), base * (1 + 1e-15), base * (1 - 1e-15), base * (1 + 1e-12), base * (1 - 1e-12), (N - 1) * pd + pd * 0.5, (N - 1) * pd + pd * 1e-8, (N - 1) * pd + pd * 1e-11, base + 1e-10, base - 1e-10, base + 2e-9, base - 2e-9};
      for (double cut : cuts) for (double start : {0.0, 1.5, 1e3}) for (int pr = 0; pr < 2; ++pr) {
        DrillParams p; p.cutDepth = cut; p.startDepth = start; p.peck = true; p.safeZ = 5; p.peckRetract = pr ? PeckRetract::AbovePreviousPass : PeckRetract::AboveStart;
        PassPlan plan; plan.startDepth = start; plan.cutDepth = cut; plan.passDepth = pd; plan.maintainExactStep = true;
        size_t L = plan.levels().size();
        auto r = drillToolpath({Contour::circle({0, 0}, 5.0)}, tool(pd), std::nullopt, p);
        bool wl = !r.diagnostics.empty() && r.diagnostics[0].issue == ToolpathIssue::WorkLimitExceeded && r.diagnostics[0].message.find("more than 10000 passes per hole") != std::string::npos && r.toolpath.moves.empty();
        bool ok = L <= 10000 ? (r.ok() && r.toolpath.count(MoveKind::Plunge) == L) : wl;
        // distinct depths check: deepest plunge level equals bottom
        ++cases; if (!r.ok()) ++refused;
        if (L == 10000 && r.ok()) ++at10000ok; if (L == 10001 && wl) ++at10001ref;
        if (!ok) { ++bad; if (bad < 15) std::printf("BAD pd %.17g cut %.17g start %g levels %zu ok %d plunges %zu diag %s\n", pd, cut, start, L, r.ok(), r.toolpath.count(MoveKind::Plunge), r.diagnostics.empty() ? "-" : r.diagnostics[0].message.c_str()); }
      }
    }
  }
  std::printf("cases %ld bad %ld refused %ld exactly10000-drilled %ld exactly10001-refused %ld\n", cases, bad, refused, at10000ok, at10001ref);
  // non-peck: one pass whatever the depth
  DrillParams p; p.cutDepth = 1e5; auto r = drillToolpath({Contour::circle({0, 0}, 5.0)}, tool(1e-4), std::nullopt, p);
  std::printf("no peck huge depth: ok %d plunges %zu\n", r.ok(), r.toolpath.count(MoveKind::Plunge));
  // tiny pass: screen
  p.peck = true; p.cutDepth = 10; r = drillToolpath({Contour::circle({0, 0}, 5.0)}, tool(1e-300), std::nullopt, p);
  std::printf("tiny pass: ok %d diag %s\n", r.ok(), r.diagnostics.empty() ? "-" : r.diagnostics[0].message.c_str());
  r = drillToolpath({Contour::circle({0, 0}, 5.0)}, tool(4.9e-324), std::nullopt, p);
  std::printf("denormal pass: ok %d diag %s\n", r.ok(), r.diagnostics.empty() ? "-" : r.diagnostics[0].message.c_str());
}
