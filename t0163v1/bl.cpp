// T-0163 verifier: single closed Bezier teardrop loops (p0 == p3) of size s, side On/Outside/Inside,
// allowance 0, all start choices, leads on: looks for collapsing walks outside the Optimise path.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
int main() {
  for (double s : {0.02, 0.005, 0.002, 0.0012, 0.001, 0.0008, 0.0005, 0.0003, 0.0001})
    for (int side = 0; side < 3; ++side) for (int st = 0; st < 3; ++st) for (int sep = 0; sep < 2; ++sep) for (int tabs = 0; tabs < 2; ++tabs) for (int vb = 0; vb < 2; ++vb) {
      Contour c; c.closed = true; Vec2 p{1.0, 2.0};
      c.append(geometry::BezierSpan{p, p + Vec2{s, 0.2 * s}, p + Vec2{0.3 * s, s}, p});
      tooling::Tool t; t.id = "em"; t.name = "EM"; t.type = vb ? tooling::ToolType::VBit : tooling::ToolType::EndMill; t.diameter = 0.004; t.includedAngle = 0.5;
      t.defaultCuttingData.passDepth = 4.0; t.defaultCuttingData.feedRate = 1000.0; t.defaultCuttingData.plungeRate = 300.0; t.defaultCuttingData.spindleSpeed = 18000.0; t.defaultCuttingData.toolNumber = 1;
      ProfileParams q; q.side = (ProfileSide)side; q.allowance = 0; q.passes.cutDepth = 0.0004; q.passes.passDepth = 0.005; q.separateLastPass = sep; q.lastPassAllowance = 0.0003;
      q.leads.type = LeadType::Circular; q.leads.length = 0.0006; q.leads.radius = 0.0003; q.leads.leadOut = true; q.leads.overcut = 0.0002; q.startAt = (StartAt)st; q.startAnchor = BoxAnchor::BottomLeft;
      q.tabs.enabled = tabs; q.tabs.length = 0.0002; q.tabs.thickness = 0.0001; q.tabs.count = 2;
      std::fflush(stdout); std::fflush(stderr);
      auto r = profileToolpath({c}, t, q);
      bool fin = true; for (auto& m : r.toolpath.moves) fin = fin && std::isfinite(m.to.x) && std::isfinite(m.to.y) && std::isfinite(m.to.z);
      std::printf("s %g side %d st %d sep %d tabs %d vb %d moves %zu diags %zu finite %d %s\n", s, side, st, sep, tabs, vb, r.toolpath.moves.size(), r.diagnostics.size(), (int)fin, r.diagnostics.empty() ? "" : r.diagnostics[0].message.c_str());
    }
}
