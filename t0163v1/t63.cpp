// T-0163 verifier: the branch's regression scene (profile_empty_walk_test.cpp tinyBlob) as a driver,
// to show it crashes main and to print the branch result. arg: lead type 0/1/2
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::BezierSpan; using geometry::Contour;
int main(int argc, char** argv) {
  Contour c; c.closed = true;
  c.spans = {
      BezierSpan{{-0.187086822, -0.120915918}, {-0.187087152, -0.12069675}, {-0.187207274, -0.120374426}, {-0.187405755, -0.120228396}},
      BezierSpan{{-0.187405755, -0.120228396}, {-0.187604236, -0.120082366}, {-0.188050888, -0.119920716}, {-0.188277708, -0.120039738}},
      BezierSpan{{-0.188277708, -0.120039738}, {-0.188504528, -0.12015876}, {-0.188793055, -0.120695069}, {-0.188766163, -0.120946042}},
      BezierSpan{{-0.188766163, -0.120946042}, {-0.18873927, -0.121197014}, {-0.188495223, -0.121448916}, {-0.188269986, -0.121547799}},
      BezierSpan{{-0.188269986, -0.121547799}, {-0.188044749, -0.121646683}, {-0.187730812, -0.121651905}, {-0.187564689, -0.121547244}},
      BezierSpan{{-0.187564689, -0.121547244}, {-0.187398566, -0.121442583}, {-0.187088059, -0.121145133}, {-0.187086822, -0.120915918}}};
  tooling::Tool t; t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::VBit; t.diameter = 0.004; t.includedAngle = 30.0 * 3.14159265358979323846 / 180.0;
  t.defaultCuttingData.passDepth = 4.0; t.defaultCuttingData.feedRate = 1000.0; t.defaultCuttingData.plungeRate = 300.0; t.defaultCuttingData.spindleSpeed = 18000.0; t.defaultCuttingData.toolNumber = 1;
  ProfileParams p; p.side = ProfileSide::On; p.passes.cutDepth = 0.0004; p.passes.passDepth = 0.005; p.separateLastPass = true;
  p.lastPassAllowance = 0.0003777814985338173; p.leads.type = (LeadType)atoi(argv[1]); p.leads.length = 0.0006; p.leads.radius = 0.0003; p.startAt = StartAt::Optimise;
  auto r = profileToolpath({c}, t, p);
  std::printf("lead %s moves %zu diags %zu\n", argv[1], r.toolpath.moves.size(), r.diagnostics.size());
  for (auto& d : r.diagnostics) std::printf("  %s\n", d.message.c_str());
}
