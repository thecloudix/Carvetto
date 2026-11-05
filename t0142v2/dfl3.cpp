// T-0142 round-2 verifier: distance exactly at the 1e-6 floor with chordal at each op's minimum, 1e-3 and 1 mm;
// prints whether any InvalidParameters diagnostic appears (a nested Pocket refusal would show here).
#include <cstdio>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "pocket_test_util.hpp"
#include "profile_test_util.hpp"
#include "v_carve_test_util.hpp"
namespace pt = carvetto::test::pocket;
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Box2;
template <class D> static std::string dg(const std::vector<D>& ds) { std::string s; for (auto& d : ds) if (d.message.find("tolerance") != std::string::npos) s += "[" + d.message + "]"; return s; }
int main() {
  std::vector<Contour> shapes{Contour::rectangle(Box2{{0, 0}, {30, 12}}), Contour::circle({15, 6}, 3, geometry::ArcDirection::CW)};
  for (double ch : {1e-4, 3e-4, 1e-3, 1.0}) for (double dist : {1e-6, 3e-6}) {
    geometry::Tolerance tol{dist, 1e-9, ch};
    ProfileParams pp; pp.passes.cutDepth = 2; pp.passes.passDepth = 1; pp.side = ProfileSide::Outside; pp.tolerance = tol;
    auto pr = profileToolpath(shapes, carvetto::test::endMill(6.0), pp);
    PocketParams kp; kp.cutDepth = 1.0; kp.tolerance = tol; auto kr = pocketToolpath(shapes, {pt::endMill("EM 3", 1, 3.0, 1.2, 1.0)}, kp);
    EngraveParams ep; ep.depth = 0.5; ep.safeZ = 5; ep.tolerance = tol; ep.strategy = EngraveStrategy::Fill; ep.fillPattern = PocketStrategy::Offset;
    auto er = engraveToolpath(shapes, carvetto::test::v_carve::endMill(2.0), std::nullopt, ep);
    VCarvingParams vp; vp.flatDepth = 1.0; vp.tolerance = tol;
    auto vr = vCarvingToolpath(shapes, carvetto::test::v_carve::vBit(90, 12.7), std::nullopt, {pt::endMill("EM 3", 3, 3.0, 1.2, 1.0)}, vp);
    std::printf("ch %g dist %g | profile %d %s | pocket %d %s | fill %d %s | vcarve %d %s\n", ch, dist, pr.ok(), dg(pr.diagnostics).c_str(), kr.ok(), dg(kr.diagnostics).c_str(), er.ok(), dg(er.diagnostics).c_str(), vr.ok(), dg(vr.diagnostics).c_str());
  }
}
