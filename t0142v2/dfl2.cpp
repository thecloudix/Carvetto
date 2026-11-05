// T-0142 round-2 verifier (d), from t0142v1/dfl.cpp plus Drill (timed, Bezier + nested shapes) and a
// V-Carve without flat depth. Original header:
// T-0142 verifier (d): the 1e-6 mm distance floor. Prints outcome (move count, hash, diagnostics) of
// Profile, Pocket, pocketCentreRegion, Engrave Fill/Outline and V-Carve (with and without an end-mill
// clearance tool, flat depth) across distance tolerances; run against main and branch and diff.
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/drill.hpp"
#include <ctime>
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "pocket_test_util.hpp"
#include "profile_test_util.hpp"
#include "v_carve_test_util.hpp"
namespace pt = carvetto::test::pocket;
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2; using geometry::Box2;
struct H { unsigned long long h = 1469598103934665603ULL; void b(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; } } void d(double x) { b(&x, 8); } };
static unsigned long long ht(const Toolpath& tp) { H h; for (auto& m : tp.moves) { h.d((double)(int)m.kind); h.d(m.to.x); h.d(m.to.y); h.d(m.to.z); h.d(m.centre.x); h.d(m.centre.y); } return h.h; }
static Contour bez() { Contour c; c.closed = true; c.spans.emplace_back(geometry::BezierSpan{{0, 0}, {10, -5}, {20, 5}, {30, 0}}); c.spans.emplace_back(geometry::LineSpan{{30, 0}, {30, 20}});
  c.spans.emplace_back(geometry::BezierSpan{{30, 20}, {20, 25}, {10, 15}, {0, 20}}); c.spans.emplace_back(geometry::LineSpan{{0, 20}, {0, 0}}); return c; }
template <class D> static std::string dg(const std::vector<D>& ds) { std::string s; for (auto& d : ds) s += "[" + d.message + "]"; return s; }
int main() {
  std::vector<Contour> shapes{Contour::rectangle(Box2{{0, 0}, {30, 12}}), Contour::circle({15, 6}, 3, geometry::ArcDirection::CW)};
  std::vector<Contour> bshape{bez()};
  for (double dist : {1e-9, 5e-7, std::nextafter(1e-6, 0.0), 1e-6, 2e-6, 1e-5, 1e-4}) {
    geometry::Tolerance tol{dist, 1e-9, 1e-3};
    // Profile (outside, Bezier shape) and Pocket
    ProfileParams pp; pp.passes.cutDepth = 2; pp.passes.passDepth = 1; pp.side = ProfileSide::Outside; pp.tolerance = tol;
    if (getenv("NOPROF") && dist < 1e-6) { std::printf("dist %.17g profile skipped\n", dist); } else {
    auto pr = profileToolpath(bshape, carvetto::test::endMill(6.0), pp);
    std::printf("dist %.17g profile ok %d moves %zu hash %016llx diags %s\n", dist, pr.ok(), pr.toolpath.moves.size(), ht(pr.toolpath), dg(pr.diagnostics).c_str());
    }
    PocketParams kp; kp.cutDepth = 1.0; kp.tolerance = tol;
    auto kr = pocketToolpath(shapes, {pt::endMill("EM 3", 1, 3.0, 1.2, 1.0)}, kp);
    std::printf("dist %.17g pocket ok %d hash %016llx diags %s\n", dist, kr.ok(), kr.toolpaths.empty() ? 0ULL : ht(kr.toolpaths[0]), dg(kr.diagnostics).c_str());
    std::vector<ToolpathDiagnostic> cd; auto cr = pocketCentreRegion(shapes, 3.0, kp, &cd);
    std::printf("dist %.17g centreRegion rings %zu diags %s\n", dist, cr.size(), dg(cd).c_str());
    // Engrave Fill and Outline
    EngraveParams ep; ep.depth = 0.5; ep.safeZ = 5; ep.tolerance = tol; ep.strategy = EngraveStrategy::Fill; ep.fillPattern = PocketStrategy::Raster;
    auto er = engraveToolpath(shapes, carvetto::test::v_carve::endMill(2.0), std::nullopt, ep);
    std::printf("dist %.17g engraveFill moves %zu hash %016llx diags %s\n", dist, er.toolpath.moves.size(), ht(er.toolpath), dg(er.diagnostics).c_str());
    ep.strategy = EngraveStrategy::Outline; auto eo = engraveToolpath(shapes, carvetto::test::v_carve::endMill(2.0), std::nullopt, ep);
    std::printf("dist %.17g engraveOutline moves %zu hash %016llx diags %s\n", dist, eo.toolpath.moves.size(), ht(eo.toolpath), dg(eo.diagnostics).c_str());
    // V-Carve: flat depth with a clearance end mill, and without
    VCarvingParams vp; vp.flatDepth = 1.0; vp.tolerance = tol;
    auto vr = vCarvingToolpath(shapes, carvetto::test::v_carve::vBit(90, 12.7), std::nullopt, {pt::endMill("EM 3", 3, 3.0, 1.2, 1.0)}, vp);
    std::printf("dist %.17g vcarveClear ok %d moves %zu clear %zu hash %016llx diags %s\n", dist, vr.ok(), vr.toolpath.moves.size(), vr.clearance.empty() ? 0 : vr.clearance[0].moves.size(), ht(vr.toolpath), dg(vr.diagnostics).c_str());
    auto vr2 = vCarvingToolpath(shapes, carvetto::test::v_carve::vBit(90, 12.7), std::nullopt, {}, vp);
    std::printf("dist %.17g vcarveFlat ok %d moves %zu hash %016llx diags %s\n", dist, vr2.ok(), vr2.toolpath.moves.size(), ht(vr2.toolpath), dg(vr2.diagnostics).c_str());
    { tooling::Tool t; t.id="drill"; t.name="Drill 3 mm"; t.type=tooling::ToolType::Drill; t.diameter=3.0; t.includedAngle=118.0*geometry::kPi/180.0;
      t.defaultCuttingData.toolNumber=7; t.defaultCuttingData.feedRate=500; t.defaultCuttingData.plungeRate=300; t.defaultCuttingData.spindleSpeed=10000; t.defaultCuttingData.passDepth=2;
      DrillParams dp; dp.cutDepth = 3; dp.tolerance = tol; std::vector<Contour> holes{bez()}; for (int i = 0; i < 2000; ++i) holes.push_back(Contour::circle({40.0 + 3.0 * (i % 50), 3.0 * (i / 50)}, 1.0 + 1e-7 * i));
      double s = double(std::clock()) / CLOCKS_PER_SEC; auto dr = drillToolpath(holes, t, std::nullopt, dp); double e = double(std::clock()) / CLOCKS_PER_SEC - s;
      std::printf("dist %.17g drill ok %d holes %zu moves %zu hash %016llx cpu %.3f diags %s\n", dist, dr.ok(), dr.holes.size(), dr.toolpath.moves.size(), ht(dr.toolpath), e, dg(dr.diagnostics).c_str()); }
    { VCarvingParams vp0; vp0.tolerance = tol; auto v0 = vCarvingToolpath(shapes, carvetto::test::v_carve::vBit(90, 12.7), std::nullopt, {}, vp0);
      std::printf("dist %.17g vcarvePlain ok %d moves %zu hash %016llx diags %s\n", dist, v0.ok(), v0.toolpath.moves.size(), ht(v0.toolpath), dg(v0.diagnostics).c_str()); }
  }
}
