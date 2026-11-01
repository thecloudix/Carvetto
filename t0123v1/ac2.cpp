// T-0123 verifier (throwaway): AC2 edge cases.
#include <cmath>
#include <cstdio>
#include <limits>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Box2;
static PocketTool em(const char* n, double d, double so, double pd) { PocketTool t; t.tool.name = n; t.tool.diameter = d; t.tool.defaultCuttingData.stepover = so; t.tool.defaultCuttingData.passDepth = pd; t.tool.defaultCuttingData.feedRate = 1000; t.tool.defaultCuttingData.toolNumber = 4; return t; }
static void show(const char* what, const PocketResult& r) { std::printf("%s: ok=%d", what, (int)r.ok()); for (auto& d : r.diagnostics) std::printf(" | issue %d tool %s contour %s: %s", (int)d.issue, d.tool ? std::to_string(*d.tool).c_str() : "-", d.contour ? std::to_string(*d.contour).c_str() : "-", d.message.c_str()); std::printf("\n"); }
int main() {
    PocketParams p; p.cutDepth = 3;
    Contour r = Contour::rectangle(Box2{{0, 0}, {50, 30}});
    show("nan diameter + invalid stepover + huge", pocketToolpath({r}, {em("A", 6, 2.4, 3), em("N", std::nan(""), 1, 3), em("Z", 0.5, -1, 3), em("H", 100, 30, 3)}, p));
    PocketTool v = em("V", 12, 1.0, 3); v.tool.type = tooling::ToolType::VBit; v.tool.includedAngle = 3.14159265358979 / 2;
    auto rv = pocketToolpath({r}, {v}, p); show("vbit 90deg depth 3", rv);
    std::printf("  vbit centre region bounds: %g..%g x %g..%g (radius expected 3 -> 3+0.01)\n", rv.plans[0].centreRegion.empty() ? 0 : rv.plans[0].centreRegion[0].bounds().min.x, rv.plans[0].centreRegion.empty() ? 0 : rv.plans[0].centreRegion[0].bounds().max.x, rv.plans[0].centreRegion.empty() ? 0 : rv.plans[0].centreRegion[0].bounds().min.y, rv.plans[0].centreRegion.empty() ? 0 : rv.plans[0].centreRegion[0].bounds().max.y);
    PocketTool vs = v; vs.tool.defaultCuttingData.stepover = 7.0; show("vbit stepover 7 > width 6", pocketToolpath({r}, {vs}, p));
    PocketTool o = em("O", 6, 2.4, 3); tooling::CuttingData cd = o.tool.defaultCuttingData; cd.stepover = 0; o.cuttingData = cd; show("override stepover 0 (default ok)", pocketToolpath({r}, {o}, p));
    PocketTool o2 = em("O2", 6, 0, 0); cd.stepover = 2.4; cd.passDepth = 1.5; cd.toolNumber = 9; o2.cuttingData = cd; auto ro = pocketToolpath({r}, {o2}, p); show("override fixes invalid default", ro); std::printf("  levels %zu toolNumber %d\n", ro.plans[0].levels.size(), ro.toolpaths[0].tool.toolNumber);
    std::vector<ToolpathDiagnostic> notes; Contour open = r; open.closed = false;
    auto cr = pocketCentreRegion({r, open}, std::nan(""), p, &notes); std::printf("centreRegion nan d: %zu parts", cr.size()); for (auto& d : notes) std::printf(" | issue %d tool %s: %s", (int)d.issue, d.tool ? "set" : "-", d.message.c_str()); std::printf("\n");
    PocketParams bad = p; bad.tolerance.chordal = 0; show("bad tolerance", pocketToolpath({r}, {em("A", 6, 2.4, 3)}, bad));
    show("empty contours", pocketToolpath({}, {em("A", 6, 2.4, 3)}, p));
    // Profile with cutting data override and an invalid default
    tooling::Tool t; t.name = "P"; t.diameter = 3; t.defaultCuttingData.passDepth = 0; ProfileParams pp; pp.passes.cutDepth = 3;
    auto pr = profileToolpath({r}, t, pp); std::printf("profile invalid default passDepth: moves %zu diags %zu %s\n", pr.toolpath.moves.size(), pr.diagnostics.size(), pr.diagnostics.empty() ? "" : pr.diagnostics[0].message.c_str());
    tooling::CuttingData pcd; pcd.passDepth = 1; pcd.feedRate = 777; pp.cuttingData = pcd; pr = profileToolpath({r}, t, pp); std::printf("profile override: moves %zu plunges %zu feed %g diags %zu tool-field-set %d\n", pr.toolpath.moves.size(), pr.toolpath.count(MoveKind::Plunge), pr.toolpath.tool.feedRate, pr.diagnostics.size(), pr.diagnostics.empty() ? 0 : (int)pr.diagnostics[0].tool.has_value());
}
