// T-0124 round-2 verifier repro (throwaway): Inside profile of a D shape, 12 mm 120-degree V-bit with sharp
// internal corners, spiral ramp, 5 tabs 3.627 x 1.253 (avoid corners; 4 left out, 1 kept). The kept tab's
// raised stretch starts exactly at the inside corner where the V-bit corner excursion is inserted. On the
// final level the tool comes back from the excursion, plunges to the bottom, then FEEDS ON A SLOPE from the
// bottom at the corner up to the tab top at the far end of the raised stretch and plunges again: the tab is
// never flat at the tab top, the tool runs below the tab top over the whole tab (down to 1.25 mm below it
// at the start), and the result still reports the tab as kept. (conn.cpp seed 41001 scene 90.)
#include <cmath>
#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main(int argc, char**) {
    geometry::Contour c; c.closed = true;
    c.append(geometry::ArcSpan{{0, 0}, 33.069952603275667, -0.82891326789637176, -2.3126793856934218, geometry::ArcDirection::CCW});
    c.append(geometry::LineSpan{{-22.344616124565331, -24.37908725585751}, {22.344616124565327, -24.379087255857513}});
    ProfileParams p;
    p.side = ProfileSide::Inside; p.allowance = 0.47890384266872976;
    p.passes.cutDepth = 5.8704981320714289; p.passes.passDepth = 2.3961106328433783; p.passes.lastPassThickness = 0.69316655279558259;
    p.tabs.enabled = true; p.tabs.count = 5; p.tabs.length = 3.6268170792542103; p.tabs.thickness = 1.2531515480900344; p.tabs.avoidCorners = true;
    p.ramp.type = RampType::Spiral; p.ramp.distance = 29.230956653218364;
    p.leads.type = LeadType::Straight; p.leads.length = 8.7415901278800483; p.leads.angle = 0.91409126210320324;
    p.corners.sharpExternal = true; p.corners.sharpInternal = argc == 1;
    p.startPosition = {-1.3579110524131295, 24.403258302710057};
    tooling::Tool t; t.id = "v"; t.name = "V"; t.type = tooling::ToolType::VBit; t.diameter = 12.0; t.includedAngle = 120.0 * 3.14159265358979323846 / 180;
    t.defaultCuttingData.passDepth = 4; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.toolNumber = 1;
    auto res = profileToolpath({c}, t, p);
    for (auto& d : res.diagnostics) std::printf("diag %d %s\n", (int)d.issue, d.message.c_str());
    const double bottom = p.passes.bottomZ(), zTab = bottom + p.tabs.thickness;
    std::printf("bottom %.4f tab top %.4f\n", bottom, zTab);
    const auto& mv = res.toolpath.moves;
    for (size_t i = 1; i < mv.size(); ++i) {
        const auto& a = mv[i - 1].to; const auto& b = mv[i].to;
        bool sloped = (mv[i].kind == MoveKind::Feed) && std::hypot(b.x - a.x, b.y - a.y) > 1.0 && a.z < zTab - 1e-6 && std::fabs(b.z - zTab) < 1e-9;
        std::printf("%3zu kind %d (%9.4f,%9.4f,%8.4f)%s\n", i, (int)mv[i].kind, b.x, b.y, b.z, sloped ? "   <-- feed from below the tab top up to it over the tab" : "");
    }
}
