// T-0124 round-2 verifier repro (throwaway): Outside profile of a 10-tooth gear, 6 mm end mill, no separate
// last pass, 5 tabs 0.388 x 1.417 (avoid corners on, but no straight stretch is long enough). Tabs land on
// the sharp concave corners of the cut (tooth roots). Each tab's raised stretch is 6.388 mm, but the tool at
// its two ends (full depth, below the tab top) stands only ~4.9 mm apart (< 2r = 6): the two discs overlap
// across the stretch, and the flank kerfs close the loop, so no material joins part and waste. Kept, no
// diagnostic about it (only "left on a corner or curve"). Found by conn.cpp seed 41001 scene 314.
#include <cmath>
#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main(int argc, char**) {
    std::vector<Vec2> P = {
        {0x1.3818209cdf692p+3, 0x0p+0},
        {0x1.b2a9c04745aap+3, 0x1.b74926870a02ep+0},
        {0x1.a0acbdaeb6c7fp+3, 0x1.0ec58cc07e1ddp+2},
        {0x1.1a642d6d90a64p+3, 0x1.09c43ba8926c3p+2},
        {0x1.f8faa6523f088p+2, 0x1.6ee387e227531p+2},
        {0x1.3f5fc61a14a5cp+3, 0x1.2be989c076739p+3},
        {0x1.0184e84e27044p+3, 0x1.6271d9c84d2cep+3},
        {0x1.2cb4817e85828p+2, 0x1.117d8298d838fp+3},
        {0x1.81c50b6abf3ecp+1, 0x1.28d1bb8aa8a1cp+3},
        {0x1.48615e189be9dp+1, 0x1.ae5b85bba9a8ap+3},
        {0x1.e345980e16ec2p-51, 0x1.b61e26d01c3bep+3},
        {-0x1.38ed1eb5f824fp+0, 0x1.35a21f6071b6ep+3},
        {-0x1.81c50b6abf3eap+1, 0x1.28d1bb8aa8a1cp+3},
        {-0x1.75153c96ac05bp+2, 0x1.8c6bbacfd9929p+3},
        {-0x1.0184e84e27043p+3, 0x1.6271d9c84d2cep+3},
        {-0x1.ab4953583239ap+2, 0x1.c7038a67b4affp+2},
        {-0x1.f8faa6523f087p+2, 0x1.6ee387e227532p+2},
        {-0x1.7feceffeae017p+3, 0x1.a62134078a3a4p+2},
        {-0x1.a0acbdaeb6c7fp+3, 0x1.0ec58cc07e1dfp+2},
        {-0x1.3290f283a7fd2p+3, 0x1.d3d854bfbd1f9p+0},
        {-0x1.3818209cdf692p+3, 0x1.58428616b06b5p-50},
        {-0x1.b2a9c04745aap+3, -0x1.b74926870a03p+0},
        {-0x1.a0acbdaeb6c8p+3, -0x1.0ec58cc07e1dcp+2},
        {-0x1.1a642d6d90a63p+3, -0x1.09c43ba8926c4p+2},
        {-0x1.f8faa6523f089p+2, -0x1.6ee387e22753p+2},
        {-0x1.3f5fc61a14a5cp+3, -0x1.2be989c07673ap+3},
        {-0x1.0184e84e27045p+3, -0x1.6271d9c84d2cdp+3},
        {-0x1.2cb4817e85827p+2, -0x1.117d8298d838fp+3},
        {-0x1.81c50b6abf3efp+1, -0x1.28d1bb8aa8a1cp+3},
        {-0x1.48615e189be9ap+1, -0x1.ae5b85bba9a8bp+3},
        {-0x1.6a74320a91311p-49, -0x1.b61e26d01c3bep+3},
        {0x1.38ed1eb5f8253p+0, -0x1.35a21f6071b6ep+3},
        {0x1.81c50b6abf3e8p+1, -0x1.28d1bb8aa8a1cp+3},
        {0x1.75153c96ac059p+2, -0x1.8c6bbacfd992ap+3},
        {0x1.0184e84e27043p+3, -0x1.6271d9c84d2cfp+3},
        {0x1.ab49535832399p+2, -0x1.c7038a67b4b01p+2},
        {0x1.f8faa6523f087p+2, -0x1.6ee387e227534p+2},
        {0x1.7feceffeae017p+3, -0x1.a62134078a3a6p+2},
        {0x1.a0acbdaeb6c7fp+3, -0x1.0ec58cc07e1e1p+2},
        {0x1.3290f283a7fd2p+3, -0x1.d3d854bfbd1fep+0}};
    geometry::Contour c; c.closed = true;
    for (size_t i = 0; i < P.size(); ++i) c.append(geometry::LineSpan{P[i], P[(i + 1) % P.size()]});
    ProfileParams p;
    p.side = ProfileSide::Outside; p.allowance = 0x1.3251f836c5991p-2;
    p.passes.cutDepth = 0x1.2b8f107f8b25ap+2; p.passes.passDepth = 0x1.58df19fd4f982p-1;
    p.tabs.enabled = true; p.tabs.count = 5; p.tabs.length = 0x1.8d5f26853ba7ep-2; p.tabs.thickness = 0x1.6ad521abda6c3p+0; p.tabs.avoidCorners = true;
    if (argc == 1) {  // as found (smooth ramp, overcut); with any argument: plain
        p.ramp.type = RampType::Smooth; p.ramp.distance = 0x1.e745ae236a21bp+4; p.ramp.onLeadIn = true; p.ramp.angle = 0x1.26594859b23a7p-3;
        p.leads.overcut = 0x1.270666784b5e4p+3;
    }
    tooling::Tool t; t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::EndMill; t.diameter = 6.0;
    t.defaultCuttingData.passDepth = 4; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.toolNumber = 1;
    auto res = profileToolpath({c}, t, p);
    for (auto& d : res.diagnostics) std::printf("diag %d %s\n", (int)d.issue, d.message.c_str());
    const double zTab = p.passes.bottomZ() + p.tabs.thickness, r = 3.0;
    const auto& mv = res.toolpath.moves;
    int tabs = 0, severed = 0;
    // every tab plateau on the final level: the tool position just before the rise and just after the plunge
    for (size_t i = 1; i + 1 < mv.size(); ++i) {
        if (!(mv[i].to.z == zTab && mv[i - 1].to.z < zTab && mv[i].to.x == mv[i - 1].to.x && mv[i].to.y == mv[i - 1].to.y)) continue;
        if (mv[i - 1].to.z != p.passes.bottomZ()) continue;
        size_t j = i + 1; while (j < mv.size() && mv[j].to.z == zTab) ++j;
        Vec2 a = mv[i - 1].to.xy(), b = mv[j].to.xy();
        double d = std::hypot(a.x - b.x, a.y - b.y);
        ++tabs; if (d < 2 * r) ++severed;
        std::printf("tab: tool at bottom before (%.4f,%.4f) and after (%.4f,%.4f) the raised stretch: %.4f apart (2r = %.1f)%s\n", a.x, a.y, b.x, b.y, d, 2 * r, d < 2 * r ? "  -> discs overlap: tab cut through" : "");
    }
    std::printf("final-level tabs %d, cut through by their own pass %d\n", tabs, severed);
}
