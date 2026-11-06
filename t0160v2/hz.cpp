// t0160v2 hz: byte hashes of V-carve output for main-vs-branch identity: shapes x {no flat + clearance tools, flat without
// clearance tools, flat + clearance (branch: sharpening off when HAS_CS)} x tools x chordal. Prints one line per job.
#include <cstdio>
#include <cstring>
#include "shapes.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static unsigned long long fnv(const void* p, size_t n, unsigned long long h) { auto* b = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ULL; } return h; }
int main() {
    auto lib = vs::all();
    for (size_t si = 0; si < lib.size(); ++si) for (int tt = 0; tt < 2; ++tt) for (double chd : {0.05, 0.005}) for (int mode = 0; mode < 3; ++mode) for (double em : {1.0, 3.0}) {
        tooling::Tool t; t.id = t.name = "C"; t.type = tt ? tooling::ToolType::BallNose : tooling::ToolType::VBit; t.diameter = tt ? 6 : 12.7; t.includedAngle = tt ? 0 : vo::PI / 3; t.defaultCuttingData = {1, 0.3, 0, 0, 18000, 1500, 500, 0, 1}; t.defaultCuttingData.stepover = 0.3; t.defaultCuttingData.passDepth = 3;
        tooling::Tool e; e.id = e.name = "EM"; e.type = tooling::ToolType::EndMill; e.diameter = em; e.defaultCuttingData = {2, 0.4 * em, 0, 0, 18000, 1500, 500, 0, 2}; e.defaultCuttingData.passDepth = 3; e.defaultCuttingData.stepover = 0.4 * em;
        VCarvingParams p; p.tolerance.chordal = chd; p.flatDepth = mode == 0 ? 0.0 : 0.8; std::vector<PocketTool> cl; if (mode != 1) cl.push_back({e, std::nullopt});
#ifdef HAS_CS
        if (mode == 2) p.cornerSharpen = false;
#endif
        auto r = vCarvingToolpath(lib[si].cs, t, std::nullopt, cl, p); unsigned long long h = 1469598103934665603ULL;
        for (auto& tp : r.toolpaths()) for (auto& m : tp.moves) { h = fnv(&m.kind, sizeof m.kind, h); h = fnv(&m.to, sizeof m.to, h); }
        for (auto& d : r.diagnostics) { h = fnv(&d.issue, sizeof d.issue, h); h = fnv(d.message.data(), d.message.size(), h); }
        std::printf("%s tt%d ch%g mode%d em%g ok%d %016llx\n", lib[si].name.c_str(), tt, chd, mode, em, (int)r.ok(), h);
    }
}
