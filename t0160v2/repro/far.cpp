// t0160v1 repro: corner passes at far-from-origin coordinates. RingRegion's eps = 1e-9 * coordinate scale swallows the
// passes' outward extension ((3c + d)/sin(a/2) along the bisector), so pieces outside the wall ring are kept:
// min exact distance(centre, walls) - rho over the carving moves at the flat depth, sharpening on vs off.
// The cone at the flat depth crosses the wall when that is < 0. usage: far
#include <cmath>
#include <cstdio>
#include "lo.hpp"
#include "shapes.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
int main() {
    for (double chd : {0.001, 0.0003, 0.01}) for (double off : {0.0, 1e4, 1e5, 3e5, 1e6, 3e6}) {
        Contour sq = vs::poly({{0, 0}, {20, 0}, {20, 20}, {0, 20}}).transformed(Transform2::translation({off, -0.7 * off}) * Transform2::rotation(0.3));
        std::vector<Contour> cs{sq};
        tooling::Tool v; v.id = v.name = "V90"; v.type = tooling::ToolType::VBit; v.diameter = 12.7; v.includedAngle = M_PI / 2; auto& d = v.defaultCuttingData; d.toolNumber = 1; d.feedRate = 1000; d.plungeRate = 300; d.spindleSpeed = 18000; d.passDepth = 3; d.stepover = 0.5;
        tooling::Tool e = v; e.id = e.name = "EM6"; e.type = tooling::ToolType::EndMill; e.diameter = 6; e.includedAngle = 0; e.defaultCuttingData.toolNumber = 2; e.defaultCuttingData.stepover = 2.4;
        VCarvingParams p; p.flatDepth = 1.0; p.tolerance.chordal = chd; p.tolerance.distance = std::min(0.001, chd);
        va::Acc AC; AC.build(cs); double rho = 1.0, fz = -1.0;
        double res[2];
        for (int on = 0; on < 2; ++on) { p.cornerSharpen = on; auto r = vCarvingToolpath(cs, v, std::nullopt, {PocketTool{e, std::nullopt}}, p);
            double mn = 1e300; Vec3 at{0, 0, p.safeZ}; for (auto& m : r.toolpath.moves) { if (m.kind == MoveKind::Feed && at.z == fz && m.to.z == fz) mn = std::min(mn, lo::segMinClear(AC, {at.x, at.y}, {m.to.x, m.to.y}) - rho); at = m.to; }
            res[on] = r.ok() ? mn : NAN; }
        std::printf("chordal %-7g offset %-8g: min dist - rho at the flat depth: off %+.3e  on %+.3e  (margin c/2 = %.3e)%s\n", chd, off, res[0], res[1], chd / 6, res[1] < 0 ? "  CONE CROSSES WALL" : "");
    }
}
