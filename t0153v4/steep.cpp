// Random V-carve jobs (V-bit/ball nose, flat depth, 0-2 end-mill clearance tools, start depth) through every
// shipped post. Writes <out>/<seed>_<job>_<post>.nc and list<seed>.txt lines for gcheck3.py (incline=1).
// steep.cpp: same jobs as gen.cpp; prints descending feed moves steeper than 1/tan(half angle) (+1e-6) in the carving toolpath.
// usage: steep <posts dir> <out dir> <N> <seed>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include "shapes.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using carvetto::geometry::Contour; using carvetto::geometry::Vec2;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static std::string rd(const std::string& p) { std::ifstream in(p, std::ios::binary); std::ostringstream s; s << in.rdbuf(); return s.str(); }
static tooling::Tool mk(int type, double D, double ang, std::string name, int num) {
    tooling::Tool t; t.id = t.name = name; t.diameter = D; t.type = type == 0 ? tooling::ToolType::VBit : type == 1 ? tooling::ToolType::BallNose : tooling::ToolType::EndMill;
    if (type == 0) t.includedAngle = ang * vo::PI / 180;
    auto& c = t.defaultCuttingData; c.toolNumber = num; c.feedRate = U(100, 3000); c.plungeRate = std::round(U(50, 800)); c.spindleSpeed = std::round(U(5000, 24000)); c.passDepth = U(0.5, 3); c.stepover = 0.4 * D;
    return t;
}
int main(int argc, char** argv) {
    std::string posts = argv[1], out = argv[2]; int N = atoi(argv[3]); rng.seed(strtoul(argv[4], nullptr, 10));
    const char* stems[] = {"grbl_mm", "grbl_inch", "mach3_mm", "mach3_inch", "mach4_mm", "linuxcnc_mm", "linuxcnc_inch", "marlin_mm"};
    std::ofstream list(out + "/list" + argv[4] + ".txt"); auto lib = vs::all();
    for (int j = 0; j < N; ++j) {
        std::vector<Contour> cs; auto& sh = lib[I(0, (int)lib.size() - 1)];
        for (auto& c : sh.cs) { Contour d = c; d.transform(geometry::Transform2{1, 0, 0, 1, U(-50, 50), U(-50, 50)}); cs.push_back(d); }
        int tt = I(0, 3) == 0 ? 1 : 0; tooling::Tool tool = mk(tt, tt == 0 ? U(2, 20) : U(1, 10), U(30, 150), "Carve", 1);
        VCarvingParams p; p.safeZ = U(0.5, 15); p.startDepth = I(0, 2) == 0 ? U(0, 2) : 0; if (I(0, 1)) p.flatDepth = U(0.2, 3);
        std::vector<PocketTool> cl; int n = I(0, 2); for (int i = 0; i < n; ++i) cl.push_back({mk(2, U(1, 6), 0, "EM" + std::to_string(i), 2 + i), std::nullopt});
        p.rampPlunge = I(0, 1); p.clearanceStrategy = I(0, 1) ? PocketStrategy::Raster : PocketStrategy::Offset;
        auto r = vCarvingToolpath(cs, tool, std::nullopt, cl, p);
        if (!r.ok()) continue;
        auto job = r.toolpaths(); const auto& tp = job.back(); double lim = tt == 0 ? 1.0 / std::tan(tool.includedAngle / 2) : 1e300;
        geometry::Vec3 at{0, 0, p.safeZ}; int cnt = 0; double worst = 0; geometry::Vec3 wa{}, wb{};
        for (auto& m : tp.moves) { if (m.kind == MoveKind::Feed) { double dxy = std::hypot(m.to.x - at.x, m.to.y - at.y), dz = at.z - m.to.z;
                if (dz > 1e-6 && dz > lim * dxy * 1.0001 + 1e-9) { ++cnt; double ratio = dxy > 0 ? dz / dxy : 1e300; if (dz > worst) { worst = dz; wa = at; wb = m.to; } (void)ratio; } }
            at = m.to; }
        std::printf("job %d tool=%d ang=%.1f D=%.3f flat=%.3f start=%.3f ncl=%zu ramp=%d: steep descents %d, largest dz %.5f from (%.5f,%.5f,%.5f) to (%.5f,%.5f,%.5f)\n", j, tt, tool.includedAngle * 180 / vo::PI, tool.diameter, p.flatDepth, p.startDepth, cl.size(), (int)p.rampPlunge, cnt, worst, wa.x, wa.y, wa.z, wb.x, wb.y, wb.z);
    }
}
