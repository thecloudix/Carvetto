// Random quick-engrave jobs through every shipped post. Writes <out>/<seed>_<post>.nc and a list line
// "<path> <post> bottom=<b> safe=<s> plunge=<p>" (in mm).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static std::mt19937_64 rng; static const double PI = 3.14159265358979323846;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static std::string rd(const std::string& p) { std::ifstream in(p, std::ios::binary); std::ostringstream s; s << in.rdbuf(); return s.str(); }
int main(int argc, char** argv) {
    std::string posts = argv[1], out = argv[2]; int N = atoi(argv[3]); rng.seed(strtoul(argv[4], nullptr, 10));
    const char* stems[] = {"grbl_mm", "grbl_inch", "mach3_mm", "mach3_inch", "mach4_mm", "linuxcnc_mm", "linuxcnc_inch", "marlin_mm"};
    std::ofstream list(out + "/list" + argv[4] + ".txt");
    for (int j = 0; j < N; ++j) {
        double safe = U(0.5, 15); double deepest = 0; double plunge = 0; std::vector<Toolpath> job; bool samePlunge = true;
        int ntp = I(1, 3);
        for (int t = 0; t < ntp; ++t) {
            std::vector<Contour> cs; double ox = U(-100, 100), oy = U(-100, 100);
            int nv = I(1, 4);
            for (int v = 0; v < nv; ++v) {
                int k = I(0, 4); Vec2 o{ox + U(-30, 30), oy + U(-30, 30)};
                if (k == 0) cs.push_back(Contour::circle(o, U(2, 10)));
                else if (k == 1) cs.push_back(Contour::rectangle(geometry::Box2{o, Vec2{o.x + U(3, 20), o.y + U(3, 20)}}));
                else if (k == 2) { Contour c; c.append(geometry::BezierSpan{o, {o.x + U(-9,9), o.y + U(-9,9)}, {o.x + U(-9,9), o.y + U(-9,9)}, {o.x + U(-9,9), o.y + U(-9,9)}}); c.closed = false; cs.push_back(c); }
                else if (k == 3) { Contour c; Vec2 q{o.x + U(-9,9), o.y + U(-9,9)}; Vec2 m{(o.x+q.x)/2 + U(-3,3), (o.y+q.y)/2 + U(-3,3)}; c.append(geometry::ArcSpan::fromCentre(m, o, q, I(0,1)? geometry::ArcDirection::CW : geometry::ArcDirection::CCW)); c.append(geometry::LineSpan{c.endPoint(), o}); c.closed = I(0,1); if (!c.closed) { Contour d; d.append(c.spans.front()); d.closed = false; c = d; } cs.push_back(c); }
                else { Contour c; c.append(geometry::LineSpan{o, {o.x + U(-9,9), o.y + U(-9,9)}}); c.closed = false; cs.push_back(c); }
            }
            tooling::Tool tool; tool.id = "t" + std::to_string(t); tool.name = "Tool " + std::to_string(t);
            int tk = I(0, 3);
            tool.type = tk == 0 ? tooling::ToolType::VBit : tk == 1 ? tooling::ToolType::Engraving : tk == 2 ? tooling::ToolType::EndMill : tooling::ToolType::BallNose;
            tool.diameter = U(1, 6.35); if (tk <= 1) tool.includedAngle = U(20, 120) * PI / 180; if (tk == 1) tool.tipDiameter = U(0.1, 0.5);
            auto& cd = tool.defaultCuttingData; cd.toolNumber = I(1, 20); cd.feedRate = U(100, 3000); cd.plungeRate = std::round(U(50, 800)); cd.spindleSpeed = std::round(U(5000, 24000)); cd.passDepth = 1; cd.stepover = 0.2;
            EngraveParams p; p.name = "E" + std::to_string(t); p.depth = U(0.05, 1.2); p.passes = I(1, 4); p.safeZ = safe;
            p.strategy = I(0, 1) ? EngraveStrategy::Fill : EngraveStrategy::Outline; p.fillPattern = I(0,1) ? PocketStrategy::Raster : PocketStrategy::Offset;
            p.rasterAngle = U(0, PI); p.stepover = tooling::cuttingWidthAtDepth(tool, p.depth) * U(0.3, 1.0);
            EngraveResult r = engraveToolpath(cs, tool, std::nullopt, p);
            if (!r.ok()) continue;
            if (plunge != 0 && plunge != cd.plungeRate) samePlunge = false;
            plunge = cd.plungeRate;
            deepest = std::max(deepest, p.depth); job.push_back(r.toolpath);
        }
        if (job.empty()) continue;
        for (const char* st : stems) {
            auto pp = post::PostProcessor::parse(rd(posts + "/" + st + ".pp"));
            post::PostContext ctx; ctx.fileName = "j"; ctx.product = "Carvetto"; ctx.home = {0, 0, safe + 10}; ctx.safeZ = safe;
            auto r = pp.value().run(job, ctx);
            if (!r.ok()) { std::printf("POST FAIL %s: %s\n", st, r.error().describe().c_str()); continue; }
            if (!r.value().warnings.empty()) std::printf("WARN %s job %d\n", st, j);
            std::string path = out + "/" + argv[4] + "_" + std::to_string(j) + "_" + st + ".nc";
            std::ofstream(path, std::ios::binary) << r.value().text;
            list << path << " " << st << " bottom=" << -deepest << " safe=" << safe << " plunge=" << (samePlunge ? plunge : -1) << "\n";
        }
    }
}
