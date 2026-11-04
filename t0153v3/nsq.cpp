// repro3/nestedsq_cone.txt: exact params of sweep fam 18 seed 318 case 5. Prints the carving tool's closed rings (plunge,
// 4+ feeds at one Z, retract) with their clearance to the exact even-odd boundary, and pocketCentreRegion of each
// resolved polygon (uni.cpp) with the wall-ring parameters.
#include <cstdio>
#include <fstream>
#include <sstream>
#include "acc.hpp"
#include "shapes.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::ifstream in(argc > 1 ? argv[1] : "repro3/nestedsq_cone.txt"); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t; if (t == "contour") { cs.emplace_back(); cs.back().closed = true; } else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); } }
    tooling::Tool tool; tool.id = tool.name = "Carve"; tool.type = tooling::ToolType::VBit; tool.diameter = 19.463036828091859; tool.includedAngle = 2.6008341471505432;
    auto& c = tool.defaultCuttingData; c.toolNumber = 1; c.feedRate = 1000; c.plungeRate = 300; c.spindleSpeed = 18000; c.passDepth = 2.1300602860042406; c.stepover = 7.7852147312367439;
    tooling::Tool em; em.id = em.name = "EM0"; em.type = tooling::ToolType::EndMill; em.diameter = 1.5527986289326468; em.defaultCuttingData = c; em.defaultCuttingData.toolNumber = 2; em.defaultCuttingData.stepover = 0.4 * em.diameter;
    VCarvingParams p; p.flatDepth = 1.0015402388490848; p.startDepth = 1.6060279502456001; p.safeZ = 2.8706530995841861; p.tolerance.chordal = 0.005; p.tolerance.distance = 0.001;
    p.clearanceStrategy = PocketStrategy::Offset; p.clearanceRasterAngle = 0.55639517665268257; p.rampPlunge = argc > 2 ? atoi(argv[2]) : 1;
    bool noEm = getenv("NOEM") != nullptr; std::vector<PocketTool> cl; if (!noEm) cl.push_back(PocketTool{em, std::nullopt});
    auto r = vCarvingToolpath(cs, tool, std::nullopt, cl, p);
    va::Acc A; A.build(cs); double tn = std::tan(tool.includedAngle / 2);
    std::printf("ok=%d moves=%zu\n", (int)r.ok(), r.toolpath.moves.size()); for (auto& d : r.diagnostics) std::printf("  diag %d %s\n", (int)d.issue, d.message.c_str());
    const auto& mv = r.toolpath.moves; double worst = 1e300; Vec3 wp{};
    for (size_t i = 1; i < mv.size(); ++i) { if (mv[i].kind != MoveKind::Feed) continue; Vec3 a = mv[i - 1].to, b = mv[i].to; double d = -b.z - p.startDepth; if (d <= 0) continue;
        for (int k = 0; k <= 200; ++k) { double t = k / 200.0; Vec2 q{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}; double dd = -(a.z + (b.z - a.z) * t) - p.startDepth; double cl2 = A.dist(q) - dd * tn; if (cl2 < worst) { worst = cl2; wp = {q.x, q.y, dd}; } } }
    std::printf("worst clearance over carving feed moves: %.6f mm at (%.5f,%.5f) depth %.5f\n", worst, wp.x, wp.y, wp.z);
    Tolerance fine; fine.chordal = 0.005 / 8; fine.distance = std::min(0.001, fine.chordal);
    auto u = unionContours(cs, fine, FillRule::EvenOdd);
    PocketParams w; w.tolerance.chordal = 0.005 / 3; w.tolerance.distance = 0.001; w.cutDepth = p.flatDepth; w.allowance = 0;
    double limR = p.flatDepth * tn;
    for (size_t i = 0; i < u.size(); ++i) { std::vector<ToolpathDiagnostic> dg; auto cr = pocketCentreRegion({u[i]}, 2 * limR, w, &dg);
        for (auto& q : cr) { auto b = q.bounds(); std::printf("poly %zu alone: centre region x[%.6f,%.6f] y[%.6f,%.6f]\n", i, b.min.x, b.max.x, b.min.y, b.max.y); } }
    auto cr = pocketCentreRegion(u, 2 * limR, w, nullptr); for (auto& q : cr) { auto b = q.bounds(); std::printf("all polys: centre region x[%.6f,%.6f] y[%.6f,%.6f]\n", b.min.x, b.max.x, b.min.y, b.max.y); }
    { std::vector<Contour> c14(cs.begin() + 1, cs.end()); auto u14 = unionContours(c14, fine, FillRule::EvenOdd); std::vector<Contour> part{cs[0]}; for (auto& q : u14) part.push_back(q);
      std::vector<ToolpathDiagnostic> dg; auto crp = pocketCentreRegion(part, 2 * limR, w, &dg); for (auto& q : crp) { auto b = q.bounds(); std::printf("V-carve part {contour 0, cluster polys}: centre region x[%.6f,%.6f] y[%.6f,%.6f]\n", b.min.x, b.max.x, b.min.y, b.max.y); }
      for (auto& d : dg) std::printf("   diag %s\n", d.message.c_str());
      std::vector<Contour> noB{cs[0]}; for (size_t i = 0; i < u14.size(); ++i) if (i != 2) noB.push_back(u14[i]); crp = pocketCentreRegion(noB, 2 * limR, w, nullptr); for (auto& q : crp) { auto b = q.bounds(); std::printf("same without strip B: centre region x[%.6f,%.6f] y[%.6f,%.6f]\n", b.min.x, b.max.x, b.min.y, b.max.y); }
      PocketParams w2 = w; w2.tolerance.chordal = 0.0005; crp = pocketCentreRegion(part, 2 * limR, w2, nullptr); for (auto& q : crp) { auto b = q.bounds(); std::printf("part at Pocket chordal 0.0005: centre region x[%.6f,%.6f] y[%.6f,%.6f]\n", b.min.x, b.max.x, b.min.y, b.max.y); } }
    std::vector<Contour> p34{u[2], u[3]}; cr = pocketCentreRegion(p34, 2 * limR, w, nullptr); for (auto& q : cr) { auto b = q.bounds(); std::printf("polys 2+3: centre region x[%.6f,%.6f] y[%.6f,%.6f]\n", b.min.x, b.max.x, b.min.y, b.max.y); }
}
