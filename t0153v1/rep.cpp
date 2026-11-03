// Repro driver: reads contours in the vc.cpp dump format from stdin, runs vcarveToolpath with a V-bit
// <angleDeg> <diameter> <chordal> [flat] [start], prints all moves.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include "carvetto/toolpath/vcarve.hpp"
#include "oracle.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s >> e >> dd >> dir; cs.back().append(ArcSpan{{x, y}, r, s, e, dir ? ArcDirection::CW : ArcDirection::CCW}); }
        else if (t == "B") { double v[8]; for (double& q : v) is >> q; cs.back().append(BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}}); }
    }
    tooling::Tool tool; tool.id = tool.name = "V"; tool.type = tooling::ToolType::VBit; tool.includedAngle = atof(argv[1]) * 3.14159265358979323846 / 180; tool.diameter = atof(argv[2]);
    auto& c = tool.defaultCuttingData; c.feedRate = 1000; c.plungeRate = 300; c.passDepth = 1; c.stepover = argc > 6 ? atof(argv[6]) : 0.2; c.toolNumber = 1; c.spindleSpeed = 10000;
    VCarveParams p; p.tolerance.chordal = atof(argv[3]); if (argc > 4) p.flatDepth = atof(argv[4]); if (argc > 5) p.startDepth = atof(argv[5]);
    auto r = vcarveToolpath(cs, tool, std::nullopt, {}, p);
    for (auto& d : r.diagnostics) std::printf("diag %d %s\n", (int)d.issue, d.message.c_str());
    bool quiet = getenv("REPQ") != nullptr;
    if (!quiet) for (auto& m : r.toolpath.moves) std::printf("%d %.6f %.6f %.6f\n", (int)m.kind, m.to.x, m.to.y, m.to.z);
    // depth shortfall along sloped feed moves (skeleton cuts): expected min(limit, (dist - chordal/2)/tan)
    vo::Boundary B = vo::makeBoundary(cs, p.tolerance.chordal);
    double tn = std::tan(tool.includedAngle / 2), full = tool.diameter / 2 / tn, limit = p.flatDepth > 0 ? std::min(p.flatDepth, full) : full;
    double worst = 0; Vec3 wa{}, wb{}; double wt = 0; Vec3 at{0, 0, 0};
    for (auto& m : r.toolpath.moves) { if (m.kind == toolpath::MoveKind::Feed && std::fabs(m.to.z - at.z) > 1e-9) {
            for (int i = 1; i < 64; ++i) { double t = i / 64.0; Vec2 c{at.x + (m.to.x - at.x) * t, at.y + (m.to.y - at.y) * t}; double z = at.z + (m.to.z - at.z) * t;
                double e = std::min(limit, std::max(0.0, vo::exactDist(B, c).d - p.tolerance.chordal / 2) / tn); double sh = e - (-z - p.startDepth);
                if (sh > worst) { worst = sh; wa = at; wb = m.to; wt = t; } } }
        at = m.to; }
    if (!getenv("REPNOCOV")) { // coverage: carved depth on a grid vs ideal min(limit, dist/tan); report worst lateral shortfall dist - footprint(D)
        std::vector<Vec3> smp; Vec3 q{0, 0, 0};
        for (auto& m : r.toolpath.moves) { if (m.kind != toolpath::MoveKind::Rapid) { double L = std::hypot(m.to.x - q.x, m.to.y - q.y); int n = std::max(1, (int)(L / (p.tolerance.chordal / 8))); for (int i = 0; i <= n; ++i) { double t = double(i) / n; smp.push_back({q.x + (m.to.x - q.x) * t, q.y + (m.to.y - q.y) * t, -(q.z + (m.to.z - q.z) * t) - p.startDepth}); } } q = m.to; }
        double step = atof(getenv("REPGRID") ? getenv("REPGRID") : "0.05"); double wl = 0; Vec2 wp{}; double wD = 0;
        for (double x = B.minx; x <= B.maxx; x += step) for (double y = B.miny; y <= B.maxy; y += step) { Vec2 pt{x, y}; if (!vo::insideEO(B, pt)) continue; double dq = vo::exactDist(B, pt).d; if (dq / tn > limit) continue;
            double Dq = 0; for (auto& s : smp) { double rr = std::hypot(s.x - x, s.y - y); if (rr <= std::min(tool.diameter / 2, s.z * tn)) Dq = std::max(Dq, s.z - rr / tn); }
            double lat = dq - Dq * tn; if (lat > wl) { wl = lat; wp = pt; wD = Dq; } }
        std::printf("coverage: worst lateral shortfall %.5f mm = %.2f*chordal at (%.4f,%.4f) carved %.5f\n", wl, wl / p.tolerance.chordal, wp.x, wp.y, wD);
    }
    std::printf("worst depth shortfall on sloped feeds: %.5f mm (promised <= chordal/tan = %.5f) on move (%.4f,%.4f,%.4f)->(%.4f,%.4f,%.4f) at t=%.3f\n", worst, p.tolerance.chordal / tn, wa.x, wa.y, wa.z, wb.x, wb.y, wb.z, wt);
}
