#define V3_NO_MAIN
#include "v3.cpp"
int main() {
    std::vector<std::pair<std::string, Scene>> sc;
    sc.push_back({"rect", rect(0, 0, 50, 30)});
    { Scene s; s.addCircle({0, 0}, 20); sc.push_back({"circle", s}); }
    for (auto& [nm, s] : sc) for (double ang : {0.0, 0.3, PI / 4}) {
        PocketParams p; p.cutDepth = 6; p.safeZ = 5; p.strategy = PocketStrategy::Raster; p.profilePass = ProfilePass::None; p.rasterAngle = ang;
        std::vector<PocketTool> ts{tool(12, 4.8, 3, "T12"), tool(3, 1.2, 3, "T3")};
        auto r = pocketToolpath(s.contours, ts, p);
        auto r1 = pocketToolpath(s.contours, {ts[1]}, p);
        auto exc = [&](const PocketResult& rr, const std::vector<PocketTool>& tt, Vec2 q) { double ex = 1e9; for (size_t u = 0; u < rr.toolpaths.size(); ++u) { Vec3 pos{}; const auto& mv = rr.toolpaths[u].moves; for (size_t i = 0; i < mv.size(); ++i) { Vec3 f = i ? pos : mv[i].to; if (mv[i].kind != MoveKind::Rapid && f.z <= -6 + 1e-9 && mv[i].to.z <= -6 + 1e-9) ex = std::min(ex, segDist(q, f.xy(), mv[i].to.xy()) - tt[u].tool.diameter / 2); pos = mv[i].to; } } return ex; };
        // scan a fine band 0..0.6 mm inside the walls
        double worst = -1; Vec2 wat{}; long n = 0;
        Box2 b = s.box();
        for (double x = b.min.x + 0.0125; x < b.max.x; x += 0.025) for (double y = b.min.y + 0.0125; y < b.max.y; y += 0.025) {
            double d = s.sd({x, y}); if (d < 0.016 || d > 0.6) continue;
            double e1 = exc(r1, {ts[1]}, {x, y}); if (e1 > 0) continue;  // T3 alone cuts it
            double e = exc(r, ts, {x, y}); if (e > 1e-9) { ++n; if (e > worst) { worst = e; wat = {x, y}; } }
        }
        printf("%s ang %.2f: points (sd 0.016..0.6) cut by T3 alone but left by T12+T3: %ld, worst excess %.4f at (%.3f,%.3f) sd %.4f\n", nm.c_str(), ang, n, worst, wat.x, wat.y, s.sd(wat));
    }
}
