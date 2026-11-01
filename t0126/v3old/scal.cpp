#define V3_NO_MAIN
#include "v3.cpp"
// Raster + Profile None: do rest tools clear the clearance tool's wall scallops? Compare leftover
// of (T12, T3) with the leftover of T3 alone (same raster params): cells left by the combination
// but cut by T3 alone are uncleared clearance scallops.
static std::vector<double> leftover(const Scene& s, const std::vector<PocketTool>& ts, const PocketParams& p, HMap& g) {
    auto r = pocketToolpath(s.contours, ts, p);
    SimStats st;
    for (size_t t = 0; t < r.toolpaths.size(); ++t) simulate(g, r.toolpaths[t], r.plans[t], ts[t].tool.diameter / 2, p.allowance, int(t), st, 0.02);
    return g.H;
}
int main() {
    std::vector<std::pair<std::string, Scene>> sc;
    sc.push_back({"rect", rect(0, 0, 50, 30)});
    { Scene s = rect(0, 0, 50, 30); s.addPoly({{15, 10}, {35, 10}, {35, 20}, {15, 20}}); sc.push_back({"rect+isl", s}); }
    { Scene s; s.addCircle({0, 0}, 20); sc.push_back({"circle", s}); }
    int fails = 0;
    for (auto& [nm, s] : sc) for (double ang : {0.0, 0.3, PI / 4}) {
        PocketParams p; p.cutDepth = 6; p.safeZ = 5; p.strategy = PocketStrategy::Raster; p.profilePass = ProfilePass::None; p.rasterAngle = ang;
        HMap g1 = makeHMap(s, 0.05, 7, 0), g2 = makeHMap(s, 0.05, 7, 0);
        auto a = leftover(s, {tool(12, 4.8, 3, "T12"), tool(3, 1.2, 3, "T3")}, p, g1);
        auto b = leftover(s, {tool(3, 1.2, 3, "T3")}, p, g2);
        long bad = 0; double worst = 0; Vec2 at{};
        for (size_t k = 0; k < a.size(); ++k) if (g1.sd[k] > 0.02 && a[k] > -6 + 0.02 && b[k] <= -6 + 1e-9) { ++bad; if (a[k] + 6 > worst) { worst = a[k] + 6; at = g1.at(int(k % g1.nx), int(k / g1.nx)); } }
        printf("%s ang %.2f: cells left by T12+T3 but cut by T3 alone: %ld (worst height %.3f at %.2f,%.2f)\n", nm.c_str(), ang, bad, worst, at.x, at.y);
        if (bad > 20) ++fails;
    }
    return fails;
}
