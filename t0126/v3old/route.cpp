// Targeted routing/guard probes with the v3 checker.
#define V3_NO_MAIN
#include "v3.cpp"
static Config mk3(std::vector<PocketTool> ts, PocketStrategy s, ProfilePass pp, double so_ang, CutDirection dir, bool ramp) {
    Config c; c.tools = ts; c.p.cutDepth = 5; c.p.safeZ = 5; c.p.strategy = s; c.p.profilePass = pp; c.p.rasterAngle = so_ang; c.p.direction = dir; c.p.rampPlunge = ramp; c.p.rampDistance = 6; return c;
}
int main(int argc, char** argv) {
    std::string fam = argc > 1 ? argv[1] : "";
    std::vector<std::pair<std::string, Scene>> sc;
    // dense grids, gaps relative to 2r = 6 (tool 6)
    for (double gap : {5.5, 5.99, 6.0, 6.02, 6.05, 6.5, 9.0, 12.03}) for (int circ = 0; circ < 2; ++circ) {
        Scene s; int nx = 5, ny = 4; double iw = 4, W = nx * iw + (nx + 1) * gap, H = ny * iw + (ny + 1) * gap;
        s.addPoly(rectPts({0, 0}, {W, H}));
        for (int a = 0; a < nx; ++a) for (int b = 0; b < ny; ++b) { double x = gap + a * (iw + gap), y = gap + b * (iw + gap); if (circ) s.addCircle({x + 2, y + 2}, 2); else s.addPoly(rectPts({x, y}, {x + iw, y + iw})); }
        sc.push_back({"grid gap" + std::to_string(gap) + (circ ? "c" : "r"), s});
    }
    // staggered grid (forces diagonal routes)
    for (double gap : {6.1, 7.0, 9.0}) { Scene s; double W = 90, H = 60; s.addPoly(rectPts({0, 0}, {W, H}));
        for (int b = 0; b < 5; ++b) for (int a = 0; a < 7; ++a) { double x = gap + a * (6 + gap) + (b % 2) * (3 + gap / 2), y = gap + b * (4 + gap * 0.6); if (x + 6 > W - gap || y + 4 > H - gap) continue; s.addPoly(rectPts({x, y}, {x + 6, y + 4})); }
        sc.push_back({"stagger" + std::to_string(gap), s}); }
    // islands touching each other (gap 0.01..) forming a wall with one door
    for (double door : {6.05, 7.0, 12.0}) { Scene s; s.addPoly(rectPts({0, 0}, {80, 50}));
        double y = 2; while (y < 48) { double h = 5; if (y < 22 && y + h > 22 - door / 2) { y = 22 + door / 2; continue; } s.addPoly(rectPts({38, y}, {42, std::min(48.0, y + h)})); y += h + 0.01; }
        s.addCircle({20, 25}, 5); s.addCircle({60, 25}, 5);
        sc.push_back({"wall-door" + std::to_string(door), s}); }
    // nested: pocket / island ring / pocket with islands / island
    { Scene s; s.addPoly(rectPts({0, 0}, {80, 60})); s.addPoly(rectPts({10, 10}, {70, 50})); s.addPoly(rectPts({14, 14}, {66, 46})); s.addCircle({30, 30}, 6); s.addCircle({50, 30}, 6); s.addCircle({40, 20}, 1.5); s.addCircle({5, 30}, 1.4); sc.push_back({"nested-isl", s}); }
    // serpentine narrow corridor 6.1 wide
    { std::vector<Vec2> p{{0, 0}}; double w = 6.1, ft = 1, H = 30; double x = 0; std::vector<std::pair<double,int>> fins; for (int f = 0; f < 6; ++f) { x += w; fins.push_back({x, f % 2}); x += ft; } double W = x + w;
      for (auto& [fx, top] : fins) if (!top) { p.push_back({fx, 0}); p.push_back({fx, H - w}); p.push_back({fx + ft, H - w}); p.push_back({fx + ft, 0}); }
      p.push_back({W, 0}); p.push_back({W, H}); for (auto it = fins.rbegin(); it != fins.rend(); ++it) if (it->second) { double fx = it->first; p.push_back({fx + ft, H}); p.push_back({fx + ft, w}); p.push_back({fx, w}); p.push_back({fx, H}); } p.push_back({0, H});
      Scene s; s.addPoly(p); sc.push_back({"serpentine6.1", s}); }
    // flowers / chains (guard)
    for (int k : {3, 5, 8}) for (double a : {0.5, 0.8}) { Scene s; std::vector<Vec2> p; int n = k * 30; for (int i = 0; i < n; ++i) { double t = 2 * PI * i / n; double rr = 20 * (1 + a * std::cos(k * t)); p.push_back({rr * std::cos(t), rr * std::sin(t)}); } s.addPoly(p); sc.push_back({"flower k" + std::to_string(k) + " a" + std::to_string(a), s}); }
    for (double ns : {2.5, 2.9, 3.0, 3.1, 4.0}) { Scene s; std::vector<Vec2> top; auto f = [&](double x) { double h = 0; for (double c : {0.0, 22.0, 38.0}) { double r = c == 0 ? 10 : (c == 22 ? 4.5 : 7); double dx = x - c; if (std::fabs(dx) < r) h = std::max(h, std::sqrt(r * r - dx * dx)); } if (x > 0 && x < 38) h = std::max(h, ns); return h; };
      std::vector<Vec2> p{{-10, 0}}; int N = 900; for (int i = 1; i < N; ++i) { double x = -10 + 55.0 * i / N; p.push_back({x, -f(x)}); } p.push_back({45, 0}); for (int i = N - 1; i >= 1; --i) { double x = -10 + 55.0 * i / N; p.push_back({x, f(x)}); } s.addPoly(p); sc.push_back({"chain neck" + std::to_string(ns), s}); }
    int fails = 0, n = 0;
    for (auto& [nm, s] : sc) {
        if (!fam.empty() && nm.find(fam) == std::string::npos) continue;
        if (!simpleScene(s)) { printf("%s NOT SIMPLE (skipped)\n", nm.c_str()); continue; }
        std::vector<Config> cfs;
        for (double so : {1.2, 2.4, 4.5, 6.0}) for (auto pp : {ProfilePass::None, ProfilePass::First, ProfilePass::Last})
            cfs.push_back(mk3({tool(6, so, 2.5)}, PocketStrategy::Offset, pp, 0, so == 4.5 ? CutDirection::Conventional : CutDirection::Climb, so == 2.4));
        for (auto pp : {ProfilePass::None, ProfilePass::Last}) { cfs.push_back(mk3({tool(6, 2.4, 2.5)}, PocketStrategy::Raster, pp, 0.3, CutDirection::Climb, false));
            cfs.push_back(mk3({tool(12, 4.8, 2.5, "T12"), tool(3, 1.2, 2, "T3")}, PocketStrategy::Offset, pp, 0, CutDirection::Climb, true));
            cfs.push_back(mk3({tool(12, 4.8, 2.5, "T12"), tool(3, 1.2, 2, "T3"), tool(1, 0.4, 1.7, "T1")}, PocketStrategy::Raster, pp, 0.7, CutDirection::Climb, false)); }
        int f = 0;
        for (auto& cf : cfs) { ++n; int rc = forked([&] { return fullCheck3("route " + nm, s, cf, 0.1, -1); }, 600); if (rc) { ++f; if (rc != 1) printf("route %s CRASH/TIMEOUT rc=%d %s\n", nm.c_str(), rc, cfgStr(cf).c_str()); } }
        printf("route %s: %zu configs, %d fail\n", nm.c_str(), cfs.size(), f); fflush(stdout);
        fails += f;
    }
    printf("route: %d runs, %d fail\n", n, fails);
}
