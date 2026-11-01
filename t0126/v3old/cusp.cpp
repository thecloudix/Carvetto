#define V3_NO_MAIN
#include "v3.cpp"
// Guard attack: rest material made of cusp wedges (most vertices near the cusp) and lobes that
// vanish under large insets; plus thin genuine rest slivers with few vertices.
int main(int argc, char** argv) {
    int fails = 0, n = 0;
    std::vector<std::pair<std::string, Scene>> sc;
    auto discs = [](std::vector<Vec2> c, std::vector<double> rad) { Scene s; Vec2 g{0, 0}; for (auto q : c) g = g + q; g = g * (1.0 / c.size()); std::vector<Vec2> p; int N = 2880;
        for (int i = 0; i < N; ++i) { double t = 2 * PI * i / N; Vec2 u{std::cos(t), std::sin(t)}; double best = -1; for (size_t j = 0; j < c.size(); ++j) { Vec2 f = g - c[j]; double B = f.dot(u), C = f.dot(f) - rad[j] * rad[j]; double disc = B * B - C; if (disc >= 0) best = std::max(best, -B + std::sqrt(disc)); } p.push_back(g + u * best); }
        s.addPoly(p); return s; };
    for (double ov : {0.2, 0.5, 0.8, 0.95}) sc.push_back({"2discs ov" + std::to_string(ov), discs({{0, 0}, {20 * (1 - ov) + 20, 0}}, {10, 10})});
    sc.push_back({"3discs", discs({{0, 0}, {16, 0}, {8, 12}}, {9, 7, 8})});
    // thin triangle wedges: acute corners (cusp-like) of various angles
    for (double ang : {5.0, 10.0, 20.0}) { Scene s; double a = ang * PI / 180; s.addPoly({{0, 0}, {40, -40 * std::tan(a / 2)}, {40, 40 * std::tan(a / 2)}}); sc.push_back({"wedge" + std::to_string(int(ang)), s}); }
    // star with acute spikes
    { Scene s; std::vector<Vec2> p; int k = 7; for (int i = 0; i < 2 * k; ++i) { double t = PI * i / k; double rr = i % 2 ? 6 : 25; p.push_back({rr * std::cos(t), rr * std::sin(t)}); } s.addPoly(p); sc.push_back({"star7", s}); }
    for (auto& [nm, s] : sc) {
        if (!simpleScene(s)) { printf("%s not simple\n", nm.c_str()); continue; }
        std::vector<Config> cfs;
        for (auto st : {PocketStrategy::Offset, PocketStrategy::Raster}) for (auto pp : {ProfilePass::First, ProfilePass::Last}) {
            Config c; c.p.cutDepth = 4; c.p.safeZ = 5; c.p.strategy = st; c.p.profilePass = pp; c.p.rasterAngle = 0.4;
            c.tools = {tool(12, 4.8, 2, "T12"), tool(3, 1.2, 2, "T3"), tool(0.8, 0.3, 2, "T08")}; cfs.push_back(c);
            c.tools = {tool(6, 6, 2, "T6"), tool(1, 1, 2, "T1")}; cfs.push_back(c);
            c.tools = {tool(6, 2.4, 2, "T6"), tool(5.5, 2.2, 2, "T55")}; cfs.push_back(c);   // nearly same-size rest tool: tiny slivers
            c.p.tolerance.chordal = 0.05; c.tools = {tool(10, 4, 2, "T10"), tool(2, 0.8, 2, "T2")}; cfs.push_back(c);
        }
        int f = 0;
        for (auto& cf : cfs) { ++n; int rc = forked([&] { return fullCheck3("cusp " + nm, s, cf, 0.05, -1); }, 900); if (rc) { ++f; if (rc != 1) printf("cusp %s CRASH/TIMEOUT rc=%d %s\n", nm.c_str(), rc, cfgStr(cf).c_str()); } }
        printf("cusp %s: %zu configs, %d fail\n", nm.c_str(), cfs.size(), f); fflush(stdout); fails += f;
    }
    printf("cusp: %d runs, %d fail\n", n, fails);
}
