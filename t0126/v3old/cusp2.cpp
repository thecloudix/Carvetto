#define V3_NO_MAIN
#include "v3.cpp"
// Two-disc unions built from exact arcs: concave cusps at the intersection points.
static Scene twoDiscs(double R1, double R2, double D) {
    Scene s; double x0 = (D * D + R1 * R1 - R2 * R2) / (2 * D), y0 = std::sqrt(R1 * R1 - x0 * x0);
    double a1 = std::atan2(y0, x0), b0 = std::atan2(-y0, x0 - D), b1 = std::atan2(y0, x0 - D);
    Contour c;
    // arc on A from a1 to 2pi - a1 (CCW, the far side)
    c.append(ArcSpan{{0, 0}, R1, a1, 2 * PI - a1, ArcDirection::CCW});
    s.prims.push_back(Prim{1, {}, {}, {0, 0}, R1, a1, 2 * PI - 2 * a1});
    double e = b1; while (e <= b0) e += 2 * PI;
    c.append(ArcSpan{{D, 0}, R2, b0, e, ArcDirection::CCW});
    s.prims.push_back(Prim{1, {}, {}, {D, 0}, R2, b0, e - b0});
    c.closed = true; s.contours.push_back(c); return s;
}
int main() {
    int fails = 0, n = 0;
    std::vector<std::pair<std::string, Scene>> sc;
    for (double ov : {0.1, 0.3, 0.6, 0.9}) sc.push_back({"2discs ov" + std::to_string(ov), twoDiscs(12, 9, (12 + 9) * (1 - ov) + 1e-3)});
    { Scene s = twoDiscs(15, 15, 22); s.addCircle({11, 0}, 2); sc.push_back({"2discs+isl", s}); }
    for (auto& [nm, s] : sc) {
        std::vector<Config> cfs;
        for (auto st : {PocketStrategy::Offset, PocketStrategy::Raster}) for (auto pp : {ProfilePass::First, ProfilePass::Last}) {
            Config c; c.p.cutDepth = 4; c.p.safeZ = 5; c.p.strategy = st; c.p.profilePass = pp; c.p.rasterAngle = 0.4;
            c.tools = {tool(12, 4.8, 2, "T12"), tool(3, 1.2, 2, "T3"), tool(0.8, 0.3, 2, "T08")}; cfs.push_back(c);
            c.tools = {tool(6, 6, 2, "T6"), tool(1, 1, 2, "T1")}; cfs.push_back(c);
            c.tools = {tool(6, 2.4, 2, "T6"), tool(5.5, 2.2, 2, "T55")}; cfs.push_back(c);
            c.p.tolerance.chordal = 0.05; c.tools = {tool(10, 4, 2, "T10"), tool(2, 0.8, 2, "T2")}; cfs.push_back(c);
        }
        int f = 0;
        for (auto& cf : cfs) { ++n; int rc = forked([&] { return fullCheck3("cusp2 " + nm, s, cf, 0.05, -1); }, 900); if (rc) { ++f; if (rc != 1) printf("cusp2 %s CRASH/TIMEOUT rc=%d\n", nm.c_str(), rc); } }
        printf("cusp2 %s: %zu configs, %d fail\n", nm.c_str(), cfs.size(), f); fflush(stdout); fails += f;
    }
    printf("cusp2: %d runs, %d fail\n", n, fails);
}
