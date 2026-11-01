#define V3_NO_MAIN
#include "v3src.cpp"
int probe2();
int main() { probe2();
    Scene3 s3; Config cf; int att; makeScene3(91000256, s3, cf, att);
    PocketResult res = pocketToolpath(s3.sc.contours, cf.tools, cf.p);
    auto& mv = res.toolpaths[0].moves;
    printf("moves %zu levels:", mv.size()); for (double z : res.plans[0].levels) printf(" %.4f", z); printf("\n");
    for (size_t i = 12385; i < 12402 && i < mv.size(); ++i) printf("%zu kind %d (%.4f,%.4f,%.4f)\n", i, int(mv[i].kind), mv[i].to.x, mv[i].to.y, mv[i].to.z);
    // sd along move 12396 segment
    Vec2 a{mv[12395].to.x, mv[12395].to.y}, b{mv[12396].to.x, mv[12396].to.y}; double mn = 1e9; Vec2 at{};
    for (int k = 0; k <= 4000; ++k) { Vec2 q = a + (b - a) * (k / 4000.0); double s = s3.sc.sd(q); if (s < mn) { mn = s; at = q; } }
    printf("min sd along travel %.4f at (%.3f,%.3f), r+margin %.4f\n", mn, at.x, at.y, cf.tools[0].tool.diameter / 2 + cf.p.tolerance.chordal);
    // paths count per role, and position of last paths
    printf("paths %zu centre parts %zu\n", res.plans[0].paths.size(), res.plans[0].centreRegion.size());
}
// (appended) second probe
int probe2() {
    Scene3 s3; Config cf; int att; makeScene3(91000256, s3, cf, att);
    PocketResult res = pocketToolpath(s3.sc.contours, cf.tools, cf.p);
    auto& mv = res.toolpaths[0].moves; Vec2 c{5.344, 2.372}; double r = cf.tools[0].tool.diameter / 2;
    double best = 1e9; size_t bi = 0;
    for (size_t i = 1; i < 12396; ++i) { if (mv[i].to.z > -1.99 || mv[i-1].to.z > -1.99) continue; Vec2 a{mv[i-1].to.x, mv[i-1].to.y}, b{mv[i].to.x, mv[i].to.y}; double d = segDist(c, a, b); if (d < best) { best = d; bi = i; } }
    printf("cell sd %.4f; nearest level-1 cut move %zu at dist %.4f (r %.4f); sd of reach-centre? stepover %.4f chordal %.4f\n", s3.sc.sd(c), bi, best, r, cf.tools[0].stepover, cf.p.tolerance.chordal);
    for (auto& m : res.messages) printf("msg: %s\n", m.c_str());
    return 0;
}
