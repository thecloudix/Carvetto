#define V3_NO_MAIN
#include "v3.cpp"
int main() {
    Scene3 s3; Config cf; int att; makeScene3(3150371, s3, cf, att); const Scene& sc = s3.sc;
    printf("contours %zu prims %zu box (%.2f,%.2f)-(%.2f,%.2f)\n", sc.contours.size(), sc.prims.size(), sc.box().min.x, sc.box().min.y, sc.box().max.x, sc.box().max.y);
    FILE* f = fopen("gap_poly.txt", "w"); for (auto& p : sc.prims) fprintf(f, "%.17g %.17g\n", p.a.x, p.a.y); fclose(f);
    int lost = 0, n = 0;
    for (double d = 1.0; d <= 3.0; d += 0.0005) for (double ch : {0.01}) {
        PocketParams p; p.cutDepth = 1; p.safeZ = 5; p.tolerance.chordal = ch;
        auto r = pocketToolpath(sc.contours, {tool(d, d * 0.5, 1)}, p);
        double rr = d / 2 + ch;
        // probe: inscribed centre (-8.017,-2.649) sd 1.9788: inside true centre region if sd > rr
        Vec2 q{-8.017, -2.649};
        bool should = sc.sd(q) > rr + 0.05;
        double dmin = 1e9; for (auto& pa : r.plans[0].paths) for (size_t k = 0; k < pa.path.points.size(); ++k) { auto a = pa.path.points[k], b = pa.path.points[(k + 1) % pa.path.points.size()]; dmin = std::min(dmin, segDist(q, a, b)); }
        ++n; if (should && dmin > d) { ++lost; printf("d=%.17g ch=%.3f: lobe lost (nearest path %.3f, centre parts %zu)\n", d, ch, dmin, r.plans[0].centreRegion.size()); }
    }
    printf("%d of %d configs lose the lobe\n", lost, n);
}
