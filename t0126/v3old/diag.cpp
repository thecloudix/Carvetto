#define V3_NO_MAIN
#include "v3.cpp"
static double carea(const Contour& c) { double A = 0; for (auto& sp : c.spans) { auto& l = std::get<LineSpan>(sp); A += l.start.x * l.end.y - l.end.x * l.start.y; } return A / 2; }
int main(int argc, char** argv) {
    uint64_t seed = strtoull(argv[1], 0, 10); size_t ti = atoi(argv[2]);
    Scene3 s3; Config cf; int att; makeScene3(seed, s3, cf, att);
    const Scene& sc = s3.sc;
    printf("%s %s\n", sc.desc.c_str(), cfgStr(cf).c_str());
    auto r = pocketToolpath(sc.contours, cf.tools, cf.p);
    double bottom = -(cf.p.startDepth + cf.p.cutDepth);
    double rmax = 0; for (auto& t : cf.tools) rmax = std::max(rmax, t.tool.diameter / 2);
    HMap g = makeHMap(sc, 0.05, rmax + 1, cf.p.startDepth); SimStats st;
    for (size_t t = 0; t < ti; ++t) simulate(g, r.toolpaths[t], r.plans[t], cf.tools[t].tool.diameter / 2, cf.p.allowance, int(t), st, 0.02);
    const auto& pl = r.plans[ti];
    printf("rest regions %zu\n", pl.restMaterial.size());
    for (auto& c : pl.restMaterial) {
        auto poly = c.polygonise(geometry::Tolerance::defaults()); Scene rs; rs.addPoly(poly.points);
        Box2 b = c.bounds(); long cells = 0, mat = 0;
        for (int j = 0; j < g.ny; ++j) for (int i = 0; i < g.nx; ++i) { Vec2 q = g.at(i, j); if (q.x < b.min.x || q.x > b.max.x || q.y < b.min.y || q.y > b.max.y) continue; if (!rs.inside(q)) continue; ++cells; size_t k = size_t(j) * g.nx + i; if (g.H[k] > bottom + 1e-6 && g.sd[k] >= cf.p.allowance) ++mat; }
        // min sd of contour vertices (how far inside pocket)
        double minsd = 1e9, maxsd = -1e9; for (auto q : poly.points) { double d = sc.sd(q); minsd = std::min(minsd, d); maxsd = std::max(maxsd, d); }
        printf("  area %.4f n=%zu box (%.3f,%.3f)-(%.3f,%.3f) cells %ld withMaterial %ld vertex sd [%.3f, %.3f]%s\n", carea(c), poly.points.size(), b.min.x, b.min.y, b.max.x, b.max.y, cells, mat, minsd, maxsd, (cells > 3 && mat == 0) ? "  <-- PHANTOM?" : "");
    }
    printf("centre region contours %zu\n", pl.centreRegion.size());
    for (auto& c : pl.centreRegion) { Box2 b = c.bounds(); printf("  area %.3f box (%.2f,%.2f)-(%.2f,%.2f)\n", carea(c), b.min.x, b.min.y, b.max.x, b.max.y); }
    for (auto& m : r.messages) printf("msg %s\n", m.c_str());
}
