#define V3_NO_MAIN
#include "v3.cpp"
static double carea(const Contour& c) { double A = 0; for (auto& sp : c.spans) { auto& l = std::get<LineSpan>(sp); A += l.start.x * l.end.y - l.end.x * l.start.y; } return A / 2; }
int main(int argc, char** argv) {
    uint64_t seed = strtoull(argv[1], 0, 10); Vec2 q{atof(argv[2]), atof(argv[3])};
    Scene3 s3; Config cf; int att; makeScene3(seed, s3, cf, att); const Scene& sc = s3.sc;
    printf("q sd=%.4f r=%.4f\n", sc.sd(q), cf.tools[0].tool.diameter / 2);
    // largest inscribed disc near q (search)
    double best = -1; Vec2 bp{}; for (double dx = -4; dx <= 4; dx += 0.01) for (double dy = -4; dy <= 4; dy += 0.01) { Vec2 p{q.x + dx, q.y + dy}; double d = sc.sd(p); if (d > best && std::hypot(dx, dy) < d) { best = d; bp = p; } }
    printf("max sd within reach of q: %.4f at (%.3f,%.3f) (tool fits if > r+0.01)\n", best, bp.x, bp.y);
    for (int pass = 0; pass < 3; ++pass) {
        Config c2 = cf; if (pass == 1) c2.p.strategy = PocketStrategy::Offset; if (pass == 2) { c2.p.profilePass = ProfilePass::Last; }
        auto r = pocketToolpath(sc.contours, c2.tools, c2.p);
        printf("pass %d strat %d pp %d: centre contours %zu:", pass, int(c2.p.strategy), int(c2.p.profilePass), r.plans[0].centreRegion.size());
        for (auto& c : r.plans[0].centreRegion) { Box2 b = c.bounds(); printf(" [%.3f (%.2f,%.2f)-(%.2f,%.2f)]", carea(c), b.min.x, b.min.y, b.max.x, b.max.y); }
        double dmin = 1e9; for (auto& pa : r.plans[0].paths) for (auto x : pa.path.points) dmin = std::min(dmin, std::hypot(x.x - bp.x, x.y - bp.y));
        printf("\n   nearest plan point to the inscribed centre: %.4f; paths %zu\n", dmin, r.plans[0].paths.size());
    }
}
