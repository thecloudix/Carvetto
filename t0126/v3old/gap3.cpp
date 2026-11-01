#define V3_NO_MAIN
#include "v3.cpp"
static double carea(const Contour& c) { double A = 0; for (auto& sp : c.spans) { auto& l = std::get<LineSpan>(sp); A += l.start.x * l.end.y - l.end.x * l.start.y; } return A / 2; }
int main() {
    Scene3 s3; Config cf; int att; makeScene3(3150371, s3, cf, att); const Scene& sc = s3.sc;
    for (double d : {1.70, 1.7462, 1.80}) {
        PocketParams p; p.cutDepth = 1; p.safeZ = 5;
        auto r = pocketToolpath(sc.contours, {tool(d, d * 0.5, 1)}, p);
        printf("d=%.4f parts %zu:", d, r.plans[0].centreRegion.size());
        for (auto& c : r.plans[0].centreRegion) { Box2 b = c.bounds(); double mn = 1e9; auto pl = c.polygonise(geometry::Tolerance::defaults()); for (auto q : pl.points) mn = std::min(mn, sc.sd(q) - d / 2 - 0.01); printf(" [A %.3f (%.2f,%.2f)-(%.2f,%.2f) n=%zu min(sd-need) %.4f]", carea(c), b.min.x, b.min.y, b.max.x, b.max.y, pl.points.size(), mn); }
        printf("\n");
    }
    // neck widths along the chain: sample sd maxima along the polyline axis
}
