#define V3_NO_MAIN
#include "v3.cpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/fill_rule.hpp"
// Standalone: polygon from gap_poly.txt, tool d = 2.029 (chordal 0.01). Is the lobe in Clipper's raw
// inset, and does pocketToolpath cut it?
int main() {
    std::vector<Vec2> pts; FILE* f = fopen("gap_poly.txt", "r"); double x, y; while (fscanf(f, "%lf %lf", &x, &y) == 2) pts.push_back({x, y}); fclose(f);
    Scene s; s.addPoly(pts);
    double d = 2.0289999999998996; Vec2 q{-8.017, -2.649};
    PocketParams p; p.cutDepth = 1; p.safeZ = 5;
    auto r = pocketToolpath(s.contours, {tool(d, d / 2, 1)}, p);
    double dmin = 1e9; for (auto& pa : r.plans[0].paths) for (size_t k = 0; k < pa.path.points.size(); ++k) dmin = std::min(dmin, segDist(q, pa.path.points[k], pa.path.points[(k + 1) % pa.path.points.size()]));
    printf("pocket: centre parts %zu, nearest path to lobe centre %.3f (sd there %.4f, r+margin %.4f), msgs %zu\n", r.plans[0].centreRegion.size(), dmin, s.sd(q), d / 2 + 0.01, r.messages.size());
    geometry::Tolerance t = geometry::Tolerance::defaults(); t.chordal = 0.005;
    auto reg = geometry::unionContours(s.contours, t, geometry::FillRule::EvenOdd);
    auto raw = geometry::offsetContours(reg, -(d / 2 + 0.01), geometry::JoinType::Round, t);
    printf("raw Clipper inset: %zu contours:", raw.size());
    for (auto& c : raw) { Box2 b = c.bounds(); auto pl = c.polygonise(t); double mn = 1e9; size_t bad = 0; for (auto v : pl.points) { double e = s.sd(v) - (d / 2 + 0.01); mn = std::min(mn, e); if (e < -0.005 - 1e-6) ++bad; } printf(" [(%.2f,%.2f)-(%.2f,%.2f) n=%zu min %.4f bad(<-slack) %zu]", b.min.x, b.min.y, b.max.x, b.max.y, pl.points.size(), mn, bad); }
    printf("\n");
}
