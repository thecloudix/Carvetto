#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry; using vo::PI; using vo::dist;
int main() {
    for (double ch : {0.01, 0.001, 0.1}) {
    Tolerance tol; tol.chordal = ch;
    for (double A : {2.0, 10.0, 30.0, 45.0, 60.0, 90.0, 120.0, 150.0, 170.0, 178.0}) {
        // apex at origin, opening along +x, legs length 20, closed by a far arc-free polygon
        double h = A * PI / 360; Vec2 p1{20 * std::cos(h), -20 * std::sin(h)}, p2{20 * std::cos(h), 20 * std::sin(h)};
        std::vector<Vec2> pts{{0, 0}, p1, {p1.x + 20, p1.y}, {p2.x + 20, p2.y}, p2};
        std::vector<Contour> cs{vs::poly(pts)};
        for (double rot : {0.0, 0.3}) {
            std::vector<Contour> c2 = cs; for (auto& c : c2) c.transform(Transform2::rotation(rot));
            MedialAxis ax = medialAxis(c2, tol);
            double best = 1e300, rr = 0; for (auto& n : ax.nodes) { double d = dist(n.point, {0, 0}); if (d < best) { best = d; rr = n.radius; } }
            // also any branch point
            for (auto& b : ax.branches) for (auto& p : b.points) { double d = dist(p.point, {0, 0}); if (d < best) { best = d; rr = p.radius; } }
            std::printf("chordal=%.3f angle=%6.1f rot=%.1f nearest-skeleton-to-apex=%.4f (%.2f c) r=%.4f (%.2f c) capped=%d\n", ch, A, rot, best, best / ch, rr, rr / ch, (int)ax.accuracyCapped);
        }
    } }
}
