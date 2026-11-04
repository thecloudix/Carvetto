// Coverage defect near a corner apex: max over interior grid points x (inscribed radius r(x)) of
// min_s(|x-s| - r_s), split by r(x) < c and r(x) >= c.
#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry; using vo::PI; using vo::dist;
struct Seg { Vec2 a, b; double ra, rb; };
static double covAt(const std::vector<Seg>& segs, Vec2 x) {
    double best = 1e300;
    for (auto& s : segs) { double lo = 0, hi = 1; auto f = [&](double t) { return dist(x, s.a + (s.b - s.a) * t) - (s.ra + (s.rb - s.ra) * t); };
        for (int it = 0; it < 50; ++it) { double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3; if (f(m1) < f(m2)) hi = m2; else lo = m1; } best = std::min(best, f(0.5 * (lo + hi))); }
    return best;
}
int main() {
    for (double ch : {0.01, 0.1}) { Tolerance tol; tol.chordal = ch;
    for (double A : {2.0, 10.0, 30.0, 60.0, 90.0, 120.0, 150.0, 170.0, 178.0}) {
        double h = A * PI / 360; Vec2 p1{20 * std::cos(h), -20 * std::sin(h)}, p2{20 * std::cos(h), 20 * std::sin(h)};
        std::vector<Contour> cs{vs::poly({{0, 0}, p1, {p1.x + 20, p1.y}, {p2.x + 20, p2.y}, p2})};
        MedialAxis ax = medialAxis(cs, tol); vo::Boundary B = vo::makeBoundary(cs, ch);
        std::vector<Seg> segs; for (auto& b : ax.branches) for (size_t i = 0; i + 1 < b.points.size(); ++i) segs.push_back({b.points[i].point, b.points[i + 1].point, b.points[i].radius, b.points[i + 1].radius});
        // keep only segments near the apex for speed
        double win = std::max(12.0 * ch, A > 160 ? 12.0 : (A > 140 ? 1.0 : 0.5));
        std::vector<Seg> near; for (auto& s : segs) if (std::min(dist(s.a, {0,0}) - s.ra, dist(s.b, {0,0}) - s.rb) < 3 * win) near.push_back(s);
        double mNarrow = -1e9, mWide = -1e9; Vec2 pw;
        int N = 160;
        for (int i = 0; i <= N; ++i) for (int j = -N; j <= N; ++j) { Vec2 x{win * i / N, win * j / N};
            if (!vo::insideEO(B, x)) continue; double r = vo::exactDist(B, x).d; double c = covAt(near, x);
            if (r < ch) mNarrow = std::max(mNarrow, c); else if (c > mWide) { mWide = c; pw = x; } }
        std::printf("chordal=%.2f angle=%5.1f window=%.3f  maxCovDefect r<c: %.4f (%.2f c)   r>=c: %.4f (%.2f c) at (%.4f,%.4f)\n", ch, A, win, mNarrow, mNarrow / ch, mWide, mWide / ch, pw.x, pw.y);
    } }
}
