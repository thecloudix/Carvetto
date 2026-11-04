// round 4: random search for a contour read on the wrong side by medialAxis without being ignored (the case the
// V-carve net must catch). Outer circle/square + 1..3 holes hugging it (D, lune, band, offset copy) at gaps below the
// classification thresholds + optional islands. Flags: ignored==0 and a skeleton point with radius > 2c+d whose
// inscribed disc is inconsistent with the region (centre outside, or exact distance to the curves < r - 2c - d).
// Crossing / self-crossing inputs are skipped (V-carve resolves them before the medial axis).
// usage: wrongside <seed> <N>
#include <cstdio>
#include <random>
#include "acc.hpp"
#include "shapes.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/intersect.hpp"
using namespace carvetto::geometry;
static std::mt19937_64 R;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(R); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(R); }
static Contour polyc(const std::vector<Vec2>& p) { return vs::poly(p); }
int main(int argc, char** argv) {
    R.seed(strtoull(argv[1], nullptr, 10)); int N = atoi(argv[2]); int flagged = 0, ignoredCases = 0, crossing = 0;
    for (int ci = 0; ci < N; ++ci) {
        double c = std::vector<double>{0.0033, 0.0167, 0.0333, 0.1, 0.333}[I(0, 4)];
        double d = std::min(c, std::vector<double>{1e-6, 1e-5, 1e-4, 1e-3}[I(0, 3)]);
        double Rr = U(3, 12); bool circ = I(0, 1); std::vector<Contour> cs;
        auto wallPt = [&](double a, double inset) { if (circ) return Vec2{(Rr - inset) * std::cos(a), (Rr - inset) * std::sin(a)};
            double t = std::fmod(a / (2 * vo::PI) * 4 + 8, 4.0); double h = Rr - inset; if (t < 1) return Vec2{-h + 2 * h * t, -h}; if (t < 2) return Vec2{h, -h + 2 * h * (t - 1)}; if (t < 3) return Vec2{h - 2 * h * (t - 2), h}; return Vec2{-h, h - 2 * h * (t - 3)}; };
        cs.push_back(circ ? Contour::circle({0, 0}, Rr) : polyc({{-Rr, -Rr}, {Rr, -Rr}, {Rr, Rr}, {-Rr, Rr}}));
        int nh = I(1, 3); double a0 = U(0, 6.28);
        for (int h = 0; h < nh; ++h) {
            double g = std::vector<double>{c / 200, c / 64, c / 20, c / 4, c, 2 * c}[I(0, 5)] * U(0.5, 1.5); if (g < 3 * d) g = 3 * d;
            double sweep = U(0.3, 6.28 / nh - 0.1); double aa = a0 + h * 6.28 / nh; int n = I(16, 300); int kind = I(0, 3);
            std::vector<Vec2> p;
            if (kind == 0) { for (int k = 0; k <= n; ++k) p.push_back(wallPt(aa + sweep * k / n, g)); }  // D (chord)
            else { double t = kind == 1 ? U(2 * d, 3 * c) : kind == 2 ? U(c, 6 * c) : U(0.5, Rr * 0.8);
                for (int k = 0; k <= n; ++k) p.push_back(wallPt(aa + sweep * k / n, g));
                for (int k = n; k >= 0; --k) { double s = kind == 1 ? std::sin(vo::PI * k / n) : 1.0; p.push_back(wallPt(aa + sweep * k / n, g + t * s)); } }
            std::rotate(p.begin(), p.begin() + I(0, (int)p.size() - 1), p.end()); if (I(0, 1)) std::reverse(p.begin(), p.end());
            cs.push_back(polyc(p));
        }
        int ni = I(0, 2); for (int k = 0; k < ni; ++k) { double r = U(0.2, Rr * 0.3); cs.push_back(Contour::circle({U(-0.3, 0.3) * Rr, U(-0.3, 0.3) * Rr}, r)); }
        Tolerance tol; tol.chordal = c; tol.distance = d;
        { bool x = false; for (size_t i = 0; i < cs.size() && !x; ++i) { if (hasSelfIntersections(cs[i], tol)) x = true; for (size_t j = i + 1; j < cs.size() && !x; ++j) if (!intersect(cs[i], cs[j], tol).isEmpty()) x = true; } if (x) { ++crossing; continue; } }
        MedialAxis ax = medialAxis(cs, tol); if (ax.accuracyCapped) continue; if (ax.ignoredContours) { ++ignoredCases; continue; }
        va::Acc A; A.build(cs); double clear = 2 * c + d; double worst = 0; Vec2 wp{}; double wr = 0; int bad = 0;
        for (auto& b : ax.branches) for (auto& m : b.points) { if (m.radius <= clear) continue; bool in = A.inside(m.point); double dd = A.dist(m.point);
            double err = !in ? m.radius + dd : std::max(0.0, m.radius - dd - clear); if (err > 0) { ++bad; if (err > worst) { worst = err; wp = m.point; wr = m.radius; } } }
        if (bad) { ++flagged; std::printf("FLAG case %d c=%g d=%g circ=%d holes=%d islands=%d bad=%d worst=%.4f at (%.4f,%.4f) r=%.4f\n", ci, c, d, (int)circ, nh, ni, bad, worst, wp.x, wp.y, wr);
            if (flagged <= 3) for (size_t i = 0; i < cs.size(); ++i) { std::printf("contour %zu closed=1\n", i); for (auto& sp : cs[i].spans) { if (auto* l = std::get_if<LineSpan>(&sp)) std::printf("    L %.17g %.17g %.17g %.17g\n", l->start.x, l->start.y, l->end.x, l->end.y); else if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("    A %.17g %.17g r %.17g %.17g %.17g dir %d\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, (int)a->direction); } } }
    }
    std::printf("SUMMARY seed=%s N=%d crossing(skipped)=%d flagged=%d ignoredCases=%d\n", argv[1], N, crossing, flagged, ignoredCases);
}
