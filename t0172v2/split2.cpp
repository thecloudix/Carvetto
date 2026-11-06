// T-0172 verifier (a): channel exactly 2d wide whose walls are polylines of N segments, a fraction `ft` of them sub-tolerance
// (length tol*[0.2,0.9]); prints for branch and main: chains lying on the middle line, their span count, total length, and the
// largest uncovered stretch of the middle (sampled every tol/4 from x0+0.05 to x1-0.05, distance > tol/2 to any chain).
// args: N d join ft seed [shift] [closed]
#include "v9lib.cpp"


static std::vector<Contour> fromPcs(const std::vector<PContour>& v) { std::vector<Contour> o; for (auto& p : v) { Contour c; c.closed = p.closed; for (auto& q : p.s) { if (q.kind == 0) c.spans.push_back(LineSpan{{q.a[0], q.a[1]}, {q.a[2], q.a[3]}}); else c.spans.push_back(ArcSpan{{q.a[0], q.a[1]}, q.a[2], q.a[3], q.a[4], q.a[5] == 0 ? ArcDirection::CCW : ArcDirection::CW}); } o.push_back(c);} return o; }
int main(int argc, char** argv) {
    int N = atoi(argv[1]); double d = atof(argv[2]); int j = atoi(argv[3]); double ft = atof(argv[4]); Rng g(strtoull(argv[5], 0, 0));
    double sh = argc > 6 ? atof(argv[6]) : 0; int closed = argc > 7 ? atoi(argv[7]) : 0;
    const double tol = 1e-3;
    std::vector<double> xs{0};
    for (int i = 0; i < N; ++i) xs.push_back(xs.back() + (g.uni() < ft ? tol * g.range(0.2, 0.9) : 1.0));
    std::vector<Vec2> P; double L = xs.back();
    for (double x : xs) P.push_back({sh + x, sh});
    if (closed) { P.push_back({sh + L + 2 * d, sh}); P.push_back({sh + L + 2 * d, sh + 2 * d}); }
    for (int i = N; i >= 0; --i) P.push_back({sh + xs[i], sh + 2 * d});
    if (closed) { P.push_back({sh - 2 * d, sh + 2 * d}); P.push_back({sh - 2 * d, sh}); }
    Contour in = fromPts(P, closed == 1);
    auto ob = offsetOpenContour(in, d, OffsetSide::Left, (JoinType)j, Tolerance{tol, 1e-9, 0.01}, 2.0);
    auto om = fromPcs(mainOffset(toP(in), d, 0, j, tol, 2.0));
    auto rep = [&](const char* nm, const std::vector<Contour>& o) {
        int mid = 0; size_t sp = 0, sub = 0; double len = 0; int nonexact = 0;
        for (auto& c : o) { bool on = true; for (auto& s : c.spans) on = on && std::fabs(startPoint(s).y - sh - d) < 1e-9 && std::fabs(endPoint(s).y - sh - d) < 1e-9;
            if (!on) continue; ++mid; sp += c.spans.size(); for (size_t k = 0; k < c.spans.size(); ++k) { len += length(c.spans[k]); if (length(c.spans[k]) <= tol) ++sub; if (k && !(endPoint(c.spans[k-1]) == startPoint(c.spans[k]))) ++nonexact; } }
        double worst = 0, run = 0; Vec2 prev{};
        for (double x = 0.05; x <= L - (closed ? 0.0 : d) - 0.05; x += tol / 4) { Vec2 q{sh + x, sh + d}; double b = 1e300; for (auto& c : o) for (auto& s : c.spans) { auto bx = bounds(s); if (q.x < bx.min.x - 1 || q.x > bx.max.x + 1) continue; b = std::min(b, distPt(toSeg(s), q)); }
            if (b > tol / 2) { run += tol / 4; worst = std::max(worst, run); } else run = 0; (void)prev; }
        std::printf("  %s: chains %zu, on middle %d (spans %zu, of them <= tol %zu, length %.6f, non-exact junctions %d), largest uncovered %.4g\n", nm, o.size(), mid, sp, sub, len, nonexact, worst);
    };
    std::printf("N=%d d=%g %s ft=%g L=%.4f shift=%g %s\n", N, d, jn((JoinType)j), ft, L, sh, closed ? "closed" : "U");
    rep("branch", ob); rep("main  ", om);
}
