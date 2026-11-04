// T-0009 round-2 verifier: CPU-time probes. B2 dense sub-tol inputs at 2000/4000/8000 spans (d = 1,
// worst over 3 joins x 2 sides), plus T-0006 AC1 speed inputs. Library under test and main (namespaced).
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <variant>
#include "carvetto/geometry/offset.hpp"
#include "pw.hpp"
using namespace carvetto::geometry;
static double cpu() { return (double)std::clock() / CLOCKS_PER_SEC; }
struct R { std::uint64_t s; double u() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return (double)(s >> 11) * 0x1.0p-53; } };
static PContour toP(const Contour& c) { PContour p; p.closed = c.closed; for (auto& s : c.spans) { PSpan q{}; if (auto* l = std::get_if<LineSpan>(&s)) { q.kind = 0; q.a[0] = l->start.x; q.a[1] = l->start.y; q.a[2] = l->end.x; q.a[3] = l->end.y; } else { auto& a = std::get<ArcSpan>(s); q.kind = 1; q.a[0] = a.centre.x; q.a[1] = a.centre.y; q.a[2] = a.radius; q.a[3] = a.startAngle; q.a[4] = a.endAngle; q.a[5] = a.direction == ArcDirection::CCW ? 0 : 1; } p.s.push_back(q); } return p; }
static bool withMain = false;
static double run(const char* nm, const Contour& c, double d, int n) {
    double worst = 0, worstM = 0; const char* wj = ""; int ws = 0; size_t ch = 0;
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
        for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
            double t0 = cpu(); auto o = offsetOpenContour(c, d, s, j); double t = cpu() - t0;
            if (t > worst) { worst = t; wj = j == JoinType::Round ? "Round" : j == JoinType::Sharp ? "Sharp" : "Square"; ws = (int)s; ch = o.size(); }
            if (withMain) { double m0 = cpu(); auto mo = mainOffset(toP(c), d, (int)s, (int)j, 1e-3, 2.0); worstM = std::max(worstM, cpu() - m0); }
        }
    std::printf("%-34s n=%5d spans %5zu d=%g worst %.3fs (%s side %d, %zu chains)%s", nm, n, c.spans.size(), d, worst, wj, ws, ch, withMain ? "" : "\n");
    if (withMain) std::printf("  main worst %.3fs\n", worstM);
    std::fflush(stdout); return worst;
}
static Contour probe(int which, int n, std::uint64_t seed) {
    R g{seed}; Contour c;
    if (which == 0) { Vec2 p{0, 0}; for (int i = 0; i < n; ++i) { double a = g.u() * 6.283; Vec2 q = p + Vec2{std::cos(a), std::sin(a)} * 1e-7; c.spans.push_back(LineSpan{p, q}); p = q; } c.spans.push_back(LineSpan{p, p + Vec2{5, 1}}); }
    else if (which == 1) { Vec2 p{0, 0}; for (int i = 0; i < n; ++i) { Vec2 q = p + Vec2{2e-4, (i % 2 ? 1 : -1) * 3e-4}; c.spans.push_back(LineSpan{p, q}); p = q; } }
    else if (which == 2) { Vec2 p{0, 0}; double h = 0; for (int i = 0; i < n; ++i) { h += (g.u() - 0.5) * 1.6; Vec2 q = p + Vec2{std::cos(h), std::sin(h)} * 5e-4; c.spans.push_back(LineSpan{p, q}); p = q; } }
    else if (which == 3) { double r = n * 4e-4 / 6.283185307179586; Vec2 p0{r, 0}, pv = p0; for (int i = 1; i <= n; ++i) { double a = 6.283185307179586 * i / n; Vec2 q = i == n ? p0 : Vec2{r * std::cos(a), r * std::sin(a)}; c.spans.push_back(LineSpan{pv, q}); pv = q; } c.closed = true; }
    // extra probes (not in the brief): cluster at a sharp corner between two long lines; pure random micro walk 1e-6
    else if (which == 4) { Vec2 p{-5, 0}; Vec2 v{0, 0}; c.spans.push_back(LineSpan{p, v}); p = v; for (int i = 0; i < n; ++i) { double a = g.u() * 6.283; Vec2 q = p + Vec2{std::cos(a), std::sin(a)} * 2e-7; c.spans.push_back(LineSpan{p, q}); p = q; } c.spans.push_back(LineSpan{p, p + Vec2{3, 4}}); }
    else if (which == 5) { Vec2 p{0, 0}; for (int i = 0; i < n; ++i) { double a = g.u() * 6.283; Vec2 q = p + Vec2{std::cos(a), std::sin(a)} * 9e-4 * g.u(); if (q != p) c.spans.push_back(LineSpan{p, q}); p = q; } }
    return c;
}
static const char* pname[] = {"micro-cluster 1e-7 + line", "zig-zag 2e-4 x 3e-4", "random walk 0.5um", "closed polyline 0.4um", "cluster 2e-7 at sharp corner", "random micro walk <0.9um"};
int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "b2"; withMain = std::getenv("MAIN") != nullptr;
    if (mode == "b2") {
        int lo = argc > 2 ? std::atoi(argv[2]) : 0, hi = argc > 3 ? std::atoi(argv[3]) : 3;
        for (int w = lo; w <= hi; ++w) { double t[3]; int k = 0; for (int n : {2000, 4000, 8000}) t[k++] = run(pname[w], probe(w, n, 12345), 1.0, n);
            std::printf("  ratios 4000/2000 %.2f  8000/4000 %.2f\n", t[1] / std::max(t[0], 1e-3), t[2] / std::max(t[1], 1e-3)); }
        return 0;
    }
    if (mode == "t6") {  // T-0006 AC1
        for (std::uint64_t s : {1ULL, 2ULL, 3ULL, 4ULL}) { R g{s}; Contour c; Vec2 p{0, 0}; double h = 0; for (int i = 0; i < 2000; ++i) { h += (g.u() - 0.5) * 2.0; Vec2 q = p + Vec2{std::cos(h), std::sin(h)} * 0.5; c.spans.push_back(LineSpan{p, q}); p = q; }
            run("walk 2000 x 0.5mm d=1", c, 1.0, 2000); run("walk 2000 x 0.5mm d=2", c, 2.0, 2000); }
        { Contour c; const int n = 8000; double R1 = 5, r1 = 3, dd = 5; Vec2 pv; for (int i = 0; i <= n; ++i) { double t = 6.283185307179586 * 3 * i / n; Vec2 q{(R1 - r1) * std::cos(t) + dd * std::cos((R1 - r1) / r1 * t), (R1 - r1) * std::sin(t) - dd * std::sin((R1 - r1) / r1 * t)}; if (i) c.spans.push_back(LineSpan{pv, q}); pv = q; } run("hypotrochoid 8000 d=2", c, 2.0, n); }
        { Contour c; const int n = 8000; Vec2 pv{0, 0}; R g{99}; for (int i = 0; i < n; ++i) { double t = i * 0.05; Vec2 q{t * 0.08 + 0.6 * std::sin(t * 1.3 + std::sin(t * 0.37)), 0.9 * std::cos(t * 0.9) + 0.1 * g.u()}; if (i) c.spans.push_back(LineSpan{pv, q}); pv = q; } run("text-like loops 8000 d=2", c, 2.0, n); }
        return 0;
    }
}
