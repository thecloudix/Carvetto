// T-0016 verifier: CPU time, branch vs main (namespaced), interleaved. Hatch-like inputs.
// usage: perf <shape> <n> <pitch> <amp> <d> <side 0/1> <mitre> <reps> [joins mask: 1 R 2 Sh 4 Sq] [nomain]
// shapes: zz open zig-zag; zc closed zig-zag (base below); arc zig-zag with arc corners (fillets r=pitch/4);
//         bez zig-zag of Bezier humps; hatch boustrophedon (raster rows width amp, step pitch); saw sawtooth
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstring>
#include <cstdlib>
#include <string>
#include <variant>
#include <algorithm>
#include "carvetto/geometry/offset.hpp"
#include "pw.hpp"
using namespace carvetto::geometry;
static double cpu() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static PContour toP(const Contour& c) { PContour p; p.closed = c.closed; for (auto& s : c.spans) { PSpan q{}; if (auto* l = std::get_if<LineSpan>(&s)) { q.kind = 0; q.a[0] = l->start.x; q.a[1] = l->start.y; q.a[2] = l->end.x; q.a[3] = l->end.y; } else if (auto* a = std::get_if<ArcSpan>(&s)) { q.kind = 1; q.a[0] = a->centre.x; q.a[1] = a->centre.y; q.a[2] = a->radius; q.a[3] = a->startAngle; q.a[4] = a->endAngle; q.a[5] = a->direction == ArcDirection::CCW ? 0 : 1; } else q.kind = 2; p.s.push_back(q); } return p; }
Contour make(const std::string& sh, int n, double p, double a) {
    Contour c; Vec2 prev{0, 0};
    if (sh == "zz" || sh == "zc") {
        for (int i = 1; i <= n; ++i) { Vec2 q{i * p, (i % 2) * a}; c.spans.push_back(LineSpan{prev, q}); prev = q; }
        if (sh == "zc") { Vec2 b1{n * p, -1.0}, b0{0, -1.0}; c.spans.push_back(LineSpan{prev, b1}); c.spans.push_back(LineSpan{b1, b0}); c.spans.push_back(LineSpan{b0, Vec2{0, 0}}); c.closed = true; }
    } else if (sh == "saw") {
        for (int i = 1; i <= n; ++i) { Vec2 q = (i % 2) ? Vec2{(i / 2) * p, a} : Vec2{(i / 2) * p, 0}; c.spans.push_back(LineSpan{prev, q}); prev = q; }
    } else if (sh == "hatch") {  // rows of length a, spaced p: n/2 rows, with short connectors
        for (int i = 0; i < n / 2; ++i) { Vec2 q{(i % 2) ? 0.0 : a, i * p}; c.spans.push_back(LineSpan{prev, q}); prev = q; Vec2 r{q.x, (i + 1) * p}; c.spans.push_back(LineSpan{prev, r}); prev = r; }
    } else if (sh == "bez") {
        for (int i = 1; i <= n; ++i) { Vec2 q{i * p, (i % 2) * a}; Vec2 c1 = prev + (q - prev) * 0.3 + Vec2{p * 0.1, 0}; Vec2 c2 = prev + (q - prev) * 0.7 - Vec2{p * 0.1, 0}; c.spans.push_back(BezierSpan{prev, c1, c2, q}); prev = q; }
    } else if (sh == "arc") {  // zig-zag whose corners are small arcs (radius r) tangent to both lines
        // build the points, then fillet each interior vertex
        std::vector<Vec2> P; for (int i = 0; i <= n; ++i) P.push_back({i * p, (i % 2) * a});
        double r = p * 0.25; Vec2 cur = P[0];
        for (int i = 1; i < n; ++i) {
            Vec2 u = (P[i - 1] - P[i]).normalized(), v = (P[i + 1] - P[i]).normalized();
            double half = std::acos(std::clamp(u.dot(v), -1.0, 1.0)) / 2; double t = r / std::tan(half);
            Vec2 A = P[i] + u * t, B = P[i] + v * t; Vec2 bis = (u + v).normalized(); Vec2 C = P[i] + bis * (r / std::sin(half));
            c.spans.push_back(LineSpan{cur, A});
            double sa = (A - C).angle(), ea = (B - C).angle(); bool ccw = (P[i] - P[i - 1]).cross(P[i + 1] - P[i]) > 0;
            if (ccw) { while (ea < sa) ea += 2 * M_PI; } else { while (ea > sa) ea -= 2 * M_PI; }
            c.spans.push_back(ArcSpan{C, r, sa, ea, ccw ? ArcDirection::CCW : ArcDirection::CW}); cur = B;
        }
        c.spans.push_back(LineSpan{cur, P[n]});
    }
    return c;
}
int main(int argc, char** argv) {
    std::string sh = argv[1]; int n = atoi(argv[2]); double p = atof(argv[3]), a = atof(argv[4]), d = atof(argv[5]); int side = atoi(argv[6]); double mitre = atof(argv[7]); int reps = atoi(argv[8]);
    int mask = argc > 9 ? atoi(argv[9]) : 7; bool nomain = argc > 10;
    Contour c = make(sh, n, p, a); PContour pc = toP(c);
    bool hasBez = false; for (auto& s : c.spans) hasBez |= std::holds_alternative<BezierSpan>(s); if (hasBez) nomain = true;
    const char* jn[] = {"Round", "Sharp", "Square"};
    double tb[3] = {1e9, 1e9, 1e9}, tm[3] = {1e9, 1e9, 1e9}; size_t nb[3] = {0}, nm[3] = {0};
    for (int r = 0; r < reps; ++r)
        for (int j = 0; j < 3; ++j) { if (!(mask >> j & 1)) continue;
            auto runB = [&] { double t0 = cpu(); auto o = offsetOpenContour(c, d, side ? OffsetSide::Right : OffsetSide::Left, (JoinType)j, Tolerance{}, mitre); double t1 = cpu();
                tb[j] = std::min(tb[j], t1 - t0); nb[j] = 0; for (auto& k : o) nb[j] += k.spans.size(); };
            auto runM = [&] { if (!nomain) { double m0 = cpu(); auto mo = mainOffset(pc, d, side, j, 1e-3, mitre); double m1 = cpu(); tm[j] = std::min(tm[j], m1 - m0); nm[j] = 0; for (auto& k : mo) nm[j] += k.s.size(); } };
            const bool mainFirst = std::getenv("MAINFIRST") ? true : (std::getenv("ALT") ? (r % 2 == 1) : false);
            if (mainFirst) { runM(); runB(); } else { runB(); runM(); }
        }
    std::printf("%s n=%d p=%g a=%g d=%g side=%d mitre=%g |", sh.c_str(), n, p, a, d, side, mitre);
    for (int j = 0; j < 3; ++j) if (mask >> j & 1) { std::printf(" %s br %.3f (%zu sp)", jn[j], tb[j], nb[j]); if (!nomain) std::printf(" main %.3f (%zu)", tm[j], nm[j]); std::printf(" |"); }
    if ((mask & 7) == 7) std::printf(" ratio Sh/R %.2f Sq/R %.2f", tb[1] / tb[0], tb[2] / tb[0]);
    std::printf("\n");
}
