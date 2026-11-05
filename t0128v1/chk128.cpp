// T-0128 verifier: quality of the perf128 inputs (comb / hatch / rects / ring), branch vs main: xk + vchk + duplicated length.
// usage: chk128 <case> <N> <d> <w> <join> <side> [rot] [shiftx shifty]
#include "v9lib.cpp"
#include "xchk.hpp"
using carvetto::geometry::testing::distanceToSpanExact;
static double distChains(Vec2 p, const std::vector<Contour>& ch) { double b = 1e300; for (auto& c : ch) for (auto& s : c.spans) b = std::min(b, distanceToSpanExact(p, s)); return b; }
// length of chain within 1e-7 of another chain or a span of the same chain at least 3 spans away (retraced / duplicated)
static double dupLen(const std::vector<Contour>& ch, double tol) {
    double dup = 0;
    for (size_t a = 0; a < ch.size(); ++a) for (size_t i = 0; i < ch[a].spans.size(); ++i) {
        const Span& s = ch[a].spans[i]; double L = length(s); int n = std::max(4, (int)std::ceil(L / (tol / 4)));
        n = std::min(n, 4000);
        int hit = 0;
        for (int k = 0; k <= n; ++k) {
            Vec2 q = pointAt(s, (double)k / n); bool h = false;
            for (size_t b = 0; b < ch.size() && !h; ++b) for (size_t j = 0; j < ch[b].spans.size() && !h; ++j) {
                if (b == a) { long dj = (long)j - (long)i; long ns = (long)ch[a].spans.size(); if (std::labs(dj) <= 2 || (ch[a].closed && ns - std::labs(dj) <= 2)) continue; }
                if (distanceToSpanExact(q, ch[b].spans[j]) <= 1e-7) h = true;
            }
            hit += h;
        }
        dup += L * hit / (n + 1.0);
    }
    return dup;
}
static Contour build(const std::string& k, int N, double d, double w) {
    std::vector<Vec2> P;
    if (k == "zz") { for (int i = 0; i <= N; ++i) P.push_back({i * w, (i % 2) * 1.0}); return fromPts(P, false); }
    if (k == "zz2") { for (int i = 0; i <= N; ++i) P.push_back({i * 0.05, (i % 2) * 0.2}); return fromPts(P, false); }
    if (k == "comb") {  // walls: tooth top y=10, slot bottom y=0; slot width 2d+w
        double x = 0, W = 2 * d + w; P.push_back({x, 10});
        for (int i = 0; i < N; ++i) { P.push_back({x + 1, 10}); P.push_back({x + 1, 0}); P.push_back({x + 1 + W, 0}); P.push_back({x + 1 + W, 10}); x += 1 + W; }
        P.push_back({x + 1, 10}); return fromPts(P, false);
    }
    if (k == "hatch") {  // boustrophedon hatch, N lines 10 long at pitch 2d + w (a raster exactly one tool width apart)
        double W = 2 * d + w; for (int i = 0; i < N; ++i) { double y = i * W; if (i % 2 == 0) { P.push_back({0, y}); P.push_back({10, y}); } else { P.push_back({10, y}); P.push_back({0, y}); } }
        return fromPts(P, false);
    }
    if (k == "rects") {
        double x = 0, H = 2 * d + w, L = 3.6;
        P.push_back({x, 0});
        for (int i = 0; i < N; ++i) { P.push_back({x + L, 0}); P.push_back({x + L, H}); P.push_back({x, H}); P.push_back({x, 0}); P.push_back({x, -5}); x += L + 3 * d + 1; P.push_back({x, -5}); P.push_back({x, 0}); }
        return fromPts(P, false);
    }
    if (k == "ring") {  // serpentine of semicircular channels: arcs of radius r and r + 2d + w alternately (a ring channel exactly one tool wide)
        Contour c; double W = 2 * d + w; double cx = 0;
        for (int i = 0; i < N; ++i) {
            double r0 = 1.0, r1 = r0 + W;
            // outer arc CW over the top from left to right, then down, inner arc back, ...
            c.spans.emplace_back(ArcSpan{{cx, 0}, r1, kPi, 0.0, ArcDirection::CW});
            c.spans.emplace_back(LineSpan{{cx + r1, 0}, {cx + r0, 0}});
            c.spans.emplace_back(ArcSpan{{cx, 0}, r0, 0.0, kPi, ArcDirection::CCW});
            c.spans.emplace_back(LineSpan{{cx - r0, 0}, {cx - r0, -3}});
            c.spans.emplace_back(LineSpan{{cx - r0, -3}, {cx + 2 * r1 + 1 - r1, -3}});
            cx += 2 * r1 + 1;
            c.spans.emplace_back(LineSpan{{cx - r1, -3}, {cx - r1, 0}});
        }
        c.closed = false; return c;
    }
    return Contour{};
}
int main(int argc, char** argv) {
    std::string k = argv[1]; int N = atoi(argv[2]); double d = atof(argv[3]), w = atof(argv[4]); int j = atoi(argv[5]), side = atoi(argv[6]);
    double th = argc > 7 ? atof(argv[7]) : 0; Vec2 sh{argc > 9 ? atof(argv[8]) : 0, argc > 9 ? atof(argv[9]) : 0};
    Contour in = build(k, N, d, w); transformC(in, th, sh); const double tol = 1e-3;
    auto ob = offsetOpenContour(in, d, (OffsetSide)side, (JoinType)j, Tolerance{tol, 1e-9, 0.01}, 2.0);
    auto mp = mainOffset(toP(in), d, side, j, tol, 2.0); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    for (int q = 0; q < 2; ++q) { auto& o = q ? om : ob; Metrics m; check(in, o, d, (OffsetSide)side, (JoinType)j, tol, 2.0, m, true); xk::Res r; xk::Checker(in, o, d, (OffsetSide)side, (JoinType)j, tol, 2.0).run(r);
        double L = 0; for (auto& c : o) for (auto& s : c.spans) L += length(s);
        std::printf("%s %s N=%d d=%g w=%g %s side=%d | chains %zu len %.6g dup %.4g | xk u%.3g o%.3g ex%ld dv%ld rv%ld cm%ld tie%ld thin%ld long%ld miss %.3g | v dv%d rv%d cm%d sx%d ov%d back%d jgap%d ste%d rev%d [%s|%s]\n", q ? "MAIN  " : "BRANCH", k.c_str(), N, d, w, jn((JoinType)j), side,
            o.size(), L, dupLen(o, tol), r.under, r.over, r.excess, r.dv, r.rv, r.cmiss, r.cmissTie, r.cmissThin, r.cmissLong, std::max(r.miss, r.missLen), m.distViol, m.roundViol, m.covMiss, m.selfX, m.overlapIn + m.overlapX, m.coderBack + m.shortBack + m.hardRev, m.lineJunctionNotBitwise + m.arcGapBeyond, m.subTolAtEnd, m.reversals, m.firstIssue.c_str(), r.first.c_str()); }
}
