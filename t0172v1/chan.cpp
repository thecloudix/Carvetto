// T-0128 verifier: channel sweep. A channel 2d + w wide and L long, offset on the inside by d, for U / reversed U /
// arc-ended U / closed rectangle, every join, shifted far from the origin and rotated. Branch vs origin/main (cvmain).
// Per offset: max distance from the channel centre line (sampled 400x) to the chains (must be <= tol), xk + vchk
// metrics for both, duplicated length (chain samples within 1e-7 of another chain or a non-neighbour span of the
// same chain), chain count.
// usage: chan [L=3.6] [verbose=0] [wmode=0: w >= 0 sweep | 1: negative w too | 2: negative only] [d only]
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
int main(int argc, char** argv) {
    const double L = argc > 1 ? atof(argv[1]) : 3.6; const int verbose = argc > 2 ? atoi(argv[2]) : 0; const int wmode = argc > 3 ? atoi(argv[3]) : 0; const double donly = argc > 4 ? atof(argv[4]) : 0;
    const double tol = 1e-3;
    std::vector<double> ws{0.0, 1e-16, 1e-15, 1e-14, 1e-13};
    for (int k = 0; k <= 28; ++k) ws.push_back(1e-12 * std::pow(10.0, k / 4.0));  // 1e-12 .. 1e-5
    for (double x : {3e-9, 4e-7, 4.4e-7, 1e-7, 3e-8, 2e-6, 5e-6}) ws.push_back(x);
    if (wmode == 1) { size_t n = ws.size(); for (size_t i = 1; i < n; ++i) ws.push_back(-ws[i]); } else if (wmode == 2) { for (auto& x : ws) x = -x; }
    const Vec2 shifts[] = {{0, 0}, {12345.678, -5014.09}, {-98765.4321, 31415.9265}};
    const double rots[] = {0.0, 0.3, kPi / 2, 2.1, -0.7853981633974483};
    long n = 0, lostB = 0, lostM = 0, worse = 0, lostBneg = 0, skippedNarrow = 0; double worstCovB = 0, worstDup = 0, worstDupM = 0;
    long sxB = 0, sxM = 0, ovB = 0, ovM = 0, backB = 0, backM = 0, dvB = 0, dvM = 0, rvB = 0, rvM = 0, exB = 0, exM = 0, chB = 0, chM = 0, morech = 0;
    double underB = 0, overB = 0;
    for (double d : {0.1317, 1.0, 3.175}) if (donly == 0 || d == donly) for (int shape = 0; shape < 4; ++shape) for (double w : ws) for (Vec2 sh : shifts) for (double th : rots) for (int j = 0; j < 3; ++j) {
        const double H = 2 * d + w; Contour in; OffsetSide side = OffsetSide::Left;
        std::vector<Vec2> P; Vec2 a{0.05, 0.5 * H}, b{L - d - 0.05, 0.5 * H};
        if (shape == 0) { in = fromPts({{0, 0}, {L, 0}, {L, H}, {0, H}}, false); }
        else if (shape == 1) { in = fromPts({{0, H}, {L, H}, {L, 0}, {0, 0}}, false); side = OffsetSide::Right; }
        else if (shape == 2) { ArcSpan e{{L, 0.5 * H}, 0.5 * H, -0.5 * kPi, 0.5 * kPi, ArcDirection::CCW};
            in.spans.emplace_back(LineSpan{{0, 0}, e.startPoint()}); in.spans.emplace_back(e); in.spans.emplace_back(LineSpan{e.endPoint(), {0, H}}); b = Vec2{L - 0.05, 0.5 * H}; }
        else { in = fromPts({{0, 0}, {L, 0}, {L, H}, {0, H}}, true); a = Vec2{d + 0.05, 0.5 * H}; }
        bool cov = !(shape == 3 && L <= 2 * d + 0.1);
        transformC(in, th, sh);
        auto R = [&](Vec2 p) { return Vec2{std::cos(th) * p.x - std::sin(th) * p.y + sh.x, std::sin(th) * p.x + std::cos(th) * p.y + sh.y}; };
        Vec2 A = R(a), B = R(b);
        JoinType join = (JoinType)j; n++;
        auto ob = offsetOpenContour(in, d, side, join, Tolerance{tol, 1e-9, 0.01}, 2.0);
        auto mp = mainOffset(toP(in), d, (int)side, j, tol, 2.0); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
        double cb = 0, cm = 0;
        if (cov) for (int k = 0; k <= 400; ++k) { Vec2 q = A + (B - A) * (k / 400.0); cb = std::max(cb, distChains(q, ob)); cm = std::max(cm, distChains(q, om)); }
        bool lb = cb > tol, lm = cm > tol;
        if (!cov) skippedNarrow++;
        if (w >= 0) { lostB += lb; } else lostBneg += lb;
        lostM += lm; if (w >= 0 && cov) worstCovB = std::max(worstCovB, cb);
        Metrics mb, mm; check(in, ob, d, side, join, tol, 2.0, mb, true); check(in, om, d, side, join, tol, 2.0, mm, true);
        xk::Res rb, rm; xk::Checker(in, ob, d, side, join, tol, 2.0).run(rb); xk::Checker(in, om, d, side, join, tol, 2.0).run(rm);
        double db = dupLen(ob, tol), dm = dupLen(om, tol); worstDup = std::max(worstDup, db); worstDupM = std::max(worstDupM, dm);
        sxB += mb.selfX; sxM += mm.selfX; ovB += mb.overlapIn + mb.overlapX; ovM += mm.overlapIn + mm.overlapX; backB += mb.coderBack + mb.shortBack + mb.hardRev; backM += mm.coderBack + mm.shortBack + mm.hardRev;
        dvB += rb.dv; dvM += rm.dv; rvB += rb.rv; rvM += rm.rv; exB += rb.excess; exM += rm.excess; chB += ob.size(); chM += om.size();
        underB = std::max(underB, rb.under); overB = std::max(overB, rb.over);
        const double al = xk::allowance(d, tol);
        bool wz = rb.under > std::max(rm.under, al) + 1e-12 || (join == JoinType::Round && rb.over > std::max(rm.over, al) + 1e-12) || rb.dv > rm.dv || rb.rv > rm.rv || rb.excess > rm.excess ||
                  mb.selfX > mm.selfX || (mb.overlapIn + mb.overlapX) > (mm.overlapIn + mm.overlapX) || (mb.coderBack + mb.shortBack + mb.hardRev) > (mm.coderBack + mm.shortBack + mm.hardRev) ||
                  mb.lineJunctionNotBitwise + mb.arcGapBeyond > mm.lineJunctionNotBitwise + mm.arcGapBeyond || mb.subTolAtEnd > mm.subTolAtEnd || mb.zeroLen > mm.zeroLen || (lb && !lm) || db > dm + 2 * tol;
        worse += wz; morech += ob.size() > om.size() + (lm && !lb ? 1 : 0);
        if (verbose >= 2 || (verbose && (wz || (lb && w >= 0 && cov)))) {
            std::printf("%s d=%g shape=%d w=%.3g shift=(%g,%g) th=%.3f %s | br cov %.3g chains %zu dup %.3g u%.3g o%.3g ex%ld dv%ld rv%ld sx%d ov%d back%d jgap%d ste%d [%s|%s] | main cov %.3g chains %zu dup %.3g u%.3g o%.3g ex%ld sx%d ov%d back%d jgap%d\n",
                wz ? "WORSE" : (lb && cov) ? "LOST" : "ok", d, shape, w, sh.x, sh.y, th, jn(join), cb, ob.size(), db, rb.under, rb.over, rb.excess, rb.dv, rb.rv, mb.selfX, mb.overlapIn + mb.overlapX, mb.coderBack + mb.shortBack + mb.hardRev,
                mb.lineJunctionNotBitwise + mb.arcGapBeyond, mb.subTolAtEnd, mb.firstIssue.c_str(), rb.first.c_str(), cm, om.size(), dm, rm.under, rm.over, rm.excess, mm.selfX, mm.overlapIn + mm.overlapX, mm.coderBack + mm.shortBack + mm.hardRev, mm.lineJunctionNotBitwise + mm.arcGapBeyond);
            std::fflush(stdout);
        }
    }
    std::printf("SUMMARY L=%g offsets %ld (no-centre-line rects %ld) | lost w>=0: branch %ld main %ld | lost w<0 branch %ld | worstCov(w>=0) %.3g | worse %ld | chains br %ld main %ld (offsets with more chains %ld) | sx %ld/%ld ov %ld/%ld back %ld/%ld dv %ld/%ld rv %ld/%ld excess %ld/%ld | br worst under %.3g over %.3g | worst dup br %.3g main %.3g\n",
        L, n, skippedNarrow, lostB, lostM, lostBneg, worstCovB, worse, chB, chM, morech, sxB, sxM, ovB, ovM, backB, backM, dvB, dvM, rvB, rvM, exB, exM, underB, overB, worstDup, worstDupM);
}
