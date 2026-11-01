// T-0017 round-3 verifier: Round cusp bridge probe.
std::vector<Vec2> cuspBase(int which, double d, double g) {
    if (which == 0) {
        static const std::vector<Vec2> P = {{6.6283223938981575, -1.2779953526139643e-11},  {5.023667459767422, 2.1623209668586711e-11},
         {6.9756096886500769, 34.51051481312367},        {6.3189901091486398, 34.312480314617368},
         {4.6667279880904458, 2.0657439862650513},       {0.69533962297162577, 2.0657439862566656},
         {1.9562893692302896, 33.119825834028326},       {7.04511858784476, 33.809684447840162},
         {6.6283223939411231, 2.0657439862576599},       {5.0236674597854529, 2.0657439862438181},
         {5.4657050614391105, 36.723239966815832},       {4.5703929732694029, 36.057839455949605},
         {4.8076418394176894, 3.1120034057046923e-12},   {8.8588515885397729, -8.7094539924375882e-13},
         {9.7907278097679189, -35.00186354855979},       {1.9705389447012767, -34.43692474497638},
         {0.69533962299185814, -2.6581188530012263e-11}, {4.6667279880936041, -3.0346127572489825e-11},
         {3.1154143986946496, 34.81196163001529},        {4.5941198790408642, 35.289788649021553},
         {5.0236674597518789, 1.9744273286251705e-11},   {6.6283223939582836, -1.3296097279548633e-11},
         {6.980860505868165, 32.133493994828015},        {9.603450489764624, 31.205313249352219},
         {8.858851588581409, 2.0657439862506815},        {4.8076418393909703, 2.0657439862946823}};
        std::vector<Vec2> Q;
        for (Vec2 p : P) Q.push_back(std::fabs(p.y - 2 * d) < 1e-6 ? Vec2{p.x, 2 * d + g} : p);
        return Q;
    }
    // synthetic: apex of a Lambda at origin, apex of a V at (0, 2d+g), joined round the right (a C shape)
    const double s = which == 1 ? 3.0 : 0.7, H = 2 * d + g;
    const double sk = which == 3 ? 0.37 : 0.0;  // skew the V apex sideways too (arcs meet off-axis)
    return {{-s, -s}, {0, 0}, {s, -s}, {2 * s + 2 * d, 0}, {2 * s + 2 * d, H}, {s + sk, H + s}, {sk, H}, {-s + sk, H + s}};
}
int cuspMode(int which, double d, int nG, double gmax, int logScale, int both) {
    const Vec2 shifts[] = {{0, 0}, {1e2, -1e2}, {1e3, 1e3}, {-1e4, 1e4}, {1e4, 1e4}, {12345.678, -9876.54321}};
    long cases = 0, worse = 0, bGaps = 0, mGaps = 0, bSx = 0, mSx = 0, bCB = 0, mCB = 0, bDV = 0, mDV = 0, bRV = 0, mRV = 0, bOv = 0, mOv = 0, bRev = 0, mRev = 0, bridges = 0;
    double bUnder = 0, mUnder = 0, bOver = 0, mOver = 0, maxBridge = 0;
    for (int gi = 0; gi <= nG; ++gi) {
        double g = logScale ? (gi == 0 ? 0.0 : gmax * std::pow(10.0, -6.0 * (nG - gi) / nG)) : gmax * gi / nG;
        for (int sgn = 0; sgn < (both ? 2 : 1); ++sgn) {
            double gg = sgn ? -g : g;
            if (sgn && g == 0) continue;
            for (Vec2 sh : shifts) {
                for (int side = 0; side < 2; ++side) {
                    std::vector<Vec2> pts = cuspBase(which, d, gg);
                    Contour in = fromPts(pts, false);
                    transformC(in, 0.0, sh);
                    OffsetSide sd = side ? OffsetSide::Right : OffsetSide::Left;
                    auto out = offsetOpenContour(in, d, sd, JoinType::Round, Tolerance{0.001, 1e-9, 0.01}, 3.8787999757578069);
                    Metrics m; check(in, out, d, sd, JoinType::Round, 0.001, 3.8787999757578069, m, false);
                    auto mp = mainOffset(toP(in), d, side, 0, 0.001, 3.8787999757578069);
                    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
                    Metrics mm; check(in, mo, d, sd, JoinType::Round, 0.001, 3.8787999757578069, mm, false);
                    cases++;
                    for (auto& c : out) for (auto& s : c.spans) if (std::holds_alternative<LineSpan>(s) && length(s) < 1e-6) { bridges++; maxBridge = std::max(maxBridge, length(s)); }
                    int bg = m.lineJunctionNotBitwise + m.arcGapBeyond, mg = mm.lineJunctionNotBitwise + mm.arcGapBeyond;
                    bGaps += bg; mGaps += mg; bSx += m.selfX; mSx += mm.selfX; bCB += m.coderBack; mCB += mm.coderBack; bDV += m.distViol; mDV += mm.distViol; bRV += m.roundViol; mRV += mm.roundViol;
                    bOv += m.overlapIn + m.overlapX; mOv += mm.overlapIn + mm.overlapX; bRev += m.reversals; mRev += mm.reversals;
                    bUnder = std::max(bUnder, m.worstUnder); mUnder = std::max(mUnder, mm.worstUnder); bOver = std::max(bOver, m.worstOver); mOver = std::max(mOver, mm.worstOver);
                    bool w = bg > mg || m.selfX > mm.selfX || m.coderBack > mm.coderBack || m.distViol > mm.distViol || m.roundViol > mm.roundViol || (m.overlapIn + m.overlapX) > (mm.overlapIn + mm.overlapX) || bg > 0 || m.coderBack > 0 || m.distViol > 0 || m.roundViol > 0;
                    if (w) { worse++; if (worse <= 40) std::printf("FLAG which=%d g=%.4g shift=(%g,%g) side=%d | br chains %d gaps %d sx %d cb %d dv %d (%.3g) rv %d (%.3g) ov %d rev %d [%s] | main chains %d gaps %d sx %d cb %d dv %d rv %d ov %d rev %d [%s]\n", which, gg, sh.x, sh.y, side, m.chains, bg, m.selfX, m.coderBack, m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.overlapIn + m.overlapX, m.reversals, m.firstIssue.c_str(), mm.chains, mg, mm.selfX, mm.coderBack, mm.distViol, mm.roundViol, mm.overlapIn + mm.overlapX, mm.reversals, mm.firstIssue.c_str()); }
                }
            }
        }
    }
    std::printf("cusp which=%d d=%g nG=%d gmax=%g log=%d neg=%d: cases %ld flagged %ld | branch gaps %ld sx %ld coderBack %ld dv %ld (worstUnder %.3g) rv %ld (worstOver %.3g) ov %ld rev %ld bridges<1e-6 %ld (max len %.3g) | main gaps %ld sx %ld coderBack %ld dv %ld (%.3g) rv %ld (%.3g) ov %ld rev %ld | allowance %.3g\n",
                which, d, nG, gmax, logScale, both, cases, worse, bGaps, bSx, bCB, bDV, bUnder, bRV, bOver, bOv, bRev, bridges, maxBridge, mGaps, mSx, mCB, mDV, mUnder, mRV, mOver, mOv, mRev, allowance(d, 0.001));
    return 0;
}
int cuspOne(int which, double d, double g, double sx, double sy, int side) {
    Contour in = fromPts(cuspBase(which, d, g), false);
    transformC(in, 0.0, Vec2{sx, sy});
    OffsetSide sd = side ? OffsetSide::Right : OffsetSide::Left;
    auto out = offsetOpenContour(in, d, sd, JoinType::Round, Tolerance{0.001, 1e-9, 0.01}, 3.8787999757578069);
    auto dump = [](const Contour& k) {
        for (auto& s : k.spans) {
            if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s));
            else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s));
        }
    };
    for (std::size_t i = 0; i < out.size(); ++i) { std::printf("branch chain %zu closed=%d\n", i, out[i].closed); dump(out[i]); }
    Metrics m; check(in, out, d, sd, JoinType::Round, 0.001, 3.8787999757578069, m);
    std::printf("branch: %s [cb %d rev %d ov %d/%d dv %d (%.3g) rv %d (%.3g) sx %d sxNJ %d cm %d]\n", m.firstIssue.c_str(), m.coderBack, m.reversals, m.overlapIn, m.overlapX, m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.selfX, m.selfXNearJ, m.covMiss);
    auto mp = mainOffset(toP(in), d, side, 0, 0.001, 3.8787999757578069);
    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
    if (std::getenv("DUMPMAIN")) for (std::size_t i = 0; i < mo.size(); ++i) { std::printf("main chain %zu closed=%d\n", i, mo[i].closed); dump(mo[i]); }
    Metrics mm; check(in, mo, d, sd, JoinType::Round, 0.001, 3.8787999757578069, mm);
    std::printf("main: %s [cb %d rev %d ov %d/%d dv %d (%.3g) rv %d (%.3g) sx %d sxNJ %d cm %d]\n", mm.firstIssue.c_str(), mm.coderBack, mm.reversals, mm.overlapIn, mm.overlapX, mm.distViol, mm.worstUnder, mm.roundViol, mm.worstOver, mm.selfX, mm.selfXNearJ, mm.covMiss);
    return 0;
}
int gapScan(long N, std::uint64_t seed0, int fam) {
    long n = 0, g = 0;
    for (long k = 0; k < N; ++k) {
        std::uint64_t seed = seed0 + (std::uint64_t)k;
        Case2 cc = makeCaseF(seed, fam);
        Case& c = cc.c;
        if (c.in.spans.empty()) continue;
        n++;
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m, false, false);
        int gg = m.lineJunctionNotBitwise + m.arcGapBeyond;
        if (gg) {
            g++;
            double ig = 0; for (std::size_t q = 0; q + 1 < c.in.spans.size(); ++q) ig = std::max(ig, distance(endPoint(c.in.spans[q]), startPoint(c.in.spans[q + 1])));
            if (c.in.closed && c.in.spans.size() > 1) ig = std::max(ig, distance(endPoint(c.in.spans.back()), startPoint(c.in.spans.front())));
            std::printf("GAP fam=%d %s%s seed=0x%llx gen=%d %s inGap %.3g | %s\n", fam, cc.name, cc.noisy ? "(noisy)" : "", (unsigned long long)seed, c.gen, jn(c.join), ig, m.firstIssue.c_str());
        }
    }
    std::printf("gapScan fam=%d N=%ld seed0=0x%llx ran %ld gaps-cases %ld\n", fam, N, (unsigned long long)seed0, n, g);
    return 0;
}
