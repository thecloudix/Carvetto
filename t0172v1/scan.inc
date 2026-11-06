// fast scan: every case with an AC3 or distance defect, with main's metrics for comparison
int scanMode(long N, std::uint64_t seed0) {
    long n = 0, worse = 0;
    long cnt[8] = {0}, cntM[8] = {0}, cntWorse[8] = {0};
    const char* names[8] = {"coderBack", "shortBack", "gap", "distViol", "roundViol", "overlap", "selfX", "subtolAtEnd/zero"};
    for (long k = 0; k < N; ++k) {
        std::uint64_t seed = seed0 + (std::uint64_t)k;
        Case2 cc = makeCaseX(seed);
        Case& c = cc.c;
        if (c.in.spans.empty()) continue;
        n++;
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m, false, true);
        int st = subTolIn(c);
        int v[8] = {m.coderBack, m.shortBack, m.lineJunctionNotBitwise + m.arcGapBeyond, st ? 0 : m.distViol, st ? 0 : m.roundViol, m.overlapIn + m.overlapX, m.selfX, m.subTolAtEnd + m.zeroLen};
        bool any = false; for (int i = 0; i < 8; ++i) if (v[i]) any = true;
        if (!any) continue;
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, false, true);
        int w[8] = {mm.coderBack, mm.shortBack, -1, st ? 0 : mm.distViol, st ? 0 : mm.roundViol, mm.overlapIn + mm.overlapX, mm.selfX, -1};
        bool isWorse = false;
        for (int i = 0; i < 8; ++i) if (v[i]) { cnt[i]++; if (w[i] > 0) cntM[i]++; if (w[i] == 0) { cntWorse[i]++; isWorse = true; } }
        if (i_dvWorse(m, mm, st)) isWorse = true;
        worse += isWorse;
        std::printf("CASE seed=0x%llx %s%s %s side=%d d=%.6g: br[cb%d sb%d gap%d dv%d(%.3g) rv%d(%.3g) ov%d sx%d se%d] main[cb%d sb%d dv%d(%.3g) rv%d ov%d sx%d] %s | %s\n", (unsigned long long)seed, cc.name, cc.noisy ? "(noisy)" : "", jn(c.join), (int)c.side, c.d,
                    v[0], v[1], v[2], v[3], m.worstUnder, v[4], m.worstOver, v[5], v[6], v[7], w[0], w[1], w[3], mm.worstUnder, w[4], w[5], w[6], isWorse ? "WORSE" : "", m.firstIssue.c_str());
    }
    std::printf("scan N=%ld seed0=0x%llx: cases with defects worse than main %ld\n", n, (unsigned long long)seed0, worse);
    for (int i = 0; i < 8; ++i) std::printf("  %-18s branch cases %ld (main also %ld, main clean %ld)\n", names[i], cnt[i], cntM[i], cntWorse[i]);
    return 0;
}
