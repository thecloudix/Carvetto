// T-0017 round-3 verifier: B-2 occupancy rule probe (needs offset_occ.o linked).
extern "C" { extern int cv_occ_hits; extern int cv_occ_off; }
int occMode(long N, std::uint64_t seed0, int fam) {
    long n = 0, hitCases = 0, worseMain = 0, worseOff = 0, betterOff = 0, identOff = 0;
    for (long k = 0; k < N; ++k) {
        std::uint64_t seed = seed0 + (std::uint64_t)k;
        Case2 cc = makeCaseF(seed, fam);
        Case& c = cc.c;
        if (c.in.spans.empty()) continue;
        if (std::getenv("SHIFTX") || std::getenv("SHIFTY")) transformC(c.in, 0.0, Vec2{std::getenv("SHIFTX") ? std::atof(std::getenv("SHIFTX")) : 0.0, std::getenv("SHIFTY") ? std::atof(std::getenv("SHIFTY")) : 0.0});
        n++;
        cv_occ_hits = 0; cv_occ_off = 0;
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        int hits = cv_occ_hits;
        if (!hits) continue;
        hitCases++;
        cv_occ_off = 1;
        auto off = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        cv_occ_off = 0;
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        Metrics m, mf, mm; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m); check(c.in, off, c.d, c.side, c.join, c.tol, c.mitre, mf); check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm);
        MissCls mcM = classifyMiss(c, mo, out), mcF = classifyMiss(c, off, out);
        auto gaps = [](const Metrics& x) { return x.lineJunctionNotBitwise + x.arcGapBeyond; };
        auto bad = [&](const Metrics& x) { return gaps(x) + x.coderBack + x.distViol + x.roundViol + x.selfX + x.overlapIn + x.overlapX + x.covMiss + x.subTolAtEnd + x.zeroLen; };
        int sp = splitJoints(out, c.tol), spM = splitJoints(mo, c.tol), spF = splitJoints(off, c.tol);
        bool wM = gaps(m) > gaps(mm) || m.coderBack > mm.coderBack || (m.distViol && m.worstUnder > mm.worstUnder + 1e-12) || m.roundViol > mm.roundViol || m.selfX > mm.selfX || (m.overlapIn + m.overlapX) > (mm.overlapIn + mm.overlapX) || (m.covMiss > mm.covMiss && mcM.loop == 0) || mcM.other > 0 || (sp > spM && out.size() > mo.size());
        if (wM) worseMain++;
        if (out == off) identOff++;
        else if (bad(m) > bad(mf) || mcF.other > 0 || sp > spF) worseOff++;
        else if (bad(m) < bad(mf)) betterOff++;
        std::printf("OCC fam=%d %s seed=0x%llx %s d=%.6g hits %d | chains br %zu off %zu main %zu | br[g%d cb%d dv%d(%.2g) rv%d sx%d ov%d cm%d split%d] off[g%d cb%d dv%d sx%d ov%d cm%d split%d] main[g%d cb%d dv%d(%.2g) rv%d sx%d ov%d cm%d split%d] | mainPtsMissing other %ld (%.3g) offPtsMissing other %ld (%.3g) %s%s\n",
                    fam, cc.name, (unsigned long long)seed, jn(c.join), c.d, hits, out.size(), off.size(), mo.size(),
                    gaps(m), m.coderBack, m.distViol, m.worstUnder, m.roundViol, m.selfX, m.overlapIn + m.overlapX, m.covMiss, sp,
                    gaps(mf), mf.coderBack, mf.distViol, mf.selfX, mf.overlapIn + mf.overlapX, mf.covMiss, spF,
                    gaps(mm), mm.coderBack, mm.distViol, mm.worstUnder, mm.roundViol, mm.selfX, mm.overlapIn + mm.overlapX, mm.covMiss, spM,
                    mcM.other, mcM.worstOther, mcF.other, mcF.worstOther, wM ? "WORSE_THAN_MAIN " : "", m.firstIssue.c_str());
        std::fflush(stdout);
    }
    std::printf("occ fam=%d N=%ld seed0=0x%llx ran %ld: cases with occupancy refusals %ld, worse than main %ld, identical to occ-off %ld, worse than occ-off %ld, better than occ-off %ld\n", fam, N, (unsigned long long)seed0, n, hitCases, worseMain, identOff, worseOff, betterOff);
    return 0;
}
