// ================================================================ T-0017 verifier: fuzz with refined main comparison
Case2 makeCaseF(std::uint64_t seed, int fam) {
    if (fam == 5) return makeCase5(seed);
    if (fam == 4) return makeCase4(seed);
    if (fam == 3) return makeCase3(seed);
    return makeCase2(seed);
}
// Main's valid points missing in branch, classified.
struct MissCls {
    long miss = 0, corner = 0, loop = 0, other = 0;
    double worstOther = 0; Vec2 atOther{};
    double loopMaxWidth = 0, loopMaxLen = 0;  // over main closed chains with dropped points
    double worstLoopOutside = 0;              // max distance of a dropped-loop point to branch output
};
// max over sample points of the distance to the nearest *other* (non-adjacent) span: the loop's width
double loopWidth(const Contour& k) {
    const std::size_t n = k.spans.size();
    double w = 0;
    for (std::size_t i = 0; i < n; ++i)
        for (int s = 1; s < 32; ++s) {
            Vec2 q = pointAt(k.spans[i], s / 32.0);
            double best = 1e300;
            for (std::size_t j = 0; j < n; ++j) {
                if (j == i) continue;
                double dj = carvetto::geometry::testing::distanceToSpanExact(q, k.spans[j]);
                bool next = j == (i + 1) % n, prev = (j + 1) % n == i;
                if (n > 2 && (next || prev)) {
                    Vec2 J = next ? endPoint(k.spans[i]) : startPoint(k.spans[i]);
                    if (dj >= distance(q, J) * (1 - 1e-9)) continue;  // foot is the shared junction
                }
                if (n == 2) {
                    Vec2 J1 = endPoint(k.spans[i]), J2 = startPoint(k.spans[i]);
                    if (dj >= std::min(distance(q, J1), distance(q, J2)) * (1 - 1e-9)) continue;
                }
                best = std::min(best, dj);
            }
            if (best < 1e299) w = std::max(w, best);
            else w = std::max(w, 1e9);  // nothing across: wide
        }
    return w;
}
MissCls classifyMiss(const Case& c, const std::vector<Contour>& mo, const std::vector<Contour>& out) {
    MissCls r;
    std::vector<Seg> inS, bS;
    for (auto& s : c.in.spans) inS.push_back(toSeg(s));
    for (auto& k : out) for (auto& s : k.spans) bS.push_back(toSeg(s));
    if (bS.empty()) bS.push_back(Seg{0, {1e300, 1e300}, {1e300, 1e300}, {}, 0, 0, 0, Box2{{1e300, 1e300}, {1e300, 1e300}}});
    Grid IG, BG; IG.build(inS, std::max(c.d / 2, 0.05)); BG.build(bS, std::max(c.d / 2, 0.05));
    const double sgn = c.side == OffsetSide::Left ? 1.0 : -1.0;
    const int ni = (int)inS.size();
    const double lim = 2 * c.tol;
    for (auto& k : mo) {
        bool loopHit = false;
        for (auto& s : k.spans) {
            Seg g = toSeg(s);
            int ns = (int)std::clamp(std::ceil(segLen(g) / (0.25 * lim)), 4.0, 400.0);
            for (int q = 0; q <= ns; ++q) {
                Vec2 p = segPoint(g, (double)q / ns);
                if (IG.minDist(p, c.d + 1) < c.d - 1e-7) continue;
                double dd = BG.minDist(p, 10 * lim + 1);
                if (dd <= lim) continue;
                {   // tie points equidistant from two non-adjacent input elements (as crossCover)
                    int kn = -1; double best = 1e300;
                    for (int ii = 0; ii < ni; ++ii) { double d0 = distPt(inS[ii], p); if (d0 < best) { best = d0; kn = ii; } }
                    int e1 = kn > 0 ? kn - 1 : (c.in.closed ? ni - 1 : -1), e2 = kn + 1 < ni ? kn + 1 : (c.in.closed ? 0 : -1);
                    double other = 1e300;
                    for (int ii = 0; ii < ni; ++ii) if (ii != kn && ii != e1 && ii != e2) other = std::min(other, distPt(inS[ii], p));
                    if (best < c.d + 1e-6 && other < c.d + 1e-6) continue;
                }
                r.miss++;
                bool corner = false;
                if (c.join != JoinType::Round)
                    for (int kk = 0; kk < ni && !corner; ++kk) {
                        int nx = kk + 1; if (nx == ni) { if (!c.in.closed) break; nx = 0; }
                        Vec2 t1 = segTan(inS[kk], 1), t2 = segTan(inS[nx], 0);
                        double cr = t1.cross(t2);
                        bool convex = c.side == OffsetSide::Left ? cr < -1e-12 : cr > 1e-12;
                        if (!convex) continue;
                        Vec2 V = inS[kk].b, v = p - V;
                        double L = v.length();
                        if (L > std::max(c.mitre, 1.5) * c.d + 1e-9) continue;
                        double a1 = (t1.perp() * sgn).angle(), sw = std::remainder((t2.perp() * sgn).angle() - a1, kTwoPi);
                        double rel = std::remainder(v.angle() - a1, kTwoPi);
                        double sl = (inS[kk].kind == 1 || inS[nx].kind == 1) ? 0.3 : 1e-6;
                        if (sw >= 0 ? (rel >= -sl && rel <= sw + sl) : (rel <= sl && rel >= sw - sl)) corner = true;
                    }
                if (corner) { r.corner++; continue; }
                if (k.closed) { r.loop++; loopHit = true; r.worstLoopOutside = std::max(r.worstLoopOutside, dd); continue; }
                r.other++;
                if (dd > r.worstOther) { r.worstOther = dd; r.atOther = p; }
            }
        }
        if (loopHit) {
            double len = 0; for (auto& s : k.spans) len += length(s);
            r.loopMaxLen = std::max(r.loopMaxLen, len);
            r.loopMaxWidth = std::max(r.loopMaxWidth, loopWidth(k));
        }
    }
    return r;
}
int suspiciousTips(const Contour& in, const std::vector<Contour>& out, double d, double tol) {
    TipStats ts; ts.printed = 1000;
    tipCheck(in, out, d, tol, ts, "", 0);
    return (int)ts.suspicious;
}
// adjacent chain ends: end of one open chain within tol of the start of another (a split)
int splitJoints(const std::vector<Contour>& out, double tol) {
    int n = 0;
    for (std::size_t i = 0; i < out.size(); ++i) {
        if (out[i].closed || out[i].spans.empty()) continue;
        Vec2 e = out[i].endPoint();
        for (std::size_t j = 0; j < out.size(); ++j) {
            if (j == i || out[j].closed || out[j].spans.empty()) continue;
            if (distance(e, out[j].startPoint()) <= tol) n++;
        }
    }
    return n;
}
int fuzz4(long N, std::uint64_t seed0, int fam) {
    long n = 0, same = 0, differ = 0, regress = 0, printed = 0;
    long sumGaps = 0, sumRev = 0, sumOv = 0, sumSb = 0, sumSusp = 0, sumSuspM = 0, exSusp = 0, sumCoder = 0, exCoder = 0, sumDv = 0, sumRv = 0;
    long sumSplit = 0, sumSplitM = 0, casesLoopDrop = 0, casesCorner = 0, dup = 0;
    double maxLoopW = 0, maxLoopL = 0, worstUnderNoST = 0, worstOverNoST = 0, maxLoopOut = 0;
    long perGen[64] = {0}, perGenReg[64] = {0};
    double cpuB = 0;
    for (long k = 0; k < N; ++k) {
        std::uint64_t seed = seed0 + (std::uint64_t)k;
        Case2 cc = makeCaseF(seed, fam);
        Case& c = cc.c;
        if (c.in.spans.empty()) continue;
        n++; perGen[c.gen]++;
        double t0 = cpuNow();
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        cpuB += cpuNow() - t0;
        Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m, true);
        int st = subTolIn(c);
        int susp = suspiciousTips(c.in, out, c.d, c.tol);
        sumGaps += m.lineJunctionNotBitwise + m.arcGapBeyond; sumRev += m.reversals; sumOv += m.overlapIn + m.overlapX; sumSb += m.shortBack;
        sumSusp += susp; sumCoder += m.coderBack; if (!cc.noisy) { exSusp += susp; exCoder += m.coderBack; }
        sumDv += m.distViol; sumRv += m.roundViol;
        if (!st) { worstUnderNoST = std::max(worstUnderNoST, m.worstUnder); worstOverNoST = std::max(worstOverNoST, m.worstOver); }
        // duplicated chains: two chains equal
        for (std::size_t i = 0; i < out.size(); ++i) for (std::size_t j = i + 1; j < out.size(); ++j) if (out[i] == out[j]) dup++;
        int sp = splitJoints(out, c.tol);
        sumSplit += sp;
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        if (std::getenv("GAPDV") && (m.lineJunctionNotBitwise + m.arcGapBeyond + m.distViol + m.roundViol)) { Metrics m2; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, m2, false, true);
            double ig = 0; for (std::size_t q = 0; q + 1 < c.in.spans.size(); ++q) ig = std::max(ig, distance(endPoint(c.in.spans[q]), startPoint(c.in.spans[q + 1])));
            std::printf("GAPDV %s%s st=%d seed=0x%llx gen=%d %s inGap %.3g | br gaps %d dv %d (%.3g) rv %d (%.3g) | main gaps %d dv %d (%.3g) rv %d (%.3g) | %s\n", cc.name, cc.noisy ? "(noisy)" : "", st, (unsigned long long)seed, c.gen, jn(c.join), ig,
              m.lineJunctionNotBitwise + m.arcGapBeyond, m.distViol, m.worstUnder, m.roundViol, m.worstOver, m2.lineJunctionNotBitwise + m2.arcGapBeyond, m2.distViol, m2.worstUnder, m2.roundViol, m2.worstOver, m.firstIssue.c_str()); }
        if (std::getenv("EXBACK") && (m.coderBack || susp)) { Metrics m2; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, m2, false, false); std::printf("BACK %s%s st=%d seed=0x%llx gen=%d %s coderBack br %d main %d | susp br %d main %d | %s\n", cc.name, cc.noisy ? "(noisy)" : "", st, (unsigned long long)seed, c.gen, jn(c.join), m.coderBack, m2.coderBack, susp, suspiciousTips(c.in, mo, c.d, c.tol), m.firstIssue.c_str()); }
        int spM = splitJoints(mo, c.tol);
        sumSplitM += spM;
        if (mo == out) { same++; continue; }
        differ++;
        Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, true);
        int suspM = suspiciousTips(c.in, mo, c.d, c.tol);
        sumSuspM += suspM;
        MissCls mc = classifyMiss(c, mo, out);
        if (mc.corner) casesCorner++;
        if (mc.loop) { casesLoopDrop++; maxLoopW = std::max(maxLoopW, mc.loopMaxWidth); maxLoopL = std::max(maxLoopL, mc.loopMaxLen); maxLoopOut = std::max(maxLoopOut, mc.worstLoopOutside); }
        std::string why;
        auto W = [&](bool b, const char* s) { if (b) { if (!why.empty()) why += ","; why += s; } };
        W(m.distViol > 0 && m.worstUnder > mm.worstUnder + 1e-12, "under");
        W(m.roundViol > mm.roundViol, "roundOver");
        { const double A2 = std::min(5e-7 * std::max(1.0, c.d), 0.1 * c.tol * c.tol / c.d), sl = 1e-12 * std::max(1.0, c.d) * 1e4;
          W(c.join == JoinType::Round && m.roundViol <= mm.roundViol && m.worstOver > A2 + sl && m.worstOver > mm.worstOver + 1e-12, "roundOverSnapBound"); }
        W(m.covMiss > mm.covMiss && mc.loop == 0, "cov");
        W(m.selfX > mm.selfX, "selfX");
        W(susp > suspM, "susTip");
        W((m.overlapIn + m.overlapX) > (mm.overlapIn + mm.overlapX), "overlap");
        W(m.shortBack > mm.shortBack, "shortBack");
        W(m.lineJunctionNotBitwise + m.arcGapBeyond > mm.lineJunctionNotBitwise + mm.arcGapBeyond, "gap");
        W(m.zeroLen > mm.zeroLen, "zeroLen");
        W(m.subTolAtEnd > mm.subTolAtEnd, "subTolEnd");
        W(m.badSpan > mm.badSpan, "badSpan");
        W(mc.other > 0, "mainPtsMissing");
        W(mc.loop > 0 && mc.loopMaxWidth > c.tol / 8, "wideLoopDropped");
        W(sp > spM && out.size() > mo.size(), "moreSplits");
        if (!why.empty()) {
            regress++; perGenReg[c.gen]++;
            if (printed < 100000) {
                printed++;
                std::printf("REG[%s] %s%s st=%d seed=0x%llx gen=%d n=%zu d=%.9g %s side=%d mitre=%.4g chains b%zu m%zu | br[%s] main[%s] miss other %ld (worst %.3g at %.9g,%.9g) loop %ld (w %.3g L %.3g) corner %ld\n",
                            why.c_str(), cc.name, cc.noisy ? "(noisy)" : "", st, (unsigned long long)seed, c.gen, c.in.spans.size(), c.d, jn(c.join), (int)c.side, c.mitre,
                            out.size(), mo.size(), m.firstIssue.c_str(), mm.firstIssue.c_str(), mc.other, mc.worstOther, mc.atOther.x, mc.atOther.y, mc.loop, mc.loopMaxWidth, mc.loopMaxLen, mc.corner);
                std::fflush(stdout);
            }
        }
    }
    std::printf("fuzz4 fam=%d N=%ld ran %ld seed0=0x%llx: identical %ld differ %ld REGRESSIONS %ld | branch totals: gaps %ld reversals %ld overlaps %ld shortBack %ld suspiciousTips %ld (exact %ld; main in differing cases %ld) coderBack %ld (exact %ld) distViol %ld roundViol %ld dupChains %ld | worstUnder(noST) %.3g worstOver(noST) %.3g | splitJoints br %ld main %ld | cases with dropped main loops %ld (max width %.3g, max len %.3g, max dist to branch %.3g), corner-only diffs %ld | cpu %.1fs\n",
                fam, N, n, (unsigned long long)seed0, same, differ, regress, sumGaps, sumRev, sumOv, sumSb, sumSusp, exSusp, sumSuspM, sumCoder, exCoder, sumDv, sumRv, dup,
                worstUnderNoST, worstOverNoST, sumSplit, sumSplitM, casesLoopDrop, maxLoopW, maxLoopL, maxLoopOut, casesCorner, cpuB);
    for (int gi = 0; gi < 64; ++gi) if (perGen[gi]) std::printf("  gen %d: %ld cases, %ld regressions\n", gi, perGen[gi], perGenReg[gi]);
    return 0;
}
int one4(std::uint64_t seed, int fam) {
    Case2 cc = makeCaseF(seed, fam);
    Case& c = cc.c;
    if (std::getenv("SHIFTX") || std::getenv("SHIFTY")) transformC(c.in, 0.0, Vec2{std::getenv("SHIFTX") ? std::atof(std::getenv("SHIFTX")) : 0.0, std::getenv("SHIFTY") ? std::atof(std::getenv("SHIFTY")) : 0.0});
    auto dump = [](const Contour& k) {
        for (auto& s : k.spans) {
            if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s));
            else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s));
        }
    };
    std::printf("%s closed=%d d=%.17g %s side=%d mitre=%.17g\n", cc.name, c.in.closed, c.d, jn(c.join), (int)c.side, c.mitre);
    if (!std::getenv("NOIN")) dump(c.in);
    for (std::size_t i = 0; i + 1 < c.in.spans.size(); ++i) { double gq = distance(endPoint(c.in.spans[i]), startPoint(c.in.spans[i + 1])); if (gq > 0) std::printf("input gap after span %zu: %.3g\n", i, gq); }
    auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    for (std::size_t i = 0; i < out.size(); ++i) { std::printf("branch chain %zu closed=%d\n", i, out[i].closed); dump(out[i]); }
    Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
    std::printf("branch: %s [coderBack %d rev %d ov %d/%d dv %d (under %.3g) rv %d (over %.3g) sx %d cm %d susp %d]\n", m.firstIssue.c_str(), m.coderBack, m.reversals, m.overlapIn, m.overlapX, m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.selfX, m.covMiss, suspiciousTips(c.in, out, c.d, c.tol));
    auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
    for (std::size_t i = 0; i < mo.size(); ++i) { std::printf("main chain %zu closed=%d\n", i, mo[i].closed); dump(mo[i]); }
    Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm);
    std::printf("main: %s [coderBack %d rev %d ov %d/%d dv %d (under %.3g) rv %d (over %.3g) sx %d cm %d susp %d]\n", mm.firstIssue.c_str(), mm.coderBack, mm.reversals, mm.overlapIn, mm.overlapX, mm.distViol, mm.worstUnder, mm.roundViol, mm.worstOver, mm.selfX, mm.covMiss, suspiciousTips(c.in, mo, c.d, c.tol));
    MissCls mc = classifyMiss(c, mo, out);
    std::printf("main pts missing in branch: %ld (corner %ld loop %ld [w %.3g L %.3g dist %.3g] other %ld worst %.3g at (%.12g,%.12g))\n", mc.miss, mc.corner, mc.loop, mc.loopMaxWidth, mc.loopMaxLen, mc.worstLoopOutside, mc.other, mc.worstOther, mc.atOther.x, mc.atOther.y);
    return 0;
}
