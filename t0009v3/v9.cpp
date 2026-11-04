// T-0009 verifier driver (independent injector; checker = vdrv4 with corrections 1-3 reviewed,
// sliver-tie misses (other input span within d + tol/2) counted apart and dumped for review).
#define main vdrv4_main
#include "vchk.cpp"
#undef main
#include <map>

static double tinyL(Rng& g, double tol) {
    double u = g.uni();
    if (u < 0.35) return tol * std::pow(10.0, -g.range(0.0, 12.0));   // 1e-15..tol log
    if (u < 0.45) return tol * g.range(0.9, 1.0);                     // just under tol
    if (u < 0.5) return tol;                                            // exactly tol
    return tol * g.range(1e-3, 1.0);
}
static Vec2 tanEnd(const Span& s) { return tangentAt(s, 1.0); }
static int g_bez = 0;
// Insert sub-tol features at vertices / along spans. Keeps exact continuity where possible.
static Contour inject9(const Contour& in, Rng& g, double tol, double prob) {
    Contour o; o.closed = in.closed;
    const std::size_t n = in.spans.size();
    auto pushLineSplit = [&](const LineSpan& l, int kind) {
        Vec2 A = l.start, B = l.end; double L = distance(A, B);
        Vec2 dir = (B - A) * (1.0 / L);
        double tl = tinyL(g, tol);
        if (L <= 10 * tol) { o.spans.push_back(l); return; }
        switch (kind) {
            case 0: {  // jog somewhere along the line
                double t = g.range(0.05, 0.95); Vec2 P = A + (B - A) * t;
                Vec2 Q = P + Vec2::fromAngle(dir.angle() + g.range(-kPi, kPi), tl);
                o.spans.push_back(LineSpan{A, P}); o.spans.push_back(LineSpan{P, Q}); o.spans.push_back(LineSpan{Q, B});
                break; }
            case 1: {  // tiny spike (exact reversal) at a point
                double t = g.range(0.05, 0.95); Vec2 P = A + (B - A) * t;
                Vec2 Q = P + Vec2::fromAngle(dir.angle() + g.range(-kPi, kPi), tl);
                o.spans.push_back(LineSpan{A, P}); o.spans.push_back(LineSpan{P, Q}); o.spans.push_back(LineSpan{Q, P}); o.spans.push_back(LineSpan{P, B});
                break; }
            case 2: {  // tiny triangle loop returning exactly to P
                double t = g.range(0.05, 0.95); Vec2 P = A + (B - A) * t;
                Vec2 Q = P + Vec2::fromAngle(g.range(-kPi, kPi), tl), R = P + Vec2::fromAngle(g.range(-kPi, kPi), tinyL(g, tol));
                o.spans.push_back(LineSpan{A, P}); o.spans.push_back(LineSpan{P, Q}); o.spans.push_back(LineSpan{Q, R}); o.spans.push_back(LineSpan{R, P}); o.spans.push_back(LineSpan{P, B});
                break; }
            case 3: {  // tiny arc in the middle (any sweep, possibly tiny radius)
                double t = g.range(0.05, 0.95); Vec2 P = A + (B - A) * t;
                double sw = g.range(-6.2, 6.2); if (std::fabs(sw) < 1e-4) sw = 1e-4;
                double r = tl / std::fabs(sw);
                double h = dir.angle();
                Vec2 left{-std::sin(h), std::cos(h)};
                Vec2 ctr = sw > 0 ? P + left * r : P - left * r;
                ArcSpan a{ctr, r, (P - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW};
                a.endAngle = a.startAngle + sw;
                Vec2 s0 = a.startPoint(), e0 = a.endPoint();
                o.spans.push_back(LineSpan{A, s0}); o.spans.push_back(a);
                if (distance(e0, B) > 0) o.spans.push_back(LineSpan{e0, B});
                break; }
            case 4: {  // tiny Bezier inserted, polygonised by the library
                double t = g.range(0.05, 0.95); Vec2 P = A + (B - A) * t;
                Vec2 Q = P + Vec2::fromAngle(g.range(-kPi, kPi), tl);
                BezierSpan bz{P, P + Vec2::fromAngle(g.range(-kPi, kPi), tl * g.range(0, 2)), Q + Vec2::fromAngle(g.range(-kPi, kPi), tl * g.range(0, 2)), Q};
                o.spans.push_back(LineSpan{A, P}); o.spans.push_back(bz); o.spans.push_back(LineSpan{Q, B}); g_bez++;
                break; }
            case 5: {  // the line becomes a Bezier (normal size) with a cusp-ish control: tiny polygon segments near ends
                Vec2 c1 = A + dir * (tl * g.range(0, 1)) + dir.perp() * (tl * g.range(-1, 1));
                Vec2 c2 = B - dir * (L * g.range(0.1, 0.6)) + dir.perp() * (L * g.range(-0.5, 0.5));
                o.spans.push_back(BezierSpan{A, c1, c2, B}); g_bez++;
                break; }
            case 6: {  // zero-length span
                double t = g.range(0.05, 0.95); Vec2 P = A + (B - A) * t;
                o.spans.push_back(LineSpan{A, P}); o.spans.push_back(LineSpan{P, P}); o.spans.push_back(LineSpan{P, B});
                break; }
            default: {  // run of 2..8 tiny lines at the end
                int k = g.irange(2, 8);
                Vec2 p = B - dir * (tl * k);
                o.spans.push_back(LineSpan{A, p});
                for (int q = 0; q < k; ++q) {
                    Vec2 e = q + 1 == k ? B : p + Vec2::fromAngle(dir.angle() + g.range(-kPi, kPi), tinyL(g, tol));
                    if (distance(p, e) > 0) o.spans.push_back(LineSpan{p, e});
                    p = e;
                }
            }
        }
    };
    for (std::size_t i = 0; i < n; ++i) {
        const Span& sp = in.spans[i];
        if (g.uni() >= prob) { o.spans.push_back(sp); continue; }
        if (const auto* l = std::get_if<LineSpan>(&sp)) {
            pushLineSplit(*l, g.irange(0, 7));
        } else if (const auto* a = std::get_if<ArcSpan>(&sp)) {
            double len = length(sp);
            if (len <= 10 * tol) { o.spans.push_back(sp); continue; }
            double sw = a->sweepAngle();
            double tl = tinyL(g, tol);
            int kind = g.irange(0, 2);
            if (kind == 0) {  // tiny piece split off the arc somewhere
                double f = g.range(0.0, 1.0);
                double cut1 = a->startAngle + sw * f;
                double cut2 = cut1 + tl / a->radius * (sw > 0 ? 1 : -1);
                if (std::fabs(cut2 - a->startAngle) >= std::fabs(sw)) { o.spans.push_back(sp); continue; }
                ArcSpan a1 = *a, a2 = *a, a3 = *a;
                a1.endAngle = cut1; a2.startAngle = cut1; a2.endAngle = cut2; a3.startAngle = cut2;
                if (f > 0) o.spans.push_back(a1);
                o.spans.push_back(a2); o.spans.push_back(a3);
            } else if (kind == 1) {  // tiny spike at the arc's end
                Vec2 E = a->endPoint();
                Vec2 Q = E + Vec2::fromAngle(g.range(-kPi, kPi), tl);
                o.spans.push_back(sp); o.spans.push_back(LineSpan{E, Q}); o.spans.push_back(LineSpan{Q, E});
            } else {  // tiny different-radius arc tangent at the end
                Vec2 E = a->endPoint(); Vec2 t = tanEnd(sp);
                double sw2 = g.range(-3.0, 3.0); if (std::fabs(sw2) < 1e-3) sw2 = 1e-3;
                double r = tl / std::fabs(sw2);
                Vec2 left = t.perp();
                Vec2 ctr = sw2 > 0 ? E + left * r : E - left * r;
                ArcSpan b{ctr, r, (E - ctr).angle(), 0, sw2 > 0 ? ArcDirection::CCW : ArcDirection::CW};
                b.endAngle = b.startAngle + sw2;
                o.spans.push_back(sp); o.spans.push_back(b);
                // reconnect to the next span by a line from b's end to E (tiny)
                Vec2 be = b.endPoint();
                if (distance(be, E) > 0) o.spans.push_back(LineSpan{be, E});
            }
        } else o.spans.push_back(sp);
    }
    // ends
    if (!o.closed && !o.spans.empty() && g.uni() < 0.25) {
        Vec2 E = endPoint(o.spans.back());
        o.spans.push_back(LineSpan{E, E + Vec2::fromAngle(g.range(-kPi, kPi), tinyL(g, tol))});
    }
    if (!o.closed && !o.spans.empty() && g.uni() < 0.25) {
        Vec2 S = startPoint(o.spans.front());
        o.spans.insert(o.spans.begin(), LineSpan{S + Vec2::fromAngle(g.range(-kPi, kPi), tinyL(g, tol)), S});
    }
    return o;
}
static bool hasBez(const Contour& c) { for (auto& s : c.spans) if (std::holds_alternative<BezierSpan>(s)) return true; return false; }
static int subTolN(const Contour& c, double tol) { int k = 0; for (auto& s : c.spans) if (length(s) <= tol) ++k; return k; }

struct C9 { Case c; const char* name; bool noisy; };
static Contour g_orig;
static C9 make9(std::uint64_t seed) {
    Rng g(seed ^ 0x7E57A11C0FFEEULL);
    int fam = g.irange(0, 5);
    Case2 cc;
    if (fam <= 1) { cc.c = makeCase(seed); cc.name = "legacy"; }
    else cc = makeCaseF(seed, fam);
    Case& c = cc.c;
    if (!c.in.closed && g.uni() < 0.25 && c.in.spans.size() >= 3) {
        Vec2 e = c.in.endPoint(), s = c.in.startPoint();
        if (distance(e, s) > 1e-2) { c.in.spans.push_back(LineSpan{e, s}); c.in.closed = true; }
    }
    double prob = g.uni() < 0.5 ? g.range(0.03, 0.3) : g.range(0.3, 1.0);
    c.in = inject9(c.in, g, c.tol, prob);
    return C9{c, cc.name, cc.noisy};
}
// Replace Beziers by the library's polygon (d = 0 result), verified against the curve.
static double g_polyErr = 0;
static Contour polyOf(const Contour& in, double tol) {
    if (!hasBez(in)) return in;
    auto z = offsetOpenContour(in, 0.0, OffsetSide::Left, JoinType::Round, Tolerance{tol, 1e-9, 0.01}, 2.0);
    Contour p; p.closed = in.closed;
    if (z.size() != 1) { std::printf("POLY d=0 gave %zu chains\n", z.size()); return in; }
    p.spans = z[0].spans;
    // every Bezier sample within chordal of the polygon, every polygon vertex on some sample's curve within chordal
    for (auto& s : in.spans) if (auto* b = std::get_if<BezierSpan>(&s)) {
        for (int k = 0; k <= 400; ++k) {
            Vec2 q = b->pointAt(k / 400.0);
            double best = 1e300;
            for (auto& ps : p.spans) best = std::min(best, distPt(toSeg(ps), q));
            g_polyErr = std::max(g_polyErr, best);
        }
    }
    return p;
}


// Tie-miss analysis: walk the locus from each tie point while valid and uncovered; measure the
// thickness of the valid region along the outward normal (a strip narrower than tol is thin).
static double distIn(const std::vector<Seg>& S, Vec2 p) { double b = 1e300; for (auto& g : S) b = std::min(b, distPt(g, p)); return b; }
static double distOut(const std::vector<Contour>& out, Vec2 p) { double b = 1e300; for (auto& k : out) for (auto& s : k.spans) b = std::min(b, distPt(toSeg(s), p)); return b; }
struct TieInfo { double maxThick = 0, maxLen = 0; int pts = 0; bool hitEnd = false; };
static double thick(const std::vector<Seg>& S, Vec2 f, Vec2 n, double d, double tol) {
    const double step = tol / 200.0;
    for (int i = 1; i <= 800; ++i) { Vec2 p = f + n * (step * i); if (distIn(S, p) < d - 1e-12) return step * (i - 1); }
    return 4 * tol;
}
static TieInfo tieAnalyse(const Contour& in, const std::vector<Contour>& out, double d, OffsetSide side, double tol) {
    TieInfo r;
    std::vector<Seg> S; for (auto& s : in.spans) if (length(s) > 0) S.push_back(toSeg(s));
    const double sgn = side == OffsetSide::Left ? 1.0 : -1.0;
    for (const TiePt& tp : g_tiePts) {
        r.pts++;
        if (tp.ex2 < 0) {
            const Seg& g = S[tp.ex1];
            // locus parameterised by t on span
            auto loc = [&](double t, Vec2& n) { Vec2 p = segPoint(g, t); n = segTan(g, t).perp() * sgn; return p + n * d; };
            // find t of q
            double bt = 0, bd = 1e300; for (int i = 0; i <= 2000; ++i) { double t = i / 2000.0; Vec2 n; double dd = distance(loc(t, n), tp.q); if (dd < bd) { bd = dd; bt = t; } }
            double L = segLen(g) * (1 + (g.kind == 1 ? d / g.r : 0));
            double dt = std::max(1e-6, (tol / 20) / std::max(L, 1e-300));
            for (int dir = -1; dir <= 1; dir += 2) {
                double len = 0;
                for (double t = bt; t >= 0 && t <= 1; t += dir * dt) {
                    Vec2 n; Vec2 f = loc(t, n);
                    if (distIn(S, f) < d - 1e-9 || distOut(out, f) <= tol) break;
                    r.maxThick = std::max(r.maxThick, thick(S, f, n, d, tol));
                    len += tol / 20;
                    if (t + dir * dt < 0 || t + dir * dt > 1) r.hitEnd = true;
                }
                r.maxLen = std::max(r.maxLen, len);
            }
        } else {
            Vec2 V = S[tp.ex1].b; Vec2 u = (tp.q - V).normalized();
            r.maxThick = std::max(r.maxThick, thick(S, tp.q, u, d, tol));
        }
    }
    return r;
}
static void dumpC9(const Contour& k) {
    std::printf("Contour in; in.closed = %s;\n", k.closed ? "true" : "false");
    for (auto& sp : k.spans) {
        if (auto* l = std::get_if<LineSpan>(&sp)) std::printf("in.spans.emplace_back(LineSpan{{%.17g, %.17g}, {%.17g, %.17g}});  // len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(sp));
        else if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("in.spans.emplace_back(ArcSpan{{%.17g, %.17g}, %.17g, %.17g, %.17g, ArcDirection::%s});  // len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(sp));
    }
}
int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "st") {
        long N = std::atol(argv[2]); std::uint64_t seed0 = std::strtoull(argv[3], nullptr, 0);
        g_covTie = 0.0005;  // tol/2: tie misses counted apart, all reported
        long n = 0, dv = 0, rv = 0, cm = 0, tie = 0, oth = 0, bez = 0, closedN = 0, perJ[3] = {0, 0, 0}, perS[2] = {0, 0};
        double wU = 0, wO = 0, wM = 0, wT = 0, cpu = 0, minL = 1e300;
        long ov = 0, back = 0, sx = 0, gaps = 0, zl = 0, se = 0, tieThin = 0; double maxThickAll = 0;
        for (long k = 0; k < N; ++k) {
            std::uint64_t seed = seed0 + (std::uint64_t)k;
            C9 cc = make9(seed); Case& c = cc.c;
            if (c.in.spans.empty() || !subTolN(c.in, c.tol)) continue;
            n++; perJ[(int)c.join]++; perS[(int)c.side]++; closedN += c.in.closed; bez += hasBez(c.in);
            for (auto& s : c.in.spans) if (length(s) > 0) minL = std::min(minL, length(s));
            double t0 = cpuNow();
            auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
            cpu += cpuNow() - t0;
            Contour ck = polyOf(c.in, c.tol);
            Metrics m; check(ck, out, c.d, c.side, c.join, c.tol, c.mitre, m, true);
            TieInfo ti; if (m.covMissTie) { ti = tieAnalyse(ck, out, c.d, c.side, c.tol); tieThin += ti.maxThick < c.tol; maxThickAll = std::max(maxThickAll, ti.maxThick); }
            g_tiePts.clear();
            dv += m.distViol > 0; rv += m.roundViol > 0; cm += m.covMiss > 0; tie += m.covMissTie > 0;
            wU = std::max(wU, m.worstUnder); wO = std::max(wO, m.worstOver); wM = std::max(wM, m.worstMiss); wT = std::max(wT, m.worstMissTie);
            int other = m.lineJunctionNotBitwise + m.arcGapBeyond + m.coderBack + m.overlapIn + m.overlapX + m.shortBack + m.selfX + m.zeroLen + m.subTolAtEnd + m.badSpan;
            oth += other > 0; ov += m.overlapIn + m.overlapX > 0; back += m.coderBack > 0; sx += m.selfX > 0; gaps += (m.lineJunctionNotBitwise + m.arcGapBeyond) > 0; zl += m.zeroLen > 0; se += m.subTolAtEnd > 0;
            if (m.distViol || m.roundViol || m.covMiss || m.covMissTie || other)
                std::printf("%s seed=0x%llx %s%s n=%zu st=%d closed=%d bez=%d d=%.6g %s side=%d mitre=%.3g: dv%d u%.3g rv%d o%.3g cm%d miss%.3g tie%d tmiss%.3g thick%.3g len%.3g end%d | gap%d back%d ov%d sb%d sx%d z%d se%d bad%d | %s\n",
                    (m.distViol || m.roundViol || m.covMiss) ? "CORE" : (m.covMissTie ? "TIE" : "OTHER"), (unsigned long long)seed, cc.name, cc.noisy ? "(noisy)" : "", c.in.spans.size(), subTolN(c.in, c.tol), c.in.closed, hasBez(c.in), c.d, jn(c.join), (int)c.side, c.mitre,
                    m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.covMiss, m.worstMiss, m.covMissTie, m.worstMissTie, ti.maxThick, ti.maxLen, (int)ti.hitEnd,
                    m.lineJunctionNotBitwise + m.arcGapBeyond, m.coderBack, m.overlapIn + m.overlapX, m.shortBack, m.selfX, m.zeroLen, m.subTolAtEnd, m.badSpan, m.firstIssue.c_str());
            std::fflush(stdout);
        }
        std::printf("SUMMARY st N=%ld seed0=0x%llx inputs %ld (R %ld Sh %ld Sq %ld; L %ld R %ld; closed %ld; bezier %ld; min span len %.3g) | dist %ld round %ld cov %ld | tie %ld (worst %.3g; thin<tol %ld, max thickness %.3g) | worst under %.3g over %.3g miss %.3g | other %ld: gaps %ld back %ld ov %ld sx %ld zl %ld se %ld | polyErr %.3g | cpu %.1f\n",
            N, (unsigned long long)seed0, n, perJ[0], perJ[1], perJ[2], perS[0], perS[1], closedN, bez, minL, dv, rv, cm, tie, wT, tieThin, maxThickAll, wU, wO, wM, oth, gaps, back, ov, sx, zl, se, g_polyErr, cpu);
        return 0;
    }
    if (mode == "time") {
        long N = std::atol(argv[2]); std::uint64_t seed0 = std::strtoull(argv[3], nullptr, 0);
        for (long k = 0; k < N; ++k) {
            std::uint64_t seed = seed0 + (std::uint64_t)k;
            C9 cc = make9(seed); Case& c = cc.c;
            double t0 = cpuNow();
            auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
            double t1 = cpuNow();
            Contour ck = polyOf(c.in, c.tol);
            Metrics m; g_covTie = 0.0005; check(ck, out, c.d, c.side, c.join, c.tol, c.mitre, m, true);
            double t2 = cpuNow();
            if (t2 - t0 > 0.2) std::printf("seed 0x%llx n=%zu %s d=%.3g gen=%d %s: offset %.2fs check %.2fs\n", (unsigned long long)seed, c.in.spans.size(), cc.name, c.d, c.gen, jn(c.join), t1 - t0, t2 - t1);
            std::fflush(stdout);
        }
        return 0;
    }
    if (mode == "poly") {  // the polygonised input, no Beziers: branch vs main, dump as C++
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0);
        C9 cc = make9(seed); Case& c = cc.c;
        Contour ck = polyOf(c.in, c.tol);
        g_covTie = 0.0005;
        auto out = offsetOpenContour(ck, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        Metrics m; check(ck, out, c.d, c.side, c.join, c.tol, c.mitre, m, true);
        std::printf("branch(poly): %s [dv%d u%.3g cm%d tie%d tmiss %.3g]\n", m.firstIssue.c_str(), m.distViol, m.worstUnder, m.covMiss, m.covMissTie, m.worstMissTie);
        auto mp = mainOffset(toP(ck), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        Metrics mm; check(ck, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, true);
        std::printf("main(poly): %s [dv%d u%.3g cm%d tie%d tmiss %.3g]\n", mm.firstIssue.c_str(), mm.distViol, mm.worstUnder, mm.covMiss, mm.covMissTie, mm.worstMissTie);
        if (argc > 3) {
            std::printf("// seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g, %zu spans\n", (unsigned long long)seed, c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre, ck.spans.size());
            std::printf("Contour in; in.closed = %s;\n", ck.closed ? "true" : "false");
            for (auto& sp : ck.spans) {
                if (auto* l = std::get_if<LineSpan>(&sp)) std::printf("in.spans.emplace_back(LineSpan{{%.17g, %.17g}, {%.17g, %.17g}});\n", l->start.x, l->start.y, l->end.x, l->end.y);
                else if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("in.spans.emplace_back(ArcSpan{{%.17g, %.17g}, %.17g, %.17g, %.17g, ArcDirection::%s});\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW");
            }
        }
        return 0;
    }
    if (mode == "shrinkpoly") {
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0);
        C9 cc = make9(seed); Case c = cc.c;
        Contour cur = polyOf(c.in, c.tol);
        auto bad = [&](const Contour& k) {
            auto o = offsetOpenContour(k, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
            Metrics m; check(k, o, c.d, c.side, c.join, c.tol, c.mitre, m, false, true);
            return m.distViol > 0 && m.worstUnder > 0.01 * c.d;
        };
        if (!bad(cur)) { std::printf("not bad\n"); return 0; }
        bool prog = true;
        while (prog) {
            prog = false;
            if (cur.closed) { Contour t = cur; t.closed = false; if (bad(t)) { cur = t; prog = true; continue; } }
            for (std::size_t i = 0; i < cur.spans.size() && cur.spans.size() > 1; ++i) {
                Contour t = cur; t.spans.erase(t.spans.begin() + (long)i);
                if (!t.closed && i != 0 && i + 1 != cur.spans.size()) continue;
                if (t.closed) continue;
                if (bad(t)) { cur = t; prog = true; break; }
            }
        }
        std::printf("// seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g, %zu spans\n", (unsigned long long)seed, c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre, cur.spans.size());
        dumpC9(cur);
        auto o = offsetOpenContour(cur, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        Metrics m; check(cur, o, c.d, c.side, c.join, c.tol, c.mitre, m, false, true);
        std::printf("branch: %s\n", m.firstIssue.c_str());
        for (std::size_t i = 0; i < o.size(); ++i) { std::printf("chain %zu closed %d\n", i, o[i].closed); for (auto& sp : o[i].spans) { if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW"); else { auto& l = std::get<LineSpan>(sp); std::printf("  L (%.17g,%.17g)->(%.17g,%.17g)\n", l.start.x, l.start.y, l.end.x, l.end.y); } } }
        auto mp = mainOffset(toP(cur), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        Metrics mm; check(cur, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, false, true);
        std::printf("main: %s [dv%d u%.3g]\n", mm.firstIssue.c_str(), mm.distViol, mm.worstUnder);
        return 0;
    }
    if (mode == "one") {
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0);
        g_covTie = argc > 3 ? std::atof(argv[3]) : 0.0;
        C9 cc = make9(seed); Case& c = cc.c;
        std::printf("// seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g, %zu spans\n", (unsigned long long)seed, c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre, c.in.spans.size());
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        Contour ck = polyOf(c.in, c.tol);
        Metrics m; check(ck, out, c.d, c.side, c.join, c.tol, c.mitre, m, true);
        std::printf("branch: %s [dv%d u%.3g rv%d o%.3g cm%d miss %.3g tie %d]\n", m.firstIssue.c_str(), m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.covMiss, m.worstMiss, m.covMissTie);
        if (!hasBez(c.in)) {
            auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre);
            std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
            Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, true);
            std::printf("main: %s [dv%d u%.3g rv%d o%.3g cm%d miss %.3g tie %d]\n", mm.firstIssue.c_str(), mm.distViol, mm.worstUnder, mm.roundViol, mm.worstOver, mm.covMiss, mm.worstMiss, mm.covMissTie);
        }
        if (argc > 4) { std::printf("INPUT\n"); for (auto& s : ck.spans) { Seg q = toSeg(s); std::printf(" k%d a(%.17g,%.17g) b(%.17g,%.17g) r %.6g len %.3g\n", q.kind, q.a.x, q.a.y, q.b.x, q.b.y, q.r, length(s)); }
            for (std::size_t i = 0; i < out.size(); ++i) { std::printf("CHAIN %zu closed %d\n", i, out[i].closed); for (auto& s : out[i].spans) { Seg q = toSeg(s); std::printf(" k%d a(%.17g,%.17g) b(%.17g,%.17g) r %.6g len %.3g\n", q.kind, q.a.x, q.a.y, q.b.x, q.b.y, q.r, length(s)); } } }
        return 0;
    }
    return vdrv4_main(argc, argv);
}
