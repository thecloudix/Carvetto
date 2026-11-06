// T-0128 round-3 (PR #61 MSVC) verifier: the tests' self-crossing count (cae3fa7, with the noise
// filter) and the count without it, the largest filtered ratio (dist x sine / unit), and the
// requireToolpathChains checks (junctions, back-steps, distance bounds), for branch (BLIB) and cvmain (MLIB).
#include <limits>
struct SX { long sxNew = 0, sxOld = 0, gaps = 0, back = 0, dv = 0, rv = 0, endShort = 0; double maxNoise = 0, maxGap = 0; double maxRealLow = 1e300; };
static void sxCount(const std::vector<Contour>& chains, SX& r, bool verbose = false) {
    const Tolerance exact{1e-9, 1e-9, 0.01};
    const double eps = std::numeric_limits<double>::epsilon();
    for (const Contour& c : chains) {
        const std::size_t n = c.spans.size();
        for (std::size_t i = 0; i < n; ++i) for (std::size_t j = i + 1; j < n; ++j) {
            const bool next = j == i + 1;
            const bool wrap = c.closed && i == 0 && j + 1 == n;
            for (const Intersection& x : intersect(c.spans[i], c.spans[j], exact).points) {
                const double slack = 1e-9 * std::max({1.0, std::fabs(x.point.x), std::fabs(x.point.y)});
                const bool atJunction = (next && distance(x.point, endPoint(c.spans[i])) <= slack) ||
                    (wrap && distance(x.point, startPoint(c.spans[i])) <= slack) ||
                    (n == 2 && c.closed && distance(x.point, endPoint(c.spans[j])) <= slack);
                bool nearJunction = false;
                if (j - i <= 3 || (c.closed && n - j + i <= 3)) for (const Span& sp : c.spans) nearJunction = nearJunction || distance(x.point, startPoint(sp)) <= slack;
                bool noise = false; double ratio = 1e300;
                if (next || wrap) {
                    const double sine = std::fabs(tangentAt(c.spans[i], x.tA).cross(tangentAt(c.spans[j], x.tB)));
                    const double unit = eps * std::max({1.0, std::fabs(x.point.x), std::fabs(x.point.y), length(c.spans[i]), length(c.spans[j])});
                    std::vector<Vec2> js; if (next) js.push_back(endPoint(c.spans[i])); if (wrap) js.push_back(startPoint(c.spans[i]));
                    for (Vec2 jn : js) { ratio = std::min(ratio, distance(x.point, jn) * sine / unit); noise = noise || distance(x.point, jn) * sine <= 100.0 * unit; }
                }
                if (!atJunction && !nearJunction) {
                    r.sxOld++;
                    if (noise) r.maxNoise = std::max(r.maxNoise, ratio); else { r.sxNew++; if (ratio < 1e300) r.maxRealLow = std::min(r.maxRealLow, ratio); }
                    if (verbose) std::printf("   X spans %zu,%zu next %d at (%.17g,%.17g) ratio %.4g %s\n", i, j, (int)next, x.point.x, x.point.y, ratio, noise ? "NOISE" : "REAL");
                }
            }
        }
    }
}
static void reqChains(const std::vector<Contour>& chains, const Contour& in, double d, JoinType join, SX& r) {
    const double tol = 0.001;
    const double allowance = std::max(tol * tol / (8.0 * d), std::min(5e-7 * std::max(1.0, d), 0.1e-6 / d));
    for (const Contour& c : chains) {
        const std::size_t n = c.spans.size();
        if (!n) { r.gaps++; continue; }
        for (std::size_t j = c.closed ? 0 : 1; j < n; ++j) {
            const Span& a = c.spans[(j + n - 1) % n]; const Span& b = c.spans[j];
            const Vec2 e = endPoint(a), s = startPoint(b);
            if (std::holds_alternative<LineSpan>(a) || std::holds_alternative<LineSpan>(b)) { if (!(e == s)) { r.gaps++; r.maxGap = std::max(r.maxGap, distance(e, s)); } }
            else if (distance(e, s) > 1e-12 * std::max({1.0, std::fabs(e.x), std::fabs(e.y), d})) { r.gaps++; r.maxGap = std::max(r.maxGap, distance(e, s)); }
        }
        if (!c.closed && n > 1 && (length(c.spans.front()) <= tol || length(c.spans.back()) <= tol)) r.endShort++;
        Tolerance kt{}; r.back += carvetto::geometry::testing::countBackwardSteps(c, kt, nullptr);
        for (const Span& s : c.spans) for (int k = 0; k <= 64; ++k) {
            double m = 1e300; for (const Span& q : in.spans) m = std::min(m, carvetto::geometry::testing::distanceToSpanExact(pointAt(s, k / 64.0), q));
            if (m < d - allowance - 1e-12 * std::max(1.0, d)) r.dv++;
            if (join == JoinType::Round && m > d + allowance + 1e-12 * std::max(1.0, d)) r.rv++;
        }
    }
    sxCount(chains, r);
}
static void sxAdd(SX& t, const SX& r) { t.sxNew += r.sxNew; t.sxOld += r.sxOld; t.gaps += r.gaps; t.back += r.back; t.dv += r.dv; t.rv += r.rv; t.endShort += r.endShort; t.maxNoise = std::max(t.maxNoise, r.maxNoise); t.maxGap = std::max(t.maxGap, r.maxGap); t.maxRealLow = std::min(t.maxRealLow, r.maxRealLow); }
static bool sxBad(const SX& r) { return r.sxNew || r.gaps || r.back || r.dv || r.rv || r.endShort; }
static void sxPrint(const char* tag, const SX& t) { std::printf("%s sxNew %ld sxOld %ld maxNoise %.3g minRealRatio %.3g gaps %ld (max %.3g) back %ld dv %ld rv %ld endShort %ld", tag, t.sxNew, t.sxOld, t.maxNoise, t.maxRealLow, t.gaps, t.maxGap, t.back, t.dv, t.rv, t.endShort); }
// sxcusp which d nG gmax [nshift]: the vdrv5 cusp sweep (log g grid, both signs) + the test's g grid; 6 shifts (+ random ones).
int sxCusp(int which, double d, int nG, double gmax, int extraShifts, int cmpMain) {
    std::vector<Vec2> shifts = {{0, 0}, {1e2, -1e2}, {1e3, 1e3}, {-1e4, 1e4}, {1e4, 1e4}, {12345.678, -9876.54321}};
    Rng rg(0x5a3c0000ULL + (unsigned)which * 977 + (unsigned)(d * 1000));
    for (int k = 0; k < extraShifts; ++k) { double m = std::pow(10.0, rg.range(0.0, 5.5)); shifts.push_back({rg.range(-m, m), rg.range(-m, m)}); }
    std::vector<double> gs{0.0};
    for (int gi = 0; gi < nG; ++gi) { double g = gmax * std::pow(10.0, -6.0 * (nG - gi) / nG); gs.push_back(g); gs.push_back(-g); }
    for (int k = 0; k <= 12; ++k) { gs.push_back(1e-6 * std::pow(10.0, -0.5 * k)); gs.push_back(-1e-6 * std::pow(10.0, -0.5 * k)); }
    for (double g : {1e-12, 3.2e-11, 1e-11, 1e-13, 1e-14}) { gs.push_back(g); gs.push_back(-g); }
    SX tb, tm; long cases = 0, bad = 0, mbad = 0;
    for (double g : gs) for (Vec2 sh : shifts) for (int side = 0; side < 2; ++side) {
        Contour in = fromPts(cuspBase(which, d, g), false); transformC(in, 0.0, sh);
        OffsetSide sd = side ? OffsetSide::Right : OffsetSide::Left;
        auto out = offsetOpenContour(in, d, sd, JoinType::Round, Tolerance{0.001, 1e-9, 0.01}, 3.8787999757578069);
        SX r; reqChains(out, in, d, JoinType::Round, r); sxAdd(tb, r); cases++;
        if (sxBad(r)) { bad++; if (bad <= 20) { std::printf("BAD which=%d d=%.17g g=%.17g shift=(%.17g,%.17g) side=%d ", which, d, g, sh.x, sh.y, side); sxPrint("", r); std::printf("\n"); } }
        if (cmpMain) {
            auto mp = mainOffset(toP(in), d, side, 0, 0.001, 3.8787999757578069);
            std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
            SX m; reqChains(mo, in, d, JoinType::Round, m); sxAdd(tm, m); if (sxBad(m)) mbad++;
        }
    }
    std::printf("sxcusp which=%d d=%g cases %ld | branch bad %ld ", which, d, cases, bad); sxPrint("", tb);
    if (cmpMain) { std::printf(" | ref bad %ld ", mbad); sxPrint("", tm); }
    std::printf("\n");
    return 0;
}
int sxOne(int which, double d, double g, double sx, double sy, int side) {
    Contour in = fromPts(cuspBase(which, d, g), false); transformC(in, 0.0, Vec2{sx, sy});
    OffsetSide sd = side ? OffsetSide::Right : OffsetSide::Left;
    auto out = offsetOpenContour(in, d, sd, JoinType::Round, Tolerance{0.001, 1e-9, 0.01}, 3.8787999757578069);
    for (std::size_t i = 0; i < out.size(); ++i) { std::printf("chain %zu closed %d\n", i, out[i].closed); for (auto& s : out[i].spans) { if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s)); else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s)); } }
    SX r; reqChains(out, in, d, JoinType::Round, r); sxCount(out, r, true); sxPrint("one", r); std::printf("\n");
    return 0;
}
// sxfz fam N seed0: fuzz family, branch and ref with the tests' rules; prints branch-only failures.
int sxFz(int fam, long N, std::uint64_t seed0) {
    SX tb, tm; long n = 0, bad = 0, mbad = 0, bOnly = 0, ident = 0;
    for (long k = 0; k < N; ++k) {
        std::uint64_t seed = seed0 + (std::uint64_t)k;
        Case2 cc = makeCaseF(seed, fam); Case& c = cc.c; if (c.in.spans.empty()) continue;
        for (int side = 0; side < 2; ++side) {
            OffsetSide sd = side ? OffsetSide::Right : OffsetSide::Left;
            auto out = offsetOpenContour(c.in, c.d, sd, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
            auto mp = mainOffset(toP(c.in), c.d, side, (int)c.join, c.tol, c.mitre);
            std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
            n++;
            auto po = toP(out.empty() ? Contour{} : out[0]);
            bool same = out.size() == mo.size();
            for (std::size_t i = 0; same && i < out.size(); ++i) { auto a = toP(out[i]), b = mp[i]; same = a.closed == b.closed && a.s.size() == b.s.size(); for (std::size_t q = 0; same && q < a.s.size(); ++q) same = a.s[q].kind == b.s[q].kind && std::memcmp(a.s[q].a, b.s[q].a, sizeof a.s[q].a) == 0; }
            if (same) { ident++; continue; }
            SX r, m; reqChains(out, c.in, c.d, c.join, r); reqChains(mo, c.in, c.d, c.join, m); sxAdd(tb, r); sxAdd(tm, m);
            bool b1 = sxBad(r), m1 = sxBad(m); bad += b1; mbad += m1;
            bool worse = r.sxNew > m.sxNew || r.gaps > m.gaps || r.back > m.back || r.dv > m.dv || r.rv > m.rv || r.endShort > m.endShort;
            if (worse) { bOnly++; if (bOnly <= 20) { std::printf("WORSE fam %d seed=0x%llx side %d %s ", fam, (unsigned long long)seed, side, jn(c.join)); sxPrint("br", r); sxPrint(" | ref", m); std::printf("\n"); } }
        }
    }
    std::printf("sxfz fam %d N %ld seed0 0x%llx: offsets %ld identical %ld | branch bad %ld ref bad %ld worse %ld | ", fam, N, (unsigned long long)seed0, n, ident, bad, mbad, bOnly);
    sxPrint("br", tb); sxPrint(" | ref", tm); std::printf("\n");
    return 0;
}
static double sxDistChains(Vec2 p, const std::vector<Contour>& ch) { double b = 1e300; for (auto& c : ch) for (auto& s : c.spans) b = std::min(b, carvetto::geometry::testing::distanceToSpanExact(p, s)); return b; }
// sxchan L d: chan.cpp's channel sweep (w >= 0 and w < 0) with the tests' rules + centre-line coverage, branch and ref.
int sxChan(double L, double d) {
    const double tol = 1e-3;
    std::vector<double> ws{0.0, 1e-16, 1e-15, 1e-14, 1e-13};
    for (int k = 0; k <= 28; ++k) ws.push_back(1e-12 * std::pow(10.0, k / 4.0));
    for (double x : {3e-9, 4e-7, 4.4e-7, 1e-7, 3e-8, 2e-6, 5e-6}) ws.push_back(x);
    { size_t n = ws.size(); for (size_t i = 1; i < n; ++i) ws.push_back(-ws[i]); }
    const Vec2 shifts[] = {{0, 0}, {12345.678, -5014.09}, {-98765.4321, 31415.9265}};
    const double rots[] = {0.0, 0.3, kPi / 2, 2.1, -0.7853981633974483};
    SX tb, tm; long n = 0, bad = 0, mbad = 0, lostB = 0, lostM = 0, worse = 0;
    for (int shape = 0; shape < 4; ++shape) for (double w : ws) for (Vec2 sh : shifts) for (double th : rots) for (int j = 0; j < 3; ++j) {
        const double H = 2 * d + w; Contour in; OffsetSide side = OffsetSide::Left; Vec2 a{0.05, 0.5 * H}, b{L - d - 0.05, 0.5 * H};
        if (shape == 0) in = fromPts({{0, 0}, {L, 0}, {L, H}, {0, H}}, false);
        else if (shape == 1) { in = fromPts({{0, H}, {L, H}, {L, 0}, {0, 0}}, false); side = OffsetSide::Right; }
        else if (shape == 2) { ArcSpan e{{L, 0.5 * H}, 0.5 * H, -0.5 * kPi, 0.5 * kPi, ArcDirection::CCW};
            in.spans.emplace_back(LineSpan{{0, 0}, e.startPoint()}); in.spans.emplace_back(e); in.spans.emplace_back(LineSpan{e.endPoint(), {0, H}}); b = Vec2{L - 0.05, 0.5 * H}; }
        else { in = fromPts({{0, 0}, {L, 0}, {L, H}, {0, H}}, true); a = Vec2{d + 0.05, 0.5 * H}; }
        bool cov = !(shape == 3 && L <= 2 * d + 0.1) && w >= 0;
        transformC(in, th, sh);
        auto R = [&](Vec2 p) { return Vec2{std::cos(th) * p.x - std::sin(th) * p.y + sh.x, std::sin(th) * p.x + std::cos(th) * p.y + sh.y}; };
        Vec2 A = R(a), B = R(b); JoinType join = (JoinType)j; n++;
        auto ob = offsetOpenContour(in, d, side, join, Tolerance{tol, 1e-9, 0.01}, 2.0);
        auto mp = mainOffset(toP(in), d, (int)side, j, tol, 2.0); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
        double cb = 0, cm = 0;
        if (cov) for (int k = 0; k <= 200; ++k) { Vec2 q = A + (B - A) * (k / 200.0); cb = std::max(cb, sxDistChains(q, ob)); cm = std::max(cm, sxDistChains(q, om)); }
        SX r, m; reqChains(ob, in, d, join, r); reqChains(om, in, d, join, m); sxAdd(tb, r); sxAdd(tm, m);
        bool lb = cov && cb > tol, lm = cov && cm > tol; lostB += lb; lostM += lm;
        bool b1 = sxBad(r) || lb; bad += b1; mbad += sxBad(m) || lm;
        bool wz = r.sxNew > m.sxNew || r.gaps > m.gaps || r.back > m.back || r.dv > m.dv || r.rv > m.rv || r.endShort > m.endShort || (lb && !lm);
        worse += wz;
        if (b1 && bad <= 15) { std::printf("BAD d=%g shape=%d w=%.17g shift=(%g,%g) th=%.17g j=%d lost %d ", d, shape, w, sh.x, sh.y, th, j, (int)lb); sxPrint("", r); std::printf("\n"); }
    }
    std::printf("sxchan L=%g d=%g offsets %ld | branch bad %ld lost %ld | ref bad %ld lost %ld | worse %ld | ", L, d, n, bad, lostB, mbad, lostM, worse); sxPrint("br", tb); sxPrint(" | ref", tm); std::printf("\n");
    return 0;
}
// sxseed fam seed: one fuzz seed, both sides: tests' rules for branch and ref, crossings listed.
int sxSeed(int fam, std::uint64_t seed) {
    Case2 cc = makeCaseF(seed, fam); Case& c = cc.c;
    std::printf("seed 0x%llx fam %d %s d %.17g join %s mitre %.17g n %zu\n", (unsigned long long)seed, fam, cc.name, c.d, jn(c.join), c.mitre, c.in.spans.size());
    for (int side = 0; side < 2; ++side) {
        OffsetSide sd = side ? OffsetSide::Right : OffsetSide::Left;
        auto out = offsetOpenContour(c.in, c.d, sd, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        auto mp = mainOffset(toP(c.in), c.d, side, (int)c.join, c.tol, c.mitre); std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        for (int w = 0; w < 2; ++w) { auto& o = w ? mo : out; SX r; reqChains(o, c.in, c.d, c.join, r); std::printf(" side %d %s chains %zu:", side, w ? "ref" : "br", o.size()); sxPrint("", r); std::printf("\n"); SX q; sxCount(o, q, true);
            if (std::getenv("SXDUMP")) for (std::size_t i = 0; i < o.size(); ++i) { std::printf("  chain %zu closed %d\n", i, o[i].closed); for (auto& s : o[i].spans) { if (auto* l = std::get_if<LineSpan>(&s)) std::printf("   L (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s)); else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("   A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s)); } } }
    }
    return 0;
}
// sxarc: arc channels (inner arc radius R, outer R + 2d + w, joined by a line cap at the end, open at the start),
// offset on the inside by d: a needle on circle R + d out and back. Centre-arc coverage (worst distance) + tests' rules.
int sxArc(int nR) {
    const double tol = 1e-3;
    SX tb, tm; long n = 0, bad = 0, mbad = 0, worse = 0, lostB = 0, lostM = 0; double worstB = 0, worstM = 0, worstDiff = 0;
    Rng rg(0xa3c0ffeeULL + (unsigned)nR);
    for (int it = 0; it < nR; ++it) {
        double R = std::exp(rg.range(std::log(0.5), std::log(2000.0))), d = std::exp(rg.range(std::log(0.05), std::log(5.0)));
        double th = rg.range(0.05, 5.5); if (th * R < 4 * d + 0.1) th = (4 * d + 0.1) / R; if (th > 6.0) continue;
        double wsel = rg.uni(); double w = wsel < 0.4 ? 0.0 : (wsel < 0.7 ? std::pow(10.0, rg.range(-15, -6)) : -std::pow(10.0, rg.range(-15, -9)));
        double m = rg.uni() < 0.3 ? 0.0 : std::pow(10.0, rg.range(0.0, 5.3)); Vec2 sh{rg.range(-m, m), rg.range(-m, m)};
        int capk = 0; (void)rg.irange(0, 1); bool ccw = rg.uni() < 0.5;
        const double Ro = R + 2 * d + w;
        Contour in;
        ArcDirection dir = ccw ? ArcDirection::CCW : ArcDirection::CW; double s0 = rg.range(-3.1, 3.1), s1 = ccw ? s0 + th : s0 - th;
        ArcSpan inner{{0, 0}, R, s0, s1, dir};
        ArcSpan outer{{0, 0}, Ro, s1, s0, ccw ? ArcDirection::CW : ArcDirection::CCW};
        in.spans.emplace_back(inner);
        if (capk == 0) in.spans.emplace_back(LineSpan{inner.endPoint(), outer.startPoint()});
        else { Vec2 mid = (inner.endPoint() + outer.startPoint()) * 0.5; double a0 = std::atan2(inner.endPoint().y - mid.y, inner.endPoint().x - mid.x);
               in.spans.emplace_back(ArcSpan{mid, 0.5 * (Ro - R), a0, a0 + (ccw ? kPi : -kPi), ccw ? ArcDirection::CCW : ArcDirection::CW}); }
        in.spans.emplace_back(outer);
        transformC(in, 0.0, sh);
        // inside: the channel lies left of a CCW inner arc travelled outward... try both sides, keep the one whose chains reach the centre arc
        for (int side = 0; side < 2; ++side) for (int j = 0; j < 3; ++j) {
            OffsetSide sd = side ? OffsetSide::Right : OffsetSide::Left; JoinType join = (JoinType)j;
            auto ob = offsetOpenContour(in, d, sd, join, Tolerance{tol, 1e-9, 0.01}, 2.0);
            auto mp = mainOffset(toP(in), d, side, j, tol, 2.0); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
            // centre arc from angle s0 + 0.02/Rc .. to s1 - (d + 0.05)/Rc (stop short of the cap by d + 0.05)
            const double Rc = R + d; double a0 = s0 + (ccw ? 1 : -1) * 0.05 / Rc, a1 = s1 - (ccw ? 1 : -1) * (std::asin(std::min(1.0, d / Rc)) + 0.05 / Rc);
            double cb = 0, cm = 0;
            for (int k = 0; k <= 200; ++k) { double a = a0 + (a1 - a0) * k / 200.0; Vec2 q = Vec2{Rc * std::cos(a), Rc * std::sin(a)} + sh; cb = std::max(cb, sxDistChains(q, ob)); cm = std::max(cm, sxDistChains(q, om)); }
            bool inside = (ccw && side == 1) || (!ccw && side == 0);
            n++;
            SX r, mm; reqChains(ob, in, d, join, r); reqChains(om, in, d, join, mm); sxAdd(tb, r); sxAdd(tm, mm);
            bool lb = inside && w >= 0 && cb > tol, lm = inside && w >= 0 && cm > tol; lostB += lb; lostM += lm;
            if (inside && w >= 0) { worstB = std::max(worstB, cb); worstM = std::max(worstM, cm); worstDiff = std::max(worstDiff, cb - cm); }
            bool b1 = sxBad(r) || lb; bad += b1; mbad += sxBad(mm) || lm;
            bool wz = r.sxNew > mm.sxNew || r.gaps > mm.gaps || r.back > mm.back || r.dv > mm.dv || r.rv > mm.rv || r.endShort > mm.endShort || (lb && !lm) || (inside && w >= 0 && cb > cm + 1e-6);
            worse += wz;
            if ((wz || b1) && (worse + bad) <= 30) { std::printf("%s it=%d R=%.17g d=%.17g th=%.17g w=%.3g sh=(%.17g,%.17g) cap %d ccw %d s0 %.17g side %d j %d cov br %.3g ref %.3g ", wz ? "WORSE" : "BAD", it, R, d, th, w, sh.x, sh.y, capk, (int)ccw, s0, side, j, cb, cm); sxPrint("", r); sxPrint(" | ref", mm); std::printf("\n"); }
        }
    }
    std::printf("sxarc N=%d offsets %ld | branch bad %ld lost %ld worstCov %.3g | ref bad %ld lost %ld worstCov %.3g | worse %ld worstCovDiff %.3g | ", nR, n, bad, lostB, worstB, mbad, lostM, worstM, worse, worstDiff); sxPrint("br", tb); sxPrint(" | ref", tm); std::printf("\n");
    return 0;
}
