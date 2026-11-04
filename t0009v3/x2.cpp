// T-0009 round-2 verifier driver. Generators: v9 (round-1 injector, make9) and g2 (new, round-2
// targets); checkers: vchk (round-1, corrected vdrv4) and xk (independent, xchk.hpp).
#include "v9lib.cpp"
#include "xchk.hpp"
#include <set>
// sub-tol count without the library's Bezier length (pathologically slow on some degenerate Beziers):
// a Bezier counts when its control polygon is no longer than tol, or a chord of its 64-point sampling is.
static int stN(const Contour& c, double tol) { int k = 0; for (auto& s : c.spans) { if (auto* b = std::get_if<BezierSpan>(&s)) { double cp = distance(b->p0, b->p1) + distance(b->p1, b->p2) + distance(b->p2, b->p3); bool t = cp <= tol; Vec2 pr = b->p0; for (int i = 1; i <= 64 && !t; ++i) { Vec2 q = b->pointAt(i / 64.0); if (distance(pr, q) <= tol) t = true; pr = q; } k += t; } else k += length(s) <= tol; } return k; }

static void dumpIn(const Contour& k, FILE* f) {
    std::fprintf(f, "Contour in; in.closed = %s;\n", k.closed ? "true" : "false");
    for (auto& sp : k.spans) {
        if (auto* l = std::get_if<LineSpan>(&sp)) std::fprintf(f, "in.spans.emplace_back(LineSpan{{%.17g, %.17g}, {%.17g, %.17g}});\n", l->start.x, l->start.y, l->end.x, l->end.y);
        else if (auto* a = std::get_if<ArcSpan>(&sp)) std::fprintf(f, "in.spans.emplace_back(ArcSpan{{%.17g, %.17g}, %.17g, %.17g, %.17g, ArcDirection::%s});\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW");
        else if (auto* b = std::get_if<BezierSpan>(&sp)) std::fprintf(f, "in.spans.emplace_back(BezierSpan{{%.17g, %.17g}, {%.17g, %.17g}, {%.17g, %.17g}, {%.17g, %.17g}});\n", b->p0.x, b->p0.y, b->p1.x, b->p1.y, b->p2.x, b->p2.y, b->p3.x, b->p3.y);
    }
}

// ------------------------------------------------------------- g2 generator
struct G2 { Contour in; double d, mitre, tol = 0.001; OffsetSide side; JoinType join; int kind; };
static double tinyLen(Rng& g, double tol) { double u = g.uni(); if (u < 0.4) return tol * std::pow(10.0, -g.range(0, 10)); if (u < 0.55) return tol * g.range(0.95, 1.0); return tol * g.range(1e-3, 1.0); }
static G2 makeG2(std::uint64_t seed) {
    Rng g(seed * 0x9E3779B97F4A7C15ULL + 0xA5A5);
    G2 c; c.kind = g.irange(0, 6);
    c.d = std::exp(g.range(std::log(0.01), std::log(5.0)));
    c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    c.join = (JoinType)g.irange(0, 2);
    c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 4.0);
    const double tol = c.tol;
    Contour& k = c.in;
    Vec2 p{0, 0}; double h = g.range(-3.14, 3.14);
    auto line = [&](double L) { Vec2 q = p + Vec2::fromAngle(h, L); k.spans.push_back(LineSpan{p, q}); p = q; };
    auto tinyRun = [&](int m, int style) {  // run of m sub-tol spans starting at p, continuous
        double hh = h;
        for (int i = 0; i < m; ++i) {
            double L = tinyLen(g, tol);
            if (style == 0) hh = g.range(-3.14159, 3.14159);                  // micro cluster
            else if (style == 1) hh = h + (i % 2 ? 1 : -1) * g.range(0.3, 1.4); // zig-zag
            else hh = h + g.range(-0.3, 0.3);                                   // noisy sampled line
            Vec2 q = p + Vec2::fromAngle(hh, L); if (q == p) continue; k.spans.push_back(LineSpan{p, q}); p = q;
        }
    };
    switch (c.kind) {
        case 0: {  // sharp corners with sub-tol runs at them
            int nv = g.irange(2, 6);
            for (int i = 0; i < nv; ++i) { line(g.range(0.2, 3)); if (g.uni() < 0.7) tinyRun(g.irange(1, 20), g.irange(0, 2)); h += g.range(-2.8, 2.8); }
            line(g.range(0.2, 3)); break; }
        case 1: {  // open chain made only of tiny pieces
            int m = g.irange(3, 400); int st = g.irange(0, 2);
            if (g.uni() < 0.3) {  // densely sampled arc of radius R
                double R = g.range(0.01, 2), a0 = g.range(0, 6.28), sw = g.range(-6, 6); int nn = std::max(3, (int)(R * std::fabs(sw) / (tol * g.range(0.3, 1.0))));
                nn = std::min(nn, 3000);
                Vec2 prev{R * std::cos(a0), R * std::sin(a0)};
                for (int i = 1; i <= nn; ++i) { double a = a0 + sw * i / nn; Vec2 q{R * std::cos(a), R * std::sin(a)}; if (q != prev) k.spans.push_back(LineSpan{prev, q}); prev = q; }
                p = prev;  // (round-2 verifier fix: closing line starts at the last sample)
            } else tinyRun(m, st);
            if (g.uni() < 0.2 && k.spans.size() > 3) { k.spans.push_back(LineSpan{p, startPoint(k.spans.front())}); if (length(k.spans.back()) > 0) k.closed = true; else k.spans.pop_back(); }
            break; }
        case 2: {  // B1: tiny large-sweep arc between nearly collinear lines
            line(g.range(0.1, 3));
            double sw = g.range(0.5, 6.2831) * (g.uni() < 0.5 ? 1 : -1);
            double r = std::pow(10.0, -g.range(4, 16));
            Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * r : p - left * r;
            ArcSpan a{ctr, r, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw;
            k.spans.push_back(a); Vec2 e = a.endPoint();
            h += g.range(-1e-3, 1e-3) + (g.uni() < 0.3 ? sw : 0.0);
            p = e; line(g.range(0.1, 3)); break; }
        case 3: {  // collapsing arcs: radius equal to d (to rounding) or just around it
            line(g.range(0.2, 2));
            double r = c.d * (g.uni() < 0.5 ? 1.0 : 1.0 + g.range(-1e-9, 1e-9)); double sw = g.range(0.2, 3.0) * (g.uni() < 0.5 ? 1 : -1);
            Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * r : p - left * r;
            ArcSpan a{ctr, r, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw;
            k.spans.push_back(a); p = a.endPoint(); h += sw + g.range(-0.5, 0.5);
            if (g.uni() < 0.5) tinyRun(g.irange(1, 5), 0);
            line(g.range(0.2, 2)); break; }
        case 4: {  // long noisy sampled line through corners (Sharp/Square heavy), tiny ends
            if (g.uni() < 0.5) tinyRun(g.irange(1, 6), 0);
            int nv = g.irange(1, 4);
            for (int i = 0; i < nv; ++i) { tinyRun(g.irange(20, 300), 2); h += g.range(-2.5, 2.5); line(g.range(0.05, 1)); }
            if (g.uni() < 0.5) tinyRun(g.irange(1, 6), 0);
            break; }
        case 5: {  // closed polygon with micro-clusters and tiny closing spans
            int nv = g.irange(3, 7); double R = g.range(0.3, 3); Vec2 c0{0, 0};
            std::vector<Vec2> V; for (int i = 0; i < nv; ++i) V.push_back(c0 + Vec2::fromAngle(6.283185307179586 * i / nv + g.range(-0.3, 0.3), R * g.range(0.5, 1.5)));
            p = V[0];
            for (int i = 0; i < nv; ++i) {
                Vec2 q = V[(i + 1) % nv]; Vec2 tgt = q;
                if (g.uni() < 0.6) { int m = g.irange(1, 15); Vec2 back = q - (q - p).normalized() * (m * tol * 0.5); k.spans.push_back(LineSpan{p, back}); p = back; h = (q - p).angle(); for (int j = 0; j < m - 1; ++j) { Vec2 r2 = p + Vec2::fromAngle(g.range(-3.14, 3.14), tinyLen(g, tol) * 0.4); if (r2 != p) { k.spans.push_back(LineSpan{p, r2}); p = r2; } } if (p != tgt) k.spans.push_back(LineSpan{p, tgt}); }
                else k.spans.push_back(LineSpan{p, tgt});
                p = tgt;
            }
            k.closed = true; break; }
        default: {  // tiny arcs of any radius/sweep sprinkled along a path with Beziers
            int nv = g.irange(2, 6);
            for (int i = 0; i < nv; ++i) {
                line(g.range(0.1, 2));
                double u = g.uni();
                if (u < 0.4) { double sw = g.range(-6.2, 6.2); double L = tinyLen(g, tol); double r = L / std::max(1e-6, std::fabs(sw));
                    Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * r : p - left * r;
                    ArcSpan a{ctr, r, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw; k.spans.push_back(a); p = a.endPoint(); h += sw; }
                else if (u < 0.7) { Vec2 q = p + Vec2::fromAngle(h + g.range(-2, 2), g.range(0.05, 1)); Vec2 c1 = p + Vec2::fromAngle(h, tinyLen(g, tol)); Vec2 c2 = q + Vec2::fromAngle(g.range(-3, 3), g.range(0, 0.5)); k.spans.push_back(BezierSpan{p, c1, c2, q}); h = (q - c2).angle(); p = q; }
                h += g.range(-2, 2);
            }
            line(g.range(0.1, 2));
        }
    }
    if (g.uni() < 0.2) { Vec2 t{g.range(-1e3, 1e3), g.range(-1e3, 1e3)}; translate(k, t); }
    return c;
}

struct Tot { long strict = 0; long n = 0, perJ[3] = {0, 0, 0}, perS[2] = {0, 0}, closed = 0, bez = 0, arcs = 0; long cside = 0, copen = 0; double csided = 0, copend = 0; long cruns = 0, cexm = 0, ccoll = 0, ccollpts = 0, cexl = 0; double cexd = 0, ccolld = 0; long vdv = 0, vrv = 0, vcm = 0, vtie = 0, xdv = 0, xrv = 0, xcm = 0, xtie = 0, xthin = 0, xex = 0, xlong = 0, xunres = 0, empty = 0; double vU = 0, vO = 0, xU = 0, xO = 0, xMiss = 0, xMissLen = 0, xTieLen = 0, cpu = 0; };
static bool hasArc(const Contour& c) { for (auto& s : c.spans) if (std::holds_alternative<ArcSpan>(s)) return true; return false; }

static bool continuous(const Contour& c) {
    auto sp = [](const Span& s) { return startPoint(s); }; auto ep = [](const Span& s) { return endPoint(s); };
    for (size_t i = 0; i + 1 < c.spans.size(); ++i) if (distance(ep(c.spans[i]), sp(c.spans[i + 1])) > 1e-9) return false;
    if (c.closed && !c.spans.empty() && distance(ep(c.spans.back()), sp(c.spans.front())) > 1e-9) return false;
    return true;
}
static long g_discont = 0;
static void judge(const char* tag, std::uint64_t seed, const Contour& in, double d, OffsetSide side, JoinType join, double tol, double mitre, Tot& t, bool verbose, FILE* dump) {
    if (!continuous(in)) { g_discont++; return; }
    { bool st = false; for (auto& sp : in.spans) if (!std::holds_alternative<BezierSpan>(sp) && length(sp) <= tol) st = true; t.strict += st; }
    t.n++; t.perJ[(int)join]++; t.perS[(int)side]++; t.closed += in.closed; t.bez += hasBez(in); t.arcs += hasArc(in);
    double t0 = cpuNow();
    auto out = offsetOpenContour(in, d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre);
    t.cpu += cpuNow() - t0;
    if (out.empty()) t.empty++;
    double tA = cpuNow(); Contour ck = polyOf(in, tol); double tB = cpuNow();
    Metrics m; g_covTie = 0.0005; check(ck, out, d, side, join, tol, mitre, m, true); g_tiePts.clear(); double tC = cpuNow();
    xk::Res r; xk::Checker X(ck, out, d, side, join, tol, mitre); X.run(r); double tD = cpuNow();
    if (std::getenv("PROF")) std::printf("prof offset %.2f poly %.2f vchk %.2f xk %.2f\n", tA - t0, tB - tA, tC - tB, tD - tC);
    t.vdv += m.distViol > 0; t.vrv += m.roundViol > 0; t.vcm += m.covMiss > 0; t.vtie += m.covMissTie > 0;
    t.vU = std::max(t.vU, m.worstUnder); t.vO = std::max(t.vO, m.worstOver);
    t.xdv += r.dv > 0; t.xrv += r.rv > 0; t.xcm += r.cmiss > 0; t.xtie += r.cmissTie > 0; t.xthin += r.cmissThin > 0; t.xex += r.excess > 0; t.xlong += r.cmissLong > 0; t.xunres += r.unres > 0;
    t.cside += r.cSideMiss > 0; t.copen += r.cOpenEndMiss > 0; t.csided = std::max(t.csided, r.cSideDout); t.copend = std::max(t.copend, r.cOpenEndDout);
    t.cruns += r.cRuns > 0; t.cexm += r.cExemptMiss > 0; t.ccoll += r.cCollMiss > 0; t.ccollpts += r.cCollPts; t.cexl += r.cExemptLoci; t.cexd = std::max(t.cexd, r.cExemptDout); t.ccolld = std::max(t.ccolld, r.cCollDout);
    t.xU = std::max(t.xU, r.under); t.xO = std::max(t.xO, r.over); t.xMiss = std::max(t.xMiss, r.miss); t.xMissLen = std::max(t.xMissLen, r.missLen); t.xTieLen = std::max(t.xTieLen, r.missTieLen);
    bool core = r.dv || r.rv || r.cmiss || r.cmissTie || r.cmissLong || r.cSideMiss || r.excess || m.distViol || m.roundViol;
    bool tie = m.covMissTie || r.cmissTie || r.cmissThin;
    if (core || (verbose && tie)) {
        std::printf("%s %s seed=0x%llx n=%zu st=%d closed=%d d=%.6g %s side=%d mitre=%.3g | v: dv%d u%.3g rv%d o%.3g cm%d miss%.3g tie%d | x: dv%ld u%.3g rv%ld o%.3g cm%ld miss%.3g len%.3g tie%ld tlen%.3g thin%ld long%ld | c: runs%ld exm%ld exd%.3g coll%ld side%ld sided%.3g | %s | %s %s | %s | %s\n",
            core ? "CORE" : "TIE", tag, (unsigned long long)seed, in.spans.size(), stN(in, tol), in.closed, d, jn(join), (int)side, mitre,
            m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.covMiss, m.worstMiss, m.covMissTie, r.dv, r.under, r.rv, r.over, r.cmiss, r.miss, r.missLen, r.cmissTie, r.missTieLen, r.cmissThin, r.cmissLong, r.cRuns, r.cExemptMiss, r.cExemptDout, r.cCollMiss, r.cSideMiss, r.cSideDout, m.firstIssue.c_str(), r.first.c_str(), r.thinFirst.c_str(), r.cFirst.c_str(), r.cSideFirst.c_str());
        if (core && dump) { std::fprintf(dump, "// %s seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g tol=%g\n", tag, (unsigned long long)seed, d, jn(join), side == OffsetSide::Left ? "Left" : "Right", mitre, tol); dumpIn(in, dump); std::fflush(dump); }
        std::fflush(stdout);
    }
}
static void summary(const char* tag, Tot& t) {
    std::printf("SUMMARY %s discontinuous-skipped %ld strict(line/arc sub-tol) %ld inputs %ld (R %ld Sh %ld Sq %ld | L %ld R %ld | closed %ld bez %ld arcs %ld | empty %ld) | vchk: dist %ld round %ld cov %ld tie %ld (under %.3g over %.3g) | xk: excess %ld dist %ld round %ld cov %ld tie %ld thin(exempt) %ld COVVIOL(non-thin,stretch>tol) %ld unres %ld (under %.3g over %.3g miss %.3g misslen %.3g tielen %.3g) | (c): inputs-with-runs %ld exempt-loci %ld inputs-with-exempt-misses %ld (max dout %.3g) collapsed-pts %ld collapsed-miss-inputs %ld (max dout %.3g) SIDE-miss-inputs %ld (max dout %.3g) open-end-exempt-miss-inputs %ld (max dout %.3g) | cpu %.1f\n",
        tag, g_discont, t.strict, t.n, t.perJ[0], t.perJ[1], t.perJ[2], t.perS[0], t.perS[1], t.closed, t.bez, t.arcs, t.empty, t.vdv, t.vrv, t.vcm, t.vtie, t.vU, t.vO, t.xex, t.xdv, t.xrv, t.xcm, t.xtie, t.xthin, t.xlong, t.xunres, t.xU, t.xO, t.xMiss, t.xMissLen, t.xTieLen, t.cruns, t.cexl, t.cexm, t.cexd, t.ccollpts, t.ccoll, t.ccolld, t.cside, t.csided, t.copen, t.copend, t.cpu);
}

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "";
    FILE* dump = std::getenv("DUMP") ? std::fopen(std::getenv("DUMP"), "a") : nullptr;
    bool verbose = std::getenv("VERB") != nullptr;
    if (mode == "st") {  // v9 injector inputs holding sub-tol spans
        long N = std::atol(argv[2]); std::uint64_t s0 = std::strtoull(argv[3], nullptr, 0); Tot t;
        for (long i = 0; i < N; ++i) { std::uint64_t seed = s0 + i; C9 cc = make9(seed); Case& c = cc.c; if (c.in.spans.empty() || !stN(c.in, c.tol)) continue; judge("v9", seed, c.in, c.d, c.side, c.join, c.tol, c.mitre, t, verbose, dump); }
        summary("v9", t); return 0;
    }
    if (mode == "g2") {
        long N = std::atol(argv[2]); std::uint64_t s0 = std::strtoull(argv[3], nullptr, 0); Tot t; long perK[8] = {0};
        for (long i = 0; i < N; ++i) { std::uint64_t seed = s0 + i; G2 c = makeG2(seed); if (c.in.spans.empty() || !stN(c.in, c.tol)) continue; perK[c.kind]++; judge("g2", seed, c.in, c.d, c.side, c.join, c.tol, c.mitre, t, verbose, dump); }
        std::printf("g2 kinds:"); for (int k = 0; k < 7; ++k) std::printf(" %ld", perK[k]); std::printf("\n");
        summary("g2", t); return 0;
    }
    if (mode == "g2one" || mode == "v9one") {
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0); Tot t; Contour in; double d, mitre, tol; OffsetSide side; JoinType join;
        if (mode == "g2one") { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; side = c.side; join = c.join; }
        else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; side = cc.c.side; join = cc.c.join; }
        if (argc > 3) { int j = std::atoi(argv[3]); if (j >= 0) join = (JoinType)j; }
        if (argc > 4) { int s = std::atoi(argv[4]); if (s >= 0) side = (OffsetSide)s; }
        if (std::getenv("POLY")) in = polyOf(in, tol);
        dumpIn(in, stdout); std::printf("d=%.17g join=%s side=%d mitre=%.17g\n", d, jn(join), (int)side, mitre);
        judge(mode.c_str(), seed, in, d, side, join, tol, mitre, t, true, nullptr);
        auto out = offsetOpenContour(in, d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre);
        for (size_t i = 0; i < out.size(); ++i) { std::printf("CHAIN %zu closed %d spans %zu\n", i, out[i].closed, out[i].spans.size()); if (std::getenv("OUTS")) for (auto& s : out[i].spans) { auto q = xk::mk(s); std::printf("  k%d (%.17g,%.17g)->(%.17g,%.17g) r %.6g sw %.6g len %.3g\n", q.k, q.a.x, q.a.y, q.b.x, q.b.y, q.r, q.sw, q.len); } }
        return 0;
    }
    if (mode == "mainone") {  // v9 seed: branch vs main (namespaced) on the polygonised input, both judged by xk
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0); int src = argc > 3 ? std::atoi(argv[3]) : 1;
        Contour in; double d, mitre, tol; OffsetSide side; JoinType join;
        if (src == 0) { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; side = c.side; join = c.join; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; side = cc.c.side; join = cc.c.join; }
        if (argc > 4) { int j = std::atoi(argv[4]); if (j >= 0) join = (JoinType)j; }
        if (argc > 5) { int s = std::atoi(argv[5]); if (s >= 0) side = (OffsetSide)s; }
        Contour ck = polyOf(in, tol);
        auto ob = offsetOpenContour(ck, d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre);
        auto mp = mainOffset(toP(ck), d, side == OffsetSide::Left ? 0 : 1, (int)join, tol, mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
        for (int w = 0; w < 2; ++w) { auto& o = w ? om : ob; xk::Res r; xk::Checker(ck, o, d, side, join, tol, mitre).run(r);
            std::printf("%s: %zu chains [", w ? "main  " : "branch", o.size()); for (auto& c : o) std::printf(" %s%zu", c.closed ? "closed:" : "open:", c.spans.size());
            std::printf(" ] xk excess %ld (%.3g) dist %ld (u %.3g) round %ld (o %.3g) cov core %ld tie %ld thin %ld viol %ld (len %.3g/%.3g) coll-miss %ld | %s | %s\n", r.excess, r.worstExcess, r.dv, r.under, r.rv, r.over, r.cmiss, r.cmissTie, r.cmissThin, r.cmissLong, r.missLen, r.missTieLen, r.cCollMiss, r.first.c_str(), r.covFirst.c_str());
            if (std::getenv("OUTS")) for (auto& c : o) { std::printf("  chain closed %d\n", c.closed); for (auto& sp : c.spans) { auto q = xk::mk(sp); std::printf("    k%d (%.17g,%.17g)->(%.17g,%.17g) c(%.10g,%.10g) r %.6g sw %.6g len %.3g\n", q.k, q.a.x, q.a.y, q.b.x, q.b.y, q.c.x, q.c.y, q.r, q.sw, q.len); } } }
        return 0;
    }
    if (mode == "libonly") {  // libonly <g2|v9> <seed> [join side] [main]: just the library call (crash isolation)
        std::uint64_t seed = std::strtoull(argv[3], nullptr, 0); Contour in; double d, mitre, tol; OffsetSide side; JoinType join;
        if (argv[2][0] == 'g') { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; side = c.side; join = c.join; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; side = cc.c.side; join = cc.c.join; }
        if (argc > 4) join = (JoinType)std::atoi(argv[4]); if (argc > 5) side = (OffsetSide)std::atoi(argv[5]);
        std::printf("input %zu spans closed %d st %d d=%.17g %s side=%d mitre=%.17g\n", in.spans.size(), in.closed, stN(in, tol), d, jn(join), (int)side, mitre); std::fflush(stdout);
        if (argc > 6) { std::printf("main (raw input, Beziers polygonised by polyOf):\n"); std::fflush(stdout); auto mp = mainOffset(toP(polyOf(in, tol)), d, (int)side, (int)join, tol, mitre); std::printf("main ok: %zu chains\n", mp.size()); return 0; }
        auto o = offsetOpenContour(in, d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre); std::printf("branch raw input ok: %zu chains\n", o.size()); std::fflush(stdout);
        auto o2 = offsetOpenContour(polyOf(in, tol), d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre); std::printf("branch polygonised ok: %zu chains\n", o2.size());
        return 0;
    }
    if (mode == "cl") {  // decision (c) "closed loops stay closed (as on main)": closed inputs with sub-tol spans, every join x side
        long N = std::atol(argv[2]); std::uint64_t s0 = std::strtoull(argv[3], nullptr, 0); int src = std::atoi(argv[4]);
        long n = 0, fewer = 0, more = 0, openWhereMainClosed = 0;
        for (long i = 0; i < N; ++i) {
            std::uint64_t seed = s0 + i; Contour in; double d, mitre, tol;
            if (src == 0) { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; }
            if (!in.closed || in.spans.empty() || !stN(in, tol) || !continuous(in)) continue;
            Contour ck = polyOf(in, tol);
            for (int j = 0; j < 3; ++j) for (int s = 0; s < 2; ++s) {
                auto ob = offsetOpenContour(ck, d, (OffsetSide)s, (JoinType)j, Tolerance{tol, 1e-9, 0.01}, mitre);
                auto mp = mainOffset(toP(ck), d, s, j, tol, mitre);
                int cb = 0, cm = 0, ob_open = 0, om_open = 0; for (auto& c : ob) { cb += c.closed; ob_open += !c.closed; } for (auto& c : mp) { cm += c.closed; om_open += !c.closed; }
                n++;
                if (cb < cm) { fewer++; std::printf("FEWER-CLOSED %s 0x%llx %s side=%d d=%.6g branch closed %d open %d | main closed %d open %d\n", src ? "v9" : "g2", (unsigned long long)seed, jn((JoinType)j), s, d, cb, ob_open, cm, om_open); }
                if (cb > cm) more++;
                if (cm > 0 && cb == 0 && ob_open > 0) openWhereMainClosed++;
            }
        }
        std::printf("SUMMARY cl src %d offsets %ld branch-fewer-closed %ld branch-more-closed %ld branch-only-open-where-main-closed %ld\n", src, n, fewer, more, openWhereMainClosed);
        return 0;
    }
    if (mode == "re") {  // recheck lines "<tag> <seed> <join> <side>" from stdin: branch vs main (xk), worst points vs main output
        char tg[16], sd[32]; int j, s;
        while (std::scanf("%15s %31s %d %d", tg, sd, &j, &s) == 4) {
            std::uint64_t seed = std::strtoull(sd, nullptr, 0); Contour in; double d, mitre, tol;
            if (tg[0] == 'g') { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; }
            JoinType join = (JoinType)j; OffsetSide side = (OffsetSide)s; Contour ck = polyOf(in, tol);
            auto ob = offsetOpenContour(ck, d, side, join, Tolerance{tol, 1e-9, 0.01}, mitre);
            auto mp = mainOffset(toP(ck), d, s, j, tol, mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
            xk::Res rb, rm; xk::Checker XB(ck, ob, d, side, join, tol, mitre); XB.run(rb); xk::Checker XM(ck, om, d, side, join, tol, mitre); XM.run(rm);
            double sideToMain = rb.cSideMiss ? XM.fOut(rb.sideP, d + 1) : -1, covToMain = rb.hasCovP ? XM.fOut(rb.covP, d + 1) : -1;
            double sideToBr = rb.cSideMiss ? XB.fOut(rb.sideP, d + 1) : -1, covToBr = rb.hasCovP ? XB.fOut(rb.covP, d + 1) : -1;
            auto cl = [](const xk::Res& r) { char b[300]; snprintf(b, sizeof b, "dv%ld(u%.2g) rv%ld(o%.2g) cov%ld+tie%ld(len %.3g) side%ld(%.3g) coll%ld chains", r.dv, r.under, r.rv, r.over, r.cmiss, r.cmissTie, std::max(r.missLen, r.missTieLen), r.cSideMiss, r.cSideDout, r.cCollMiss); return std::string(b); };
            std::printf("RE %s %s %s side=%d d=%.6g closed=%d | branch %s %zu | main %s %zu | covP->branch %.4g main %.4g | sideP->branch %.4g main %.4g | covP (%.17g,%.17g) sideP (%.17g,%.17g)\n", tg, sd, jn(join), s, d, ck.closed,
                cl(rb).c_str(), ob.size(), cl(rm).c_str(), om.size(), covToBr, covToMain, sideToBr, sideToMain, rb.covP.x, rb.covP.y, rb.sideP.x, rb.sideP.y);
            std::fflush(stdout);
        }
        return 0;
    }
    if (mode == "shrink") {  // v9 seed [join side]: drop spans from the ends while a coverage violation (non-thin, > tol) remains
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0); C9 cc = make9(seed); Case c = cc.c;
        if (argc > 3) c.join = (JoinType)std::atoi(argv[3]); if (argc > 4) c.side = (OffsetSide)std::atoi(argv[4]);
        Contour cur = polyOf(c.in, c.tol);
        auto bad = [&](const Contour& k) { auto o = offsetOpenContour(k, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre); xk::Res r; xk::Checker(k, o, c.d, c.side, c.join, c.tol, c.mitre).run(r); return r.cmissLong > 0; };
        if (!bad(cur)) { std::printf("not bad\n"); return 0; }
        if (cur.closed) { Contour t = cur; t.closed = false; if (bad(t)) cur = t; }
        bool prog = true;
        while (prog && !cur.closed) { prog = false;
            for (int e = 0; e < 2 && cur.spans.size() > 1; ++e) { Contour t = cur; if (e == 0) t.spans.erase(t.spans.begin()); else t.spans.pop_back(); if (bad(t)) { cur = t; prog = true; } } }
        std::printf("// v9 seed 0x%llx shrunk: d=%.17g join=%s side=%s mitre=%.17g, %zu spans\n", (unsigned long long)seed, c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre, cur.spans.size());
        dumpIn(cur, stdout);
        auto o = offsetOpenContour(cur, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre); xk::Res r; xk::Checker(cur, o, c.d, c.side, c.join, c.tol, c.mitre).run(r);
        std::printf("// branch: %s\n", r.covFirst.c_str());
        return 0;
    }
    if (mode == "allj") {  // every join and side for g2 / v9 inputs (each input judged 6 ways)
        long N = std::atol(argv[2]); std::uint64_t s0 = std::strtoull(argv[3], nullptr, 0); int src = std::atoi(argv[4]); Tot t;
        for (long i = 0; i < N; ++i) {
            std::uint64_t seed = s0 + i; Contour in; double d, mitre, tol;
            if (src == 0) { G2 c = makeG2(seed); in = c.in; d = c.d; mitre = c.mitre; tol = c.tol; } else { C9 cc = make9(seed); in = cc.c.in; d = cc.c.d; mitre = cc.c.mitre; tol = cc.c.tol; }
            if (in.spans.empty() || !stN(in, tol)) continue;
            for (int j = 0; j < 3; ++j) for (int s = 0; s < 2; ++s) judge(src ? "v9all" : "g2all", seed, in, d, (OffsetSide)s, (JoinType)j, tol, mitre, t, verbose, dump);
        }
        summary(src ? "v9all" : "g2all", t); return 0;
    }
    if (mode == "cmp") {  // AC3: inputs without sub-tol spans, branch vs main (xk + vchk metrics), bit-identity
        long N = std::atol(argv[2]); std::uint64_t s0 = std::strtoull(argv[3], nullptr, 0); int fam = std::atoi(argv[4]);
        long n = 0, same = 0, worse = 0, better = 0, bothBad = 0, cnt = 0; double cpuB = 0, cpuM = 0;
        for (long i = 0; i < N; ++i) {
            std::uint64_t seed = s0 + i; Case c;
            if (fam == 7) { g_tinyMul = 2.0; g_tinyAdd = 1.000001e-3; c = make9(seed).c; } else if (fam <= 1) c = makeCase(seed); else c = makeCaseF(seed, fam).c;
            if (c.in.spans.empty() || stN(c.in, c.tol) || hasBez(c.in)) continue;
            n++;
            double t0 = cpuNow(); auto ob = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre); double t1 = cpuNow();
            auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre); double t2 = cpuNow();
            cpuB += t1 - t0; cpuM += t2 - t1;
            std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
            bool eq = ob.size() == om.size(); for (size_t k = 0; eq && k < ob.size(); ++k) eq = ob[k].closed == om[k].closed && ob[k].spans == om[k].spans;
            if (eq) { same++; continue; }
            cnt++;
            Metrics mb, mm; g_covTie = 0.0005; check(c.in, ob, c.d, c.side, c.join, c.tol, c.mitre, mb, true); g_tiePts.clear(); check(c.in, om, c.d, c.side, c.join, c.tol, c.mitre, mm, true); g_tiePts.clear();
            xk::Res rb, rm; xk::Checker(c.in, ob, c.d, c.side, c.join, c.tol, c.mitre).run(rb); xk::Checker(c.in, om, c.d, c.side, c.join, c.tol, c.mitre).run(rm);
            auto oth = [](const Metrics& m) { return m.lineJunctionNotBitwise + m.arcGapBeyond + m.coderBack + m.overlapIn + m.overlapX + m.shortBack + m.selfX + m.zeroLen + m.subTolAtEnd + m.badSpan; };
            int bB = (rb.dv > 0) + (rb.rv > 0) + (rb.cmiss > 0) + (rb.cmissTie > 0) + (mb.distViol > 0) + (mb.roundViol > 0) + (mb.covMiss > 0) + (mb.covMissTie > 0) + (oth(mb) > 0);
            int bM = (rm.dv > 0) + (rm.rv > 0) + (rm.cmiss > 0) + (rm.cmissTie > 0) + (mm.distViol > 0) + (mm.roundViol > 0) + (mm.covMiss > 0) + (mm.covMissTie > 0) + (oth(mm) > 0);
            // worse: any metric magnitude worse than main beyond tol/1000 (under/over/miss)
            bool w = rb.under > std::max(rm.under, xk::allowance(c.d, c.tol)) + 1e-12 || (c.join == JoinType::Round && rb.over > std::max(rm.over, xk::allowance(c.d, c.tol)) + 1e-12)
                  || (rb.cmiss + rb.cmissTie > 0 && std::max(rb.miss, rb.missLen) > std::max(rm.miss, rm.missLen) + 1e-9) || oth(mb) > oth(mm) || bB > bM;
            bool b = bM > bB;
            worse += w; better += b && !w; bothBad += bB && bM;
            std::printf("%s fam%d seed=0x%llx n=%zu closed=%d d=%.6g %s side=%d | br: x[dv%ld u%.3g rv%ld o%.3g cm%ld m%.3g ml%.3g tie%ld tl%.3g] v[%s oth%d] | main: x[dv%ld u%.3g rv%ld o%.3g cm%ld m%.3g ml%.3g tie%ld tl%.3g] v[%s oth%d]\n",
                w ? "WORSE" : (b ? "BETTER" : "DIFF"), fam, (unsigned long long)seed, c.in.spans.size(), c.in.closed, c.d, jn(c.join), (int)c.side,
                rb.dv, rb.under, rb.rv, rb.over, rb.cmiss, rb.miss, rb.missLen, rb.cmissTie, rb.missTieLen, mb.firstIssue.c_str(), oth(mb),
                rm.dv, rm.under, rm.rv, rm.over, rm.cmiss, rm.miss, rm.missLen, rm.cmissTie, rm.missTieLen, mm.firstIssue.c_str(), oth(mm));
            if (w && dump) { std::fprintf(dump, "// cmp fam%d seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g\n", fam, (unsigned long long)seed, c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre); dumpIn(c.in, dump); }
            std::fflush(stdout);
        }
        std::printf("SUMMARY cmp fam%d N=%ld s0=0x%llx inputs %ld identical %ld differ %ld worse %ld better %ld bothflagged %ld | cpu branch %.2f main %.2f\n", fam, N, (unsigned long long)s0, n, same, cnt, worse, better, bothBad, cpuB, cpuM);
        return 0;
    }
    return v9_main(argc, argv);
}
