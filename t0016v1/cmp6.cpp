// T-0016 verifier AC2: branch vs main (namespaced) on the T-0006 fuzz families and later adversarial families,
// every join x side; bit identity; otherwise exact checker (xk) + vchk metrics for both and a two-way
// sampled distance between the outputs (brute force, exact span distance).
// usage: cmp6 <fam> <N> <seed0>    fam: 0 makeCase (T-0006), 2 makeCase2, 3 makeCase3, 4 makeCase4, 5 makeCase5,
//        20 dense zig-zag / hatch family (this verifier), 21 same with jitter / rotation / translation
#include "v9lib.cpp"
#include "xchk.hpp"
static void dumpIn(const Contour& k, FILE* f) {
    std::fprintf(f, "Contour in; in.closed = %s;\n", k.closed ? "true" : "false");
    for (auto& sp : k.spans) if (auto* l = std::get_if<LineSpan>(&sp)) std::fprintf(f, "in.spans.emplace_back(LineSpan{{%.17g, %.17g}, {%.17g, %.17g}});\n", l->start.x, l->start.y, l->end.x, l->end.y);
}
using carvetto::geometry::testing::distanceToSpanExact;
static double sideDist(const std::vector<Contour>& A, const std::vector<Contour>& B, Vec2& at) {
    double worst = 0;
    for (auto& c : A) for (auto& s : c.spans) for (int k = 0; k <= 16; ++k) {
        Vec2 q = pointAt(s, k / 16.0); double best = 1e300;
        for (auto& c2 : B) for (auto& s2 : c2.spans) { best = std::min(best, distanceToSpanExact(q, s2)); if (best == 0) break; }
        if (best > worst) { worst = best; at = q; }
    }
    return worst;
}
Case makeZ(std::uint64_t seed, bool noisy) {
    Rng g(seed * 0x9E3779B97F4A7C15ULL + 0x16);
    Case c; c.tol = 0.001; c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 6.0);
    c.join = (JoinType)g.irange(0, 2); c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    int kind = g.irange(0, 4); int n = g.irange(6, 120);
    double p = std::exp(g.range(std::log(0.005), std::log(0.5))), a = p * std::exp(g.range(std::log(0.5), std::log(100.0)));
    double jy = noisy ? std::pow(10.0, g.range(-14, -6)) : 0.0;
    std::vector<Vec2> pts{{0, 0}};
    if (kind == 0) { for (int i = 1; i <= n; ++i) pts.push_back({i * p + (noisy ? g.range(-jy, jy) : 0), (i % 2) * a + (noisy ? g.range(-jy, jy) : 0)}); }
    else if (kind == 1) { for (int i = 1; i <= n; ++i) pts.push_back({(i / 2) * p, (i % 2) * a + (noisy ? g.range(-jy, jy) : 0)}); }  // sawtooth (vertical drops)
    else if (kind == 2) { for (int i = 0; i < n / 2; ++i) { pts.push_back({(i % 2) ? 0.0 : a, i * p}); pts.push_back({(i % 2) ? 0.0 : a, (i + 1) * p}); } }  // boustrophedon hatch
    else if (kind == 3) { double amp2 = a * g.range(0.2, 1.0); for (int i = 1; i <= n; ++i) pts.push_back({i * p, (i % 2) * ((i % 4 == 1) ? a : amp2)}); }  // two amplitudes
    else { for (int i = 1; i <= n; ++i) pts.push_back({i * p * g.range(0.7, 1.3), (i % 2) * a * g.range(0.9, 1.1)}); }  // irregular
    bool closed = g.uni() < 0.25;
    if (closed && kind != 2) { Vec2 l = pts.back(); double B = -g.range(0.1, 3.0) * a; pts.push_back({l.x, B}); pts.push_back({0, B}); }
    c.in = fromPts(pts, closed);
    if (closed && kind == 2) c.in.closed = false;
    double dm = std::exp(g.range(std::log(0.3), std::log(40.0)));
    c.d = std::max(p, std::min(a, 50.0)) * dm * 0.5;
    if (c.d > 60) c.d = 60;
    double u = g.uni();
    if (noisy || u < 0.3) transformC(c.in, g.uni() < 0.5 ? 0.0 : g.range(-kPi, kPi), g.uni() < 0.5 ? Vec2{0, 0} : Vec2{g.range(-1000, 1000), g.range(-1000, 1000)});
    return c;
}
// 22: zig-zag with a random subset of vertices jittered by 1e-9..1e-5 (copies on nearly, not exactly, one carrier:
//     mergeOnCarriers groups them by `lateral`, pruneLineCopies only by `overlapping`).
// 23: S-hatch: zig-zag forwards, then back above it so its valley chamfers lie on the forward peak chamfer line
//     (copies running opposite ways), the gap perturbed by 0 / ulps / 1e-12 / 1e-9 / 1e-6.
Case makeZ2(std::uint64_t seed, int fam) {
    Rng g(seed * 0xD1B54A32D192ED03ULL + 0x22);
    Case c; c.tol = 0.001; c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 6.0);
    c.join = (JoinType)g.irange(0, 2); c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    int n = g.irange(6, 80);
    double p = std::exp(g.range(std::log(0.005), std::log(0.5))), a = p * std::exp(g.range(std::log(0.5), std::log(60.0)));
    c.d = std::min(60.0, std::max(p, a) * std::exp(g.range(std::log(0.3), std::log(20.0))) * 0.5);
    std::vector<Vec2> pts{{0, 0}};
    if (fam == 26) {  // every vertex jittered by 1e-12..1e-9 (copies tilted within sameCarrier: carriers cross inside stretches), d >> pitch
        c.d = std::min(60.0, std::max(p, a) * std::exp(g.range(std::log(2.0), std::log(30.0))));
        double jy = std::pow(10.0, g.range(-12, -9)) * std::max(1.0, a);
        for (int i = 1; i <= n; ++i) pts.push_back({i * p + g.range(-jy, jy), (i % 2) * a + g.range(-jy, jy)});
    } else if (fam == 25) {  // zig-zag with some taller or deeper teeth (their neighbourhoods and corner regions cut the shared chamfer line)
        double frac = g.range(0.02, 0.3);
        for (int i = 1; i <= n; ++i) { Vec2 q{i * p, (i % 2) * a}; if (g.uni() < frac) q.y += (i % 2 ? 1 : -1) * g.range(0.0, 2.0 * c.d) * (g.uni() < 0.5 ? 1 : -1); pts.push_back(q); }
    } else if (fam == 24) {  // irregular pitch, one amplitude (chamfers of different lengths on one line), subset tilted by jitter
        double frac = g.range(0.05, 0.5), jy = std::pow(10.0, g.range(-8, -5)); double x = 0;
        for (int i = 1; i <= n; ++i) { x += p * g.range(0.3, 3.0); Vec2 q{x, (i % 2) * a}; if (g.uni() < frac) { q.y += g.range(-jy, jy); q.x += g.range(-jy, jy) * p; } pts.push_back(q); }
    } else if (fam == 22) {
        double frac = g.range(0.05, 0.6), jy = std::pow(10.0, g.range(-9, -5));
        for (int i = 1; i <= n; ++i) { Vec2 q{i * p, (i % 2) * a}; if (g.uni() < frac) { q.y += g.range(-jy, jy); if (g.uni() < 0.5) q.x += g.range(-jy, jy); } pts.push_back(q); }
    } else {
        for (int i = 1; i <= n; ++i) pts.push_back({i * p, (i % 2) * a});
        double k = g.uni() < 0.5 ? 1.0 : c.mitre;
        double pert[] = {0, 2e-16, -2e-16, 1e-15, -1e-15, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6};
        double Y0 = a + 2 * c.d * k + pert[g.irange(0, 10)] * std::max(1.0, a + 2 * c.d);
        c.side = OffsetSide::Left;  // forward peaks' chamfers on the left at a + d; the return runs -x, its left is -y
        double xr = pts.back().x + p * g.range(0.0, 2.0); int m = g.irange(4, n + 4);
        double sh = g.uni() < 0.5 ? 0.0 : p * g.range(0, 1);
        for (int i = 0; i <= m; ++i) pts.push_back({xr - i * p - sh * (i > 0), Y0 + (i % 2) * a});
        if (g.uni() < 0.3) c.side = OffsetSide::Right;
    }
    c.in = fromPts(pts, false);
    if (g.uni() < 0.3) transformC(c.in, g.range(-kPi, kPi), Vec2{g.range(-1000, 1000), g.range(-1000, 1000)});
    return c;
}
int main(int argc, char** argv) {
    int fam = atoi(argv[1]); long N = atol(argv[2]); std::uint64_t s0 = strtoull(argv[3], nullptr, 0);
    long n = 0, same = 0, diff = 0, geomSame = 0, metricsSame = 0, worse = 0, better = 0, skipped = 0; double cb = 0, cm = 0;
    for (long i = 0; i < N; ++i) {
        std::uint64_t seed = s0 + i; Case c;
        if (fam == 0) c = makeCase(seed); else if (fam == 2) c = makeCase2(seed).c; else if (fam == 3) c = makeCase3(seed).c;
        else if (fam == 4) c = makeCase4(seed).c; else if (fam == 5) c = makeCase5(seed).c; else if (fam >= 22) c = makeZ2(seed, fam); else c = makeZ(seed, fam == 21);
        if (c.in.spans.empty() || hasBez(c.in)) { skipped++; continue; }
        for (int j = 0; j < 3; ++j) for (int sd = 0; sd < 2; ++sd) {
            n++; OffsetSide side = (OffsetSide)sd; JoinType join = (JoinType)j;
            double t0 = cpuNow(); auto ob = offsetOpenContour(c.in, c.d, side, join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre); double t1 = cpuNow();
            auto mp = mainOffset(toP(c.in), c.d, sd, j, c.tol, c.mitre); double t2 = cpuNow(); cb += t1 - t0; cm += t2 - t1;
            std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
            bool eq = ob.size() == om.size(); for (size_t k = 0; eq && k < ob.size(); ++k) eq = ob[k].closed == om[k].closed && ob[k].spans == om[k].spans;
            if (eq) { same++; continue; }
            diff++;
            Vec2 a1, a2; double hb = sideDist(ob, om, a1), hm = sideDist(om, ob, a2);
            double scale = 1; for (auto& s : c.in.spans) { Vec2 q = startPoint(s); scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), c.d}); }
            bool topo = ob.size() == om.size(); size_t spB = 0, spM = 0; for (size_t k = 0; k < ob.size(); ++k) spB += ob[k].spans.size(); for (size_t k = 0; k < om.size(); ++k) spM += om[k].spans.size();
            for (size_t k = 0; topo && k < ob.size(); ++k) topo = ob[k].closed == om[k].closed;
            bool gs = topo && hb <= 1e-9 * scale && hm <= 1e-9 * scale; geomSame += gs;
            Metrics mb, mmm; g_covTie = 0.0005; check(c.in, ob, c.d, side, join, c.tol, c.mitre, mb, true); g_tiePts.clear(); check(c.in, om, c.d, side, join, c.tol, c.mitre, mmm, true); g_tiePts.clear();
            xk::Res rb, rm; xk::Checker(c.in, ob, c.d, side, join, c.tol, c.mitre).run(rb); xk::Checker(c.in, om, c.d, side, join, c.tol, c.mitre).run(rm);
            auto oth = [](const Metrics& m) { return m.lineJunctionNotBitwise + m.arcGapBeyond + m.coderBack + m.overlapIn + m.overlapX + m.shortBack + m.selfX + m.zeroLen + m.subTolAtEnd + m.badSpan + m.reversals + m.hardRev; };
            auto key = [&](const xk::Res& r, const Metrics& m) { char b[512]; snprintf(b, sizeof b, "x[ex%ld dv%ld rv%ld cm%ld tie%ld thin%ld long%ld unres%ld] v[dv%d rv%d cm%d tie%d jnb%d agb%d back%d ovi%d ovx%d sb%d sx%d zl%d ste%d bad%d rev%d hr%d]", r.excess, r.dv, r.rv, r.cmiss, r.cmissTie, r.cmissThin, r.cmissLong, r.unres, m.distViol, m.roundViol, m.covMiss, m.covMissTie, m.lineJunctionNotBitwise, m.arcGapBeyond, m.coderBack, m.overlapIn, m.overlapX, m.shortBack, m.selfX, m.zeroLen, m.subTolAtEnd, m.badSpan, m.reversals, m.hardRev); return std::string(b); };
            std::string kb = key(rb, mb), km = key(rm, mmm);
            bool ms = kb == km && std::fabs(rb.under - rm.under) <= 1e-12 * scale && std::fabs(rb.over - rm.over) <= 1e-12 * scale && std::fabs(mb.worstUnder - mmm.worstUnder) <= 1e-12 * scale;
            metricsSame += ms;
            const double al = xk::allowance(c.d, c.tol);
            bool w = rb.under > std::max(rm.under, al) + 1e-12 || (join == JoinType::Round && rb.over > std::max(rm.over, al) + 1e-12) ||
                     (rb.cmiss + rb.cmissTie > 0 && std::max(rb.miss, rb.missLen) > std::max(rm.miss, rm.missLen) + 1e-9) || oth(mb) > oth(mmm) ||
                     rb.dv > rm.dv || rb.rv > rm.rv || rb.cmiss > rm.cmiss || rb.cmissLong > rm.cmissLong || rb.excess > rm.excess || mb.distViol > mmm.distViol || mb.covMiss > mmm.covMiss;
            bool bt = !w && (oth(mb) < oth(mmm) || rb.dv < rm.dv || rb.cmiss < rm.cmiss || rb.excess < rm.excess);
            worse += w; better += bt;
            std::printf("%s%s fam%d seed=0x%llx n=%zu closed=%d d=%.6g %s side=%d mitre=%.3g | chains %zu/%zu spans %zu/%zu | hd br->main %.3g main->br %.3g (scale %.3g) at (%.17g,%.17g)/(%.17g,%.17g) | br u%.3g o%.3g %s | main u%.3g o%.3g %s\n",
                w ? "WORSE" : bt ? "BETTER" : "DIFF", gs ? (ms ? "-geomsame-metsame" : "-geomsame-METDIFF") : (ms ? "-GEOMDIFF-metsame" : "-GEOMDIFF-METDIFF"), fam, (unsigned long long)seed, c.in.spans.size(), c.in.closed, c.d, jn(join), sd, c.mitre,
                ob.size(), om.size(), spB, spM, hb, hm, scale, a1.x, a1.y, a2.x, a2.y, rb.under, rb.over, kb.c_str(), rm.under, rm.over, km.c_str());
            if (std::getenv("DUMPONE")) { dumpIn(c.in, stdout); std::printf("mitre=%.17g d=%.17g\n", c.mitre, c.d);
                for (int w = 0; w < 2; ++w) { auto& o = w ? om : ob; std::printf("%s:\n", w ? "MAIN" : "BRANCH"); for (auto& ch : o) { std::printf(" chain closed %d\n", ch.closed); for (auto& sp : ch.spans) { auto q = xk::mk(sp); std::printf("   k%d (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", q.k, q.a.x, q.a.y, q.b.x, q.b.y, q.len); } } } }
            std::fflush(stdout);
        }
    }
    std::printf("SUMMARY fam%d N=%ld s0=0x%llx offsets %ld identical %ld differ %ld (geom-same %ld, metrics-same %ld) worse %ld better %ld skipped-inputs %ld | cpu branch %.2f main %.2f\n",
        fam, N, (unsigned long long)s0, n, same, diff, geomSame, metricsSame, worse, better, skipped, cb, cm);
}
