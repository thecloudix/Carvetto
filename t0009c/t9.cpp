// T-0009 sub-tolerance fuzz driver (scratch). Reuses the T-0017 verifier's exact checker (vdrv4.cpp).
#define main vdrv4_main
#include "vdrv4.cpp"
#undef main

// Insert sub-tolerance spans into a continuous contour. Kinds:
//  0 jog: last tl of a line replaced by a tiny line in a random direction
//  1 collinear split of a line near its start or end
//  2 tiny arc replacing the last chord of a line (random sweep up to +-3 rad)
//  3 run of 2-6 tiny lines (tiny zig-zag) at the end of a line
//  4 tiny piece split off an arc (same circle)
//  5 tiny chord line at the end of an arc
//  6 tiny span at the very start / end of the contour
static double tinyLen(Rng& g, double tol) {
    double u = g.uni();
    if (std::getenv("ABOVETOL")) return tol * (1.0005 + u);
    if (u < 0.08) return logU(g, 1e-12, 1e-8) * tol;   // rounding-ish
    if (u < 0.2) return logU(g, 1e-8, 1e-3) * tol;
    return g.range(0.001, 1.0) * tol;
}
static Contour injectSubTol(const Contour& in, Rng& g, double tol, double prob) {
    Contour o;
    o.closed = in.closed;
    const std::size_t n = in.spans.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Span& sp = in.spans[i];
        const bool lastClosed = (i + 1 == n) && in.closed;
        if (g.uni() >= prob) { o.spans.push_back(sp); continue; }
        const double tl = tinyLen(g, tol);
        int kind = g.irange(0, 6);
        if (const auto* l = std::get_if<LineSpan>(&sp)) {
            Vec2 dir = (l->end - l->start).normalized();
            double L = distance(l->start, l->end);
            if (L <= 8 * tol) { o.spans.push_back(sp); continue; }
            if (kind >= 4 && kind != 6) kind = g.irange(0, 3);
            if (kind == 0) {
                Vec2 jog = Vec2::fromAngle(dir.angle() + g.range(-3.1, 3.1), tl);
                Vec2 mid = l->end - jog;
                o.spans.push_back(LineSpan{l->start, mid});
                o.spans.push_back(LineSpan{mid, l->end});
            } else if (kind == 1 || kind == 6) {
                bool atStart = g.uni() < 0.5;
                Vec2 mid = atStart ? l->start + dir * tl : l->end - dir * tl;
                o.spans.push_back(LineSpan{l->start, mid});
                o.spans.push_back(LineSpan{mid, l->end});
            } else if (kind == 2) {
                double sw = g.range(-3.0, 3.0);
                if (std::fabs(sw) < 1e-3) sw = 1e-3;
                double c = tl * 0.6;
                Vec2 mid = l->end - dir * c;
                // arc from mid to l->end with sweep sw: centre on the perpendicular bisector
                double r = c / (2 * std::sin(std::fabs(sw) / 2));
                Vec2 m = (mid + l->end) * 0.5;
                double h = std::sqrt(std::max(0.0, r * r - c * c / 4));
                Vec2 nrm = dir.perp() * (sw > 0 ? 1.0 : -1.0);
                if (std::fabs(sw) > kPi) h = -h;
                Vec2 ctr = m + nrm * h;
                ArcSpan a{ctr, r, (mid - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW};
                a.endAngle = a.startAngle + sw;
                Vec2 st = a.startPoint();
                Vec2 en = a.endPoint();
                o.spans.push_back(LineSpan{l->start, st});
                o.spans.push_back(a);
                if (!lastClosed && i + 1 < n) {
                    // keep continuity: next span starts at en; if next is a line, move its start
                    Span nx = in.spans[i + 1];
                    if (auto* nl = std::get_if<LineSpan>(&nx)) { nl->start = en; o.spans.push_back(nx); ++i; }
                    else if (distance(en, l->end) > 0) o.spans.push_back(LineSpan{en, l->end});
                } else if (distance(en, l->end) > 0) {
                    o.spans.push_back(LineSpan{en, l->end});
                }
            } else {
                int k = g.irange(2, 6);
                Vec2 p = l->end - dir * (tl * k);
                o.spans.push_back(LineSpan{l->start, p});
                for (int q = 0; q < k; ++q) {
                    Vec2 e = q + 1 == k ? l->end : p + Vec2::fromAngle(dir.angle() + g.range(-3.1, 3.1), tl);
                    o.spans.push_back(LineSpan{p, e});
                    p = e;
                }
            }
        } else if (const auto* a = std::get_if<ArcSpan>(&sp)) {
            double len = length(sp);
            if (len <= 8 * tol) { o.spans.push_back(sp); continue; }
            double sw = a->sweepAngle();
            double dsw = tl / a->radius * (sw > 0 ? 1 : -1);
            if (kind <= 4 || kind == 6) {
                bool atStart = g.uni() < 0.5;
                ArcSpan a1 = *a, a2 = *a;
                double cut = atStart ? a->startAngle + dsw : a->endAngle - dsw;
                a1.endAngle = cut;
                a2.startAngle = cut;
                o.spans.push_back(a1);
                o.spans.push_back(a2);
            } else {
                ArcSpan a1 = *a;
                a1.endAngle = a->endAngle - dsw;
                o.spans.push_back(a1);
                Vec2 e1 = a1.endPoint();
                Vec2 e = a->endPoint();
                if (distance(e1, e) > 0) o.spans.push_back(LineSpan{e1, e});
            }
        } else {
            o.spans.push_back(sp);
        }
    }
    return o;
}

static Contour g_base;
static Case2 makeST(std::uint64_t seed) {
    Rng g(seed ^ 0x9A0D5EEDULL);
    int fam = g.irange(0, 6);
    Case2 cc;
    if (fam <= 1) { cc.c = makeCase(seed); cc.name = "legacy"; }
    else if (fam == 2) cc = makeCase2(seed);
    else if (fam == 3) cc = makeCase3(seed);
    else if (fam == 4) cc = makeCase4(seed);
    else if (fam == 5) cc = makeCase5(seed);
    else { cc.c = makeCase(seed); cc.name = "legacy"; }
    Case& c = cc.c;
    if (c.in.closed == false && g.uni() < 0.2 && c.in.spans.size() >= 3) {
        Vec2 e = c.in.endPoint(), s = c.in.startPoint();
        if (distance(e, s) > 1e-2) { c.in.spans.push_back(LineSpan{e, s}); c.in.closed = true; }
    }
    double prob = g.uni() < 0.5 ? g.range(0.05, 0.3) : g.range(0.3, 0.9);
    g_base = c.in;
    if (!std::getenv("NOINJ")) c.in = injectSubTol(c.in, g, c.tol, prob);
    return cc;
}
static int subTolCount(const Contour& c, double tol) { int k = 0; for (auto& s : c.spans) if (length(s) <= tol) ++k; return k; }
static double minLen(const Contour& c) { double m = 1e300; for (auto& s : c.spans) m = std::min(m, length(s)); return m; }

static void dumpC(const Contour& k) {
    for (auto& s : k.spans) {
        if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s));
        else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s));
    }
}


static int issueClass(const Metrics& m) { return (m.distViol ? 1 : 0) | (m.roundViol ? 2 : 0) | (m.covMiss ? 4 : 0) | (m.selfX ? 8 : 0) | ((m.coderBack + m.overlapIn + m.overlapX) ? 16 : 0); }
static int classOf(const Contour& in, const Case& c) {
    auto out = offsetOpenContour(in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    Metrics m; check(in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
    return issueClass(m);
}
static void dumpCpp(const Contour& k) {
    std::printf("Contour in; in.closed = %s;\n", k.closed ? "true" : "false");
    for (auto& s : k.spans) {
        if (auto* l = std::get_if<LineSpan>(&s)) std::printf("in.spans.emplace_back(LineSpan{{%.17g, %.17g}, {%.17g, %.17g}});\n", l->start.x, l->start.y, l->end.x, l->end.y);
        else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("in.spans.emplace_back(ArcSpan{{%.17g, %.17g}, %.17g, %.17g, %.17g, ArcDirection::%s});\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW");
    }
}
static int shrinkMode2(std::uint64_t seed, int want) {
    Case2 cc = makeST(seed);
    Case c = cc.c;
    int cls = classOf(c.in, c);
    if (want == 0) want = cls;
    std::printf("class %d want %d\n", cls, want);
    Contour cur = c.in;
    bool progress = true;
    while (progress) {
        progress = false;
        // drop a prefix/suffix span, or (open contours) any single span
        for (std::size_t i = 0; i < cur.spans.size() && cur.spans.size() > 1; ++i) {
            Contour t = cur;
            t.spans.erase(t.spans.begin() + static_cast<long>(i));
            if (i != 0 && i + 1 != cur.spans.size()) {
                // keep continuity only if it stays continuous
                if (!(endPoint(t.spans[i - 1]) == startPoint(t.spans[i]))) continue;
            }
            if (t.closed && (i == 0 || i + 1 == cur.spans.size())) t.closed = false;
            if ((classOf(t, c) & want) == want) { cur = t; progress = true; break; }
        }
        if (!progress && cur.closed) { Contour t = cur; t.closed = false; if ((classOf(t, c) & want) == want) { cur = t; progress = true; } }
    }
    std::printf("// seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g, %zu spans\n", (unsigned long long)seed, c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre, cur.spans.size());
    dumpCpp(cur);
    auto out = offsetOpenContour(cur, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    for (std::size_t i = 0; i < out.size(); ++i) { std::printf("chain %zu closed=%d\n", i, out[i].closed); dumpC(out[i]); }
    Metrics m; check(cur, out, c.d, c.side, c.join, c.tol, c.mitre, m);
    std::printf("%s\n", m.firstIssue.c_str());
    return 0;
}

static bool loadCase(const char* path, Case& c) {
    std::FILE* f = std::fopen(path, "r");
    if (!f) return false;
    char line[1024];
    c.in = Contour{};
    c.tol = 0.001;
    while (std::fgets(line, sizeof line, f)) {
        std::string L(line);
        if (L.rfind("// seed", 0) == 0) {
            char join[32] = {0}, side[32] = {0};
            unsigned long long sd; double d, mi;
            if (std::sscanf(line, "// seed 0x%llx d=%lf join=%31s side=%31s mitre=%lf", &sd, &d, join, side, &mi) == 5) {
                std::string j(join), sdn(side);
                if (!j.empty() && j.back() == ' ') j.pop_back();
                c.d = d; c.mitre = mi;
                c.join = j.rfind("Round", 0) == 0 ? JoinType::Round : j.rfind("Sharp", 0) == 0 ? JoinType::Sharp : JoinType::Square;
                c.side = sdn.rfind("Left", 0) == 0 ? OffsetSide::Left : OffsetSide::Right;
            }
        } else if (L.find("in.closed = true") != std::string::npos) c.in.closed = true;
        else if (L.rfind("in.spans.emplace_back(LineSpan", 0) == 0) {
            double a, b, e, g;
            if (std::sscanf(line, "in.spans.emplace_back(LineSpan{{%lf, %lf}, {%lf, %lf}});", &a, &b, &e, &g) == 4) c.in.spans.push_back(LineSpan{{a, b}, {e, g}});
        } else if (L.rfind("in.spans.emplace_back(ArcSpan", 0) == 0) {
            double a, b, r, s0, s1; char dir[8] = {0};
            if (std::sscanf(line, "in.spans.emplace_back(ArcSpan{{%lf, %lf}, %lf, %lf, %lf, ArcDirection::%3s", &a, &b, &r, &s0, &s1, dir) == 6)
                c.in.spans.push_back(ArcSpan{{a, b}, r, s0, s1, dir[1] == 'C' ? ArcDirection::CCW : ArcDirection::CW});
        }
    }
    std::fclose(f);
    return true;
}
static void explainPoint(const Contour& in, Vec2 q, double d) {
    std::printf("  point (%.12g,%.12g):", q.x, q.y);
    for (std::size_t i = 0; i < in.spans.size(); ++i) {
        double dd = distPt(toSeg(in.spans[i]), q);
        if (dd < d + 1e-3) std::printf(" span%zu %.12g (d%+.3g)", i, dd, dd - d);
    }
    std::printf("\n");
}
static int fileMode(const char* path) {
    Case c;
    if (!loadCase(path, c)) { std::printf("cannot read\n"); return 1; }
    std::printf("loaded %zu spans closed=%d d=%.17g %s side=%d mitre=%.17g\n", c.in.spans.size(), c.in.closed, c.d, jn(c.join), (int)c.side, c.mitre);
    auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    for (std::size_t i = 0; i < out.size(); ++i) { std::printf("branch chain %zu closed=%d\n", i, out[i].closed); dumpC(out[i]); }
    Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
    std::printf("branch: %s [dv%d u%.3g rv%d o%.3g cm%d miss %.3g back %d]\n", m.firstIssue.c_str(), m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.covMiss, m.worstMiss, m.coderBack);
    double qx, qy;
    if (std::sscanf(m.firstIssue.c_str(), "coverage miss at (%lf,%lf)", &qx, &qy) == 2) explainPoint(c.in, Vec2{qx, qy}, c.d);
    auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
    for (std::size_t i = 0; i < mo.size(); ++i) { std::printf("main chain %zu closed=%d\n", i, mo[i].closed); dumpC(mo[i]); }
    Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm);
    std::printf("main: %s [dv%d u%.3g rv%d o%.3g cm%d miss %.3g back %d]\n", mm.firstIssue.c_str(), mm.distViol, mm.worstUnder, mm.roundViol, mm.worstOver, mm.covMiss, mm.worstMiss, mm.coderBack);
    return 0;
}

static int dumpMode(std::uint64_t seed, bool base) {
    Case2 cc = makeST(seed);
    Case c = cc.c;
    if (base) c.in = g_base;
    std::printf("// seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g, %zu spans\n", (unsigned long long)seed, c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre, c.in.spans.size());
    dumpCpp(c.in);
    return 0;
}

static int ac3Mode(long N, std::uint64_t seed0, int fam) {
    long n = 0, skipped = 0, same = 0, differ = 0;
    double cpuB = 0, cpuM = 0;
    for (long k = 0; k < N; ++k) {
        std::uint64_t seed = seed0 + static_cast<std::uint64_t>(k);
        Case c;
        if (fam == 0) c = makeCase(seed); else c = makeCaseF(seed, fam).c;
        if (c.in.spans.empty()) continue;
        bool st = false;
        for (auto& s : c.in.spans) if (length(s) <= c.tol) st = true;
        if (st) { skipped++; continue; }
        n++;
        double t0 = cpuNow();
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        cpuB += cpuNow() - t0;
        t0 = cpuNow();
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
        cpuM += cpuNow() - t0;
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        if (mo == out) same++; else { differ++; if (differ <= 20) std::printf("DIFFER seed=0x%llx fam=%d\n", (unsigned long long)seed, fam); }
    }
    std::printf("ac3 fam=%d N=%ld seed0=0x%llx: inputs without sub-tol spans %ld (skipped %ld with): bit-identical to main %ld, differ %ld | cpu branch %.2fs main %.2fs\n", fam, N, (unsigned long long)seed0, n, skipped, same, differ, cpuB, cpuM);
    return 0;
}

static Case vstCase(std::uint64_t seed0, long k) {
    Case c = makeCase(seed0 + static_cast<std::uint64_t>(k));
    if (!subTolIn(c)) {
        Rng g(seed0 * 31 + static_cast<std::uint64_t>(k));
        Contour o; o.closed = c.in.closed;
        for (std::size_t i = 0; i < c.in.spans.size(); ++i) {
            const Span& sp = c.in.spans[i];
            const auto* l = std::get_if<LineSpan>(&sp);
            bool last = i + 1 == c.in.spans.size();
            if (l && g.uni() < 0.3 && !(last && c.in.closed)) {
                Vec2 dir = (l->end - l->start).normalized();
                double L = distance(l->start, l->end);
                double tl = g.range(0.05, 1.0) * c.tol;
                if (L > 3 * tl) {
                    Vec2 jog = Vec2::fromAngle(dir.angle() + g.range(-3.0, 3.0), tl);
                    Vec2 mid = l->end - jog;
                    o.spans.push_back(LineSpan{l->start, mid});
                    o.spans.push_back(LineSpan{mid, l->end});
                    continue;
                }
            }
            o.spans.push_back(sp);
        }
        c.in = o;
    }
    return c;
}
static int vstMode(std::uint64_t seed0, long k) {
    Case c = vstCase(seed0, k);
    std::printf("// seed 0x%llx d=%.17g join=%s side=%s mitre=%.17g, %zu spans\n", (unsigned long long)(seed0 + k), c.d, jn(c.join), c.side == OffsetSide::Left ? "Left" : "Right", c.mitre, c.in.spans.size());
    dumpCpp(c.in);
    return 0;
}
static int vstScan(std::uint64_t seed0, long N) {
    for (long k = 0; k < N; ++k) {
        Case c = vstCase(seed0, k);
        if (!subTolIn(c)) continue;
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm);
        if (mm.distViol || mm.roundViol || mm.covMiss)
            std::printf("k=0x%lx n=%zu %s side=%d d=%.4g main: dv%d u%.3g rv%d o%.3g cm%d miss %.3g\n", k, c.in.spans.size(), jn(c.join), (int)c.side, c.d, mm.distViol, mm.worstUnder, mm.roundViol, mm.worstOver, mm.covMiss, mm.worstMiss);
    }
    return 0;
}

static int stScanMain(std::uint64_t seed0, long N) {
    for (long k = 0; k < N; ++k) {
        Case2 cc = makeST(seed0 + static_cast<std::uint64_t>(k));
        Case& c = cc.c;
        if (c.in.spans.empty() || !subTolCount(c.in, c.tol) || c.join != JoinType::Round) continue;
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, false, true);
        if (mm.roundViol) std::printf("seed=0x%llx %s n=%zu d=%.4g main: rv%d over %.3g\n", (unsigned long long)(seed0 + k), cc.name, c.in.spans.size(), c.d, mm.roundViol, mm.worstOver);
    }
    return 0;
}

static int perfMode() {
    auto run = [](const char* name, const Contour& in, double d) {
        double worst = 0, worstM = 0;
        for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
            for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
                double t0 = cpuNow();
                auto out = offsetOpenContour(in, d, s, j, Tolerance::defaults(), 2.0);
                worst = std::max(worst, cpuNow() - t0);
                t0 = cpuNow();
                auto mo = mainOffset(toP(in), d, s == OffsetSide::Left ? 0 : 1, static_cast<int>(j), 0.001, 2.0);
                worstM = std::max(worstM, cpuNow() - t0);
            }
        std::printf("%-46s spans %6zu d=%g worst CPU %.4f s (main %.4f s)\n", name, in.spans.size(), d, worst, worstM);
    };
    for (int n : {2000, 4000, 8000}) {
        Rng g(0xABCD + n);
        Contour w = walk(g, n, 0.0005, 0.8);
        char nm[96]; std::snprintf(nm, sizeof nm, "walk of sub-tol steps 0.0005 n=%d", n);
        run(nm, w, 1.0);
        run(nm, w, 0.05);
    }
    for (int n : {2000, 4000, 8000}) {
        Contour c;  // smooth dense curve: a sine wave sampled every ~0.5 um
        Vec2 prev{0, 0};
        for (int i = 1; i <= n; ++i) { double x = i * 0.0005; Vec2 q{x, 0.3 * std::sin(x * 7.0)}; c.spans.push_back(LineSpan{prev, q}); prev = q; }
        char nm[96]; std::snprintf(nm, sizeof nm, "dense sine, 0.5 um steps n=%d", n);
        run(nm, c, 1.0);
        Contour k;  // circle of radius 0.5 sampled every ~0.4 um (closed)
        Vec2 p0{0.5, 0};
        Vec2 pv = p0;
        for (int i = 1; i <= n; ++i) { double a = kTwoPi * i / n; Vec2 q = i == n ? p0 : Vec2{0.5 * std::cos(a), 0.5 * std::sin(a)}; k.spans.push_back(LineSpan{pv, q}); pv = q; }
        k.closed = true;
        std::snprintf(nm, sizeof nm, "dense circle r=0.5 closed n=%d", n);
        run(nm, k, 1.0);
    }
    for (int n : {2000, 4000, 8000}) {
        Rng g(0xBEEF + n);
        Contour w = walk(g, n, 0.5, kPi);
        Rng g2(0x5EED + n);
        Contour j = injectSubTol(w, g2, 0.001, 0.3);
        char nm[96]; std::snprintf(nm, sizeof nm, "walk 0.5 with 30%% sub-tol jogs n=%d", n);
        run(nm, j, 2.0);
    }
    return 0;
}

static int perf1(int n, int which) {
    Contour k;
    Vec2 p0{0.5, 0};
    Vec2 pv = p0;
    for (int i = 1; i <= n; ++i) { double a = kTwoPi * i / n; Vec2 q = i == n ? p0 : Vec2{0.5 * std::cos(a), 0.5 * std::sin(a)}; k.spans.push_back(LineSpan{pv, q}); pv = q; }
    k.closed = true;
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
        for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
            if (which >= 0 && which != static_cast<int>(j) * 2 + static_cast<int>(s)) continue;
            double t0 = cpuNow();
            auto out = offsetOpenContour(k, 1.0, s, j, Tolerance::defaults(), 2.0);
            double el = cpuNow() - t0;
            Metrics m; if (std::getenv("PCHECK")) check(k, out, 1.0, s, j, 0.001, 2.0, m);
            std::printf("%s side %d: %.3f s, %zu chains (%zu spans) %s\n", jn(j), (int)s, el, out.size(), out.empty() ? 0 : out[0].spans.size(), m.firstIssue.c_str());
        }
    return 0;
}

static int vstScanBranch(std::uint64_t seed0, long N) {
    for (long k = 0; k < N; ++k) {
        Case c = vstCase(seed0, k);
        if (!subTolIn(c)) continue;
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
        if (m.distViol || m.roundViol || m.covMiss || m.covMissTie)
            std::printf("k=0x%lx n=%zu %s side=%d d=%.4g branch: dv%d rv%d cm%d tie%d (worst tie miss %.3g) %s\n", k, c.in.spans.size(), jn(c.join), (int)c.side, c.d, m.distViol, m.roundViol, m.covMiss, m.covMissTie, m.worstMissTie, m.firstIssue.c_str());
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 3 && std::string(argv[1]) == "vstscanb") { if (std::getenv("COVTIE")) g_covTie = std::atof(std::getenv("COVTIE")); return vstScanBranch(std::strtoull(argv[2], nullptr, 0), std::strtol(argv[3], nullptr, 0)); }
    if (argc > 3 && std::string(argv[1]) == "perf1") return perf1(std::atoi(argv[2]), std::atoi(argv[3]));
    if (argc > 1 && std::string(argv[1]) == "perf") return perfMode();
    if (argc > 3 && std::string(argv[1]) == "stscanmain") return stScanMain(std::strtoull(argv[2], nullptr, 0), std::strtol(argv[3], nullptr, 0));
    if (argc > 3 && std::string(argv[1]) == "vst") return vstMode(std::strtoull(argv[2], nullptr, 0), std::strtol(argv[3], nullptr, 0));
    if (argc > 3 && std::string(argv[1]) == "vstscan") return vstScan(std::strtoull(argv[2], nullptr, 0), std::strtol(argv[3], nullptr, 0));
    if (argc > 4 && std::string(argv[1]) == "ac3") return ac3Mode(std::atol(argv[2]), std::strtoull(argv[3], nullptr, 0), std::atoi(argv[4]));
    if (argc > 2 && std::string(argv[1]) == "dump") return dumpMode(std::strtoull(argv[2], nullptr, 0), argc > 3);
    if (argc > 2 && std::string(argv[1]) == "file") return fileMode(argv[2]);
    if (argc > 2 && std::string(argv[1]) == "shrink") return shrinkMode2(std::strtoull(argv[2], nullptr, 0), argc > 3 ? std::atoi(argv[3]) : 0);
    std::string mode = argc > 1 ? argv[1] : "";
    if (std::getenv("COVTIE")) g_covTie = std::atof(std::getenv("COVTIE"));
    if (mode == "st" || mode == "stm") {
        long N = std::atol(argv[2]);
        std::uint64_t seed0 = std::strtoull(argv[3], nullptr, 0);
        bool cmp = mode == "stm";
        long baseFail = 0, attributable = 0, n = 0, bad = 0, dvC = 0, rvC = 0, cmC = 0, otherC = 0, mBad = 0, worse = 0, printed = 0;
        long tieMissCases = 0; double tieMissWorst = 0;
        long gaps = 0, back = 0, ov = 0, sb = 0, sx = 0, rev = 0, zl = 0, se = 0;
        double wU = 0, wO = 0, wM = 0, cpu = 0;
        long perJoin[3] = {0, 0, 0}, perSide[2] = {0, 0}, closedN = 0;
        for (long k = 0; k < N; ++k) {
            std::uint64_t seed = seed0 + static_cast<std::uint64_t>(k);
            Case2 cc = makeST(seed);
            Case& c = cc.c;
            if (c.in.spans.empty() || (!std::getenv("NOINJ") && !std::getenv("ABOVETOL") && !subTolCount(c.in, c.tol))) continue;
            n++; perJoin[static_cast<int>(c.join)]++; perSide[static_cast<int>(c.side)]++; closedN += c.in.closed;
            double t0 = cpuNow();
            auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
            cpu += cpuNow() - t0;
            Metrics m;
            check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m, true);
            tieMissCases += m.covMissTie > 0; tieMissWorst = std::max(tieMissWorst, m.worstMissTie);
            wU = std::max(wU, m.worstUnder); wO = std::max(wO, m.worstOver); wM = std::max(wM, m.worstMiss);
            int core = m.distViol + m.roundViol + m.covMiss;
            int other = m.lineJunctionNotBitwise + m.arcGapBeyond + m.coderBack + m.overlapIn + m.overlapX + m.shortBack + m.selfX + m.zeroLen + m.subTolAtEnd + m.badSpan;
            gaps += m.lineJunctionNotBitwise + m.arcGapBeyond; back += m.coderBack; ov += m.overlapIn + m.overlapX; sb += m.shortBack; sx += m.selfX; rev += m.reversals; zl += m.zeroLen; se += m.subTolAtEnd;
            dvC += m.distViol > 0; rvC += m.roundViol > 0; cmC += m.covMiss > 0; otherC += other > 0;
            int baseBad = 0;
            if (core || other) {
                auto ob = offsetOpenContour(g_base, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
                Metrics mb; check(g_base, ob, c.d, c.side, c.join, c.tol, c.mitre, mb, true);
                baseBad = mb.distViol + mb.roundViol + mb.covMiss + mb.lineJunctionNotBitwise + mb.arcGapBeyond + mb.coderBack + mb.overlapIn + mb.overlapX + mb.shortBack + mb.selfX + mb.zeroLen + mb.subTolAtEnd + mb.badSpan;
                if (baseBad) baseFail++; else attributable++;
            }
            if (core || other) {
                bad++;
                if (printed < 400) {
                    printed++;
                    std::printf("FAIL%s %s%s seed=0x%llx minlen=%.2g gen=%d n=%zu st=%d closed=%d d=%.6g %s side=%d mitre=%.3g: %s [dv%d u%.3g rv%d o%.3g cm%d gap%d back%d ov%d sb%d sx%d z%d se%d]\n",
                                baseBad ? "(base)" : "", cc.name, cc.noisy ? "(noisy)" : "", (unsigned long long)seed, minLen(c.in), c.gen, c.in.spans.size(), subTolCount(c.in, c.tol), c.in.closed, c.d, jn(c.join), (int)c.side, c.mitre,
                                m.firstIssue.c_str(), m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.covMiss, m.lineJunctionNotBitwise + m.arcGapBeyond, m.coderBack, m.overlapIn + m.overlapX, m.shortBack, m.selfX, m.zeroLen, m.subTolAtEnd);
                    std::fflush(stdout);
                }
            }
            if (cmp) {
                auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
                std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
                Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, true);
                int mcore = mm.distViol + mm.roundViol + mm.covMiss;
                int mother = mm.lineJunctionNotBitwise + mm.arcGapBeyond + mm.coderBack + mm.overlapIn + mm.overlapX + mm.shortBack + mm.selfX + mm.zeroLen + mm.subTolAtEnd + mm.badSpan;
                mBad += (mcore + mother) > 0;
                worse += (core + other) > 0 && (mcore + mother) == 0;
            }
        }
        std::printf("st N=%ld seed0=0x%llx: sub-tol inputs %ld (Round %ld Sharp %ld Square %ld; Left %ld Right %ld; closed %ld): failing %ld | distViol %ld roundViol %ld covMiss %ld other %ld | worst under %.3g over %.3g miss %.3g | gaps %ld back %ld overlaps %ld shortBack %ld selfX %ld reversals %ld zeroLen %ld subTolEnd %ld | cpu %.1fs\n",
                    N, (unsigned long long)seed0, n, perJoin[0], perJoin[1], perJoin[2], perSide[0], perSide[1], closedN, bad, dvC, rvC, cmC, otherC, wU, wO, wM, gaps, back, ov, sb, sx, rev, zl, se, cpu);
        std::printf("  coverage misses next to another input span within d + %.3g (sliver loops/lenses, documented): cases %ld, worst %.3g\n", g_covTie, tieMissCases, tieMissWorst);
        std::printf("  failing with a failing base case %ld; failing with a clean base (attributable to sub-tol spans) %ld\n", baseFail, attributable);
        if (cmp) std::printf("  main failing %ld; branch failing where main clean %ld\n", mBad, worse);
        return 0;
    }
    if (mode == "one") {
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0);
        Case2 cc = makeST(seed);
        Case& c = cc.c;
        std::printf("%s closed=%d d=%.17g %s side=%d mitre=%.17g\n", cc.name, c.in.closed, c.d, jn(c.join), (int)c.side, c.mitre);
        dumpC(c.in);
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        for (std::size_t i = 0; i < out.size(); ++i) { std::printf("branch chain %zu closed=%d\n", i, out[i].closed); dumpC(out[i]); }
        Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
        std::printf("branch: %s [dv%d u%.3g rv%d o%.3g cm%d miss %.3g back %d]\n", m.firstIssue.c_str(), m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.covMiss, m.worstMiss, m.coderBack);
        return 0;
    }
    return vdrv4_main(argc, argv);
}
