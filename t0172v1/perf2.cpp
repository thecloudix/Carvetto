// T-0128 round-2 verifier AC4: CPU time branch vs origin/main, ALTERNATING call order per rep (even reps
// branch first, odd reps main first), min of reps per side.
// usage: perf2 <case> <N> <d> <w> <join 0..2> <reps> [side] [far]
//  cases of perf128 (zz zz2 comb hatch rects ring) plus:
//  chatch: compact hatch (lines 1 mm long, pitch 2d + w) followed by a far U-channel 2d wide at (far, far): grid cells
//          laid out over the strays' extent are huge, so all strays of the hatch share one cell (clustered strays).
//  crects: N closed 3.6 x (2d+w) rectangles in a compact row + far one.   long: one U channel N mm long, 2d + w wide.
//  longz: one channel N mm long whose walls are fine polylines (1 mm segments) with 2d + w width (many joined runs).
#include "v9lib.cpp"
static double g_far = 0;
static Contour build(const std::string& k, int N, double d, double w) {
    std::vector<Vec2> P;
    if (k == "zz") { for (int i = 0; i <= N; ++i) P.push_back({i * w, (i % 2) * 1.0}); return fromPts(P, false); }
    if (k == "zz2") { for (int i = 0; i <= N; ++i) P.push_back({i * 0.05, (i % 2) * 0.2}); return fromPts(P, false); }
    if (k == "comb") {
        double x = 0, W = 2 * d + w; P.push_back({x, 10});
        for (int i = 0; i < N; ++i) { P.push_back({x + 1, 10}); P.push_back({x + 1, 0}); P.push_back({x + 1 + W, 0}); P.push_back({x + 1 + W, 10}); x += 1 + W; }
        P.push_back({x + 1, 10}); return fromPts(P, false);
    }
    if (k == "hatch" || k == "chatch") {
        double W = 2 * d + w, Lh = k == "hatch" ? 10.0 : 1.0;
        for (int i = 0; i < N; ++i) { double y = i * W; if (i % 2 == 0) { P.push_back({0, y}); P.push_back({Lh, y}); } else { P.push_back({Lh, y}); P.push_back({0, y}); } }
        if (k == "chatch") { double F = g_far; P.push_back({F, F}); P.push_back({F + 5, F}); P.push_back({F + 5, F + W}); P.push_back({F, F + W}); }
        return fromPts(P, false);
    }
    if (k == "rects" || k == "crects") {
        double x = 0, H = 2 * d + w, L = 3.6;
        P.push_back({x, 0});
        for (int i = 0; i < N; ++i) { P.push_back({x + L, 0}); P.push_back({x + L, H}); P.push_back({x, H}); P.push_back({x, 0}); P.push_back({x, -5}); x += L + 3 * d + 1; P.push_back({x, -5}); P.push_back({x, 0}); }
        if (k == "crects") { double F = g_far; P.push_back({F, F}); P.push_back({F + L, F}); P.push_back({F + L, F + H}); P.push_back({F, F + H}); P.push_back({F, F}); }
        return fromPts(P, false);
    }
    if (k == "fhatch") {  // compact hatch (N lines 1 mm, pitch 2d + w) + background serpentine of N lines over [10, F] x [0, F]
                          // at pitch F / N (no strays) + far U-channel 2d wide at (F, F): main's input grid stays fine, the
                          // strays' grid has all hatch strays in a few cells.
        double W = 2 * d + w, F = g_far;
        for (int i = 0; i < N; ++i) { double y = i * W; if (i % 2 == 0) { P.push_back({0, y}); P.push_back({1, y}); } else { P.push_back({1, y}); P.push_back({0, y}); } }
        double pitch = F / N;
        for (int i = 0; i < N; ++i) { double y = i * pitch; if (i % 2 == 0) { P.push_back({10, y}); P.push_back({F, y}); } else { P.push_back({F, y}); P.push_back({10, y}); } }
        P.push_back({F + 20, F}); P.push_back({F + 25, F}); P.push_back({F + 25, F + W}); P.push_back({F + 20, F + W});
        return fromPts(P, false);
    }
    if (k == "lhatch") {  // hatch of N lines 10 mm at pitch 2d + w (d small: compact) + far U channel at (far, far)
        double W = 2 * d + w, F = g_far;
        for (int i = 0; i < N; ++i) { double y = i * W; if (i % 2 == 0) { P.push_back({0, y}); P.push_back({10, y}); } else { P.push_back({10, y}); P.push_back({0, y}); } }
        P.push_back({F, F}); P.push_back({F + 5, F}); P.push_back({F + 5, F + W}); P.push_back({F, F + W});
        return fromPts(P, false);
    }
    if (k == "rhatch" || k == "vhatch") {  // T-0172 verifier: hatch of N lines at pitch 2d + w with random x-extents in [0, 10]
        // (seed RH_SEED) + far U channel at (far, far) unless far == 0; vhatch: the same rotated 90 degrees
        double W = 2 * d + w, F = g_far; Rng rg(std::getenv("RH_SEED") ? strtoull(std::getenv("RH_SEED"), 0, 0) : 1);
        for (int i = 0; i < N; ++i) { double y = i * W, a = rg.range(0, 4), b = rg.range(6, 10); if (i % 2 == 0) { P.push_back({a, y}); P.push_back({b, y}); } else { P.push_back({b, y}); P.push_back({a, y}); } }
        if (F != 0) { P.push_back({F, F}); P.push_back({F + 5, F}); P.push_back({F + 5, F + W}); P.push_back({F, F + W}); }
        if (k == "vhatch") for (auto& p : P) p = Vec2{-p.y, p.x};
        return fromPts(P, false);
    }
    if (k == "rrect") {  // T-0172 verifier: N rectangles 2d + w tall sharing one band, random x-extents (RH_SEED), as one open
        // polyline (a_i,0) (b_i,0) (b_i,W) (a_i,W) -> next; + far U at (far, far) unless far == 0. Inside offsets: N overlapping
        // strays on y = d.
        double W = 2 * d + w, F = g_far; Rng rg(std::getenv("RH_SEED") ? strtoull(std::getenv("RH_SEED"), 0, 0) : 1);
        for (int i = 0; i < N; ++i) { double a = rg.range(0, 4), b = rg.range(6, 10); P.push_back({a, 0}); P.push_back({b, 0}); P.push_back({b, W}); P.push_back({a, W}); }
        P.push_back(P[0]);
        if (F != 0) { P.push_back({F, F}); P.push_back({F + 5, F}); P.push_back({F + 5, F + W}); P.push_back({F, F + W}); }
        return fromPts(P, false);
    }
    if (k == "rcomb") {  // T-0172 verifier: comb of N slots 2d + w wide, depths and tooth widths log-uniform over 3 decades
        // (RH_SEED), diagonal tooth tops; + far U at (far, far) unless far == 0
        double W = 2 * d + w, F = g_far, x = 0; Rng rg(std::getenv("RH_SEED") ? strtoull(std::getenv("RH_SEED"), 0, 0) : 1);
        auto lu = [&](double a, double b) { return std::exp(rg.range(std::log(a), std::log(b))); };
        P.push_back({x, 10});
        for (int i = 0; i < N; ++i) { double tw = lu(0.01, 10), dp = lu(0.01, 10); P.push_back({x + tw, 10 + lu(0.001, 1)}); P.push_back({x + tw, 10 - dp}); P.push_back({x + tw + W, 10 - dp}); P.push_back({x + tw + W, 10}); x += tw + W; }
        P.push_back({x + 1, 10});
        if (F != 0) { P.push_back({F, F}); P.push_back({F + 5, F}); P.push_back({F + 5, F + W}); P.push_back({F, F + W}); }
        return fromPts(P, false);
    }
    if (k == "scrib") {  // T-0172 verifier: random walk of N segments, lengths log-uniform 0.01..10, reflected into [0,10]^2,
        // then a U channel exactly 2d wide (a stray) and a far U at (far, far) unless far == 0
        double W = 2 * d + w, F = g_far; Rng rg(std::getenv("RH_SEED") ? strtoull(std::getenv("RH_SEED"), 0, 0) : 1);
        Vec2 p{5, 5}; P.push_back(p);
        for (int i = 0; i < N; ++i) { double L = std::exp(rg.range(std::log(0.01), std::log(10.0))), a = rg.range(0, 2 * kPi);
            Vec2 q = p + Vec2{L * std::cos(a), L * std::sin(a)}; q.x = std::clamp(q.x, 0.0, 10.0); q.y = std::clamp(q.y, 0.0, 10.0); if (distance(p, q) > 1e-3) { P.push_back(q); p = q; } }
        P.push_back({20, 0}); P.push_back({30, 0}); P.push_back({30, W}); P.push_back({20, W});
        if (F != 0) { P.push_back({F, F}); P.push_back({F + 5, F}); P.push_back({F + 5, F + W}); P.push_back({F, F + W}); }
        return fromPts(P, false);
    }
    if (k == "nrect") {  // T-0172 verifier: N rectangles 2d + w tall sharing one band, all centred at x = 5, widths log-uniform
        // [10 tol, 10] (RH_SEED), as one open polyline; inside offsets: collinear strays of nested lengths on y = d
        double W = 2 * d + w, F = g_far; Rng rg(std::getenv("RH_SEED") ? strtoull(std::getenv("RH_SEED"), 0, 0) : 1);
        for (int i = 0; i < N; ++i) { double h = 0.5 * std::exp(rg.range(std::log(0.01), std::log(10.0))); double a = 5 - h, b = 5 + h; P.push_back({a, 0}); P.push_back({b, 0}); P.push_back({b, W}); P.push_back({a, W}); }
        P.push_back(P[0]);
        if (F != 0) { P.push_back({F, F}); P.push_back({F + 5, F}); P.push_back({F + 5, F + W}); P.push_back({F, F + W}); }
        return fromPts(P, false);
    }
    if (k == "spiral") {  // T-0172 verifier: Archimedean spiral of N semicircular arcs (pitch 2d + w + 1 per half turn... radius
        // grows by 0.5 (2d + w + gap) per half turn), then a U channel exactly 2d wide 5 mm beyond (a stray), + far U unless far == 0
        Contour c; double W = 2 * d + w, step = std::getenv("SP_STEP") ? atof(std::getenv("SP_STEP")) : 4 * d; double F = g_far;
        Vec2 ctr{0, 0}; double r = step; double a0 = 0;
        for (int i = 0; i < N; ++i) { ArcSpan arc{ctr, r, a0, a0 + kPi, ArcDirection::CCW}; c.spans.emplace_back(arc);
            Vec2 e = arc.endPoint(); double r2 = r + step / 2; ctr = e - Vec2::fromAngle(a0 + kPi, r2); r = r2; a0 += kPi; }
        Vec2 e = endPoint(c.spans.back()); std::vector<Vec2> Q{e, e + Vec2{0, -5}, e + Vec2{10, -5}, e + Vec2{10, -5 - W}, e + Vec2{0, -5 - W}};
        if (F != 0) { Q.push_back({F, F}); Q.push_back({F + 5, F}); Q.push_back({F + 5, F + W}); Q.push_back({F, F + W}); }
        for (size_t q = 1; q < Q.size(); ++q) c.spans.emplace_back(LineSpan{Q[q - 1], Q[q]});
        c.closed = false; return c;
    }
    if (k == "sslot") {  // T-0172 verifier: a spiral slot exactly 2d + w wide: centre line = N semicircles (radius grows by
        // SP_STEP/2 per half turn, default 4d), input = outer wall out, cap, inner wall back; offset by d: one long arc stray
        Contour c; double W = 2 * d + w, step = std::getenv("SP_STEP") ? atof(std::getenv("SP_STEP")) : 4 * d; double F = g_far;
        std::vector<ArcSpan> mid; Vec2 ctr{0, 0}; double r = step + W; double a0 = 0;
        for (int i = 0; i < N; ++i) { ArcSpan arc{ctr, r, a0, a0 + kPi, ArcDirection::CCW}; mid.push_back(arc);
            Vec2 e = arc.endPoint(); double r2 = r + step / 2; ctr = e - Vec2::fromAngle(a0 + kPi, r2); r = r2; a0 += kPi; }
        for (auto a : mid) { a.radius += W / 2; c.spans.emplace_back(a); }
        ArcSpan lo = mid.back(); lo.radius -= W / 2; ArcSpan last = std::get<ArcSpan>(c.spans.back());
        c.spans.emplace_back(LineSpan{last.endPoint(), lo.endPoint()});
        for (int i = N - 1; i >= 0; --i) { ArcSpan a = mid[i]; a.radius -= W / 2; std::swap(a.startAngle, a.endAngle); a.direction = ArcDirection::CW; c.spans.emplace_back(a); }
        if (F != 0) { Vec2 e = endPoint(c.spans.back()); std::vector<Vec2> Q{e, {F, F}, {F + 5, F}, {F + 5, F + W}, {F, F + W}}; for (size_t q = 1; q < Q.size(); ++q) c.spans.emplace_back(LineSpan{Q[q - 1], Q[q]}); }
        c.closed = false; return c;
    }
    if (k == "long") { double W = 2 * d + w; P = {{0, 0}, {double(N), 0}, {double(N), W}, {0, W}}; return fromPts(P, false); }
    if (k == "longz") { double W = 2 * d + w; for (int i = 0; i <= N; ++i) P.push_back({double(i), 0}); for (int i = N; i >= 0; --i) P.push_back({double(i), W}); return fromPts(P, false); }
    if (k == "ring") {
        Contour c; double W = 2 * d + w; double cx = 0;
        for (int i = 0; i < N; ++i) {
            double r0 = 1.0, r1 = r0 + W;
            c.spans.emplace_back(ArcSpan{{cx, 0}, r1, kPi, 0.0, ArcDirection::CW});
            c.spans.emplace_back(LineSpan{{cx + r1, 0}, {cx + r0, 0}});
            c.spans.emplace_back(ArcSpan{{cx, 0}, r0, 0.0, kPi, ArcDirection::CCW});
            c.spans.emplace_back(LineSpan{{cx - r0, 0}, {cx - r0, -3}});
            c.spans.emplace_back(LineSpan{{cx - r0, -3}, {cx + 2 * r1 + 1 - r1, -3}});
            cx += 2 * r1 + 1;
            c.spans.emplace_back(LineSpan{{cx - r1, -3}, {cx - r1, 0}});
        }
        c.closed = false; return c;
    }
    std::fprintf(stderr, "unknown case\n"); std::exit(2);
}
int main(int argc, char** argv) {
    std::string k = argv[1]; int N = atoi(argv[2]); double d = atof(argv[3]), w = atof(argv[4]); int j = atoi(argv[5]), reps = atoi(argv[6]); int side = argc > 7 ? atoi(argv[7]) : 0;
    g_far = argc > 8 ? atof(argv[8]) : 1e4;
    Contour in = build(k, N, d, w);
    PContour pin = toP(in);
    double bb = 1e300, bm = 1e300; size_t nb = 0, nm = 0, sb = 0, sm = 0;
    Tolerance tol{1e-3, 1e-9, 0.01};
    for (int r = 0; r < reps; ++r) {
        auto runB = [&] { double t0 = cpuNow(); auto ob = offsetOpenContour(in, d, (OffsetSide)side, (JoinType)j, tol, 2.0); double t1 = cpuNow();
                          bb = std::min(bb, t1 - t0); nb = ob.size(); sb = 0; for (auto& c : ob) sb += c.spans.size(); };
        auto runM = [&] { double t0 = cpuNow(); auto om = mainOffset(pin, d, side, j, 1e-3, 2.0); double t1 = cpuNow();
                          bm = std::min(bm, t1 - t0); nm = om.size(); sm = 0; for (auto& c : om) sm += c.s.size(); };
        if (r % 2 == 0) { runB(); runM(); } else { runM(); runB(); }
    }
    if (std::getenv("CMP")) {  // T-0172 verifier: bit identity branch vs reference on this input
        auto ob = offsetOpenContour(in, d, (OffsetSide)side, (JoinType)j, tol, 2.0); auto om = mainOffset(pin, d, side, j, 1e-3, 2.0);
        std::vector<PContour> pb; for (auto& c : ob) pb.push_back(toP(c));
        bool same = pb.size() == om.size(); size_t firstDiff = 0;
        for (size_t i = 0; same && i < pb.size(); ++i) { same = pb[i].closed == om[i].closed && pb[i].s.size() == om[i].s.size();
            for (size_t k = 0; same && k < pb[i].s.size(); ++k) { same = pb[i].s[k].kind == om[i].s[k].kind; for (int q = 0; q < 6 && same; ++q) same = pb[i].s[k].a[q] == om[i].s[k].a[q]; }
            if (!same) firstDiff = i; }
        std::printf("CMP %s: %s (chains %zu / %zu, first differing chain %zu)\n", k.c_str(), same ? "IDENTICAL" : "DIFFER", pb.size(), om.size(), firstDiff);
    }
    std::printf("%s N=%d d=%g w=%g %s side=%d far=%g spans_in=%zu | branch %.4f s (chains %zu spans %zu) | main %.4f s (chains %zu spans %zu) | ratio %.3f\n", k.c_str(), N, d, w, jn((JoinType)j), side, g_far, in.spans.size(), bb, nb, sb, bm, nm, sm, bb / bm);
}
