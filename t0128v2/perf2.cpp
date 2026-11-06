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
    std::printf("%s N=%d d=%g w=%g %s side=%d far=%g spans_in=%zu | branch %.4f s (chains %zu spans %zu) | main %.4f s (chains %zu spans %zu) | ratio %.3f\n", k.c_str(), N, d, w, jn((JoinType)j), side, g_far, in.spans.size(), bb, nb, sb, bm, nm, sm, bb / bm);
}
