// T-0128 verifier AC4: CPU time branch vs origin/main, interleaved, min of reps.
// usage: perf128 <case> <N> <d> <w> <join 0..2> <reps> [side]
//  case zz: zig-zag N spans pitch p=w amplitude 1 (T-0016 card: zz 1000 7 0.02)    zz2: pitch 0.05 amp 0.2
//  comb: N slots each 2d + w wide, 10 deep, teeth 1 wide (inside: Left)      rects: N closed rectangles 3.6 x (2d+w) traced in one open path
//  ring: N concentric half-ring channels (U turns of arcs) 2d + w wide        needles: N slots 2d+w, needle tips meeting a far wall
#include "v9lib.cpp"
static Contour build(const std::string& k, int N, double d, double w) {
    std::vector<Vec2> P;
    if (k == "zz") { for (int i = 0; i <= N; ++i) P.push_back({i * w, (i % 2) * 1.0}); return fromPts(P, false); }
    if (k == "zz2") { for (int i = 0; i <= N; ++i) P.push_back({i * 0.05, (i % 2) * 0.2}); return fromPts(P, false); }
    if (k == "comb") {  // walls: tooth top y=10, slot bottom y=0; slot width 2d+w
        double x = 0, W = 2 * d + w; P.push_back({x, 10});
        for (int i = 0; i < N; ++i) { P.push_back({x + 1, 10}); P.push_back({x + 1, 0}); P.push_back({x + 1 + W, 0}); P.push_back({x + 1 + W, 10}); x += 1 + W; }
        P.push_back({x + 1, 10}); return fromPts(P, false);
    }
    if (k == "hatch") {  // boustrophedon hatch, N lines 10 long at pitch 2d + w (a raster exactly one tool width apart)
        double W = 2 * d + w; for (int i = 0; i < N; ++i) { double y = i * W; if (i % 2 == 0) { P.push_back({0, y}); P.push_back({10, y}); } else { P.push_back({10, y}); P.push_back({0, y}); } }
        return fromPts(P, false);
    }
    if (k == "rects") {
        double x = 0, H = 2 * d + w, L = 3.6;
        P.push_back({x, 0});
        for (int i = 0; i < N; ++i) { P.push_back({x + L, 0}); P.push_back({x + L, H}); P.push_back({x, H}); P.push_back({x, 0}); P.push_back({x, -5}); x += L + 3 * d + 1; P.push_back({x, -5}); P.push_back({x, 0}); }
        return fromPts(P, false);
    }
    if (k == "ring") {  // serpentine of semicircular channels: arcs of radius r and r + 2d + w alternately (a ring channel exactly one tool wide)
        Contour c; double W = 2 * d + w; double cx = 0;
        for (int i = 0; i < N; ++i) {
            double r0 = 1.0, r1 = r0 + W;
            // outer arc CW over the top from left to right, then down, inner arc back, ...
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
    return Contour{};
}
int main(int argc, char** argv) {
    std::string k = argv[1]; int N = atoi(argv[2]); double d = atof(argv[3]), w = atof(argv[4]); int j = atoi(argv[5]), reps = atoi(argv[6]); int side = argc > 7 ? atoi(argv[7]) : 0;
    Contour in = build(k, N, d, w);
    double bb = 1e300, bm = 1e300; size_t nb = 0, nm = 0, sb = 0, sm = 0;
    for (int r = 0; r < reps; ++r) {
        double t0 = cpuNow(); auto ob = offsetOpenContour(in, d, (OffsetSide)side, (JoinType)j, Tolerance{1e-3, 1e-9, 0.01}, 2.0); double t1 = cpuNow();
        auto om = mainOffset(toP(in), d, side, j, 1e-3, 2.0); double t2 = cpuNow();
        bb = std::min(bb, t1 - t0); bm = std::min(bm, t2 - t1); nb = ob.size(); nm = om.size(); sb = 0; sm = 0; for (auto& c : ob) sb += c.spans.size(); for (auto& c : om) sm += c.s.size();
    }
    std::printf("%s N=%d d=%g w=%g %s side=%d spans_in=%zu | branch %.4f s (chains %zu spans %zu) | main %.4f s (chains %zu spans %zu) | ratio %.3f\n", k.c_str(), N, d, w, jn((JoinType)j), side, in.spans.size(), bb, nb, sb, bm, nm, sm, bb / bm);
}
